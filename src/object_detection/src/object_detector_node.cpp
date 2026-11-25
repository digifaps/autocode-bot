/**
 * object_detector: people and objects in the left camera image (YOLOX, TensorRT on the DLA or GPU), with a
 * 3D position from the stereo depth image when it is available.
 *
 * Lifecycle node, so a supervisor can switch it between off / loaded / running:
 *   configure  -> load the TensorRT engine and allocate GPU buffers (a few seconds)
 *   activate   -> run (camera subscriptions are still made only while an output has a subscriber)
 *   deactivate -> stop processing and drop the camera subscriptions; the engine stays loaded
 *   cleanup    -> free the engine and its memory
 * With `autostart` (default) the node configures and activates itself at start.
 *
 * Subscribes (while active and wanted):
 *   camera/left/image_small     sensor_msgs/Image bgr8, 640x360 every frame (detection input; the camera
 *                               scales it in hardware, so this adds no second full-size raw stream)
 *   camera/left/camera_info     raw intrinsics (K, D) to map boxes into the rectified depth image
 *   stereo/depth, stereo/camera_info (only with use_depth): rectified depth in mm and its projection
 * Publishes:
 *   detections                  vision_msgs/Detection2DArray, boxes in full-size left image pixels (1280x720)
 *   detections_3d               vision_msgs/Detection3DArray, objects with depth, in camera_left_optical
 *   detections/annotations      foxglove_msgs/ImageAnnotations, boxes + labels in full-size left image pixels
 *   detections/markers          visualization_msgs/MarkerArray, 3D positions in camera_left_optical
 *   /diagnostics                "Object detection" status once per second
 *
 * Pairing with depth: stereo/depth carries the stamp of the left frame it was computed from. While depth
 * flows, each depth message triggers detection on that same cached left frame, so box and depth are from
 * the same instant. Without depth (use_depth false, or no depth for 1 s) detection runs on the left
 * frames directly, rate-limited, 2D only.
 *
 * Engines are built offline (scripts/fetch_detector_models.sh), never inside the node: a GPU engine takes
 * ~20 minutes to build on the Xavier NX.
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <fstream>
#include <memory>
#include <numeric>
#include <optional>
#include <string>
#include <vector>

#include <cuda_runtime_api.h>
#include <NvInfer.h>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <foxglove_msgs/msg/image_annotations.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/core/cuda_stream_accessor.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudawarping.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;
using foxglove_msgs::msg::ImageAnnotations;
using sensor_msgs::msg::CameraInfo;
using sensor_msgs::msg::Image;
using vision_msgs::msg::Detection2DArray;
using vision_msgs::msg::Detection3DArray;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

namespace
{
const char * const kCoco[80] = {
  "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light",
  "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
  "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
  "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
  "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
  "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
  "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard",
  "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors",
  "teddy bear", "hair drier", "toothbrush"};

double stamp_sec(const builtin_interfaces::msg::Time & t) { return t.sec + t.nanosec * 1e-9; }

KeyValue kv(const std::string & k, const std::string & v)
{
  KeyValue x;
  x.key = k;
  x.value = v;
  return x;
}

std::string expand_home(const std::string & p)
{
  if (!p.empty() && p[0] == '~') {
    const char * h = std::getenv("HOME");
    return std::string(h ? h : "") + p.substr(1);
  }
  return p;
}

struct Det
{
  cv::Rect2f box;  // full-size left image pixels (camera_info resolution)
  float score;
  int cls;
  bool has_3d = false;
  double x = 0, y = 0, z = 0;  // camera_left_optical, m
  double width_m = 0, height_m = 0;
};

float iou(const cv::Rect2f & a, const cv::Rect2f & b)
{
  const float inter = (a & b).area();
  return inter <= 0 ? 0.f : inter / (a.area() + b.area() - inter);
}

class TrtLogger : public nvinfer1::ILogger
{
public:
  explicit TrtLogger(rclcpp::Logger l) : logger_(l) {}
  void log(Severity s, const char * msg) noexcept override
  {
    if (s <= Severity::kERROR) {
      RCLCPP_ERROR(logger_, "TensorRT: %s", msg);
    } else if (s == Severity::kWARNING) {
      RCLCPP_WARN(logger_, "TensorRT: %s", msg);
    }
  }

private:
  rclcpp::Logger logger_;
};
}  // namespace

class ObjectDetector : public rclcpp_lifecycle::LifecycleNode
{
public:
  ObjectDetector() : LifecycleNode("object_detector"), trt_logger_(get_logger())
  {
    image_topic_ = declare_parameter("image_topic", "camera/left/image_small");
    model_ = declare_parameter("model", "yolox_tiny");
    model_dir_ = expand_home(declare_parameter("model_dir", "~/.local/share/autocode-bot/models"));
    dla_core_ = declare_parameter("dla_core", 0);                // -1 = GPU
    use_depth_ = declare_parameter("use_depth", true);
    max_rate_ = declare_parameter("max_rate", 10.0);              // Hz without depth
    score_thr_ = declare_parameter("score_threshold", 0.4);
    nms_thr_ = declare_parameter("nms_threshold", 0.45);
    max_pair_dt_ = declare_parameter("max_pair_dt", 0.06);         // s, depth <-> left frame fallback
    auto classes = declare_parameter("classes", std::vector<std::string>{});  // empty = all COCO classes
    for (int c = 0; c < 80; ++c) {
      class_on_[c] = classes.empty() || std::find(classes.begin(), classes.end(), kCoco[c]) != classes.end();
    }
    declare_parameter("autostart", true);

    // Diagnostics publish in every lifecycle state, so this is a plain publisher
    diag_pub_ = rclcpp::create_publisher<DiagnosticArray>(get_node_topics_interface(), "/diagnostics", 10);
    diag_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { housekeeping(); });
  }

  ~ObjectDetector() override { free_engine(); }

  // ------------------------------------------------------------------ lifecycle
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    if (!load_engine()) {
      free_engine();
      return CallbackReturn::FAILURE;
    }
    det2d_pub_ = create_publisher<Detection2DArray>("detections", 10);
    det3d_pub_ = create_publisher<Detection3DArray>("detections_3d", 10);
    ann_pub_ = create_publisher<ImageAnnotations>("detections/annotations", 10);
    marker_pub_ = create_publisher<MarkerArray>("detections/markers", 10);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    det2d_pub_->on_activate();
    det3d_pub_->on_activate();
    ann_pub_->on_activate();
    marker_pub_->on_activate();
    active_since_ = std::chrono::steady_clock::now();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    unsubscribe();
    det2d_pub_->on_deactivate();
    det3d_pub_->on_deactivate();
    ann_pub_->on_deactivate();
    marker_pub_->on_deactivate();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    unsubscribe();
    det2d_pub_.reset();
    det3d_pub_.reset();
    ann_pub_.reset();
    marker_pub_.reset();
    free_engine();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
  {
    unsubscribe();
    free_engine();
    return CallbackReturn::SUCCESS;
  }

private:
  // ------------------------------------------------------------------ engine
  std::string engine_path() const
  {
    return model_dir_ + "/" + model_ + (dla_core_ >= 0 ? "_dla.engine" : "_gpu.engine");
  }

  bool load_engine()
  {
    error_.clear();
    // Sleep instead of spin while waiting for the DLA/GPU; must precede the CUDA context
    cudaSetDeviceFlags(cudaDeviceScheduleBlockingSync);
    const std::string path = engine_path();
    std::ifstream f(path, std::ios::binary);
    if (!f) {
      error_ = "engine " + path + " not found (run scripts/fetch_detector_models.sh)";
      RCLCPP_ERROR(get_logger(), "%s", error_.c_str());
      return false;
    }
    std::vector<char> blob((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    runtime_.reset(nvinfer1::createInferRuntime(trt_logger_));
    if (!runtime_) {
      error_ = "TensorRT runtime could not be created";
      return false;
    }
    if (dla_core_ >= 0) {
      if (runtime_->getNbDLACores() <= dla_core_) {
        error_ = "DLA core " + std::to_string(dla_core_) + " not available";
        return false;
      }
      runtime_->setDLACore(dla_core_);
    }
    engine_.reset(runtime_->deserializeCudaEngine(blob.data(), blob.size()));
    if (!engine_) {
      error_ = "could not load engine " + path + " (built for another TensorRT version or device?)";
      return false;
    }
    context_.reset(engine_->createExecutionContext());
    if (!context_) {
      error_ = "could not create TensorRT execution context";
      return false;
    }
    for (int i = 0; i < engine_->getNbIOTensors(); ++i) {
      const char * name = engine_->getIOTensorName(i);
      const auto dims = engine_->getTensorShape(name);
      if (engine_->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT) {
        in_name_ = name;
        in_h_ = dims.d[2];
        in_w_ = dims.d[3];
      } else {
        out_name_ = name;
        out_rows_ = dims.d[1];
        out_cols_ = dims.d[2];
      }
    }
    if (in_h_ <= 0 || out_cols_ != 85) {
      error_ = "unexpected model shape (want 1x3xHxW in, 1xNx85 out: YOLOX COCO)";
      return false;
    }
    // Expected outputs: one row per anchor point over the stride 8/16/32 grids
    int anchors = 0;
    for (int s : {8, 16, 32}) {
      anchors += (in_h_ / s) * (in_w_ / s);
    }
    if (anchors != out_rows_) {
      error_ = "output rows " + std::to_string(out_rows_) + " do not match YOLOX grids (" +
               std::to_string(anchors) + ")";
      return false;
    }
    if (cudaMalloc(&in_dev_, sizeof(float) * 3 * in_h_ * in_w_) != cudaSuccess ||
        cudaMalloc(&out_dev_, sizeof(float) * out_rows_ * out_cols_) != cudaSuccess ||
        cudaMallocHost(&out_host_, sizeof(float) * out_rows_ * out_cols_) != cudaSuccess) {
      error_ = "GPU buffer allocation failed";
      return false;
    }
    if (!cvs_) {
      cvs_.emplace();
    }
    context_->setTensorAddress(in_name_.c_str(), in_dev_);
    context_->setTensorAddress(out_name_.c_str(), out_dev_);
    RCLCPP_INFO(get_logger(), "loaded %s: input %dx%d on %s", path.c_str(), in_w_, in_h_,
                dla_core_ >= 0 ? ("DLA " + std::to_string(dla_core_)).c_str() : "GPU");
    return true;
  }

  void free_engine()
  {
    context_.reset();
    engine_.reset();
    runtime_.reset();
    if (in_dev_) cudaFree(in_dev_);
    if (out_dev_) cudaFree(out_dev_);
    if (out_host_) cudaFreeHost(out_host_);
    in_dev_ = out_dev_ = nullptr;
    out_host_ = nullptr;
  }

  // ------------------------------------------------------------------ subscriptions on demand
  bool is_active()
  {
    return get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
  }

  bool wanted()
  {
    return is_active() && ann_pub_ &&
           (det2d_pub_->get_subscription_count() + det3d_pub_->get_subscription_count() +
            ann_pub_->get_subscription_count() + marker_pub_->get_subscription_count() > 0);
  }

  void housekeeping()
  {
    const bool want = wanted();
    if (want && !left_sub_) {
      auto qos = rclcpp::SensorDataQoS().keep_last(2);
      left_sub_ = create_subscription<Image>(image_topic_, qos,
        [this](Image::ConstSharedPtr m) { on_left(m); });
      left_info_sub_ = create_subscription<CameraInfo>("camera/left/camera_info", 10,
        [this](CameraInfo::ConstSharedPtr m) { left_info_ = m; });
      if (use_depth_) {
        depth_sub_ = create_subscription<Image>("stereo/depth", rclcpp::SensorDataQoS().keep_last(2),
          [this](Image::ConstSharedPtr m) { on_depth(m); });
        depth_info_sub_ = create_subscription<CameraInfo>("stereo/camera_info", rclcpp::SensorDataQoS(),
          [this](CameraInfo::ConstSharedPtr m) { depth_info_ = m; });
      }
      RCLCPP_INFO(get_logger(), "detection on (subscriber present)");
    } else if (!want && left_sub_) {
      unsubscribe();
      RCLCPP_INFO(get_logger(), "detection off (no subscribers)");
    }
    publish_diagnostics();
  }

  void unsubscribe()
  {
    left_sub_.reset();
    left_info_sub_.reset();
    depth_sub_.reset();
    depth_info_sub_.reset();
    cache_.clear();
  }

  // ------------------------------------------------------------------ frame selection
  bool depth_flowing() const
  {
    return use_depth_ && last_depth_rx_.time_since_epoch().count() > 0 &&
           std::chrono::steady_clock::now() - last_depth_rx_ < std::chrono::seconds(1);
  }

  void on_left(const Image::ConstSharedPtr & m)
  {
    cache_.push_back(m);
    while (cache_.size() > 8) {  // 400 ms at 20 Hz: covers the depth node's latency
      cache_.pop_front();
    }
    if (depth_flowing()) {
      return;  // on_depth drives processing
    }
    // Rate cap with 20 % slack (frames come on a 50 ms grid with jitter)
    const auto t = std::chrono::steady_clock::now();
    if (processed_once_ && std::chrono::duration<double>(t - last_processed_).count() < 0.8 / max_rate_) {
      return;
    }
    last_processed_ = t;
    processed_once_ = true;
    run(m, nullptr);
  }

  void on_depth(const Image::ConstSharedPtr & d)
  {
    last_depth_rx_ = std::chrono::steady_clock::now();
    const double td = stamp_sec(d->header.stamp);
    Image::ConstSharedPtr best;
    double best_dt = max_pair_dt_;
    for (const auto & l : cache_) {
      const double dt = std::fabs(stamp_sec(l->header.stamp) - td);
      if (dt <= best_dt) {
        best_dt = dt;
        best = l;
      }
    }
    if (!best) {
      ++unpaired_;
      return;
    }
    run(best, d);
  }

  void run(const Image::ConstSharedPtr & img, const Image::ConstSharedPtr & depth)
  {
    if (img->encoding != "bgr8") {
      error_ = "unexpected image encoding " + img->encoding;
      return;
    }
    try {
      const auto t0 = std::chrono::steady_clock::now();
      std::vector<Det> dets = infer(img);
      // Boxes to full-size left pixels (the resolution of camera_info and of the image Foxglove shows)
      const double full_w = left_info_ ? left_info_->width : 1280.0, full_h = left_info_ ? left_info_->height : 720.0;
      const float sx = static_cast<float>(full_w / img->width), sy = static_cast<float>(full_h / img->height);
      for (auto & d : dets) {
        d.box = cv::Rect2f(d.box.x * sx, d.box.y * sy, d.box.width * sx, d.box.height * sy);
      }
      if (depth && depth->encoding == "16UC1" && left_info_ && depth_info_) {
        locate(dets, *depth);
      }
      publish(img->header, dets);
      const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
      proc_ms_ = proc_ms_ <= 0 ? ms : 0.9 * proc_ms_ + 0.1 * ms;
      ++frames_;
      last_count_ = dets.size();
      with_depth_ = depth != nullptr;
      if (error_.rfind("unexpected image", 0) == 0) {
        error_.clear();
      }
    } catch (const cv::Exception & e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "detection failed: %s", e.what());
    }
  }

  // ------------------------------------------------------------------ inference
  std::vector<Det> infer(const Image::ConstSharedPtr & img)
  {
    // YOLOX preprocessing: resize keeping the aspect ratio, pad bottom/right with 114, BGR, 0..255 float, CHW
    cv::cuda::Stream & stream_ = *cvs_;
    const float r = std::min(static_cast<float>(in_w_) / img->width, static_cast<float>(in_h_) / img->height);
    const int rw = static_cast<int>(img->width * r), rh = static_cast<int>(img->height * r);
    cv::Mat frame(img->height, img->width, CV_8UC3, const_cast<uint8_t *>(img->data.data()), img->step);
    g_bgr_.upload(frame, stream_);
    cv::cuda::resize(g_bgr_, g_small_, cv::Size(rw, rh), 0, 0, cv::INTER_LINEAR, stream_);
    g_canvas_.create(in_h_, in_w_, CV_8UC3);
    g_canvas_.setTo(cv::Scalar(114, 114, 114), stream_);
    g_small_.copyTo(g_canvas_(cv::Rect(0, 0, rw, rh)), stream_);
    g_canvas_.convertTo(g_f32_, CV_32FC3, stream_);
    float * in = static_cast<float *>(in_dev_);
    std::vector<cv::cuda::GpuMat> planes = {
      cv::cuda::GpuMat(in_h_, in_w_, CV_32FC1, in),
      cv::cuda::GpuMat(in_h_, in_w_, CV_32FC1, in + in_h_ * in_w_),
      cv::cuda::GpuMat(in_h_, in_w_, CV_32FC1, in + 2 * in_h_ * in_w_)};
    cv::cuda::split(g_f32_, planes, stream_);
    cudaStream_t cs = cv::cuda::StreamAccessor::getStream(stream_);
    if (!context_->enqueueV3(cs)) {
      throw cv::Exception(0, "TensorRT enqueue failed", __func__, __FILE__, __LINE__);
    }
    cudaMemcpyAsync(out_host_, out_dev_, sizeof(float) * out_rows_ * out_cols_, cudaMemcpyDeviceToHost, cs);
    stream_.waitForCompletion();

    // Decode: per stride grid, row = [dx, dy, log w, log h, objectness, 80 class scores] (sigmoids applied)
    std::vector<Det> cand;
    int row = 0;
    for (int s : {8, 16, 32}) {
      const int gw = in_w_ / s, gh = in_h_ / s;
      for (int gy = 0; gy < gh; ++gy) {
        for (int gx = 0; gx < gw; ++gx, ++row) {
          const float * p = out_host_ + row * out_cols_;
          const float obj = p[4];
          if (obj < score_thr_) {
            continue;
          }
          const float * cls = p + 5;
          const int c = static_cast<int>(std::max_element(cls, cls + 80) - cls);
          const float score = obj * cls[c];
          if (score < score_thr_ || !class_on_[c]) {
            continue;
          }
          const float cx = (p[0] + gx) * s, cy = (p[1] + gy) * s;
          const float w = std::exp(p[2]) * s, h = std::exp(p[3]) * s;
          Det d;
          d.box = cv::Rect2f((cx - w / 2) / r, (cy - h / 2) / r, w / r, h / r) &
                  cv::Rect2f(0, 0, img->width, img->height);
          d.score = score;
          d.cls = c;
          cand.push_back(d);
        }
      }
    }
    // Class-aware greedy NMS
    std::sort(cand.begin(), cand.end(), [](const Det & a, const Det & b) { return a.score > b.score; });
    std::vector<Det> out;
    for (const auto & d : cand) {
      bool keep = true;
      for (const auto & k : out) {
        if (k.cls == d.cls && iou(k.box, d.box) > nms_thr_) {
          keep = false;
          break;
        }
      }
      if (keep) {
        out.push_back(d);
      }
    }
    return out;
  }

  // ------------------------------------------------------------------ 3D from depth
  void locate(std::vector<Det> & dets, const Image & depth)
  {
    if (dets.empty()) {
      return;
    }
    const auto & L = *left_info_;
    const auto & S = *depth_info_;
    cv::Mat K(3, 3, CV_64F, const_cast<double *>(L.k.data()));
    cv::Mat D(1, static_cast<int>(L.d.size()), CV_64F, const_cast<double *>(L.d.data()));
    cv::Mat R(3, 3, CV_64F, const_cast<double *>(S.r.data()));
    cv::Mat P = cv::Mat(3, 4, CV_64F, const_cast<double *>(S.p.data()))(cv::Rect(0, 0, 3, 3));
    const double fx = S.p[0], fy = S.p[5], cx = S.p[2], cy = S.p[6];

    // Per detection: its centre plus a 7x7 grid over the central half of the box, mapped from raw left
    // pixels into the rectified depth image (not a plain scale: different distortion and rotation)
    constexpr int G = 7;
    std::vector<cv::Point2f> raw;
    raw.reserve(dets.size() * (G * G + 1));
    for (const auto & d : dets) {
      raw.emplace_back(d.box.x + d.box.width / 2, d.box.y + d.box.height / 2);
      for (int j = 0; j < G; ++j) {
        for (int i = 0; i < G; ++i) {
          raw.emplace_back(d.box.x + d.box.width * (0.25f + 0.5f * i / (G - 1)),
                           d.box.y + d.box.height * (0.25f + 0.5f * j / (G - 1)));
        }
      }
    }
    std::vector<cv::Point2f> rect;
    cv::undistortPoints(raw, rect, K, D, R, P);

    cv::Mat dm(depth.height, depth.width, CV_16UC1, const_cast<uint8_t *>(depth.data.data()), depth.step);
    for (size_t n = 0; n < dets.size(); ++n) {
      const cv::Point2f * pts = &rect[n * (G * G + 1)];
      std::vector<uint16_t> zs;
      for (int k = 1; k <= G * G; ++k) {
        const int u = static_cast<int>(std::lround(pts[k].x)), v = static_cast<int>(std::lround(pts[k].y));
        if (u >= 0 && v >= 0 && u < dm.cols && v < dm.rows) {
          const uint16_t z = dm.at<uint16_t>(v, u);
          if (z > 0) {
            zs.push_back(z);
          }
        }
      }
      if (zs.size() < 5) {
        continue;  // too little depth on the object (textureless, too close, or in the left edge strip)
      }
      std::nth_element(zs.begin(), zs.begin() + zs.size() / 2, zs.end());
      const double Z = zs[zs.size() / 2] / 1000.0;
      const double X = (pts[0].x - cx) * Z / fx, Y = (pts[0].y - cy) * Z / fy;
      // Rectified -> left optical frame: p = R1^T * p_rect (as stereo/points)
      const auto & r1 = S.r;
      auto & d = dets[n];
      d.x = r1[0] * X + r1[3] * Y + r1[6] * Z;
      d.y = r1[1] * X + r1[4] * Y + r1[7] * Z;
      d.z = r1[2] * X + r1[5] * Y + r1[8] * Z;
      d.width_m = d.box.width * Z / L.k[0];
      d.height_m = d.box.height * Z / L.k[4];
      d.has_3d = true;
    }
  }

  // ------------------------------------------------------------------ output
  static foxglove_msgs::msg::Color colour_for(int cls, double a = 1.0)
  {
    foxglove_msgs::msg::Color c;
    if (cls == 0) {  // people stand out
      c.r = 1.0; c.g = 0.2; c.b = 0.2;
    } else {
      const double h = std::fmod(cls * 0.618034, 1.0) * 6.0;  // spread hues
      const double x = 1.0 - std::fabs(std::fmod(h, 2.0) - 1.0);
      const int i = static_cast<int>(h);
      const double rgb[6][3] = {{1, x, 0}, {x, 1, 0}, {0, 1, x}, {0, x, 1}, {x, 0, 1}, {1, 0, x}};
      c.r = rgb[i][0]; c.g = rgb[i][1]; c.b = rgb[i][2];
    }
    c.a = a;
    return c;
  }

  void publish(const std_msgs::msg::Header & hdr, const std::vector<Det> & dets)
  {
    if (det2d_pub_->get_subscription_count() > 0) {
      auto arr = std::make_unique<Detection2DArray>();
      arr->header = hdr;
      for (const auto & d : dets) {
        vision_msgs::msg::Detection2D v;
        v.header = hdr;
        vision_msgs::msg::ObjectHypothesisWithPose h;
        h.hypothesis.class_id = kCoco[d.cls];
        h.hypothesis.score = d.score;
        v.results.push_back(h);
        v.bbox.center.position.x = d.box.x + d.box.width / 2;
        v.bbox.center.position.y = d.box.y + d.box.height / 2;
        v.bbox.size_x = d.box.width;
        v.bbox.size_y = d.box.height;
        arr->detections.push_back(v);
      }
      det2d_pub_->publish(std::move(arr));
    }
    if (det3d_pub_->get_subscription_count() > 0) {
      auto arr = std::make_unique<Detection3DArray>();
      arr->header = hdr;
      for (const auto & d : dets) {
        if (!d.has_3d) {
          continue;
        }
        vision_msgs::msg::Detection3D v;
        v.header = hdr;
        vision_msgs::msg::ObjectHypothesisWithPose h;
        h.hypothesis.class_id = kCoco[d.cls];
        h.hypothesis.score = d.score;
        h.pose.pose.position.x = d.x;
        h.pose.pose.position.y = d.y;
        h.pose.pose.position.z = d.z;
        h.pose.pose.orientation.w = 1.0;
        v.results.push_back(h);
        v.bbox.center = h.pose.pose;
        v.bbox.size.x = d.width_m;   // camera_left_optical: x right, y down, z forward
        v.bbox.size.y = d.height_m;
        v.bbox.size.z = 0.0;         // unknown from one view
        arr->detections.push_back(v);
      }
      det3d_pub_->publish(std::move(arr));
    }
    if (ann_pub_->get_subscription_count() > 0) {
      auto ann = std::make_unique<ImageAnnotations>();
      for (const auto & d : dets) {
        foxglove_msgs::msg::PointsAnnotation box;
        box.timestamp = hdr.stamp;
        box.type = foxglove_msgs::msg::PointsAnnotation::LINE_LOOP;
        const cv::Point2f c[4] = {d.box.tl(), {d.box.x + d.box.width, d.box.y}, d.box.br(),
                                  {d.box.x, d.box.y + d.box.height}};
        for (const auto & p : c) {
          foxglove_msgs::msg::Point2 q;
          q.x = p.x;
          q.y = p.y;
          box.points.push_back(q);
        }
        box.outline_color = colour_for(d.cls);
        box.thickness = 3.0;
        ann->points.push_back(box);

        foxglove_msgs::msg::TextAnnotation t;
        t.timestamp = hdr.stamp;
        t.position.x = d.box.x + 2;
        t.position.y = std::max(0.f, d.box.y - 4);
        char buf[96];
        if (d.has_3d) {
          std::snprintf(buf, sizeof(buf), "%s %.0f%% %.1f m", kCoco[d.cls], d.score * 100.0,
                        std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z));
        } else {
          std::snprintf(buf, sizeof(buf), "%s %.0f%%", kCoco[d.cls], d.score * 100.0);
        }
        t.text = buf;
        t.font_size = 22.0;
        t.text_color.r = t.text_color.g = t.text_color.b = 1.0;
        t.text_color.a = 1.0;
        t.background_color = colour_for(d.cls, 0.7);
        ann->texts.push_back(t);
      }
      ann_pub_->publish(std::move(ann));
    }

    if (marker_pub_->get_subscription_count() > 0) {
      auto ma = std::make_unique<MarkerArray>();
      Marker clear;
      clear.header = hdr;
      clear.action = Marker::DELETEALL;
      ma->markers.push_back(clear);
      int id = 0;
      for (const auto & d : dets) {
        if (!d.has_3d) {
          continue;
        }
        const auto col = colour_for(d.cls, 0.5);
        Marker m;
        m.header = hdr;  // camera_left_optical: x right, y down, z forward
        m.ns = "objects";
        m.id = id++;
        m.type = Marker::CUBE;
        m.action = Marker::ADD;
        m.pose.position.x = d.x;
        m.pose.position.y = d.y;
        m.pose.position.z = d.z;
        m.pose.orientation.w = 1.0;
        m.scale.x = std::max(0.05, d.width_m);
        m.scale.y = std::max(0.05, d.height_m);
        m.scale.z = std::max(0.05, std::min(d.width_m, 0.5));  // depth of the object is unknown
        m.color.r = col.r; m.color.g = col.g; m.color.b = col.b; m.color.a = col.a;
        m.lifetime = rclcpp::Duration::from_seconds(0.5);
        ma->markers.push_back(m);

        Marker t = m;
        t.ns = "labels";
        t.type = Marker::TEXT_VIEW_FACING;
        t.pose.position.y -= d.height_m / 2 + 0.1;
        t.scale.x = t.scale.y = 0.0;
        t.scale.z = 0.15;
        t.color.r = t.color.g = t.color.b = t.color.a = 1.0;
        t.text = kCoco[d.cls];
        ma->markers.push_back(t);
      }
      marker_pub_->publish(std::move(ma));
    }
  }

  // ------------------------------------------------------------------ diagnostics
  void publish_diagnostics()
  {
    DiagnosticStatus st;
    st.name = "Object detection";
    st.hardware_id = "object_detector";
    const uint64_t hz = frames_ - frames_reported_;
    frames_reported_ = frames_;
    const auto state = get_current_state().id();
    if (!error_.empty()) {
      st.level = DiagnosticStatus::ERROR;
      st.message = error_;
    } else if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
      st.level = DiagnosticStatus::OK;
      st.message = "off (model not loaded)";
    } else if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
      st.level = DiagnosticStatus::OK;
      st.message = "paused (model loaded)";
    } else if (!left_sub_) {
      st.level = DiagnosticStatus::OK;
      st.message = "idle (no subscribers)";
    } else if (hz == 0) {
      const bool settling = std::chrono::steady_clock::now() - active_since_ < std::chrono::seconds(5);
      st.level = settling ? DiagnosticStatus::OK : DiagnosticStatus::WARN;
      st.message = "no camera frames";
    } else {
      st.level = DiagnosticStatus::OK;
      char buf[160];
      std::snprintf(buf, sizeof(buf), "%lu Hz, %.0f ms per frame, %zu objects%s", hz, proc_ms_, last_count_,
                    with_depth_ ? " (with depth)" : " (2D only, no depth)");
      st.message = buf;
    }
    st.values = {kv("model", model_), kv("device", dla_core_ >= 0 ? "DLA " + std::to_string(dla_core_) : "GPU"),
                 kv("input", std::to_string(in_w_) + "x" + std::to_string(in_h_)),
                 kv("use_depth", use_depth_ ? "true" : "false"),
                 kv("unpaired_depth_frames", std::to_string(unpaired_))};
    DiagnosticArray arr;
    arr.header.stamp = now();
    arr.status.push_back(st);
    diag_pub_->publish(arr);
  }

  // parameters
  std::string image_topic_, model_, model_dir_;
  int dla_core_;
  bool use_depth_;
  double max_rate_, score_thr_, nms_thr_, max_pair_dt_;
  bool class_on_[80];

  // ROS
  rclcpp_lifecycle::LifecyclePublisher<Detection2DArray>::SharedPtr det2d_pub_;
  rclcpp_lifecycle::LifecyclePublisher<Detection3DArray>::SharedPtr det3d_pub_;
  rclcpp_lifecycle::LifecyclePublisher<ImageAnnotations>::SharedPtr ann_pub_;
  rclcpp_lifecycle::LifecyclePublisher<MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::Subscription<Image>::SharedPtr left_sub_, depth_sub_;
  rclcpp::Subscription<CameraInfo>::SharedPtr left_info_sub_, depth_info_sub_;
  rclcpp::TimerBase::SharedPtr diag_timer_;

  // state
  std::deque<Image::ConstSharedPtr> cache_;
  CameraInfo::ConstSharedPtr left_info_, depth_info_;
  std::chrono::steady_clock::time_point last_processed_, last_depth_rx_, active_since_;
  bool processed_once_ = false, with_depth_ = false;
  std::string error_;
  double proc_ms_ = 0;
  uint64_t frames_ = 0, frames_reported_ = 0, unpaired_ = 0;
  size_t last_count_ = 0;

  // TensorRT + GPU
  TrtLogger trt_logger_;
  std::unique_ptr<nvinfer1::IRuntime> runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine> engine_;
  std::unique_ptr<nvinfer1::IExecutionContext> context_;
  std::string in_name_, out_name_;
  int in_w_ = 0, in_h_ = 0, out_rows_ = 0, out_cols_ = 0;
  void * in_dev_ = nullptr;
  void * out_dev_ = nullptr;
  float * out_host_ = nullptr;
  std::optional<cv::cuda::Stream> cvs_;  // created after cudaSetDeviceFlags (creating it makes the context)
  cv::cuda::GpuMat g_bgr_, g_small_, g_canvas_, g_f32_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObjectDetector>();
  if (node->get_parameter("autostart").as_bool()) {
    if (node->configure().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
      node->activate();
    }
  }
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}

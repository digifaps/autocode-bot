/**
 * stereo_depth_node: depth from the calibrated stereo cameras, on the GPU (OpenCV CUDA).
 *
 * Subscribes (only while one of its outputs has a subscriber):
 *   camera/{left,right}/image_raw    sensor_msgs/Image (bgr8, 1280x720)
 *   camera/{left,right}/camera_info  sensor_msgs/CameraInfo (calibration: K, D, R, P)
 * Publishes:
 *   stereo/depth                     sensor_msgs/Image 16UC1, depth in mm (0 = unknown), rectified left view
 *   stereo/camera_info               sensor_msgs/CameraInfo for stereo/depth
 *   stereo/depth_preview/compressed  sensor_msgs/CompressedImage, colour-coded depth (JPEG, for Foxglove)
 *   stereo/points                    sensor_msgs/PointCloud2 xyz + rgb in the left camera's optical frame
 *   /diagnostics                     "Stereo depth" status once per second
 *
 * Pipeline per pair: upload -> grey -> rectify + downscale in one remap -> StereoSGM (GPU, one CUDA stream,
 * results downloaded into pinned memory) -> speckle filter (CPU) -> depth and preview colours through lookup
 * tables indexed by the fixed-point disparity. The CUDA context uses blocking sync, so the thread sleeps
 * instead of spinning while it waits for the GPU. Left and right sensors are not synchronized; pairs whose
 * stamps differ by more than `max_pair_dt` are skipped.
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <cuda_runtime_api.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudastereo.hpp>
#include <opencv2/cudawarping.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

using sensor_msgs::msg::CameraInfo;
using sensor_msgs::msg::CompressedImage;
using sensor_msgs::msg::Image;
using sensor_msgs::msg::PointCloud2;
using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;

namespace
{
double stamp_sec(const builtin_interfaces::msg::Time & t) { return t.sec + t.nanosec * 1e-9; }

KeyValue kv(const std::string & k, const std::string & v)
{
  KeyValue x;
  x.key = k;
  x.value = v;
  return x;
}
}  // namespace

class StereoDepthNode : public rclcpp::Node
{
public:
  StereoDepthNode() : Node("stereo_depth")
  {
    out_width_ = declare_parameter("width", 640);              // output width (height keeps the aspect)
    num_disp_ = declare_parameter("num_disparities", 128);     // 64, 128 or 256 (CUDA SGM)
    p1_ = declare_parameter("p1", 10);                         // SGM smoothness penalties
    p2_ = declare_parameter("p2", 120);
    uniqueness_ = declare_parameter("uniqueness_ratio", 5);
    max_rate_ = declare_parameter("max_rate", 10.0);           // Hz; the cameras give 20
    min_depth_ = declare_parameter("min_depth", 0.25);         // m
    max_depth_ = declare_parameter("max_depth", 6.0);          // m
    preview_max_ = declare_parameter("preview_max_depth", 5.0);  // m, colour scale of the preview
    point_step_ = declare_parameter("point_step", 4);          // every n-th pixel in the point cloud
    speckle_size_ = declare_parameter("speckle_size", 100);    // px; 0 disables the speckle filter
    speckle_range_ = declare_parameter("speckle_range", 2);    // disparity px
    max_pair_dt_ = declare_parameter("max_pair_dt", 0.03);     // s between left and right stamps

    if (num_disp_ != 64 && num_disp_ != 128 && num_disp_ != 256) {
      RCLCPP_WARN(get_logger(), "num_disparities must be 64, 128 or 256 for CUDA SGM; using 128");
      num_disp_ = 128;
    }

    depth_pub_ = create_publisher<Image>("stereo/depth", rclcpp::SensorDataQoS());
    info_pub_ = create_publisher<CameraInfo>("stereo/camera_info", rclcpp::SensorDataQoS());
    preview_pub_ = create_publisher<CompressedImage>("stereo/depth_preview/compressed", rclcpp::SensorDataQoS());
    points_pub_ = create_publisher<PointCloud2>("stereo/points", rclcpp::SensorDataQoS());
    diag_pub_ = create_publisher<DiagnosticArray>("/diagnostics", 10);

    timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { housekeeping(); });
    RCLCPP_INFO(get_logger(), "stereo_depth ready: %d disparities, output width %d, max %.0f Hz "
                "(idle until stereo/depth, stereo/points or the preview has a subscriber)",
                num_disp_, out_width_, max_rate_);
  }

private:
  // ------------------------------------------------------------------ subscriptions on demand
  bool wanted() const
  {
    return depth_pub_->get_subscription_count() + points_pub_->get_subscription_count() +
           preview_pub_->get_subscription_count() + info_pub_->get_subscription_count() > 0;
  }

  void housekeeping()
  {
    const bool want = wanted();
    if (want && !left_sub_) {
      if (!gpu_ok_ && !init_gpu()) {
        publish_diagnostics();
        return;
      }
      auto qos = rclcpp::SensorDataQoS().keep_last(2);
      left_sub_ = create_subscription<Image>("camera/left/image_raw", qos,
        [this](Image::ConstSharedPtr m) { left_ = m; try_process(); });
      right_sub_ = create_subscription<Image>("camera/right/image_raw", qos,
        [this](Image::ConstSharedPtr m) { right_ = m; try_process(); });
      left_info_sub_ = create_subscription<CameraInfo>("camera/left/camera_info", 10,
        [this](CameraInfo::ConstSharedPtr m) { on_info(m, true); });
      right_info_sub_ = create_subscription<CameraInfo>("camera/right/camera_info", 10,
        [this](CameraInfo::ConstSharedPtr m) { on_info(m, false); });
      active_since_ = now();
      RCLCPP_INFO(get_logger(), "depth on (subscriber present)");
    } else if (!want && left_sub_) {
      left_sub_.reset();
      right_sub_.reset();
      left_info_sub_.reset();
      right_info_sub_.reset();
      left_.reset();
      right_.reset();
      RCLCPP_INFO(get_logger(), "depth off (no subscribers)");
    }
    publish_diagnostics();
  }

  bool init_gpu()
  {
    try {
      // Sleep in cudaStreamSynchronize instead of spinning a CPU core while StereoSGM runs.
      // Must happen before anything creates the CUDA context.
      const cudaError_t flag_err = cudaSetDeviceFlags(cudaDeviceScheduleBlockingSync);
      if (flag_err != cudaSuccess) {
        RCLCPP_WARN(get_logger(), "could not enable CUDA blocking sync (%s); GPU waits will spin",
                    cudaGetErrorString(flag_err));
      }
      if (cv::cuda::getCudaEnabledDeviceCount() < 1) {
        gpu_error_ = "no CUDA device (OpenCV built without CUDA?)";
        return false;
      }
      sgm_ = cv::cuda::createStereoSGM(0, num_disp_, p1_, p2_, uniqueness_, cv::cuda::StereoSGM::MODE_HH4);
      gpu_ok_ = true;
      gpu_error_.clear();
      return true;
    } catch (const cv::Exception & e) {
      gpu_error_ = std::string("CUDA init failed: ") + e.what();
      RCLCPP_ERROR(get_logger(), "%s", gpu_error_.c_str());
      return false;
    }
  }

  // ------------------------------------------------------------------ calibration
  void on_info(const CameraInfo::ConstSharedPtr & m, bool left)
  {
    auto & mine = left ? left_info_ : right_info_;
    // Calibration is static; rebuild only if it changes (e.g. a new calibration was installed)
    if (mine && mine->k == m->k && mine->d == m->d && mine->p == m->p && mine->width == m->width) {
      return;
    }
    mine = m;
    maps_ready_ = false;
  }

  bool build_maps()
  {
    if (!left_info_ || !right_info_) {
      return false;
    }
    const auto & L = *left_info_;
    const auto & R = *right_info_;
    calibrated_ = L.k[0] > 0.0 && R.k[0] > 0.0 && R.p[3] != 0.0 && L.d.size() >= 5 && R.d.size() >= 5;
    if (!calibrated_) {
      return false;
    }
    const double scale = static_cast<double>(out_width_) / L.width;
    out_size_ = cv::Size(out_width_, static_cast<int>(std::lround(L.height * scale)));

    auto build = [&](const CameraInfo & ci, cv::cuda::GpuMat & gx, cv::cuda::GpuMat & gy) {
      cv::Mat K(3, 3, CV_64F, const_cast<double *>(ci.k.data()));
      cv::Mat D(1, static_cast<int>(ci.d.size()), CV_64F, const_cast<double *>(ci.d.data()));
      cv::Mat Rm(3, 3, CV_64F, const_cast<double *>(ci.r.data()));
      cv::Mat P = cv::Mat(3, 4, CV_64F, const_cast<double *>(ci.p.data())).clone();
      P.row(0) *= scale;  // rectify and downscale in the same remap
      P.row(1) *= scale;
      cv::Mat mx, my;
      cv::initUndistortRectifyMap(K, D, Rm, P(cv::Rect(0, 0, 3, 3)), out_size_, CV_32FC1, mx, my);
      gx.upload(mx);
      gy.upload(my);
    };
    build(L, map_lx_, map_ly_);
    build(R, map_rx_, map_ry_);

    fx_ = L.p[0] * scale;
    fy_ = L.p[5] * scale;
    cx_ = L.p[2] * scale;
    cy_ = L.p[6] * scale;
    baseline_ = -R.p[3] / R.p[0];  // Tx = -fx * B
    for (int i = 0; i < 9; ++i) {
      r1_[i] = L.r[i];
    }

    rect_info_ = CameraInfo();
    rect_info_.width = out_size_.width;
    rect_info_.height = out_size_.height;
    rect_info_.distortion_model = "plumb_bob";
    rect_info_.d.assign(5, 0.0);
    rect_info_.k = {fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1};
    rect_info_.r = L.r;
    rect_info_.p = {fx_, 0, cx_, 0, 0, fy_, cy_, 0, 0, 0, 1, 0};

    build_luts();
    maps_ready_ = true;
    RCLCPP_INFO(get_logger(), "rectification ready: %dx%d, f=%.1f px, baseline %.1f mm, range %.2f-%.1f m",
                out_size_.width, out_size_.height, fx_, baseline_ * 1000.0,
                fx_ * baseline_ / (num_disp_ - 1), max_depth_);
    return true;
  }

  /// StereoSGM gives integer disparities * 16 in [0, 16 * num_disparities), so depth and preview colour can be
  /// looked up per disparity value instead of computed per pixel.
  void build_luts()
  {
    const int n = num_disp_ * 16;
    depth_lut_.assign(n, 0);
    colour_lut_.assign(n, cv::Vec3b(0, 0, 0));
    cv::Mat ramp(1, 256, CV_8UC1), turbo;
    for (int i = 0; i < 256; ++i) {
      ramp.at<uint8_t>(i) = static_cast<uint8_t>(i);
    }
    cv::applyColorMap(ramp, turbo, cv::COLORMAP_TURBO);
    const double fb16 = fx_ * baseline_ * 16.0;
    const double lo = min_depth_, hi = preview_max_;
    for (int d = 1; d < n; ++d) {
      const double zm = fb16 / d;
      if (zm < min_depth_ || zm > max_depth_) {
        continue;
      }
      depth_lut_[d] = static_cast<uint16_t>(std::min(zm * 1000.0 + 0.5, 65535.0));
      // Near = red, far = blue (turbo); 0 stays black for "unknown"
      const double t = std::clamp((zm - lo) / (hi - lo), 0.0, 1.0);
      colour_lut_[d] = turbo.at<cv::Vec3b>(static_cast<int>(1 + (1.0 - t) * 254.0));
    }
  }

  // ------------------------------------------------------------------ processing
  void try_process()
  {
    if (!left_ || !right_) {
      return;
    }
    const double tl = stamp_sec(left_->header.stamp), tr = stamp_sec(right_->header.stamp);
    if (std::fabs(tl - tr) > max_pair_dt_) {
      // Drop the older frame and wait for its partner
      (tl < tr ? left_ : right_).reset();
      ++skipped_pairs_;
      return;
    }
    // Rate cap with 20 % slack: pairs arrive on the camera's 50 ms grid with jitter, so a strict
    // 1/max_rate test rejects the pair that is a few ms early and the next accepted one is 50 ms late
    // (10 Hz cap gave ~7.7 Hz).
    const auto t_now = std::chrono::steady_clock::now();
    if (processed_once_ &&
        std::chrono::duration<double>(t_now - last_processed_).count() < 0.8 / max_rate_) {
      return;
    }
    Image::ConstSharedPtr l = left_, r = right_;
    left_.reset();
    right_.reset();
    if (!maps_ready_ && !build_maps()) {
      return;
    }
    if (l->encoding != "bgr8" || r->encoding != "bgr8" || l->width != left_info_->width) {
      bad_input_ = "unexpected image (" + l->encoding + " " + std::to_string(l->width) + "x" +
                   std::to_string(l->height) + ")";
      return;
    }
    last_processed_ = t_now;
    processed_once_ = true;
    try {
      process(l, r);
      bad_input_.clear();
    } catch (const cv::Exception & e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "processing failed: %s", e.what());
    }
  }

  void process(const Image::ConstSharedPtr & l, const Image::ConstSharedPtr & r)
  {
    const auto t0 = std::chrono::steady_clock::now();
    const bool want_depth = depth_pub_->get_subscription_count() > 0;
    const bool want_preview = preview_pub_->get_subscription_count() > 0;
    const bool want_points = points_pub_->get_subscription_count() > 0;

    cv::Mat lm(l->height, l->width, CV_8UC3, const_cast<uint8_t *>(l->data.data()), l->step);
    cv::Mat rm(r->height, r->width, CV_8UC3, const_cast<uint8_t *>(r->data.data()), r->step);
    // Everything on one stream; downloads land in pinned memory and the single wait below sleeps
    // (blocking sync) until the GPU is done.
    g_lbgr_.upload(lm, stream_);
    g_rbgr_.upload(rm, stream_);
    cv::cuda::cvtColor(g_lbgr_, g_lgrey_, cv::COLOR_BGR2GRAY, 0, stream_);
    cv::cuda::cvtColor(g_rbgr_, g_rgrey_, cv::COLOR_BGR2GRAY, 0, stream_);
    cv::cuda::remap(g_lgrey_, g_lrect_, map_lx_, map_ly_, cv::INTER_LINEAR, cv::BORDER_CONSTANT,
                    cv::Scalar(), stream_);
    cv::cuda::remap(g_rgrey_, g_rrect_, map_rx_, map_ry_, cv::INTER_LINEAR, cv::BORDER_CONSTANT,
                    cv::Scalar(), stream_);
    sgm_->compute(g_lrect_, g_rrect_, g_disp_, stream_);
    g_disp_.download(h_disp_, stream_);  // CV_16S, disparity * 16, <= 0 invalid
    if (want_points) {
      cv::cuda::remap(g_lbgr_, g_lcolor_, map_lx_, map_ly_, cv::INTER_LINEAR, cv::BORDER_CONSTANT,
                      cv::Scalar(), stream_);
      g_lcolor_.download(h_color_, stream_);
    }
    stream_.waitForCompletion();
    disp_ = h_disp_.createMatHeader();
    if (want_points) {
      color_ = h_color_.createMatHeader();
    }
    if (speckle_size_ > 0) {
      cv::filterSpeckles(disp_, -16, speckle_size_, speckle_range_ * 16);
    }

    // Depth in mm (16UC1): Z = f * B / d, by lookup (0 = invalid or out of range)
    const int nlut = static_cast<int>(depth_lut_.size());
    const uint16_t * lut = depth_lut_.data();
    depth_mm_.create(disp_.size(), CV_16UC1);
    int valid = 0;
    for (int v = 0; v < disp_.rows; ++v) {
      const int16_t * d = disp_.ptr<int16_t>(v);
      uint16_t * z = depth_mm_.ptr<uint16_t>(v);
      for (int u = 0; u < disp_.cols; ++u) {
        const int dv = d[u];
        const uint16_t zv = (dv > 0 && dv < nlut) ? lut[dv] : 0;
        z[u] = zv;
        valid += zv != 0;
      }
    }

    std_msgs::msg::Header hdr = l->header;  // left stamp, left optical frame
    if (want_depth) {
      auto msg = std::make_unique<Image>();
      msg->header = hdr;
      msg->height = depth_mm_.rows;
      msg->width = depth_mm_.cols;
      msg->encoding = "16UC1";
      msg->step = depth_mm_.cols * 2;
      msg->data.assign(depth_mm_.datastart, depth_mm_.dataend);
      depth_pub_->publish(std::move(msg));
    }
    if (info_pub_->get_subscription_count() > 0 || want_depth) {
      CameraInfo info = rect_info_;
      info.header = hdr;
      info_pub_->publish(info);
    }
    if (want_preview) {
      publish_preview(hdr);
    }
    if (want_points) {
      publish_points(hdr);
    }

    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    proc_ms_ = proc_ms_ <= 0 ? ms : 0.9 * proc_ms_ + 0.1 * ms;
    valid_frac_ = static_cast<double>(valid) / (disp_.rows * disp_.cols);
    ++frames_;
  }

  void publish_preview(const std_msgs::msg::Header & hdr)
  {
    // Near = red, far = blue (turbo), unknown = black; colour looked up per disparity (see build_luts)
    preview_bgr_.create(disp_.size(), CV_8UC3);
    const int nlut = static_cast<int>(colour_lut_.size());
    const cv::Vec3b * lut = colour_lut_.data();
    for (int v = 0; v < disp_.rows; ++v) {
      const int16_t * d = disp_.ptr<int16_t>(v);
      cv::Vec3b * c = preview_bgr_.ptr<cv::Vec3b>(v);
      for (int u = 0; u < disp_.cols; ++u) {
        const int dv = d[u];
        c[u] = (dv > 0 && dv < nlut) ? lut[dv] : cv::Vec3b(0, 0, 0);
      }
    }
    std::vector<uchar> jpg;
    cv::imencode(".jpg", preview_bgr_, jpg, {cv::IMWRITE_JPEG_QUALITY, 80});
    auto msg = std::make_unique<CompressedImage>();
    msg->header = hdr;
    msg->format = "jpeg";
    msg->data = std::move(jpg);
    preview_pub_->publish(std::move(msg));
  }

  void publish_points(const std_msgs::msg::Header & hdr)
  {
    struct P { float x, y, z; uint8_t b, g, r; };
    std::vector<P> pts;
    pts.reserve((depth_mm_.rows / point_step_ + 1) * (depth_mm_.cols / point_step_ + 1));
    for (int v = 0; v < depth_mm_.rows; v += point_step_) {
      const uint16_t * z = depth_mm_.ptr<uint16_t>(v);
      const cv::Vec3b * c = color_.ptr<cv::Vec3b>(v);
      for (int u = 0; u < depth_mm_.cols; u += point_step_) {
        if (z[u] == 0) {
          continue;
        }
        const double Z = z[u] / 1000.0;
        const double X = (u - cx_) * Z / fx_, Y = (v - cy_) * Z / fy_;
        // Rectified -> unrectified left optical frame: p = R1^T * p_rect
        P p;
        p.x = static_cast<float>(r1_[0] * X + r1_[3] * Y + r1_[6] * Z);
        p.y = static_cast<float>(r1_[1] * X + r1_[4] * Y + r1_[7] * Z);
        p.z = static_cast<float>(r1_[2] * X + r1_[5] * Y + r1_[8] * Z);
        p.b = c[u][0];
        p.g = c[u][1];
        p.r = c[u][2];
        pts.push_back(p);
      }
    }
    auto msg = std::make_unique<PointCloud2>();
    msg->header = hdr;
    sensor_msgs::PointCloud2Modifier mod(*msg);
    mod.setPointCloud2FieldsByString(2, "xyz", "rgb");
    mod.resize(pts.size());
    sensor_msgs::PointCloud2Iterator<float> ix(*msg, "x"), iy(*msg, "y"), iz(*msg, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> ir(*msg, "r"), ig(*msg, "g"), ib(*msg, "b");
    for (const auto & p : pts) {
      *ix = p.x; *iy = p.y; *iz = p.z; *ir = p.r; *ig = p.g; *ib = p.b;
      ++ix; ++iy; ++iz; ++ir; ++ig; ++ib;
    }
    points_pub_->publish(std::move(msg));
  }

  // ------------------------------------------------------------------ diagnostics
  void publish_diagnostics()
  {
    DiagnosticStatus st;
    st.name = "Stereo depth";
    st.hardware_id = "stereo_depth";
    const double hz = frames_ - frames_reported_;
    frames_reported_ = frames_;
    if (!gpu_error_.empty()) {
      st.level = DiagnosticStatus::ERROR;
      st.message = gpu_error_;
    } else if (!left_sub_) {
      st.level = DiagnosticStatus::OK;
      st.message = "idle (no subscribers)";
    } else if (left_info_ && right_info_ && !calibrated_ && (left_info_->k[0] == 0.0 || right_info_->p[3] == 0.0)) {
      st.level = DiagnosticStatus::ERROR;
      st.message = "cameras not calibrated (camera_info has no stereo projection)";
    } else if (!bad_input_.empty()) {
      st.level = DiagnosticStatus::ERROR;
      st.message = bad_input_;
    } else if (hz < 0.5) {
      st.level = (now() - active_since_).seconds() > 5.0 ? DiagnosticStatus::WARN : DiagnosticStatus::OK;
      st.message = maps_ready_ ? "no stereo pairs (left/right images missing or out of sync)"
                               : "waiting for camera images and calibration";
    } else {
      st.level = DiagnosticStatus::OK;
      char buf[160];
      std::snprintf(buf, sizeof(buf), "%.0f Hz, %.0f ms per frame, %.0f %% of pixels with depth",
                    hz, proc_ms_, valid_frac_ * 100.0);
      st.message = buf;
    }
    st.values = {kv("num_disparities", std::to_string(num_disp_)),
                 kv("output", std::to_string(out_size_.width) + "x" + std::to_string(out_size_.height)),
                 kv("baseline_mm", std::to_string(baseline_ * 1000.0)),
                 kv("min_range_m", maps_ready_ ? std::to_string(fx_ * baseline_ / (num_disp_ - 1)) : "-"),
                 kv("skipped_pairs", std::to_string(skipped_pairs_))};
    DiagnosticArray arr;
    arr.header.stamp = now();
    arr.status.push_back(st);
    diag_pub_->publish(arr);
  }

  // parameters
  int out_width_, num_disp_, p1_, p2_, uniqueness_, point_step_, speckle_size_, speckle_range_;
  double max_rate_, min_depth_, max_depth_, preview_max_, max_pair_dt_;

  // ROS
  rclcpp::Publisher<Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<CameraInfo>::SharedPtr info_pub_;
  rclcpp::Publisher<CompressedImage>::SharedPtr preview_pub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr points_pub_;
  rclcpp::Publisher<DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::Subscription<Image>::SharedPtr left_sub_, right_sub_;
  rclcpp::Subscription<CameraInfo>::SharedPtr left_info_sub_, right_info_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // state
  Image::ConstSharedPtr left_, right_;
  CameraInfo::ConstSharedPtr left_info_, right_info_;
  CameraInfo rect_info_;
  bool gpu_ok_ = false, maps_ready_ = false, calibrated_ = false;
  std::string gpu_error_, bad_input_;
  rclcpp::Time active_since_{0, 0, RCL_ROS_TIME};
  std::chrono::steady_clock::time_point last_processed_;
  bool processed_once_ = false;
  std::vector<uint16_t> depth_lut_;     // fixed-point disparity -> depth in mm (0 = invalid)
  std::vector<cv::Vec3b> colour_lut_;   // fixed-point disparity -> preview colour
  cv::Size out_size_{0, 0};
  double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0, baseline_ = 0;
  double r1_[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  double proc_ms_ = 0, valid_frac_ = 0;
  uint64_t frames_ = 0, frames_reported_ = 0, skipped_pairs_ = 0;

  // GPU and CPU buffers (reused)
  cv::Ptr<cv::cuda::StereoSGM> sgm_;
  cv::cuda::GpuMat map_lx_, map_ly_, map_rx_, map_ry_;
  cv::cuda::GpuMat g_lbgr_, g_rbgr_, g_lgrey_, g_rgrey_, g_lrect_, g_rrect_, g_disp_, g_lcolor_;
  cv::cuda::Stream stream_;
  cv::cuda::HostMem h_disp_{cv::cuda::HostMem::PAGE_LOCKED}, h_color_{cv::cuda::HostMem::PAGE_LOCKED};
  cv::Mat disp_, depth_mm_, color_, preview_bgr_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StereoDepthNode>());
  rclcpp::shutdown();
  return 0;
}

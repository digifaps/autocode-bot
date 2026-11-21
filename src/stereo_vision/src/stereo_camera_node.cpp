/**
 * @file stereo_camera_node.cpp
 * @brief Stereo camera capture node for dual IMX219 cameras via CSI
 *
 * Captures the Waveshare IMX219-83 stereo camera with one GStreamer pipeline per
 * sensor and publishes, per camera:
 *   camera/<side>/image_raw   sensor_msgs/Image (bgr8)
 *   camera/<side>/compressed  sensor_msgs/CompressedImage (jpeg, hardware nvjpegenc)
 *   camera/<side>/camera_info sensor_msgs/CameraInfo, once per published frame, same stamp
 *
 * Each pipeline splits the NVMM frame with a tee. The raw branch converts to BGR on
 * the CPU; the JPEG branch is encoded by the Jetson's NVJPG engine straight from
 * NVMM memory. A branch only runs while its topic has subscribers.
 *
 * This target deliberately does not link OpenCV/cv_bridge: OpenCV's libjpeg in the
 * same process can clash with nvjpegenc's libjpeg.
 */

#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/video/video.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <camera_calibration_parsers/parse.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <pthread.h>
#include <signal.h>

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <chrono>
#include <deque>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

namespace
{
constexpr int64_t kMaxStampAgeNs = 500'000'000;   // older PTS → fall back to now()
constexpr int64_t kStallTimeoutNs = 3'000'000'000;  // no frames from the sensor → exit (respawn)

/// Blocks SIGINT/SIGTERM in the calling thread for its lifetime. Threads created meanwhile
/// (GStreamer streaming threads, Argus client threads) inherit the blocked mask, so Ctrl+C is
/// only delivered to the main thread. A signal landing in an Argus thread during teardown can
/// leave nvarguscamerasrc stuck in stop().
class ScopedSignalBlock
{
public:
  ScopedSignalBlock()
  {
    sigset_t set;
    sigemptyset(&set);
    sigaddset(&set, SIGINT);
    sigaddset(&set, SIGTERM);
    pthread_sigmask(SIG_BLOCK, &set, &old_);
  }
  ~ScopedSignalBlock()
  {
    pthread_sigmask(SIG_SETMASK, &old_, nullptr);
  }
  ScopedSignalBlock(const ScopedSignalBlock &) = delete;
  ScopedSignalBlock & operator=(const ScopedSignalBlock &) = delete;

private:
  sigset_t old_;
};

int64_t steady_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
}  // namespace

class StereoCameraNode;

struct Camera
{
  StereoCameraNode * node = nullptr;
  std::string side;
  int sensor_id = 0;
  std::string frame_id;

  GstElement * pipeline = nullptr;
  GstElement * raw_valve = nullptr;
  GstElement * jpeg_valve = nullptr;
  bool raw_open = false;
  bool jpeg_open = false;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_pub;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr jpeg_pub;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub;
  sensor_msgs::msg::CameraInfo info;  // from the calibration file, or uncalibrated (zeros)

  // Raw and JPEG branches deliver the same sensor frame on different threads; the first one
  // computes the stamp and publishes camera_info, the other reuses the stamp by PTS.
  std::mutex stamp_mutex;
  std::deque<std::pair<GstClockTime, rclcpp::Time>> stamp_cache;

  std::atomic<int64_t> last_frame_ns{0};   // steady clock, updated per sensor frame
  std::atomic<uint64_t> sensor_frames{0};
  std::atomic<uint64_t> raw_frames{0};
  std::atomic<uint64_t> jpeg_frames{0};
  std::atomic<uint64_t> jpeg_bytes{0};
  std::atomic<uint64_t> stale_stamps{0};
  std::atomic<int64_t> stamp_age_sum_ns{0};
  std::atomic<uint64_t> stamp_age_count{0};
  int64_t started_ns = 0;
};

class StereoCameraNode : public rclcpp::Node
{
public:
  StereoCameraNode() : Node("stereo_camera_node")
  {
    this->declare_parameter("frame_rate", 30.0);
    this->declare_parameter("image_width", 1280);
    this->declare_parameter("image_height", 720);
    this->declare_parameter("left_sensor_id", 1);   // CSI-1 = left (was swapped with 0)
    this->declare_parameter("right_sensor_id", 0);  // CSI-0 = right
    // Must match the optical frame links in robot_description/urdf/robot.urdf.xacro
    this->declare_parameter("left_frame_id", "camera_left_optical");
    this->declare_parameter("right_frame_id", "camera_right_optical");
    this->declare_parameter("publish_raw", true);
    this->declare_parameter("publish_compressed", true);
    this->declare_parameter("jpeg_quality", 85);
    // Max JPEG frames per second per camera (integer, GStreamer videorate); 0 = every frame
    this->declare_parameter("jpeg_max_rate", 10);
    // camera_info YAML (camera_calibration_parsers format). Empty = <share>/config/calibration/<side>.yaml
    this->declare_parameter("left_calibration_file", "");
    this->declare_parameter("right_calibration_file", "");

    frame_rate_ = this->get_parameter("frame_rate").as_double();
    width_ = this->get_parameter("image_width").as_int();
    height_ = this->get_parameter("image_height").as_int();
    publish_raw_ = this->get_parameter("publish_raw").as_bool();
    publish_compressed_ = this->get_parameter("publish_compressed").as_bool();
    jpeg_quality_ = static_cast<int>(this->get_parameter("jpeg_quality").as_int());
    jpeg_max_rate_ = static_cast<int>(this->get_parameter("jpeg_max_rate").as_int());
    if (jpeg_quality_ < 1 || jpeg_quality_ > 100) {
      RCLCPP_WARN(get_logger(), "jpeg_quality %d out of range, using 85", jpeg_quality_);
      jpeg_quality_ = 85;
    }

    gst_init(nullptr, nullptr);

    cams_.emplace_back(std::make_unique<Camera>());
    cams_.emplace_back(std::make_unique<Camera>());
    setup_camera(*cams_[0], "left", this->get_parameter("left_sensor_id").as_int(),
      this->get_parameter("left_frame_id").as_string());
    setup_camera(*cams_[1], "right", this->get_parameter("right_sensor_id").as_int(),
      this->get_parameter("right_frame_id").as_string());

    for (auto & cam : cams_) {
      start_pipeline(*cam);
    }

    // Subscriber gating, bus errors and stall watchdog (exit on failure; launch respawns)
    timer_ = this->create_wall_timer(500ms, std::bind(&StereoCameraNode::housekeeping, this));

    RCLCPP_INFO(get_logger(),
      "Stereo camera node started: %dx%d @ %d fps, raw %s, compressed %s (nvjpegenc q=%d, max %s fps); "
      "branches run only while subscribed",
      width_, height_, static_cast<int>(frame_rate_), publish_raw_ ? "on" : "off",
      publish_compressed_ ? "on" : "off", jpeg_quality_,
      jpeg_max_rate_ > 0 ? std::to_string(jpeg_max_rate_).c_str() : "all");
  }

  ~StereoCameraNode() override
  {
    stop();
  }

  /// Stop both pipelines. Must run before the publishers go away: appsink callbacks publish
  /// from GStreamer streaming threads, and set_state(NULL) waits for those threads.
  void stop()
  {
    if (timer_) {
      timer_->cancel();
    }
    // nvarguscamerasrc can hang in its stop() (joining its Argus consumer thread) when the
    // pipeline goes to NULL mid-stream. Send EOS to both cameras first and let it drain;
    // the valves are opened so EOS reaches the appsinks and is posted on the bus.
    for (auto & cam : cams_) {
      if (cam->pipeline) {
        g_object_set(cam->raw_valve, "drop", FALSE, nullptr);
        g_object_set(cam->jpeg_valve, "drop", FALSE, nullptr);
        gst_element_send_event(cam->pipeline, gst_event_new_eos());
      }
    }
    // In parallel: nvarguscamerasrc occasionally sits out its 5 s acquire-wait in stop().
    std::vector<std::thread> stoppers;
    {
      ScopedSignalBlock block_signals;
      for (auto & cam : cams_) {
        stoppers.emplace_back([this, c = cam.get()] {shutdown_pipeline(*c, 2 * GST_SECOND);});
      }
    }
    for (auto & t : stoppers) {
      t.join();
    }
  }

  /// Wait (up to timeout) for EOS or an error on the bus, then set NULL and release.
  void shutdown_pipeline(Camera & cam, GstClockTime eos_timeout)
  {
    if (!cam.pipeline) {
      return;
    }
    if (eos_timeout > 0) {
      GstBus * bus = gst_element_get_bus(cam.pipeline);
      GstMessage * msg = gst_bus_timed_pop_filtered(bus, eos_timeout,
        static_cast<GstMessageType>(GST_MESSAGE_EOS | GST_MESSAGE_ERROR));
      if (!msg) {
        RCLCPP_WARN(get_logger(), "%s camera: no EOS within %.1f s, stopping anyway",
          cam.side.c_str(), eos_timeout / 1e9);
      } else {
        gst_message_unref(msg);
      }
      gst_object_unref(bus);
    }
    gst_element_set_state(cam.pipeline, GST_STATE_NULL);
    gst_object_unref(cam.raw_valve);
    gst_object_unref(cam.jpeg_valve);
    gst_object_unref(cam.pipeline);
    cam.pipeline = nullptr;
  }

  // Called from GStreamer streaming threads.
  /// Stamp for this sensor frame, identical for every branch that publishes it. The first
  /// branch to see a frame also publishes camera_info with that stamp.
  rclcpp::Time frame_stamp(Camera & cam, GstSample * sample, GstBuffer * buf)
  {
    const GstClockTime pts = GST_BUFFER_PTS(buf);
    if (GST_CLOCK_TIME_IS_VALID(pts)) {
      std::lock_guard<std::mutex> lock(cam.stamp_mutex);
      for (const auto & entry : cam.stamp_cache) {
        if (entry.first == pts) {
          return entry.second;
        }
      }
    }
    const rclcpp::Time stamp = capture_stamp(cam, sample, buf);
    if (GST_CLOCK_TIME_IS_VALID(pts)) {
      std::lock_guard<std::mutex> lock(cam.stamp_mutex);
      cam.stamp_cache.emplace_back(pts, stamp);
      if (cam.stamp_cache.size() > 16) {
        cam.stamp_cache.pop_front();
      }
    }
    auto info = std::make_unique<sensor_msgs::msg::CameraInfo>(cam.info);
    info->header.stamp = stamp;
    info->header.frame_id = cam.frame_id;
    publish(cam.info_pub, std::move(info));
    return stamp;
  }

  rclcpp::Time capture_stamp(Camera & cam, GstSample * sample, GstBuffer * buf)
  {
    const rclcpp::Time now = this->now();
    const GstClockTime pts = GST_BUFFER_PTS(buf);
    if (!GST_CLOCK_TIME_IS_VALID(pts)) {
      cam.stale_stamps++;
      return now;
    }
    const GstSegment * seg = gst_sample_get_segment(sample);
    const GstClockTime running = seg ?
      gst_segment_to_running_time(seg, GST_FORMAT_TIME, pts) : pts;
    GstClock * clock = gst_element_get_clock(cam.pipeline);
    if (!clock || !GST_CLOCK_TIME_IS_VALID(running)) {
      if (clock) {
        gst_object_unref(clock);
      }
      cam.stale_stamps++;
      return now;
    }
    const GstClockTime clock_now = gst_clock_get_time(clock);
    gst_object_unref(clock);
    const GstClockTime base = gst_element_get_base_time(cam.pipeline);
    const int64_t age = static_cast<int64_t>(clock_now) - static_cast<int64_t>(base + running);
    if (age < 0 || age > kMaxStampAgeNs) {
      cam.stale_stamps++;
      return now;
    }
    cam.stamp_age_sum_ns += age;
    cam.stamp_age_count++;
    return now - rclcpp::Duration(std::chrono::nanoseconds(age));
  }

private:
  void setup_camera(Camera & cam, const std::string & side, int64_t sensor_id,
    const std::string & frame_id)
  {
    cam.node = this;
    cam.side = side;
    cam.sensor_id = static_cast<int>(sensor_id);
    cam.frame_id = frame_id;
    cam.raw_pub = this->create_publisher<sensor_msgs::msg::Image>(
      "camera/" + side + "/image_raw", 10);
    cam.jpeg_pub = this->create_publisher<sensor_msgs::msg::CompressedImage>(
      "camera/" + side + "/compressed", 10);
    cam.info_pub = this->create_publisher<sensor_msgs::msg::CameraInfo>(
      "camera/" + side + "/camera_info", 10);
    load_calibration(cam, this->get_parameter(side + "_calibration_file").as_string());
  }

  void load_calibration(Camera & cam, std::string path)
  {
    cam.info = sensor_msgs::msg::CameraInfo();
    cam.info.width = static_cast<uint32_t>(width_);
    cam.info.height = static_cast<uint32_t>(height_);
    const bool explicit_path = !path.empty();
    if (!explicit_path) {
      try {
        path = ament_index_cpp::get_package_share_directory("stereo_vision") +
          "/config/calibration/" + cam.side + ".yaml";
      } catch (const std::exception &) {
        path.clear();
      }
    }
    if (path.empty() || !std::ifstream(path).good()) {
      if (explicit_path) {
        RCLCPP_ERROR(get_logger(), "%s calibration file %s not found; publishing uncalibrated "
          "camera_info", cam.side.c_str(), path.c_str());
      } else {
        RCLCPP_WARN(get_logger(), "No %s camera calibration (%s); publishing uncalibrated "
          "camera_info. See docs/calibration/stereo_wide_angle.md", cam.side.c_str(),
          path.c_str());
      }
      return;
    }
    std::string camera_name;
    sensor_msgs::msg::CameraInfo info;
    if (!camera_calibration_parsers::readCalibration(path, camera_name, info)) {
      RCLCPP_ERROR(get_logger(), "Could not parse %s calibration %s; publishing uncalibrated "
        "camera_info", cam.side.c_str(), path.c_str());
      return;
    }
    if (info.width != cam.info.width || info.height != cam.info.height) {
      RCLCPP_WARN(get_logger(), "%s calibration %s is for %ux%u but the camera runs at %ux%u; "
        "ignoring it (recalibrate at this resolution)", cam.side.c_str(), path.c_str(),
        info.width, info.height, cam.info.width, cam.info.height);
      return;
    }
    cam.info = info;
    RCLCPP_INFO(get_logger(), "%s calibration loaded from %s (fx=%.1f fy=%.1f, %s)",
      cam.side.c_str(), path.c_str(), info.k[0], info.k[4], info.distortion_model.c_str());
  }

  std::string pipeline_description(const Camera & cam) const
  {
    const std::string sink_opts = " max-buffers=1 drop=true sync=false async=false";
    const std::string queue = "queue max-size-buffers=1 max-size-bytes=0 max-size-time=0 leaky=downstream";
    std::string jpeg_rate;
    if (jpeg_max_rate_ > 0) {
      jpeg_rate = " ! videorate drop-only=true max-rate=" + std::to_string(jpeg_max_rate_);
    }
    return
      "nvarguscamerasrc sensor-id=" + std::to_string(cam.sensor_id) +
      " ! video/x-raw(memory:NVMM),width=" + std::to_string(width_) +
      ",height=" + std::to_string(height_) +
      ",framerate=" + std::to_string(static_cast<int>(frame_rate_)) + "/1"
      " ! tee name=t"
      " t. ! " + queue + " ! valve name=raw_valve drop=true"
      " ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=BGR"
      " ! appsink name=raw_sink" + sink_opts +
      // videorate sits before the valve so it sees an unbroken stream of timestamps
      " t. ! " + queue + jpeg_rate + " ! valve name=jpeg_valve drop=true"
      " ! nvjpegenc quality=" + std::to_string(jpeg_quality_) +
      " ! appsink name=jpeg_sink" + sink_opts;
  }

  bool start_pipeline(Camera & cam)
  {
    ScopedSignalBlock block_signals;  // threads spawned here must not receive Ctrl+C
    const std::string desc = pipeline_description(cam);
    RCLCPP_INFO(get_logger(), "Opening %s camera: %s", cam.side.c_str(), desc.c_str());
    GError * err = nullptr;
    GstElement * pipeline = gst_parse_launch(desc.c_str(), &err);
    if (err) {
      RCLCPP_ERROR(get_logger(), "Failed to build %s camera pipeline: %s", cam.side.c_str(),
        err->message);
      g_error_free(err);
      if (pipeline) {
        gst_object_unref(pipeline);
      }
      return false;
    }
    cam.pipeline = pipeline;
    cam.raw_valve = gst_bin_get_by_name(GST_BIN(pipeline), "raw_valve");
    cam.jpeg_valve = gst_bin_get_by_name(GST_BIN(pipeline), "jpeg_valve");
    cam.raw_open = false;
    cam.jpeg_open = false;

    GstElement * raw_sink = gst_bin_get_by_name(GST_BIN(pipeline), "raw_sink");
    GstElement * jpeg_sink = gst_bin_get_by_name(GST_BIN(pipeline), "jpeg_sink");
    GstAppSinkCallbacks raw_cb{};
    raw_cb.new_sample = &StereoCameraNode::on_raw_sample;
    gst_app_sink_set_callbacks(GST_APP_SINK(raw_sink), &raw_cb, &cam, nullptr);
    GstAppSinkCallbacks jpeg_cb{};
    jpeg_cb.new_sample = &StereoCameraNode::on_jpeg_sample;
    gst_app_sink_set_callbacks(GST_APP_SINK(jpeg_sink), &jpeg_cb, &cam, nullptr);
    gst_object_unref(raw_sink);
    gst_object_unref(jpeg_sink);

    // Count frames at the tee so the watchdog works while both branches are closed.
    GstElement * tee = gst_bin_get_by_name(GST_BIN(pipeline), "t");
    GstPad * pad = gst_element_get_static_pad(tee, "sink");
    gst_pad_add_probe(pad, GST_PAD_PROBE_TYPE_BUFFER, &StereoCameraNode::on_sensor_frame, &cam,
      nullptr);
    gst_object_unref(pad);
    gst_object_unref(tee);

    cam.started_ns = steady_ns();
    cam.last_frame_ns = 0;
    update_valves(cam, true);
    if (gst_element_set_state(pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
      RCLCPP_ERROR(get_logger(), "Failed to start %s camera (sensor-id=%d)", cam.side.c_str(),
        cam.sensor_id);
      return false;
    }
    return true;
  }

  /// A camera failed (stalled, pipeline error, nvargus-daemon gone). Argus cannot recover
  /// inside the same process: after a daemon restart, even tearing the old pipeline down
  /// segfaults in libnvargus_socketclient. Exit without teardown and let launch respawn us.
  [[noreturn]] void fail(const Camera & cam, const std::string & reason)
  {
    RCLCPP_FATAL(get_logger(), "%s camera failed: %s. Exiting so launch can respawn the node.",
      cam.side.c_str(), reason.c_str());
    fflush(nullptr);
    std::_Exit(2);
  }

  void update_valves(Camera & cam, bool force = false)
  {
    const size_t raw_subs = cam.raw_pub->get_subscription_count();
    const size_t jpeg_subs = cam.jpeg_pub->get_subscription_count();
    const bool raw = publish_raw_ && raw_subs > 0;
    const bool jpeg = publish_compressed_ && jpeg_subs > 0;
    if (force || raw != cam.raw_open) {
      g_object_set(cam.raw_valve, "drop", raw ? FALSE : TRUE, nullptr);
      if (!force || raw) {
        RCLCPP_INFO(get_logger(), "%s raw: %s (%zu subscribers)", cam.side.c_str(),
          raw ? "on" : "off", raw_subs);
      }
      cam.raw_open = raw;
    }
    if (force || jpeg != cam.jpeg_open) {
      g_object_set(cam.jpeg_valve, "drop", jpeg ? FALSE : TRUE, nullptr);
      if (!force || jpeg) {
        RCLCPP_INFO(get_logger(), "%s compressed: %s (%zu subscribers)", cam.side.c_str(),
          jpeg ? "on" : "off", jpeg_subs);
      }
      cam.jpeg_open = jpeg;
    }
  }

  void housekeeping()
  {
    const int64_t now = steady_ns();
    for (auto & cam_ptr : cams_) {
      Camera & cam = *cam_ptr;
      if (!cam.pipeline) {
        fail(cam, "pipeline not running");
      }
      // Bus: errors and EOS (e.g. nvargus-daemon failures)
      GstBus * bus = gst_element_get_bus(cam.pipeline);
      std::string fatal;
      while (GstMessage * msg = gst_bus_pop_filtered(bus,
        static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_WARNING | GST_MESSAGE_EOS)))
      {
        if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_EOS) {
          fatal = "end of stream";
        } else {
          GError * err = nullptr;
          gchar * dbg = nullptr;
          const bool is_error = GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR;
          if (is_error) {
            gst_message_parse_error(msg, &err, &dbg);
          } else {
            gst_message_parse_warning(msg, &err, &dbg);
          }
          RCLCPP_WARN(get_logger(), "%s camera %s from %s: %s", cam.side.c_str(),
            is_error ? "error" : "warning", GST_OBJECT_NAME(GST_MESSAGE_SRC(msg)),
            err ? err->message : "?");
          if (is_error) {
            fatal = err ? err->message : "error";
          }
          if (err) {
            g_error_free(err);
          }
          g_free(dbg);
        }
        gst_message_unref(msg);
      }
      gst_object_unref(bus);
      if (!fatal.empty()) {
        fail(cam, fatal);
      }

      const int64_t last = cam.last_frame_ns.load();
      const int64_t since = now - (last ? last : cam.started_ns);
      if (since > kStallTimeoutNs) {
        fail(cam, "no frames for " + std::to_string(since / 1'000'000) + " ms");
      }
      update_valves(cam);
    }

    // Status every 30 s
    if (now - last_status_ns_ >= 30'000'000'000LL) {
      const double dt = last_status_ns_ ? (now - last_status_ns_) * 1e-9 : 0.0;
      last_status_ns_ = now;
      if (dt > 0.0) {
        for (auto & cam_ptr : cams_) {
          Camera & cam = *cam_ptr;
          const uint64_t sensor = cam.sensor_frames.exchange(0);
          const uint64_t raw = cam.raw_frames.exchange(0);
          const uint64_t jpeg = cam.jpeg_frames.exchange(0);
          const uint64_t bytes = cam.jpeg_bytes.exchange(0);
          const uint64_t n_age = cam.stamp_age_count.exchange(0);
          const int64_t sum_age = cam.stamp_age_sum_ns.exchange(0);
          const uint64_t stale = cam.stale_stamps.exchange(0);
          RCLCPP_INFO(get_logger(),
            "%s: sensor %.1f fps, raw %.1f fps, jpeg %.1f fps (%.0f kB avg), "
            "capture-to-publish %.1f ms, %lu stamps fell back to now()",
            cam.side.c_str(), sensor / dt, raw / dt, jpeg / dt,
            jpeg ? bytes / 1024.0 / jpeg : 0.0,
            n_age ? sum_age / 1e6 / n_age : 0.0, static_cast<unsigned long>(stale));
        }
      }
    }
  }

  static GstPadProbeReturn on_sensor_frame(GstPad *, GstPadProbeInfo *, gpointer user)
  {
    auto * cam = static_cast<Camera *>(user);
    cam->last_frame_ns = steady_ns();
    cam->sensor_frames++;
    return GST_PAD_PROBE_OK;
  }

  static GstFlowReturn on_raw_sample(GstAppSink * sink, gpointer user)
  {
    auto * cam = static_cast<Camera *>(user);
    GstSample * sample = gst_app_sink_pull_sample(sink);
    if (!sample) {
      return GST_FLOW_OK;
    }
    GstBuffer * buf = gst_sample_get_buffer(sample);
    GstVideoInfo info;
    GstMapInfo map;
    if (buf && gst_video_info_from_caps(&info, gst_sample_get_caps(sample)) &&
      gst_buffer_map(buf, &map, GST_MAP_READ))
    {
      const size_t stride = static_cast<size_t>(GST_VIDEO_INFO_PLANE_STRIDE(&info, 0));
      const size_t height = static_cast<size_t>(GST_VIDEO_INFO_HEIGHT(&info));
      if (map.size >= stride * height) {
        auto msg = std::make_unique<sensor_msgs::msg::Image>();
        msg->header.stamp = cam->node->frame_stamp(*cam, sample, buf);
        msg->header.frame_id = cam->frame_id;
        msg->height = static_cast<uint32_t>(height);
        msg->width = static_cast<uint32_t>(GST_VIDEO_INFO_WIDTH(&info));
        msg->encoding = "bgr8";
        msg->is_bigendian = 0;
        msg->step = static_cast<uint32_t>(stride);
        msg->data.assign(map.data, map.data + stride * height);
        publish(cam->raw_pub, std::move(msg));
        cam->raw_frames++;
      }
      gst_buffer_unmap(buf, &map);
    }
    gst_sample_unref(sample);
    return GST_FLOW_OK;
  }

  static GstFlowReturn on_jpeg_sample(GstAppSink * sink, gpointer user)
  {
    auto * cam = static_cast<Camera *>(user);
    GstSample * sample = gst_app_sink_pull_sample(sink);
    if (!sample) {
      return GST_FLOW_OK;
    }
    GstBuffer * buf = gst_sample_get_buffer(sample);
    GstMapInfo map;
    if (buf && gst_buffer_map(buf, &map, GST_MAP_READ)) {
      auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
      msg->header.stamp = cam->node->frame_stamp(*cam, sample, buf);
      msg->header.frame_id = cam->frame_id;
      msg->format = "jpeg";
      msg->data.assign(map.data, map.data + map.size);
      publish(cam->jpeg_pub, std::move(msg));
      cam->jpeg_frames++;
      cam->jpeg_bytes += map.size;
      gst_buffer_unmap(buf, &map);
    }
    gst_sample_unref(sample);
    return GST_FLOW_OK;
  }

  template<typename PubT, typename MsgT>
  static void publish(const PubT & pub, std::unique_ptr<MsgT> msg)
  {
    // Streaming threads can race shutdown; never let an exception escape into GStreamer.
    if (!rclcpp::ok()) {
      return;
    }
    try {
      pub->publish(std::move(msg));
    } catch (const std::exception &) {
    }
  }

  double frame_rate_;
  int width_;
  int height_;
  bool publish_raw_;
  bool publish_compressed_;
  int jpeg_quality_;
  int jpeg_max_rate_;
  std::vector<std::unique_ptr<Camera>> cams_;
  rclcpp::TimerBase::SharedPtr timer_;
  int64_t last_status_ns_ = 0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StereoCameraNode>();
  rclcpp::spin(node);
  node->stop();
  node.reset();
  rclcpp::shutdown();
  return 0;
}

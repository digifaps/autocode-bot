/**
 * @file stereo_camera_node.cpp
 * @brief Stereo camera capture node for dual IMX219 cameras via CSI
 *
 * Captures synchronized stereo images from the Waveshare IMX219-83 stereo camera
 * and publishes them as ROS2 image messages.
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>

using namespace std::chrono_literals;

class StereoCameraNode : public rclcpp::Node
{
public:
  StereoCameraNode() : Node("stereo_camera_node")
  {
    // Declare parameters
    this->declare_parameter("frame_rate", 30.0);
    this->declare_parameter("image_width", 1280);
    this->declare_parameter("image_height", 720);
    this->declare_parameter("left_sensor_id", 0);
    this->declare_parameter("right_sensor_id", 1);

    frame_rate_ = this->get_parameter("frame_rate").as_double();
    width_ = this->get_parameter("image_width").as_int();
    height_ = this->get_parameter("image_height").as_int();
    left_sensor_id_ = this->get_parameter("left_sensor_id").as_int();
    right_sensor_id_ = this->get_parameter("right_sensor_id").as_int();

    // Create publishers
    left_pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera/left/image_raw", 10);
    right_pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera/right/image_raw", 10);

    // Build GStreamer pipelines
    std::string left_pipeline = build_gstreamer_pipeline(left_sensor_id_);
    std::string right_pipeline = build_gstreamer_pipeline(right_sensor_id_);

    RCLCPP_INFO(this->get_logger(), "Opening left camera: %s", left_pipeline.c_str());
    left_cap_.open(left_pipeline, cv::CAP_GSTREAMER);

    RCLCPP_INFO(this->get_logger(), "Opening right camera: %s", right_pipeline.c_str());
    right_cap_.open(right_pipeline, cv::CAP_GSTREAMER);

    if (!left_cap_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open left camera!");
      return;
    }
    if (!right_cap_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open right camera!");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Both cameras opened successfully");

    // Create timer for capture
    auto period = std::chrono::duration<double>(1.0 / frame_rate_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&StereoCameraNode::capture_and_publish, this));

    RCLCPP_INFO(this->get_logger(), "Stereo camera node started at %.1f FPS", frame_rate_);
  }

  ~StereoCameraNode()
  {
    if (left_cap_.isOpened()) left_cap_.release();
    if (right_cap_.isOpened()) right_cap_.release();
  }

private:
  std::string build_gstreamer_pipeline(int sensor_id)
  {
    // GStreamer pipeline for NVIDIA Argus camera source
    return "nvarguscamerasrc sensor-id=" + std::to_string(sensor_id) +
           " ! video/x-raw(memory:NVMM), width=" + std::to_string(width_) +
           ", height=" + std::to_string(height_) +
           ", framerate=" + std::to_string(static_cast<int>(frame_rate_)) + "/1" +
           " ! nvvidconv ! video/x-raw, format=BGRx" +
           " ! videoconvert ! video/x-raw, format=BGR ! appsink drop=1";
  }

  void capture_and_publish()
  {
    cv::Mat left_frame, right_frame;

    // Capture from both cameras (software sync - grab as close together as possible)
    left_cap_.grab();
    right_cap_.grab();
    left_cap_.retrieve(left_frame);
    right_cap_.retrieve(right_frame);

    if (left_frame.empty() || right_frame.empty()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "Empty frame received");
      return;
    }

    // Get current timestamp
    auto stamp = this->now();

    // Convert and publish left image
    auto left_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", left_frame).toImageMsg();
    left_msg->header.stamp = stamp;
    left_msg->header.frame_id = "left_camera_optical_frame";
    left_pub_->publish(*left_msg);

    // Convert and publish right image
    auto right_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", right_frame).toImageMsg();
    right_msg->header.stamp = stamp;
    right_msg->header.frame_id = "right_camera_optical_frame";
    right_pub_->publish(*right_msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  cv::VideoCapture left_cap_;
  cv::VideoCapture right_cap_;

  double frame_rate_;
  int width_;
  int height_;
  int left_sensor_id_;
  int right_sensor_id_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StereoCameraNode>());
  rclcpp::shutdown();
  return 0;
}

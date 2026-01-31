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
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.hpp>
#include <opencv2/opencv.hpp>

class StereoCameraNode : public rclcpp::Node
{
public:
  StereoCameraNode() : Node("stereo_camera_node")
  {
    // Declare parameters
    this->declare_parameter("frame_rate", 30.0);
    this->declare_parameter("image_width", 1920);
    this->declare_parameter("image_height", 1080);
    this->declare_parameter("left_camera_info_url", "");
    this->declare_parameter("right_camera_info_url", "");

    // TODO: Initialize GStreamer pipelines for CSI cameras
    // TODO: Create image publishers
    // TODO: Start capture timer

    RCLCPP_INFO(this->get_logger(), "Stereo camera node initialized");
  }

private:
  void capture_and_publish()
  {
    // TODO: Capture stereo pair
    // TODO: Software synchronization
    // TODO: Publish left and right images
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StereoCameraNode>());
  rclcpp::shutdown();
  return 0;
}

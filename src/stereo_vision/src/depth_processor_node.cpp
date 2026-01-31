/**
 * @file depth_processor_node.cpp
 * @brief Stereo depth processing node with CUDA acceleration
 *
 * Performs stereo rectification and block matching to generate depth maps
 * and point clouds from stereo image pairs.
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/stereo_camera_model.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

#ifdef USE_CUDA
#include <opencv2/cudastereo.hpp>
#endif

class DepthProcessorNode : public rclcpp::Node
{
public:
  DepthProcessorNode() : Node("depth_processor_node")
  {
    // Declare parameters
    this->declare_parameter("use_cuda", true);
    this->declare_parameter("num_disparities", 128);
    this->declare_parameter("block_size", 11);
    this->declare_parameter("min_depth", 0.3);  // meters
    this->declare_parameter("max_depth", 10.0);  // meters

    // TODO: Subscribe to stereo image topics
    // TODO: Load calibration data
    // TODO: Initialize stereo matcher (CPU or CUDA)
    // TODO: Create depth and point cloud publishers

    RCLCPP_INFO(this->get_logger(), "Depth processor node initialized");
  }

private:
  void stereo_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr & left_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & right_msg)
  {
    // TODO: Rectify images
    // TODO: Compute disparity map
    // TODO: Convert to depth map
    // TODO: Generate point cloud
    // TODO: Publish results
  }

  image_geometry::StereoCameraModel stereo_model_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DepthProcessorNode>());
  rclcpp::shutdown();
  return 0;
}

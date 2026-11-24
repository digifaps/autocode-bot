cmake_minimum_required(VERSION 3.8)
project(stereo_vision)

# Default to an optimized build; plain `colcon build` otherwise compiles without -O.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(camera_calibration_parsers REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(image_geometry REQUIRED)
# OpenCV 4.10 with CUDA (built into /usr/local on the Jetson); the stereo_depth node needs its CUDA modules
find_package(OpenCV 4.10 REQUIRED COMPONENTS core imgproc calib3d imgcodecs cudaimgproc cudawarping cudastereo)
# Camera capture + hardware JPEG (nvarguscamerasrc / nvjpegenc come with L4T)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED IMPORTED_TARGET gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Stereo camera node
add_executable(stereo_camera_node
  src/stereo_camera_node.cpp
)
ament_target_dependencies(stereo_camera_node
  rclcpp
  sensor_msgs
  camera_calibration_parsers
  ament_index_cpp
  diagnostic_msgs
)
# No OpenCV/cv_bridge here: OpenCV's libjpeg in the same process can clash with nvjpegenc.
target_link_libraries(stereo_camera_node
  PkgConfig::GST
)

# Stereo depth on the GPU (rectify + StereoSGM): depth image, preview and point cloud
add_executable(stereo_depth_node
  src/stereo_depth_node.cpp
)
ament_target_dependencies(stereo_depth_node
  rclcpp
  sensor_msgs
  diagnostic_msgs
)
target_link_libraries(stereo_depth_node
  ${OpenCV_LIBS}
)

# Install targets
install(TARGETS
  stereo_camera_node
  stereo_depth_node
  DESTINATION lib/${PROJECT_NAME}
)

# Headless stereo calibration tool (preview in Foxglove): ros2 run stereo_vision stereo_calibrator.py
install(PROGRAMS
  scripts/stereo_calibrator.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

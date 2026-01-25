cmake_minimum_required(VERSION 3.8)
project(stereo_vision)

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
find_package(image_geometry REQUIRED)
find_package(OpenCV REQUIRED)

# Optional: CUDA support for accelerated stereo matching
find_package(CUDA QUIET)
if(CUDA_FOUND)
  message(STATUS "CUDA found: ${CUDA_VERSION}")
  add_definitions(-DUSE_CUDA)
  include_directories(${CUDA_INCLUDE_DIRS})
endif()

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
  std_msgs
  sensor_msgs
  image_transport
  cv_bridge
  camera_info_manager
)
target_link_libraries(stereo_camera_node
  ${OpenCV_LIBS}
)

# Depth processor node
add_executable(depth_processor_node
  src/depth_processor_node.cpp
)
ament_target_dependencies(depth_processor_node
  rclcpp
  std_msgs
  sensor_msgs
  image_transport
  cv_bridge
  image_geometry
)
target_link_libraries(depth_processor_node
  ${OpenCV_LIBS}
)

# Install targets
install(TARGETS
  stereo_camera_node
  depth_processor_node
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

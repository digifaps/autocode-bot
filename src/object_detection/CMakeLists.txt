cmake_minimum_required(VERSION 3.8)
project(object_detection)

# Default to an optimized build; plain `colcon build` otherwise compiles without -O.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(foxglove_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
# OpenCV 4.10 with CUDA (built into /usr/local on the Jetson): GPU preprocessing
find_package(OpenCV 4.10 REQUIRED COMPONENTS core imgproc calib3d cudaarithm cudawarping)
find_package(CUDAToolkit REQUIRED)
# TensorRT (comes with JetPack): runtime only, engines are built by scripts/fetch_detector_models.sh
find_library(NVINFER_LIB nvinfer REQUIRED)

add_executable(object_detector_node src/object_detector_node.cpp)
target_include_directories(object_detector_node PRIVATE ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(object_detector_node
  rclcpp rclcpp_lifecycle lifecycle_msgs sensor_msgs visualization_msgs vision_msgs diagnostic_msgs foxglove_msgs)
target_link_libraries(object_detector_node ${OpenCV_LIBS} CUDA::cudart ${NVINFER_LIB})

install(TARGETS object_detector_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

cmake_minimum_required(VERSION 3.8)
project(robot_odometry)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(imu_odom_node src/imu_odom_node.cpp)
ament_target_dependencies(imu_odom_node rclcpp sensor_msgs nav_msgs geometry_msgs tf2_ros)

add_executable(scan_resampler src/scan_resampler.cpp)
ament_target_dependencies(scan_resampler rclcpp sensor_msgs)

install(TARGETS imu_odom_node scan_resampler DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

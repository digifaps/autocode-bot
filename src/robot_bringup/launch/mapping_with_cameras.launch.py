"""Launch mapping (LiDAR + IMU + slam_toolbox) plus stereo cameras.

Same as mapping.launch.py plus:
- Stereo camera node (/camera/left/image_raw, /camera/right/image_raw)
- Compressed image relay (for Foxglove: /camera/left/compressed, /camera/right/compressed)
- Optional depth processor (set use_depth:=false to disable)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def _load_robot_description():
    try:
        import xacro
        pkg_share = get_package_share_directory("robot_description")
        xacro_path = os.path.join(pkg_share, "urdf", "robot.urdf.xacro")
        doc = xacro.process_file(xacro_path)
        return doc.toxml()
    except Exception:
        return (
            '<?xml version="1.0"?><robot name="autocode_bot">'
            '<link name="base_footprint"/>'
            '<joint name="base_joint" type="fixed">'
            '<parent link="base_footprint"/><child link="base_link"/>'
            '</joint><link name="base_link"/></robot>'
        )


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    i2c_bus = LaunchConfiguration("i2c_bus", default="8")
    i2c_addr = LaunchConfiguration("i2c_addr", default="104")
    lidar_use_usb = LaunchConfiguration("lidar_use_usb", default="true")
    lidar_use_power = LaunchConfiguration("lidar_use_power", default="false")
    lidar_power_gpio = LaunchConfiguration("lidar_power_gpio_pin", default="12")
    use_depth = LaunchConfiguration("use_depth", default="false")

    robot_description = _load_robot_description()
    bringup_share = get_package_share_directory("robot_bringup")
    slam_params = os.path.join(bringup_share, "config", "slam_toolbox_params.yaml")

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"robot_description": robot_description},
        ],
    )

    imu_node = Node(
        package="motor_driver",
        executable="imu_node",
        name="imu_node",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"i2c_bus": i2c_bus},
            {"i2c_addr": i2c_addr},
        ],
        output="screen",
    )

    imu_odom_node = Node(
        package="motor_driver",
        executable="imu_odom_node",
        name="imu_odom_node",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        parameters=[
            slam_params,
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
    )

    stereo_camera = Node(
        package="stereo_vision",
        executable="stereo_camera_node",
        name="stereo_camera",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"frame_rate": 20.0},
        ],
        output="screen",
    )

    compressed_image = Node(
        package="image_relay",
        executable="compressed_image_node",
        name="compressed_image_node",
        parameters=[
            {"jpeg_quality": 85},
            {"max_hz": 10.0},
        ],
        output="screen",
    )

    depth_processor = Node(
        package="stereo_vision",
        executable="depth_processor_node",
        name="depth_processor",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
        condition=IfCondition(use_depth),
    )

    d500_lidar_launch_path = os.path.join(
        get_package_share_directory("robot_bringup"),
        "launch",
        "d500_lidar.launch.py",
    )
    d500_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(d500_lidar_launch_path),
        launch_arguments=[
            ("use_usb", lidar_use_usb),
            ("use_lidar_power", lidar_use_power),
            ("power_enable_gpio_pin", lidar_power_gpio),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false", description="Use simulation time"),
        DeclareLaunchArgument("i2c_bus", default_value="8", description="I2C bus for ICM20948"),
        DeclareLaunchArgument("i2c_addr", default_value="104", description="I2C address for ICM20948"),
        DeclareLaunchArgument("lidar_use_usb", default_value="true", description="Use USB for D500 LiDAR"),
        DeclareLaunchArgument("lidar_use_power", default_value="false", description="LiDAR GPIO power control"),
        DeclareLaunchArgument("lidar_power_gpio_pin", default_value="12", description="GPIO for LiDAR power"),
        DeclareLaunchArgument("use_depth", default_value="false", description="Run depth_processor node (stub)"),
        robot_state_publisher,
        imu_node,
        imu_odom_node,
        d500_include,
        slam_toolbox_node,
        stereo_camera,
        compressed_image,
        depth_processor,
    ])

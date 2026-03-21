"""Launch file for bringing up the complete robot system."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _load_robot_description():
    """Load URDF from xacro if available; otherwise return minimal URDF so other nodes still run."""
    try:
        import xacro
        pkg_share = get_package_share_directory("robot_description")
        xacro_path = os.path.join(pkg_share, "urdf", "robot.urdf.xacro")
        doc = xacro.process_file(xacro_path)
        return doc.toxml()
    except Exception:
        return '<?xml version="1.0"?><robot name="autocode_bot"><link name="base_footprint"/><joint name="base_joint" type="fixed"><parent link="base_footprint"/><child link="base_link"/></joint><link name="base_link"/></robot>'


def generate_launch_description():
    """Generate launch description for the robot."""

    # Launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    i2c_bus = LaunchConfiguration("i2c_bus", default="8")
    i2c_addr = LaunchConfiguration("i2c_addr", default="104")  # 0x68

    robot_description = _load_robot_description()

    # Robot state publisher (parameter takes precedence over file argument)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"robot_description": robot_description},
        ],
    )

    # Motor driver node
    motor_driver = Node(
        package="motor_driver",
        executable="motor_driver_node",
        name="motor_driver",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    # IMU node (publishes /imu/data_raw from stereo camera board)
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

    # Stereo camera node
    stereo_camera = Node(
        package="stereo_vision",
        executable="stereo_camera_node",
        name="stereo_camera",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    # Depth processor node
    depth_processor = Node(
        package="stereo_vision",
        executable="depth_processor_node",
        name="depth_processor",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    # CompressedImage relay: raw -> JPEG for rosbridge/Foxglove (reduces memory/bandwidth)
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

    # D500 LiDAR (USB by default when launched with robot)
    lidar_use_usb = LaunchConfiguration("lidar_use_usb", default="true")
    lidar_use_power = LaunchConfiguration("lidar_use_power", default="false")
    lidar_power_gpio = LaunchConfiguration("lidar_power_gpio_pin", default="12")
    d500_lidar_launch_path = PathJoinSubstitution([
        FindPackageShare("robot_bringup"),
        "launch",
        "d500_lidar.launch.py",
    ])
    d500_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(d500_lidar_launch_path),
        launch_arguments=[
            ("use_usb", lidar_use_usb),
            ("use_lidar_power", lidar_use_power),
            ("power_enable_gpio_pin", lidar_power_gpio),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation clock if true",
        ),
        DeclareLaunchArgument(
            "i2c_bus",
            default_value="8",
            description="I2C bus number for ICM20948 IMU (e.g. 8 on Jetson with stereo board)",
        ),
        DeclareLaunchArgument(
            "i2c_addr",
            default_value="104",
            description="I2C address for ICM20948 (104 = 0x68, 105 = 0x69)",
        ),
        DeclareLaunchArgument(
            "lidar_use_usb",
            default_value="true",
            description="Use USB for D500 LiDAR (default /dev/ttyUSB0); false = onboard UART /dev/ttyTHS1",
        ),
        DeclareLaunchArgument(
            "lidar_use_power",
            default_value="false",
            description="Enable LiDAR GPIO power control (lidar_control_node)",
        ),
        DeclareLaunchArgument(
            "lidar_power_gpio_pin",
            default_value="12",
            description="Jetson BOARD GPIO pin for LiDAR power enable",
        ),
        robot_state_publisher,
        motor_driver,
        imu_node,
        stereo_camera,
        depth_processor,
        compressed_image,
        d500_lidar,
    ])

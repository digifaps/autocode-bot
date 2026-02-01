"""Launch file for Waveshare D500 LiDAR (LDROBOT LD19 compatible)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for D500 LiDAR."""
    port_name_arg = DeclareLaunchArgument(
        "port_name",
        default_value="/dev/ttyTHS1",
        description="Serial port for D500 LiDAR (e.g. /dev/ttyTHS1 for onboard UART, /dev/ttyUSB0 for USB adapter)",
    )
    use_lidar_power_arg = DeclareLaunchArgument(
        "use_lidar_power",
        default_value="false",
        description="If true, start lidar_control_node to switch LiDAR power via GPIO",
    )
    power_enable_gpio_pin_arg = DeclareLaunchArgument(
        "power_enable_gpio_pin",
        default_value="12",
        description="Jetson 40-pin BOARD number for LiDAR power enable (0 = disabled)",
    )

    lidar_control_node = Node(
        package="lidar_control",
        executable="lidar_control_node",
        name="lidar_control_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_lidar_power")),
        parameters=[
            {
                "power_enable_gpio_pin": LaunchConfiguration("power_enable_gpio_pin"),
            },
            {"initial_power_on": True},
            {"scan_speed_hz": 10.0},
        ],
    )

    ldlidar_node = Node(
        package="ldlidar_stl_ros2",
        executable="ldlidar_stl_ros2_node",
        name="d500_lidar",
        output="screen",
        parameters=[
            {"product_name": "LDLiDAR_LD19"},
            {"topic_name": "scan"},
            {"frame_id": "base_laser"},
            {"port_name": LaunchConfiguration("port_name")},
            {"port_baudrate": 230400},
            {"laser_scan_dir": True},
            {"enable_angle_crop_func": False},
            {"angle_crop_min": 135.0},
            {"angle_crop_max": 225.0},
        ],
    )

    base_link_to_laser = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_base_laser",
        arguments=["0", "0", "0.18", "0", "0", "0", "base_link", "base_laser"],
    )

    ld = LaunchDescription()
    ld.add_action(port_name_arg)
    ld.add_action(use_lidar_power_arg)
    ld.add_action(power_enable_gpio_pin_arg)
    ld.add_action(lidar_control_node)
    ld.add_action(ldlidar_node)
    ld.add_action(base_link_to_laser)
    return ld

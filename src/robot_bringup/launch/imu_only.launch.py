"""Launch only the IMU node (ICM20948 on I2C). Useful for testing without motors/cameras."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("i2c_bus", default_value="8", description="I2C bus number (e.g. 8 for /dev/i2c-8)"),
        DeclareLaunchArgument("i2c_addr", default_value="104", description="I2C address (104 = 0x68)"),
        Node(
            package="motor_driver",
            executable="imu_node",
            name="imu_node",
            parameters=[
                {"i2c_bus": LaunchConfiguration("i2c_bus")},
                {"i2c_addr": LaunchConfiguration("i2c_addr")},
            ],
            output="screen",
        ),
    ])

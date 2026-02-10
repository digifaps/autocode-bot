"""Launch only the CompressedImage relay (for use with stereo + rosbridge when robot.launch.py is not used)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("max_hz", default_value="10.0", description="Max publish rate (Hz) per camera"),
        DeclareLaunchArgument("jpeg_quality", default_value="85", description="JPEG quality 1-100"),
        Node(
            package="image_relay",
            executable="compressed_image_node",
            name="compressed_image_node",
            parameters=[
                {"max_hz": LaunchConfiguration("max_hz")},
                {"jpeg_quality": LaunchConfiguration("jpeg_quality")},
            ],
            output="screen",
        ),
    ])

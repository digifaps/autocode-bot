"""Launch stereo cameras and compressed image relay only (no motors, IMU, depth)."""

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    stereo_camera = Node(
        package="stereo_vision",
        executable="stereo_camera_node",
        name="stereo_camera",
        parameters=[{"use_sim_time": use_sim_time}],
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

    return LaunchDescription([
        stereo_camera,
        compressed_image,
    ])

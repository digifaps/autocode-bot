"""Launch stereo cameras only: raw + hardware JPEG compressed topics (no motors, IMU, depth)."""

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    stereo_camera = Node(
        package="stereo_vision",
        executable="stereo_camera_node",
        name="stereo_camera",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"frame_rate": 20.0},
            # Hardware JPEG (nvjpegenc) on camera/{left,right}/compressed
            {"jpeg_quality": 85},
            {"jpeg_max_rate": 10},
        ],
        output="screen",
        # nvarguscamerasrc can take up to its 5 s acquire-wait to stop; don't escalate at 5 s
        sigterm_timeout="8",
        # The node exits if a camera stalls or nvargus-daemon restarts (Argus cannot recover in-process)
        respawn=True,
        respawn_delay=3.0,
    )

    return LaunchDescription([
        stereo_camera,
    ])

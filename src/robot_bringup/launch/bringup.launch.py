"""Everything the robot runs at boot (scripts/install_autostart.sh -> autocode-bot.service).

- robot.launch.py: robot model, IMU, stereo cameras, D500 LiDAR
- laser odometry: scan_resampler + rf2o_laser_odometry + imu_odom_node (/odom, odom -> base_footprint)
- foxglove_bridge on port 8765 (ws://<ip>:8765, also ws://10.42.0.1:8765 on the robot's access point)
- robot_health: OK/WARN/ERROR per sensor on /diagnostics (Foxglove Diagnostics panels)
- mapping_manager: /mapping/start, /mapping/stop, /mapping/save (slam_toolbox on demand)
- stereo_depth: depth image, preview and point cloud on the GPU (idle until subscribed)
- object_detector: YOLOX on the DLA, boxes + 3D positions (idle until subscribed)
- vision_manager: /vision/{depth,detection}/enable, /vision/detection/accurate (Foxglove Vision panel)

Every node respawns on its own: a missing or failing sensor shows up in Foxglove and never stops
the rest of the robot.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

RESPAWN = {"respawn": True, "respawn_delay": 3.0}


def generate_launch_description():
    bringup_share = get_package_share_directory("robot_bringup")
    foxglove_port = LaunchConfiguration("foxglove_port")

    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "robot.launch.py")),
    )

    scan_resampler = Node(
        package="robot_odometry", executable="scan_resampler", name="scan_resampler",
        output="screen", **RESPAWN,
    )
    rf2o = Node(
        package="rf2o_laser_odometry", executable="rf2o_laser_odometry_node",
        name="rf2o_laser_odometry",
        parameters=[os.path.join(bringup_share, "config", "rf2o_params.yaml")],
        # rf2o logs "Waiting for laser_scans" on every loop without a new scan (10-40 per second
        # on /rosout and in the journal); robot_health reports a real laser odometry outage.
        ros_arguments=["--log-level", "rf2o_laser_odometry:=error"],
        output="screen", **RESPAWN,
    )
    imu_odom = Node(
        package="robot_odometry", executable="imu_odom_node", name="imu_odom_node",
        output="screen", **RESPAWN,
    )

    foxglove_bridge = Node(
        package="foxglove_bridge", executable="foxglove_bridge", name="foxglove_bridge",
        parameters=[{
            "port": foxglove_port,
            "address": "0.0.0.0",
            # No "parameters"/"parametersSubscribe"/"connectionGraph": the bridge polled every
            # node's parameters and the ROS graph on its main thread (with timeouts on nodes that
            # do not answer), which saturated it; a slow bridge then makes every reliable
            # publisher it subscribes to (TF, /odom, /rosout) burn CPU on resends.
            "capabilities": ["clientPublish", "services", "assets"],
        }],
        # Its only warnings are "Limiting history depth for topic ..." on every viewer
        # subscription (it sums the queue depths of all publishers); they flooded Foxglove's log.
        ros_arguments=["--log-level", "foxglove_bridge:=error"],
        output="screen", **RESPAWN,
    )

    health = Node(
        package="robot_supervisor", executable="health_node", name="robot_health",
        output="screen", **RESPAWN,
    )
    mapping_manager = Node(
        package="robot_supervisor", executable="mapping_manager.py", name="mapping_manager",
        parameters=[{"slam_params": os.path.join(bringup_share, "config", "slam_toolbox_params.yaml")}],
        output="screen", sigterm_timeout="15", **RESPAWN,
    )
    # Stereo depth on the GPU; idle (no camera subscriptions) until something subscribes to its outputs
    stereo_depth = Node(
        package="stereo_vision", executable="stereo_depth_node", name="stereo_depth",
        output="screen", **RESPAWN,
    )

    # Object detection (YOLOX on the DLA); idle until something subscribes to its outputs. Needs its
    # TensorRT engine: scripts/fetch_detector_models.sh (reported on /diagnostics if missing).
    object_detector = Node(
        package="object_detection", executable="object_detector_node", name="object_detector",
        output="screen", **RESPAWN,
    )

    # Switches depth / detection on and off and picks the detection model (saved across restarts)
    vision_manager = Node(
        package="robot_supervisor", executable="vision_manager.py", name="vision_manager",
        output="screen", **RESPAWN,
    )

    return LaunchDescription([
        DeclareLaunchArgument("foxglove_port", default_value="8765", description="foxglove_bridge port"),
        sensors,
        scan_resampler,
        rf2o,
        imu_odom,
        foxglove_bridge,
        health,
        mapping_manager,
        stereo_depth,
        object_detector,
        vision_manager,
    ])

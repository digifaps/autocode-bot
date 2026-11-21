"""Launch LiDAR + IMU for basic 2D mapping with slam_toolbox.

Brings up:
- Robot state publisher (URDF TF: base_footprint -> base_link -> ...)
- D500 LiDAR (/scan, base_link -> base_laser)
- IMU node (/imu/data_raw)
- Laser odometry: scan_resampler (/scan_fixed) + rf2o_laser_odometry (/odom_rf2o)
- IMU odom node (odom -> base_footprint and /odom: IMU heading + rf2o forward speed)
- slam_toolbox (map, map -> odom)

Requires: sudo apt install ros-humble-slam-toolbox
TF chain: map -> odom (slam_toolbox) -> base_footprint (imu_odom_node) -> base_link (URDF) -> base_laser (lidar static).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
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
    use_laser_odom = LaunchConfiguration("use_laser_odom", default="true")

    robot_description = _load_robot_description()

    # Path to our slam_toolbox params
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
        package="imu_driver",
        executable="imu_node",
        name="imu_node",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"i2c_bus": i2c_bus},
            {"i2c_addr": i2c_addr},
        ],
        output="screen",
    )

    # Laser odometry: /scan -> fixed-size /scan_fixed -> rf2o -> /odom_rf2o (forward speed)
    scan_resampler = Node(
        package="robot_odometry",
        executable="scan_resampler",
        name="scan_resampler",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
        condition=IfCondition(use_laser_odom),
    )

    rf2o = Node(
        package="rf2o_laser_odometry",
        executable="rf2o_laser_odometry_node",
        name="rf2o_laser_odometry",
        parameters=[
            os.path.join(get_package_share_directory("robot_bringup"), "config", "rf2o_params.yaml"),
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
        condition=IfCondition(use_laser_odom),
    )

    # odom -> base_footprint and /odom: IMU heading + rf2o forward speed
    imu_odom_node = Node(
        package="robot_odometry",
        executable="imu_odom_node",
        name="imu_odom_node",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"laser_odom_topic": PythonExpression(["'odom_rf2o' if '", use_laser_odom, "' == 'true' else ''"])},
        ],
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

    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource

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
        DeclareLaunchArgument("use_laser_odom", default_value="true", description="Laser odometry (rf2o) for translation; false = IMU heading only"),
        robot_state_publisher,
        imu_node,
        scan_resampler,
        rf2o,
        imu_odom_node,
        d500_include,
        slam_toolbox_node,
    ])

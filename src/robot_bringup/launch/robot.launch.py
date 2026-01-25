"""Launch file for bringing up the complete robot system."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for the robot."""

    # Launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            PathJoinSubstitution([
                FindPackageShare("robot_description"),
                "urdf",
                "robot.urdf.xacro"
            ])
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

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation clock if true",
        ),
        robot_state_publisher,
        motor_driver,
        stereo_camera,
        depth_processor,
    ])

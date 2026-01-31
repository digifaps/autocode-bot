"""Motor driver node for differential drive control.

Subscribes to /cmd_vel and controls the RioRand motor controllers via GPIO.
Publishes odometry based on motor speed feedback.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class MotorDriverNode(Node):
    """ROS2 node for differential drive motor control."""

    def __init__(self) -> None:
        super().__init__("motor_driver_node")

        # Parameters
        self.declare_parameter("wheel_separation", 0.5)  # meters
        self.declare_parameter("wheel_radius", 0.0825)  # meters (6.5" wheel)
        self.declare_parameter("max_linear_velocity", 1.0)  # m/s
        self.declare_parameter("max_angular_velocity", 2.0)  # rad/s

        self.wheel_separation = self.get_parameter("wheel_separation").value
        self.wheel_radius = self.get_parameter("wheel_radius").value

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, "cmd_vel", self.cmd_vel_callback, 10
        )

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)

        # Timer for odometry publishing
        self.create_timer(0.02, self.publish_odometry)  # 50 Hz

        self.get_logger().info("Motor driver node initialized")

    def cmd_vel_callback(self, msg: Twist) -> None:
        """Handle velocity commands and convert to wheel velocities."""
        linear = msg.linear.x
        angular = msg.angular.z

        # Differential drive kinematics
        left_vel = linear - (angular * self.wheel_separation / 2.0)
        right_vel = linear + (angular * self.wheel_separation / 2.0)

        # TODO: Send PWM signals to motor controllers
        self.get_logger().debug(f"Wheel velocities: L={left_vel:.2f}, R={right_vel:.2f}")

    def publish_odometry(self) -> None:
        """Publish odometry based on wheel encoder feedback."""
        # TODO: Read speed pulses from motor controllers
        # TODO: Calculate and publish odometry
        pass


def main(args=None) -> None:
    """Entry point for the motor driver node."""
    rclpy.init(args=args)
    node = MotorDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

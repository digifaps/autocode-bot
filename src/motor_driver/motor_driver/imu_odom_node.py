#!/usr/bin/env python3
"""
Odometry from IMU yaw + laser odometry translation.

Publishes odom -> base_footprint (TF) and nav_msgs/Odometry on /odom:
- Heading: integrated IMU yaw rate. The gyro vector is projected onto the measured gravity
  direction, so this works for any IMU mounting (on this robot the stereo board stands
  upright and tilted, so gravity is mostly on the IMU's y axis, not z).
- Position: forward speed from rf2o_laser_odometry (/odom_rf2o twist.linear.x), integrated
  along the IMU heading. Without laser odometry the position stays at the origin.

base_footprint, not base_link: robot_state_publisher already gives base_link its parent.
TF chain: map -> odom (slam_toolbox) -> base_footprint (this node) -> base_link (URDF) -> base_laser.
"""

import math
import signal

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster

G = 9.80665


def _stamp_s(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class ImuOdomNode(Node):
    def __init__(self):
        super().__init__("imu_odom_node")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("laser_odom_topic", "odom_rf2o")  # "" = IMU heading only
        self.declare_parameter("gravity_samples", 100)           # ~1 s at 100 Hz, robot still
        # rf2o occasionally mis-matches a scan (seen on the robot: 1.96 m/s while standing still).
        # Readings beyond what the robot can physically do are dropped.
        self.declare_parameter("max_laser_speed", 1.0)           # m/s
        self.declare_parameter("max_laser_accel", 3.0)           # m/s^2
        # rf2o reads ~0.003 m/s (up to a few cm/s) while standing still; treat that as zero
        self.declare_parameter("laser_speed_deadband", 0.02)     # m/s (p99 while still: ~0.026)

        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.gravity_samples = int(self.get_parameter("gravity_samples").value)
        self.max_speed = float(self.get_parameter("max_laser_speed").value)
        self.max_accel = float(self.get_parameter("max_laser_accel").value)
        self.deadband = float(self.get_parameter("laser_speed_deadband").value)

        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._v = 0.0
        self._yaw_rate = 0.0
        self._up = None                # unit gravity-up vector in the IMU frame
        self._acc_sum = [0.0, 0.0, 0.0]
        self._acc_n = 0
        self._last_imu_t = None
        self._last_laser_t = None
        self._laser_msgs = 0
        self._laser_rejected = 0
        self._pending = []             # consecutive gated speeds

        self._tf = TransformBroadcaster(self)
        self._odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.create_subscription(Imu, "imu/data_raw", self._imu_cb, 50)
        laser_topic = self.get_parameter("laser_odom_topic").value
        if laser_topic:
            self.create_subscription(Odometry, laser_topic, self._laser_cb, 10)
        self.create_timer(1.0 / float(self.get_parameter("publish_rate").value), self._publish)
        self.create_timer(30.0, self._status)
        self.get_logger().info(
            "imu_odom_node: %s -> %s, heading from IMU (gravity-projected), position from %s"
            % (self.odom_frame, self.base_frame, laser_topic or "nothing (fixed at origin)"))

    def _imu_cb(self, msg):
        a = msg.linear_acceleration
        if self._up is None:
            # Gravity direction from the first samples (robot still at startup, as for gyro bias)
            self._acc_sum[0] += a.x
            self._acc_sum[1] += a.y
            self._acc_sum[2] += a.z
            self._acc_n += 1
            if self._acc_n >= self.gravity_samples:
                n = math.sqrt(sum(c * c for c in self._acc_sum))
                if n > 0:
                    self._up = [c / n for c in self._acc_sum]
                    self.get_logger().info(
                        "gravity in IMU frame: (%.2f, %.2f, %.2f); yaw rate = gyro . up"
                        % tuple(self._up))
            self._last_imu_t = _stamp_s(msg.header.stamp)
            return
        # Slowly track the gravity direction while the robot is not accelerating
        norm = math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z)
        if abs(norm - G) < 0.3:
            k = 0.002
            up = [(1 - k) * u + k * c / norm for u, c in zip(self._up, (a.x, a.y, a.z))]
            n = math.sqrt(sum(c * c for c in up))
            self._up = [c / n for c in up]
        w = msg.angular_velocity
        rate = w.x * self._up[0] + w.y * self._up[1] + w.z * self._up[2]
        t = _stamp_s(msg.header.stamp)
        if self._last_imu_t is not None:
            dt = t - self._last_imu_t
            if 0.0 < dt < 0.5:
                self._yaw = math.atan2(math.sin(self._yaw + rate * dt), math.cos(self._yaw + rate * dt))
        self._last_imu_t = t
        self._yaw_rate = rate

    def _laser_cb(self, msg):
        t = _stamp_s(msg.header.stamp)
        v = msg.twist.twist.linear.x
        self._laser_msgs += 1
        dt = t - self._last_laser_t if self._last_laser_t is not None else 0.0
        self._last_laser_t = t
        if not 0.0 < dt < 1.0:
            return
        # Gate against the last accepted speed: a real ramp passes, a scan-match spike does not.
        # A jump that persists (3 consistent readings, 0.3 s) is real motion and is accepted.
        if not math.isfinite(v) or abs(v) > self.max_speed:
            self._laser_rejected += 1
            self._pending = []
            v = self._v * 0.5
        elif abs(v - self._v) > self.max_accel * dt + 0.05:
            self._pending.append(v)
            if len(self._pending) >= 3 and max(self._pending[-3:]) - min(self._pending[-3:]) < 0.1:
                self._pending = []
            else:
                self._laser_rejected += 1
                v = self._v * 0.5  # hold a decaying estimate through the glitch
        else:
            self._pending = []
        self._v = v
        if abs(v) < self.deadband:
            v = 0.0
        self._x += v * math.cos(self._yaw) * dt
        self._y += v * math.sin(self._yaw) * dt

    def _publish(self):
        now = self.get_clock().now().to_msg()
        qz, qw = math.sin(self._yaw * 0.5), math.cos(self._yaw * 0.5)
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self._x
            t.transform.translation.y = self._y
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self._tf.sendTransform(t)
        o = Odometry()
        o.header.stamp = now
        o.header.frame_id = self.odom_frame
        o.child_frame_id = self.base_frame
        o.pose.pose.position.x = self._x
        o.pose.pose.position.y = self._y
        o.pose.pose.orientation.z = qz
        o.pose.pose.orientation.w = qw
        o.twist.twist.linear.x = self._v
        o.twist.twist.angular.z = self._yaw_rate
        # Rough, diagonal: x, y, z, roll, pitch, yaw
        o.pose.covariance[0] = o.pose.covariance[7] = 0.01
        o.pose.covariance[14] = o.pose.covariance[21] = o.pose.covariance[28] = 1e6
        o.pose.covariance[35] = 0.02
        o.twist.covariance[0] = 0.01
        o.twist.covariance[7] = o.twist.covariance[14] = 1e6
        o.twist.covariance[21] = o.twist.covariance[28] = 1e6
        o.twist.covariance[35] = 0.005
        self._odom_pub.publish(o)

    def _status(self):
        self.get_logger().info(
            "odom x=%.2f y=%.2f yaw=%.1f deg; laser odom msgs %d (%d rejected) in 30 s"
            % (self._x, self._y, math.degrees(self._yaw), self._laser_msgs, self._laser_rejected))
        self._laser_msgs = 0
        self._laser_rejected = 0


def _sigint_once(signum, frame):
    # launch sends SIGINT twice on Ctrl+C; ignore the second one so cleanup is not interrupted
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    raise KeyboardInterrupt


def main(args=None):
    signal.signal(signal.SIGINT, _sigint_once)  # rclpy chains to this handler
    rclpy.init(args=args)
    node = None
    try:
        node = ImuOdomNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

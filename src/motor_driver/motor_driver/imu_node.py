#!/usr/bin/env python3
"""
ICM20948 IMU ROS2 Node

Reads accelerometer, gyroscope, and magnetometer data from the ICM20948
on the Waveshare IMX219-83 stereo camera board and publishes as ROS2 IMU messages.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
import math

try:
    from smbus2 import SMBus
    from icm20948 import ICM20948
except ImportError as e:
    print(f"ERROR: {e}. Install with: pip3 install icm20948 smbus2")
    raise


class ImuNode(Node):
    def __init__(self):
        super().__init__("imu_node")
        
        # Parameters
        self.declare_parameter("i2c_bus", 4)  # ICM20948 on Waveshare stereo board: bus 4 @ 0x68 on Jetson Xavier NX
        self.declare_parameter("i2c_addr", 0x68)
        self.declare_parameter("publish_rate", 100.0)
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("lowpass_alpha", 0.08)  # 0=no filter, 1=raw; smaller=more smoothing (0.05--0.15 typical)
        
        i2c_bus = self.get_parameter("i2c_bus").value
        i2c_addr = self.get_parameter("i2c_addr").value
        self.publish_rate = self.get_parameter("publish_rate").value
        self.frame_id = self.get_parameter("frame_id").value
        self.lowpass_alpha = self.get_parameter("lowpass_alpha").value
        self._filtered = None  # (ax, ay, az, gx, gy, gz) for exponential moving average
        
        # Initialize IMU
        try:
            self.bus = SMBus(i2c_bus)
            self.imu = ICM20948(i2c_addr=i2c_addr, i2c_bus=self.bus)
            self.get_logger().info(f"ICM20948 initialized on bus {i2c_bus}, addr 0x{i2c_addr:02x}")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize ICM20948: {e}")
            raise
        
        # Publishers
        self.imu_pub = self.create_publisher(Imu, "imu/data_raw", 10)
        self.mag_pub = self.create_publisher(MagneticField, "imu/mag", 10)
        
        # Timer
        period = 1.0 / self.publish_rate
        self.timer = self.create_timer(period, self.publish_imu)
        
        self.get_logger().info(f"IMU node started at {self.publish_rate} Hz")
    
    def publish_imu(self):
        try:
            # Read accelerometer and gyroscope
            ax, ay, az, gx, gy, gz = self.imu.read_accelerometer_gyro_data()
            
            # Optional low-pass filter to reduce oscillation/noise
            a = self.lowpass_alpha
            if self._filtered is None:
                self._filtered = (ax, ay, az, gx, gy, gz)
            else:
                fax, fay, faz, fgx, fgy, fgz = self._filtered
                self._filtered = (
                    a * ax + (1 - a) * fax,
                    a * ay + (1 - a) * fay,
                    a * az + (1 - a) * faz,
                    a * gx + (1 - a) * fgx,
                    a * gy + (1 - a) * fgy,
                    a * gz + (1 - a) * fgz,
                )
            ax, ay, az, gx, gy, gz = self._filtered
            
            # Read magnetometer
            mx, my, mz = self.imu.read_magnetometer_data()
            
            stamp = self.get_clock().now().to_msg()
            
            # Publish IMU message
            imu_msg = Imu()
            imu_msg.header.stamp = stamp
            imu_msg.header.frame_id = self.frame_id
            
            # Convert from g to m/s^2
            imu_msg.linear_acceleration.x = ax * 9.80665
            imu_msg.linear_acceleration.y = ay * 9.80665
            imu_msg.linear_acceleration.z = az * 9.80665
            
            # Convert from deg/s to rad/s
            imu_msg.angular_velocity.x = math.radians(gx)
            imu_msg.angular_velocity.y = math.radians(gy)
            imu_msg.angular_velocity.z = math.radians(gz)
            
            # No orientation estimate (set covariance to -1)
            imu_msg.orientation_covariance[0] = -1.0
            
            self.imu_pub.publish(imu_msg)
            
            # Publish magnetometer message
            mag_msg = MagneticField()
            mag_msg.header.stamp = stamp
            mag_msg.header.frame_id = self.frame_id
            
            # Convert from uT to Tesla
            mag_msg.magnetic_field.x = mx * 1e-6
            mag_msg.magnetic_field.y = my * 1e-6
            mag_msg.magnetic_field.z = mz * 1e-6
            
            self.mag_pub.publish(mag_msg)
            
        except Exception as e:
            self.get_logger().warn(f"IMU read error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
ICM20948 IMU ROS2 Node

Reads accelerometer, gyroscope, and magnetometer data from the ICM20948
on the Waveshare IMX219-83 stereo camera board and publishes as ROS2 IMU messages.
"""

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField

try:
    from smbus2 import SMBus
    from icm20948 import ICM20948
except ImportError as e:
    print(f"ERROR: {e}. Install with: pip3 install icm20948 smbus2")
    raise

ICM20948_WHO_AM_I = 0x00  # Bank 0
ICM20948_BANK_SEL = 0x7F
ICM20948_CHIP_ID = 0xEA


def _read_who_am_i(bus: SMBus, addr: int) -> int:
    """Read ICM20948 WHO_AM_I (bank 0, reg 0). Returns the byte or -1 on error."""
    try:
        bus.write_byte_data(addr, ICM20948_BANK_SEL, 0 << 4)  # select bank 0
        return bus.read_byte_data(addr, ICM20948_WHO_AM_I)
    except (OSError, IOError):
        return -1


class ImuNode(Node):
    def __init__(self):
        super().__init__("imu_node")
        
        # Parameters
        self.declare_parameter("i2c_bus", 8)  # ICM20948 on this board: bus 8 @ 0x68 (use i2cdetect -y -r 8 to verify)
        self.declare_parameter("i2c_addr", 0x68)
        self.declare_parameter("publish_rate", 100.0)
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("lowpass_alpha", 0.08)  # 0=no filter, 1=raw; smaller=more smoothing (0.05--0.15 typical)
        self.declare_parameter("log_interval", 100)  # log IMU sample every N messages (0=off); at 100 Hz, 100 = once per second
        
        i2c_bus = int(self.get_parameter("i2c_bus").value)
        i2c_addr = int(self.get_parameter("i2c_addr").value)
        self.publish_rate = self.get_parameter("publish_rate").value
        self.frame_id = self.get_parameter("frame_id").value
        self.lowpass_alpha = self.get_parameter("lowpass_alpha").value
        self.log_interval = int(self.get_parameter("log_interval").value)
        self._filtered = None  # (ax, ay, az, gx, gy, gz) for exponential moving average
        self._publish_count = 0

        last_error = None
        for force in (True, False):
            bus = None
            try:
                bus = SMBus(i2c_bus, force=force)
                who = _read_who_am_i(bus, i2c_addr)
                addr_used = i2c_addr
                if who != ICM20948_CHIP_ID:
                    who_alt = _read_who_am_i(bus, 0x69)
                    if who_alt == ICM20948_CHIP_ID:
                        addr_used = 0x69
                        who = who_alt
                    else:
                        if bus is not None:
                            try:
                                bus.close()
                            except Exception:
                                pass
                        w68 = f"0x{who:02x}" if who >= 0 else "read_error"
                        w69 = f"0x{who_alt:02x}" if who_alt >= 0 else "read_error"
                        last_error = RuntimeError(
                            f"ICM20948 WHO_AM_I: 0x68={w68}, 0x69={w69} (expected 0xEA). "
                            "Check I2C bus/address (see docs/hardware/imu_i2c.md)."
                        )
                        continue
                self.imu = ICM20948(i2c_addr=addr_used, i2c_bus=bus)
                self.bus = bus
                self.get_logger().info(
                    f"ICM20948 initialized on bus {i2c_bus}, addr 0x{addr_used:02x} (force={force})"
                )
                break
            except OSError as e:
                last_error = e
                if bus is not None:
                    try:
                        bus.close()
                    except Exception:
                        pass
                continue
            except RuntimeError as e:
                if "Unable to find ICM20948" in str(e) or "Unable to find AK09916" in str(e):
                    last_error = e
                    if bus is not None:
                        try:
                            bus.close()
                        except Exception:
                            pass
                    continue
                raise
        else:
            self.get_logger().error(f"Failed to initialize ICM20948: {last_error}")
            raise RuntimeError(f"ICM20948 init failed: {last_error}") from last_error
        
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
            self._publish_count += 1
            
            # Periodic log: sample of IMU data (accel m/s^2, gyro deg/s, mag uT)
            if self.log_interval > 0 and self._publish_count % self.log_interval == 0:
                self.get_logger().info(
                    "[IMU] accel=(%.2f, %.2f, %.2f) m/s^2 | gyro=(%.1f, %.1f, %.1f) deg/s | mag=(%.0f, %.0f, %.0f) uT | count=%d"
                    % (ax * 9.80665, ay * 9.80665, az * 9.80665, gx, gy, gz, mx, my, mz, self._publish_count)
                )
            
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

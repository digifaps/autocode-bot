#!/usr/bin/env python3
"""
ICM20948 IMU ROS2 Node

Reads accelerometer, gyroscope, and magnetometer data from the ICM20948
on the Waveshare IMX219-83 stereo camera board and publishes as ROS2 IMU messages.

The Pimoroni ``icm20948`` library is only used to reset and probe the chip. After that the
node configures the chip itself so each sample is a single I2C burst read:

- The AK09916 magnetometer runs in continuous 100 Hz mode and the ICM20948's internal I2C
  master copies its registers into EXT_SLV_SENS_DATA automatically. The library's
  ``read_magnetometer_data()`` instead triggers a single measurement and waits on several
  5 ms sleeps per call, which limited the node to ~20 Hz.
- Gyro noise is handled by the chip's hardware low-pass filter (``gyro_dlpf_mode``), not by a
  software moving average (which lagged yaw by ~0.5 s).
- The gyro bias is measured at startup (robot must be stationary) and subtracted.
"""

import math
import signal
import statistics
import struct
import time

import rclpy
from rclpy.executors import ExternalShutdownException
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

# Bank 0
REG_USER_CTRL = 0x03
USER_CTRL_I2C_MST_EN = 0x20
REG_ACCEL_XOUT_H = 0x2D  # accel(6) gyro(6) temp(2) then EXT_SLV_SENS_DATA_00 at 0x3B
# Bank 2
REG_GYRO_SMPLRT_DIV = 0x00
REG_GYRO_CONFIG_1 = 0x01
REG_ACCEL_SMPLRT_DIV_1 = 0x10
REG_ACCEL_SMPLRT_DIV_2 = 0x11
REG_ACCEL_CONFIG = 0x14
# Bank 3 (I2C master slave 0)
REG_I2C_SLV0_ADDR = 0x03
REG_I2C_SLV0_REG = 0x04
REG_I2C_SLV0_CTRL = 0x05
# AK09916 magnetometer (behind the ICM20948 I2C master)
AK09916_I2C_ADDR = 0x0C
AK09916_ST1 = 0x10  # ST1, HXL..HZH, TMPS, ST2 = 9 bytes; reading ST2 releases the data lock
AK09916_CNTL2 = 0x31
AK09916_MODE_CONT_100HZ = 0x08
AK09916_ST1_DRDY = 0x01
AK09916_ST2_HOFL = 0x08
AK09916_UT_PER_LSB = 0.15

BURST_LEN = 12 + 2 + 9  # accel+gyro, temp, magnetometer block
ICM_INTERNAL_RATE_HZ = 1125.0
ACCEL_LSB_PER_G = (16384.0, 8192.0, 4096.0, 2048.0)
GYRO_LSB_PER_DPS = (131.0, 65.5, 32.8, 16.4)
G = 9.80665


def _read_who_am_i(bus: SMBus, addr: int) -> int:
    """Read ICM20948 WHO_AM_I (bank 0, reg 0). Returns the byte or -1 on error."""
    try:
        bus.write_byte_data(addr, ICM20948_BANK_SEL, 0 << 4)  # select bank 0
        return bus.read_byte_data(addr, ICM20948_WHO_AM_I)
    except (OSError, IOError):
        return -1


class ImuNode(Node):
    """Publishes /imu/data_raw and /imu/mag from the ICM20948."""

    def __init__(self):
        super().__init__("imu_node")

        # Parameters
        self.declare_parameter("i2c_bus", 8)  # ICM20948 on this board: bus 8 @ 0x68 (use i2cdetect -y -r 8 to verify)
        self.declare_parameter("i2c_addr", 0x68)
        self.declare_parameter("publish_rate", 100.0)
        self.declare_parameter("frame_id", "imu_link")
        # Software EMA on the accelerometer only (0 < alpha <= 1; 1 = off). Gyro uses the hardware DLPF.
        self.declare_parameter("lowpass_alpha", 0.08)
        # ICM20948 gyro DLPF (GYRO_DLPFCFG): 0=197 Hz, 1=152, 2=120, 3=51, 4=24, 5=12, 6=6 Hz (3 dB bandwidth)
        self.declare_parameter("gyro_dlpf_mode", 4)
        # Seconds of stationary data averaged at startup to estimate gyro bias (0 = no calibration).
        # 5 s keeps residual yaw drift at ~1 deg/min on our board; 2 s left up to ~8 deg/min.
        self.declare_parameter("gyro_calibration_time", 5.0)
        # Calibration is rejected (robot moving) if gyro std dev exceeds this (deg/s; stationary noise ~0.45)
        self.declare_parameter("gyro_calibration_max_std", 2.0)
        # ...or if the estimated bias exceeds this (deg/s; ICM20948 spec is +-5 deg/s initial offset)
        self.declare_parameter("gyro_calibration_max_bias", 5.0)
        self.declare_parameter("log_interval", 100)  # log IMU sample every N messages (0=off); at 100 Hz, 100 = once per second

        self._bus_num = int(self.get_parameter("i2c_bus").value)
        self._addr_param = int(self.get_parameter("i2c_addr").value)
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.frame_id = self.get_parameter("frame_id").value
        self.lowpass_alpha = float(self.get_parameter("lowpass_alpha").value)
        self.gyro_dlpf_mode = int(self.get_parameter("gyro_dlpf_mode").value)
        self.log_interval = int(self.get_parameter("log_interval").value)
        if not 0.0 < self.lowpass_alpha <= 1.0:
            self.get_logger().warn("lowpass_alpha=%s out of range (0, 1]; using 1.0 (off)" % self.lowpass_alpha)
            self.lowpass_alpha = 1.0
        if not 0 <= self.gyro_dlpf_mode <= 7:
            raise ValueError("gyro_dlpf_mode must be 0..7")

        self.bus = None
        self.imu = None
        self.addr = None
        self._accel_scale = 1.0  # g per LSB
        self._gyro_scale = 1.0  # deg/s per LSB
        self._accel_filtered = None
        self._gyro_bias = (0.0, 0.0, 0.0)  # rad/s
        self._last_mag_raw = None
        self._publish_count = 0
        self._mag_count = 0
        self._consecutive_errors = 0
        self._next_reinit = 0.0
        self._last_mag_time = 0.0

        self._init_device()
        self._calibrate_gyro(float(self.get_parameter("gyro_calibration_time").value),
                             float(self.get_parameter("gyro_calibration_max_std").value),
                             float(self.get_parameter("gyro_calibration_max_bias").value))

        # Publishers
        self.imu_pub = self.create_publisher(Imu, "imu/data_raw", 10)
        self.mag_pub = self.create_publisher(MagneticField, "imu/mag", 10)

        # Timer
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_imu)

        self.get_logger().info(
            f"IMU node started at {self.publish_rate} Hz (burst read, mag continuous 100 Hz, "
            f"gyro DLPF mode {self.gyro_dlpf_mode})"
        )

    # ------------------------------------------------------------------ device setup
    def _open_chip(self):
        """Find the ICM20948 (0x68/0x69, force on/off) and reset it with the Pimoroni library."""
        last_error = None
        for force in (True, False):
            bus = None
            try:
                bus = SMBus(self._bus_num, force=force)
                who = _read_who_am_i(bus, self._addr_param)
                addr_used = self._addr_param
                if who != ICM20948_CHIP_ID:
                    who_alt = _read_who_am_i(bus, 0x69)
                    if who_alt == ICM20948_CHIP_ID:
                        addr_used = 0x69
                    else:
                        w68 = f"0x{who:02x}" if who >= 0 else "read_error"
                        w69 = f"0x{who_alt:02x}" if who_alt >= 0 else "read_error"
                        raise RuntimeError(
                            f"ICM20948 WHO_AM_I: 0x68={w68}, 0x69={w69} (expected 0xEA). "
                            "Check I2C bus/address (see docs/hardware/imu_i2c.md)."
                        )
                imu = ICM20948(i2c_addr=addr_used, i2c_bus=bus)  # resets chip, probes AK09916
                return bus, imu, addr_used, force
            except (OSError, RuntimeError) as e:
                last_error = e
                if bus is not None:
                    try:
                        bus.close()
                    except Exception:
                        pass
        raise RuntimeError(f"ICM20948 init failed: {last_error}") from last_error

    def _configure(self):
        """Sample rates, gyro DLPF, magnetometer continuous mode + automatic I2C-master read."""
        imu = self.imu
        # Sensor ODR above the publish rate so every timer tick sees a fresh sample.
        odr = min(ICM_INTERNAL_RATE_HZ, max(2.0 * self.publish_rate, 100.0))
        div = max(0, int(round(ICM_INTERNAL_RATE_HZ / odr)) - 1)
        imu.bank(2)
        imu.write(REG_GYRO_SMPLRT_DIV, div & 0xFF)
        imu.write(REG_ACCEL_SMPLRT_DIV_1, (div >> 8) & 0x0F)
        imu.write(REG_ACCEL_SMPLRT_DIV_2, div & 0xFF)
        imu.set_gyro_low_pass(enabled=True, mode=self.gyro_dlpf_mode)
        accel_cfg = imu.read(REG_ACCEL_CONFIG)
        gyro_cfg = imu.read(REG_GYRO_CONFIG_1)
        self._accel_scale = 1.0 / ACCEL_LSB_PER_G[(accel_cfg >> 1) & 0x03]
        self._gyro_scale = 1.0 / GYRO_LSB_PER_DPS[(gyro_cfg >> 1) & 0x03]

        # Magnetometer: continuous 100 Hz (one-off write through the library), verify.
        imu.mag_write(AK09916_CNTL2, AK09916_MODE_CONT_100HZ)
        mode = imu.mag_read(AK09916_CNTL2)
        if mode != AK09916_MODE_CONT_100HZ:
            raise RuntimeError(f"AK09916 CNTL2 readback 0x{mode:02x}, expected 0x08")

        # Slave 0: read ST1..ST2 (9 bytes) from the AK09916 every sample, into EXT_SLV_SENS_DATA_00.
        imu.bank(3)
        imu.write(REG_I2C_SLV0_ADDR, AK09916_I2C_ADDR | 0x80)
        imu.write(REG_I2C_SLV0_REG, AK09916_ST1)
        imu.write(REG_I2C_SLV0_CTRL, 0x80 | 9)
        imu.bank(0)
        user = imu.read(REG_USER_CTRL)
        imu.write(REG_USER_CTRL, user | USER_CTRL_I2C_MST_EN)  # keep the I2C master running
        time.sleep(0.05)  # let the first magnetometer sample arrive
        return odr

    def _init_device(self):
        self._close_bus()
        self.bus, self.imu, self.addr, force = self._open_chip()
        odr = self._configure()
        self._accel_filtered = None
        self._last_mag_raw = None
        self._last_mag_time = time.monotonic()
        self.get_logger().info(
            f"ICM20948 initialized on bus {self._bus_num}, addr 0x{self.addr:02x} (force={force}), "
            f"ODR {odr:.0f} Hz, accel {self._accel_scale * 1000:.3f} mg/LSB, "
            f"gyro {self._gyro_scale * 1000:.2f} mdps/LSB"
        )

    def _close_bus(self):
        if self.bus is not None:
            try:
                self.bus.close()
            except Exception:
                pass
        self.bus = None
        self.imu = None

    # ------------------------------------------------------------------ reading
    def _read_burst(self):
        """One I2C transaction (bank 0 stays selected): returns g, deg/s, and raw magnetometer info."""
        data = bytes(self.bus.read_i2c_block_data(self.addr, REG_ACCEL_XOUT_H, BURST_LEN))
        ax, ay, az, gx, gy, gz = struct.unpack_from(">6h", data, 0)
        st1 = data[14]
        mx, my, mz = struct.unpack_from("<3h", data, 15)
        st2 = data[22]
        a = self._accel_scale
        g = self._gyro_scale
        return (ax * a, ay * a, az * a), (gx * g, gy * g, gz * g), (st1, mx, my, mz, st2)

    def _calibrate_gyro(self, seconds, max_std_dps, max_bias_dps):
        if seconds <= 0:
            self.get_logger().info("Gyro bias calibration disabled (gyro_calibration_time=0)")
            return
        period = 1.0 / self.publish_rate
        samples = []
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            try:
                samples.append(self._read_burst()[1])
            except OSError:
                pass
            time.sleep(period)
        if len(samples) < 10:
            self.get_logger().warn("Gyro calibration: too few samples (%d); bias not applied" % len(samples))
            return
        axes = list(zip(*samples))
        mean = [statistics.fmean(v) for v in axes]
        std = [statistics.pstdev(v) for v in axes]
        if max(std) > max_std_dps or max(abs(m) for m in mean) > max_bias_dps:
            self.get_logger().warn(
                "Gyro calibration: robot moving (mean %.2f/%.2f/%.2f, std %.2f/%.2f/%.2f deg/s; limits "
                "|mean| %.1f, std %.1f); bias NOT applied. Restart imu_node while stationary."
                % (mean[0], mean[1], mean[2], std[0], std[1], std[2], max_bias_dps, max_std_dps)
            )
            return
        self._gyro_bias = tuple(math.radians(m) for m in mean)
        self.get_logger().info(
            "Gyro bias (deg/s): x=%.3f y=%.3f z=%.3f (std %.3f/%.3f/%.3f, %d samples)"
            % (mean[0], mean[1], mean[2], std[0], std[1], std[2], len(samples))
        )

    def _reinit(self, reason):
        """Re-initialize the chip (at most every 5 s; retry after 1 s if it fails). Keeps the gyro bias."""
        now = time.monotonic()
        if now < self._next_reinit:
            return
        self._next_reinit = now + 5.0
        self.get_logger().warn(f"Re-initializing ICM20948: {reason}")
        try:
            self._init_device()
            self._consecutive_errors = 0
        except Exception as e:  # keep the node alive; retry soon (bus may be briefly busy)
            self._next_reinit = time.monotonic() + 1.0
            self.get_logger().error(f"IMU re-init failed (retrying in 1 s): {e}")

    def _check_health(self):
        """Detect a chip that was reset or reconfigured behind our back (reads still succeed then)."""
        try:
            bank = self.bus.read_byte_data(self.addr, ICM20948_BANK_SEL)
            user = self.bus.read_byte_data(self.addr, REG_USER_CTRL) if bank == 0 else 0
        except OSError:
            return  # read errors are handled by the error counter
        if bank != 0:
            self._reinit(f"register bank changed to {bank >> 4}")
        elif not user & USER_CTRL_I2C_MST_EN:
            self._reinit("I2C master disabled (chip was reset?)")
        elif time.monotonic() - self._last_mag_time > 1.0:
            self._reinit("no new magnetometer data for 1 s")

    def _handle_error(self, e):
        self._consecutive_errors += 1
        if self._consecutive_errors in (1, 10) or self._consecutive_errors % 100 == 0:
            self.get_logger().warn(f"IMU read error ({self._consecutive_errors} in a row): {e}")
        if self._consecutive_errors >= 20:
            self._reinit(f"{self._consecutive_errors} consecutive read errors")

    def publish_imu(self):
        if self.bus is None:
            self._handle_error(RuntimeError("I2C bus not open"))
            return
        try:
            (ax, ay, az), (gx, gy, gz), (st1, mx, my, mz, st2) = self._read_burst()
        except OSError as e:
            self._handle_error(e)
            return
        stamp = self.get_clock().now().to_msg()
        if self._consecutive_errors:
            self.get_logger().info(f"IMU reads recovered after {self._consecutive_errors} errors")
            self._consecutive_errors = 0

        # Optional EMA on accelerometer only
        a = self.lowpass_alpha
        if self._accel_filtered is None or a >= 1.0:
            self._accel_filtered = (ax, ay, az)
        else:
            fx, fy, fz = self._accel_filtered
            self._accel_filtered = (a * ax + (1 - a) * fx, a * ay + (1 - a) * fy, a * az + (1 - a) * fz)
        ax, ay, az = self._accel_filtered

        bx, by, bz = self._gyro_bias
        wx = math.radians(gx) - bx
        wy = math.radians(gy) - by
        wz = math.radians(gz) - bz

        imu_msg = Imu()
        imu_msg.header.stamp = stamp
        imu_msg.header.frame_id = self.frame_id
        imu_msg.linear_acceleration.x = ax * G
        imu_msg.linear_acceleration.y = ay * G
        imu_msg.linear_acceleration.z = az * G
        imu_msg.angular_velocity.x = wx
        imu_msg.angular_velocity.y = wy
        imu_msg.angular_velocity.z = wz
        imu_msg.orientation_covariance[0] = -1.0  # no orientation estimate
        self.imu_pub.publish(imu_msg)
        self._publish_count += 1

        # Magnetometer: publish only new, non-overflowed samples (~100 Hz from the AK09916)
        raw = (mx, my, mz)
        if (raw != (0, 0, 0) and not (st2 & AK09916_ST2_HOFL)
                and ((st1 & AK09916_ST1_DRDY) or raw != self._last_mag_raw)):
            self._last_mag_raw = raw
            self._last_mag_time = time.monotonic()
            mag_msg = MagneticField()
            mag_msg.header.stamp = stamp
            mag_msg.header.frame_id = self.frame_id
            mag_msg.magnetic_field.x = mx * AK09916_UT_PER_LSB * 1e-6
            mag_msg.magnetic_field.y = my * AK09916_UT_PER_LSB * 1e-6
            mag_msg.magnetic_field.z = mz * AK09916_UT_PER_LSB * 1e-6
            self.mag_pub.publish(mag_msg)
            self._mag_count += 1

        if self._publish_count % max(1, int(self.publish_rate)) == 0:  # ~1 Hz
            self._check_health()

        if self.log_interval > 0 and self._publish_count % self.log_interval == 0:
            m = self._last_mag_raw or (0, 0, 0)
            self.get_logger().info(
                "[IMU] accel=(%.2f, %.2f, %.2f) m/s^2 | gyro=(%.2f, %.2f, %.2f) deg/s | "
                "mag=(%.0f, %.0f, %.0f) uT | count=%d mag_count=%d"
                % (ax * G, ay * G, az * G, math.degrees(wx), math.degrees(wy), math.degrees(wz),
                   m[0] * AK09916_UT_PER_LSB, m[1] * AK09916_UT_PER_LSB, m[2] * AK09916_UT_PER_LSB,
                   self._publish_count, self._mag_count)
            )

    def destroy_node(self):
        self._close_bus()
        super().destroy_node()


def _sigint_once(signum, frame):
    """First Ctrl+C stops the node; ignore repeats (terminal process group + ros2 launch both send one)."""
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    raise KeyboardInterrupt


def main(args=None):
    signal.signal(signal.SIGINT, _sigint_once)  # rclpy chains to this handler
    rclpy.init(args=args)
    node = None
    try:
        node = ImuNode()  # inside try: Ctrl+C during gyro calibration is a normal exit
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
        # Shutdown signal arrived while a timer callback was publishing: not an error.
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)  # no second interrupt during cleanup
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

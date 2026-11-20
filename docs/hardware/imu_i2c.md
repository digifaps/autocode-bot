# IMU (ICM20948) on Jetson – I2C bus

The **imu_node** reads the ICM20948 (with AK09916 magnetometer) on the Waveshare IMX219-83 stereo camera board over I2C.

## Defaults

- **i2c_bus:** `8` (parameter in imu_node; detected on Jetson Xavier NX with stereo board)
- **i2c_addr:** `0x68` (ICM20948 default; the node also tries `0x69`)

So the node looks for the IMU on **bus 8** by default.

## Find the IMU on your Jetson

1. **List I2C buses**
   ```bash
   ls /dev/i2c-*
   ```
   You might see `/dev/i2c-0`, `/dev/i2c-1`, …, `/dev/i2c-8`, etc.

2. **Scan the bus the node uses (default 8)**
   ```bash
   sudo i2cdetect -y -r 8
   ```
   If the ICM20948 is on bus 8, you should see `68` or `69` (or `UU` while imu_node is running).

3. **If bus 8 has no device, scan other buses**
   ```bash
   for i in 0 1 2 7 8; do echo "=== Bus $i ==="; sudo i2cdetect -y $i 2>/dev/null | grep -E "68|69|0c"; done
   ```
   ICM20948 is usually at **0x68** or **0x69**; the magnetometer (AK09916) is often at **0x0c** (via the main chip).

4. **If the IMU is on another bus**, start the node with that bus, e.g. bus 1:
   ```bash
   ros2 run motor_driver imu_node --ros-args -p i2c_bus:=1
   ```
   Or in a launch file, pass the parameter so that `imu_node` uses the bus where you see 0x68/0x69.

I2C works as-is on the Jetson; no udev rules or unbind scripts are required.

## “Unable to find ICM20948” / “Unable to find AK09916”

- **i2cdetect -y 1** showed no devices → bus 1 is not the bus the IMU uses.
- Run **i2cdetect -y 8** (and other buses) to see which bus has 0x68/0x69.
- If no bus shows the IMU: check wiring and power of the stereo camera board; the IMU is on that board and shares its I2C connection (often via the same cable/connector as the cameras).

## Python dependencies (required for imu_node)

The ROS2 `imu_node` uses the Pimoroni `icm20948` library (Waveshare-compatible). Install on the Jetson:

```bash
pip3 install icm20948 smbus2
```

(Or use `pip3 install --user` if you prefer not to touch system Python.)

## Checking IMU up to the ROS module

1. **Start the IMU node** (standalone, to isolate from other nodes):
   ```bash
   source ~/ros2_ws/install/setup.bash
   ros2 run motor_driver imu_node
   ```
   You should see `ICM20948 initialized on bus 8, addr 0x68 ...`, a `Gyro bias (deg/s): ...` line and `IMU node started at 100.0 Hz`.  
   **Keep the robot still for the first 5 seconds**: the node averages the gyro to remove its bias (about 0.35 deg/s on the z axis on our board, which otherwise makes the IMU yaw drift by ~24 deg/min; with calibration it is ~0.3-1.6 deg/min). If it detects motion it logs a warning and publishes without bias correction.  
   If you see `Unable to find ICM20948` or `Unable to find AK09916`, use another I2C bus (see above) or check wiring.

2. **In another terminal, check that data is published:**
   ```bash
   source ~/ros2_ws/install/setup.bash
   ros2 topic list | grep imu
   ros2 topic echo /imu/data_raw --once
   ros2 topic echo /imu/mag --once
   ```
   You should see `sensor_msgs/msg/Imu` on `/imu/data_raw` (linear_acceleration in m/s², angular_velocity in rad/s) and `sensor_msgs/msg/MagneticField` on `/imu/mag` (magnetic_field in Tesla).

3. **When using the full robot launch**, the IMU node is started by `robot.launch.py`. You can override I2C bus/address via parameters, e.g.:
   ```bash
   ros2 launch robot_bringup robot.launch.py i2c_bus:=1
   ```
   The main robot launch file supports `i2c_bus` and `i2c_addr`; e.g. `ros2 launch robot_bringup robot.launch.py i2c_bus:=1`.

## How the node reads the sensor

- One 23-byte I2C burst per sample (accel, gyro, and the magnetometer block). The ICM20948's internal I2C master reads the AK09916 magnetometer (continuous 100 Hz mode) by itself, so `/imu/data_raw` runs at 100 Hz and `/imu/mag` at ~80-100 Hz. The Pimoroni library is only used to reset and probe the chip; its `read_magnetometer_data()` waits on several 5 ms sleeps per call and limited the node to ~20 Hz.
- Gyro noise is filtered by the chip's hardware low-pass filter (`gyro_dlpf_mode`, default 4 = 24 Hz bandwidth). `lowpass_alpha` (software moving average) now applies to the accelerometer only.
- The node re-initializes the chip (at most every 5 s, keeping the gyro bias) after 20 consecutive I2C errors, or when its once-per-second health check finds the chip was reset or reconfigured (register bank changed, I2C master disabled, or no new magnetometer data for 1 s). Previously a reset left the node failing or publishing stale data until restarted.
- Do not open the IMU from a second process (e.g. a test script calling `ICM20948()`) while imu_node runs: that resets the chip under the node. The node recovers by itself, but drops samples meanwhile.

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `i2c_bus` / `i2c_addr` | `8` / `0x68` | I2C bus and address |
| `publish_rate` | `100.0` | Hz; the chip samples at ~2x this |
| `gyro_dlpf_mode` | `4` | Gyro DLPF: 0=197, 1=152, 2=120, 3=51, 4=24, 5=12, 6=6 Hz |
| `gyro_calibration_time` | `5.0` | Seconds of startup gyro averaging (0 = off). Shorter windows are noticeably less accurate |
| `gyro_calibration_max_std` | `2.0` | deg/s; above this the robot is considered moving and no bias is applied (stationary noise is ~0.45) |
| `gyro_calibration_max_bias` | `5.0` | deg/s; a larger estimated bias is treated as motion (sensor spec is +-5) |
| `lowpass_alpha` | `0.08` | Accelerometer moving-average factor (1 = off) |
| `log_interval` | `100` | Log one sample every N messages (0 = off) |

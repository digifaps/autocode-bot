# IMU (ICM20948) on Jetson – I2C bus

The **imu_node** (C++, package `imu_driver`) reads the ICM20948 (with AK09916 magnetometer) on the Waveshare IMX219-83 stereo camera board over I2C (`/dev/i2c-8`, user in group `i2c`). It needs no Python libraries.

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
   ros2 run imu_driver imu_node --ros-args -p i2c_bus:=1
   ```
   Or in a launch file, pass the parameter so that `imu_node` uses the bus where you see 0x68/0x69.

I2C works as-is on the Jetson; no udev rules or unbind scripts are required.

## "ICM20948 not found" / "AK09916 magnetometer not found"

The node keeps running and retries every 2 s; Foxglove's Diagnostics panel shows the error under **IMU (ICM20948)**.

- Run `sudo i2cdetect -y -r 8` (and other buses) to see which bus has 0x68/0x69.
- If no bus shows the IMU: check wiring and power of the stereo camera board; the IMU is on that board and shares its connection with the cameras.

## Checking IMU up to the ROS module

1. **Start the IMU node** (standalone, to isolate from other nodes):
   ```bash
   source ~/ros2_ws/install/setup.bash
   sudo systemctl stop autocode-bot      # the boot service already runs imu_node
   ros2 run imu_driver imu_node
   ```
   You should see `ICM20948 initialized on /dev/i2c-8 addr 0x68 ...`, a `Gyro bias (deg/s): ...` line and `IMU running at 100 Hz`.  
   **Keep the robot still for the first 5 seconds**: the node averages the gyro to remove its bias (about 0.35 deg/s on the z axis on our board, which otherwise makes the IMU yaw drift by ~24 deg/min; with calibration it is ~0.3-1.6 deg/min). If it detects motion it logs a warning and publishes without bias correction.  
   If you see `ICM20948 not found`, use another I2C bus (see above) or check wiring.

2. **In another terminal, check that data is published:**
   ```bash
   source ~/ros2_ws/install/setup.bash
   ros2 topic list | grep imu
   ros2 topic echo /imu/data_raw --once
   ros2 topic echo /imu/mag --once
   ```
   You should see `sensor_msgs/msg/Imu` on `/imu/data_raw` (linear_acceleration in m/s², angular_velocity in rad/s) and `sensor_msgs/msg/MagneticField` on `/imu/mag` (magnetic_field in Tesla).

3. **In the full stack** the IMU node is started by `robot.launch.py` (included by `bringup.launch.py`, which runs at boot). `robot.launch.py` takes `i2c_bus` and `i2c_addr`, e.g. `ros2 launch robot_bringup robot.launch.py i2c_bus:=1`.

## How the node reads the sensor

- One 23-byte I2C burst per sample (accel, gyro, and the magnetometer block). The ICM20948's internal I2C master reads the AK09916 magnetometer (continuous 100 Hz mode) by itself, so `/imu/data_raw` runs at 100 Hz and `/imu/mag` at ~80-100 Hz. (The Pimoroni Python library's `read_magnetometer_data()` waited on several 5 ms sleeps per call and limited the original Python node to ~20 Hz.)
- Gyro noise is filtered by the chip's hardware low-pass filter (`gyro_dlpf_mode`, default 4 = 24 Hz bandwidth). The Pimoroni library wrote this setting 1 bit too high (`<< 4` instead of `<< 3` into GYRO_CONFIG_1), so the old Python node actually ran at mode 0 (197 Hz); the C++ driver sets it correctly, and stationary gyro noise dropped from ~0.45 to ~0.15-0.2 deg/s. `lowpass_alpha` (software moving average) applies to the accelerometer only (accelerometer: hardware DLPF mode 5, +-16 g).
- The node re-initializes the chip (at most every 5 s, keeping the gyro bias) after 20 consecutive I2C errors, or when its once-per-second health check finds the chip was reset or reconfigured (register bank changed, I2C master disabled, or no new magnetometer data for 1 s).
- If the chip is missing at startup the node does not exit: it retries every 2 s and reports ERROR on `/diagnostics`.
- Status on `/diagnostics` once per second (name `IMU (ICM20948)`): rate, magnetometer rate, gyro bias, read errors, re-inits.
- Do not open the IMU from a second process (e.g. a Python test script using the Pimoroni `ICM20948()` class) while imu_node runs: that resets the chip under the node. The node recovers by itself, but drops samples meanwhile.

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `i2c_bus` / `i2c_addr` | `8` / `0x68` | I2C bus and address |
| `publish_rate` | `100.0` | Hz; the chip samples at ~2x this |
| `gyro_dlpf_mode` | `4` | Gyro DLPF: 0=197, 1=152, 2=120, 3=51, 4=24, 5=12, 6=6 Hz |
| `gyro_calibration_time` | `5.0` | Seconds of startup gyro averaging (0 = off). Shorter windows are noticeably less accurate |
| `gyro_calibration_max_std` | `2.0` | deg/s; above this the robot is considered moving and no bias is applied (stationary noise is ~0.15-0.2) |
| `gyro_calibration_max_bias` | `5.0` | deg/s; a larger estimated bias is treated as motion (sensor spec is +-5) |
| `lowpass_alpha` | `0.08` | Accelerometer moving-average factor (1 = off) |
| `log_interval` | `0` | Log one sample every N messages (0 = off) |

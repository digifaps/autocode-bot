# IMU (ICM20948) on Jetson – I2C bus

The **imu_node** reads the ICM20948 (with AK09916 magnetometer) on the Waveshare IMX219-83 stereo camera board over I2C.

## Defaults

- **i2c_bus:** `4` (parameter in imu_node; detected on Jetson Xavier NX with stereo board)
- **i2c_addr:** `0x68` (ICM20948 default)

So the node looks for the IMU on **bus 4** by default.

## Find the IMU on your Jetson

1. **List I2C buses**
   ```bash
   ls /dev/i2c-*
   ```
   You might see `/dev/i2c-0`, `/dev/i2c-1`, …, `/dev/i2c-8`, etc.

2. **Scan the bus the node uses (default 4)**
   ```bash
   sudo i2cdetect -y 4
   ```
   If the ICM20948 is on bus 4, you should see `68` or `69` at 0x68 or 0x69.

3. **If bus 4 has no device, scan other buses**
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
   You should see: `ICM20948 initialized on bus 4, addr 0x68` and `IMU node started at 100.0 Hz`.  
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

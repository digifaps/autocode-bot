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

## “Unable to find ICM20948” / “Unable to find AK09916”

- **i2cdetect -y 1** showed no devices → bus 1 is not the bus the IMU uses.
- Run **i2cdetect -y 8** (and other buses) to see which bus has 0x68/0x69.
- If no bus shows the IMU: check wiring and power of the stereo camera board; the IMU is on that board and shares its I2C connection (often via the same cable/connector as the cameras).

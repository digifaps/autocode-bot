# Waveshare D500 LiDAR

The D500 is a 360° 2D LiDAR compatible with the LDROBOT LD19 protocol. It publishes `sensor_msgs/LaserScan` on the `/scan` topic for Nav2 and SLAM.

## Specifications

- **Range:** up to 12 m
- **Scan rate:** ~10 Hz (default; may be configurable if protocol supports it)
- **Interface:** UART (230400 baud)
- **ROS2 driver:** [ldlidar_stl_ros2](https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2) (LD19 profile)

## Wiring

### Option A: Jetson onboard UART (recommended for control)

Connect the D500 to a UART on the Jetson 40-pin GPIO header for direct control (no USB adapter).

| Jetson 40-pin (BOARD) | Signal   | D500 / adapter |
|-----------------------|----------|-----------------|
| Pin 8                 | UART1 TX | RX of D500      |
| Pin 10                | UART1 RX | TX of D500      |
| GND (e.g. Pin 6, 9)   | GND      | GND             |

- **Linux device:** `/dev/ttyTHS1` (UART1 on Jetson Xavier NX).
- **Levels:** Jetson GPIO is 3.3 V. If the D500 or your adapter expects 5 V TTL, use a level shifter (e.g. 3.3 V → 5 V) on TX/RX.
- **Power:** Supply 5 V to the D500 separately (see Power control below to switch it on/off from software).

### Option B: USB-to-UART adapter

1. Connect D500 to Jetson via USB-to-UART (e.g. CP2102, CH340).
2. Power the D500 (5 V).
3. After plugging in, check the device: `ls -l /dev/ttyUSB*` (often `/dev/ttyUSB0`).

## Setup on Jetson

### 1. Add driver and lidar_control (one-time)

```bash
cd ~/ros2_ws/src
git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git
# lidar_control is in autocode-bot repo
cd ~/ros2_ws
source ~/ros2_humble/install/setup.bash
colcon build --packages-select ldlidar_stl_ros2 lidar_control --symlink-install
source install/setup.bash
```

### 2. Serial port

USB (e.g. `/dev/ttyUSB0`) and onboard UART (`/dev/ttyTHS1`) work as-is; no udev rules or extra setup required.

### 3. Launch

```bash
source ~/ros2_humble/install/setup.bash
source ~/ros2_ws/install/setup.bash
```

**Onboard UART (default):**

```bash
ros2 launch robot_bringup d500_lidar.launch.py
# Uses port_name:=/dev/ttyTHS1 by default
```

**With power control (GPIO):** use a GPIO pin to switch LiDAR 5 V on/off (e.g. via MOSFET). Example with BOARD pin 12:

```bash
ros2 launch robot_bringup d500_lidar.launch.py use_lidar_power:=true power_enable_gpio_pin:=12
```

**USB adapter:** use the `use_usb` option (default port `/dev/ttyUSB0`) or override with `port_name`:

```bash
# Short form (uses /dev/ttyUSB0)
ros2 launch robot_bringup d500_lidar.launch.py use_usb:=true

# Or specify port explicitly
ros2 launch robot_bringup d500_lidar.launch.py port_name:=/dev/ttyUSB0
ros2 launch robot_bringup d500_lidar.launch.py port_name:=/dev/ttyUSB1
```

## Power control (switch on when needed)

Use the `lidar_control` package to turn LiDAR power on/off via a GPIO pin (e.g. driving a MOSFET or enable input on a 5 V regulator).

- **Service:** `lidar/set_power` (`std_srvs/SetBool`) — `data: true` = power ON, `data: false` = power OFF.
- **Parameters:** `power_enable_gpio_pin` (BOARD number, 0 = disabled), `initial_power_on` (bool), `scan_speed_hz` (desired scan rate; see below).

Example:

```bash
# Turn LiDAR on
ros2 service call /lidar/set_power std_srvs/srv/SetBool "{data: true}"

# Turn LiDAR off
ros2 service call /lidar/set_power std_srvs/srv/SetBool "{data: false}"
```

If you use `use_lidar_power:=true` in the launch file, the `lidar_control_node` is started and power is applied (or not) according to `initial_power_on` before the LiDAR driver starts.

## Scan speed

- **Default:** ~10 Hz. The `lidar_control_node` has a parameter `scan_speed_hz` (default 10.0) for desired scan rate.
- **Runtime change:** The LD19/D500 protocol may support motor/scan speed commands; the current `ldlidar_stl_ros2` driver does not expose this. To support runtime scan-speed control, the driver or a wrapper would need to send the appropriate serial command. For now, use `scan_speed_hz` as documentation or future integration; changing it does not alter the LiDAR until the driver supports it.

## Topics and TF

- **Topic:** `/scan` (`sensor_msgs/msg/LaserScan`)
- **TF:** `base_link` → `base_laser` (static, z = 0.18 m)

## Troubleshooting

| Issue | Check |
|-------|--------|
| No `/dev/ttyTHS1` | Onboard UART: check 40-pin wiring (8=TX, 10=RX); verify pinmux if needed |
| No `/dev/ttyUSB*` | USB cable and adapter; run `lsusb` and `dmesg \| tail` |
| Permission denied | Add user to `dialout`: `sudo usermod -aG dialout $USER`, then log out and back in |
| No data on `/scan` | Correct port in launch; baud 230400; LiDAR powered (use `lidar/set_power` if using GPIO) |
| Wrong port | List with `ls /dev/tty*` and use `port_name:=/dev/ttyXXX` |

## Use with Nav2 / SLAM

The `/scan` topic is the standard input for:

- **slam_toolbox** – 2D SLAM
- **Nav2** – costmaps and obstacle avoidance

Ensure `base_laser` (or your scan frame) is in the TF tree relative to `odom`/`map` when running navigation.

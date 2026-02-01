# Waveshare D500 LiDAR

The D500 is a 360° 2D LiDAR compatible with the LDROBOT LD19 protocol. It publishes `sensor_msgs/LaserScan` on the `/scan` topic for Nav2 and SLAM.

## Specifications

- **Range:** up to 12 m
- **Scan rate:** ~10 Hz
- **Interface:** UART (230400 baud), typically via USB-to-UART adapter
- **ROS2 driver:** [ldlidar_stl_ros2](https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2) (LD19 profile)

## Wiring

1. Connect D500 to Jetson via USB-to-UART (e.g. CP2102, CH340).
2. Power the D500 (5V).
3. After plugging in, check the device: `ls -l /dev/ttyUSB*` (often `/dev/ttyUSB0`).

## Setup on Jetson

### 1. Add driver to workspace (one-time)

```bash
cd ~/ros2_ws/src
git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git
cd ~/ros2_ws
source ~/ros2_humble/install/setup.bash
colcon build --packages-select ldlidar_stl_ros2 --symlink-install
source install/setup.bash
```

### 2. Serial port permissions

```bash
# Temporary (until unplug)
sudo chmod 666 /dev/ttyUSB0

# Persistent: add user to dialout
sudo usermod -aG dialout $USER
# Then log out and back in
```

Optional udev rule for a fixed name (e.g. `/dev/d500`):

```bash
# /etc/udev/rules.d/99-d500-lidar.rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="d500", MODE="0666"
```

Replace `idVendor`/`idProduct` with values from `lsusb` for your adapter. Then: `sudo udevadm control --reload-rules`.

### 3. Launch

```bash
source ~/ros2_humble/install/setup.bash
source ~/ros2_ws/install/setup.bash

# Default port /dev/ttyUSB0
ros2 launch robot_bringup d500_lidar.launch.py

# Custom port
ros2 launch robot_bringup d500_lidar.launch.py port_name:=/dev/ttyUSB1
```

## Topics and TF

- **Topic:** `/scan` (`sensor_msgs/msg/LaserScan`)
- **TF:** `base_link` → `base_laser` (static, z = 0.18 m)

## Troubleshooting

| Issue | Check |
|-------|--------|
| No `/dev/ttyUSB*` | USB cable and adapter; run `lsusb` and `dmesg \| tail` |
| Permission denied | `sudo chmod 666 /dev/ttyUSB0` or add user to `dialout` |
| No data on `/scan` | Correct port in launch; baud 230400; LiDAR powered |
| Wrong port | List with `ls /dev/tty*` and use `port_name:=/dev/ttyXXX` |

## Use with Nav2 / SLAM

The `/scan` topic is the standard input for:

- **slam_toolbox** – 2D SLAM
- **Nav2** – costmaps and obstacle avoidance

Ensure `base_laser` (or your scan frame) is in the TF tree relative to `odom`/`map` when running navigation.

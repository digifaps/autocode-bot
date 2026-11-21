# autocode-bot

An autonomous wheeled robot with stereo vision, built entirely with AI-assisted code generation.

## Overview

This project implements an autonomous differential-drive robot running ROS2 on NVIDIA Jetson Xavier NX. The robot uses stereo cameras for depth perception and visual SLAM, enabling autonomous navigation and obstacle avoidance.

**Key highlight:** All code in this repository is AI-generated using Cursor with Claude, demonstrating the potential of AI-assisted robotics development.

## Hardware

- **Compute:** NVIDIA Jetson Xavier NX (JetPack 5.1.3)
- **Vision:** Waveshare IMX219-83 stereo camera + ICM20948 IMU
- **LiDAR:** Waveshare D500 (LDROBOT LD19 compatible, 360° 2D)
- **Motors:** 2x hoverboard BLDC motors with RioRand 350W controllers
- **Power:** 36V battery system (2x hoverboard packs in parallel)
- **Drivetrain:** Differential drive

## Software Stack

- Ubuntu 20.04
- ROS2 Humble (built from source)
- CUDA-accelerated stereo vision
- Nav2 navigation stack

## Repository Structure

```
autocode-bot/
├── config/                 # Configuration files
│   ├── navigation/         # Nav2 parameters
│   └── robot_description/  # URDF parameters
├── docs/                   # Documentation
└── src/                    # ROS2 packages
    ├── robot_bringup/      # Launch files
    ├── motor_driver/       # Motor control & IMU node
    ├── robot_odometry/     # IMU + laser odometry fusion, scan resampler (C++)
    ├── stereo_vision/      # Camera and depth processing
    ├── robot_description/  # URDF and meshes
    └── telemetry/          # Custom WebSocket bridge + optional web viewer (Foxglove uses foxglove_bridge)
```

## Quick Start

### 1. Python dependencies (IMU node)

The IMU node requires the Pimoroni `icm20948` library (Waveshare-compatible). On the Jetson:

```bash
pip3 install icm20948 smbus2
```

See [docs/hardware/imu_i2c.md](docs/hardware/imu_i2c.md) for I2C bus detection and troubleshooting.

### 2. Build the Workspace

```bash
source ~/ros2_humble/install/setup.bash
cd ~/ros2_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Always pass `-DCMAKE_BUILD_TYPE=Release`: colcon's default is an empty build type, which compiles C++ packages such as the LiDAR driver, cv_bridge and image_geometry without optimization. `stereo_vision` (and slam_toolbox) default to Release on their own; most third-party packages do not.

### 3. Start for Foxglove (IMU, cameras, LiDAR)

On the Jetson, from the workspace root:

```bash
source install/setup.bash
scripts/start_foxglove.sh
```

Then in **Foxglove Studio**: Open connection → **Foxglove WebSocket** → `ws://<JETSON_IP>:8765`. Use panels for `/imu/data_raw`, `/scan`, `/camera/left/compressed`, `/camera/right/compressed` (raw `image_raw` topics also work). One-time setup: see [Foxglove](#foxglove-recommended) below.

**Cameras:** `stereo_camera_node` publishes both topic types per camera from one GStreamer pipeline:

| Topic | Type | Rate | How |
|---|---|---|---|
| `/camera/{left,right}/compressed` | `CompressedImage` (JPEG) | 10 Hz (`jpeg_max_rate`, `jpeg_quality` 85) | Jetson hardware JPEG encoder (`nvjpegenc`) |
| `/camera/{left,right}/image_raw` | `Image` (bgr8, 1280×720) | 20 Hz (`frame_rate`) | `nvvidconv` + CPU BGR conversion |

Each branch only runs while its topic has subscribers, so with no client connected the cameras cost about 30 % of one core in total. Launch respawns the node if a camera stalls or `nvargus-daemon` restarts (Argus cannot recover inside a running process); on Ctrl+C the camera source occasionally needs up to 6 s to stop. The old Python relay (`image_relay`, `compressed_image.launch.py`) is kept only as a fallback; start the camera node with `publish_compressed:=false` if you use it.

### 4. Start LiDAR (Waveshare D500)

**Onboard UART (default):** Connect D500 to Jetson 40-pin (Pin 8=TX, 10=RX, GND). Default port is `/dev/ttyTHS1`.

```bash
# Launch D500 LiDAR (publishes /scan)
ros2 launch robot_bringup d500_lidar.launch.py
```

**Optional power control (GPIO):** Switch LiDAR 5 V on/off via a GPIO pin (e.g. BOARD 12, with MOSFET):

```bash
ros2 launch robot_bringup d500_lidar.launch.py use_lidar_power:=true power_enable_gpio_pin:=12
# Then: ros2 service call /lidar/set_power std_srvs/srv/SetBool "{data: true/false}"
```

**USB adapter:** Use `use_usb:=true` (default `/dev/ttyUSB0`) or `port_name:=/dev/ttyUSB0`. USB and I2C work as-is; no extra setup required.

See [docs/hardware/d500_lidar.md](docs/hardware/d500_lidar.md) for wiring, scan-speed notes, and troubleshooting.

### Foxglove (recommended)

| Port | Description |
|------|-------------|
| 8765 | **foxglove_bridge** (default) — native Foxglove WebSocket. Foxglove Studio → Open connection → **Foxglove WebSocket** → `ws://<JETSON_IP>:8765`. |
| 9090 | Rosbridge WebSocket — fallback only (`FOXGLOVE_BRIDGE=rosbridge scripts/start_foxglove.sh`). |

**Install foxglove_bridge (one time, ~5 min build on the Jetson):**

```bash
cd ~/ros2_ws && ./src/autocode-bot/scripts/install_foxglove_bridge.sh
source install/setup.bash
```

`scripts/start_foxglove.sh` uses foxglove_bridge when it is built, otherwise falls back to rosbridge, then to the custom bridge. The install script pins foxglove-sdk and rosx_introspection to tested commits and builds rosx_introspection with `BUILD_TESTING=OFF` (with tests on, foxglove_bridge fails to configure on missing `sensor_msgs::` targets).

Why foxglove_bridge instead of rosbridge (measured on the Xavier NX, one client, 15 s):

| | foxglove_bridge | rosbridge |
|---|---|---|
| IMU + LiDAR + 2× compressed camera | full rates, **8 %** of one core, 54 MB RSS | full rates, 55 % CPU, 100 MB |
| + 2× raw 1280×720 camera | raw at ~19 fps, other topics unaffected, 95 % CPU | **all topics drop to 1 Hz**, 100 % CPU |

**Rosbridge fallback:** `./src/autocode-bot/scripts/install_rosbridge.sh` builds rosbridge_suite **2.0.4** with `scripts/patches/rosbridge_humble_py38.patch` (upstream Humble rosbridge uses Python 3.10+ features and fails on Ubuntu 20.04's Python 3.8). It also builds **xacro**, which `robot.launch.py` needs for the full robot model (camera and IMU frames); without xacro it silently falls back to a minimal `base_footprint` → `base_link` URDF. With rosbridge, subscribe to the **compressed** camera topics only: raw images over rosbridge's JSON saturate the CPU.

**Optional web panel:** Run `scripts/start_telemetry_real.sh` instead to also start the HTML dashboard (8080) and MJPEG server (8081). Panel setup: [telemetry_debug.md](docs/software/telemetry_debug.md#foxglove-studio-accelerometer--cameras).

### Jetson WiFi: Client vs Access Point

To use the Jetson as a WiFi hotspot (connect your laptop/phone directly to it), run on the Jetson:

- **AP mode:** `sudo scripts/jetson_wifi_ap.sh` — then connect to WiFi **JetsonAP** (default password: `jetsonap123`), Jetson at **10.42.0.1**
- **Back to client (zander):** `sudo scripts/jetson_wifi_client.sh`

See [docs/hardware/jetson_wifi_ap.md](docs/hardware/jetson_wifi_ap.md) for details and custom SSID/password.

### Basic mapping (LiDAR + IMU)

Use the D500 LiDAR and IMU with **slam_toolbox** for 2D mapping. Install or build slam_toolbox (once), then run the mapping launch.

**If you use ROS2 from the system** (e.g. `/opt/ros/humble`):

```bash
sudo apt install ros-humble-slam-toolbox
```

**If you built ROS2 from source** (e.g. in `~/ros2_humble`), build slam_toolbox from source in your workspace. Install system deps first, then build without the RViz plugin (avoids needing the full rviz2 stack):

```bash
# System deps (Boost, SuiteSparse, Ceres)
sudo apt install -y libboost-system-dev libboost-filesystem-dev libboost-thread-dev \
  libboost-serialization-dev libsuitesparse-dev libceres-dev

cd ~/ros2_ws/src
git clone https://github.com/SteveMacenski/slam_toolbox.git -b humble
cd ~/ros2_ws
source ~/ros2_humble/install/setup.bash   # or your ROS2 install path
colcon build --packages-select slam_toolbox --symlink-install --allow-overriding slam_toolbox \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_RVIZ_PLUGIN=OFF -DBoost_NO_BOOST_CMAKE=ON
```

Then run mapping (from workspace root, after `source install/setup.bash`):

```bash
ros2 launch robot_bringup mapping.launch.py
```

This starts: robot state publisher, D500 LiDAR (`/scan`), IMU (`/imu/data_raw`), laser odometry, `imu_odom_node` and slam_toolbox. The map is published on `/map`; TF chain is `map` → `odom` (slam_toolbox) → `base_footprint` (imu_odom_node) → `base_link` → `base_laser`.

**Odometry** (no wheel encoders yet; C++ nodes in `robot_odometry`): `scan_resampler` turns `/scan` into `/scan_fixed` (fixed 500 beams centred on straight ahead, which rf2o needs; the D500 sends 501–505 points per turn), [rf2o_laser_odometry](https://github.com/MAPIRlab/rf2o_laser_odometry) estimates the forward speed from consecutive scans (`/odom_rf2o`), and `imu_odom_node` integrates it along the IMU heading into `/odom` and `odom` → `base_footprint`. The heading uses the gyro projected onto the measured gravity direction, because the IMU on the stereo board is mounted upright. Keep the robot still for the first 5 s (gyro bias and gravity direction). `use_laser_odom:=false` falls back to heading only. Build rf2o once:

```bash
cd ~/ros2_ws/src && git clone -b ros2 https://github.com/MAPIRlab/rf2o_laser_odometry.git
cd ~/ros2_ws && colcon build --symlink-install --packages-select rf2o_laser_odometry --cmake-args -DCMAKE_BUILD_TYPE=Release
``` View the map in Foxglove (Map panel, topic `/map`) or RViz. Move the robot (by hand or with teleop) to build the map; save it later with `ros2 run nav2_map_server map_saver_cli` when using Nav2.

## LiDAR (Waveshare D500)

The D500 is a 360° 2D LiDAR (LDROBOT LD19 compatible) that publishes `sensor_msgs/LaserScan` on `/scan`. It is used for 2D SLAM and Nav2 obstacle avoidance.

- **Topic:** `/scan` (`sensor_msgs/msg/LaserScan`)
- **TF frame:** `base_laser` (static transform from `base_link`)
- **Connection:** Onboard UART `/dev/ttyTHS1` (40-pin: 8=TX, 10=RX) or USB-to-UART
- **Power control:** Optional GPIO via `lidar_control` package; service `lidar/set_power` (SetBool)
- **Scan speed:** Parameter in `lidar_control_node`; runtime change may require driver/protocol support
- **Driver:** [ldlidar_stl_ros2](https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2) (clone into `ros2_ws/src` and build)

Full setup, wiring, and troubleshooting: [docs/hardware/d500_lidar.md](docs/hardware/d500_lidar.md).

## Development Phases

1. **Foundation & Perception** - ROS2 setup, stereo camera integration, depth processing
2. **Navigation & Control** - Motor control, SLAM, path planning
3. **Intelligence & Autonomy** - Object detection, behavior planning

## Development Guidelines

### Code Style
- Python: Follow PEP 8
- C++: Follow ROS2 coding standards
- Use descriptive variable and function names

### Testing
- Test on hardware before committing
- Verify camera feeds and IMU data are working
- Check for memory leaks on long-running nodes

### Commit Messages
- Use clear, descriptive commit messages
- Reference issues where applicable
- Format: `type: brief description`

### Adding New Nodes
1. Create package in `src/` directory
2. Add dependencies to `package.xml`
3. Update CMakeLists.txt or setup.py
4. Build and test locally before pushing

## Troubleshooting

### Cameras not detected
```bash
ls /dev/video*  # Should show video0 and video1
```

### ROS2 commands not found
```bash
source ~/ros2_humble/install/setup.bash
source ~/ros2_ws/install/setup.bash
```

### Port already in use
```bash
pkill -f video_server
pkill -f combined_server
```

### LiDAR not publishing
```bash
ls -l /dev/ttyUSB*   # Check device exists
ros2 launch robot_bringup d500_lidar.launch.py use_usb:=true
# or: port_name:=/dev/ttyUSB0
```

## License

GPL-3.0

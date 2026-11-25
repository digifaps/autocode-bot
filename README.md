# autocode-bot

An autonomous wheeled robot with stereo vision, built entirely with AI-assisted code generation.

## Overview

This project implements an autonomous differential-drive robot running ROS2 on NVIDIA Jetson Xavier NX. The robot uses stereo cameras for depth perception and a 2D LiDAR for mapping, working towards autonomous navigation and obstacle avoidance.

**Key highlight:** All code in this repository is AI-generated (Cursor and Claude), demonstrating the potential of AI-assisted robotics development.

## Hardware

- **Compute:** NVIDIA Jetson Xavier NX (JetPack 5.1.4, L4T 35.6)
- **Vision:** Waveshare IMX219-83 stereo camera + ICM20948 IMU (on the camera board)
- **LiDAR:** Waveshare D500 (LDROBOT LD19 compatible, 360° 2D) on a CP2102 USB adapter
- **Motors:** 2x hoverboard BLDC motors with RioRand 350W controllers (driver not implemented yet)
- **Power:** 36V battery system (2x hoverboard packs in parallel)
- **Drivetrain:** Differential drive

## Software Stack

- Ubuntu 20.04, ROS2 Humble (built from source in `~/ros2_humble`)
- foxglove_bridge for remote viewing in Foxglove
- slam_toolbox (2D mapping on demand), rf2o_laser_odometry
- Nav2 (planned)

## Repository Structure

```
autocode-bot/
├── config/
│   ├── foxglove/           # Foxglove layout (import from file)
│   ├── navigation/         # Nav2 parameters (planned)
│   └── robot_description/  # URDF parameters
├── docs/                   # Hardware and calibration notes
├── scripts/                # Setup, boot service, network, udev rule, patches
└── src/                    # ROS2 packages
    ├── robot_bringup/      # Launch files (bringup.launch.py runs at boot) and configs
    ├── imu_driver/         # ICM20948 + AK09916 IMU driver (C++)
    ├── stereo_vision/      # Stereo cameras (hardware JPEG), calibration tool, depth (stub)
    ├── robot_odometry/     # IMU + laser odometry fusion, scan resampler (C++)
    ├── robot_supervisor/   # Health on /diagnostics, on-demand mapping
    ├── lidar_control/      # Optional LiDAR power switching via GPIO
    ├── motor_driver/       # Motor driver (stub, not launched yet)
    └── robot_description/  # URDF
```

## Setup (once, on the Jetson)

### 1. Third-party packages in the workspace

These are built from source next to this repo in `~/ros2_ws/src` (ROS2 Humble has no binaries for Ubuntu 20.04):

```bash
cd ~/ros2_ws/src
git clone --depth 1 -b 2.1.1 https://github.com/ros/xacro.git                 # full robot model
git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git            # D500 LiDAR driver
git -C ldlidar_stl_ros2 apply ../autocode-bot/scripts/patches/ldlidar_stl_ros2_reconnect.patch
git clone -b ros2 https://github.com/MAPIRlab/rf2o_laser_odometry.git          # laser odometry
git clone https://github.com/SteveMacenski/slam_toolbox.git -b humble          # mapping
cd ~/ros2_ws && ./src/autocode-bot/scripts/install_foxglove_bridge.sh          # foxglove_bridge (~5 min)
```

slam_toolbox needs `sudo apt install -y libboost-system-dev libboost-filesystem-dev libboost-thread-dev libboost-serialization-dev libsuitesparse-dev libceres-dev`, and is built with `--allow-overriding slam_toolbox --cmake-args -DBUILD_RVIZ_PLUGIN=OFF -DBoost_NO_BOOST_CMAKE=ON`. The foxglove_bridge script pins foxglove-sdk and rosx_introspection to tested commits (rosx_introspection must be built with `BUILD_TESTING=OFF`).

### 2. Build the workspace

```bash
source ~/ros2_humble/install/setup.bash
cd ~/ros2_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Always pass `-DCMAKE_BUILD_TYPE=Release`: colcon's default is an empty build type, which compiles third-party C++ packages without optimization.

### 3. Network: Ethernet + the robot's own Wi-Fi

```bash
sudo ~/ros2_ws/src/autocode-bot/scripts/setup_network.sh <wifi-password> [ssid]   # default ssid: autocode-bot
```

- **Ethernet (`eth0`)** is the normal network: DHCP from your router, internet, SSH.
- **Wi-Fi (`wlan0`)** is always the robot's access point (2.4 GHz channel 6), robot at **10.42.0.1**, DHCP for clients. AP clients get internet through Ethernet when it is plugged in.
- The password is stored only in NetworkManager on the Jetson. Details: [docs/hardware/jetson_wifi_ap.md](docs/hardware/jetson_wifi_ap.md).

### 4. Start the robot at boot

```bash
sudo ~/ros2_ws/src/autocode-bot/scripts/install_autostart.sh
```

This installs the LiDAR udev rule (`/dev/lidar`) and the systemd service **autocode-bot**, which runs `robot_bringup/bringup.launch.py` at every boot, restarts it if it stops, and logs to the journal:

```bash
journalctl -u autocode-bot -f                 # logs
sudo systemctl restart autocode-bot           # after a rebuild
sudo systemctl stop autocode-bot              # to run things by hand (scripts/start_foxglove.sh)
sudo scripts/install_autostart.sh --remove    # no autostart
```

Optional: run the robot without the desktop (frees ~400 MB of RAM; ssh, Foxglove and the access point are unaffected):

```bash
sudo scripts/headless.sh            # console at boot, desktop stopped, unused services (Docker, Bluetooth, ModemManager, ...) off
sudo systemctl start gdm            # desktop when needed, without changing the boot default
sudo scripts/headless.sh --undo     # desktop at boot again, services back on
```

## What runs at boot

`bringup.launch.py` starts:

| Part | Nodes | Topics |
|---|---|---|
| Robot model | robot_state_publisher, LiDAR static TF | `/tf`, `/tf_static`, `/robot_description` |
| IMU | `imu_driver/imu_node` (C++) | `/imu/data_raw` 100 Hz, `/imu/mag` |
| Cameras | `stereo_vision/stereo_camera_node` | `/camera/{left,right}/compressed` (JPEG 10 Hz), `image_raw` (20 Hz), `camera_info` |
| LiDAR | `ldlidar_stl_ros2_node` | `/scan` 10 Hz |
| Odometry | `scan_resampler`, `rf2o_laser_odometry`, `imu_odom_node` | `/odom`, TF `odom` → `base_footprint` |
| Foxglove | `foxglove_bridge` | WebSocket on port 8765 |
| Health | `robot_supervisor/health_node`, plus each sensor's own status | `/diagnostics` |
| Mapping | `robot_supervisor/mapping_manager.py` | services `/mapping/start`, `/mapping/stop`, `/mapping/save` |

**Missing sensors don't stop the robot.** Every node respawns on its own, and a missing or failing sensor is reported on `/diagnostics` (shown in Foxglove), e.g. *LiDAR: not connected (/dev/lidar missing)*, *IMU: ICM20948 not found on /dev/i2c-8*, *Camera left: not detected*. The IMU driver retries a missing chip every 2 s; the LiDAR driver is restarted until the USB adapter is back; a camera missing at startup is reported and the other camera keeps running (CSI cameras are only detected when `nvargus-daemon` starts, so reconnecting one needs a restart).

**Middleware: Cyclone DDS.** The stack runs with `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` (set in `scripts/robot_bringup.sh`). With Fast-DDS 2.6 (shared memory), stopping slam_toolbox could leave the TF publisher and its subscribers in a heartbeat storm (+80 % CPU until restart). ROS tools that talk to the robot must use the same middleware, because services between Fast-DDS and Cyclone DDS do not work in Humble: add `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` to `~/.bashrc` on the robot (and on any other machine running ROS tools against it). Foxglove is not affected. Cyclone's settings are in `config/cyclonedds.xml` (larger UDP messages and a 16 MB receive buffer, so raw camera images keep their full 20 Hz; the kernel limit for that is set by `install_autostart.sh`).

**Keep the robot still for the first 5 seconds** after start: the IMU measures its gyro bias and the gravity direction then.

## Foxglove

Foxglove → Open connection → **Foxglove WebSocket** → `ws://<robot-ip>:8765` (on the robot's Wi-Fi: `ws://10.42.0.1:8765`). Install the extension [`config/foxglove/autocode-bot-panels.foxe`](config/foxglove/autocode-bot-panels.foxe) once (drag it onto the Foxglove window, or Settings → Extensions → **Install local extension**; it provides the Mapping panel; Foxglove names it after the extension displayName "autocode-bot panels", which the layout refers to), then import the layout [`config/foxglove/autocode-bot.json`](config/foxglove/autocode-bot.json) (layout menu → **Import from file…**):

- **3D view** following the robot: map, LiDAR scan, robot model, TF, grid
- **Mapping** panel: Start mapping / Stop mapping / Save map buttons, the mapping status and the reply of the last call (slam_toolbox only runs while mapping). Source: `config/foxglove/extension/` (plain JavaScript, repackage with `python3 config/foxglove/extension/package.py`)
- both **camera images**, an **odometry** plot (x, y, speed)
- **Diagnostics**: OK / WARN / ERROR per sensor, odometry, Jetson (temperature, memory, disk), network (Ethernet, access point clients), mapping
- **CPU per core** (%), from `/system/cpu_usage` (published by `robot_health`: `data[0..5]` per core, `data[6]` all cores)
- **Log**: warnings and errors from all nodes

Saved maps go to `~/maps/map_<date_time>.{pgm,yaml}` (map_server format) plus slam_toolbox's pose graph (`.posegraph`, `.data`) to continue mapping later.

**Cameras:** each camera has one GStreamer pipeline split into a hardware JPEG branch (`nvjpegenc`, 10 Hz, quality 85) and a raw bgr8 branch (20 Hz); each branch only runs while its topic has subscribers. `camera_info` comes from `stereo_vision/config/calibration/{left,right}.yaml`; see [docs/calibration/stereo_wide_angle.md](docs/calibration/stereo_wide_angle.md) for the calibration procedure.

## Mapping and odometry

There are no wheel encoders yet. Odometry combines the IMU heading with laser odometry:

- `scan_resampler` turns `/scan` into `/scan_fixed` (fixed 500 beams centred on straight ahead, which rf2o needs; the D500 sends 501–505 points per turn).
- [rf2o_laser_odometry](https://github.com/MAPIRlab/rf2o_laser_odometry) estimates the forward speed from consecutive scans (`/odom_rf2o`).
- `imu_odom_node` integrates it along the IMU heading into `/odom` and `odom` → `base_footprint`. The heading uses the gyro projected onto the measured gravity direction, because the IMU on the stereo board is mounted upright. Implausible laser speeds (scan-match glitches) are rejected.

TF chain while mapping: `map` → `odom` (slam_toolbox) → `base_footprint` (imu_odom_node) → `base_link` → `base_laser`.

Mapping without the boot stack (e.g. for testing) also works: `ros2 launch robot_bringup mapping.launch.py` (LiDAR, IMU, odometry, slam_toolbox always on) or `mapping_with_cameras.launch.py`.

## LiDAR (Waveshare D500)

- **Topic:** `/scan`, frame `base_laser` (static transform from `base_link`)
- **Connection:** USB adapter at `/dev/lidar` (udev rule), or onboard UART `/dev/ttyTHS1`
- **Driver:** [ldlidar_stl_ros2](https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2) with `scripts/patches/ldlidar_stl_ros2_reconnect.patch` (exits when the adapter drops out so it is restarted; fixes an abort on shutdown)
- **Power control:** optional GPIO via `lidar_control`; service `lidar/set_power` (SetBool)

Wiring and troubleshooting: [docs/hardware/d500_lidar.md](docs/hardware/d500_lidar.md). IMU details: [docs/hardware/imu_i2c.md](docs/hardware/imu_i2c.md).

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
- Check `/diagnostics` in Foxglove: every sensor should be OK
- Check for memory leaks on long-running nodes

### Commit Messages
- Use clear, descriptive commit messages
- Reference issues where applicable
- Format: `type: brief description`

### Adding New Nodes
1. Create package in `src/` directory
2. Add dependencies to `package.xml`
3. Update CMakeLists.txt or setup.py
4. Add it to `robot_bringup/launch/bringup.launch.py` with `respawn=True`, and report its health on `/diagnostics`
5. Build and test on the robot before pushing

## Troubleshooting

### Something is red in Foxglove's Diagnostics panel
The message says what is wrong (not connected, no data, low rate). Logs: `journalctl -u autocode-bot -f`.

### Cameras not detected
```bash
ls /dev/video*                    # Should show video0 and video1
sudo systemctl restart nvargus-daemon && sudo systemctl restart autocode-bot
```

### ROS2 commands not found
```bash
source ~/ros2_ws/install/setup.bash
```

### LiDAR not publishing
```bash
ls -l /dev/lidar /dev/ttyUSB*     # adapter present?
dmesg | grep -i cp210x            # USB drop-outs
```

### Can't reach the robot over Wi-Fi
Connect to the robot's Wi-Fi (default SSID `autocode-bot`) and use `10.42.0.1`. Check on the robot: `nmcli connection show --active` should list `autocode-bot-ap`.

## License

GPL-3.0

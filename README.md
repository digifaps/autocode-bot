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
│   ├── camera_calibration/ # Stereo calibration data
│   ├── navigation/         # Nav2 parameters
│   └── robot_description/  # URDF parameters
├── docs/                   # Documentation
└── src/                    # ROS2 packages
    ├── robot_bringup/      # Launch files
    ├── motor_driver/       # Motor control & IMU node
    ├── stereo_vision/      # Camera and depth processing
    ├── robot_description/  # URDF and meshes
    └── telemetry/          # Web-based telemetry dashboard
```

## Quick Start

### 1. Build the Workspace

```bash
source ~/ros2_humble/install/setup.bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 2. Start Telemetry Dashboard

The telemetry system provides a browser-based dashboard for monitoring robot sensors.

```bash
# Terminal 1: Start the IMU data bridge (rosbridge protocol)
python3 ~/ros2_ws/src/autocode-bot/src/telemetry/combined_server.py &

# Terminal 2: Start the video streaming server
python3 ~/ros2_ws/src/autocode-bot/src/telemetry/video_server.py &

# Terminal 3: Start the web server
cd ~/ros2_ws/src/autocode-bot/src/telemetry
python3 -m http.server 8080 &
```

Then open in your browser: **http://<JETSON_IP>:8080/web_viewer.html**

### 3. Start LiDAR (Waveshare D500)

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

**USB adapter:** Use `use_usb:=true` (default `/dev/ttyUSB0`) or `port_name:=/dev/ttyUSB0`. Set permissions: `sudo chmod 666 /dev/ttyUSB0`.

See [docs/hardware/d500_lidar.md](docs/hardware/d500_lidar.md) for wiring, scan-speed notes, and troubleshooting.

### Services Overview

| Service | Port | Description |
|---------|------|-------------|
| Web Dashboard | 8080 | HTML telemetry viewer |
| Video Streams | 8081 | MJPEG stereo camera feeds |
| Rosbridge | 9090 | WebSocket for IMU data |

## Telemetry Dashboard

The web-based telemetry dashboard displays:

- **Stereo Camera Feeds** - Live left/right camera streams via MJPEG
- **LiDAR Range Map** - Top-down 2D view of `/scan` (LaserScan); robot at center, range rings every 2 m
- **IMU Data** - Real-time linear acceleration and angular velocity
- **Connection Status** - WebSocket connection indicator

- **Fake data (no ROS):** Run `combined_server.py` on port 9090 for demo IMU + fake `/scan`.
- **Real data (ROS + LiDAR + stereo + IMU):** Run `scripts/start_telemetry_real.sh` from your workspace root (or from the repo root). It starts ROS (robot + LiDAR over USB), the ROS2→WebSocket bridge on 9090, video server (8081), and HTTP server (8080). Then open the same page for live `/scan`, `/imu/data_raw`, and stereo camera feeds.

### Accessing from External Devices

1. Ensure your device is on the same network as the Jetson
2. Find Jetson IP: `hostname -I`
3. Open browser to: `http://<JETSON_IP>:8080/web_viewer.html`

### Jetson WiFi: Client vs Access Point

To use the Jetson as a WiFi hotspot (connect your laptop/phone directly to it), run on the Jetson:

- **AP mode:** `sudo scripts/jetson_wifi_ap.sh` — then connect to WiFi **JetsonAP** (default password: `jetsonap123`), Jetson at **10.42.0.1**
- **Back to client (zander):** `sudo scripts/jetson_wifi_client.sh`

See [docs/hardware/jetson_wifi_ap.md](docs/hardware/jetson_wifi_ap.md) for details and custom SSID/password.

### Foxglove Studio (Alternative)

You can also connect Foxglove Studio for advanced visualization:

1. Open Foxglove Studio
2. Select "Rosbridge (WebSocket)"
3. Connect to: `ws://<JETSON_IP>:9090`

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
sudo chmod 666 /dev/ttyUSB0
ros2 launch robot_bringup d500_lidar.launch.py use_usb:=true
# or: port_name:=/dev/ttyUSB0
```

## License

GPL-3.0

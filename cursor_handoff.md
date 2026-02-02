# Robot Project - Complete Context for Cursor

## Project Overview
Building an autonomous wheeled robot with stereo vision running ROS2 on NVIDIA Jetson Xavier NX.

---

## Current Status

**Last updated:** 2026-02-01 — *Update this section when the repo or milestones change. Check the repo (e.g. `git status`, `git branch -a`, `git log main -5`) for source of truth.*

| Item | Status |
|------|--------|
| **Repo** | `autocode-bot` on GitHub (digifaps/autocode-bot) |
| **main** | D500 LiDAR merged (UART, power control, telemetry LiDAR map, IMU filtering). Milestone 1.2 done. |
| **Phase** | 1 – Foundation & Perception |
| **Next** | LiDAR sensor fine-tuning; then Milestone 1.3 (depth) or 2.1 (motor control) |

For detailed task log and ownership, see **`agent_sync.md`** in this repo (same directory).

---

## Hardware Specifications

### Compute Platform
- **Main Computer:** NVIDIA Jetson Xavier NX Developer Kit
  - 384-core Volta GPU with 48 Tensor Cores
  - 6-core ARM Carmel CPU
  - 8GB RAM
  - **OS:** Ubuntu 20.04 (via JetPack 5.1.3)
  - **ROS Version:** ROS2 Humble
  - Power modes: 10W, 15W, 20W configurations

### Vision System
- **Camera:** Waveshare IMX219-83 Stereo Camera
  - Dual IMX219 sensors (8MP each)
  - **Wide-angle lenses:** 83° diagonal FOV, 2.6mm focal length (calibrate with full-frame coverage; see `docs/calibration/stereo_wide_angle.md`)
  - 60mm baseline for stereo depth
  - **Bonus:** ICM20948 9-DOF IMU (accelerometer, gyroscope, magnetometer)
  - Interface: Dual CSI to Jetson
  - Software sync (not hardware sync)

### LiDAR (added)
- **Sensor:** Waveshare D500 (LDROBOT LD19 compatible)
  - 360° 2D, up to 12 m range, ~10 Hz
  - UART 230400 baud: onboard Jetson `/dev/ttyTHS1` (40-pin: 8=TX, 10=RX) or USB-to-UART
  - **Power control:** optional GPIO (e.g. BOARD 12) via `lidar_control` package; service `lidar/set_power` (SetBool)
  - **Scan speed:** param `scan_speed_hz` in `lidar_control_node`; runtime change may require LD19 protocol support in driver
  - ROS2: `ldlidar_stl_ros2`, topic `/scan` (`sensor_msgs/LaserScan`), frame `base_laser`
  - See `docs/hardware/d500_lidar.md` and launch `robot_bringup d500_lidar.launch.py`

### Motor System
- **Motors:** 2x Hoverboard BLDC Motors (6", 8", or 10" wheel size)
  - 350W nominal @ 36V
  - Built-in Hall sensors for commutation
  - Built-in planetary gearbox
  - KV rating: ~16

- **Motor Controllers:** 2x RioRand 350W 6-60V Brushless Controllers
  - Operating voltage: 6-60V (running at 36V)
  - Rated: 16A continuous, 20A peak
  - **Control Pins:**
    - PWM input (0-100% duty cycle for speed)
    - DIR pin (direction control)
    - BRAKE pin (braking)
  - **Feedback Pin:**
    - SC speed pulse output (motor RPM feedback)
  - **Configuration:** Onboard potentiometer must be set fully counter-clockwise, two-pin jumper must be shorted for PWM mode
  - Reference: https://mad-ee.com/easy-inexpensive-hoverboard-motor-controller/

### Power System
- **Batteries:** 2x Hoverboard Battery Packs (in parallel)
  - 36V nominal (42V fully charged, 26V cutoff)
  - 4.4Ah each = 8.8Ah total
  - Built-in BMS (Battery Management System)
  - Max discharge: 20A per pack, 40A total
  - Total capacity: 316Wh
  - Configuration: 10S lithium-ion (18650 cells)

- **Power Distribution:**
  ```
  2x Battery Packs (parallel) → 36V Bus
    ├─→ Buck Converter (36V → 12V) → Jetson Xavier NX
    ├─→ RioRand Controller #1 (Left motor)
    └─→ RioRand Controller #2 (Right motor)
  ```

- **Estimated Runtime:** 80-160 minutes continuous operation
  - Jetson: 10-20W
  - Motors (cruising): 100-200W
  - Camera: ~5W

### Drivetrain
- **Configuration:** Differential drive (2-wheel)
- Left and right motors independently controlled
- Odometry from motor speed pins + IMU fusion

---

## System Architecture

### Hardware Architecture
```
┌─────────────────────────────────────────┐
│     Jetson Xavier NX (ROS2 Humble)      │
│                                         │
│  ┌─────────────────────────────────┐   │
│  │   ROS2 Navigation Stack         │   │
│  │   - SLAM                        │   │
│  │   - Path Planning               │   │
│  │   - Obstacle Avoidance          │   │
│  └─────────────────────────────────┘   │
│                                         │
│  ┌──────────┐  ┌──────────────────┐   │
│  │ Stereo   │  │ Motor Driver     │   │
│  │ Vision   │  │ Node             │   │
│  │ Node     │  │ (Custom)         │   │
│  └──────────┘  └──────────────────┘   │
└─────────────────────────────────────────┘
         │                    │
         │ CSI                │ GPIO (PWM, DIR, BRAKE, SPEED)
         ▼                    ▼
┌──────────────────┐  ┌────────────────────┐
│ IMX219-83        │  │ 2x RioRand         │
│ Stereo Camera    │  │ Controllers        │
│ + IMU            │  │                    │
└──────────────────┘  └────────────────────┘
                               │
                               │ 3-phase + Hall
                               ▼
                      ┌────────────────────┐
                      │ 2x Hoverboard      │
                      │ BLDC Motors        │
                      └────────────────────┘
```

### Software Architecture (ROS2)
```
/cmd_vel (geometry_msgs/Twist)
    ↓
┌─────────────────────────────────┐
│  Motor Driver Node (custom)     │
│  - Differential drive kinematics│
│  - PID velocity control         │
│  - Speed pulse processing       │
└─────────────────────────────────┘
    ↓ PWM signals (GPIO)
    ↓ Speed pulses (GPIO interrupts)
RioRand Controllers
    ↓
/odom (nav_msgs/Odometry)
    ↓
Navigation Stack (Nav2)
```

### ROS2 Topic Structure
- `/cmd_vel` - Velocity commands (Twist)
- `/odom` - Odometry from motor encoders + IMU
- `/camera/left/image_raw` - Left camera image
- `/camera/right/image_raw` - Right camera image
- `/camera/depth` - Stereo depth map
- `/camera/points` - Point cloud
- `/imu/data` - IMU data (accel, gyro, mag)
- `/scan` - Virtual laser scan from depth
- `/map` - SLAM-generated map
- `/goal_pose` - Navigation goals

---

## Development Milestones

### Phase 1: Foundation & Perception

**Milestone 1.1: ROS2 Environment Setup**
- Flash Jetson Xavier NX with JetPack 5.1.3
- Install ROS2 Humble
- Create workspace structure
- Configure GitHub integration
- Setup SSH access from host PC
- **Validation:** `ros2 topic list` shows healthy system, SSH works

**Milestone 1.2: Stereo Camera Integration**
- Install camera drivers for dual IMX219
- Configure GStreamer pipelines
- Software synchronization of image pairs
- ROS2 camera publisher nodes
- Stereo calibration (intrinsic + extrinsic)
- IMU integration (ICM20948)
- **Validation:** Stereo streams at 1920x1080 @ 20-30 FPS in RViz2, IMU publishing

**Milestone 1.3: Depth Perception**
- CUDA-accelerated stereo block matching
- Point cloud generation
- PointCloud2 publisher
- Optional: VPI acceleration
- **Validation:** Real-time depth maps >15 FPS, point cloud in RViz2

### Phase 2: Navigation & Control

**Milestone 2.1: Motor Control Interface**
- Custom ROS2 motor driver node:
  - Subscribe to `/cmd_vel`
  - Differential drive kinematics (twist → wheel velocities)
  - PWM output via Jetson.GPIO
  - Speed pulse input (interrupt-based)
  - RPM calculation from pulse frequency
  - PID velocity control loop
  - Odometry publisher
- GPIO configuration (hardware PWM channels)
- Calibration:
  - Pulses-per-revolution
  - Wheel diameter
  - Wheelbase
  - PWM-to-velocity mapping
- **Validation:** Keyboard teleoperation works, odometry published, velocity control accurate

**Milestone 2.2: SLAM (Simultaneous Localization and Mapping)**
- Visual-inertial odometry or stereo SLAM
- Map building
- Localization
- Integration with Nav2
- **Validation:** Navigate environment, build consistent map

**Milestone 2.3: Path Planning & Navigation**
- Nav2 stack integration
- Costmap from depth data
- Autonomous waypoint navigation
- Obstacle avoidance
- **Validation:** Navigate to goal pose autonomously, avoid obstacles

### Phase 3: Intelligence & Autonomy

**Milestone 3.1: Object Detection/Recognition**
- YOLOv8 (or similar) with TensorRT optimization
- Real-time object detection
- Integration with navigation (semantic understanding)
- **Validation:** Real-time detection >15 FPS

**Milestone 3.2: Behavior/Mission Planning**
- State machine for autonomous behaviors
- Task execution framework
- **Validation:** Execute multi-step autonomous mission

---

## Development Environment

### Host Machine (Ubuntu 20.04)
- **Purpose:** Development workstation, Jetson flashing
- **Tools:**
  - Cursor (AI-powered IDE)
  - Git/GitLab
  - NVIDIA SDK Manager (for Jetson flashing)
  - SSH client
  - ROS2 Foxy (optional, for local testing)
- **Connection to Jetson:** 
  - Network (SSH, primary)
  - Serial (backup/debugging)

### Jetson Xavier NX (Ubuntu 20.04 via JetPack 5.1.3)
- **SSH:** `ssh fabrice@192.168.8.171` (or `ssh jetson` if shortcut configured)
- **Web telemetry:** http://192.168.8.171:8080/web_viewer.html
- **Purpose:** Robot brain, runs all ROS2 nodes
- **Software Stack:**
  - JetPack 5.1.3
  - Ubuntu 20.04
  - ROS2 Humble
  - CUDA 11.4+
  - TensorRT 8.5+
  - OpenCV with CUDA
  - GStreamer
  - Jetson.GPIO (Python GPIO library)

### GitHub Repository Structure
```
robot_ws/
├── .clang-format           # C++ code style (to be defined)
├── .pylintrc              # Python style (to be defined)
├── .editorconfig          # Editor config (to be defined)
├── DEVELOPMENT.md         # Development rules (to be defined)
├── README.md
├── cursor_handoff.md      # Project context (this file)
├── agent_sync.md           # Task log, ownership
├── docs/
│   ├── hardware/
│   ├── software/
│   └── calibration/
├── config/
│   ├── camera_calibration/
│   ├── navigation/
│   └── robot_description/
└── src/                   # ROS2 packages
    ├── robot_bringup/     # Launch files
    ├── motor_driver/      # Custom motor controller node
    ├── stereo_vision/     # Camera + depth processing
    └── robot_description/ # URDF, meshes
```

---

## GPIO Pin Assignments (Jetson Xavier NX)

**Motor Control (to be finalized):**
- PWM_LEFT (motor controller 1)
- DIR_LEFT (motor controller 1)
- BRAKE_LEFT (motor controller 1)
- SPEED_LEFT (motor controller 1 feedback)
- PWM_RIGHT (motor controller 2)
- DIR_RIGHT (motor controller 2)
- BRAKE_RIGHT (motor controller 2)
- SPEED_RIGHT (motor controller 2 feedback)

**LiDAR (D500):**
- UART1: `/dev/ttyTHS1` (40-pin: Pin 8 = TX, Pin 10 = RX)
- LIDAR_POWER_ENABLE: optional GPIO (BOARD number, e.g. 12) to switch 5 V on/off via `lidar_control_node`

**Camera:**
- CSI-0: Left camera
- CSI-1: Right camera

**IMU:**
- I2C bus (standard Jetson I2C pins)

---

## Key Technical Details

### Motor Control Algorithm
```python
# Pseudocode for motor driver node

class MotorDriverNode:
    def cmd_vel_callback(twist_msg):
        # Extract linear and angular velocity
        linear = twist_msg.linear.x
        angular = twist_msg.angular.z
        
        # Differential drive kinematics
        wheel_separation = 0.5  # meters (to be measured)
        left_vel = linear - (angular * wheel_separation / 2)
        right_vel = linear + (angular * wheel_separation / 2)
        
        # PID control to match desired velocities
        left_pwm = pid_control(left_vel, measured_left_vel)
        right_pwm = pid_control(right_vel, measured_right_vel)
        
        # Output PWM signals
        set_pwm(LEFT_MOTOR, left_pwm)
        set_pwm(RIGHT_MOTOR, right_pwm)
    
    def speed_pulse_interrupt():
        # Measure pulse frequency to calculate RPM
        pulse_count++
        if time_elapsed >= sample_period:
            rpm = (pulse_count / pulses_per_rev) * (60 / sample_period)
            velocity = rpm_to_velocity(rpm)
            publish_odometry(velocity)
```

### Stereo Vision Pipeline
```
Left Camera → Rectification ┐
                             ├→ Stereo Matching → Disparity Map → Depth Map → Point Cloud
Right Camera → Rectification┘
```

### Calibration Requirements
1. **Stereo Camera Calibration:**
   - Intrinsic parameters (focal length, distortion) for each camera
   - Extrinsic parameters (rotation, translation between cameras)
   - Use checkerboard pattern, ROS2 camera_calibration package

2. **Motor Calibration:**
   - Measure wheel diameter
   - Measure wheelbase (distance between wheels)
   - Count speed pulses per wheel revolution
   - Map PWM duty cycle to actual velocity

3. **IMU Calibration:**
   - Magnetometer calibration
   - Gyroscope bias
   - Accelerometer calibration

---

## Development Workflow

### Validation Gates
Each milestone requires validation before proceeding:
1. **Architecture Review** - Before major implementation
2. **Integration Points** - When connecting components
3. **Performance Targets** - FPS, latency, power consumption
4. **Feature Completion** - Before next milestone

### Git Workflow
- `main` branch: Stable, validated code (protected - no direct push)
- `feature/*` branches: Active development
- **All changes via Pull Requests** - no direct commits to main
- Feature branch → Test → Validate → PR → Merge to main  
  *(Do not push directly to main; use a branch and open a PR.)*

### Commit Message Format (to be defined by user)
Example standard:
```
<type>(<scope>): <subject>

Types: feat, fix, docs, refactor, test, chore
Example: feat(camera): add stereo rectification node
```

---

## Safety Features (Required)

1. **Emergency Stop** - Hardware kill switch for motors
2. **Low Voltage Cutoff** - Protect batteries (<26V shutdown)
3. **Watchdog Timer** - Reset if ROS2 node crashes
4. **Current Monitoring** - Track power consumption
5. **Thermal Management** - Monitor Jetson temperature
6. **Obstacle Detection** - Stop before collision

---

## Performance Targets

### Compute Performance
- Jetson power mode: 15W or 20W (to be determined based on battery life)
- Camera processing: >20 FPS
- Depth map generation: >15 FPS
- Object detection: >15 FPS
- Total latency (perception → action): <100ms

### Navigation Performance
- Maximum speed: 1-2 m/s (to be tuned)
- Minimum obstacle detection range: 2m
- Path planning frequency: 10 Hz
- Localization accuracy: ±5cm position, ±2° orientation

### Power Budget
- Target runtime: 90+ minutes continuous operation
- Peak power: <300W
- Cruise power: ~150W

---

## Immediate Next Steps

1. **Milestone 1.3** - Depth perception (stereo block matching, point cloud) or **Milestone 2.1** - Motor control interface.
2. **LiDAR sensor fine-tuning** - Angle crop, scan rate, validation.
3. **Define development rules** - Code style, commit format (if not yet done).
4. **Chassis / hardware** - Resolve wheel size, frame, any extra sensors.

---

## Questions to Resolve

1. **Hoverboard motor wheel size?** (6", 8", or 10")
2. **Code style preferences?** (Python: PEP8? C++: Google style?)
3. **Commit message format?** (Conventional commits?)
4. ~~GitLab location?~~ **Resolved: GitHub.com**
5. **Chassis design?** (Custom or commercial frame)
6. **Additional sensors?** (Lidar, ultrasonic, etc.)

---

## References & Resources

### Hardware Documentation
- Jetson Xavier NX: https://developer.nvidia.com/embedded/jetson-xavier-nx
- JetPack 5.1.3: https://developer.nvidia.com/jetpack-sdk-513
- Waveshare IMX219-83: https://www.waveshare.com/wiki/IMX219-83_Stereo_Camera
- RioRand Controller: https://mad-ee.com/easy-inexpensive-hoverboard-motor-controller/

### Software Documentation
- ROS2 Humble: https://docs.ros.org/en/humble/
- Nav2: https://navigation.ros.org/
- Jetson.GPIO: https://github.com/NVIDIA/jetson-gpio
- GStreamer: https://gstreamer.freedesktop.org/

### Related Projects
- Stereo camera ROS packages for Jetson
- Hoverboard motor control examples
- Differential drive robot examples

---

## Contact & Support

**User Location:** Mechelen, Flanders, BE
**Development Style:** Command-line focused, autonomous AI development with validation checkpoints

---

*This document contains the complete context for the robot project. Use it to understand the system architecture, hardware specifications, and development roadmap.*

**Keeping this file up to date:** Update the **Current Status** section (and Immediate Next Steps if needed) whenever you merge a feature, finish a milestone, or change active work. Prefer updating at least when switching tasks or after a PR merge; when in doubt, check the repo and `agent_sync.md` first.

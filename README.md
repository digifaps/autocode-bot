# autocode-bot

An autonomous wheeled robot with stereo vision, built entirely with AI-assisted code generation.

## Overview

This project implements an autonomous differential-drive robot running ROS2 on NVIDIA Jetson Xavier NX. The robot uses stereo cameras for depth perception and visual SLAM, enabling autonomous navigation and obstacle avoidance.

**Key highlight:** All code in this repository is AI-generated using Cursor with Claude, demonstrating the potential of AI-assisted robotics development.

## Hardware

- **Compute:** NVIDIA Jetson Xavier NX (JetPack 5.1.3)
- **Vision:** Waveshare IMX219-83 stereo camera + ICM20948 IMU
- **Motors:** 2x hoverboard BLDC motors with RioRand 350W controllers
- **Power:** 36V battery system (2x hoverboard packs in parallel)
- **Drivetrain:** Differential drive

## Software Stack

- Ubuntu 20.04
- ROS2 Humble
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
│   ├── hardware/           # Hardware specs and wiring
│   ├── software/           # Software architecture
│   └── calibration/        # Calibration procedures
└── src/                    # ROS2 packages
    ├── robot_bringup/      # Launch files
    ├── motor_driver/       # Motor control node
    ├── stereo_vision/      # Camera and depth processing
    └── robot_description/  # URDF and meshes
```

## Development Phases

1. **Foundation & Perception** - ROS2 setup, stereo camera integration, depth processing
2. **Navigation & Control** - Motor control, SLAM, path planning
3. **Intelligence & Autonomy** - Object detection, behavior planning

## License

GPL-3.0

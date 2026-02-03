# Agent Sync Log

Shared workspace for Cursor agents working on the Jetson NX robot project.  
*Stored in repo root with `cursor_handoff.md` so they are versioned and travel with the code.*

---

## Current Project Status

- **Phase:** 1 - Foundation & Perception
- **main branch:** D500 LiDAR merged (UART, power control, telemetry LiDAR map, IMU filtering). WiFi AP scripts on main. Next: LiDAR fine-tuning.
- **Active Milestone:** LiDAR sensor fine-tuning
- **Last Updated:** 2026-02-01

---

## Active Tasks

| Agent | Task | Status | Branch |
|-------|------|--------|--------|
| - | ~~D500 LiDAR support~~ | merged to main | - |
| - | LiDAR sensor fine-tuning | next | - |
| - | ~~Create GitHub repo~~ | completed | main |
| - | ~~ROS2 packages scaffold~~ | completed | main |
| - | ~~Stereo camera + IMU drivers~~ | completed | main (PR #4) |
| - | ~~Web telemetry dashboard~~ | completed | main (PR #5) |

---

## Recent Updates

### 2026-02-01
- **Repo status checked:** main has D500 LiDAR, telemetry LiDAR map, IMU filtering, WiFi AP scripts.
- Handoff docs (`cursor_handoff.md`, `agent_sync.md`) added to repo root for version control and branch+PR workflow.
- `agent_sync.md` updated to match actual git state.

### 2026-01-31
- ROS2 Humble built from source on Jetson (346 packages, ~2.5 hours)
- Fixed rclpy Python 3.8 compatibility (PyErr_WarnFormat shim)
- Fixed pybind11_vendor Python 2.7 cache issue
- autocode-bot workspace cloned and built on Jetson
- cv_bridge/vision_opencv added for stereo_vision package
- All 4 robot packages building: motor_driver, stereo_vision, robot_description, robot_bringup
- SSH config added for `ssh jetson` shortcut
- Passwordless sudo enabled on Jetson

### 2026-01-25
- Project handoff document created (`cursor_handoff.md`)
- StreamSDK setup completed
- Agent sync file initialized
- GitHub repo created and cloned (`digifaps/autocode-bot`)
- ROS2 workspace structure created and pushed to main
- ROS2 packages scaffolded (PR #2 merged)

---

## Key Decisions

- **2026-01-25:** Code hosted on **GitHub.com** (not GitLab)
- **2026-01-25:** Repo: **https://github.com/digifaps/autocode-bot** (public, GPL-3.0)
- **2026-01-25:** Git workflow: **feature branches + PRs only** (no direct push to main)
- **Note:** WiFi AP scripts (commit 45ece31) were pushed directly to main; from now on use a feature branch and open a PR, then merge to main.

---

## Blockers / Open Questions

<!-- Add any blockers or questions that need resolution -->

- Hoverboard motor wheel size? (6", 8", or 10")
- Code style preferences? (Python: PEP8? C++: Google style?)
- ~~GitLab location?~~ **Resolved: GitHub.com**

---

## Jetson access

- **SSH:** `ssh fabrice@192.168.8.171` (or `ssh jetson` if shortcut configured)
- **Web telemetry:** http://192.168.8.171:8080/web_viewer.html

---

## Shared Notes

<!-- Free-form notes for cross-agent communication -->

---

## File Ownership

To avoid conflicts, claim files here before major edits:

| File/Directory | Agent | Started | Notes |
|----------------|-------|---------|-------|
| - | - | - | - |

---

## Completed Work

- **Milestone 1.2 (Stereo Camera Integration):** Merged to main via PR #4 (stereo camera + IMU drivers), PR #5 (web telemetry). Telemetry: combined_server (rosbridge), video_server (MJPEG), web_viewer.html.
- **D500 LiDAR:** Merged to main (UART, power control, lidar_control, docs). Uses ldlidar_stl_ros2 (LD19).
- **WiFi AP/client switch:** Scripts in `scripts/` (jetson_wifi_ap.sh, jetson_wifi_client.sh); doc in docs/hardware/jetson_wifi_ap.md.

---

*Instructions: Update this file when starting/completing tasks. Check before editing shared files. When status or milestones change, also update **cursor_handoff.md** → Current Status and Immediate Next Steps. Use a feature branch and open a PR to merge changes to main.*

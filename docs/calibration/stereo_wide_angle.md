# Stereo calibration (wide-angle lenses)

The robot uses a Waveshare IMX219-83 stereo camera with **wide-angle lenses** (83° diagonal, ~60 mm baseline). Depth needs a stereo calibration: intrinsics and distortion per camera, plus the rotation and translation between them, so the images can be rectified (matching points on the same row).

## Why it matters

- **Wide angle ⇒ strong radial distortion** (barrel). If calibration ignores the edges, rectified images will be wrong near the borders and stereo matching fails there.
- **Model**: we calibrate with the standard **plumb_bob** model (k1, k2, p1, p2, k3), which `sensor_msgs/CameraInfo`, `image_geometry` and OpenCV's stereo functions all understand.
- **Not synchronized**: the two IMX219 sensors are free-running, so left and right frames can be up to one frame apart. The calibrator therefore only takes a sample when the board is **held still** in both views.
- **Resolution**: a calibration only holds for the sensor mode it was made at (1280×720 here). The camera node ignores a calibration file with a different size.

## What you need

- **Ready to print (A4):** [calibration_board_A4_9x6_25mm.pdf](calibration_board_A4_9x6_25mm.pdf): 10×7 squares of 25 mm (9×6 inner corners, the calibrator's defaults), with a 100 mm check line. Print at **100 % / Actual size** in landscape. Other sizes: `scripts/make_calibration_board.py --paper A3 --square 35 -o board.pdf` (it prints the matching calibrator parameters). An A4 board is small for wide-angle lenses: take most samples at 0.3–0.8 m; if the edges stay poor, use a larger board.
- A **checkerboard** with an unequal number of inner corners, e.g. **10×7 squares = 9×6 inner corners**. Print it at 100 % scale and glue it to something rigid and flat (foam board, a clipboard, glass). A curled sheet ruins the calibration.
- Squares of **25–40 mm**, so the board is readable from 0.5 to 2 m.
- **Measure the square size with a ruler** after printing, in metres. A wrong value scales the baseline (and every depth) by the same factor.
- Good, even light; avoid glare on the board.

## Procedure

1. **The robot stack must be running** (it starts at boot as the `autocode-bot` service; check with `systemctl status autocode-bot`).
2. **Start the calibrator** in a second terminal, with your board's inner corners and measured square size:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ros2 run stereo_vision stereo_calibrator.py --ros-args \
     -p board_cols:=9 -p board_rows:=6 -p square_size:=0.025
   ```
3. **In Foxglove**, add an **Image** panel on `/calibration/preview`. It shows both views with the detected corners, the sample count, a status line and four coverage bars (X, Y, Size, Skew). It updates about once a second while no board is visible and a few times a second once it is.
4. **Collect samples.** Hold the board in a pose until the status says `sample N saved` (about one second of holding still), then move to a different pose:
   - all over the image: centre, **all four corners and edges** (most important with wide-angle lenses);
   - near (≈0.4 m, board filling most of the view) and far (≈2 m);
   - tilted 30–45° left/right and up/down.
   The board must be fully visible in **both** cameras. Aim for **30–40 samples** and all four bars green. Each pair is saved as full-resolution PNGs in `~/calibration/<date_time>/samples/`.
5. **Solve:**
   ```bash
   ros2 service call /calibration/calibrate std_srvs/srv/Trigger
   ```
   This writes `left.yaml`, `right.yaml` and `report.txt` into the session folder and prints the report, e.g.:
   ```
   RMS reprojection error: left 0.31 px, right 0.22 px, stereo 0.28 px
   rectified row mismatch (epipolar): mean 0.05 px, max 1.49 px
   baseline: 59.9 mm (board spec ~60 mm)
   verdict: GOOD
   ```
   What to look for: stereo RMS **below ~0.5 px** (1 px at most), mean epipolar error **below 0.5 px**, and a baseline **close to 60 mm**. A baseline that is clearly off almost always means a wrong `square_size` or a bent board. You can keep collecting and call the service again.
6. **Install the calibration** and restart the camera node:
   ```bash
   cp ~/calibration/<date_time>/{left,right}.yaml \
      ~/ros2_ws/src/autocode-bot/src/stereo_vision/config/calibration/
   cd ~/ros2_ws && colcon build --symlink-install --packages-select stereo_vision
   ```
   `stereo_camera_node` loads `config/calibration/{left,right}.yaml` from the `stereo_vision` share directory at startup (or the files given by the `left_calibration_file` / `right_calibration_file` parameters) and publishes them on `camera/{left,right}/camera_info`, with exactly the stamp of each published image. Without a file it publishes uncalibrated (all-zero) `camera_info` and logs a warning. Commit the two YAML files so the calibration travels with the repo.

**Re-solving offline** (e.g. after fixing the square size), from the saved PNGs:
```bash
python3 ~/ros2_ws/src/autocode-bot/src/stereo_vision/scripts/stereo_calibrator.py \
  --solve ~/calibration/<date_time> --cols 9 --rows 6 --square 0.0245
```

## If the edges stay bad

If rectified images are still distorted near the borders after a good capture (more corner samples did not help), the lenses may need OpenCV's **fisheye** model. That needs custom rectification, because `image_geometry` assumes plumb_bob; it is not implemented yet.

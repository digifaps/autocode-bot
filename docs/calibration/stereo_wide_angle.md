# Stereo calibration (wide-angle lenses)

The robot uses a Waveshare IMX219-83 stereo camera with **wide-angle lenses**. Strong distortion (especially at the edges) must be captured correctly for good rectification and depth.

## Why it matters

- **Wide angle ⇒ strong radial distortion** (barrel). If calibration ignores the edges or uses a weak model, rectified images will be wrong near the borders and stereo matching will fail there.
- **Standard model**: ROS2 and `image_geometry` use the **plumb_bob** (radial-tangential) distortion model in `CameraInfo` (typically 5 coefficients: k1, k2, p1, p2, k3). This is what the current depth pipeline expects.
- **Fisheye**: For very wide or fisheye lenses, OpenCV’s **fisheye** model can fit better, but then you need custom rectification; `image_geometry::StereoCameraModel` assumes the standard model.

## Recommended approach

1. **Calibrate with the standard (plumb_bob) model first**
   - Use the ROS2 **camera_calibration** stereo flow (or any tool that outputs `sensor_msgs/CameraInfo` with standard distortion).
   - **Cover the whole image**: move the checkerboard into the **center and all four corners/edges**. With wide angle, edge samples are critical; if you only calibrate in the center, edges will be poorly rectified.
   - Use a checkerboard with enough inner corners for your resolution (e.g. 8×6 or 9×6). Print it flat and use a known square size in metres.

2. **Check rectification quality**
   - After saving calibration, run rectification (e.g. in `depth_processor_node` or with `image_proc`) and inspect:
     - Horizontal epipolar lines (same row in left and right).
     - No strong blur or stretching at the edges.
   - If the **edges are still bad**, consider:
     - More calibration poses, especially at the corners.
     - Or switching to **fisheye calibration** and implementing fisheye rectification (custom code; not covered here).

3. **Store calibration**
   - Save the left/right calibration (YAML or CameraInfo) under `config/camera_calibration/` (e.g. a dated subfolder).
   - Ensure the stereo camera node (or `camera_info_manager`) loads and publishes `CameraInfo` for left and right so the depth node can use it.

## Summary

| Lens type   | Calibration model | Coverage during capture      |
|------------|-------------------|------------------------------|
| Wide-angle | Standard (plumb_bob) first | Full frame, especially edges |
| Very wide / fisheye | Consider OpenCV fisheye later | Full frame required          |

Calibrate with **full-frame coverage**; if the standard model gives poor edges, plan for a fisheye model and custom rectification.

# autocode-bot panels

Foxglove extension with a **Mapping** panel: Start mapping / Stop mapping / Save map buttons (services
`/mapping/start|stop|save` from `robot_supervisor/mapping_manager.py`), the "Mapping" status from
`/diagnostics`, and the reply of the last call.

**Vision** panel: switches for stereo depth and object detection, the detection model (Fast = yolox_tiny,
Accurate = yolox_s), live status of both from `/diagnostics`, GPU load and memory. Calls the
`std_srvs/SetBool` services of `robot_supervisor/vision_manager.py` (`/vision/depth/enable`,
`/vision/detection/enable`, `/vision/detection/accurate`); the switch positions follow its "Vision" status.

Plain JavaScript (`dist/extension.js`), no build step. Install `../autocode-bot-panels.foxe` in Foxglove:
drag the file onto the Foxglove window, or Settings → Extensions → Install local extension.
After editing, repackage with `python3 package.py`.

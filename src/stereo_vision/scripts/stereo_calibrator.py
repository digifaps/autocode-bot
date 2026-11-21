#!/usr/bin/env python3
"""
Headless stereo calibration for the IMX219-83, watched from Foxglove.

Live capture (run next to the camera node, e.g. with scripts/start_foxglove.sh):
    ros2 run stereo_vision stereo_calibrator.py --ros-args \
        -p board_cols:=9 -p board_rows:=6 -p square_size:=0.025

  - subscribes camera/{left,right}/image_raw
  - publishes /calibration/preview (CompressedImage): both views with detected corners,
    sample count and coverage bars. Open it in a Foxglove Image panel.
  - saves a sample pair (full-resolution PNGs) only when the board is held STILL in both
    views: the two sensors are not hardware-synchronized, so a moving board would pair
    frames taken at different moments.
  - ros2 service call /calibration/calibrate std_srvs/srv/Trigger  -> solve and write
    left.yaml / right.yaml (camera_calibration_parsers format) + report.txt into the session dir.

Offline (re-solve a saved session, e.g. with a different board size):
    stereo_calibrator.py --solve ~/calibration/<session> --cols 9 --rows 6 --square 0.025

board_cols / board_rows are INNER corners (a 10x7-square board has 9x6 inner corners).
"""

import argparse
import datetime
import glob
import math
import os
import sys
import threading
import time

import cv2
import numpy as np

# --------------------------------------------------------------------------------------
# Detection and solving (no ROS; also used by --solve and by tests)
# --------------------------------------------------------------------------------------

DETECT_WIDTH = 640  # detect on a downscaled copy, refine corners at full resolution
SUBPIX_CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 0.01)


def find_corners(gray, cols, rows):
    """Chessboard inner corners at full resolution, consistently ordered, or None."""
    h, w = gray.shape[:2]
    scale = DETECT_WIDTH / float(w) if w > DETECT_WIDTH else 1.0
    small = cv2.resize(gray, (int(round(w * scale)), int(round(h * scale)))) if scale < 1.0 else gray
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK
    ok, corners = cv2.findChessboardCorners(small, (cols, rows), flags)
    if not ok:
        return None
    corners = (corners / scale).astype(np.float32)
    win = max(5, int(round(0.5 / scale * 5)))
    corners = cv2.cornerSubPix(gray, corners, (win, win), (-1, -1), SUBPIX_CRITERIA)
    return canonical_order(corners, cols, rows)


def canonical_order(corners, cols, rows):
    """OpenCV may return the grid rotated by 180 degrees; make the first corner the top-left
    one in both views so left/right point correspondences match."""
    c = corners.reshape(-1, 2)
    first, last = c[0], c[-1]
    if first[0] + first[1] > last[0] + last[1]:
        c = c[::-1].copy()
    return c.reshape(-1, 1, 2).astype(np.float32)


def match_pair(lc, rc, max_row_diff=40.0):
    """Return right corners ordered like the left ones, or None. With side-by-side cameras a
    corner sits on roughly the same row in both views and further left in the right view."""
    for cand in (rc, rc[::-1].copy()):
        d = lc.reshape(-1, 2) - cand.reshape(-1, 2)
        if np.abs(d[:, 1]).mean() < max_row_diff and d[:, 0].min() > -5.0:
            return cand
    return None


def board_params(corners, cols, rows, width, height):
    """(x, y, size, skew) in [0, 1], the same coverage measures ROS camera_calibration uses."""
    c = corners.reshape(-1, 2)
    up_left, up_right = c[0], c[cols - 1]
    down_right, down_left = c[-1], c[-cols]
    a, b, d = up_right - up_left, down_right - up_right, down_left - down_right
    p = b + d
    area = abs(p[0] * (a[1] + b[1]) - p[1] * (a[0] + b[0])) / 2.0  # quadrilateral area
    size = math.sqrt(area / float(width * height))

    def angle(u, v):
        return math.acos(np.dot(u, v) / (np.linalg.norm(u) * np.linalg.norm(v)))
    skew = min(1.0, 2.0 * abs(math.pi / 2 - angle(up_left - up_right, down_right - up_right)))
    mean = c.mean(axis=0)
    border = math.sqrt(area)
    x = min(1.0, max(0.0, (mean[0] - border / 2) / (width - border))) if width > border else 0.5
    y = min(1.0, max(0.0, (mean[1] - border / 2) / (height - border))) if height > border else 0.5
    return (x, y, size, skew)


def coverage(params_list):
    """Per-dimension progress in [0, 1] (spread of x, y, size, skew), like camera_calibration."""
    if not params_list:
        return [0.0, 0.0, 0.0, 0.0]
    arr = np.array(params_list)
    lo, hi = arr.min(axis=0), arr.max(axis=0)
    # x and y should span the frame; size should range from far to close; some tilt is needed
    targets = [0.7, 0.7, 0.4, 0.5]
    lo[2] = 0.0  # size progress counts from zero up to the largest board seen
    lo[3] = 0.0
    return [min(1.0, (hi[i] - lo[i]) / targets[i]) for i in range(4)]


def object_points(cols, rows, square):
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square
    return objp


def solve(left_corners, right_corners, image_size, cols, rows, square):
    """Intrinsics per camera, stereo extrinsics with intrinsics fixed, rectification.
    Returns a dict with matrices and quality metrics."""
    objp = object_points(cols, rows, square)
    obj = [objp] * len(left_corners)
    flags = 0  # plumb_bob: k1, k2, p1, p2, k3
    rms_l, K1, D1, _, _ = cv2.calibrateCamera(obj, left_corners, image_size, None, None, flags=flags)
    rms_r, K2, D2, _, _ = cv2.calibrateCamera(obj, right_corners, image_size, None, None, flags=flags)
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6)
    rms_s, K1, D1, K2, D2, R, T, E, F = cv2.stereoCalibrate(
        obj, left_corners, right_corners, K1, D1, K2, D2, image_size,
        criteria=crit, flags=cv2.CALIB_FIX_INTRINSIC)
    R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(
        K1, D1, K2, D2, image_size, R, T, flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)
    # Epipolar check: after rectification, matching corners must lie on the same row
    errs = []
    for lc, rc in zip(left_corners, right_corners):
        lr = cv2.undistortPoints(lc, K1, D1, R=R1, P=P1).reshape(-1, 2)
        rr = cv2.undistortPoints(rc, K2, D2, R=R2, P=P2).reshape(-1, 2)
        errs.append(np.abs(lr[:, 1] - rr[:, 1]))
    errs = np.concatenate(errs)
    return dict(K1=K1, D1=D1, K2=K2, D2=D2, R=R, T=T, R1=R1, R2=R2, P1=P1, P2=P2, Q=Q,
                rms_left=rms_l, rms_right=rms_r, rms_stereo=rms_s,
                epipolar_mean=float(errs.mean()), epipolar_max=float(errs.max()),
                baseline=float(np.linalg.norm(T)), samples=len(left_corners),
                image_size=image_size)


def _matrix_yaml(name, m, rows, cols):
    data = ", ".join("%.8g" % v for v in np.asarray(m, dtype=np.float64).reshape(-1))
    return "%s:\n  rows: %d\n  cols: %d\n  data: [%s]\n" % (name, rows, cols, data)


def camera_info_yaml(name, size, K, D, R, P):
    """camera_calibration_parsers YAML (what stereo_camera_node and camera_info_manager read)."""
    return ("image_width: %d\nimage_height: %d\ncamera_name: %s\n" % (size[0], size[1], name)
            + _matrix_yaml("camera_matrix", K, 3, 3)
            + "distortion_model: plumb_bob\n"
            + _matrix_yaml("distortion_coefficients", np.asarray(D).reshape(-1)[:5], 1, 5)
            + _matrix_yaml("rectification_matrix", R, 3, 3)
            + _matrix_yaml("projection_matrix", P, 3, 4))


def report_text(res, cols, rows, square, expected_baseline=0.06):
    b = res["baseline"]
    lines = [
        "Stereo calibration %s" % datetime.datetime.now().isoformat(timespec="seconds"),
        "board: %dx%d inner corners, square %.4f m, %d sample pairs, image %dx%d" % (
            cols, rows, square, res["samples"], res["image_size"][0], res["image_size"][1]),
        "RMS reprojection error: left %.3f px, right %.3f px, stereo %.3f px" % (
            res["rms_left"], res["rms_right"], res["rms_stereo"]),
        "rectified row mismatch (epipolar): mean %.3f px, max %.3f px" % (
            res["epipolar_mean"], res["epipolar_max"]),
        "baseline: %.1f mm (board spec ~%.0f mm)" % (b * 1000, expected_baseline * 1000),
        "left  fx=%.1f fy=%.1f cx=%.1f cy=%.1f D=%s" % (
            res["K1"][0, 0], res["K1"][1, 1], res["K1"][0, 2], res["K1"][1, 2],
            np.round(res["D1"].reshape(-1), 4).tolist()),
        "right fx=%.1f fy=%.1f cx=%.1f cy=%.1f D=%s" % (
            res["K2"][0, 0], res["K2"][1, 1], res["K2"][0, 2], res["K2"][1, 2],
            np.round(res["D2"].reshape(-1), 4).tolist()),
    ]
    verdict = []
    if res["rms_stereo"] > 1.0:
        verdict.append("stereo RMS > 1 px: check the square size, board flatness and blurry samples")
    if res["epipolar_mean"] > 0.5:
        verdict.append("rows misaligned by > 0.5 px after rectification: depth will be poor")
    if abs(b - expected_baseline) > 0.01:
        verdict.append("baseline differs from ~%.0f mm by > 10 mm: wrong square size or bad samples"
                       % (expected_baseline * 1000))
    lines.append("verdict: " + ("GOOD" if not verdict else "CHECK: " + "; ".join(verdict)))
    return "\n".join(lines) + "\n"


def write_results(out_dir, res, cols, rows, square):
    size = res["image_size"]
    with open(os.path.join(out_dir, "left.yaml"), "w") as f:
        f.write(camera_info_yaml("left", size, res["K1"], res["D1"], res["R1"], res["P1"]))
    with open(os.path.join(out_dir, "right.yaml"), "w") as f:
        f.write(camera_info_yaml("right", size, res["K2"], res["D2"], res["R2"], res["P2"]))
    text = report_text(res, cols, rows, square)
    with open(os.path.join(out_dir, "report.txt"), "w") as f:
        f.write(text)
    return text


def solve_directory(session_dir, cols, rows, square):
    """Re-detect corners in saved PNG pairs and solve."""
    lefts = sorted(glob.glob(os.path.join(session_dir, "samples", "left_*.png")))
    lc, rc, size = [], [], None
    for lp in lefts:
        rp = lp.replace("left_", "right_")
        if not os.path.exists(rp):
            continue
        gl = cv2.imread(lp, cv2.IMREAD_GRAYSCALE)
        gr = cv2.imread(rp, cv2.IMREAD_GRAYSCALE)
        a, b = find_corners(gl, cols, rows), find_corners(gr, cols, rows)
        if a is not None and b is not None:
            b = match_pair(a, b)
        if a is None or b is None:
            print("skipping %s: board not found in both images" % os.path.basename(lp))
            continue
        lc.append(a)
        rc.append(b)
        size = (gl.shape[1], gl.shape[0])
    if len(lc) < 10:
        raise RuntimeError("only %d usable sample pairs in %s (need >= 10)" % (len(lc), session_dir))
    res = solve(lc, rc, size, cols, rows, square)
    return write_results(session_dir, res, cols, rows, square)


# --------------------------------------------------------------------------------------
# ROS node
# --------------------------------------------------------------------------------------

def run_node():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage, Image
    from std_srvs.srv import Trigger

    class StereoCalibrator(Node):
        def __init__(self):
            super().__init__("stereo_calibrator")
            self.declare_parameter("board_cols", 9)
            self.declare_parameter("board_rows", 6)
            self.declare_parameter("square_size", 0.025)
            self.declare_parameter("output_dir", os.path.expanduser("~/calibration"))
            self.declare_parameter("detect_rate", 4.0)
            self.declare_parameter("still_px", 1.5)            # max corner motion between checks
            self.declare_parameter("pair_max_dt", 0.03)       # max left/right stamp difference
            self.declare_parameter("min_param_distance", 0.15)  # novelty needed for a new sample
            self.declare_parameter("target_samples", 40)

            self.cols = int(self.get_parameter("board_cols").value)
            self.rows = int(self.get_parameter("board_rows").value)
            self.square = float(self.get_parameter("square_size").value)
            self.still_px = float(self.get_parameter("still_px").value)
            self.pair_max_dt = float(self.get_parameter("pair_max_dt").value)
            self.min_dist = float(self.get_parameter("min_param_distance").value)
            self.target = int(self.get_parameter("target_samples").value)
            stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            self.session = os.path.join(os.path.expanduser(self.get_parameter("output_dir").value), stamp)
            os.makedirs(os.path.join(self.session, "samples"), exist_ok=True)

            self._lock = threading.Lock()
            self._latest = {"left": None, "right": None}
            self._prev = None          # corners of the previous check (left, right)
            self._still_count = 0
            self.samples = []          # (left_corners, right_corners, params)
            self.size = None
            self.status = "waiting for images"
            self.last_report = ""

            for side in ("left", "right"):
                self.create_subscription(Image, "camera/%s/image_raw" % side,
                                         lambda m, s=side: self._on_image(s, m), qos_profile_sensor_data)
            self.preview_pub = self.create_publisher(CompressedImage, "calibration/preview", 2)
            self.create_service(Trigger, "calibration/calibrate", self._on_calibrate)
            self.create_timer(1.0 / float(self.get_parameter("detect_rate").value), self._tick)
            self.get_logger().info(
                "Stereo calibrator: board %dx%d inner corners, square %.4f m. Session: %s. "
                "Preview: /calibration/preview. Solve: ros2 service call /calibration/calibrate "
                "std_srvs/srv/Trigger" % (self.cols, self.rows, self.square, self.session))

        def _on_image(self, side, msg):
            with self._lock:
                self._latest[side] = msg

        @staticmethod
        def _to_bgr(msg):
            if msg.encoding != "bgr8":
                return None
            buf = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
            return buf[:, :msg.width * 3].reshape(msg.height, msg.width, 3)

        def _tick(self):
            with self._lock:
                lm, rm = self._latest["left"], self._latest["right"]
            if lm is None or rm is None:
                self.status = "waiting for camera/left|right/image_raw"
                self._publish_preview(None, None, None, None)
                return
            dt = abs((lm.header.stamp.sec - rm.header.stamp.sec)
                     + (lm.header.stamp.nanosec - rm.header.stamp.nanosec) * 1e-9)
            left, right = self._to_bgr(lm), self._to_bgr(rm)
            if left is None or right is None:
                self.status = "unsupported encoding %s" % lm.encoding
                return
            self.size = (left.shape[1], left.shape[0])
            gl = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
            gr = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
            lc = find_corners(gl, self.cols, self.rows)
            rc = find_corners(gr, self.cols, self.rows) if lc is not None else None
            if rc is not None:
                rc = match_pair(lc, rc)
                if rc is None:
                    self.status = "left/right corners do not match (board too oblique?)"
            if lc is None or rc is None:
                if not self.status.startswith("left/right corners"):
                    self.status = "board not visible in both views"
                self._prev, self._still_count = None, 0
            elif dt > self.pair_max_dt:
                self.status = "left/right frames %.0f ms apart, waiting" % (dt * 1000)
            else:
                moved = 1e9
                if self._prev is not None:
                    moved = max(np.abs(lc - self._prev[0]).max(), np.abs(rc - self._prev[1]).max())
                self._prev = (lc, rc)
                self._still_count = self._still_count + 1 if moved < self.still_px else 0
                if self._still_count < 2:
                    self.status = "hold the board still (moved %.1f px)" % min(moved, 999)
                else:
                    params = board_params(lc, self.cols, self.rows, *self.size)
                    novel = all(sum(abs(a - b) for a, b in zip(params, s[2])) > self.min_dist
                                for s in self.samples)
                    if novel:
                        self._save_sample(left, right, lc, rc, params)
                        self.status = "sample %d saved - move to a new position/angle/distance" % len(self.samples)
                    else:
                        self.status = "already have this pose - move/tilt the board"
            self._publish_preview(left, right, lc, rc)

        def _save_sample(self, left, right, lc, rc, params):
            i = len(self.samples)
            cv2.imwrite(os.path.join(self.session, "samples", "left_%03d.png" % i), left)
            cv2.imwrite(os.path.join(self.session, "samples", "right_%03d.png" % i), right)
            self.samples.append((lc, rc, params))
            self._still_count = 0
            self.get_logger().info("sample %d: x=%.2f y=%.2f size=%.2f skew=%.2f"
                                   % ((i + 1,) + tuple(params)))

        def _publish_preview(self, left, right, lc, rc):
            w, h = 640, 360
            views = []
            for img, c in ((left, lc), (right, rc)):
                if img is None:
                    v = np.zeros((h, w, 3), np.uint8)
                else:
                    sx = w / float(img.shape[1])
                    v = cv2.resize(img, (w, h))
                    if c is not None:
                        cv2.drawChessboardCorners(v, (self.cols, self.rows), (c * sx).astype(np.float32), True)
                views.append(v)
            canvas = np.zeros((h + 110, 2 * w, 3), np.uint8)
            canvas[:h, :w], canvas[:h, w:] = views
            cov = coverage([s[2] for s in self.samples])
            cv2.putText(canvas, "samples %d/%d   %s" % (len(self.samples), self.target, self.status),
                        (10, h + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
            for i, (name, val) in enumerate(zip(("X", "Y", "Size", "Skew"), cov)):
                x0 = 10 + i * 315
                cv2.putText(canvas, name, (x0, h + 60), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
                cv2.rectangle(canvas, (x0 + 50, h + 47), (x0 + 290, h + 63), (80, 80, 80), 1)
                color = (0, 200, 0) if val >= 1.0 else (0, 200, 255)
                cv2.rectangle(canvas, (x0 + 50, h + 47), (x0 + 50 + int(240 * val), h + 63), color, -1)
            if self.last_report:
                cv2.putText(canvas, self.last_report.splitlines()[-1][:110], (10, h + 95),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
            ok, jpg = cv2.imencode(".jpg", canvas, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                msg = CompressedImage()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "calibration_preview"
                msg.format = "jpeg"
                msg.data = jpg.tobytes()
                self.preview_pub.publish(msg)

        def _on_calibrate(self, request, response):
            if len(self.samples) < 10:
                response.success = False
                response.message = "need at least 10 samples, have %d" % len(self.samples)
                return response
            try:
                res = solve([s[0] for s in self.samples], [s[1] for s in self.samples],
                            self.size, self.cols, self.rows, self.square)
                text = write_results(self.session, res, self.cols, self.rows, self.square)
            except Exception as e:  # noqa: BLE001 - report any solver failure to the caller
                response.success = False
                response.message = "calibration failed: %s" % e
                return response
            self.last_report = text
            self.get_logger().info("\n" + text + "Written to %s (left.yaml, right.yaml)" % self.session)
            response.success = True
            response.message = text + "Files: %s/left.yaml, right.yaml" % self.session
            return response

    rclpy.init(args=sys.argv)
    node = StereoCalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    if "--solve" in sys.argv:
        ap = argparse.ArgumentParser(description="Solve a saved stereo calibration session")
        ap.add_argument("--solve", required=True, metavar="SESSION_DIR")
        ap.add_argument("--cols", type=int, default=9, help="inner corners per row")
        ap.add_argument("--rows", type=int, default=6, help="inner corners per column")
        ap.add_argument("--square", type=float, default=0.025, help="square size in metres")
        a = ap.parse_args()
        print(solve_directory(os.path.expanduser(a.solve), a.cols, a.rows, a.square))
        return
    run_node()


if __name__ == "__main__":
    main()

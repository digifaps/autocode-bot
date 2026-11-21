#!/usr/bin/env python3
"""
Start / stop / save slam_toolbox mapping on demand (e.g. from Foxglove Service Call buttons).

Services (std_srvs/srv/Trigger):
  /mapping/start  launch slam_toolbox (async, robot_bringup/config/slam_toolbox_params.yaml)
  /mapping/stop   stop it (SIGINT, then SIGTERM/SIGKILL if it hangs)
  /mapping/save   write the current map to ~/maps/<date_time>.{pgm,yaml} (map_server format) and
                  slam_toolbox's pose graph (<date_time>.posegraph/.data) to continue mapping later

Reports "Mapping" on /diagnostics once per second.
"""

import datetime
import math
import os
import signal
import subprocess
import threading
import time

import rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import OccupancyGrid
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger

try:
    from slam_toolbox.srv import SerializePoseGraph
except ImportError:  # slam_toolbox not built: start fails with a clear message
    SerializePoseGraph = None


class MappingManager(Node):
    def __init__(self):
        super().__init__("mapping_manager")
        self.declare_parameter("slam_params", os.path.join(
            get_package_share_directory("robot_bringup"), "config", "slam_toolbox_params.yaml"))
        self.declare_parameter("maps_dir", os.path.expanduser("~/maps"))
        self.slam_params = self.get_parameter("slam_params").value
        self.maps_dir = os.path.expanduser(self.get_parameter("maps_dir").value)

        self._proc = None
        self._started_at = None
        self._unexpected_exit = None
        self._lock = threading.Lock()
        self._map = None
        self._map_time = None
        self._last_saved = None

        services = ReentrantCallbackGroup()
        self.create_service(Trigger, "mapping/start", self._start, callback_group=services)
        self.create_service(Trigger, "mapping/stop", self._stop, callback_group=services)
        self.create_service(Trigger, "mapping/save", self._save, callback_group=services)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, "map", self._on_map, latched,
                                 callback_group=MutuallyExclusiveCallbackGroup())
        self._serialize = (self.create_client(SerializePoseGraph, "slam_toolbox/serialize_map",
                                              callback_group=services)
                           if SerializePoseGraph else None)
        self._diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self._stop_leftovers()
        self.create_timer(1.0, self._diagnostics, callback_group=MutuallyExclusiveCallbackGroup())
        self.get_logger().info("mapping_manager ready: /mapping/start, /mapping/stop, /mapping/save "
                               "(maps go to %s)" % self.maps_dir)

    def _stop_leftovers(self):
        """A slam_toolbox we started before a crash of this node would keep running unseen."""
        for pid in filter(str.isdigit, os.listdir("/proc")):
            try:
                with open("/proc/%s/cmdline" % pid, "rb") as f:
                    cmd = f.read().split(b"\0")
            except OSError:
                continue
            if any(c.endswith(b"async_slam_toolbox_node") for c in cmd) and b"__node:=slam_toolbox" in cmd:
                self.get_logger().warn("stopping leftover slam_toolbox (pid %s)" % pid)
                try:
                    os.kill(int(pid), signal.SIGINT)
                except OSError:
                    pass

    # ----------------------------------------------------------------------------- services
    def _running(self):
        return self._proc is not None and self._proc.poll() is None

    def _start(self, request, response):
        with self._lock:
            if self._running():
                response.success, response.message = True, "mapping already running"
                return response
            try:
                exe = os.path.join(get_package_prefix("slam_toolbox"), "lib", "slam_toolbox",
                                   "async_slam_toolbox_node")
            except Exception:  # noqa: BLE001 - package lookup errors
                response.success, response.message = False, "slam_toolbox is not installed"
                return response
            # Same cgroup as the robot service, so stopping the service also ends slam_toolbox
            self._proc = subprocess.Popen(
                [exe, "--ros-args", "-r", "__node:=slam_toolbox", "--params-file", self.slam_params])
            self._started_at = time.monotonic()
            self._unexpected_exit = None
            self._map = None
        self.get_logger().info("slam_toolbox started (pid %d)" % self._proc.pid)
        response.success, response.message = True, "mapping started"
        return response

    def _stop_proc(self):
        proc = self._proc
        if proc is None or proc.poll() is not None:
            self._proc = None
            return False
        for sig, wait in ((signal.SIGINT, 10), (signal.SIGTERM, 5), (signal.SIGKILL, 2)):
            try:
                proc.send_signal(sig)
            except ProcessLookupError:
                break
            try:
                proc.wait(wait)
                break
            except subprocess.TimeoutExpired:
                self.get_logger().warn("slam_toolbox did not stop after %s" % sig.name)
        self._proc = None
        return True

    def _stop(self, request, response):
        with self._lock:
            was_running = self._stop_proc()
        response.success = True
        response.message = "mapping stopped" if was_running else "mapping was not running"
        if was_running:
            self.get_logger().info("slam_toolbox stopped")
        return response

    def _save(self, request, response):
        grid = self._map
        if grid is None:
            response.success = False
            response.message = "no map yet (start mapping and move the robot first)"
            return response
        os.makedirs(self.maps_dir, exist_ok=True)
        base = os.path.join(self.maps_dir, datetime.datetime.now().strftime("map_%Y%m%d_%H%M%S"))
        try:
            self._write_map(grid, base)
        except OSError as e:
            response.success, response.message = False, "could not write map: %s" % e
            return response
        saved = [base + ".pgm", base + ".yaml"]
        note = ""
        if self._running() and self._serialize is not None:
            if self._serialize.wait_for_service(timeout_sec=2.0):
                req = SerializePoseGraph.Request()
                req.filename = base
                future = self._serialize.call_async(req)
                end = time.monotonic() + 15.0
                while not future.done() and time.monotonic() < end:
                    time.sleep(0.05)
                if future.done() and future.result() is not None:
                    saved += [base + ".posegraph", base + ".data"]
                else:
                    note = " (pose graph not saved: slam_toolbox did not answer)"
            else:
                note = " (pose graph not saved: slam_toolbox service unavailable)"
        self._last_saved = base
        response.success = True
        response.message = "saved " + ", ".join(os.path.basename(p) for p in saved) + \
            " in " + self.maps_dir + note
        self.get_logger().info(response.message)
        return response

    @staticmethod
    def _write_map(grid, base):
        """map_server format: PGM (white free, black occupied, grey unknown) + YAML."""
        w, h = grid.info.width, grid.info.height
        data = grid.data
        pixels = bytearray(w * h)
        for row in range(h):  # PGM rows go top-down, the grid bottom-up
            src = (h - 1 - row) * w
            dst = row * w
            for col in range(w):
                v = data[src + col]
                pixels[dst + col] = 205 if v < 0 else (254 if v <= 25 else (0 if v >= 65 else 205))
        with open(base + ".pgm", "wb") as f:
            f.write(b"P5\n# autocode-bot map\n%d %d\n255\n" % (w, h))
            f.write(bytes(pixels))
        o = grid.info.origin
        yaw = 2.0 * math.atan2(o.orientation.z, o.orientation.w)
        with open(base + ".yaml", "w") as f:
            f.write("image: %s.pgm\nmode: trinary\nresolution: %.6f\norigin: [%.6f, %.6f, %.6f]\n"
                    "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n"
                    % (os.path.basename(base), grid.info.resolution, o.position.x, o.position.y, yaw))

    # ----------------------------------------------------------------------------- status
    def _on_map(self, msg):
        self._map = msg
        self._map_time = time.monotonic()

    def _diagnostics(self):
        st = DiagnosticStatus()
        st.name = "Mapping"
        st.hardware_id = "slam_toolbox"
        if self._proc is not None and self._proc.poll() is not None and self._unexpected_exit is None:
            self._unexpected_exit = self._proc.returncode
            self.get_logger().error("slam_toolbox exited unexpectedly (code %s)" % self._proc.returncode)
        if self._running():
            st.level = DiagnosticStatus.OK
            up = time.monotonic() - self._started_at
            if self._map is not None:
                info = self._map.info
                st.message = "running %d s, map %.1f x %.1f m" % (
                    up, info.width * info.resolution, info.height * info.resolution)
            else:
                st.message = "running %d s, waiting for the first map" % up
        elif self._unexpected_exit is not None:
            st.level = DiagnosticStatus.ERROR
            st.message = "slam_toolbox exited unexpectedly (code %s); press Start again" % self._unexpected_exit
        else:
            st.level = DiagnosticStatus.OK
            st.message = "off (call /mapping/start)"
        st.values = [KeyValue(key="last_saved", value=self._last_saved or "-"),
                     KeyValue(key="maps_dir", value=self.maps_dir)]
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [st]
        self._diag_pub.publish(arr)

    def shutdown(self):
        with self._lock:
            self._stop_proc()


def _sigint_once(signum, frame):
    # launch sends SIGINT twice on Ctrl+C; ignore the second so slam_toolbox is stopped cleanly
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    raise KeyboardInterrupt


def main(args=None):
    signal.signal(signal.SIGINT, _sigint_once)
    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor(num_threads=3)
    try:
        node = MappingManager()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

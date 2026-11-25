#!/usr/bin/env python3
"""
Choose what the GPU and DLA run: stereo depth and object detection on / off, detection model fast / accurate.

Services (std_srvs/srv/SetBool, e.g. from the Foxglove Vision panel):
  /vision/depth/enable      true = stereo depth may run (stereo_depth parameter `enabled`)
  /vision/detection/enable  true = object_detector loaded and running; false = unloaded (frees its memory)
  /vision/detection/accurate  true = yolox_s (more accurate, ~25 Hz on the DLA), false = yolox_tiny (fast)

The choice is saved in ~/.config/autocode-bot/vision.json and applied at start. A reconcile loop keeps the
nodes in the chosen state, so a respawned node is put back automatically. Both nodes still idle while
nothing subscribes to their outputs: "on" means allowed to run.

Detection without depth runs 2D only (no distances). Reports "Vision" on /diagnostics once per second, with
the GPU load and memory use.
"""

import json
import os
import threading
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import SetBool

MODELS = {False: "yolox_tiny", True: "yolox_s"}
DEFAULTS = {"depth": True, "detection": True, "accurate": False}
GPU_LOAD = "/sys/devices/gpu.0/load"  # 0..1000


class VisionManager(Node):
    def __init__(self):
        super().__init__("vision_manager")
        self.declare_parameter("depth_node", "stereo_depth")
        self.declare_parameter("detector_node", "object_detector")
        self.declare_parameter("state_file", os.path.expanduser("~/.config/autocode-bot/vision.json"))
        self.depth_node = self.get_parameter("depth_node").value
        self.det_node = self.get_parameter("detector_node").value
        self.state_file = os.path.expanduser(self.get_parameter("state_file").value)

        self.want = self._load()
        self.actual = {"depth": None, "detection": None, "model": None}  # last seen, None = unknown
        self.last_error = ""
        self._lock = threading.Lock()  # one reconcile at a time
        self._gpu = []
        self._load_failed = None  # (model, monotonic time) of the last failed load: retry after 30 s

        clients = ReentrantCallbackGroup()
        self._depth_get = self.create_client(GetParameters, f"/{self.depth_node}/get_parameters", callback_group=clients)
        self._depth_set = self.create_client(SetParameters, f"/{self.depth_node}/set_parameters", callback_group=clients)
        self._det_state = self.create_client(GetState, f"/{self.det_node}/get_state", callback_group=clients)
        self._det_change = self.create_client(ChangeState, f"/{self.det_node}/change_state", callback_group=clients)
        self._det_get = self.create_client(GetParameters, f"/{self.det_node}/get_parameters", callback_group=clients)
        self._det_set = self.create_client(SetParameters, f"/{self.det_node}/set_parameters", callback_group=clients)

        services = ReentrantCallbackGroup()
        self.create_service(SetBool, "vision/depth/enable", lambda q, r: self._set("depth", q, r), callback_group=services)
        self.create_service(SetBool, "vision/detection/enable", lambda q, r: self._set("detection", q, r), callback_group=services)
        self.create_service(SetBool, "vision/detection/accurate", lambda q, r: self._set("accurate", q, r), callback_group=services)

        self._diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.create_timer(2.0, self._reconcile_tick, callback_group=MutuallyExclusiveCallbackGroup())
        self.create_timer(0.25, self._sample_gpu, callback_group=MutuallyExclusiveCallbackGroup())
        self.create_timer(1.0, self._diagnostics, callback_group=MutuallyExclusiveCallbackGroup())
        self.get_logger().info("vision manager: depth %s, detection %s (%s)" % (
            on(self.want["depth"]), on(self.want["detection"]), MODELS[self.want["accurate"]]))

    # ------------------------------------------------------------------ saved choice
    def _load(self):
        want = dict(DEFAULTS)
        try:
            with open(self.state_file) as f:
                saved = json.load(f)
            want.update({k: bool(saved[k]) for k in DEFAULTS if k in saved})
        except FileNotFoundError:
            pass
        except (OSError, ValueError) as e:
            self.get_logger().warn("ignoring %s: %s" % (self.state_file, e))
        return want

    def _save(self):
        try:
            os.makedirs(os.path.dirname(self.state_file), exist_ok=True)
            tmp = self.state_file + ".tmp"
            with open(tmp, "w") as f:
                json.dump(self.want, f, indent=2)
            os.replace(tmp, self.state_file)
        except OSError as e:
            self.get_logger().warn("could not save %s: %s" % (self.state_file, e))

    # ------------------------------------------------------------------ services
    def _set(self, key, request, response):
        with self._lock:
            self.want[key] = bool(request.data)
            self._load_failed = None  # an explicit request always retries
            self._save()
            ok, msg = self._reconcile()
        what = {"depth": "stereo depth", "detection": "object detection", "accurate": "detection model"}[key]
        value = MODELS[self.want["accurate"]] if key == "accurate" else on(self.want[key])
        response.success = ok
        response.message = ("%s: %s" % (what, value)) + ("" if ok else " (%s)" % msg)
        self.get_logger().info(response.message)
        return response

    # ------------------------------------------------------------------ reconcile
    def _reconcile_tick(self):
        if self._lock.acquire(blocking=False):
            try:
                self._reconcile()
            finally:
                self._lock.release()

    def _reconcile(self):
        """Bring both nodes to the wanted state. Returns (ok, message)."""
        errors = []
        # Stereo depth: parameter `enabled`
        enabled = self._get_bool(self._depth_get, "enabled")
        if enabled is None:
            errors.append("%s not reachable" % self.depth_node)
        elif enabled != self.want["depth"]:
            if self._set_param(self._depth_set, "enabled", self.want["depth"]):
                enabled = self.want["depth"]
            else:
                errors.append("could not switch %s" % self.depth_node)
        self.actual["depth"] = enabled

        # Object detector: lifecycle state + model parameter
        state = self._get_state()
        if state is None:
            errors.append("%s not reachable" % self.det_node)
        else:
            model = MODELS[self.want["accurate"]]
            current_model = self._get_string(self._det_get, "model")
            if current_model is not None and current_model != model:
                # Takes effect on the next configure; unload now if it is loaded
                self._set_param(self._det_set, "model", model)
                if state in (State.PRIMARY_STATE_ACTIVE, State.PRIMARY_STATE_INACTIVE):
                    state = self._unload(state)
            if self.want["detection"]:
                failed = self._load_failed
                if failed and failed[0] == model and time.monotonic() - failed[1] < 30.0:
                    errors.append("detector could not load %s (see its diagnostics)" % model)
                elif state == State.PRIMARY_STATE_UNCONFIGURED:
                    if self._change(Transition.TRANSITION_CONFIGURE):
                        state = State.PRIMARY_STATE_INACTIVE
                        self._load_failed = None
                    else:
                        self._load_failed = (model, time.monotonic())
                        errors.append("detector could not load %s (see its diagnostics)" % model)
                if state == State.PRIMARY_STATE_INACTIVE:
                    if self._change(Transition.TRANSITION_ACTIVATE):
                        state = State.PRIMARY_STATE_ACTIVE
            elif state in (State.PRIMARY_STATE_ACTIVE, State.PRIMARY_STATE_INACTIVE):
                state = self._unload(state)
            self.actual["detection"] = state == State.PRIMARY_STATE_ACTIVE
            self.actual["model"] = self._get_string(self._det_get, "model")
        self.last_error = "; ".join(errors)
        return not errors, self.last_error

    def _unload(self, state):
        if state == State.PRIMARY_STATE_ACTIVE and self._change(Transition.TRANSITION_DEACTIVATE):
            state = State.PRIMARY_STATE_INACTIVE
        if state == State.PRIMARY_STATE_INACTIVE and self._change(Transition.TRANSITION_CLEANUP):
            state = State.PRIMARY_STATE_UNCONFIGURED
        return state

    # ------------------------------------------------------------------ service helpers
    def _call(self, client, request, timeout=10.0):
        if not client.wait_for_service(timeout_sec=0.5):
            return None
        future = client.call_async(request)
        end = time.monotonic() + timeout
        while not future.done() and time.monotonic() < end:
            time.sleep(0.02)
        return future.result() if future.done() else None

    def _get_value(self, client, name):
        req = GetParameters.Request()
        req.names = [name]
        res = self._call(client, req, 3.0)
        return res.values[0] if res and res.values else None

    def _get_bool(self, client, name):
        v = self._get_value(client, name)
        return v.bool_value if v is not None and v.type == ParameterType.PARAMETER_BOOL else None

    def _get_string(self, client, name):
        v = self._get_value(client, name)
        return v.string_value if v is not None and v.type == ParameterType.PARAMETER_STRING else None

    def _set_param(self, client, name, value):
        p = Parameter(name=name)
        if isinstance(value, bool):
            p.value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=value)
        else:
            p.value = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=value)
        req = SetParameters.Request()
        req.parameters = [p]
        res = self._call(client, req, 3.0)
        return bool(res and res.results and res.results[0].successful)

    def _get_state(self):
        res = self._call(self._det_state, GetState.Request(), 3.0)
        return res.current_state.id if res else None

    def _change(self, transition_id):
        req = ChangeState.Request()
        req.transition.id = transition_id
        res = self._call(self._det_change, req, 30.0)  # configure loads the engine (a few seconds)
        return bool(res and res.success)

    # ------------------------------------------------------------------ diagnostics
    def _sample_gpu(self):
        try:
            with open(GPU_LOAD) as f:
                self._gpu.append(int(f.read().strip()) / 10.0)
        except (OSError, ValueError):
            pass

    @staticmethod
    def _memory():
        info = {}
        with open("/proc/meminfo") as f:
            for line in f:
                k, v = line.split(":", 1)
                info[k] = int(v.split()[0])
        total = info.get("MemTotal", 0)
        used = total - info.get("MemAvailable", total)
        return used / 1024.0, total / 1024.0

    def _diagnostics(self):
        gpu = sum(self._gpu) / len(self._gpu) if self._gpu else None
        self._gpu = []
        try:
            mem_used, mem_total = self._memory()
        except OSError:
            mem_used = mem_total = 0.0
        st = DiagnosticStatus()
        st.name = "Vision"
        st.hardware_id = "vision_manager"
        if self.last_error:
            st.level = DiagnosticStatus.WARN
            st.message = self.last_error
        else:
            st.level = DiagnosticStatus.OK
            st.message = "depth %s, detection %s (%s)" % (
                on(self.want["depth"]), on(self.want["detection"]), MODELS[self.want["accurate"]])
        st.values = [
            KeyValue(key="depth", value=on(self.want["depth"])),
            KeyValue(key="detection", value=on(self.want["detection"])),
            KeyValue(key="accurate", value="true" if self.want["accurate"] else "false"),
            KeyValue(key="model", value=MODELS[self.want["accurate"]]),
            KeyValue(key="gpu_load_percent", value="-" if gpu is None else "%.0f" % gpu),
            KeyValue(key="memory_used_mb", value="%.0f" % mem_used),
            KeyValue(key="memory_total_mb", value="%.0f" % mem_total),
        ]
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status.append(st)
        self._diag_pub.publish(arr)


def on(v):
    return "on" if v else "off"


def main(args=None):
    rclpy.init(args=args)
    node = VisionManager()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

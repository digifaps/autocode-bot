#!/usr/bin/env python3
"""
ROS2 node for D500 LiDAR control: power on/off via GPIO, optional scan-speed parameter.

- Power: optional GPIO pin (Jetson 40-pin header, BOARD mode) to enable 5V to the LiDAR
  (e.g. via MOSFET or power module). Service std_srvs/SetBool to turn on/off.
- Scan speed: parameter scan_speed_hz (default 10). Applying it at runtime may require
  LD19 protocol support in the driver; currently used as desired value for documentation
  or future driver integration.
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

# Optional: Jetson.GPIO only on Jetson hardware
try:
    import Jetson.GPIO as GPIO

    GPIO_AVAILABLE = True
except (ImportError, OSError):
    GPIO_AVAILABLE = False


class LidarControlNode(Node):
    """Node for LiDAR power (GPIO) and scan-speed parameter."""

    def __init__(self):
        super().__init__("lidar_control_node")

        self.declare_parameter("power_enable_gpio_pin", 0)  # 0 = disabled, use BOARD number
        self.declare_parameter("initial_power_on", True)
        self.declare_parameter("scan_speed_hz", 10.0)

        raw = self.get_parameter("power_enable_gpio_pin").get_parameter_value()
        # Launch passes string from LaunchConfiguration; support int or string
        try:
            if raw.string_value:
                self._power_pin = int(raw.string_value)
            else:
                self._power_pin = int(raw.integer_value)
        except (ValueError, TypeError, AttributeError):
            self._power_pin = 0
        self._initial = self.get_parameter("initial_power_on").get_parameter_value().bool_value
        self._scan_speed_hz = (
            self.get_parameter("scan_speed_hz").get_parameter_value().double_value
        )

        self._gpio_ready = False
        if self._power_pin > 0:
            if not GPIO_AVAILABLE:
                self.get_logger().warn(
                    "Jetson.GPIO not available; LiDAR power control disabled. "
                    "Install on Jetson: sudo pip install Jetson.GPIO"
                )
            else:
                try:
                    GPIO.setmode(GPIO.BOARD)
                    GPIO.setwarnings(False)
                    GPIO.setup(self._power_pin, GPIO.OUT)
                    GPIO.output(self._power_pin, GPIO.HIGH if self._initial else GPIO.LOW)
                    self._gpio_ready = True
                    self.get_logger().info(
                        "LiDAR power control on GPIO (BOARD) pin %d, initial=%s"
                        % (self._power_pin, "ON" if self._initial else "OFF")
                    )
                except Exception as e:
                    self.get_logger().error("Failed to setup GPIO for LiDAR power: %s" % e)
        else:
            self.get_logger().info(
                "LiDAR power control disabled (power_enable_gpio_pin=0). "
                "Set a BOARD pin number to enable."
            )

        self._srv = self.create_service(
            SetBool,
            "lidar/set_power",
            self._handle_set_power,
        )
        self.get_logger().info(
            "lidar_control_node running. scan_speed_hz=%.1f (param only; "
            "runtime change may require driver/protocol support)" % self._scan_speed_hz
        )

    def _handle_set_power(self, request, response):
        """Turn LiDAR power on (True) or off (False) via GPIO."""
        if not self._gpio_ready:
            response.success = True
            response.message = "Power control disabled (no GPIO configured)."
            return response
        try:
            GPIO.output(self._power_pin, GPIO.HIGH if request.data else GPIO.LOW)
            response.success = True
            response.message = "LiDAR power %s" % ("ON" if request.data else "OFF")
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def destroy_node(self):
        """Clean up GPIO on shutdown."""
        if self._gpio_ready and GPIO_AVAILABLE:
            try:
                GPIO.output(self._power_pin, GPIO.LOW)
                GPIO.cleanup(self._power_pin)
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LidarControlNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

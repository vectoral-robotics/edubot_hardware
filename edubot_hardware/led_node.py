# edubot_hardware/led_node.py
"""
ROS 2 node for the EduBot corner status LEDs (WS2812B / NeoPixel).

Drives one addressable RGB pixel per robot corner from the Raspberry Pi 5 over
SPI. See ``led_interface`` for the wiring and the reason SPI is used instead of
the PIO path on the Pi 5.

The topic layout is deliberately student-friendly: one topic per corner plus an
``all`` topic, each a plain ``std_msgs/ColorRGBA`` with r/g/b in **0..255**
(not the 0..1 RViz convention). Setting a corner reads almost like a sentence::

    ros2 topic pub /led/front_left std_msgs/ColorRGBA "{r: 255, g: 0, b: 0}"

Topics (subscriptions), with the default (empty) namespace:
  - ``/led/front_left``, ``/led/front_right``, ``/led/rear_left``,
    ``/led/rear_right`` -- set that one corner (names come from ``corner_names``).
  - ``/led/all`` -- set every corner to the same colour.

Values are clamped to 0..255, so an out-of-range publish can never crash the
node. Corner order follows the physical chain (pixel 0 = first LED after MOSI).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import ColorRGBA

from .led_interface import clamp_color


class LedNode(Node):
    """ROS 2 node managing the EduBot corner NeoPixels (real SPI or null backend)."""

    def __init__(self):
        super().__init__("led_node")
        self.get_logger().info("EduBot LED Node starting up...")

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter("use_sim", False)
        self.declare_parameter("num_pixels", 4)
        self.declare_parameter("brightness", 0.4)
        self.declare_parameter("pixel_order", "GRB")
        self.declare_parameter(
            "corner_names", ["front_left", "front_right", "rear_left", "rear_right"]
        )
        # Colour applied on startup as [r, g, b] in 0..255. All zeros = off.
        self.declare_parameter("startup_color", [0, 0, 0])
        self.declare_parameter("clear_on_shutdown", True)

        use_sim = bool(self.get_parameter("use_sim").value)
        self._num_pixels = int(self.get_parameter("num_pixels").value)
        brightness = float(self.get_parameter("brightness").value)
        pixel_order = str(self.get_parameter("pixel_order").value)
        self._corner_names = list(self.get_parameter("corner_names").value)
        self._clear_on_shutdown = bool(self.get_parameter("clear_on_shutdown").value)

        # One corner name per pixel. If they disagree, trust num_pixels and pad
        # or trim the names so the mapping below is always well defined.
        if len(self._corner_names) != self._num_pixels:
            self.get_logger().warn(
                f"corner_names has {len(self._corner_names)} entries but num_pixels="
                f"{self._num_pixels}; adjusting to match."
            )
            self._corner_names = (
                self._corner_names + [f"corner_{i}" for i in range(self._num_pixels)]
            )[: self._num_pixels]

        # Current colour of every pixel; the node owns this state and pushes the
        # full array to the backend on each change.
        self._colors = [(0, 0, 0)] * self._num_pixels

        # ------------------------------------------------------------------
        # Backend selection (real SPI <-> null), mirroring the motor path.
        # Fall back to the null backend if hardware / library is unavailable
        # so the node never brings the stack down on a dev laptop or in sim.
        # ------------------------------------------------------------------
        self.backend = self._make_backend(use_sim, brightness, pixel_order)

        # ------------------------------------------------------------------
        # ROS interfaces: one topic per corner + an "all" topic.
        # ------------------------------------------------------------------
        # Latched-style QoS: a late-joining publisher still gets the last set.
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._subs = []
        for index, name in enumerate(self._corner_names):
            # Absolute names so they resolve to /led/<corner> regardless of the
            # node name (intuitive for students and the web dashboard).
            self._subs.append(
                self.create_subscription(
                    ColorRGBA,
                    f"/led/{name}",
                    self._make_corner_cb(index),
                    qos,
                )
            )
        self._subs.append(self.create_subscription(ColorRGBA, "/led/all", self._all_cb, qos))

        # Apply the configured startup colour to every corner.
        startup = list(self.get_parameter("startup_color").value)
        if len(startup) == 3 and any(startup):
            self._set_all(clamp_color(*startup))

        self.get_logger().info(
            f"LED Node ready: {self._num_pixels} corners "
            f"({', '.join(f'/led/{n}' for n in self._corner_names)}, /led/all)"
        )

    # ------------------------------------------------------------------
    def _make_backend(self, use_sim: bool, brightness: float, pixel_order: str):
        from .led_interface import NullLEDBackend

        if use_sim:
            return NullLEDBackend(self._num_pixels, logger=self.get_logger(), reason="use_sim")

        from .led_interface import NeoPixelSPIBackend

        try:
            return NeoPixelSPIBackend(
                self._num_pixels,
                brightness=brightness,
                pixel_order=pixel_order,
                logger=self.get_logger(),
            )
        except Exception as e:  # any import / SPI failure -> safe null fallback
            self.get_logger().warn(
                f"NeoPixel SPI backend unavailable ({e}); falling back to null backend. "
                "Check 'dtparam=spi=on' in /boot/firmware/config.txt and that "
                "adafruit-circuitpython-neopixel-spi is installed."
            )
            return NullLEDBackend(
                self._num_pixels, logger=self.get_logger(), reason="hardware unavailable"
            )

    # ------------------------------------------------------------------
    def _make_corner_cb(self, index: int):
        def _cb(msg: ColorRGBA) -> None:
            self._colors[index] = clamp_color(msg.r, msg.g, msg.b)
            self.backend.set_pixels(self._colors)

        return _cb

    # ------------------------------------------------------------------
    def _all_cb(self, msg: ColorRGBA) -> None:
        self._set_all(clamp_color(msg.r, msg.g, msg.b))

    # ------------------------------------------------------------------
    def _set_all(self, color) -> None:
        self._colors = [color] * self._num_pixels
        self.backend.set_pixels(self._colors)

    # ------------------------------------------------------------------
    def destroy_node(self) -> bool:
        if self._clear_on_shutdown:
            try:
                self.backend.clear()
            except Exception as e:  # best effort on shutdown
                self.get_logger().warn(f"Failed to clear LEDs on shutdown: {e}")
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

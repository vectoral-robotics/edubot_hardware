# edubot_hardware/led_interface.py
"""
LED interface for the EduBot corner NeoPixels (WS2812B).

The pixels hang off the Raspberry Pi 5 directly (not the ESP32). On the Pi 5 /
Ubuntu 24.04 the usual PIO-based ``adafruit-circuitpython-neopixel`` path fails
(``Failed to open PIO device`` / no ``/dev/pio0``), so we drive the strip over
SPI instead via ``adafruit-circuitpython-neopixel-spi``:

    NeoPixel DIN -> GPIO10 / MOSI (pin 19)
    NeoPixel 5V  -> pin 2
    NeoPixel GND -> pin 6

Requires ``dtparam=spi=on`` in ``/boot/firmware/config.txt`` so that
``/dev/spidev0.0`` exists.

Mirroring the motor path (SerialBridge vs SimulationInterface), this module
exposes two interchangeable backends behind a common ``set_pixels`` / ``clear``
API:

  - ``NeoPixelSPIBackend`` — real hardware over SPI.
  - ``NullLEDBackend``     — no hardware (simulation / dev laptop / library or
                             device unavailable); keeps state and logs.

The pure colour helpers below carry no ROS / hardware imports so they can be
unit tested on any machine.
"""

from __future__ import annotations

Color = tuple[int, int, int]


# ----------------------------------------------------------------------------
# Pure colour helpers (no ROS, no hardware — unit tested)
# ----------------------------------------------------------------------------
# Colours are plain 0..255 RGB (what a std_msgs/ColorRGBA carries in this
# stack — r/g/b as 0..255, not the 0..1 RViz convention, because 0..255 is more
# intuitive for the students this robot targets). Out-of-range values are
# clamped rather than rejected so a beginner can never crash the node.
def clamp8(value: float) -> int:
    """Clamp a number to a single 0..255 byte (rounded)."""
    return max(0, min(255, round(value)))


def clamp_color(r: float, g: float, b: float) -> Color:
    """Clamp an (r, g, b) triple (0..255 domain) to a byte RGB tuple."""
    return (clamp8(r), clamp8(g), clamp8(b))


# ----------------------------------------------------------------------------
# Backends
# ----------------------------------------------------------------------------
class NullLEDBackend:
    """
    Hardware-free LED backend. Used in simulation, on a dev laptop, or as a
    graceful fallback when the SPI device / adafruit library is unavailable.

    Keeps the last commanded colours in memory and logs changes so the rest of
    the stack behaves identically to the real thing.
    """

    def __init__(self, num_pixels: int, logger=None, reason: str = "simulation"):
        self.num_pixels = int(num_pixels)
        self.logger = logger
        self._state: list[Color] = [(0, 0, 0)] * self.num_pixels
        self._log_info(f"NullLEDBackend active ({reason}); {self.num_pixels} virtual pixels")

    def set_pixels(self, colors: list[Color]) -> None:
        for i, c in enumerate(colors[: self.num_pixels]):
            self._state[i] = (clamp8(c[0]), clamp8(c[1]), clamp8(c[2]))
        self._log_debug(f"LED state -> {self._state}")

    def clear(self) -> None:
        self._state = [(0, 0, 0)] * self.num_pixels
        self._log_debug("LED cleared")

    @property
    def state(self) -> list[Color]:
        return list(self._state)

    def _log_info(self, msg: str) -> None:
        if self.logger:
            self.logger.info(msg)

    def _log_debug(self, msg: str) -> None:
        if self.logger:
            self.logger.debug(msg)


class NeoPixelSPIBackend:
    """
    Real NeoPixel backend driving a WS2812B chain over SPI (MOSI / GPIO10).

    Importing ``board`` / ``busio`` / ``neopixel_spi`` and opening the SPI bus
    happens in ``__init__``; a failure here (wrong platform, SPI disabled,
    missing library) raises so the node can fall back to ``NullLEDBackend``.
    """

    def __init__(
        self,
        num_pixels: int,
        brightness: float = 0.4,
        pixel_order: str = "GRB",
        logger=None,
    ):
        self.num_pixels = int(num_pixels)
        self.logger = logger

        # Imported lazily so this module stays importable (and testable) on
        # machines without Adafruit-Blinka / SPI hardware.
        import board
        import busio
        import neopixel_spi

        spi = busio.SPI(board.SCLK, MOSI=board.MOSI)
        order = getattr(neopixel_spi, pixel_order.upper())
        self._pixels = neopixel_spi.NeoPixel_SPI(
            spi,
            self.num_pixels,
            brightness=float(brightness),
            auto_write=False,
            pixel_order=order,
        )
        self.clear()
        self._log_info(
            f"NeoPixel_SPI ready: {self.num_pixels} pixels, order={pixel_order.upper()}, "
            f"brightness={brightness}"
        )

    def set_pixels(self, colors: list[Color]) -> None:
        for i, c in enumerate(colors[: self.num_pixels]):
            self._pixels[i] = (clamp8(c[0]), clamp8(c[1]), clamp8(c[2]))
        self._pixels.show()

    def clear(self) -> None:
        self._pixels.fill((0, 0, 0))
        self._pixels.show()

    def _log_info(self, msg: str) -> None:
        if self.logger:
            self.logger.info(msg)

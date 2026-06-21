"""Unit tests for the serial bridge that do not require a physical device.

Opening a non-existent port leaves the bridge gracefully disconnected, which
lets us exercise the parser and the no-connection guards without hardware.
"""

from edubot_hardware.serial_interface import SerialBridge

BAD_PORT = "/dev/this-port-does-not-exist-edubot"


def make_bridge() -> SerialBridge:
    # No device present -> SerialException is caught, self.ser stays None.
    return SerialBridge(port=BAD_PORT, baud=115200)


def test_missing_device_is_not_connected():
    assert make_bridge().is_connected() is False


def test_guards_when_disconnected_do_not_raise():
    bridge = make_bridge()
    assert bridge.read_lines() == []
    bridge.send_motor_speeds(1.0, 2.0, 3.0, 4.0)  # must be a safe no-op
    bridge.close()  # must not raise


def test_parse_valid_encoder_line():
    bridge = make_bridge()
    assert bridge.parse_encoder_line("E 7 123456 10 20 30 40") == (7, 123456, 10, 20, 30, 40)


def test_parse_rejects_wrong_prefix():
    assert make_bridge().parse_encoder_line("M 1 2 3 4 5 6") is None


def test_parse_rejects_wrong_field_count():
    assert make_bridge().parse_encoder_line("E 1 2 3") is None


def test_parse_rejects_non_integer_fields():
    assert make_bridge().parse_encoder_line("E 1 2 a b c d") is None

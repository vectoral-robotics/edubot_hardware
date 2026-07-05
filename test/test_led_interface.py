"""Unit tests for the pure LED colour helpers (no ROS, no hardware)."""

from edubot_hardware.led_interface import (
    NullLEDBackend,
    clamp8,
    clamp_color,
)


def test_clamp8_rounds_and_bounds():
    assert clamp8(-5) == 0
    assert clamp8(0) == 0
    assert clamp8(127.4) == 127
    assert clamp8(127.6) == 128
    assert clamp8(255) == 255
    assert clamp8(999) == 255


def test_clamp_color_passes_through_in_range():
    assert clamp_color(0, 0, 0) == (0, 0, 0)
    assert clamp_color(255, 128, 0) == (255, 128, 0)


def test_clamp_color_clamps_and_rounds():
    # out of range on both ends, plus a fractional value that rounds up
    assert clamp_color(300, -1, 127.6) == (255, 0, 128)


def test_null_backend_tracks_state_and_clears():
    backend = NullLEDBackend(4)
    backend.set_pixels([(10, 20, 30)] * 4)
    assert backend.state == [(10, 20, 30)] * 4
    backend.clear()
    assert backend.state == [(0, 0, 0)] * 4


def test_null_backend_clamps_and_ignores_extra_pixels():
    backend = NullLEDBackend(2)
    backend.set_pixels([(300, -1, 128), (1, 2, 3), (9, 9, 9)])
    assert backend.state == [(255, 0, 128), (1, 2, 3)]

"""Unit tests for the pure LED colour helpers (no ROS, no hardware)."""

from edubot_hardware.led_interface import (
    NullLEDBackend,
    breathing_level,
    clamp8,
    clamp_color,
    scale_color,
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


def test_breathing_level_endpoints_and_peak():
    period = 4.0
    # Dark at the start and after a full period, brightest at half a period.
    assert breathing_level(0.0, period) == 0.0
    assert breathing_level(period, period) < 1e-6
    assert abs(breathing_level(period / 2.0, period) - 1.0) < 1e-9


def test_breathing_level_stays_in_unit_range():
    period = 4.0
    for i in range(41):
        level = breathing_level(period * i / 40.0, period)
        assert 0.0 <= level <= 1.0


def test_breathing_level_zero_period_is_safe():
    assert breathing_level(1.0, 0.0) == 0.0


def test_scale_color():
    assert scale_color((200, 100, 50), 0.0) == (0, 0, 0)
    assert scale_color((200, 100, 50), 1.0) == (200, 100, 50)
    assert scale_color((200, 100, 50), 0.5) == (100, 50, 25)
    # level is clamped to 0..1
    assert scale_color((200, 100, 50), 5.0) == (200, 100, 50)
    assert scale_color((200, 100, 50), -1.0) == (0, 0, 0)


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

"""Unit tests for the simulation backend (pure logic, no ROS, no hardware)."""

import math

import pytest

import edubot_hardware.simulation_interface as si
from edubot_hardware.simulation_interface import SimulationInterface

TICKS_PER_REV = 4320
WHEEL_RADIUS = 0.04


@pytest.fixture
def sim(monkeypatch) -> SimulationInterface:
    """A simulator with a controllable clock so timing is deterministic."""
    clock = {"now": 1000.0}
    monkeypatch.setattr(si.time, "time", lambda: clock["now"])
    backend = SimulationInterface(ticks_per_rev=TICKS_PER_REV, wheel_radius=WHEEL_RADIUS)
    backend._clock = clock  # test handle to advance time
    return backend


def test_is_connected_is_always_true(sim: SimulationInterface):
    assert sim.is_connected() is True


def test_no_output_before_minimum_interval(sim: SimulationInterface):
    sim._clock["now"] += 0.01  # below the ~50 Hz (0.02 s) threshold
    assert sim.read_lines() == []


def test_one_rev_per_second_yields_ticks_per_rev(sim: SimulationInterface):
    sim.send_motor_speeds(*([2 * math.pi] * 4))  # 1 rev/s on every wheel
    sim._clock["now"] += 1.0
    lines = sim.read_lines()

    assert len(lines) == 1
    seq, ts_us, t_rr, t_fr, t_rl, t_fl = sim.parse_encoder_line(lines[0])
    assert seq == 1
    assert ts_us == int(1001.0 * 1e6)
    for ticks in (t_rr, t_fr, t_rl, t_fl):
        assert ticks == pytest.approx(TICKS_PER_REV, abs=1)


def test_ticks_accumulate_across_reads(sim: SimulationInterface):
    sim.send_motor_speeds(*([2 * math.pi] * 4))
    sim._clock["now"] += 1.0
    sim.read_lines()
    sim._clock["now"] += 1.0
    _, _, t_rr, *_ = sim.parse_encoder_line(sim.read_lines()[0])
    assert t_rr == pytest.approx(2 * TICKS_PER_REV, abs=2)


def test_parse_rejects_malformed_lines(sim: SimulationInterface):
    assert sim.parse_encoder_line("garbage") is None
    assert sim.parse_encoder_line("E 1 2 3") is None  # too few fields
    assert sim.parse_encoder_line("X 1 2 3 4 5 6") is None  # wrong prefix


def test_close_is_a_noop(sim: SimulationInterface):
    sim.close()  # must not raise

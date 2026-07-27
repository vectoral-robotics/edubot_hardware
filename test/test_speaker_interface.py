"""Unit tests for the pure TTS helpers and backends (no ROS, no audio)."""

from array import array

import pytest

from edubot_hardware.speaker_interface import (
    aplay_default_command,
    NullTTSBackend,
    PiperTTSBackend,
    TTSUnavailable,
    aplay_command,
    clamp_volume,
    piper_command,
    scale_pcm16,
)


def test_clamp_volume_bounds_and_rounds():
    assert clamp_volume(-5) == 0
    assert clamp_volume(0) == 0
    assert clamp_volume(80) == 80
    assert clamp_volume(100) == 100
    assert clamp_volume(250) == 100
    assert clamp_volume(42.9) == 42  # int() truncates, matching a 0..100 knob


def test_clamp_volume_nan_is_safe():
    assert clamp_volume(float("nan")) == 0


def test_piper_command_argv():
    assert piper_command("/usr/bin/piper", "/voices/de.onnx", "/tmp/out.wav") == [
        "/usr/bin/piper",
        "--model",
        "/voices/de.onnx",
        "--output_file",
        "/tmp/out.wav",
    ]


def test_aplay_command_argv():
    assert aplay_command("/usr/bin/aplay", "plughw:CARD=x,DEV=0", "/tmp/out.wav") == [
        "/usr/bin/aplay",
        "-D",
        "plughw:CARD=x,DEV=0",
        "/tmp/out.wav",
    ]


def test_aplay_default_command_argv():
    assert aplay_default_command("/usr/bin/aplay", "/tmp/out.wav") == [
        "/usr/bin/aplay",
        "/tmp/out.wav",
    ]


def test_scale_pcm16_full_volume_is_identity():
    pcm = array("h", [1000, -1000, 32767, -32768]).tobytes()
    assert scale_pcm16(pcm, 100) == pcm
    assert scale_pcm16(pcm, 150) == pcm  # clamped to 100 -> untouched


def test_scale_pcm16_zero_is_silence():
    pcm = array("h", [1000, -1000, 32767, -32768]).tobytes()
    assert scale_pcm16(pcm, 0) == array("h", [0, 0, 0, 0]).tobytes()


def test_scale_pcm16_halves_samples_and_preserves_length():
    pcm = array("h", [1000, -1000, 32767, -32768]).tobytes()
    out = array("h")
    out.frombytes(scale_pcm16(pcm, 50))
    assert list(out) == [500, -500, 16383, -16384]
    assert len(scale_pcm16(pcm, 50)) == len(pcm)


def test_null_backend_records_and_never_raises():
    backend = NullTTSBackend(reason="test")
    backend.speak("hallo welt", 70)
    assert backend.last_text == "hallo welt"
    assert backend.last_volume == 70


def test_piper_backend_requires_voice_model(tmp_path):
    with pytest.raises(TTSUnavailable, match="voice model"):
        PiperTTSBackend("", "dev", piper_bin="/usr/bin/piper", aplay_bin="/usr/bin/aplay")
    missing = tmp_path / "nope.onnx"
    with pytest.raises(TTSUnavailable, match="voice model"):
        PiperTTSBackend(
            str(missing), "dev", piper_bin="/usr/bin/piper", aplay_bin="/usr/bin/aplay"
        )


def test_piper_backend_requires_binaries(tmp_path):
    model = tmp_path / "voice.onnx"
    model.write_bytes(b"\x00")  # presence is all the constructor checks
    with pytest.raises(TTSUnavailable, match="piper"):
        PiperTTSBackend(str(model), "dev", piper_bin=None, aplay_bin="/usr/bin/aplay")
    with pytest.raises(TTSUnavailable, match="aplay"):
        PiperTTSBackend(str(model), "dev", piper_bin="/usr/bin/piper", aplay_bin=None)


def test_piper_backend_constructs_when_everything_present(tmp_path):
    model = tmp_path / "voice.onnx"
    model.write_bytes(b"\x00")
    backend = PiperTTSBackend(
        str(model), "plughw:CARD=x,DEV=0", piper_bin="/usr/bin/piper", aplay_bin="/usr/bin/aplay"
    )
    assert backend.voice_model == str(model)
    assert backend.alsa_device == "plughw:CARD=x,DEV=0"

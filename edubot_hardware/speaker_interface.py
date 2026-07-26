# edubot_hardware/speaker_interface.py
"""
Speaker / TTS interface for the EduBot speaker bridge.

Speech is synthesized with **Piper** — a fast, local *neural* TTS (a VITS model
exported to ONNX). It runs fully offline on the robot and sounds far more natural
than a formant synthesizer, while still being light enough for a Raspberry Pi.

Mirroring the motor and LED paths (SerialBridge/SimulationInterface,
NeoPixelSPIBackend/NullLEDBackend), this module exposes two interchangeable
backends behind a common ``speak(text, volume)`` API:

  - ``PiperTTSBackend`` — real synthesis via the ``piper`` binary, rendered to a
    WAV and played on a known ALSA device with ``aplay``. We render to a file and
    play it explicitly because Piper's own audio output opens the default ALSA
    device (dmix), which is not configured on the robot's I2S sound card.
  - ``NullTTSBackend`` — no audio (dev laptop, or ``piper``/``aplay``/the voice
    model unavailable); it records and logs the utterance so the rest of the
    stack behaves identically.

The pure helpers (command building, PCM gain, volume clamp) carry no ROS or
subprocess imports so they can be unit tested on any machine.
"""

from __future__ import annotations

import shutil
import subprocess
import tempfile
import wave
from array import array
from pathlib import Path


class TTSUnavailable(RuntimeError):
    """Raised when a real TTS backend cannot be constructed (missing tool/voice)."""


# ----------------------------------------------------------------------------
# Pure helpers (no ROS, no subprocess — unit tested)
# ----------------------------------------------------------------------------
def clamp_volume(volume: float) -> int:
    """Clamp a volume to a 0..100 integer; NaN maps to 0 so a bad publish is safe."""
    if volume != volume:  # NaN is the only value not equal to itself
        return 0
    return max(0, min(100, int(volume)))


def piper_command(piper_bin: str, voice_model: str, output_path: str) -> list[str]:
    """Argv for rendering stdin text to ``output_path`` with the given voice."""
    return [piper_bin, "--model", voice_model, "--output_file", output_path]


def aplay_command(aplay_bin: str, alsa_device: str, wav_path: str) -> list[str]:
    """Argv for playing a WAV on a specific ALSA device."""
    return [aplay_bin, "-D", alsa_device, wav_path]


def scale_pcm16(frames: bytes, volume: int) -> bytes:
    """Scale signed 16-bit PCM samples by ``volume``/100 (0..100), with clipping.

    Piper renders at a fixed amplitude, so this is how the volume topic is
    honoured: 100 leaves the audio untouched, 0 is silence. PCM is assumed
    16-bit little-endian (Piper's output; the robot and dev hosts are
    little-endian), so ``array('h')`` reads it directly.
    """
    factor = clamp_volume(volume) / 100.0
    if factor >= 1.0:
        return frames
    samples = array("h")
    samples.frombytes(frames)
    for i in range(len(samples)):
        scaled = int(samples[i] * factor)
        samples[i] = max(-32768, min(32767, scaled))
    return samples.tobytes()


# ----------------------------------------------------------------------------
# Backends
# ----------------------------------------------------------------------------
class _TTSBackend:
    """Shared optional-logger helpers (``None`` off-robot)."""

    logger = None

    def _log_info(self, msg: str) -> None:
        if self.logger:
            self.logger.info(msg)


class NullTTSBackend(_TTSBackend):
    """Audio-free backend: keeps the last utterance and logs it. Never raises."""

    def __init__(self, logger=None, reason: str = "no audio"):
        self.logger = logger
        self.last_text: str | None = None
        self.last_volume: int | None = None
        self._log_info(f"NullTTSBackend active ({reason}); speech will be logged, not played")

    def speak(self, text: str, volume: int) -> None:
        self.last_text = text
        self.last_volume = clamp_volume(volume)
        self._log_info(f"[silent TTS] ({self.last_volume}) {text}")


class PiperTTSBackend(_TTSBackend):
    """Real Piper backend: synthesize with ``piper`` and play with ``aplay``.

    Validation happens in ``__init__`` (voice model present, ``piper`` and
    ``aplay`` on ``PATH``); a failure raises :class:`TTSUnavailable` so the node
    can fall back to :class:`NullTTSBackend`. ``piper_bin``/``aplay_bin`` can be
    injected for tests.
    """

    def __init__(
        self,
        voice_model: str,
        alsa_device: str,
        *,
        piper_bin: str | None = None,
        aplay_bin: str | None = None,
        logger=None,
    ):
        self.logger = logger
        self.voice_model = str(voice_model)
        self.alsa_device = str(alsa_device)
        self._piper = piper_bin or shutil.which("piper")
        self._aplay = aplay_bin or shutil.which("aplay")

        if not self.voice_model or not Path(self.voice_model).is_file():
            raise TTSUnavailable(f"voice model not found: {self.voice_model or '(unset)'}")
        if not self._piper:
            raise TTSUnavailable("'piper' binary not found on PATH")
        if not self._aplay:
            raise TTSUnavailable("'aplay' binary not found on PATH")

        self._log_info(
            f"PiperTTSBackend ready: voice={self.voice_model}, device={self.alsa_device}"
        )

    def speak(self, text: str, volume: int) -> None:
        with tempfile.NamedTemporaryFile("wb", suffix=".wav", delete=False) as handle:
            wav_path = Path(handle.name)
        try:
            self._render(text, wav_path)
            self._apply_gain(wav_path, volume)
            subprocess.run(aplay_command(self._aplay, self.alsa_device, str(wav_path)), check=True)
        finally:
            wav_path.unlink(missing_ok=True)

    def _render(self, text: str, wav_path: Path) -> None:
        # Piper reads text from stdin; the voice's <model>.onnx.json is picked up
        # automatically from next to the .onnx file.
        subprocess.run(
            piper_command(self._piper, self.voice_model, str(wav_path)),
            input=text,
            text=True,
            check=True,
            capture_output=True,
        )

    def _apply_gain(self, wav_path: Path, volume: int) -> None:
        if clamp_volume(volume) >= 100:
            return
        with wave.open(str(wav_path), "rb") as reader:
            params = reader.getparams()
            frames = reader.readframes(reader.getnframes())
        if params.sampwidth != 2:  # scale_pcm16 only handles 16-bit PCM (Piper's output)
            return
        scaled = scale_pcm16(frames, volume)
        with wave.open(str(wav_path), "wb") as writer:
            writer.setparams(params)
            writer.writeframes(scaled)

# edubot_hardware/speaker_interface.py
"""
Speaker / TTS interface for the EduBot speaker bridge.

Two-tier playback strategy:

1. **Instant pre-recorded phrases** — WAV files generated at Docker build time
   from ``phrases.json`` and stored in ``/opt/piper/phrases/``.  A text message
   is normalised (lowercase, punctuation stripped) and looked up in the phrase
   map.  If a matching WAV exists it is played immediately via ``aplay``
   with zero synthesis latency — ideal for demos.

2. **On-the-fly Piper TTS** — used for any text that has no pre-recorded match.
   The ``piper`` binary synthesises the text to a temp WAV which is then played
   via ``aplay``.  Falls back to the ALSA default device if the configured device
   fails.

Null fallback (``NullTTSBackend``) is used on dev laptops or when both
``piper`` and ``aplay`` are unavailable.
"""

from __future__ import annotations

import re
import shutil
import subprocess
import tempfile
import wave
from array import array
from pathlib import Path

PIPER_TIMEOUT_S = 20
APLAY_TIMEOUT_S = 30

# Default location for pre-recorded phrase WAVs baked into the Docker image.
DEFAULT_PHRASES_DIR = Path("/opt/piper/phrases")

# Silence padding appended to every WAV before playback to prevent the
# ALSA/speaker hardware from cutting off the last syllable with a click/pop.
SILENCE_PADDING_MS = 700
FADE_IN_MS = 25
FADE_OUT_MS = 120


class TTSUnavailable(RuntimeError):
    """Raised when a real TTS backend cannot be constructed (missing tool/voice)."""


# ----------------------------------------------------------------------------
# Pure helpers (no ROS, no subprocess — unit tested)
# ----------------------------------------------------------------------------


def normalize_phrase_key(text: str) -> str:
    """Lowercase + strip punctuation so 'Hello, EduBot!' == 'hello edubot'."""
    return re.sub(r"[^\w\s]", "", text.lower()).strip()


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


def aplay_default_command(aplay_bin: str, wav_path: str) -> list[str]:
    """Argv for playing a WAV on ALSA's current default device."""
    return [aplay_bin, wav_path]


def fade_edges_pcm16(
    frames: bytes,
    nchannels: int,
    fade_in_frames: int,
    fade_out_frames: int,
) -> bytes:
    """Apply linear fade-in and fade-out on signed 16-bit PCM."""
    if nchannels <= 0 or (fade_in_frames <= 0 and fade_out_frames <= 0):
        return frames
    samples = array("h")
    samples.frombytes(frames)
    total_frames = len(samples) // nchannels
    if total_frames <= 0:
        return frames
    # Fade-in
    fade_in_frames = min(max(0, fade_in_frames), total_frames)
    if fade_in_frames > 0:
        denom_in = max(1, fade_in_frames - 1)
        for i in range(fade_in_frames):
            gain = i / denom_in
            base = i * nchannels
            for ch in range(nchannels):
                samples[base + ch] = int(samples[base + ch] * gain)

    # Fade-out
    fade_out_frames = min(max(0, fade_out_frames), total_frames)
    if fade_out_frames > 0:
        denom_out = max(1, fade_out_frames - 1)
        start_frame = total_frames - fade_out_frames
        for i in range(fade_out_frames):
            gain = (fade_out_frames - 1 - i) / denom_out
            base = (start_frame + i) * nchannels
            for ch in range(nchannels):
                samples[base + ch] = int(samples[base + ch] * gain)
    return samples.tobytes()


def _postprocess_wav_for_playback(
    src: Path,
    dst: Path,
    volume: int,
    *,
    fade_in_ms: int = FADE_IN_MS,
    fade_out_ms: int = FADE_OUT_MS,
    silence_padding_ms: int = SILENCE_PADDING_MS,
) -> None:
    """Scale volume, smooth tail, and append silence before playback."""
    with wave.open(str(src), "rb") as r:
        params = r.getparams()
        frames = r.readframes(r.getnframes())

    if params.sampwidth == 2:
        frames = scale_pcm16(frames, volume)
        fade_in_frames = int(params.framerate * fade_in_ms / 1000)
        fade_frames = int(params.framerate * fade_out_ms / 1000)
        frames = fade_edges_pcm16(frames, params.nchannels, fade_in_frames, fade_frames)

    n_silence = (
        int(params.framerate * silence_padding_ms / 1000) * params.nchannels * params.sampwidth
    )
    with wave.open(str(dst), "wb") as w:
        w.setparams(params)
        w.writeframes(frames)
        w.writeframes(b"\x00" * n_silence)


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
        tail_silence_ms: int = SILENCE_PADDING_MS,
        tail_fade_in_ms: int = FADE_IN_MS,
        tail_fade_ms: int = FADE_OUT_MS,
        *,
        piper_bin: str | None = None,
        aplay_bin: str | None = None,
        logger=None,
    ):
        self.logger = logger
        self.voice_model = str(voice_model)
        self.alsa_device = str(alsa_device)
        self.tail_silence_ms = max(0, int(tail_silence_ms))
        self.tail_fade_in_ms = max(0, int(tail_fade_in_ms))
        self.tail_fade_ms = max(0, int(tail_fade_ms))
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
            _postprocess_wav_for_playback(
                wav_path,
                wav_path,
                volume,
                fade_in_ms=self.tail_fade_in_ms,
                fade_out_ms=self.tail_fade_ms,
                silence_padding_ms=self.tail_silence_ms,
            )
            self._play(wav_path)
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
            timeout=PIPER_TIMEOUT_S,
        )

    def _play(self, wav_path: Path) -> None:
        """Play WAV with configured ALSA device, then retry default device."""
        configured = aplay_command(self._aplay, self.alsa_device, str(wav_path))
        try:
            subprocess.run(
                configured,
                check=True,
                capture_output=True,
                text=True,
                timeout=APLAY_TIMEOUT_S,
            )
            return
        except subprocess.CalledProcessError as exc:
            err = (exc.stderr or exc.stdout or "").strip()
            self._log_info(
                f"aplay on '{self.alsa_device}' failed (rc={exc.returncode}); retrying default device"
                + (f": {err}" if err else "")
            )
        except subprocess.TimeoutExpired:
            self._log_info(
                f"aplay on '{self.alsa_device}' timed out after {APLAY_TIMEOUT_S}s; retrying default device"
            )

        fallback = aplay_default_command(self._aplay, str(wav_path))
        subprocess.run(fallback, check=True, timeout=APLAY_TIMEOUT_S)


# ---------------------------------------------------------------------------
# Phrase library: instant pre-recorded playback
# ---------------------------------------------------------------------------


class PhraseLibrary:
    """Maps normalised phrase text to pre-generated WAV files.

    WAV files are stored as ``<phrases_dir>/<key>.wav`` where ``<key>`` is the
    snake_case identifier from ``phrases.json``.  The library builds a lookup
    dict from *normalised text → wav path* so publishing the exact phrase text
    (case-insensitive, punctuation-tolerant) plays the file instantly.
    """

    def __init__(
        self,
        phrases_dir: Path,
        phrase_map: dict[str, str],  # key → phrase text
        alsa_device: str,
        tail_silence_ms: int = SILENCE_PADDING_MS,
        tail_fade_in_ms: int = FADE_IN_MS,
        tail_fade_ms: int = FADE_OUT_MS,
        *,
        aplay_bin: str | None = None,
        logger=None,
    ):
        self.logger = logger
        self._aplay = aplay_bin or shutil.which("aplay")
        self.alsa_device = alsa_device
        self.tail_silence_ms = max(0, int(tail_silence_ms))
        self.tail_fade_in_ms = max(0, int(tail_fade_in_ms))
        self.tail_fade_ms = max(0, int(tail_fade_ms))
        # Build normalised-text → wav-path lookup
        self._lookup: dict[str, Path] = {}
        for key, text in phrase_map.items():
            wav = phrases_dir / f"{key}.wav"
            if wav.is_file():
                self._lookup[normalize_phrase_key(text)] = wav
        if logger:
            logger.info(
                f"PhraseLibrary loaded {len(self._lookup)}/{len(phrase_map)} phrases "
                f"from {phrases_dir}"
            )

    def lookup(self, text: str) -> Path | None:
        """Return WAV path for text, or None if no pre-recorded match."""
        return self._lookup.get(normalize_phrase_key(text))

    def play(self, wav_path: Path, volume: int) -> None:
        """Play pre-recorded WAV with volume + tail smoothing."""
        if not self._aplay:
            return
        with tempfile.NamedTemporaryFile("wb", suffix=".wav", delete=False) as fh:
            prepared_path = Path(fh.name)
        try:
            _postprocess_wav_for_playback(
                wav_path,
                prepared_path,
                volume,
                fade_in_ms=self.tail_fade_in_ms,
                fade_out_ms=self.tail_fade_ms,
                silence_padding_ms=self.tail_silence_ms,
            )
            self._play_file(prepared_path)
        finally:
            prepared_path.unlink(missing_ok=True)

    def _play_file(self, wav_path: Path) -> None:
        if not self._aplay:
            return
        configured = aplay_command(self._aplay, self.alsa_device, str(wav_path))
        try:
            subprocess.run(
                configured,
                check=True,
                capture_output=True,
                text=True,
                timeout=APLAY_TIMEOUT_S,
            )
            return
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired):
            pass
        fallback = aplay_default_command(self._aplay, str(wav_path))
        subprocess.run(fallback, check=True, timeout=APLAY_TIMEOUT_S)

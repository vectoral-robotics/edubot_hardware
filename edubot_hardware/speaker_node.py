"""
ROS 2 node for the EduBot speaker bridge.

Blockly and the web dashboard can publish plain ROS topics to this node:

  - ``/speaker/text``   — ``std_msgs/String`` with the text to speak
  - ``/speaker/volume`` — ``std_msgs/UInt8`` with a 0..100 volume value

Playback is two-tier:

1. **Instant** — if the incoming text matches a pre-recorded phrase (from
   ``phrases.json`` + ``/opt/piper/phrases/``), the cached WAV is played
   immediately via ``aplay`` with zero TTS latency.
2. **On-the-fly Piper** — any text without a pre-recorded match is synthesised
   at runtime.  Ideal for dynamic strings from Blockly.
"""

from __future__ import annotations

import json
from pathlib import Path
from queue import Empty, Queue
from threading import Event, Thread

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String, UInt8

from edubot_hardware.speaker_interface import (
    DEFAULT_PHRASES_DIR,
    NullTTSBackend,
    PhraseLibrary,
    PiperTTSBackend,
    TTSUnavailable,
    clamp_volume,
)

# phrases.json lives next to this file inside the installed package.
_PHRASES_JSON = Path(__file__).parent / "phrases.json"
_DEFAULT_PHRASES_DIR = str(DEFAULT_PHRASES_DIR)
_DEFAULT_VOICE_MODEL = "/opt/piper/voices/en_US-lessac-high.onnx"


class SpeakerNode(Node):
    """ROS 2 node that speaks text received on a topic."""

    def __init__(self):
        # Keep parameter services explicitly enabled for robust ros2 param access.
        super().__init__("speaker_node", start_parameter_services=True)
        self.get_logger().info("EduBot Speaker Node starting up...")

        self.declare_parameter("default_volume", 80)
        self.declare_parameter("alsa_device", "plughw:0")
        self.declare_parameter("voice_model", _DEFAULT_VOICE_MODEL)
        self.declare_parameter("phrases_dir", _DEFAULT_PHRASES_DIR)
        self.declare_parameter("tail_silence_ms", 700)
        self.declare_parameter("tail_fade_in_ms", 25)
        self.declare_parameter("tail_fade_ms", 120)

        self._volume = clamp_volume(int(self.get_parameter("default_volume").value))
        alsa_device = str(self.get_parameter("alsa_device").value)
        voice_model = self._resolve_voice_model(
            str(self.get_parameter("voice_model").value).strip()
        )
        phrases_dir = Path(str(self.get_parameter("phrases_dir").value).strip())
        tail_silence_ms = max(0, int(self.get_parameter("tail_silence_ms").value))
        tail_fade_in_ms = max(0, int(self.get_parameter("tail_fade_in_ms").value))
        tail_fade_ms = max(0, int(self.get_parameter("tail_fade_ms").value))

        self._phrase_lib = self._build_phrase_library(
            phrases_dir,
            alsa_device,
            tail_silence_ms,
            tail_fade_in_ms,
            tail_fade_ms,
        )
        self._tts = self._build_backend(
            voice_model,
            alsa_device,
            tail_silence_ms,
            tail_fade_in_ms,
            tail_fade_ms,
        )
        self.get_logger().info(f"Speaker backend: {type(self._tts).__name__}")

        self._speak_queue: Queue[tuple[str, int]] = Queue(maxsize=32)
        self._stop_worker = Event()
        self._worker = Thread(target=self._speak_worker, name="speaker-tts-worker", daemon=True)
        self._worker.start()

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # Absolute topic names so dashboard publishers reach this node
        # even when bringup runs it inside a namespace.
        self._text_sub = self.create_subscription(String, "/speaker/text", self._on_text, qos)
        self._volume_sub = self.create_subscription(UInt8, "/speaker/volume", self._on_volume, qos)

        self.get_logger().info(
            "Speaker Node ready: listening on /speaker/text and /speaker/volume "
            f"(default_volume={self._volume})"
        )

    # ------------------------------------------------------------------
    def _resolve_voice_model(self, configured_path: str) -> str:
        """Return a usable Piper voice model path from config or known locations."""
        candidates: list[Path] = []
        if configured_path:
            candidates.append(Path(configured_path))
        candidates.append(Path(_DEFAULT_VOICE_MODEL))

        # Also try the old alba voice as last known fallback.
        candidates.append(Path("/opt/piper/voices/en_GB-alba-medium.onnx"))

        for candidate in candidates:
            if candidate.is_file():
                if configured_path and str(candidate) != configured_path:
                    self.get_logger().warn(
                        f"Configured voice model not found ({configured_path}); "
                        f"using {candidate}"
                    )
                return str(candidate)

        # Last resort: pick any .onnx in /opt/piper/voices/
        voices_dir = Path("/opt/piper/voices")
        if voices_dir.is_dir():
            matches = sorted(voices_dir.glob("*.onnx"))
            if matches:
                selected = matches[0]
                self.get_logger().warn(
                    f"Configured voice model not found ({configured_path or 'unset'}); "
                    f"using discovered model {selected}"
                )
                return str(selected)

        return configured_path

    def _build_phrase_library(
        self,
        phrases_dir: Path,
        alsa_device: str,
        tail_silence_ms: int,
        tail_fade_in_ms: int,
        tail_fade_ms: int,
    ) -> PhraseLibrary | None:
        """Load pre-recorded phrases if the WAV directory and JSON exist."""
        if not _PHRASES_JSON.is_file():
            self.get_logger().warn(
                f"phrases.json not found at {_PHRASES_JSON}; no instant phrases"
            )
            return None
        try:
            phrase_map: dict[str, str] = json.loads(_PHRASES_JSON.read_text())
        except Exception as exc:
            self.get_logger().warn(f"Failed to load phrases.json: {exc}")
            return None

        if not phrases_dir.is_dir():
            self.get_logger().warn(
                f"Phrases directory {phrases_dir} not found; "
                "run generate_phrases.py to create pre-recorded WAVs"
            )
            return None

        return PhraseLibrary(
            phrases_dir,
            phrase_map,
            alsa_device,
            tail_silence_ms=tail_silence_ms,
            tail_fade_in_ms=tail_fade_in_ms,
            tail_fade_ms=tail_fade_ms,
            logger=self.get_logger(),
        )

    def _build_backend(
        self,
        voice_model: str,
        alsa_device: str,
        tail_silence_ms: int,
        tail_fade_in_ms: int,
        tail_fade_ms: int,
    ):
        """Use Piper when available; otherwise degrade to a logging-only backend."""
        try:
            return PiperTTSBackend(
                voice_model,
                alsa_device,
                tail_silence_ms=tail_silence_ms,
                tail_fade_in_ms=tail_fade_in_ms,
                tail_fade_ms=tail_fade_ms,
                logger=self.get_logger(),
            )
        except TTSUnavailable as exc:
            self.get_logger().warn(f"Piper TTS unavailable ({exc}); speech will not be played.")
            return NullTTSBackend(logger=self.get_logger(), reason=str(exc))

    # ------------------------------------------------------------------
    def _on_volume(self, msg: UInt8) -> None:
        self._volume = clamp_volume(msg.data)
        self.get_logger().info(f"Speaker volume set to {self._volume}")

    def _on_text(self, msg: String) -> None:
        text = msg.data.strip()
        if not text:
            return
        try:
            self._speak_queue.put_nowait((text, self._volume))
        except Exception:
            self.get_logger().warn("Speaker queue is full; dropping utterance")

    def _speak_worker(self) -> None:
        while not self._stop_worker.is_set():
            try:
                text, volume = self._speak_queue.get(timeout=0.2)
            except Empty:
                continue
            try:
                # Tier 1: instant pre-recorded phrase
                if self._phrase_lib is not None:
                    wav = self._phrase_lib.lookup(text)
                    if wav is not None:
                        self.get_logger().info(f"[phrase] ({volume}) {text}")
                        self._phrase_lib.play(wav, volume)
                        continue
                # Tier 2: on-the-fly Piper synthesis
                self._tts.speak(text, volume)
            except Exception as exc:
                self.get_logger().error(f"Failed to speak text: {exc}")

    def destroy_node(self):
        self._stop_worker.set()
        if hasattr(self, "_worker") and self._worker.is_alive():
            self._worker.join(timeout=1.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SpeakerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

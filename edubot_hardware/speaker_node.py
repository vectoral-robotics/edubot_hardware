"""
ROS 2 node for the EduBot speaker bridge.

Blockly and the web dashboard can publish plain ROS topics to this node:

  - ``speaker/text``   — ``std_msgs/String`` with the text to speak
  - ``speaker/volume`` — ``std_msgs/UInt8`` with a 0..100 volume value

The node is a thin ROS wrapper; the synthesis and playback live in
``speaker_interface`` (Piper neural TTS, with a hardware-free fallback), which
is unit tested without ROS.
"""

from __future__ import annotations

from queue import Empty, Queue
from threading import Event, Thread

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String, UInt8

from edubot_hardware.speaker_interface import (
    NullTTSBackend,
    PiperTTSBackend,
    TTSUnavailable,
    clamp_volume,
)


class SpeakerNode(Node):
    """ROS 2 node that speaks text received on a topic."""

    def __init__(self):
        super().__init__("speaker_node")
        self.get_logger().info("EduBot Speaker Node starting up...")

        self.declare_parameter("default_volume", 80)
        self.declare_parameter("alsa_device", "plughw:0")
        # Absolute path to a Piper voice model (.onnx). The matching
        # <model>.onnx.json must sit next to it. Empty -> no audio (Null backend).
        self.declare_parameter("voice_model", "")

        self._volume = clamp_volume(int(self.get_parameter("default_volume").value))
        alsa_device = str(self.get_parameter("alsa_device").value)
        voice_model = str(self.get_parameter("voice_model").value).strip()

        self._tts = self._build_backend(voice_model, alsa_device)
        self._speak_queue: Queue[tuple[str, int]] = Queue(maxsize=32)
        self._stop_worker = Event()
        self._worker = Thread(target=self._speak_worker, name="speaker-tts-worker", daemon=True)
        self._worker.start()

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # Use absolute topic names so dashboard publishers always reach this node,
        # even when bringup runs the node inside a namespace.
        self._text_sub = self.create_subscription(String, "/speaker/text", self._on_text, qos)
        self._volume_sub = self.create_subscription(UInt8, "/speaker/volume", self._on_volume, qos)

        self.get_logger().info(
            "Speaker Node ready: listening on speaker/text and speaker/volume "
            f"(default_volume={self._volume})"
        )

    def _build_backend(self, voice_model: str, alsa_device: str):
        """Use Piper when available; otherwise degrade to a logging-only backend."""
        try:
            return PiperTTSBackend(voice_model, alsa_device, logger=self.get_logger())
        except TTSUnavailable as exc:
            self.get_logger().warn(f"Piper TTS unavailable ({exc}); speech will not be played.")
            return NullTTSBackend(logger=self.get_logger(), reason=str(exc))

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

"""
ROS 2 node for the EduBot speaker bridge.

Blockly and the web dashboard can publish plain ROS topics to this node:

  - ``speaker/text``   — ``std_msgs/String`` with the text to speak
  - ``speaker/volume`` — ``std_msgs/UInt8`` with a 0..100 volume value

The node runs on the robot and uses a local TTS backend so the web side stays
simple and does not need a custom message package.
"""

from __future__ import annotations

import subprocess
import tempfile
from pathlib import Path
import shutil

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String, UInt8


class SpeakerNode(Node):
    """ROS 2 node that speaks text received on a topic."""

    def __init__(self):
        super().__init__("speaker_node")
        self.get_logger().info("EduBot Speaker Node starting up...")

        self.declare_parameter("default_volume", 80)
        self.declare_parameter("alsa_device", "plughw:CARD=sndrpigooglevoi,DEV=0")
        self._default_volume = self._clamp_volume(int(self.get_parameter("default_volume").value))
        self._volume = self._default_volume
        self._alsa_device = str(self.get_parameter("alsa_device").value)
        self._backend = self._find_backend()
        self._aplay = shutil.which("aplay")

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._text_sub = self.create_subscription(String, "speaker/text", self._on_text, qos)
        self._volume_sub = self.create_subscription(UInt8, "speaker/volume", self._on_volume, qos)

        backend_name = self._backend if self._backend else "none"
        self.get_logger().info(
            f"Speaker Node ready: listening on speaker/text and speaker/volume "
            f"(backend={backend_name}, aplay={'yes' if self._aplay else 'no'}, "
            f"alsa_device={self._alsa_device}, default_volume={self._default_volume})"
        )

    def _find_backend(self) -> str | None:
        for executable in ("espeak-ng", "espeak"):
            if shutil.which(executable):
                return executable
        return None

    def _clamp_volume(self, volume: int) -> int:
        return max(0, min(100, int(volume)))

    def _on_volume(self, msg: UInt8) -> None:
        self._volume = self._clamp_volume(msg.data)
        self.get_logger().info(f"Speaker volume set to {self._volume}")

    def _on_text(self, msg: String) -> None:
        text = msg.data.strip()
        if not text:
            return
        if self._backend is None:
            self.get_logger().error("No TTS backend found. Install espeak-ng on the robot image.")
            return

        try:
            self._speak(self._backend, text, self._volume)
        except Exception as exc:
            self.get_logger().error(f"Failed to speak text: {exc}")

    def _speak(self, executable: str, text: str, volume: int) -> None:
        # espeak-ng's own ALSA output uses the default device (dmix), which
        # is not configured on the robot's I2S sound card and fails with
        # "unable to open slave". Render to a WAV file instead and play it
        # explicitly on the known-good ALSA device via aplay.
        with tempfile.NamedTemporaryFile("w", suffix=".txt", delete=False) as handle:
            text_path = Path(handle.name)
            handle.write(text)

        wav_path = text_path.with_suffix(".wav")
        try:
            self.get_logger().info(f"Speaking at volume {volume}: {text}")
            subprocess.run(
                [executable, "-a", str(volume), "-f", str(text_path), "-w", str(wav_path)],
                check=True,
            )

            if self._aplay:
                subprocess.run([self._aplay, "-D", self._alsa_device, str(wav_path)], check=True)
            else:
                self.get_logger().error("aplay not found; cannot play synthesized speech.")
        finally:
            for path in (text_path, wav_path):
                try:
                    path.unlink(missing_ok=True)
                except Exception:
                    pass


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
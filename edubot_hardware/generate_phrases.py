#!/usr/bin/env python3
"""
Generate pre-recorded phrase WAVs for the EduBot speaker node.

Run at Docker build time (after Piper and the voice model are installed):

    ros2 run edubot_hardware generate_phrases \
        --voice-model /opt/piper/voices/en_US-lessac-high.onnx \
        --phrases     /workspace/install/.../site-packages/edubot_hardware/phrases.json \
        --output-dir  /opt/piper/phrases

Each entry in phrases.json is synthesised to <key>.wav inside --output-dir.
Existing WAVs are skipped unless --force is passed.
"""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser(description="Pre-generate speaker phrase WAVs")
    parser.add_argument(
        "--voice-model",
        default="/opt/piper/voices/en_US-lessac-high.onnx",
        help="Piper voice model (.onnx)",
    )
    parser.add_argument(
        "--phrases",
        default=str(Path(__file__).parent / "phrases.json"),
        help="Path to phrases.json",
    )
    parser.add_argument(
        "--output-dir",
        default="/opt/piper/phrases",
        help="Directory where WAVs are written",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="Re-generate even if WAV already exists",
    )
    args = parser.parse_args()

    piper_bin = shutil.which("piper")
    if not piper_bin:
        print("ERROR: 'piper' binary not found on PATH", file=sys.stderr)
        sys.exit(1)

    voice_model = Path(args.voice_model)
    if not voice_model.is_file():
        print(f"ERROR: voice model not found: {voice_model}", file=sys.stderr)
        sys.exit(1)

    phrases_path = Path(args.phrases)
    if not phrases_path.is_file():
        print(f"ERROR: phrases.json not found: {phrases_path}", file=sys.stderr)
        sys.exit(1)

    phrase_map: dict[str, str] = json.loads(phrases_path.read_text())
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    ok = 0
    skipped = 0
    failed = 0

    for key, text in phrase_map.items():
        wav_path = output_dir / f"{key}.wav"
        if wav_path.is_file() and not args.force:
            skipped += 1
            continue

        print(f"  [{key}] {text!r} → {wav_path}")
        try:
            subprocess.run(
                [piper_bin, "--model", str(voice_model), "--output_file", str(wav_path)],
                input=text,
                text=True,
                check=True,
                capture_output=True,
                timeout=30,
            )
            ok += 1
        except Exception as exc:
            print(f"  FAILED: {exc}", file=sys.stderr)
            failed += 1

    total = len(phrase_map)
    print(f"\nDone: {ok} generated, {skipped} skipped, {failed} failed  (total {total})")
    if failed:
        sys.exit(1)


if __name__ == "__main__":
    main()

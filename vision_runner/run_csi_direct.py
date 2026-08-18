#!/usr/bin/env python3
"""Read IMX708 directly on Jetson and run the server-free extractor."""

from __future__ import annotations

import argparse
import json
import os
import select
import signal
import subprocess
import time
from pathlib import Path
from typing import Iterator

import cv2
import numpy as np

from birdseye_extractor import BirdseyeExtractor, ExtractionResult


def gstreamer_command(
    *,
    sensor_id: int,
    sensor_mode: int,
    width: int,
    height: int,
    fps: int,
) -> list[str]:
    return [
        "gst-launch-1.0",
        "-q",
        "-e",
        "nvarguscamerasrc",
        f"sensor-id={sensor_id}",
        f"sensor-mode={sensor_mode}",
        "!",
        (
            "video/x-raw(memory:NVMM),"
            f"width={width},height={height},format=NV12,framerate={fps}/1"
        ),
        "!",
        "nvjpegenc",
        "quality=90",
        "!",
        "multipartmux",
        "boundary=frame",
        "!",
        "fdsink",
        "fd=1",
    ]


def camera_frames(command: list[str]) -> Iterator[np.ndarray]:
    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=None,
        bufsize=0,
        start_new_session=True,
    )
    assert process.stdout is not None
    buffer = bytearray()
    try:
        while process.poll() is None:
            readable, _, _ = select.select([process.stdout], [], [], 2.5)
            if not readable:
                raise TimeoutError("no CSI camera frame for 2.5 seconds")
            chunk = process.stdout.read(64 * 1024)
            if not chunk:
                break
            buffer.extend(chunk)
            while True:
                start = buffer.find(b"\xff\xd8")
                if start < 0:
                    if len(buffer) > 2 * 1024 * 1024:
                        del buffer[:-2]
                    break
                end = buffer.find(b"\xff\xd9", start + 2)
                if end < 0:
                    if start > 0:
                        del buffer[:start]
                    break
                jpeg = bytes(buffer[start : end + 2])
                del buffer[: end + 2]
                frame = cv2.imdecode(
                    np.frombuffer(jpeg, dtype=np.uint8),
                    cv2.IMREAD_COLOR,
                )
                if frame is not None:
                    yield frame
    finally:
        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=3)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass


def save_result(
    output_dir: Path,
    frame_number: int,
    result: ExtractionResult,
) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    prefix = output_dir / f"frame_{frame_number:06d}"
    cv2.imwrite(f"{prefix}_birdseye.jpg", result.birdseye)
    if result.lanes is not None:
        cv2.imwrite(f"{prefix}_overlay.jpg", result.lanes.overlay)
        cv2.imwrite(f"{prefix}_yellow.png", result.lanes.yellow_mask)
        cv2.imwrite(f"{prefix}_dash.png", result.lanes.dash_mask)
        cv2.imwrite(f"{prefix}_stop.png", result.lanes.stop_mask)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--sensor-id", type=int, default=0)
    parser.add_argument("--sensor-mode", type=int, default=1)
    parser.add_argument("--fps", type=int, default=10)
    parser.add_argument(
        "--calibration-dir",
        type=Path,
        default=Path(__file__).resolve().parent / "calibration",
    )
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument(
        "--save-every",
        type=int,
        default=0,
        help="Save every Nth result; 0 disables file output.",
    )
    parser.add_argument(
        "--max-frames",
        type=int,
        default=0,
        help="Stop after N frames; 0 runs until Ctrl+C.",
    )
    parser.add_argument(
        "--no-heuristic-lanes",
        action="store_true",
        help="Produce only the bird's-eye image for a segmentation model.",
    )
    args = parser.parse_args()
    if args.fps <= 0:
        parser.error("--fps must be positive")
    if args.save_every < 0 or args.max_frames < 0:
        parser.error("frame counts must not be negative")
    if args.save_every > 0 and args.output_dir is None:
        parser.error("--output-dir is required when --save-every is used")

    extractor = BirdseyeExtractor(args.calibration_dir)
    command = gstreamer_command(
        sensor_id=args.sensor_id,
        sensor_mode=args.sensor_mode,
        width=extractor.image_width,
        height=extractor.image_height,
        fps=args.fps,
    )
    last_log_at = 0.0
    started_at = time.perf_counter()
    try:
        for frame_number, frame in enumerate(camera_frames(command), start=1):
            process_started = time.perf_counter()
            result = extractor.process(
                frame,
                extract_lanes=not args.no_heuristic_lanes,
            )
            process_ms = (time.perf_counter() - process_started) * 1000.0

            # Segmentation integration point:
            # model_input_bgr = result.birdseye

            if (
                args.output_dir is not None
                and args.save_every > 0
                and frame_number % args.save_every == 0
            ):
                save_result(args.output_dir, frame_number, result)

            now = time.monotonic()
            if now - last_log_at >= 1.0:
                message = {
                    "frame": frame_number,
                    "process_ms": round(process_ms, 1),
                    "birdseye_size": list(extractor.output_size),
                    "metrics": (
                        None
                        if result.lanes is None
                        else result.lanes.metrics
                    ),
                }
                print(json.dumps(message, ensure_ascii=False), flush=True)
                last_log_at = now

            if args.max_frames > 0 and frame_number >= args.max_frames:
                break
    except KeyboardInterrupt:
        pass
    finally:
        elapsed = max(time.perf_counter() - started_at, 1e-9)
        print(f"stopped after {elapsed:.1f}s", flush=True)


if __name__ == "__main__":
    main()

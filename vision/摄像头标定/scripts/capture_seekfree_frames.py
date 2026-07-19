#!/usr/bin/env python3
"""Capture SeekFree Assistant camera frames from a serial port."""

from __future__ import annotations

import argparse
import json
import struct
import sys
import time
from pathlib import Path
from typing import Callable

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:  # pragma: no cover
    raise SystemExit("Missing dependency: pip install pyserial") from exc

try:
    import numpy as np
    from PIL import Image
except ImportError:  # pragma: no cover
    np = None
    Image = None


HEAD = 0xAA
CAMERA_FUNCTION = 0x02
DOT_FUNCTION = 0x03
CAMERA_HEADER_LEN = 8
PROJECT_ROOT = Path(__file__).resolve().parents[1]


def list_serial_ports() -> list[str]:
    ports = []
    for item in list_ports.comports():
        if item.device:
            ports.append(item.device)
    return ports


def image_payload_size(image_type: int, width: int, height: int) -> int:
    if image_type == 1:
        return (width * height + 7) // 8
    if image_type == 2:
        return width * height
    if image_type == 3:
        return width * height * 2
    raise ValueError(f"unsupported image type: {image_type}")


def read_exact(port: serial.Serial, size: int) -> bytes:
    data = bytearray()
    deadline = time.monotonic() + max(port.timeout or 0.0, 0.1) + 2.0
    while len(data) < size:
        chunk = port.read(size - len(data))
        if chunk:
            data.extend(chunk)
            deadline = time.monotonic() + max(port.timeout or 0.0, 0.1) + 2.0
            continue
        if time.monotonic() > deadline:
            raise TimeoutError(f"timeout while reading {size} bytes, got {len(data)}")
    return bytes(data)


def find_camera_header(port: serial.Serial) -> tuple[int, int, int, int, int]:
    while True:
        byte = port.read(1)
        if not byte:
            raise TimeoutError("timeout waiting for frame head 0xAA")
        if byte[0] != HEAD:
            continue

        rest = read_exact(port, CAMERA_HEADER_LEN - 1)
        function = rest[0]
        if function != CAMERA_FUNCTION:
            continue

        packed_type = rest[1]
        header_len = rest[2]
        width, height = struct.unpack_from("<HH", rest, 3)
        if header_len != CAMERA_HEADER_LEN or width == 0 or height == 0:
            continue

        image_type = (packed_type >> 5) & 0x07
        no_image = (packed_type >> 4) & 0x01
        boundary_num = packed_type & 0x0F
        return image_type, no_image, boundary_num, width, height


def payload_to_grayscale(payload: bytes, image_type: int, width: int, height: int) -> bytes | None:
    if image_type == 2:
        return payload

    if image_type == 1:
        bits = []
        for value in payload:
            for bit in range(7, -1, -1):
                bits.append(255 if (value >> bit) & 1 else 0)
        return bytes(bits[: width * height])

    return None


def save_image(payload: bytes, image_type: int, width: int, height: int, out_path: Path) -> Path | None:
    gray = payload_to_grayscale(payload, image_type, width, height)
    if gray is None:
        return None

    if np is None or Image is None:
        pgm = out_path.with_suffix(".pgm")
        pgm.write_bytes(f"P5\n{width} {height}\n255\n".encode("ascii") + gray)
        return pgm

    image = np.frombuffer(gray, dtype=np.uint8).reshape((height, width))
    Image.fromarray(image, mode="L").save(out_path)
    return out_path


def capture_single_frame(port: serial.Serial) -> dict:
    image_type, no_image, boundary_num, width, height = find_camera_header(port)
    if no_image:
        return {
            "skipped": True,
            "boundary_num": boundary_num,
            "image_type": image_type,
            "width": width,
            "height": height,
        }

    size = image_payload_size(image_type, width, height)
    payload = read_exact(port, size)
    return {
        "skipped": False,
        "boundary_num": boundary_num,
        "image_type": image_type,
        "width": width,
        "height": height,
        "payload_size": size,
        "payload": payload,
    }


def save_frame(frame: dict, out_dir: Path, prefix: str, frame_index: int) -> dict:
    stem = out_dir / f"{prefix}_{frame_index:04d}_{frame['width']}x{frame['height']}_type{frame['image_type']}"
    raw_path = stem.with_suffix(".raw")
    image_path = stem.with_suffix(".png")
    json_path = stem.with_suffix(".json")

    raw_path.write_bytes(frame["payload"])
    saved_image_path = save_image(frame["payload"], frame["image_type"], frame["width"], frame["height"], image_path)
    json_path.write_text(
        json.dumps(
            {
                "image_type": frame["image_type"],
                "width": frame["width"],
                "height": frame["height"],
                "payload_size": frame["payload_size"],
                "boundary_num": frame["boundary_num"],
            },
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )

    result = dict(frame)
    result.update(
        {
            "index": frame_index,
            "raw_path": raw_path,
            "image_path": saved_image_path,
            "json_path": json_path,
        }
    )
    return result


def capture_frames(
    port_name: str,
    baud: int = 115200,
    count: int = 10,
    out: Path | str = Path("../frames/uart_capture"),
    prefix: str = "frame",
    timeout: float = 1.0,
    on_frame: Callable[[dict], None] | None = None,
    on_log: Callable[[str], None] | None = None,
    should_stop: Callable[[], bool] | None = None,
) -> list[dict]:
    out_dir = Path(out)
    out_dir.mkdir(parents=True, exist_ok=True)

    if on_log is None:
        on_log = print

    saved_frames: list[dict] = []
    with serial.Serial(port_name, baud, timeout=timeout) as port:
        on_log(f"opened {port_name} @ {baud}")
        while len(saved_frames) < count:
            if should_stop and should_stop():
                on_log("capture stopped by user")
                break

            frame = capture_single_frame(port)
            if frame["skipped"]:
                on_log(f"skip frame without image, boundary_num={frame['boundary_num']}")
                continue

            saved = save_frame(frame, out_dir, prefix, len(saved_frames))
            saved_frames.append(saved)
            on_log(f"saved {saved['raw_path'].name} ({saved['payload_size']} bytes)")
            if on_frame is not None:
                on_frame(saved)

    return saved_frames


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True, help="Serial port, for example COM7")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--count", type=int, default=10)
    parser.add_argument("--out", type=Path, default=PROJECT_ROOT / "frames" / "uart_capture")
    parser.add_argument("--prefix", default="frame")
    parser.add_argument("--timeout", type=float, default=1.0)
    args = parser.parse_args()

    capture_frames(
        port_name=args.port,
        baud=args.baud,
        count=args.count,
        out=args.out,
        prefix=args.prefix,
        timeout=args.timeout,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())

"""SeekFree Assistant camera frame capture for the vision simulator UI."""

from __future__ import annotations

import json
import struct
import time
from pathlib import Path
from typing import Callable

from PIL import Image

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover - handled by the UI at runtime
    serial = None
    list_ports = None


HEAD = 0xAA
CAMERA_FUNCTION = 0x02
CAMERA_HEADER_LEN = 8


def list_serial_ports() -> list[str]:
    if list_ports is None:
        return []
    return [item.device for item in list_ports.comports() if item.device]


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
        elif time.monotonic() > deadline:
            raise TimeoutError(f"timeout while reading {size} bytes, got {len(data)}")
    return bytes(data)


def find_camera_header(port: serial.Serial) -> tuple[int, int, int, int, int]:
    while True:
        byte = port.read(1)
        if not byte:
            raise TimeoutError("timeout waiting for camera frame header")
        if byte[0] != HEAD:
            continue
        rest = read_exact(port, CAMERA_HEADER_LEN - 1)
        if rest[0] != CAMERA_FUNCTION or rest[2] != CAMERA_HEADER_LEN:
            continue
        image_type = (rest[1] >> 5) & 0x07
        no_image = (rest[1] >> 4) & 0x01
        boundary_num = rest[1] & 0x0F
        width, height = struct.unpack_from("<HH", rest, 3)
        if width and height:
            return image_type, no_image, boundary_num, width, height


def payload_to_grayscale(payload: bytes, image_type: int, width: int, height: int) -> bytes | None:
    if image_type == 2:
        return payload
    if image_type == 1:
        values = bytearray()
        for value in payload:
            for bit in range(7, -1, -1):
                values.append(255 if (value >> bit) & 1 else 0)
        return bytes(values[: width * height])
    return None


def save_frame(frame: dict, out_dir: Path, prefix: str, index: int) -> dict:
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = out_dir / f"{prefix}_{index:04d}_{frame['width']}x{frame['height']}_type{frame['image_type']}"
    raw_path = stem.with_suffix(".raw")
    image_path = stem.with_suffix(".png")
    json_path = stem.with_suffix(".json")
    raw_path.write_bytes(frame["payload"])
    gray = payload_to_grayscale(frame["payload"], frame["image_type"], frame["width"], frame["height"])
    if gray is not None:
        Image.frombytes("L", (frame["width"], frame["height"]), gray).save(image_path)
    else:
        image_path = None
    json_path.write_text(json.dumps({
        "image_type": frame["image_type"],
        "width": frame["width"],
        "height": frame["height"],
        "payload_size": frame["payload_size"],
        "boundary_num": frame["boundary_num"],
    }, ensure_ascii=True, indent=2), encoding="utf-8")
    return {**frame, "index": index, "raw_path": raw_path, "image_path": image_path, "json_path": json_path}


def capture_single_frame(port: serial.Serial) -> dict:
    image_type, no_image, boundary_num, width, height = find_camera_header(port)
    if no_image:
        return {"skipped": True, "boundary_num": boundary_num, "image_type": image_type, "width": width, "height": height}
    size = image_payload_size(image_type, width, height)
    return {
        "skipped": False,
        "boundary_num": boundary_num,
        "image_type": image_type,
        "width": width,
        "height": height,
        "payload_size": size,
        "payload": read_exact(port, size),
    }


def capture_frames(
    port_name: str,
    baud: int,
    count: int,
    out_dir: Path,
    prefix: str,
    timeout: float,
    on_frame: Callable[[dict], None],
    on_log: Callable[[str], None],
    should_stop: Callable[[], bool],
) -> list[dict]:
    if serial is None:
        raise RuntimeError("pyserial is not installed; run: python -m pip install pyserial")
    frames: list[dict] = []
    with serial.Serial(port_name, baudrate=baud, timeout=timeout) as port:
        on_log(f"opened {port_name} @ {baud}")
        while len(frames) < count and not should_stop():
            frame = capture_single_frame(port)
            if frame["skipped"]:
                on_log(f"skip frame without image, boundary={frame['boundary_num']}")
                continue
            saved = save_frame(frame, out_dir, prefix, len(frames))
            frames.append(saved)
            on_log(f"saved {saved['raw_path'].name}")
            on_frame(saved)
    return frames

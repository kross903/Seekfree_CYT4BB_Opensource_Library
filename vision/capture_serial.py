"""SeekFree Assistant camera frame capture for the vision simulator UI."""

from __future__ import annotations

import json
import struct
import time
from datetime import datetime, timedelta
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
METADATA_FUNCTION = 0x12
METADATA_VERSION_V1 = 0x01
METADATA_VERSION = 0x02
METADATA_PACKET_LEN_V1 = 33
METADATA_PACKET_LEN = 64
METADATA_PACKET_LENGTHS = {
    METADATA_VERSION_V1: METADATA_PACKET_LEN_V1,
    METADATA_VERSION: METADATA_PACKET_LEN,
}


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


def parse_metadata_packet(packet: bytes) -> dict:
    if len(packet) < 4:
        raise ValueError(f"invalid metadata packet length: {len(packet)}")
    if packet[0] != HEAD or packet[1] != METADATA_FUNCTION:
        raise ValueError("invalid metadata packet header")
    version = packet[2]
    expected_length = METADATA_PACKET_LENGTHS.get(version)
    if expected_length is None or packet[3] != expected_length or len(packet) != expected_length:
        raise ValueError("unsupported metadata packet version or size")
    if (sum(packet[:-1]) & 0xFF) != packet[-1]:
        raise ValueError("metadata checksum mismatch")
    image_seq, input_seq, roll, pitch, yaw, tof_height, fused_height = struct.unpack(
        "<IIfffff", packet[4:32]
    )
    metadata = {
        "version": version,
        "image_seq": image_seq,
        "input_seq": input_seq,
        "roll_deg": roll,
        "pitch_deg": pitch,
        "yaw_deg": yaw,
        "tof_height_cm": tof_height,
        "fused_height_cm": fused_height,
    }
    if version == METADATA_VERSION:
        (
            car_yaw,
            car_yaw_valid,
            car_yaw_age_ms,
            mount_roll,
            mount_pitch,
            mount_yaw,
            offset_x,
            offset_y,
            offset_z,
        ) = struct.unpack("<fBHffffff", packet[32:63])
        metadata.update({
            "car_yaw_deg": car_yaw,
            "car_yaw_valid": bool(car_yaw_valid),
            "car_yaw_age_ms": car_yaw_age_ms,
            "camera_mount_roll_deg": mount_roll,
            "camera_mount_pitch_deg": mount_pitch,
            "camera_mount_yaw_deg": mount_yaw,
            "camera_offset_x_cm": offset_x,
            "camera_offset_y_cm": offset_y,
            "camera_offset_z_cm": offset_z,
        })
    return metadata


def find_camera_packet(port: serial.Serial) -> tuple[dict | None, int, int, int, int, int]:
    metadata = None
    while True:
        byte = port.read(1)
        if not byte:
            raise TimeoutError("timeout waiting for camera frame header")
        if byte[0] != HEAD:
            continue

        prefix = read_exact(port, 3)
        function = prefix[0]
        if function == METADATA_FUNCTION:
            declared_length = prefix[2]
            if declared_length < 4:
                continue
            packet = bytes((HEAD, prefix[0], prefix[1], prefix[2])) + read_exact(
                port, declared_length - 4
            )
            try:
                metadata = parse_metadata_packet(packet)
            except ValueError:
                metadata = None
            continue

        if function != CAMERA_FUNCTION or prefix[2] != CAMERA_HEADER_LEN:
            continue

        camera_type = prefix[1]
        image_type = (camera_type >> 5) & 0x07
        no_image = (camera_type >> 4) & 0x01
        boundary_num = camera_type & 0x0F
        width, height = struct.unpack("<HH", read_exact(port, 4))
        if width and height:
            return metadata, image_type, no_image, boundary_num, width, height


def find_camera_header(port: serial.Serial) -> tuple[int, int, int, int, int]:
    _metadata, image_type, no_image, boundary_num, width, height = find_camera_packet(port)
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


def save_frame(
    frame: dict,
    out_dir: Path,
    prefix: str,
    index: int,
    timestamp_name: bool = False,
) -> dict:
    out_dir.mkdir(parents=True, exist_ok=True)
    captured_at = datetime.now().astimezone()
    if timestamp_name:
        stem = out_dir / captured_at.strftime("%Y%m%d_%H%M%S_%f")
        while any(stem.with_suffix(suffix).exists() for suffix in (".raw", ".png", ".json")):
            captured_at += timedelta(microseconds=1)
            stem = out_dir / captured_at.strftime("%Y%m%d_%H%M%S_%f")
    else:
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
        "captured_at": captured_at.isoformat(timespec="microseconds"),
        "image_type": frame["image_type"],
        "width": frame["width"],
        "height": frame["height"],
        "payload_size": frame["payload_size"],
        "boundary_num": frame["boundary_num"],
        "telemetry": frame.get("metadata"),
    }, ensure_ascii=True, indent=2), encoding="utf-8")
    return {**frame, "index": index, "raw_path": raw_path, "image_path": image_path, "json_path": json_path}


def capture_single_frame(port: serial.Serial) -> dict:
    metadata, image_type, no_image, boundary_num, width, height = find_camera_packet(port)
    if no_image:
        return {
            "skipped": True,
            "boundary_num": boundary_num,
            "image_type": image_type,
            "width": width,
            "height": height,
            "metadata": metadata,
        }
    size = image_payload_size(image_type, width, height)
    return {
        "skipped": False,
        "boundary_num": boundary_num,
        "image_type": image_type,
        "width": width,
        "height": height,
        "payload_size": size,
        "payload": read_exact(port, size),
        "metadata": metadata,
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
    timestamp_names: bool = False,
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
            saved = save_frame(
                frame,
                out_dir,
                prefix,
                len(frames),
                timestamp_name=timestamp_names,
            )
            frames.append(saved)
            metadata = saved.get("metadata") or {}
            if metadata:
                on_log(
                    f"saved {saved['raw_path'].name} "
                    f"R={metadata['roll_deg']:.2f} P={metadata['pitch_deg']:.2f} "
                    f"Y={metadata['yaw_deg']:.2f} H={metadata['tof_height_cm']:.1f}cm"
                )
            else:
                on_log(f"saved {saved['raw_path'].name} (no telemetry)")
            on_frame(saved)
    return frames

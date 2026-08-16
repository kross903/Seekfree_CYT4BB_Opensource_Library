#!/usr/bin/env python3
"""Image processing primitives for the offline MT9V03 image viewer."""

from __future__ import annotations

from array import array
from dataclasses import dataclass
import math
from pathlib import Path
import re

from PIL import Image


INVALID_Q8 = 0xFFFF


@dataclass(frozen=True)
class CameraCalibration:
    image_width: int
    image_height: int
    output_width: int
    output_height: int
    fx: float
    fy: float
    cx: float
    cy: float
    k1: float
    k2: float
    k3: float
    k4: float

    @classmethod
    def from_header(cls, header_path: Path) -> "CameraCalibration":
        text = header_path.read_text(encoding="utf-8")

        def macro(name: str) -> float:
            match = re.search(
                rf"^\s*#define\s+{re.escape(name)}\s+\(?\s*([-+0-9.eE]+)",
                text,
                re.MULTILINE,
            )
            if not match:
                raise ValueError(f"标定文件缺少宏：{name}")
            return float(match.group(1))

        return cls(
            image_width=int(macro("CAMERA_CALIBRATION_IMAGE_WIDTH")),
            image_height=int(macro("CAMERA_CALIBRATION_IMAGE_HEIGHT")),
            output_width=int(macro("CAMERA_CALIBRATION_UNDISTORT_WIDTH")),
            output_height=int(macro("CAMERA_CALIBRATION_UNDISTORT_HEIGHT")),
            fx=macro("CAMERA_CALIBRATION_FX"),
            fy=macro("CAMERA_CALIBRATION_FY"),
            cx=macro("CAMERA_CALIBRATION_CX"),
            cy=macro("CAMERA_CALIBRATION_CY"),
            k1=macro("CAMERA_CALIBRATION_K1"),
            k2=macro("CAMERA_CALIBRATION_K2"),
            k3=macro("CAMERA_CALIBRATION_K3"),
            k4=macro("CAMERA_CALIBRATION_K4"),
        )


class FisheyeUndistorter:
    """Firmware-equivalent fisheye inverse map with Q8 bilinear sampling."""

    def __init__(self, calibration: CameraCalibration) -> None:
        self.calibration = calibration
        self.map_x_q8 = array("H")
        self.map_y_q8 = array("H")
        self._build_map()

    def _build_map(self) -> None:
        c = self.calibration
        center_x = (c.output_width - 1.0) * 0.5
        center_y = (c.output_height - 1.0) * 0.5
        append_x = self.map_x_q8.append
        append_y = self.map_y_q8.append

        for y in range(c.output_height):
            ny = (y - center_y) / c.fy
            for x in range(c.output_width):
                nx = (x - center_x) / c.fx
                radius = math.hypot(nx, ny)
                if radius < 1e-12:
                    source_x = c.cx
                    source_y = c.cy
                else:
                    theta = math.atan(radius)
                    theta2 = theta * theta
                    theta4 = theta2 * theta2
                    theta6 = theta4 * theta2
                    theta8 = theta4 * theta4
                    theta_distorted = theta * (
                        1.0
                        + c.k1 * theta2
                        + c.k2 * theta4
                        + c.k3 * theta6
                        + c.k4 * theta8
                    )
                    scale = theta_distorted / radius
                    source_x = c.fx * nx * scale + c.cx
                    source_y = c.fy * ny * scale + c.cy

                # Bilinear sampling requires the right and bottom neighbor.
                if (
                    0.0 <= source_x < c.image_width - 1
                    and 0.0 <= source_y < c.image_height - 1
                ):
                    append_x(int(round(source_x * 256.0)))
                    append_y(int(round(source_y * 256.0)))
                else:
                    append_x(INVALID_Q8)
                    append_y(INVALID_Q8)

    def apply(self, image: Image.Image) -> Image.Image:
        c = self.calibration
        gray = image.convert("L")
        if gray.size != (c.image_width, c.image_height):
            raise ValueError(
                f"图像尺寸为 {gray.width}x{gray.height}，"
                f"标定要求 {c.image_width}x{c.image_height}"
            )

        source = gray.tobytes()
        output = bytearray(c.output_width * c.output_height)
        source_width = c.image_width

        for index, (source_x_q8, source_y_q8) in enumerate(
            zip(self.map_x_q8, self.map_y_q8)
        ):
            if source_x_q8 == INVALID_Q8 or source_y_q8 == INVALID_Q8:
                continue
            source_x = source_x_q8 >> 8
            source_y = source_y_q8 >> 8
            weight_x = source_x_q8 & 0xFF
            weight_y = source_y_q8 & 0xFF
            row0 = source_y * source_width
            row1 = row0 + source_width
            top = (
                source[row0 + source_x] * (256 - weight_x)
                + source[row0 + source_x + 1] * weight_x
            )
            bottom = (
                source[row1 + source_x] * (256 - weight_x)
                + source[row1 + source_x + 1] * weight_x
            )
            output[index] = (
                top * (256 - weight_y) + bottom * weight_y + 32768
            ) >> 16

        return Image.frombytes("L", (c.output_width, c.output_height), bytes(output))

    def is_valid_output_pixel(self, x: int, y: int) -> bool:
        """Return whether a rectified pixel has a valid bilinear source mapping."""
        c = self.calibration
        if x < 0 or y < 0 or x >= c.output_width or y >= c.output_height:
            return False
        index = y * c.output_width + x
        return (
            self.map_x_q8[index] != INVALID_Q8
            and self.map_y_q8[index] != INVALID_Q8
        )


@dataclass
class ConnectedComponent:
    label: int
    area: int
    min_x: int
    min_y: int
    max_x: int
    max_y: int
    centroid_x: float
    centroid_y: float
    boundary_pixels: int
    circularity: float
    occupancy: float

    @property
    def width(self) -> int:
        return self.max_x - self.min_x + 1

    @property
    def height(self) -> int:
        return self.max_y - self.min_y + 1


@dataclass(frozen=True)
class _Run:
    row: int
    start_x: int
    end_x: int


def threshold_image(image: Image.Image, threshold: int) -> Image.Image:
    threshold = max(0, min(255, int(threshold)))
    table = [0] * threshold + [255] * (256 - threshold)
    return image.convert("L").point(table)


def find_components(
    binary: Image.Image,
    min_area: int = 1,
    eight_connected: bool = True,
) -> list[ConnectedComponent]:
    """Find components using horizontal runs, matching the firmware strategy."""
    binary = binary.convert("L")
    width, height = binary.size
    foreground = binary.tobytes()
    visited = bytearray(width * height)
    components: list[ConnectedComponent] = []
    min_area = max(1, int(min_area))
    neighbor_padding = 1 if eight_connected else 0

    def is_foreground(x: int, y: int) -> bool:
        return 0 <= x < width and 0 <= y < height and foreground[y * width + x] != 0

    def append_run(row: int, seed_x: int, runs: list[_Run]) -> None:
        start_x = seed_x
        end_x = seed_x
        row_offset = row * width
        while (
            start_x > 0
            and foreground[row_offset + start_x - 1]
            and not visited[row_offset + start_x - 1]
        ):
            start_x -= 1
        while (
            end_x + 1 < width
            and foreground[row_offset + end_x + 1]
            and not visited[row_offset + end_x + 1]
        ):
            end_x += 1
        visited[row_offset + start_x : row_offset + end_x + 1] = b"\x01" * (
            end_x - start_x + 1
        )
        runs.append(_Run(row, start_x, end_x))

    for y in range(height):
        row_offset = y * width
        for x in range(width):
            index = row_offset + x
            if not foreground[index] or visited[index]:
                continue

            runs: list[_Run] = []
            append_run(y, x, runs)
            front = 0
            while front < len(runs):
                run = runs[front]
                front += 1
                for neighbor_y in (run.row - 1, run.row + 1):
                    if neighbor_y < 0 or neighbor_y >= height:
                        continue
                    scan_x = max(0, run.start_x - neighbor_padding)
                    scan_end = min(width - 1, run.end_x + neighbor_padding)
                    neighbor_offset = neighbor_y * width
                    while scan_x <= scan_end:
                        neighbor_index = neighbor_offset + scan_x
                        if foreground[neighbor_index] and not visited[neighbor_index]:
                            append_run(neighbor_y, scan_x, runs)
                            scan_x = runs[-1].end_x + 1
                        else:
                            scan_x += 1

            area = 0
            x_sum = 0
            y_sum = 0
            min_x = width
            min_y = height
            max_x = -1
            max_y = -1
            boundary_pixels = 0
            for run in runs:
                run_length = run.end_x - run.start_x + 1
                area += run_length
                x_sum += (run.start_x + run.end_x) * run_length // 2
                y_sum += run.row * run_length
                min_x = min(min_x, run.start_x)
                max_x = max(max_x, run.end_x)
                min_y = min(min_y, run.row)
                max_y = max(max_y, run.row)
                for pixel_x in range(run.start_x, run.end_x + 1):
                    boundary = False
                    for dy in (-1, 0, 1):
                        for dx in (-1, 0, 1):
                            if dx == 0 and dy == 0:
                                continue
                            if not is_foreground(pixel_x + dx, run.row + dy):
                                boundary = True
                                break
                        if boundary:
                            break
                    if boundary:
                        boundary_pixels += 1

            if area < min_area:
                continue
            box_area = (max_x - min_x + 1) * (max_y - min_y + 1)
            circularity = 0.0
            if boundary_pixels:
                circularity = min(
                    1.0,
                    4.0 * math.pi * area / (boundary_pixels * boundary_pixels),
                )
            components.append(
                ConnectedComponent(
                    label=0,
                    area=area,
                    min_x=min_x,
                    min_y=min_y,
                    max_x=max_x,
                    max_y=max_y,
                    centroid_x=x_sum / area,
                    centroid_y=y_sum / area,
                    boundary_pixels=boundary_pixels,
                    circularity=circularity,
                    occupancy=area / box_area if box_area else 0.0,
                )
            )

    components.sort(key=lambda component: component.area, reverse=True)
    for label, component in enumerate(components, start=1):
        component.label = label
    return components


def self_test() -> None:
    source = Image.frombytes("L", (3, 3), bytes((255, 0, 0, 0, 255, 0, 0, 0, 0)))
    assert len(find_components(source, eight_connected=True)) == 1
    assert len(find_components(source, eight_connected=False)) == 2
    gray = Image.frombytes("L", (3, 1), bytes((39, 40, 41)))
    assert threshold_image(gray, 40).tobytes() == bytes((0, 255, 255))


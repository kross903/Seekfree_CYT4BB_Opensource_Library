#!/usr/bin/env python3
"""Positive/negative grayscale sampling around a projected ideal target."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Callable, Sequence

from PIL import Image


Point2D = tuple[float, float]
PixelPoint = tuple[int, int]
PixelValidator = Callable[[int, int], bool]
MAX_SAMPLING_GRID_POINTS = 30000


@dataclass(frozen=True)
class SamplingConfig:
    positive_step_px: int = 2
    negative_step_px: int = 4
    margin_px: float = 4.0
    positive_band_px: float = 1.5
    negative_guard_px: float = 3.0

    def validate(self) -> None:
        values = (
            self.margin_px,
            self.positive_band_px,
            self.negative_guard_px,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("采样参数必须是有限数值")
        if not 1 <= self.positive_step_px <= 32:
            raise ValueError("正形采样步长必须在1到32像素之间")
        if not 1 <= self.negative_step_px <= 32:
            raise ValueError("负形采样步长必须在1到32像素之间")
        if self.margin_px < 0.0:
            raise ValueError("采样框余量不能小于0")
        if self.positive_band_px <= 0.0:
            raise ValueError("正形轮廓带必须大于0")
        if self.negative_guard_px <= self.positive_band_px:
            raise ValueError("负形最小距离必须大于正形轮廓带")


@dataclass(frozen=True)
class SamplingResult:
    bounds: tuple[int, int, int, int] | None
    positive_points: tuple[PixelPoint, ...]
    negative_points: tuple[PixelPoint, ...]
    positive_mean: float | None
    negative_mean: float | None
    ignored_count: int
    invalid_count: int
    box_clipped: bool
    near_shape_points: tuple[PixelPoint, ...] = ()


def _point_segment_distance_sq(point: Point2D, start: Point2D, end: Point2D) -> float:
    segment_x = end[0] - start[0]
    segment_y = end[1] - start[1]
    length_sq = segment_x * segment_x + segment_y * segment_y
    if length_sq <= 1e-12:
        delta_x = point[0] - start[0]
        delta_y = point[1] - start[1]
        return delta_x * delta_x + delta_y * delta_y
    projection = (
        (point[0] - start[0]) * segment_x
        + (point[1] - start[1]) * segment_y
    ) / length_sq
    projection = max(0.0, min(1.0, projection))
    closest_x = start[0] + projection * segment_x
    closest_y = start[1] + projection * segment_y
    delta_x = point[0] - closest_x
    delta_y = point[1] - closest_y
    return delta_x * delta_x + delta_y * delta_y


def _point_outline_distance_sq(
    point: Point2D,
    outline: Sequence[Point2D],
    closed: bool,
) -> float:
    segment_count = len(outline) if closed else len(outline) - 1
    return min(
        _point_segment_distance_sq(
            point,
            outline[index],
            outline[(index + 1) % len(outline)],
        )
        for index in range(segment_count)
    )


def point_segment_distance_sq(
    point: Point2D,
    start: Point2D,
    end: Point2D,
) -> float:
    """Public geometry helper used by the independent matcher."""
    return _point_segment_distance_sq(point, start, end)


def point_outline_distance_sq(
    point: Point2D,
    outline: Sequence[Point2D],
    closed: bool,
) -> float:
    """Public geometry helper used by the independent matcher."""
    return _point_outline_distance_sq(point, outline, closed)


def _point_in_polygon(point: Point2D, polygon: Sequence[Point2D]) -> bool:
    inside = False
    previous = polygon[-1]
    for current in polygon:
        crosses = (current[1] > point[1]) != (previous[1] > point[1])
        if crosses:
            crossing_x = (
                (previous[0] - current[0])
                * (point[1] - current[1])
                / (previous[1] - current[1])
                + current[0]
            )
            if point[0] < crossing_x:
                inside = not inside
        previous = current
    return inside


def _mean(values: list[int]) -> float | None:
    return sum(values) / len(values) if values else None


def sample_projected_shape(
    gray_image: Image.Image,
    outline: Sequence[Point2D],
    *,
    closed: bool,
    filled_positive: bool,
    config: SamplingConfig,
    is_valid_pixel: PixelValidator | None = None,
    collect_near_shape: bool = False,
) -> SamplingResult:
    """Classify a globally aligned pixel grid inside an expanded outline box."""
    config.validate()
    minimum_points = 3 if closed else 2
    if len(outline) < minimum_points:
        raise ValueError("理想轮廓点数量不足")
    if filled_positive and not closed:
        raise ValueError("只有闭合轮廓可以将内部作为正形")
    if not all(math.isfinite(value) for point in outline for value in point):
        raise ValueError("理想轮廓包含无效坐标")

    gray = gray_image.convert("L")
    width, height = gray.size
    if width <= 0 or height <= 0:
        raise ValueError("灰度图尺寸无效")

    raw_min_x = math.floor(min(point[0] for point in outline) - config.margin_px)
    raw_min_y = math.floor(min(point[1] for point in outline) - config.margin_px)
    raw_max_x = math.ceil(max(point[0] for point in outline) + config.margin_px)
    raw_max_y = math.ceil(max(point[1] for point in outline) + config.margin_px)
    box_clipped = (
        raw_min_x < 0 or raw_min_y < 0
        or raw_max_x >= width or raw_max_y >= height
    )
    min_x = max(0, raw_min_x)
    min_y = max(0, raw_min_y)
    max_x = min(width - 1, raw_max_x)
    max_y = min(height - 1, raw_max_y)
    if min_x > max_x or min_y > max_y:
        return SamplingResult(
            bounds=None,
            positive_points=(),
            negative_points=(),
            positive_mean=None,
            negative_mean=None,
            ignored_count=0,
            invalid_count=0,
            box_clipped=True,
        )

    box_width = max_x - min_x + 1
    box_height = max_y - min_y + 1
    automatic_step = max((box_width + 15) // 16, (box_height + 15) // 16)
    positive_step = max(int(config.positive_step_px), automatic_step)
    negative_step = max(int(config.negative_step_px), automatic_step)
    # 两套采样共用一个基础网格，避免重复读取同一像素；只有对齐到
    # 对应步长的网格点才参与正形或负形统计。
    step = min(positive_step, negative_step)
    start_x = ((min_x + step - 1) // step) * step
    start_y = ((min_y + step - 1) // step) * step
    column_count = (max_x - start_x) // step + 1
    row_count = (max_y - start_y) // step + 1
    if column_count * row_count > MAX_SAMPLING_GRID_POINTS:
        raise ValueError("采样点超过30000，请增大采样步长")
    positive_limit_sq = config.positive_band_px * config.positive_band_px
    negative_limit_sq = config.negative_guard_px * config.negative_guard_px
    pixels = gray.tobytes()
    positive_points: list[PixelPoint] = []
    negative_points: list[PixelPoint] = []
    positive_values: list[int] = []
    negative_values: list[int] = []
    near_shape_points: list[PixelPoint] = []
    ignored_count = 0
    invalid_count = 0

    for y in range(start_y, max_y + 1, step):
        for x in range(start_x, max_x + 1, step):
            positive_grid = (
                (x - start_x) % positive_step == 0
                and (y - start_y) % positive_step == 0
            )
            negative_grid = (
                (x - start_x) % negative_step == 0
                and (y - start_y) % negative_step == 0
            )
            if not positive_grid and not negative_grid:
                continue
            if is_valid_pixel is not None and not is_valid_pixel(x, y):
                invalid_count += 1
                continue
            point = (float(x), float(y))
            distance_sq = _point_outline_distance_sq(point, outline, closed)
            inside = closed and _point_in_polygon(point, outline)
            value = pixels[y * width + x]
            classified = False
            if positive_grid and ((filled_positive and inside) or
                                  distance_sq <= positive_limit_sq):
                positive_points.append((x, y))
                positive_values.append(value)
                classified = True
            if (negative_grid and not classified and
                    distance_sq >= negative_limit_sq and
                    (not closed or not inside)):
                negative_points.append((x, y))
                negative_values.append(value)
                classified = True
            if not classified:
                near_shape = (
                    collect_near_shape
                    and distance_sq > positive_limit_sq
                    and distance_sq < negative_limit_sq
                )
                if near_shape:
                    near_shape_points.append((x, y))
                else:
                    ignored_count += 1

    return SamplingResult(
        bounds=(min_x, min_y, max_x, max_y),
        positive_points=tuple(positive_points),
        negative_points=tuple(negative_points),
        positive_mean=_mean(positive_values),
        negative_mean=_mean(negative_values),
        ignored_count=ignored_count,
        invalid_count=invalid_count,
        box_clipped=box_clipped,
        near_shape_points=tuple(near_shape_points),
    )


def self_test() -> None:
    signal_pixels = bytearray([20] * (11 * 11))
    for y in range(3, 8):
        for x in range(3, 8):
            signal_pixels[y * 11 + x] = 200
    signal = Image.frombytes("L", (11, 11), bytes(signal_pixels))
    square = ((3.0, 3.0), (7.0, 3.0), (7.0, 7.0), (3.0, 7.0))
    signal_result = sample_projected_shape(
        signal,
        square,
        closed=True,
        filled_positive=True,
        config=SamplingConfig(
            positive_step_px=1,
            negative_step_px=1,
            margin_px=2.0,
            positive_band_px=0.25,
            negative_guard_px=1.5,
        ),
        is_valid_pixel=lambda x, y: (x, y) != (1, 1),
    )
    assert signal_result.bounds == (1, 1, 9, 9)
    assert (5, 5) in signal_result.positive_points
    assert signal_result.invalid_count == 1
    assert signal_result.positive_mean == 200.0
    assert signal_result.negative_mean == 20.0

    arrow = ((2.0, 2.0), (5.0, 5.0), (8.0, 2.0))
    arrow_pixels = bytearray([30] * (11 * 8))
    for y in range(8):
        for x in range(11):
            if _point_outline_distance_sq((float(x), float(y)), arrow, False) <= 0.25:
                arrow_pixels[y * 11 + x] = 180
    arrow_result = sample_projected_shape(
        Image.frombytes("L", (11, 8), bytes(arrow_pixels)),
        arrow,
        closed=False,
        filled_positive=False,
        config=SamplingConfig(
            positive_step_px=1,
            negative_step_px=1,
            margin_px=2.0,
            positive_band_px=0.5,
            negative_guard_px=1.5,
        ),
    )
    assert arrow_result.positive_mean == 180.0
    assert arrow_result.negative_mean == 30.0

    aligned_result = sample_projected_shape(
        signal,
        square,
        closed=True,
        filled_positive=True,
        config=SamplingConfig(positive_step_px=2, negative_step_px=4),
    )
    assert all(x % 2 == 0 and y % 2 == 0 for x, y in (
        aligned_result.positive_points + aligned_result.negative_points
    ))

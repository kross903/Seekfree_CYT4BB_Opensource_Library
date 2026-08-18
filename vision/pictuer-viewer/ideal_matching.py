#!/usr/bin/env python3
"""Lightweight metrics for matching sampled grayscale against an ideal target."""

from __future__ import annotations

from dataclasses import dataclass, replace
import math
from typing import Iterable, Mapping, Sequence

from PIL import Image

from ideal_sampling import (
    Point2D,
    SamplingResult,
    point_outline_distance_sq,
    point_segment_distance_sq,
)


TARGET_ARROW = "arrow"
TARGET_BEACON = "beacon"

GRAY_DIFFERENCE = "gray_difference"
STANDARDIZED_SEPARATION = "standardized_separation"
POSITIVE_HIT_RATE = "positive_hit_rate"
NEGATIVE_HIT_RATE = "negative_hit_rate"
BINARY_CONSISTENCY = "binary_consistency"
COMPLETENESS = "completeness"
NEAR_DIFFERENCE = "near_difference"
FAR_DIFFERENCE = "far_difference"
POSITIVE_POINTS = "positive_points"
NEGATIVE_POINTS = "negative_points"

THRESHOLD_KEYS = (
    GRAY_DIFFERENCE,
    STANDARDIZED_SEPARATION,
    POSITIVE_HIT_RATE,
    NEGATIVE_HIT_RATE,
    BINARY_CONSISTENCY,
    COMPLETENESS,
    NEAR_DIFFERENCE,
    FAR_DIFFERENCE,
    POSITIVE_POINTS,
    NEGATIVE_POINTS,
)

THRESHOLD_ATTRIBUTES = {
    GRAY_DIFFERENCE: "min_gray_difference",
    STANDARDIZED_SEPARATION: "min_standardized_separation",
    POSITIVE_HIT_RATE: "min_positive_hit_rate",
    NEGATIVE_HIT_RATE: "min_negative_hit_rate",
    BINARY_CONSISTENCY: "min_binary_consistency",
    COMPLETENESS: "min_completeness",
    NEAR_DIFFERENCE: "min_near_difference",
    FAR_DIFFERENCE: "min_far_difference",
    POSITIVE_POINTS: "min_positive_points",
    NEGATIVE_POINTS: "min_negative_points",
}

METRIC_ATTRIBUTES = {
    GRAY_DIFFERENCE: "gray_difference",
    STANDARDIZED_SEPARATION: "standardized_separation",
    POSITIVE_HIT_RATE: "positive_hit_rate",
    NEGATIVE_HIT_RATE: "negative_hit_rate",
    BINARY_CONSISTENCY: "binary_consistency",
    COMPLETENESS: "completeness",
    NEAR_DIFFERENCE: "near_difference",
    FAR_DIFFERENCE: "far_difference",
    POSITIVE_POINTS: "positive_count",
    NEGATIVE_POINTS: "negative_count",
}


@dataclass(frozen=True)
class MatchingThresholds:
    min_gray_difference: float
    min_standardized_separation: float
    min_positive_hit_rate: float
    min_negative_hit_rate: float
    min_binary_consistency: float
    min_completeness: float
    min_near_difference: float
    min_far_difference: float
    min_positive_points: int
    min_negative_points: int
    enabled: tuple[str, ...] = THRESHOLD_KEYS

    def validate(self) -> None:
        scalar_values = (
            self.min_gray_difference,
            self.min_standardized_separation,
            self.min_near_difference,
            self.min_far_difference,
        )
        if not all(math.isfinite(value) and value >= 0.0 for value in scalar_values):
            raise ValueError("灰度和分离度门槛必须是非负有限数")
        rate_values = (
            self.min_positive_hit_rate,
            self.min_negative_hit_rate,
            self.min_binary_consistency,
            self.min_completeness,
        )
        if not all(math.isfinite(value) and 0.0 <= value <= 1.0 for value in rate_values):
            raise ValueError("命中率和完整度门槛必须在0到1之间")
        if self.min_positive_points < 1 or self.min_negative_points < 1:
            raise ValueError("最少采样点数必须大于0")
        if any(key not in THRESHOLD_KEYS for key in self.enabled):
            raise ValueError("启用的识别门槛名称无效")

    def to_mapping(self) -> dict[str, object]:
        result: dict[str, object] = {"enabled": list(self.enabled)}
        for key, attribute in THRESHOLD_ATTRIBUTES.items():
            result[key] = getattr(self, attribute)
        return result

    @classmethod
    def from_mapping(
        cls,
        mapping: object,
        fallback: "MatchingThresholds",
    ) -> "MatchingThresholds":
        if not isinstance(mapping, Mapping):
            return fallback
        updates: dict[str, object] = {}
        for key, attribute in THRESHOLD_ATTRIBUTES.items():
            raw_value = mapping.get(key, getattr(fallback, attribute))
            try:
                updates[attribute] = (
                    int(raw_value)
                    if key in (POSITIVE_POINTS, NEGATIVE_POINTS)
                    else float(raw_value)
                )
            except (TypeError, ValueError):
                updates[attribute] = getattr(fallback, attribute)
        raw_enabled = mapping.get("enabled", fallback.enabled)
        if isinstance(raw_enabled, (list, tuple)):
            updates["enabled"] = tuple(
                str(key) for key in raw_enabled if str(key) in THRESHOLD_KEYS
            )
        else:
            updates["enabled"] = fallback.enabled
        result = replace(fallback, **updates)
        try:
            result.validate()
        except ValueError:
            return fallback
        return result


ARROW_DEFAULT_THRESHOLDS = MatchingThresholds(
    min_gray_difference=70.0,
    min_standardized_separation=1.25,
    min_positive_hit_rate=0.60,
    min_negative_hit_rate=0.65,
    min_binary_consistency=0.65,
    min_completeness=0.45,
    min_near_difference=55.0,
    min_far_difference=110.0,
    min_positive_points=8,
    min_negative_points=16,
    enabled=tuple(key for key in THRESHOLD_KEYS if key != FAR_DIFFERENCE),
)

BEACON_DEFAULT_THRESHOLDS = MatchingThresholds(
    min_gray_difference=90.0,
    min_standardized_separation=2.0,
    min_positive_hit_rate=0.68,
    min_negative_hit_rate=0.75,
    min_binary_consistency=0.70,
    min_completeness=0.30,
    min_near_difference=80.0,
    min_far_difference=90.0,
    min_positive_points=8,
    min_negative_points=16,
    enabled=tuple(key for key in THRESHOLD_KEYS if key != FAR_DIFFERENCE),
)


@dataclass(frozen=True)
class MatchingMetrics:
    positive_mean: float | None
    negative_mean: float | None
    near_negative_mean: float | None
    far_negative_mean: float | None
    local_threshold: float | None
    gray_difference: float | None
    standardized_separation: float | None
    positive_hit_rate: float | None
    negative_hit_rate: float | None
    binary_consistency: float | None
    completeness: float | None
    near_difference: float | None
    far_difference: float | None
    near_shape_variance: float | None
    positive_count: int
    negative_count: int
    near_negative_count: int
    far_negative_count: int
    near_shape_count: int


@dataclass(frozen=True)
class MatchingDecision:
    passed: bool
    failures: tuple[str, ...]


def _mean(values: Sequence[int]) -> float | None:
    return sum(values) / len(values) if values else None


def _variance(values: Sequence[int], mean: float) -> float:
    return sum((value - mean) ** 2 for value in values) / len(values)


def _hit_rate(values: Sequence[int], threshold: float, positive: bool) -> float | None:
    if not values:
        return None
    hits = sum((value >= threshold) if positive else (value < threshold) for value in values)
    return hits / len(values)


def _minimum_group_hit_rate(
    groups: Sequence[Sequence[int]],
    threshold: float,
) -> float | None:
    if not groups or any(not group for group in groups):
        return None
    return min(
        sum(value >= threshold for value in group) / len(group)
        for group in groups
    )


def calculate_matching_metrics(
    gray_image: Image.Image,
    outline: Sequence[Point2D],
    sampling: SamplingResult,
    *,
    target_kind: str,
    center: Point2D,
    binary_threshold: int,
    negative_guard_px: float,
    near_negative_width_px: float,
) -> MatchingMetrics:
    if target_kind not in (TARGET_ARROW, TARGET_BEACON):
        raise ValueError("理想目标类型无效")
    if target_kind == TARGET_ARROW and len(outline) != 3:
        raise ValueError("箭头轮廓必须包含三个端点")
    if target_kind == TARGET_BEACON and len(outline) < 3:
        raise ValueError("信标轮廓点数不足")
    if not math.isfinite(negative_guard_px) or negative_guard_px < 0.0:
        raise ValueError("负形最小距离无效")
    if not math.isfinite(near_negative_width_px) or near_negative_width_px <= 0.0:
        raise ValueError("近负形环宽度必须大于0")

    gray = gray_image.convert("L")
    width, height = gray.size
    pixels = gray.tobytes()

    def values_at(points: Iterable[tuple[int, int]]) -> list[int]:
        return [
            pixels[y * width + x]
            for x, y in points
            if 0 <= x < width and 0 <= y < height
        ]

    positive_values = values_at(sampling.positive_points)
    negative_values = values_at(sampling.negative_points)
    positive_mean = _mean(positive_values)
    negative_mean = _mean(negative_values)

    local_threshold = None
    gray_difference = None
    standardized_separation = None
    positive_hit_rate = None
    negative_hit_rate = None
    binary_consistency = None
    completeness = None
    if positive_mean is not None and negative_mean is not None:
        local_threshold = 0.5 * (positive_mean + negative_mean)
        gray_difference = positive_mean - negative_mean
        positive_variance = _variance(positive_values, positive_mean)
        negative_variance = _variance(negative_values, negative_mean)
        standardized_separation = gray_difference / math.sqrt(
            0.5 * (positive_variance + negative_variance) + 1.0
        )
        positive_hit_rate = _hit_rate(positive_values, local_threshold, True)
        negative_hit_rate = _hit_rate(negative_values, local_threshold, False)
        positive_binary_hit = _hit_rate(positive_values, binary_threshold, True)
        negative_binary_hit = _hit_rate(negative_values, binary_threshold, False)
        if positive_binary_hit is not None and negative_binary_hit is not None:
            binary_consistency = 0.5 * (positive_binary_hit + negative_binary_hit)

        if target_kind == TARGET_ARROW:
            groups: list[list[int]] = [[], []]
            for point in sampling.positive_points:
                distances = (
                    point_segment_distance_sq(point, outline[0], outline[1]),
                    point_segment_distance_sq(point, outline[1], outline[2]),
                )
                group_index = 0 if distances[0] <= distances[1] else 1
                groups[group_index].extend(values_at((point,)))
        else:
            groups = [[], [], [], []]
            for point in sampling.positive_points:
                quadrant = (1 if point[0] >= center[0] else 0) + (
                    2 if point[1] >= center[1] else 0
                )
                groups[quadrant].extend(values_at((point,)))
        completeness = _minimum_group_hit_rate(groups, local_threshold)

    split_distance = negative_guard_px + near_negative_width_px
    near_points: list[tuple[int, int]] = []
    far_points: list[tuple[int, int]] = []
    closed = target_kind == TARGET_BEACON
    for point in sampling.negative_points:
        distance = math.sqrt(point_outline_distance_sq(point, outline, closed))
        (near_points if distance < split_distance else far_points).append(point)
    near_values = values_at(near_points)
    far_values = values_at(far_points)
    near_negative_mean = _mean(near_values)
    far_negative_mean = _mean(far_values)
    near_shape_values = values_at(sampling.near_shape_points)
    near_shape_mean = _mean(near_shape_values)
    near_shape_variance = (
        _variance(near_shape_values, near_shape_mean)
        if near_shape_mean is not None
        else None
    )
    near_difference = (
        positive_mean - near_negative_mean
        if positive_mean is not None and near_negative_mean is not None
        else None
    )
    far_difference = (
        positive_mean - far_negative_mean
        if positive_mean is not None and far_negative_mean is not None
        else None
    )

    return MatchingMetrics(
        positive_mean=positive_mean,
        negative_mean=negative_mean,
        near_negative_mean=near_negative_mean,
        far_negative_mean=far_negative_mean,
        local_threshold=local_threshold,
        gray_difference=gray_difference,
        standardized_separation=standardized_separation,
        positive_hit_rate=positive_hit_rate,
        negative_hit_rate=negative_hit_rate,
        binary_consistency=binary_consistency,
        completeness=completeness,
        near_difference=near_difference,
        far_difference=far_difference,
        near_shape_variance=near_shape_variance,
        positive_count=len(positive_values),
        negative_count=len(negative_values),
        near_negative_count=len(near_values),
        far_negative_count=len(far_values),
        near_shape_count=len(near_shape_values),
    )


def evaluate_matching_metrics(
    metrics: MatchingMetrics,
    thresholds: MatchingThresholds,
    *,
    target_kind: str,
) -> MatchingDecision:
    thresholds.validate()
    labels = {
        GRAY_DIFFERENCE: "正负形灰度差",
        STANDARDIZED_SEPARATION: "标准化分离度",
        POSITIVE_HIT_RATE: "正形命中率",
        NEGATIVE_HIT_RATE: "负形命中率",
        BINARY_CONSISTENCY: "固定阈值一致率",
        COMPLETENESS: "左右臂完整度" if target_kind == TARGET_ARROW else "四象限完整度",
        NEAR_DIFFERENCE: "近负形灰度差",
        FAR_DIFFERENCE: "远背景灰度差",
        POSITIVE_POINTS: "正形采样点",
        NEGATIVE_POINTS: "负形采样点",
    }
    failures: list[str] = []
    for key in THRESHOLD_KEYS:
        if key not in thresholds.enabled:
            continue
        value = getattr(metrics, METRIC_ATTRIBUTES[key])
        minimum = getattr(thresholds, THRESHOLD_ATTRIBUTES[key])
        if value is None:
            failures.append(f"{labels[key]}无有效数据")
        elif value < minimum:
            if key in (POSITIVE_POINTS, NEGATIVE_POINTS):
                failures.append(f"{labels[key]} {int(value)} < {int(minimum)}")
            else:
                failures.append(f"{labels[key]} {value:.3f} < {minimum:.3f}")
    return MatchingDecision(passed=not failures, failures=tuple(failures))


def self_test() -> None:
    pixels = bytearray([20] * (11 * 8))
    outline = ((2.0, 2.0), (5.0, 5.0), (8.0, 2.0))
    positive_points: list[tuple[int, int]] = []
    negative_points: list[tuple[int, int]] = []
    for y in range(8):
        for x in range(11):
            distance = math.sqrt(point_outline_distance_sq((x, y), outline, False))
            if distance <= 0.5:
                pixels[y * 11 + x] = 200
                positive_points.append((x, y))
            elif distance >= 2.0:
                negative_points.append((x, y))
    sampling = SamplingResult(
        bounds=(0, 0, 10, 7),
        positive_points=tuple(positive_points),
        negative_points=tuple(negative_points),
        positive_mean=200.0,
        negative_mean=20.0,
        ignored_count=0,
        invalid_count=0,
        box_clipped=False,
    )
    metrics = calculate_matching_metrics(
        Image.frombytes("L", (11, 8), bytes(pixels)),
        outline,
        sampling,
        target_kind=TARGET_ARROW,
        center=(5.0, 4.0),
        binary_threshold=100,
        negative_guard_px=2.0,
        near_negative_width_px=2.0,
    )
    assert metrics.gray_difference == 180.0
    assert metrics.positive_hit_rate == 1.0
    assert metrics.negative_hit_rate == 1.0
    assert metrics.completeness == 1.0
    decision = evaluate_matching_metrics(
        metrics,
        replace(
            ARROW_DEFAULT_THRESHOLDS,
            min_gray_difference=100.0,
            enabled=(
                GRAY_DIFFERENCE,
                POSITIVE_HIT_RATE,
                NEGATIVE_HIT_RATE,
                COMPLETENESS,
            ),
        ),
        target_kind=TARGET_ARROW,
    )
    assert decision.passed

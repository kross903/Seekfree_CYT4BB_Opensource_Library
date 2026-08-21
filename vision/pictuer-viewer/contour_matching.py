#!/usr/bin/env python3
"""Geometry-only target recognition based on an extracted 8-neighbour contour."""

from __future__ import annotations

from dataclasses import dataclass, fields
import math
from typing import Callable, Mapping, Sequence

from PIL import Image, ImageDraw

from image_analysis import ConnectedComponent, find_contour_components


TARGET_ARROW = "arrow"
TARGET_BEACON = "beacon"

CONTOUR_THRESHOLD_FIELDS = (
    ("min_contour_points", "最少轮廓点"),
    ("min_gray_contrast", "最低前景/背景灰度差"),
    ("min_aspect_ratio", "长短边比例下限"),
    ("max_aspect_ratio", "长短边比例上限"),
    ("min_circularity", "圆度下限"),
    ("max_circularity", "圆度上限"),
    ("min_solidity", "凸包填充率下限"),
    ("max_solidity", "凸包填充率上限"),
    ("min_radial_cv", "径向波动下限"),
    ("max_radial_cv", "径向波动上限"),
    ("min_concavity_ratio", "最大凹陷比例下限"),
    ("max_concavity_ratio", "最大凹陷比例上限"),
    ("min_direction_entropy", "方向熵下限"),
    ("max_direction_entropy", "方向熵上限"),
    ("min_dominant_axis_ratio", "两组主方向占比下限"),
    ("max_dominant_axis_ratio", "两组主方向占比上限"),
    ("min_vertex_angle_deg", "候选顶点夹角下限(°)"),
    ("max_vertex_angle_deg", "候选顶点夹角上限(°)"),
    ("min_arm_balance", "候选两臂平衡度下限"),
    ("direction_window_points", "方向均值窗口点数"),
    ("min_endpoint_turn_deg", "端点方向差下限(°)"),
    ("min_vertex_turn_deg", "外顶点方向差下限(°)"),
    ("max_vertex_turn_deg", "外顶点方向差上限(°)"),
    ("min_notch_turn_deg", "内凹口反向差下限(°)"),
    ("max_negative_turn_deg", "最大反向转角上限(°)"),
)
CONTOUR_PARAMETER_FIELDS = frozenset({"direction_window_points"})
CONTOUR_GATE_FIELDS = tuple(
    field_name
    for field_name, _label in CONTOUR_THRESHOLD_FIELDS
    if field_name not in CONTOUR_PARAMETER_FIELDS
)
CONTOUR_YAW_RESCUE_FIELDS = frozenset(
    {
        "min_vertex_angle_deg",
        "max_vertex_angle_deg",
        "min_arm_balance",
        "min_endpoint_turn_deg",
        "min_vertex_turn_deg",
        "max_vertex_turn_deg",
        "min_notch_turn_deg",
        "max_negative_turn_deg",
    }
)


@dataclass(frozen=True)
class ContourThresholds:
    min_contour_points: int
    min_gray_contrast: float
    min_aspect_ratio: float
    max_aspect_ratio: float
    min_circularity: float
    max_circularity: float
    min_solidity: float
    max_solidity: float
    min_radial_cv: float
    max_radial_cv: float
    min_concavity_ratio: float
    max_concavity_ratio: float
    min_direction_entropy: float
    max_direction_entropy: float
    min_dominant_axis_ratio: float
    max_dominant_axis_ratio: float
    min_vertex_angle_deg: float
    max_vertex_angle_deg: float
    min_arm_balance: float
    direction_window_points: int
    min_endpoint_turn_deg: float
    min_vertex_turn_deg: float
    max_vertex_turn_deg: float
    min_notch_turn_deg: float
    max_negative_turn_deg: float
    enabled: tuple[str, ...]

    def validate(self) -> None:
        invalid_enabled = set(self.enabled) - set(CONTOUR_GATE_FIELDS)
        if invalid_enabled:
            raise ValueError(
                f"未知轮廓门槛开关：{', '.join(sorted(invalid_enabled))}"
            )
        enabled = set(self.enabled)
        if self.min_contour_points < 1:
            raise ValueError("最少轮廓点必须大于0")
        if not math.isfinite(self.min_gray_contrast) or not 0.0 <= self.min_gray_contrast <= 255.0:
            raise ValueError("最低灰度差必须在0到255之间")
        for minimum_name, maximum_name, label in (
            ("min_aspect_ratio", "max_aspect_ratio", "长短边比例"),
            ("min_circularity", "max_circularity", "圆度"),
            ("min_solidity", "max_solidity", "凸包填充率"),
            ("min_radial_cv", "max_radial_cv", "径向波动"),
            ("min_concavity_ratio", "max_concavity_ratio", "最大凹陷比例"),
            ("min_direction_entropy", "max_direction_entropy", "方向熵"),
            ("min_dominant_axis_ratio", "max_dominant_axis_ratio", "两组主方向占比"),
        ):
            minimum = float(getattr(self, minimum_name))
            maximum = float(getattr(self, maximum_name))
            if not (
                math.isfinite(minimum)
                and math.isfinite(maximum)
                and 0.0 <= minimum <= 1.0
                and 0.0 <= maximum <= 1.0
            ):
                raise ValueError(f"{label}上下限必须在0到1之间")
            if (
                minimum_name in enabled
                and maximum_name in enabled
                and minimum > maximum
            ):
                raise ValueError(f"{label}已启用的下限不能大于上限")
        if not (
            math.isfinite(self.min_vertex_angle_deg)
            and math.isfinite(self.max_vertex_angle_deg)
            and 0.0 <= self.min_vertex_angle_deg <= 180.0
            and 0.0 <= self.max_vertex_angle_deg <= 180.0
        ):
            raise ValueError("候选顶点夹角上下限必须在0到180度之间")
        if (
            "min_vertex_angle_deg" in enabled
            and "max_vertex_angle_deg" in enabled
            and self.min_vertex_angle_deg > self.max_vertex_angle_deg
        ):
            raise ValueError("候选顶点夹角已启用的下限不能大于上限")
        if not math.isfinite(self.min_arm_balance) or not 0.0 <= self.min_arm_balance <= 1.0:
            raise ValueError("候选两臂平衡度下限必须在0到1之间")
        if not 2 <= self.direction_window_points <= 32:
            raise ValueError("方向均值窗口点数必须在2到32之间")
        turn_values = (
            self.min_endpoint_turn_deg,
            self.min_vertex_turn_deg,
            self.max_vertex_turn_deg,
            self.min_notch_turn_deg,
            self.max_negative_turn_deg,
        )
        if not all(math.isfinite(value) and 0.0 <= value <= 180.0 for value in turn_values):
            raise ValueError("轮廓方向差阈值必须在0到180度之间")
        if (
            "min_vertex_turn_deg" in enabled
            and "max_vertex_turn_deg" in enabled
            and self.min_vertex_turn_deg > self.max_vertex_turn_deg
        ):
            raise ValueError("外顶点方向差已启用的下限不能大于上限")
        if (
            "min_endpoint_turn_deg" in enabled
            and "max_vertex_turn_deg" in enabled
            and self.min_endpoint_turn_deg > 0.0
            and self.min_endpoint_turn_deg < self.max_vertex_turn_deg
        ):
            raise ValueError("端点方向差下限不能小于外顶点方向差上限")

    def to_mapping(self) -> dict[str, float | int | list[str]]:
        mapping: dict[str, float | int | list[str]] = {}
        for field in fields(self):
            value = getattr(self, field.name)
            mapping[field.name] = list(value) if field.name == "enabled" else value
        return mapping

    @classmethod
    def from_mapping(
        cls,
        mapping: object,
        fallback: "ContourThresholds",
    ) -> "ContourThresholds":
        if not isinstance(mapping, Mapping):
            return fallback
        values: dict[str, float | int | tuple[str, ...]] = {}
        for field in fields(fallback):
            fallback_value = getattr(fallback, field.name)
            raw_value = mapping.get(field.name, fallback_value)
            if field.name == "enabled":
                if isinstance(raw_value, (list, tuple)):
                    values[field.name] = tuple(
                        str(item)
                        for item in raw_value
                        if str(item) in CONTOUR_GATE_FIELDS
                    )
                else:
                    values[field.name] = fallback_value
                continue
            try:
                values[field.name] = (
                    int(raw_value)
                    if field.name in {"min_contour_points", "direction_window_points"}
                    else float(raw_value)
                )
            except (TypeError, ValueError):
                values[field.name] = fallback_value
        result = cls(**values)
        try:
            result.validate()
        except ValueError:
            return fallback
        return result


# These are deliberately broad starting values. The viewer is intended to expose
# the metrics first; captured data should be used to tighten them later.
ARROW_DEFAULT_THRESHOLDS = ContourThresholds(
    min_contour_points=8,
    min_gray_contrast=5.0,
    min_aspect_ratio=0.15,
    max_aspect_ratio=1.0,
    min_circularity=0.0,
    max_circularity=0.82,
    min_solidity=0.0,
    max_solidity=0.92,
    min_radial_cv=0.08,
    max_radial_cv=1.0,
    min_concavity_ratio=0.03,
    max_concavity_ratio=1.0,
    min_direction_entropy=0.0,
    max_direction_entropy=0.99,
    min_dominant_axis_ratio=0.50,
    max_dominant_axis_ratio=1.0,
    min_vertex_angle_deg=25.0,
    max_vertex_angle_deg=130.0,
    min_arm_balance=0.25,
    direction_window_points=12,
    min_endpoint_turn_deg=135.0,
    min_vertex_turn_deg=80.0,
    max_vertex_turn_deg=135.0,
    min_notch_turn_deg=60.0,
    max_negative_turn_deg=180.0,
    enabled=CONTOUR_GATE_FIELDS,
)

BEACON_DEFAULT_THRESHOLDS = ContourThresholds(
    min_contour_points=8,
    min_gray_contrast=5.0,
    min_aspect_ratio=0.45,
    max_aspect_ratio=1.0,
    min_circularity=0.35,
    max_circularity=1.0,
    min_solidity=0.65,
    max_solidity=1.0,
    min_radial_cv=0.0,
    max_radial_cv=0.45,
    min_concavity_ratio=0.0,
    max_concavity_ratio=0.18,
    min_direction_entropy=0.55,
    max_direction_entropy=1.0,
    min_dominant_axis_ratio=0.0,
    max_dominant_axis_ratio=1.0,
    min_vertex_angle_deg=0.0,
    max_vertex_angle_deg=180.0,
    min_arm_balance=0.0,
    direction_window_points=12,
    min_endpoint_turn_deg=0.0,
    min_vertex_turn_deg=0.0,
    max_vertex_turn_deg=180.0,
    min_notch_turn_deg=0.0,
    max_negative_turn_deg=90.0,
    enabled=tuple(
        field_name
        for field_name in CONTOUR_GATE_FIELDS
        if field_name
        not in {
            "min_vertex_angle_deg",
            "max_vertex_angle_deg",
            "min_arm_balance",
            "min_endpoint_turn_deg",
            "min_vertex_turn_deg",
            "max_vertex_turn_deg",
            "min_notch_turn_deg",
        }
    ),
)


@dataclass(frozen=True)
class ContourMetrics:
    contour_points: int
    area: int
    perimeter: float
    aspect_ratio: float
    circularity: float
    occupancy: float
    solidity: float
    radial_cv: float
    concavity_ratio: float
    direction_entropy: float
    dominant_axis_ratio: float
    turn_transition_ratio: float
    vertex_angle_deg: float | None
    arm_balance: float | None
    direction_window_points: int
    endpoint_turn_angles: tuple[float, ...]
    vertex_turn_angle: float | None
    notch_turn_angle: float | None
    max_negative_turn_deg: float
    negative_turn_window_points: int
    vertex_point: tuple[int, int] | None
    deepest_point: tuple[int, int] | None
    arm_tip_points: tuple[tuple[int, int], ...]
    foreground_mean: float | None
    background_mean: float | None
    gray_contrast: float | None
    foreground_samples: int
    background_samples: int
    boundary_clipped: bool
    contour_closed: bool
    contour_overflow: bool


@dataclass(frozen=True)
class ContourDecision:
    passed: bool
    failures: tuple[str, ...]


def _convex_hull(points: Sequence[tuple[int, int]]) -> list[tuple[int, int]]:
    unique = sorted(set(points))
    if len(unique) <= 1:
        return unique

    def cross(
        origin: tuple[int, int],
        first: tuple[int, int],
        second: tuple[int, int],
    ) -> int:
        return (
            (first[0] - origin[0]) * (second[1] - origin[1])
            - (first[1] - origin[1]) * (second[0] - origin[0])
        )

    lower: list[tuple[int, int]] = []
    for point in unique:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0:
            lower.pop()
        lower.append(point)
    upper: list[tuple[int, int]] = []
    for point in reversed(unique):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0:
            upper.pop()
        upper.append(point)
    return lower[:-1] + upper[:-1]


def _polygon_area(points: Sequence[tuple[int, int]]) -> float:
    if len(points) < 3:
        return 0.0
    doubled = 0
    for first, second in zip(points, (*points[1:], points[0])):
        doubled += first[0] * second[1] - first[1] * second[0]
    return abs(doubled) * 0.5


def _polygon_perimeter(points: Sequence[tuple[int, int]]) -> float:
    if len(points) < 2:
        return 0.0
    return sum(
        math.hypot(second[0] - first[0], second[1] - first[1])
        for first, second in zip(points, (*points[1:], points[0]))
    )


def _point_segment_distance(
    point: tuple[int, int],
    start: tuple[int, int],
    end: tuple[int, int],
) -> float:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length_sq = dx * dx + dy * dy
    if length_sq == 0:
        return math.hypot(point[0] - start[0], point[1] - start[1])
    projection = (
        (point[0] - start[0]) * dx + (point[1] - start[1]) * dy
    ) / length_sq
    projection = max(0.0, min(1.0, projection))
    nearest_x = start[0] + projection * dx
    nearest_y = start[1] + projection * dy
    return math.hypot(point[0] - nearest_x, point[1] - nearest_y)


@dataclass(frozen=True)
class _TurnRegion:
    indices: tuple[int, ...]
    peak_index: int
    peak_angle: float


def _signed_direction_differences(
    points: Sequence[tuple[int, int]],
    requested_window: int,
) -> tuple[int, list[float]]:
    """Measure signed tangent-direction change at every point on a closed contour."""
    count = len(points)
    if count < 5:
        return 0, [0.0] * count
    maximum_window = max(2, (count - 1) // 4)
    window = max(2, min(int(requested_window), maximum_window))
    area_twice = sum(
        first[0] * second[1] - first[1] * second[0]
        for first, second in zip(points, (*points[1:], points[0]))
    )
    orientation = 1.0 if area_twice >= 0 else -1.0
    unit_edges: list[tuple[float, float]] = []
    for first, second in zip(points, (*points[1:], points[0])):
        dx = second[0] - first[0]
        dy = second[1] - first[1]
        length = math.hypot(dx, dy)
        unit_edges.append(
            (dx / length, dy / length) if length > 0.0 else (0.0, 0.0)
        )

    signed_angles: list[float] = []
    for index in range(count):
        before_x = before_y = after_x = after_y = 0.0
        for offset in range(window):
            before = unit_edges[(index - window + offset) % count]
            after = unit_edges[(index + offset) % count]
            before_x += before[0]
            before_y += before[1]
            after_x += after[0]
            after_y += after[1]
        before_length = math.hypot(before_x, before_y)
        after_length = math.hypot(after_x, after_y)
        if before_length <= 1e-9 or after_length <= 1e-9:
            signed_angles.append(0.0)
            continue
        cosine = (
            before_x * after_x + before_y * after_y
        ) / (before_length * after_length)
        angle = math.degrees(math.acos(max(-1.0, min(1.0, cosine))))
        cross = before_x * after_y - before_y * after_x
        signed_angles.append(angle if orientation * cross >= 0.0 else -angle)
    return window, signed_angles


def _circular_regions(mask: Sequence[bool]) -> list[tuple[int, ...]]:
    count = len(mask)
    if count == 0 or not any(mask):
        return []
    if all(mask):
        return [tuple(range(count))]
    boundary = next(index for index, enabled in enumerate(mask) if not enabled)
    regions: list[tuple[int, ...]] = []
    current: list[int] = []
    for step in range(1, count + 1):
        index = (boundary + step) % count
        if mask[index]:
            current.append(index)
        elif current:
            regions.append(tuple(current))
            current = []
    if current:
        regions.append(tuple(current))
    return regions


def _build_turn_regions(
    signed_angles: Sequence[float],
    minimum_magnitude: float,
    convex: bool,
) -> list[_TurnRegion]:
    mask = [
        angle >= minimum_magnitude if convex else angle <= -minimum_magnitude
        for angle in signed_angles
    ]
    regions: list[_TurnRegion] = []
    for indices in _circular_regions(mask):
        peak_index = (
            max(indices, key=signed_angles.__getitem__)
            if convex
            else min(indices, key=signed_angles.__getitem__)
        )
        regions.append(
            _TurnRegion(indices, peak_index, signed_angles[peak_index])
        )
    return regions


def _region_point(
    points: Sequence[tuple[int, int]],
    signed_angles: Sequence[float],
    region: _TurnRegion,
) -> tuple[int, int]:
    peak_magnitude = abs(region.peak_angle)
    selected = [
        index
        for index in region.indices
        if abs(signed_angles[index]) >= peak_magnitude * 0.80
    ]
    if not selected:
        return points[region.peak_index]
    weights = [abs(signed_angles[index]) for index in selected]
    total = sum(weights)
    return (
        int(round(sum(points[index][0] * weight for index, weight in zip(selected, weights)) / total)),
        int(round(sum(points[index][1] * weight for index, weight in zip(selected, weights)) / total)),
    )


def _extract_arrow_turn_geometry(
    points: Sequence[tuple[int, int]],
    thresholds: ContourThresholds,
) -> tuple[
    int,
    tuple[tuple[int, int], ...],
    tuple[float, ...],
    tuple[int, int] | None,
    float | None,
    tuple[int, int] | None,
    float | None,
    float,
    float | None,
    float | None,
]:
    window, signed_angles = _signed_direction_differences(
        points,
        thresholds.direction_window_points,
    )
    if window == 0:
        return 0, (), (), None, None, None, None, 0.0, None, None
    max_negative_turn = max(0.0, -min(signed_angles, default=0.0))
    convex_regions = _build_turn_regions(
        signed_angles,
        thresholds.min_vertex_turn_deg,
        convex=True,
    )
    endpoint_regions = sorted(
        (
            region for region in convex_regions
            if region.peak_angle >= thresholds.min_endpoint_turn_deg
        ),
        key=lambda region: region.peak_angle,
        reverse=True,
    )
    endpoint_regions = endpoint_regions[:2]
    endpoints = tuple(
        _region_point(points, signed_angles, region)
        for region in endpoint_regions
    )
    endpoint_angles = tuple(region.peak_angle for region in endpoint_regions)

    enabled = set(thresholds.enabled)
    effective_vertex_min = (
        thresholds.min_vertex_turn_deg
        if "min_vertex_turn_deg" in enabled
        else 0.0
    )
    effective_vertex_max = (
        thresholds.max_vertex_turn_deg
        if "max_vertex_turn_deg" in enabled
        else 180.0
    )
    vertex_regions = [
        region for region in convex_regions
        if effective_vertex_min
        <= region.peak_angle
        <= effective_vertex_max
        and (
            thresholds.min_endpoint_turn_deg <= 0.0
            or region.peak_angle < thresholds.min_endpoint_turn_deg
        )
    ]
    notch_regions = sorted(
        _build_turn_regions(
            signed_angles,
            thresholds.min_notch_turn_deg,
            convex=False,
        ),
        key=lambda region: region.peak_angle,
    )
    notch_region = notch_regions[0] if notch_regions else None
    notch_point = (
        _region_point(points, signed_angles, notch_region)
        if notch_region is not None
        else None
    )
    notch_angle = notch_region.peak_angle if notch_region is not None else None

    vertex_region: _TurnRegion | None = None
    if vertex_regions:
        expected_turn = 110.0

        def vertex_score(region: _TurnRegion) -> tuple[float, float]:
            point = _region_point(points, signed_angles, region)
            notch_distance = (
                math.dist(point, notch_point)
                if notch_point is not None
                else 0.0
            )
            return (
                -notch_distance,
                -abs(region.peak_angle - expected_turn),
            )

        vertex_region = max(vertex_regions, key=vertex_score)
    vertex_point = (
        _region_point(points, signed_angles, vertex_region)
        if vertex_region is not None
        else None
    )
    vertex_turn = vertex_region.peak_angle if vertex_region is not None else None
    physical_angle = 180.0 - vertex_turn if vertex_turn is not None else None
    arm_balance: float | None = None
    if vertex_point is not None and len(endpoints) == 2:
        first_length = math.dist(vertex_point, endpoints[0])
        second_length = math.dist(vertex_point, endpoints[1])
        if first_length > 0.0 and second_length > 0.0:
            arm_balance = min(first_length, second_length) / max(
                first_length, second_length
            )
    return (
        window,
        endpoints,
        endpoint_angles,
        vertex_point,
        vertex_turn,
        notch_point,
        notch_angle,
        max_negative_turn,
        physical_angle,
        arm_balance,
    )


def _gray_statistics(
    component: ConnectedComponent,
    gray: Image.Image,
    binary: Image.Image,
) -> tuple[float | None, float | None, float | None, int, int]:
    gray = gray.convert("L")
    binary = binary.convert("L")
    if gray.size != binary.size:
        raise ValueError("灰度图与二值图尺寸不一致")
    width, height = gray.size
    padding = max(2, min(8, max(component.width, component.height) // 4))
    min_x = max(0, component.min_x - padding)
    min_y = max(0, component.min_y - padding)
    max_x = min(width - 1, component.max_x + padding)
    max_y = min(height - 1, component.max_y + padding)
    gray_pixels = gray.load()
    binary_pixels = binary.load()
    foreground_sum = 0
    foreground_count = 0
    background_sum = 0
    background_count = 0
    for y in range(min_y, max_y + 1):
        for x in range(min_x, max_x + 1):
            is_inside_box = (
                component.min_x <= x <= component.max_x
                and component.min_y <= y <= component.max_y
            )
            if is_inside_box and binary_pixels[x, y]:
                foreground_sum += int(gray_pixels[x, y])
                foreground_count += 1
            elif not binary_pixels[x, y]:
                background_sum += int(gray_pixels[x, y])
                background_count += 1
    foreground_mean = (
        foreground_sum / foreground_count if foreground_count else None
    )
    background_mean = (
        background_sum / background_count if background_count else None
    )
    contrast = (
        foreground_mean - background_mean
        if foreground_mean is not None and background_mean is not None
        else None
    )
    return (
        foreground_mean,
        background_mean,
        contrast,
        foreground_count,
        background_count,
    )


def calculate_contour_metrics(
    component: ConnectedComponent,
    gray: Image.Image,
    binary: Image.Image,
    turn_thresholds: ContourThresholds | None = None,
    negative_turn_window_points: int | None = None,
    is_valid_pixel: Callable[[int, int], bool] | None = None,
) -> ContourMetrics:
    points = list(component.contour_points)
    if len(points) >= 2 and points[0] == points[-1]:
        points.pop()
    hull = _convex_hull(points)
    hull_perimeter = _polygon_perimeter(hull)
    # Pixel centres define the hull. The half-perimeter and one-pixel terms turn
    # its continuous area into a closer approximation of the enclosed pixel cells.
    hull_cell_area = _polygon_area(hull) + hull_perimeter * 0.5 + 1.0
    solidity = (
        min(1.0, component.area / hull_cell_area)
        if hull_cell_area > 0.0
        else 0.0
    )
    diagonal = math.hypot(component.width, component.height)
    max_concavity = 0.0
    if len(hull) >= 2:
        hull_edges = list(zip(hull, (*hull[1:], hull[0])))
        max_concavity = max(
            (
                min(
                    _point_segment_distance(point, start, end)
                    for start, end in hull_edges
                )
                for point in points
            ),
            default=0.0,
        )
    concavity_ratio = max_concavity / diagonal if diagonal > 0.0 else 0.0
    geometry_thresholds = turn_thresholds or ARROW_DEFAULT_THRESHOLDS
    (
        effective_window,
        arm_tip_points,
        endpoint_turn_angles,
        vertex_point,
        vertex_turn_angle,
        deepest_point,
        notch_turn_angle,
        max_negative_turn_deg,
        vertex_angle_deg,
        arm_balance,
    ) = _extract_arrow_turn_geometry(points, geometry_thresholds)
    effective_negative_window = effective_window
    if (
        negative_turn_window_points is not None
        and int(negative_turn_window_points) != effective_window
    ):
        effective_negative_window, negative_angles = _signed_direction_differences(
            points,
            int(negative_turn_window_points),
        )
        max_negative_turn_deg = max(
            0.0,
            -min(negative_angles, default=0.0),
        )

    radial_distances = [
        math.hypot(x - component.centroid_x, y - component.centroid_y)
        for x, y in points
    ]
    radial_mean = (
        sum(radial_distances) / len(radial_distances)
        if radial_distances
        else 0.0
    )
    radial_cv = 0.0
    if radial_mean > 0.0:
        radial_variance = sum(
            (distance - radial_mean) ** 2 for distance in radial_distances
        ) / len(radial_distances)
        radial_cv = min(1.0, math.sqrt(radial_variance) / radial_mean)

    direction_total = sum(component.direction_counts)
    direction_entropy = 0.0
    dominant_axis_ratio = 0.0
    if direction_total > 0:
        for count in component.direction_counts:
            if count:
                probability = count / direction_total
                direction_entropy -= probability * math.log(probability)
        direction_entropy /= math.log(8.0)
        axis_counts = [
            component.direction_counts[index]
            + component.direction_counts[index + 4]
            for index in range(4)
        ]
        dominant_axis_ratio = sum(sorted(axis_counts, reverse=True)[:2]) / direction_total
    direction_sequence = component.contour_directions
    turn_transition_ratio = 0.0
    if direction_sequence:
        transitions = sum(
            first != second
            for first, second in zip(
                direction_sequence,
                (*direction_sequence[1:], direction_sequence[0]),
            )
        )
        turn_transition_ratio = transitions / len(direction_sequence)

    foreground_mean, background_mean, contrast, foreground_count, background_count = (
        _gray_statistics(component, gray, binary)
    )
    width, height = gray.size
    boundary_clipped = (
        component.min_x <= 0
        or component.min_y <= 0
        or component.max_x >= width - 1
        or component.max_y >= height - 1
    )
    if not boundary_clipped and is_valid_pixel is not None:
        boundary_clipped = any(
            not is_valid_pixel(point_x + offset_x, point_y + offset_y)
            for point_x, point_y in points
            for offset_x, offset_y in ((0, -1), (1, 0), (0, 1), (-1, 0))
        )

    return ContourMetrics(
        contour_points=len(points),
        area=component.area,
        perimeter=component.contour_perimeter,
        aspect_ratio=(
            min(component.width, component.height) / max(component.width, component.height)
            if component.width and component.height
            else 0.0
        ),
        circularity=max(0.0, min(1.0, component.circularity)),
        occupancy=max(0.0, min(1.0, component.occupancy)),
        solidity=max(0.0, min(1.0, solidity)),
        radial_cv=max(0.0, min(1.0, radial_cv)),
        concavity_ratio=max(0.0, min(1.0, concavity_ratio)),
        direction_entropy=max(0.0, min(1.0, direction_entropy)),
        dominant_axis_ratio=max(0.0, min(1.0, dominant_axis_ratio)),
        turn_transition_ratio=max(0.0, min(1.0, turn_transition_ratio)),
        vertex_angle_deg=vertex_angle_deg,
        arm_balance=arm_balance,
        direction_window_points=effective_window,
        endpoint_turn_angles=endpoint_turn_angles,
        vertex_turn_angle=vertex_turn_angle,
        notch_turn_angle=notch_turn_angle,
        max_negative_turn_deg=max_negative_turn_deg,
        negative_turn_window_points=effective_negative_window,
        vertex_point=vertex_point,
        deepest_point=deepest_point,
        arm_tip_points=arm_tip_points,
        foreground_mean=foreground_mean,
        background_mean=background_mean,
        gray_contrast=contrast,
        foreground_samples=foreground_count,
        background_samples=background_count,
        boundary_clipped=boundary_clipped,
        contour_closed=component.contour_closed,
        contour_overflow=component.contour_overflow,
    )


def evaluate_contour_metrics(
    metrics: ContourMetrics,
    thresholds: ContourThresholds,
) -> ContourDecision:
    thresholds.validate()
    enabled = set(thresholds.enabled)
    failures: list[str] = []
    if not metrics.contour_closed:
        failures.append("轮廓未闭合")
    if metrics.contour_overflow:
        failures.append("轮廓跟踪步数溢出")
    if metrics.boundary_clipped:
        failures.append("轮廓接触图像有效边界")
    if (
        "min_contour_points" in enabled
        and metrics.contour_points < thresholds.min_contour_points
    ):
        failures.append(
            f"轮廓点 {metrics.contour_points} < {thresholds.min_contour_points}"
        )
    if "min_gray_contrast" in enabled:
        if metrics.gray_contrast is None:
            failures.append("缺少可用的前景或背景灰度样本")
        elif metrics.gray_contrast < thresholds.min_gray_contrast:
            failures.append(
                f"前景/背景灰度差 {metrics.gray_contrast:.3f} < "
                f"{thresholds.min_gray_contrast:.3f}"
            )
    checks = (
        ("min_aspect_ratio", "长短边比例", metrics.aspect_ratio, "min"),
        ("max_aspect_ratio", "长短边比例", metrics.aspect_ratio, "max"),
        ("min_circularity", "圆度", metrics.circularity, "min"),
        ("max_circularity", "圆度", metrics.circularity, "max"),
        ("min_solidity", "凸包填充率", metrics.solidity, "min"),
        ("max_solidity", "凸包填充率", metrics.solidity, "max"),
        ("min_radial_cv", "径向波动", metrics.radial_cv, "min"),
        ("max_radial_cv", "径向波动", metrics.radial_cv, "max"),
        ("min_concavity_ratio", "最大凹陷比例", metrics.concavity_ratio, "min"),
        ("max_concavity_ratio", "最大凹陷比例", metrics.concavity_ratio, "max"),
        ("min_direction_entropy", "方向熵", metrics.direction_entropy, "min"),
        ("max_direction_entropy", "方向熵", metrics.direction_entropy, "max"),
        ("min_dominant_axis_ratio", "两组主方向占比", metrics.dominant_axis_ratio, "min"),
        ("max_dominant_axis_ratio", "两组主方向占比", metrics.dominant_axis_ratio, "max"),
    )
    for field_name, label, value, bound_kind in checks:
        if field_name not in enabled:
            continue
        bound = float(getattr(thresholds, field_name))
        if bound_kind == "min" and value < bound:
            failures.append(f"{label} {value:.3f} < {bound:.3f}")
        elif bound_kind == "max" and value > bound:
            failures.append(f"{label} {value:.3f} > {bound:.3f}")
    if (
        "max_negative_turn_deg" in enabled
        and metrics.max_negative_turn_deg > thresholds.max_negative_turn_deg
    ):
        failures.append(
            f"最大反向转角 {metrics.max_negative_turn_deg:.3f} > "
            f"{thresholds.max_negative_turn_deg:.3f}"
        )
    angle_is_enabled = bool(
        {"min_vertex_angle_deg", "max_vertex_angle_deg"} & enabled
    )
    if angle_is_enabled:
        if metrics.vertex_angle_deg is None:
            failures.append("无法根据外顶点方向峰计算箭头物理夹角")
        elif (
            "min_vertex_angle_deg" in enabled
            and metrics.vertex_angle_deg < thresholds.min_vertex_angle_deg
        ):
            failures.append(
                f"候选顶点夹角 {metrics.vertex_angle_deg:.3f} < "
                f"{thresholds.min_vertex_angle_deg:.3f}"
            )
        elif (
            "max_vertex_angle_deg" in enabled
            and metrics.vertex_angle_deg > thresholds.max_vertex_angle_deg
        ):
            failures.append(
                f"候选顶点夹角 {metrics.vertex_angle_deg:.3f} > "
                f"{thresholds.max_vertex_angle_deg:.3f}"
            )
    if "min_arm_balance" in enabled:
        if metrics.arm_balance is None:
            failures.append("未形成外顶点与两个端点，无法计算两臂平衡度")
        elif metrics.arm_balance < thresholds.min_arm_balance:
            failures.append(
                f"候选两臂平衡度 {metrics.arm_balance:.3f} < "
                f"{thresholds.min_arm_balance:.3f}"
            )
    if "min_endpoint_turn_deg" in enabled:
        if len(metrics.endpoint_turn_angles) < 2:
            failures.append(
                f"只找到 {len(metrics.endpoint_turn_angles)} 个端点方向峰，需要2个"
            )
    vertex_turn_enabled = bool(
        {"min_vertex_turn_deg", "max_vertex_turn_deg"} & enabled
    )
    if vertex_turn_enabled:
        if metrics.vertex_turn_angle is None:
            failures.append("未找到外顶点方向峰")
        elif (
            "min_vertex_turn_deg" in enabled
            and metrics.vertex_turn_angle < thresholds.min_vertex_turn_deg
        ):
            failures.append(
                f"外顶点方向差 {metrics.vertex_turn_angle:.3f} < "
                f"{thresholds.min_vertex_turn_deg:.3f}"
            )
        elif (
            "max_vertex_turn_deg" in enabled
            and metrics.vertex_turn_angle > thresholds.max_vertex_turn_deg
        ):
            failures.append(
                f"外顶点方向差 {metrics.vertex_turn_angle:.3f} > "
                f"{thresholds.max_vertex_turn_deg:.3f}"
            )
    if "min_notch_turn_deg" in enabled:
        if metrics.notch_turn_angle is None:
            failures.append("未找到反向内凹口方向峰")
        elif abs(metrics.notch_turn_angle) < thresholds.min_notch_turn_deg:
            failures.append(
                f"内凹口反向差 {abs(metrics.notch_turn_angle):.3f} < "
                f"{thresholds.min_notch_turn_deg:.3f}"
            )
    return ContourDecision(not failures, tuple(failures))


def self_test() -> None:
    circle = Image.new("L", (96, 96), 0)
    circle_draw = ImageDraw.Draw(circle)
    circle_draw.ellipse((26, 26, 70, 70), fill=255)
    circle_component = find_contour_components(circle)[0]
    circle_metrics = calculate_contour_metrics(circle_component, circle, circle)
    assert circle_metrics.contour_closed
    assert circle_metrics.circularity > 0.5
    assert circle_metrics.solidity > 0.8

    chevron = Image.new("L", (96, 96), 0)
    chevron_draw = ImageDraw.Draw(chevron)
    chevron_draw.line(((20, 25), (48, 68), (76, 25)), fill=255, width=5)
    chevron_component = find_contour_components(chevron)[0]
    chevron_metrics = calculate_contour_metrics(chevron_component, chevron, chevron)
    assert chevron_metrics.contour_closed
    assert chevron_metrics.concavity_ratio > circle_metrics.concavity_ratio
    assert chevron_metrics.max_negative_turn_deg > circle_metrics.max_negative_turn_deg
    assert chevron_metrics.solidity < circle_metrics.solidity
    assert len(chevron_metrics.endpoint_turn_angles) == 2
    assert all(
        angle >= ARROW_DEFAULT_THRESHOLDS.min_endpoint_turn_deg
        for angle in chevron_metrics.endpoint_turn_angles
    )
    assert chevron_metrics.vertex_turn_angle is not None
    assert chevron_metrics.notch_turn_angle is not None
    assert chevron_metrics.notch_turn_angle < 0.0
    assert evaluate_contour_metrics(
        circle_metrics, BEACON_DEFAULT_THRESHOLDS
    ).passed
    assert evaluate_contour_metrics(
        chevron_metrics, ARROW_DEFAULT_THRESHOLDS
    ).passed

    rotation_source = Image.new("L", (128, 128), 0)
    rotation_draw = ImageDraw.Draw(rotation_source)
    rotation_draw.line(((28, 35), (64, 91), (100, 35)), fill=255, width=7)
    for angle in range(0, 360, 15):
        rotated = rotation_source.rotate(
            angle,
            resample=Image.Resampling.NEAREST,
            fillcolor=0,
        )
        rotated_component = find_contour_components(rotated)[0]
        rotated_metrics = calculate_contour_metrics(
            rotated_component,
            rotated,
            rotated,
            ARROW_DEFAULT_THRESHOLDS,
        )
        assert evaluate_contour_metrics(
            rotated_metrics,
            ARROW_DEFAULT_THRESHOLDS,
        ).passed


if __name__ == "__main__":
    self_test()

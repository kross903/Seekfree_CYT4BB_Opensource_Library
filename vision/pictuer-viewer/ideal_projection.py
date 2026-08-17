#!/usr/bin/env python3
"""Physical target projection for the standalone MT9V03 picture viewer."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import re


Point2D = tuple[float, float]
Matrix3 = tuple[tuple[float, float, float], ...]


def wrap_degrees(angle_deg: float) -> float:
    return (angle_deg + 180.0) % 360.0 - 180.0


def build_ideal_chevron_points(
    center: Point2D,
    arm_length_cm: float,
    opening_angle_deg: float,
    opening_direction_deg: float,
) -> dict[str, Point2D]:
    """Build a V-shaped arrow around the physical triangle centroid."""
    if arm_length_cm <= 0.0:
        raise ValueError("箭头单臂长度必须大于0")
    if not 1.0 <= opening_angle_deg < 180.0:
        raise ValueError("箭头夹角必须在1到180度之间")

    half_angle = math.radians(opening_angle_deg * 0.5)
    triangle_height = arm_length_cm * math.cos(half_angle)
    half_base = arm_length_cm * math.sin(half_angle)
    apex_direction = math.radians(opening_direction_deg - 180.0)
    forward = (math.cos(apex_direction), math.sin(apex_direction))
    left = (-forward[1], forward[0])
    apex = (
        center[0] + (2.0 * triangle_height / 3.0) * forward[0],
        center[1] + (2.0 * triangle_height / 3.0) * forward[1],
    )
    base_center = (
        center[0] - (triangle_height / 3.0) * forward[0],
        center[1] - (triangle_height / 3.0) * forward[1],
    )
    return {
        "center": center,
        "apex": apex,
        "left_tip": (
            base_center[0] + half_base * left[0],
            base_center[1] + half_base * left[1],
        ),
        "right_tip": (
            base_center[0] - half_base * left[0],
            base_center[1] - half_base * left[1],
        ),
    }


def build_ideal_beacon_points(
    center: Point2D,
    diameter_cm: float,
    point_count: int = 48,
) -> tuple[Point2D, ...]:
    """Sample a horizontal circular beacon in physical coordinates."""
    if diameter_cm <= 0.0:
        raise ValueError("信标直径必须大于0")
    point_count = max(12, int(point_count))
    radius = diameter_cm * 0.5
    return tuple(
        (
            center[0] + radius * math.cos(2.0 * math.pi * index / point_count),
            center[1] + radius * math.sin(2.0 * math.pi * index / point_count),
        )
        for index in range(point_count)
    )


def build_ideal_beacon_cardinal_points(
    center: Point2D,
    diameter_cm: float,
) -> dict[str, Point2D]:
    """Return the center and four physical points used by the firmware."""
    if diameter_cm <= 0.0:
        raise ValueError("信标直径必须大于0")
    radius = diameter_cm * 0.5
    return {
        "center": center,
        "left": (center[0], center[1] - radius),
        "right": (center[0], center[1] + radius),
        "up": (center[0] + radius, center[1]),
        "down": (center[0] - radius, center[1]),
    }


def build_projected_ellipse_outline(
    center: Point2D,
    left: Point2D,
    right: Point2D,
    up: Point2D,
    down: Point2D,
    point_count: int = 32,
) -> tuple[Point2D, ...]:
    """Interpolate an image ellipse after only five physical projections."""
    point_count = max(12, int(point_count))
    ex = ((right[0] - left[0]) * 0.5, (right[1] - left[1]) * 0.5)
    ey = ((down[0] - up[0]) * 0.5, (down[1] - up[1]) * 0.5)
    return tuple(
        (
            center[0] + ex[0] * math.cos(2.0 * math.pi * index / point_count)
            + ey[0] * math.sin(2.0 * math.pi * index / point_count),
            center[1] + ex[1] * math.cos(2.0 * math.pi * index / point_count)
            + ey[1] * math.sin(2.0 * math.pi * index / point_count),
        )
        for index in range(point_count)
    )


@dataclass(frozen=True)
class ProjectionCalibration:
    image_width: int
    image_height: int
    output_width: int
    output_height: int
    fx: float
    fy: float
    cx: float
    cy: float
    offset_x_cm: float
    offset_y_cm: float
    offset_z_cm: float

    @classmethod
    def from_header(cls, header_path: Path) -> "ProjectionCalibration":
        source = header_path.read_text(encoding="utf-8")

        def macro(name: str) -> float:
            match = re.search(
                rf"^\s*#define\s+{re.escape(name)}\s+\(?\s*([-+0-9.eE]+)",
                source,
                re.MULTILINE,
            )
            if match is None:
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
            offset_x_cm=macro("CAMERA_OFFSET_X_CM"),
            offset_y_cm=macro("CAMERA_OFFSET_Y_CM"),
            offset_z_cm=macro("CAMERA_OFFSET_Z_CM"),
        )


class TargetPlaneProjector:
    """Project between a horizontal target plane and the rectified image."""

    def __init__(
        self,
        calibration: ProjectionCalibration,
        mount_roll_deg: float = 0.0,
        mount_pitch_deg: float = 0.0,
        mount_yaw_deg: float = 0.0,
    ) -> None:
        self.calibration = calibration
        self.mount_angles = (0.0, 0.0, 0.0)
        self.mount_rotation: Matrix3
        self.set_mount_angles(mount_roll_deg, mount_pitch_deg, mount_yaw_deg)

    def set_mount_angles(
        self,
        roll_deg: float,
        pitch_deg: float,
        yaw_deg: float,
    ) -> None:
        angles = (float(roll_deg), float(pitch_deg), float(yaw_deg))
        if not all(math.isfinite(value) for value in angles):
            raise ValueError("相机安装角必须是有限数值")
        if angles == self.mount_angles and hasattr(self, "mount_rotation"):
            return
        self.mount_angles = angles
        self.mount_rotation = self._rotation(
            angles[0], angles[1], angles[2],
        )

    @staticmethod
    def _rotation(roll_deg: float, pitch_deg: float, yaw_deg: float) -> Matrix3:
        roll = math.radians(roll_deg)
        pitch = math.radians(pitch_deg)
        yaw = math.radians(yaw_deg)
        sr, cr = math.sin(roll), math.cos(roll)
        sp, cp = math.sin(pitch), math.cos(pitch)
        sy, cy = math.sin(yaw), math.cos(yaw)
        return (
            (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr),
        )

    @staticmethod
    def _attitude_rotation(roll_deg: float, pitch_deg: float) -> Matrix3:
        # The target-plane coordinates rotate with UAV yaw. Target orientation
        # therefore uses car yaw minus UAV yaw, while projection needs roll/pitch.
        roll = math.radians(roll_deg)
        pitch = math.radians(pitch_deg)
        sr, cr = math.sin(roll), math.cos(roll)
        sp, cp = math.sin(pitch), math.cos(pitch)
        return (
            (cp, 0.0, sp),
            (sr * sp, cr, -sr * cp),
            (-cr * sp, sr, cr * cp),
        )

    @staticmethod
    def _mat_vec(matrix: Matrix3, vector: tuple[float, float, float]) -> tuple[float, float, float]:
        return tuple(
            sum(matrix[row][column] * vector[column] for column in range(3))
            for row in range(3)
        )  # type: ignore[return-value]

    @staticmethod
    def _transpose_mat_vec(
        matrix: Matrix3,
        vector: tuple[float, float, float],
    ) -> tuple[float, float, float]:
        return tuple(
            sum(matrix[row][column] * vector[row] for row in range(3))
            for column in range(3)
        )  # type: ignore[return-value]

    def image_to_plane(
        self,
        image_point: Point2D,
        roll_deg: float,
        pitch_deg: float,
        height_cm: float,
        plane_height_cm: float,
    ) -> Point2D | None:
        calibration = self.calibration
        image_x, image_y = image_point
        if not (
            0.0 <= image_x < calibration.output_width
            and 0.0 <= image_y < calibration.output_height
        ):
            return None

        center_x = (calibration.output_width - 1.0) * 0.5
        center_y = (calibration.output_height - 1.0) * 0.5
        ray_camera = (
            (image_y - center_y) / calibration.fy,
            (image_x - center_x) / calibration.fx,
            1.0,
        )
        ray_body = self._mat_vec(self.mount_rotation, ray_camera)
        ray_plane = self._mat_vec(
            self._attitude_rotation(roll_deg, pitch_deg), ray_body,
        )
        camera_z = -height_cm + calibration.offset_z_cm
        plane_z = -plane_height_cm
        if abs(ray_plane[2]) < 1e-9:
            return None
        distance = (plane_z - camera_z) / ray_plane[2]
        if distance <= 0.0:
            return None
        return (
            -(calibration.offset_x_cm + distance * ray_plane[0]),
            -(calibration.offset_y_cm + distance * ray_plane[1]),
        )

    def plane_to_image(
        self,
        plane_point: Point2D,
        roll_deg: float,
        pitch_deg: float,
        height_cm: float,
        plane_height_cm: float,
    ) -> Point2D | None:
        calibration = self.calibration
        camera_z = -height_cm + calibration.offset_z_cm
        vector_plane = (
            -plane_point[0] - calibration.offset_x_cm,
            -plane_point[1] - calibration.offset_y_cm,
            -plane_height_cm - camera_z,
        )
        vector_body = self._transpose_mat_vec(
            self._attitude_rotation(roll_deg, pitch_deg), vector_plane,
        )
        vector_camera = self._transpose_mat_vec(self.mount_rotation, vector_body)
        if vector_camera[2] <= 1e-9:
            return None
        center_x = (calibration.output_width - 1.0) * 0.5
        center_y = (calibration.output_height - 1.0) * 0.5
        return (
            center_x + calibration.fx * vector_camera[1] / vector_camera[2],
            center_y + calibration.fy * vector_camera[0] / vector_camera[2],
        )

    def is_inside_image(self, point: Point2D) -> bool:
        return (
            0.0 <= point[0] < self.calibration.output_width
            and 0.0 <= point[1] < self.calibration.output_height
        )


def self_test(calibration_path: Path) -> None:
    ideal_points = build_ideal_chevron_points((0.0, 0.0), 19.0, 70.0, 180.0)
    assert abs(math.dist(ideal_points["apex"], ideal_points["left_tip"]) - 19.0) < 1e-6
    assert abs(math.dist(ideal_points["apex"], ideal_points["right_tip"]) - 19.0) < 1e-6
    left_vector = tuple(
        ideal_points["left_tip"][index] - ideal_points["apex"][index]
        for index in range(2)
    )
    right_vector = tuple(
        ideal_points["right_tip"][index] - ideal_points["apex"][index]
        for index in range(2)
    )
    included_angle = math.degrees(math.acos(
        sum(left_vector[index] * right_vector[index] for index in range(2))
        / (math.hypot(*left_vector) * math.hypot(*right_vector))
    ))
    assert abs(included_angle - 70.0) < 1e-6

    beacon_points = build_ideal_beacon_points((2.0, -3.0), 10.0, 48)
    assert len(beacon_points) == 48
    assert abs(math.dist(beacon_points[0], beacon_points[24]) - 10.0) < 1e-6
    cardinal = build_ideal_beacon_cardinal_points((2.0, -3.0), 10.0)
    assert set(cardinal) == {"center", "left", "right", "up", "down"}
    ellipse = build_projected_ellipse_outline(
        (10.0, 10.0), (6.0, 10.0), (14.0, 10.0),
        (10.0, 7.0), (10.0, 13.0),
    )
    assert len(ellipse) == 32
    assert abs(ellipse[0][0] - 14.0) < 1e-6

    calibration = ProjectionCalibration.from_header(calibration_path)
    assert (calibration.output_width, calibration.output_height) == (376, 240)
    projector = TargetPlaneProjector(calibration)

    def projected_beacon_width(image_x: float) -> float:
        beacon_center = projector.image_to_plane(
            (image_x, 120.0), 0.0, 0.0, 140.0, 0.0,
        )
        assert beacon_center is not None
        projected_beacon = [
            projector.plane_to_image(point, 0.0, 0.0, 140.0, 0.0)
            for point in build_ideal_beacon_points(beacon_center, 10.0)
        ]
        assert all(point is not None for point in projected_beacon)
        image_x_values = [point[0] for point in projected_beacon if point is not None]
        return max(image_x_values) - min(image_x_values)

    assert projector.mount_angles == (0.0, 0.0, 0.0)
    widths = [projected_beacon_width(x) for x in (80.0, 188.0, 296.0)]
    assert max(widths) - min(widths) < 1e-6

    projector.set_mount_angles(-15.0, 0.0, 0.0)
    assert projector.mount_angles == (-15.0, 0.0, 0.0)
    tilted_widths = [projected_beacon_width(x) for x in (80.0, 188.0, 296.0)]
    assert max(tilted_widths) - min(tilted_widths) > 0.1
    projector.set_mount_angles(0.0, 0.0, 0.0)
    image_point = (190.0, 125.0)
    plane_point = projector.image_to_plane(image_point, 3.0, -2.0, 100.0, 17.0)
    assert plane_point is not None
    reprojected = projector.plane_to_image(plane_point, 3.0, -2.0, 100.0, 17.0)
    assert reprojected is not None
    assert math.dist(image_point, reprojected) < 1e-6
    assert wrap_degrees(190.0) == -170.0

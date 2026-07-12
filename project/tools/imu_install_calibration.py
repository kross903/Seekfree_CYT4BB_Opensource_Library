#!/usr/bin/env python3
"""Solve the IMU sensor-to-body installation matrix from six VOFA CSV logs."""

import argparse
import csv
from pathlib import Path

import numpy as np


POSES = {
    "px": np.array([1.0, 0.0, 0.0]),
    "nx": np.array([-1.0, 0.0, 0.0]),
    "py": np.array([0.0, 1.0, 0.0]),
    "ny": np.array([0.0, -1.0, 0.0]),
    "pz": np.array([0.0, 0.0, 1.0]),
    "nz": np.array([0.0, 0.0, -1.0]),
}

MIN_VALID_ROWS = 50
SEGMENT_BREAK_ANGLE_DEG = 25.0


def read_mean(path: Path, columns: tuple[str, str, str], valid_column: str) -> np.ndarray:
    segments: list[list[np.ndarray]] = []
    current_segment: list[np.ndarray] = []
    previous_unit: np.ndarray | None = None
    break_cos = float(np.cos(np.radians(SEGMENT_BREAK_ANGLE_DEG)))

    def finish_segment() -> None:
        nonlocal current_segment, previous_unit
        if current_segment:
            segments.append(current_segment)
        current_segment = []
        previous_unit = None

    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError(f"{path}: CSV has no header")
        missing = [name for name in columns if name not in reader.fieldnames]
        if missing:
            raise ValueError(f"{path}: missing columns {missing}; found {reader.fieldnames}")
        for row in reader:
            if valid_column in reader.fieldnames:
                try:
                    valid_value = float(row[valid_column])
                    if valid_value < 0.5 or valid_value > 1.5:
                        finish_segment()
                        continue
                except (TypeError, ValueError):
                    finish_segment()
                    continue
            try:
                sample = np.array([float(row[name]) for name in columns], dtype=float)
            except (TypeError, ValueError):
                finish_segment()
                continue
            norm = float(np.linalg.norm(sample))
            if not np.all(np.isfinite(sample)) or norm < 1.0:
                finish_segment()
                continue

            unit = sample / norm
            if previous_unit is not None and float(np.dot(previous_unit, unit)) < break_cos:
                finish_segment()
            current_segment.append(sample)
            previous_unit = unit

    finish_segment()
    usable_segments = [segment for segment in segments if len(segment) >= MIN_VALID_ROWS]
    if not usable_segments:
        lengths = [len(segment) for segment in segments]
        raise ValueError(
            f"{path}: no continuous stable block has {MIN_VALID_ROWS} rows; blocks={lengths}"
        )

    selected = usable_segments[-1]
    data = np.vstack(selected)
    print(f"{path.name}: selected last stable block with {data.shape[0]} rows")
    trim = max(1, int(data.shape[0] * 0.05))
    sorted_data = np.sort(data, axis=0)
    if data.shape[0] > 2 * trim:
        sorted_data = sorted_data[trim:-trim]
    return np.mean(sorted_data, axis=0)


def matrix_to_euler_deg(matrix: np.ndarray) -> tuple[float, float, float]:
    roll = np.degrees(np.arctan2(matrix[2, 1], matrix[2, 2]))
    pitch = np.degrees(np.arcsin(np.clip(-matrix[2, 0], -1.0, 1.0)))
    yaw = np.degrees(np.arctan2(matrix[1, 0], matrix[0, 0]))
    return float(roll), float(pitch), float(yaw)


def main() -> None:
    parser = argparse.ArgumentParser()
    for pose in POSES:
        parser.add_argument(f"--{pose}", required=True, type=Path)
    parser.add_argument("--columns", nargs=3, default=("I0", "I1", "I2"))
    parser.add_argument("--valid-column", default="I11")
    args = parser.parse_args()

    sensor_means = np.vstack([
        read_mean(getattr(args, pose), tuple(args.columns), args.valid_column) for pose in POSES
    ])
    body_vectors = np.vstack(list(POSES.values()))

    bias = np.mean(sensor_means, axis=0)
    centered = sensor_means - bias
    norms = np.linalg.norm(centered, axis=1)
    if np.any(norms < 1.0):
        raise ValueError("At least one pose has an invalid acceleration magnitude")
    sensor_units = centered / norms[:, None]

    covariance = sensor_units.T @ body_vectors
    u, _, vt = np.linalg.svd(covariance)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0.0:
        vt[-1, :] *= -1.0
        rotation = vt.T @ u.T

    predicted = (rotation @ sensor_units.T).T
    angular_errors = np.degrees(np.arccos(np.clip(np.sum(predicted * body_vectors, axis=1), -1.0, 1.0)))
    roll, pitch, yaw = matrix_to_euler_deg(rotation)

    print("Sensor raw bias:", " ".join(f"{value:.6f}" for value in bias))
    print("Sensor-to-body matrix:")
    for row in rotation:
        print("    {" + ", ".join(f"{value:.9f}f" for value in row) + "},")
    print(f"Euler check: roll={roll:.4f} deg pitch={pitch:.4f} deg yaw={yaw:.4f} deg")
    for pose, norm, error in zip(POSES, norms, angular_errors):
        print(f"  {pose}: gravity={norm:.3f} count residual={error:.4f} deg")
    print(f"Pose residual: max={np.max(angular_errors):.4f} deg rms={np.sqrt(np.mean(angular_errors**2)):.4f} deg")
    print(f"det(R)={np.linalg.det(rotation):.9f}")


if __name__ == "__main__":
    main()

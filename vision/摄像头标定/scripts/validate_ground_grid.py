#!/usr/bin/env python3
"""Validate pixel-to-ground mapping against manually measured points."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

from generate_firmware_headers import pixel_to_ray


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--calibration", type=Path, default=Path("output/camera_calibration.json"))
    parser.add_argument("--points", type=Path, required=True, help="CSV: img_x,img_y,world_x_cm,world_y_cm,height_cm")
    args = parser.parse_args()

    cfg = json.loads(args.calibration.read_text(encoding="utf-8-sig"))
    errors = []
    with args.points.open("r", encoding="utf-8-sig", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            img_x = float(row["img_x"])
            img_y = float(row["img_y"])
            true_x = float(row["world_x_cm"])
            true_y = float(row["world_y_cm"])
            height = float(row["height_cm"])
            rx, ry, rz, ok = pixel_to_ray(img_x, img_y, cfg)
            if not ok or abs(rz) < 1e-9:
                continue
            t = height / rz
            pred_x = -(t * rx)
            pred_y = -(t * ry)
            err = math.hypot(pred_x - true_x, pred_y - true_y)
            errors.append(err)
            print(f"{img_x:.1f},{img_y:.1f}: pred=({pred_x:.2f},{pred_y:.2f}) true=({true_x:.2f},{true_y:.2f}) err={err:.2f}cm")

    if not errors:
        raise SystemExit("No valid points.")
    errors.sort()
    mean = sum(errors) / len(errors)
    p95 = errors[min(len(errors) - 1, int(0.95 * (len(errors) - 1)))]
    print(f"count={len(errors)} mean={mean:.2f}cm p95={p95:.2f}cm max={errors[-1]:.2f}cm")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

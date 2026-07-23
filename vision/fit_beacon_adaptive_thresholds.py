#!/usr/bin/env python3
"""用现有分类抓图拟合信标姿态自适应门限的轻量系数。"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import statistics
from pathlib import Path

from generate_coarse_circle_map import load_calibration
from vision_tool import (
    COARSE_THRESHOLDS,
    DEFAULT_SOURCE,
    RUN_ROOT,
    Simulator,
)


IDEAL_AREA_PX = 180.0
TILT_FULL_RELAX_DEG = 15.0


def telemetry_for(raw_path: Path) -> tuple[float, float, float] | None:
    metadata_path = raw_path.with_suffix(".json")
    if not metadata_path.is_file():
        return None
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    telemetry = metadata.get("telemetry") or metadata
    try:
        height_value = telemetry.get("tof_height_cm", telemetry.get("height_cm"))
        return (
            float(telemetry["roll_deg"]),
            float(telemetry["pitch_deg"]),
            float(height_value),
        )
    except (KeyError, TypeError, ValueError):
        return None


def candidate_features(
    payload: dict,
    candidate: dict,
    image_width: int,
    center_x: float,
    center_y: float,
) -> dict[str, float]:
    center_col = 0.5 * (candidate["raw_min_col"] + candidate["raw_max_col"])
    center_row = 0.5 * (candidate["raw_min_row"] + candidate["raw_max_row"])
    # 与固件一致：纵横坐标都使用较长的水平半径归一化。
    radius = max(center_x, image_width - 1.0 - center_x)
    radius_sq = min(
        1.0,
        ((center_col - center_x) / radius) ** 2
        + ((center_row - center_y) / radius) ** 2,
    )
    tilt_ratio = min(
        1.0,
        (max(abs(payload["roll_deg"]), abs(payload["pitch_deg"]))
         + 0.375 * min(abs(payload["roll_deg"]), abs(payload["pitch_deg"])))
        / TILT_FULL_RELAX_DEG,
    )
    return {
        "area": float(candidate["beacon_area_pixels"]),
        "circularity": float(candidate["circularity"]),
        "radius_sq": radius_sq,
        "tilt_ratio": tilt_ratio,
        "height_cm": float(payload["height_cm"]),
    }


def robust_area_fit(samples: list[dict[str, float]]) -> tuple[float, float, float]:
    best: tuple[float, float, float] | None = None
    for radial_step in range(0, 41):
        radial_loss = radial_step / 100.0
        for tilt_step in range(0, 26):
            tilt_loss = tilt_step / 100.0
            normalized = []
            for sample in samples:
                position_scale = max(0.5, 1.0 - radial_loss * sample["radius_sq"])
                tilt_scale = max(0.7, 1.0 - tilt_loss * sample["tilt_ratio"])
                normalized.append(
                    sample["area"] * sample["height_cm"] ** 2
                    / (position_scale * tilt_scale)
                )
            median = statistics.median(normalized)
            loss = sum(min(math.log(value / median) ** 2, 0.5) for value in normalized)
            if best is None or loss < best[0]:
                best = (loss, radial_loss, tilt_loss)

    assert best is not None
    _, radial_loss, tilt_loss = best
    reference_heights = []
    for sample in samples:
        position_scale = max(0.5, 1.0 - radial_loss * sample["radius_sq"])
        tilt_scale = max(0.7, 1.0 - tilt_loss * sample["tilt_ratio"])
        reference_heights.append(
            sample["height_cm"]
            * math.sqrt(sample["area"] / (IDEAL_AREA_PX * position_scale * tilt_scale))
        )
    return statistics.median(reference_heights), radial_loss, tilt_loss


def shape_fit(samples: list[dict[str, float]]) -> tuple[float, float]:
    best: tuple[float, float, float] | None = None
    for radial_step in range(0, 51):
        radial_relaxation = radial_step / 100.0
        for tilt_step in range(0, 31):
            tilt_relaxation = tilt_step / 100.0
            corrected = [
                sample["circularity"]
                + radial_relaxation * sample["radius_sq"]
                + tilt_relaxation * sample["tilt_ratio"]
                for sample in samples
            ]
            center = statistics.mean(corrected)
            loss = sum((value - center) ** 2 for value in corrected)
            if best is None or loss < best[0]:
                best = (loss, radial_relaxation, tilt_relaxation)
    assert best is not None
    return best[1], best[2]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--dataset",
        type=Path,
        default=Path(__file__).resolve().parent / "capture" / "分类",
    )
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--output", type=Path, default=RUN_ROOT / "beacon_adaptive_fit")
    args = parser.parse_args()

    simulator = Simulator(args.source)
    image_width, _, center_x, center_y = load_calibration(
        args.source / "camera_calibration_params.h"
    )
    samples: list[dict[str, float]] = []
    seen_images: set[str] = set()
    processed_images = 0
    for raw_path in sorted(args.dataset.rglob("*.raw")):
        pose = telemetry_for(raw_path)
        if pose is None:
            continue
        digest = hashlib.sha1(raw_path.read_bytes()).hexdigest()
        if digest in seen_images:
            continue
        seen_images.add(digest)
        frame_dir = args.output / f"frame_{processed_images:04d}"
        ok, log, result_path, *_ = simulator.run(
            raw_path,
            frame_dir,
            pose[0],
            pose[1],
            pose[2],
            vehicle_threshold=110,
            beacon_threshold=10,
            coarse_thresholds=COARSE_THRESHOLDS,
        )
        if not ok or result_path is None:
            raise RuntimeError(f"仿真失败: {raw_path}\n{log}")
        payload = json.loads(result_path.read_text(encoding="utf-8"))
        for candidate in payload.get("coarse_candidates", []):
            if candidate.get("beacon_valid") and candidate.get("beacon_area_pixels", 0) > 0:
                samples.append(candidate_features(
                    payload, candidate, image_width, center_x, center_y
                ))
        processed_images += 1

    if len(samples) < 8:
        raise RuntimeError(f"有效信标样本仅 {len(samples)} 个，无法稳定拟合")

    reference_height, area_radial_loss, area_tilt_loss = robust_area_fit(samples)
    shape_radial, shape_tilt = shape_fit(samples)
    # 当前样本中倾角与信标位置、距离相关，纯倾角项容易拟合为零；固件保留小幅下限。
    firmware_tilt_relaxation = max(0.10, shape_tilt)
    report = {
        "image_count": processed_images,
        "beacon_sample_count": len(samples),
        "fitted": {
            "reference_height_cm": round(reference_height, 3),
            "area_radial_loss": round(area_radial_loss, 3),
            "area_tilt_loss": round(area_tilt_loss, 3),
            "shape_radial_relaxation": round(shape_radial, 3),
            "shape_tilt_relaxation": round(shape_tilt, 3),
        },
        "firmware_recommendation": {
            "reference_height_cm": round(reference_height),
            "area_radial_loss": round(area_radial_loss, 2),
            "area_tilt_loss": max(0.05, round(area_tilt_loss, 2)),
            "shape_radial_relaxation": round(shape_radial, 2),
            "shape_tilt_relaxation": round(firmware_tilt_relaxation, 2),
        },
    }
    args.output.mkdir(parents=True, exist_ok=True)
    report_path = args.output / "fit_report.json"
    report_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print(f"报告: {report_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

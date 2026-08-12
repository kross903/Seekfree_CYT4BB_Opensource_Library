#!/usr/bin/env python3
"""Generate firmware C headers from camera_calibration.json."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np


def c_float(value: float) -> str:
    """Format a finite Python number as a valid single-precision C literal."""
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"Non-finite calibration value: {number}")
    text = f"{number:.9g}"
    if "." not in text and "e" not in text.lower():
        text += ".0"
    return text + "f"


def solve_theta(theta_d: float, k: tuple[float, float, float, float]) -> float:
    t = theta_d
    for _ in range(12):
        t2 = t * t
        t4 = t2 * t2
        t6 = t4 * t2
        t8 = t4 * t4
        f = t * (1.0 + k[0] * t2 + k[1] * t4 + k[2] * t6 + k[3] * t8) - theta_d
        df = 1.0 + 3.0 * k[0] * t2 + 5.0 * k[1] * t4 + 7.0 * k[2] * t6 + 9.0 * k[3] * t8
        if abs(df) < 1e-9:
            break
        t = max(0.0, t - f / df)
    return t


def pixel_to_ray(x: float, y: float, cfg: dict) -> tuple[float, float, float, bool]:
    dx = x - cfg["cx"]
    dy = y - cfg["cy"]
    radius = math.hypot(dx, dy)
    if cfg["valid_radius_px"] > 0 and radius > cfg["valid_radius_px"]:
        return 0.0, 0.0, 0.0, False
    mx = dx / cfg["fx"]
    my = dy / cfg["fy"]
    theta_d = math.hypot(mx, my)
    if theta_d < 1e-9:
        return 0.0, 0.0, 1.0, True
    theta = solve_theta(theta_d, (cfg["k1"], cfg["k2"], cfg["k3"], cfg["k4"]))
    s = math.sin(theta) / theta_d
    return my * s, mx * s, math.cos(theta), True


def c_array_2d(
    name: str,
    values: list[list[int]],
    c_type: str,
    row_dim: str = "MT9V03X_H",
    col_dim: str = "MT9V03X_W",
) -> str:
    rows = []
    for row in values:
        rows.append("    {" + ", ".join(str(v) for v in row) + "}")
    return f"const {c_type} {name}[{row_dim}][{col_dim}] = {{\n" + ",\n".join(rows) + "\n};\n"


def write_generated(path: Path, text: str) -> None:
    """Write generated text while preserving the target's existing EOL style."""
    newline = "\n"
    if path.exists():
        raw = path.read_bytes()
        existing = raw.decode("utf-8-sig").replace("\r\n", "\n")
        if existing == text:
            return
        if b"\r\n" in raw:
            newline = "\r\n"
    path.write_text(text, encoding="utf-8", newline=newline)


def generate_headers(calibration_path: Path | str, output_dir: Path | str, q: int = 14) -> dict:
    calibration_path = Path(calibration_path)
    output_dir = Path(output_dir)

    cfg = json.loads(calibration_path.read_text(encoding="utf-8-sig"))
    output_dir.mkdir(parents=True, exist_ok=True)
    w, h = int(cfg["image_width"]), int(cfg["image_height"])
    if (w, h) != (188, 120):
        raise SystemExit(f"Expected 188x120 calibration, got {w}x{h}")
    undistort_w = int(cfg.get("undistort_width", 376))
    undistort_h = int(cfg.get("undistort_height", 240))
    if undistort_w <= 0 or undistort_h <= 0:
        raise SystemExit(
            f"Expected positive undistort dimensions, got {undistort_w}x{undistort_h}"
        )

    params = f"""/**
 * @file camera_calibration_params.h
 * @brief 由 scripts/generate_firmware_headers.py 自动生成。
 *
 * 初版标定：{cfg.get("used_count", 0)} 张有效图片，RMS {cfg.get("rms", 0.0):.6f} px。
 * 相机安装角和相对机体位置不是棋盘格内参标定的输出，需按实机安装配置。
 */

#ifndef CAMERA_CALIBRATION_PARAMS_H_
#define CAMERA_CALIBRATION_PARAMS_H_

#define CAMERA_CALIBRATION_HAS_GENERATED_PARAMS     (1)
#define CAMERA_CALIBRATION_IMAGE_WIDTH              ({w})
#define CAMERA_CALIBRATION_IMAGE_HEIGHT             ({h})
#define CAMERA_CALIBRATION_UNDISTORT_WIDTH           ({undistort_w})
#define CAMERA_CALIBRATION_UNDISTORT_HEIGHT          ({undistort_h})
#define CAMERA_CALIBRATION_FALLBACK_DIAGONAL_FOV_DEG (170.0f)
#define CAMERA_CALIBRATION_FX                       ({c_float(cfg["fx"])})
#define CAMERA_CALIBRATION_FY                       ({c_float(cfg["fy"])})
#define CAMERA_CALIBRATION_CX                       ({c_float(cfg["cx"])})
#define CAMERA_CALIBRATION_CY                       ({c_float(cfg["cy"])})
#define CAMERA_CALIBRATION_K1                       ({c_float(cfg["k1"])})
#define CAMERA_CALIBRATION_K2                       ({c_float(cfg["k2"])})
#define CAMERA_CALIBRATION_K3                       ({c_float(cfg["k3"])})
#define CAMERA_CALIBRATION_K4                       ({c_float(cfg["k4"])})
#define CAMERA_CALIBRATION_VALID_RADIUS_PX          ({c_float(cfg["valid_radius_px"])})
#define CAMERA_MOUNT_ROLL_OFFSET_DEG                ({c_float(cfg.get("mount_roll_offset_deg", 0.0))})
#define CAMERA_MOUNT_PITCH_OFFSET_DEG               ({c_float(cfg.get("mount_pitch_offset_deg", 0.0))})
#define CAMERA_MOUNT_YAW_OFFSET_DEG                 ({c_float(cfg.get("mount_yaw_offset_deg", 0.0))})
#define CAMERA_OFFSET_X_CM                          ({c_float(cfg.get("offset_x_cm", 0.0))})
#define CAMERA_OFFSET_Y_CM                          ({c_float(cfg.get("offset_y_cm", 0.0))})
#define CAMERA_OFFSET_Z_CM                          ({c_float(cfg.get("offset_z_cm", 0.0))})

#endif /* CAMERA_CALIBRATION_PARAMS_H_ */
"""
    params_path = output_dir / "camera_calibration_params.h"
    write_generated(params_path, params)

    scale = 1 << q
    xs, ys, zs, valid = [], [], [], []
    for y in range(h):
        xr, yr, zr, vr = [], [], [], []
        for x in range(w):
            rx, ry, rz, ok = pixel_to_ray(float(x), float(y), cfg)
            xr.append(int(round(rx * scale)))
            yr.append(int(round(ry * scale)))
            zr.append(int(round(rz * scale)))
            vr.append(1 if ok else 0)
        xs.append(xr)
        ys.append(yr)
        zs.append(zr)
        valid.append(vr)

    ray = f"""/**
 * @file camera_ray_table.h
 * @brief 由 scripts/generate_firmware_headers.py 自动生成。
 */

#ifndef CAMERA_RAY_TABLE_H_
#define CAMERA_RAY_TABLE_H_

#define CAMERA_RAY_TABLE_HAS_DATA    (1)
#define CAMERA_RAY_TABLE_Q           ({q})

"""
    ray += c_array_2d("camera_ray_table_x", xs, "int16")
    ray += c_array_2d("camera_ray_table_y", ys, "int16")
    ray += c_array_2d("camera_ray_table_z", zs, "int16")
    ray += c_array_2d("camera_ray_table_valid", valid, "uint8")
    ray += "\n#endif /* CAMERA_RAY_TABLE_H_ */\n"
    ray_path = output_dir / "camera_ray_table.h"
    write_generated(ray_path, ray)

    k = np.array([[cfg["fx"], 0, cfg["cx"]], [0, cfg["fy"], cfg["cy"]], [0, 0, 1]], dtype=np.float64)
    d = np.array([[cfg["k1"]], [cfg["k2"]], [cfg["k3"]], [cfg["k4"]]], dtype=np.float64)

    # Keep the complete configured rectified field of view, but downsample it
    # back to the camera's 188x120 runtime resolution.  The generated source
    # coordinates use Q8 so firmware can perform true subpixel bilinear sampling
    # without evaluating the fisheye model for every frame.
    rect_scale_x = undistort_w / w
    rect_scale_y = undistort_h / h
    rect_k = np.array(
        [
            [cfg["fx"] / rect_scale_x, 0, (w - 1) * 0.5],
            [0, cfg["fy"] / rect_scale_y, (h - 1) * 0.5],
            [0, 0, 1],
        ],
        dtype=np.float64,
    )
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(
        k, d, np.eye(3), rect_k, (w, h), cv2.CV_32FC1
    )
    map_q = 8
    map_scale = 1 << map_q
    map_invalid = 0xFFFF
    mx, my = [], []
    for y in range(h):
        mxr, myr = [], []
        for x in range(w):
            sx = float(map1[y, x])
            sy = float(map2[y, x])
            # Bilinear sampling needs the complete 2x2 source neighborhood.
            ok = 0.0 <= sx < (w - 1) and 0.0 <= sy < (h - 1)
            mxr.append(int(round(sx * map_scale)) if ok else map_invalid)
            myr.append(int(round(sy * map_scale)) if ok else map_invalid)
        mx.append(mxr)
        my.append(myr)

    rect = f"""/**
 * @file camera_undistort_rect_map.h
 * @brief 由 scripts/generate_firmware_headers.py 自动生成。
 */

#ifndef CAMERA_UNDISTORT_RECT_MAP_H_
#define CAMERA_UNDISTORT_RECT_MAP_H_

#define CAMERA_UNDISTORT_RECT_MAP_HAS_DATA    (1)
#define CAMERA_UNDISTORT_RECT_MAP_Q           ({map_q})
#define CAMERA_UNDISTORT_RECT_MAP_INVALID     (0xFFFFU)
#define CAMERA_UNDISTORT_RECT_SCALE_X         ({c_float(rect_scale_x)})
#define CAMERA_UNDISTORT_RECT_SCALE_Y         ({c_float(rect_scale_y)})

"""
    rect += c_array_2d("camera_undistort_rect_src_x_q8", mx, "uint16")
    rect += c_array_2d("camera_undistort_rect_src_y_q8", my, "uint16")

    # Full-resolution map used for thresholding before the 2x2 binary merge.
    full_k = np.array(
        [
            [cfg["fx"], 0, (undistort_w - 1) * 0.5],
            [0, cfg["fy"], (undistort_h - 1) * 0.5],
            [0, 0, 1],
        ],
        dtype=np.float64,
    )
    full_map_x, full_map_y = cv2.fisheye.initUndistortRectifyMap(
        k, d, np.eye(3), full_k, (undistort_w, undistort_h), cv2.CV_32FC1
    )
    full_mx, full_my = [], []
    for y in range(undistort_h):
        full_mxr, full_myr = [], []
        for x in range(undistort_w):
            sx = float(full_map_x[y, x])
            sy = float(full_map_y[y, x])
            ok = 0.0 <= sx < (w - 1) and 0.0 <= sy < (h - 1)
            full_mxr.append(int(round(sx * map_scale)) if ok else map_invalid)
            full_myr.append(int(round(sy * map_scale)) if ok else map_invalid)
        full_mx.append(full_mxr)
        full_my.append(full_myr)

    rect += c_array_2d(
        "camera_undistort_full_src_x_q8",
        full_mx,
        "uint16",
        "CAMERA_CALIBRATION_UNDISTORT_HEIGHT",
        "CAMERA_CALIBRATION_UNDISTORT_WIDTH",
    )
    rect += c_array_2d(
        "camera_undistort_full_src_y_q8",
        full_my,
        "uint16",
        "CAMERA_CALIBRATION_UNDISTORT_HEIGHT",
        "CAMERA_CALIBRATION_UNDISTORT_WIDTH",
    )
    rect += "\n#endif /* CAMERA_UNDISTORT_RECT_MAP_H_ */\n"
    rect_path = output_dir / "camera_undistort_rect_map.h"
    write_generated(rect_path, rect)

    return {
        "params_path": params_path,
        "ray_path": ray_path,
        "rect_path": rect_path,
        "output_dir": output_dir,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--calibration", type=Path, default=Path("output/camera_calibration.json"))
    parser.add_argument("--output", type=Path, default=Path("output"))
    parser.add_argument("--q", type=int, default=14)
    args = parser.parse_args()

    result = generate_headers(args.calibration, args.output, args.q)
    print(f"Wrote firmware headers to {result['output_dir']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

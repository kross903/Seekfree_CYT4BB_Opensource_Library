#!/usr/bin/env python3
"""Calibrate the 188x120 fisheye camera and write a JSON result."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np


IMAGE_SUFFIXES = {".png", ".jpg", ".jpeg", ".bmp", ".pgm"}
DEFAULT_DETECTION_SCALE = 2.0
DEFAULT_MIN_VALID_IMAGES = 10
DEFAULT_UNDISTORT_WIDTH = 376
DEFAULT_UNDISTORT_HEIGHT = 240
FISHEYE_CALIBRATION_FLAG_NAMES = (
    "CALIB_RECOMPUTE_EXTRINSIC",
    "CALIB_CHECK_COND",
    "CALIB_FIX_SKEW",
)


class CalibrationInputError(RuntimeError):
    """标定输入不足或格式不一致。"""

    def __init__(self, message: str, report: dict | None = None):
        super().__init__(message)
        self.report = report or {}


def iter_images(frames_dir: Path):
    """递归枚举图像，兼容采集程序创建的 png 子目录。"""
    frames_dir = Path(frames_dir)
    if not frames_dir.exists():
        raise CalibrationInputError(f"图像目录不存在: {frames_dir}")
    if not frames_dir.is_dir():
        raise CalibrationInputError(f"图像路径不是目录: {frames_dir}")

    paths = [
        path
        for path in frames_dir.rglob("*")
        if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES
    ]
    yield from sorted(paths, key=lambda path: str(path).lower())


def detect_chessboard_corners(
    gray: np.ndarray,
    pattern_size: tuple[int, int],
    detection_scale: float = DEFAULT_DETECTION_SCALE,
) -> tuple[np.ndarray | None, str]:
    """在低分辨率图像上检测棋盘格，并返回原图坐标系中的角点。"""
    if gray is None or gray.ndim != 2:
        return None, "invalid_grayscale_image"
    if detection_scale < 1.0:
        raise ValueError("detection_scale 必须大于或等于 1.0")

    if detection_scale > 1.0:
        work = cv2.resize(
            gray,
            None,
            fx=detection_scale,
            fy=detection_scale,
            interpolation=cv2.INTER_CUBIC,
        )
    else:
        work = gray

    sb_flags = (
        cv2.CALIB_CB_NORMALIZE_IMAGE
        | cv2.CALIB_CB_EXHAUSTIVE
        | cv2.CALIB_CB_ACCURACY
    )
    ok, corners = cv2.findChessboardCornersSB(work, pattern_size, sb_flags)
    if ok:
        corners = corners.astype(np.float32) / float(detection_scale)
        return corners.reshape(1, -1, 2), "findChessboardCornersSB"

    classic_flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    ok, corners = cv2.findChessboardCorners(work, pattern_size, classic_flags)
    if not ok:
        return None, "corners_not_found"

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 1e-4)
    corners = cv2.cornerSubPix(work, corners, (5, 5), (-1, -1), criteria)
    corners = corners.astype(np.float32) / float(detection_scale)
    return corners.reshape(1, -1, 2), "findChessboardCorners"


def write_detection_report(output_dir: Path, report: dict) -> Path:
    output_dir.mkdir(parents=True, exist_ok=True)
    report_path = output_dir / "corner_detection_report.json"
    report_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    return report_path


def read_grayscale_image(path: Path) -> np.ndarray | None:
    """读取包含中文路径的图像，绕过 Windows 下 cv2.imread 的路径限制。"""
    try:
        encoded = np.frombuffer(path.read_bytes(), dtype=np.uint8)
    except OSError:
        return None
    if encoded.size == 0:
        return None
    return cv2.imdecode(encoded, cv2.IMREAD_GRAYSCALE)


def get_fisheye_calibration_flags() -> int:
    """兼容 OpenCV 4 的 fisheye 常量和 OpenCV 5 的顶层常量。"""
    flags = 0
    for name in FISHEYE_CALIBRATION_FLAG_NAMES:
        if hasattr(cv2.fisheye, name):
            flags |= int(getattr(cv2.fisheye, name))
        elif hasattr(cv2, name):
            flags |= int(getattr(cv2, name))
        else:
            raise RuntimeError(f"当前 OpenCV 缺少鱼眼标定常量: {name}")
    return flags


def load_calibration(path: Path | str) -> dict:
    calibration_path = Path(path)
    try:
        result = json.loads(calibration_path.read_text(encoding="utf-8-sig"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise CalibrationInputError(f"无法读取标定 JSON: {calibration_path}: {exc}") from exc

    required = (
        "image_width",
        "image_height",
        "fx",
        "fy",
        "cx",
        "cy",
        "k1",
        "k2",
        "k3",
        "k4",
    )
    missing = [name for name in required if name not in result]
    if missing:
        raise CalibrationInputError(f"标定 JSON 缺少字段: {', '.join(missing)}")
    return result


def build_undistort_maps(calibration: dict) -> tuple[np.ndarray, np.ndarray]:
    """按原焦距尺度生成居中的有限视场去畸变反查表。"""
    width = int(calibration["image_width"])
    height = int(calibration["image_height"])
    output_width = int(calibration.get("undistort_width", DEFAULT_UNDISTORT_WIDTH))
    output_height = int(calibration.get("undistort_height", DEFAULT_UNDISTORT_HEIGHT))
    if width <= 0 or height <= 0 or output_width <= 0 or output_height <= 0:
        raise CalibrationInputError("标定图像尺寸和去畸变输出尺寸必须大于 0。")

    k = np.array(
        [
            [calibration["fx"], 0.0, calibration["cx"]],
            [0.0, calibration["fy"], calibration["cy"]],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    d = np.array(
        [
            calibration["k1"],
            calibration["k2"],
            calibration["k3"],
            calibration["k4"],
        ],
        dtype=np.float64,
    ).reshape(4, 1)
    rectified_k = k.copy()
    rectified_k[0, 2] = (output_width - 1.0) * 0.5
    rectified_k[1, 2] = (output_height - 1.0) * 0.5
    return cv2.fisheye.initUndistortRectifyMap(
        k,
        d,
        np.eye(3, dtype=np.float64),
        rectified_k,
        (output_width, output_height),
        cv2.CV_32FC1,
    )


def undistort_fisheye_image(image: np.ndarray, calibration: dict) -> np.ndarray:
    """输出原焦距尺度的有限视场去畸变图，原图保持不变。"""
    if image is None or image.ndim not in (2, 3):
        raise CalibrationInputError("预览图像格式无效。")

    width = int(calibration["image_width"])
    height = int(calibration["image_height"])
    if (image.shape[1], image.shape[0]) != (width, height):
        raise CalibrationInputError(
            f"预览图像尺寸为 {image.shape[1]}x{image.shape[0]}，"
            f"标定尺寸为 {width}x{height}。"
        )

    map_x, map_y = build_undistort_maps(calibration)
    return cv2.remap(
        image,
        map_x,
        map_y,
        interpolation=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
    )


def run_calibration(
    frames_dir: Path | str,
    output_dir: Path | str,
    pattern_cols: int,
    pattern_rows: int,
    square_size_cm: float,
    valid_radius_px: float = 0.0,
    detection_scale: float = DEFAULT_DETECTION_SCALE,
    min_valid_images: int = DEFAULT_MIN_VALID_IMAGES,
) -> dict:
    frames_dir = Path(frames_dir)
    output_dir = Path(output_dir)
    if pattern_cols <= 1 or pattern_rows <= 1:
        raise CalibrationInputError("内角点行列数必须都大于 1。")
    if square_size_cm <= 0.0:
        raise CalibrationInputError("方格边长必须大于 0。")
    if min_valid_images < 3:
        raise CalibrationInputError("最少有效图像数不能小于 3。")

    pattern_size = (pattern_cols, pattern_rows)
    objp = np.zeros((1, pattern_cols * pattern_rows, 3), np.float32)
    grid = np.mgrid[0:pattern_cols, 0:pattern_rows].T.reshape(-1, 2)
    objp[0, :, :2] = grid * square_size_cm

    object_points = []
    image_points = []
    image_size = None
    used = []
    rejected = []
    detectors = {}
    image_paths = list(iter_images(frames_dir))
    readable_count = 0

    for path in image_paths:
        gray = read_grayscale_image(path)
        if gray is None:
            rejected.append({"file": str(path.relative_to(frames_dir)), "reason": "read_failed"})
            continue
        readable_count += 1
        if image_size is None:
            image_size = (gray.shape[1], gray.shape[0])
        elif image_size != (gray.shape[1], gray.shape[0]):
            rejected.append(
                {
                    "file": str(path.relative_to(frames_dir)),
                    "reason": "size_mismatch",
                    "size": [gray.shape[1], gray.shape[0]],
                }
            )
            continue

        corners, detector = detect_chessboard_corners(gray, pattern_size, detection_scale)
        if corners is None:
            rejected.append({"file": str(path.relative_to(frames_dir)), "reason": detector})
            continue

        object_points.append(objp.copy())
        image_points.append(corners)
        used.append(str(path.relative_to(frames_dir)))
        detectors[detector] = detectors.get(detector, 0) + 1

    report = {
        "frames_dir": str(frames_dir.resolve()),
        "pattern_cols": pattern_cols,
        "pattern_rows": pattern_rows,
        "square_size_cm": square_size_cm,
        "detection_scale": detection_scale,
        "min_valid_images": min_valid_images,
        "discovered_count": len(image_paths),
        "readable_count": readable_count,
        "valid_count": len(used),
        "detectors": detectors,
        "used_images": used,
        "rejected_images": rejected,
    }
    detection_report_path = write_detection_report(output_dir, report)

    if image_size is None or len(object_points) < min_valid_images:
        raise CalibrationInputError(
            "有效标定图像不足："
            f"发现 {len(image_paths)} 张，读取成功 {readable_count} 张，"
            f"角点识别成功 {len(object_points)} 张，至少需要 {min_valid_images} 张。"
            f"详细结果: {detection_report_path}",
            report,
        )

    k = np.zeros((3, 3), dtype=np.float64)
    d = np.zeros((4, 1), dtype=np.float64)
    rvecs = []
    tvecs = []
    calib_flags = get_fisheye_calibration_flags()
    rms, k, d, rvecs, tvecs = cv2.fisheye.calibrate(
        object_points,
        image_points,
        image_size,
        k,
        d,
        rvecs,
        tvecs,
        calib_flags,
        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6),
    )

    per_view_errors = []
    for obj, img, rvec, tvec, file_name in zip(
        object_points, image_points, rvecs, tvecs, used
    ):
        projected, _ = cv2.fisheye.projectPoints(obj, rvec, tvec, k, d)
        error = np.sqrt(np.mean(np.sum((projected - img) ** 2, axis=2)))
        per_view_errors.append({"file": file_name, "rms_px": float(error)})

    # 0 表示不做圆形裁剪，使用完整的传感器矩形。仅当实测确认镜头边缘
    # 存在无成像区或误差不可接受时，才通过 --valid-radius-px 显式限制半径。
    resolved_valid_radius = max(float(valid_radius_px), 0.0)

    result = {
        "model": "opencv_fisheye",
        "image_width": image_size[0],
        "image_height": image_size[1],
        "rms": float(rms),
        "used_count": len(used),
        "used_images": used,
        "rejected_images": rejected,
        "discovered_count": len(image_paths),
        "readable_count": readable_count,
        "detection_scale": detection_scale,
        "detectors": detectors,
        "per_view_errors": per_view_errors,
        "fx": float(k[0, 0]),
        "fy": float(k[1, 1]),
        "cx": float(k[0, 2]),
        "cy": float(k[1, 2]),
        "k1": float(d[0, 0]),
        "k2": float(d[1, 0]),
        "k3": float(d[2, 0]),
        "k4": float(d[3, 0]),
        "valid_radius_px": float(resolved_valid_radius),
        "undistort_width": DEFAULT_UNDISTORT_WIDTH,
        "undistort_height": DEFAULT_UNDISTORT_HEIGHT,
        "mount_roll_offset_deg": 0.0,
        "mount_pitch_offset_deg": 0.0,
        "mount_yaw_offset_deg": 0.0,
        "offset_x_cm": 0.0,
        "offset_y_cm": 0.0,
        "offset_z_cm": 0.0,
    }

    json_path = output_dir / "camera_calibration.json"
    json_path.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")

    report = [
        "# Camera Calibration Report",
        "",
        f"- model: {result['model']}",
        f"- image: {image_size[0]}x{image_size[1]}",
        f"- discovered images: {len(image_paths)}",
        f"- readable images: {readable_count}",
        f"- used images: {len(used)}",
        f"- rejected images: {len(rejected)}",
        f"- corner detector: findChessboardCornersSB, scale {detection_scale:.1f}x",
        f"- RMS: {rms:.6f}",
        f"- max per-view RMS: {max(item['rms_px'] for item in per_view_errors):.6f}",
        f"- fx/fy/cx/cy: {result['fx']:.6f}, {result['fy']:.6f}, {result['cx']:.6f}, {result['cy']:.6f}",
        f"- k1..k4: {result['k1']:.8f}, {result['k2']:.8f}, {result['k3']:.8f}, {result['k4']:.8f}",
        f"- valid radius px: {resolved_valid_radius:.2f}",
        f"- undistort output: {DEFAULT_UNDISTORT_WIDTH}x{DEFAULT_UNDISTORT_HEIGHT}",
        "- undistort projection: rectilinear, original focal scale, centered finite canvas",
    ]
    report_path = output_dir / "calibration_report.md"
    report_path.write_text("\n".join(report) + "\n", encoding="utf-8")

    summary = dict(result)
    summary["json_path"] = json_path
    summary["report_path"] = report_path
    summary["detection_report_path"] = detection_report_path
    return summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--frames", type=Path, default=Path("frames"))
    parser.add_argument("--output", type=Path, default=Path("output"))
    parser.add_argument("--pattern-cols", type=int, required=True, help="棋盘格内角点列数")
    parser.add_argument("--pattern-rows", type=int, required=True, help="棋盘格内角点行数")
    parser.add_argument("--square-size-cm", type=float, required=True)
    parser.add_argument("--valid-radius-px", type=float, default=0.0)
    parser.add_argument("--detection-scale", type=float, default=DEFAULT_DETECTION_SCALE)
    parser.add_argument("--min-valid-images", type=int, default=DEFAULT_MIN_VALID_IMAGES)
    args = parser.parse_args()

    try:
        result = run_calibration(
            frames_dir=args.frames,
            output_dir=args.output,
            pattern_cols=args.pattern_cols,
            pattern_rows=args.pattern_rows,
            square_size_cm=args.square_size_cm,
            valid_radius_px=args.valid_radius_px,
            detection_scale=args.detection_scale,
            min_valid_images=args.min_valid_images,
        )
    except CalibrationInputError as exc:
        print(str(exc), file=sys.stderr)
        return 2
    print(f"Wrote {result['json_path']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

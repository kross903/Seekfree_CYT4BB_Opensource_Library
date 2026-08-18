#!/usr/bin/env python3
"""Standalone viewer for inspecting captured MT9V03 grayscale frames."""

from __future__ import annotations

from dataclasses import dataclass, replace
import json
import math
from pathlib import Path
import sys
from time import perf_counter
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from PIL import Image, ImageDraw, ImageTk

from image_analysis import (
    CameraCalibration,
    ConnectedComponent,
    FisheyeUndistorter,
    UNDISTORT_BILINEAR,
    UNDISTORT_NEAREST,
    find_contour_components,
    find_components,
    self_test as analysis_self_test,
    threshold_image,
)
from ideal_projection import (
    ProjectionCalibration,
    TargetPlaneProjector,
    build_ideal_beacon_cardinal_points,
    build_ideal_beacon_points,
    build_ideal_chevron_points,
    build_projected_ellipse_outline,
    self_test as projection_self_test,
    wrap_degrees,
)
from ideal_sampling import (
    SamplingConfig,
    SamplingResult,
    sample_projected_shape,
    self_test as sampling_self_test,
)
from ideal_matching import (
    ARROW_DEFAULT_THRESHOLDS,
    BEACON_DEFAULT_THRESHOLDS,
    BINARY_CONSISTENCY,
    COMPLETENESS,
    FAR_DIFFERENCE,
    GRAY_DIFFERENCE,
    MatchingMetrics,
    MatchingThresholds,
    NEAR_DIFFERENCE,
    NEGATIVE_HIT_RATE,
    NEGATIVE_POINTS,
    POSITIVE_HIT_RATE,
    POSITIVE_POINTS,
    STANDARDIZED_SEPARATION,
    TARGET_ARROW,
    TARGET_BEACON,
    THRESHOLD_ATTRIBUTES,
    THRESHOLD_KEYS,
    calculate_matching_metrics,
    evaluate_matching_metrics,
    self_test as matching_self_test,
)
from contour_matching import (
    ARROW_DEFAULT_THRESHOLDS as CONTOUR_ARROW_DEFAULT_THRESHOLDS,
    BEACON_DEFAULT_THRESHOLDS as CONTOUR_BEACON_DEFAULT_THRESHOLDS,
    CONTOUR_THRESHOLD_FIELDS,
    ContourMetrics,
    ContourThresholds,
    calculate_contour_metrics,
    evaluate_contour_metrics,
    self_test as contour_matching_self_test,
)


APP_DIR = Path(__file__).resolve().parent
VISION_DIR = APP_DIR.parent
REPO_ROOT = VISION_DIR.parent
DEFAULT_CAPTURE_DIR = VISION_DIR / "capture"
CALIBRATION_HEADER = REPO_ROOT / "project" / "code" / "vision" / "camera_calibration_params.h"
SETTINGS_PATH = APP_DIR / "viewer_settings.json"
SUPPORTED_SUFFIXES = {".png", ".pgm", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff", ".raw"}
VIEW_NAMES = ("原始灰度图", "去畸变灰度图", "单阈值二值图", "连通域候选")
IDEAL_TARGET_NONE = "不显示"
IDEAL_TARGET_ARROW = "理想箭头"
IDEAL_TARGET_BEACON = "理想信标"
IDEAL_TARGET_NAMES = (IDEAL_TARGET_NONE, IDEAL_TARGET_ARROW, IDEAL_TARGET_BEACON)
IDEAL_BACKGROUND_NAMES = ("去畸变灰度图", "单阈值二值图")
MATCHING_TARGET_KINDS = (TARGET_ARROW, TARGET_BEACON)
MATCHING_TARGET_NAMES = {
    TARGET_ARROW: IDEAL_TARGET_ARROW,
    TARGET_BEACON: IDEAL_TARGET_BEACON,
}
MATCHING_FIELD_LABELS = {
    GRAY_DIFFERENCE: "正负形灰度差",
    STANDARDIZED_SEPARATION: "标准化分离度",
    POSITIVE_HIT_RATE: "正形命中率",
    NEGATIVE_HIT_RATE: "负形命中率",
    BINARY_CONSISTENCY: "固定阈值一致率",
    COMPLETENESS: "完整度",
    NEAR_DIFFERENCE: "近负形灰度差",
    FAR_DIFFERENCE: "远背景灰度差",
    POSITIVE_POINTS: "最少正形点",
    NEGATIVE_POINTS: "最少负形点",
}


@dataclass(frozen=True)
class BeaconRecognitionResult:
    metrics: MatchingMetrics | None
    passed: bool
    failures: tuple[str, ...]
    box_clipped: bool = False
    target_kind: str | None = None
    route: str = ""
    attempts: tuple[str, ...] = ()
    projected_outline: tuple[tuple[float, float], ...] = ()
    sampling: SamplingResult | None = None
    sampling_config: SamplingConfig | None = None
    binary_threshold: int | None = None
    contour_metrics: ContourMetrics | None = None
    recognition_engine: str = ""
    yaw_assisted: bool = False
    yaw_direction_error_deg: float | None = None
    yaw_direction_source: str = ""


FIRMWARE_ROUTE_ARROW_MIN_AREA = 30
BEACON_ACCEPT_LIMIT_DEFAULT = 3
BEACON_ACCEPT_LIMIT_MIN = 1
BEACON_ACCEPT_LIMIT_MAX = 16
ARROW_SHAPE_GATE_DEFAULT_ENABLED = True
ARROW_MAX_OCCUPANCY_DEFAULT = 0.5
ARROW_MAX_HORIZONTAL_RUN_DEFAULT = 0.5
CANDIDATE_MODE_CONTOUR = "八邻域轮廓爬线"
CANDIDATE_MODE_RLE = "RLE连通域"
CANDIDATE_MODE_NAMES = (CANDIDATE_MODE_CONTOUR, CANDIDATE_MODE_RLE)
UNDISTORT_MODE_BILINEAR = "定点双线性"
UNDISTORT_MODE_NEAREST = "最近邻"
UNDISTORT_MODE_NAMES = (UNDISTORT_MODE_BILINEAR, UNDISTORT_MODE_NEAREST)
RECOGNITION_MODE_IDEAL = "理想形状匹配"
RECOGNITION_MODE_CONTOUR = "轮廓几何识别"
RECOGNITION_MODE_NAMES = (RECOGNITION_MODE_IDEAL, RECOGNITION_MODE_CONTOUR)


class PictureViewerApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("MT9V03 图像特征查看器")
        self.root.geometry("1440x860")
        self.root.minsize(1050, 650)

        self.calibration = CameraCalibration.from_header(CALIBRATION_HEADER)
        self.undistorter = FisheyeUndistorter(self.calibration)
        self.base_projection_calibration = ProjectionCalibration.from_header(
            CALIBRATION_HEADER
        )
        self.projection_calibration = self.base_projection_calibration
        settings = self._load_settings()
        ideal_settings = settings.get("ideal_target", {})
        if not isinstance(ideal_settings, dict):
            ideal_settings = {}
        ideal_target = str(ideal_settings.get("type", IDEAL_TARGET_NONE))
        if ideal_target not in IDEAL_TARGET_NAMES:
            ideal_target = IDEAL_TARGET_NONE
        ideal_background = str(ideal_settings.get("background", "去畸变灰度图"))
        if ideal_background not in IDEAL_BACKGROUND_NAMES:
            ideal_background = "去畸变灰度图"
        matching_settings = settings.get("matching_thresholds", {})
        if not isinstance(matching_settings, dict):
            matching_settings = {}
        contour_settings = settings.get("contour_thresholds", {})
        if not isinstance(contour_settings, dict):
            contour_settings = {}
        contour_yaw_settings = contour_settings.get("yaw_assist", {})
        if not isinstance(contour_yaw_settings, dict):
            contour_yaw_settings = {}
        sampling_settings = ideal_settings.get("sampling", {})
        if not isinstance(sampling_settings, dict):
            sampling_settings = {}

        self.folder_var = tk.StringVar(
            value=str(Path(settings.get("folder", DEFAULT_CAPTURE_DIR)))
        )
        self.filter_var = tk.StringVar()
        initial_view = (
            ideal_background if ideal_target != IDEAL_TARGET_NONE else "去畸变灰度图"
        )
        self.view_var = tk.StringVar(value=initial_view)
        self.threshold_var = tk.IntVar(value=int(settings.get("threshold", 50)))
        self.min_area_var = tk.IntVar(value=int(settings.get("min_area", 6)))
        self.connectivity_var = tk.IntVar(value=int(settings.get("connectivity", 8)))
        candidate_mode = str(settings.get("candidate_mode", CANDIDATE_MODE_CONTOUR))
        if candidate_mode not in CANDIDATE_MODE_NAMES:
            candidate_mode = CANDIDATE_MODE_CONTOUR
        undistort_mode = str(
            settings.get("undistort_mode", UNDISTORT_MODE_BILINEAR)
        )
        if undistort_mode not in UNDISTORT_MODE_NAMES:
            undistort_mode = UNDISTORT_MODE_BILINEAR
        recognition_mode = str(
            settings.get("recognition_mode", RECOGNITION_MODE_CONTOUR)
        )
        if recognition_mode not in RECOGNITION_MODE_NAMES:
            recognition_mode = RECOGNITION_MODE_CONTOUR
        self.candidate_mode_var = tk.StringVar(value=candidate_mode)
        self.undistort_mode_var = tk.StringVar(value=undistort_mode)
        self.recognition_mode_var = tk.StringVar(value=recognition_mode)
        try:
            beacon_accept_limit = int(
                settings.get("beacon_accept_limit", BEACON_ACCEPT_LIMIT_DEFAULT)
            )
        except (TypeError, ValueError):
            beacon_accept_limit = BEACON_ACCEPT_LIMIT_DEFAULT
        beacon_accept_limit = max(
            BEACON_ACCEPT_LIMIT_MIN,
            min(BEACON_ACCEPT_LIMIT_MAX, beacon_accept_limit),
        )
        self.beacon_accept_limit_var = tk.IntVar(value=beacon_accept_limit)
        self.status_var = tk.StringVar(value="就绪")
        self.pixel_var = tk.StringVar(value="移动鼠标查看坐标和灰度")
        self.metadata_var = tk.StringVar(value="尚未加载图像")
        self.ideal_target_var = tk.StringVar(value=ideal_target)
        self.ideal_background_var = tk.StringVar(value=ideal_background)
        self.roll_var = tk.StringVar(value=str(ideal_settings.get("roll_deg", "0")))
        self.pitch_var = tk.StringVar(value=str(ideal_settings.get("pitch_deg", "0")))
        self.uav_yaw_var = tk.StringVar(value=str(ideal_settings.get("uav_yaw_deg", "0")))
        self.height_var = tk.StringVar(value=str(ideal_settings.get("height_cm", "100")))
        self.car_yaw_var = tk.StringVar(value=str(ideal_settings.get("car_yaw_deg", "0")))
        self.car_yaw_relative_var = tk.BooleanVar(
            value=bool(ideal_settings.get("car_yaw_relative", False))
        )
        self.mount_roll_var = tk.StringVar(
            value=str(ideal_settings.get("mount_roll_deg", "0"))
        )
        self.mount_pitch_var = tk.StringVar(
            value=str(ideal_settings.get("mount_pitch_deg", "0"))
        )
        self.mount_yaw_var = tk.StringVar(
            value=str(ideal_settings.get("mount_yaw_deg", "0"))
        )
        self.target_projector = TargetPlaneProjector(
            self.projection_calibration,
            self.safe_float(self.mount_roll_var, 0.0),
            self.safe_float(self.mount_pitch_var, 0.0),
            self.safe_float(self.mount_yaw_var, 0.0),
        )
        self.arrow_arm_length_var = tk.StringVar(
            value=str(ideal_settings.get("arrow_arm_length_cm", "19"))
        )
        self.arrow_angle_var = tk.StringVar(
            value=str(ideal_settings.get("arrow_angle_deg", "70"))
        )
        self.arrow_plane_height_var = tk.StringVar(
            value=str(ideal_settings.get("arrow_plane_height_cm", "17"))
        )
        self.arrow_opening_offset_var = tk.StringVar(
            value=str(ideal_settings.get("arrow_opening_offset_deg", "180"))
        )
        self.beacon_diameter_var = tk.StringVar(
            value=str(ideal_settings.get("beacon_diameter_cm", "10"))
        )
        self.beacon_plane_height_var = tk.StringVar(
            value=str(ideal_settings.get("beacon_plane_height_cm", "0"))
        )
        legacy_step = ideal_settings.get("sampling_step_px", None)
        legacy_sampling = {
            "positive_step_px": ideal_settings.get(
                "sampling_positive_step_px",
                "2" if legacy_step is None else legacy_step,
            ),
            "negative_step_px": ideal_settings.get(
                "sampling_negative_step_px",
                "4" if legacy_step is None else legacy_step,
            ),
            "margin_px": ideal_settings.get("sampling_margin_px", "4"),
            "positive_band_px": ideal_settings.get(
                "sampling_positive_band_px", "1.5"
            ),
            "negative_guard_px": ideal_settings.get(
                "sampling_negative_guard_px", "3"
            ),
        }
        self.sampling_vars: dict[str, dict[str, tk.StringVar]] = {}
        for target_kind in MATCHING_TARGET_KINDS:
            target_settings = sampling_settings.get(target_kind, {})
            if not isinstance(target_settings, dict):
                target_settings = {}
            target_legacy_step = target_settings.get("step_px", None)
            self.sampling_vars[target_kind] = {
                key: tk.StringVar(
                    value=str(
                        target_settings.get(
                            key,
                            target_legacy_step
                            if target_legacy_step is not None and key in {
                                "positive_step_px",
                                "negative_step_px",
                            }
                            else fallback,
                        )
                    )
                )
                for key, fallback in legacy_sampling.items()
            }
        self.near_negative_width_var = tk.StringVar(
            value=str(matching_settings.get("near_negative_width_px", "2"))
        )
        self.near_shape_variance_enabled_var = tk.BooleanVar(
            value=bool(ideal_settings.get("near_shape_variance_enabled", False))
        )
        arrow_shape_settings = matching_settings.get("arrow_shape_gate", {})
        if not isinstance(arrow_shape_settings, dict):
            arrow_shape_settings = {}
        self.arrow_shape_gate_enabled_var = tk.BooleanVar(
            value=bool(
                arrow_shape_settings.get(
                    "enabled", ARROW_SHAPE_GATE_DEFAULT_ENABLED
                )
            )
        )
        self.arrow_max_occupancy_var = tk.StringVar(
            value=str(
                arrow_shape_settings.get(
                    "max_occupancy", ARROW_MAX_OCCUPANCY_DEFAULT
                )
            )
        )
        self.arrow_max_horizontal_run_var = tk.StringVar(
            value=str(
                arrow_shape_settings.get(
                    "max_horizontal_run_ratio",
                    ARROW_MAX_HORIZONTAL_RUN_DEFAULT,
                )
            )
        )
        self.matching_defaults = {
            TARGET_ARROW: ARROW_DEFAULT_THRESHOLDS,
            TARGET_BEACON: BEACON_DEFAULT_THRESHOLDS,
        }
        self.matching_enabled_vars: dict[str, dict[str, tk.BooleanVar]] = {}
        self.matching_value_vars: dict[str, dict[str, tk.StringVar]] = {}
        for target_kind in MATCHING_TARGET_KINDS:
            config = MatchingThresholds.from_mapping(
                matching_settings.get(target_kind),
                self.matching_defaults[target_kind],
            )
            self.matching_enabled_vars[target_kind] = {
                key: tk.BooleanVar(value=key in config.enabled)
                for key in THRESHOLD_KEYS
            }
            self.matching_value_vars[target_kind] = {
                key: tk.StringVar(value=str(getattr(config, THRESHOLD_ATTRIBUTES[key])))
                for key in THRESHOLD_KEYS
            }
        self.contour_defaults = {
            TARGET_ARROW: CONTOUR_ARROW_DEFAULT_THRESHOLDS,
            TARGET_BEACON: CONTOUR_BEACON_DEFAULT_THRESHOLDS,
        }
        self.contour_threshold_vars: dict[str, dict[str, tk.StringVar]] = {}
        for target_kind in MATCHING_TARGET_KINDS:
            config = ContourThresholds.from_mapping(
                contour_settings.get(target_kind),
                self.contour_defaults[target_kind],
            )
            self.contour_threshold_vars[target_kind] = {
                field_name: tk.StringVar(value=str(getattr(config, field_name)))
                for field_name, _label in CONTOUR_THRESHOLD_FIELDS
            }
        self.contour_yaw_assist_enabled_var = tk.BooleanVar(
            value=bool(contour_yaw_settings.get("enabled", True))
        )
        self.contour_yaw_tolerance_var = tk.StringVar(
            value=str(contour_yaw_settings.get("tolerance_deg", "20"))
        )
        self.ideal_source_var = tk.StringVar(value="姿态：手动；小车Yaw：手动")
        self.ideal_status_var = tk.StringVar(value="选择目标后可在图像中左键拖动")
        self.sampling_summary_var = tk.StringVar(
            value="正形灰度 --（0点） | 负形灰度 --（0点） | 近形方差未启用"
        )
        self.sampling_count_var = tk.StringVar(
            value="采样点总数：0 | 参与评分：0（正形 0 / 负形 0） | 近形 0"
        )
        self._ideal_pose_source = "手动"
        self._ideal_car_yaw_source = "手动"
        self.sampling_window: tk.Toplevel | None = None
        self.matching_window: tk.Toplevel | None = None
        self.contour_matching_window: tk.Toplevel | None = None

        self.all_files: list[Path] = []
        self.visible_files: list[Path] = []
        self.current_path: Path | None = None
        self.original: Image.Image | None = None
        self.undistorted: Image.Image | None = None
        self.binary: Image.Image | None = None
        self.components: list[ConnectedComponent] = []
        self.selected_component: int | None = None
        self.beacon_recognition_results: dict[int, BeaconRecognitionResult] = {}
        self.current_telemetry: dict = {}

        self.canvas_photo: ImageTk.PhotoImage | None = None
        self.magnifier_photo: ImageTk.PhotoImage | None = None
        self.zoom = 1.0
        self.offset_x = 0.0
        self.offset_y = 0.0
        self._fit_pending = False
        self._pan_anchor: tuple[int, int, float, float] | None = None
        self._threshold_job: str | None = None
        self.undistort_elapsed_ms = 0.0
        self.threshold_elapsed_ms = 0.0
        self.candidate_elapsed_ms = 0.0
        ideal_center = (
            (self.calibration.output_width - 1.0) * 0.5,
            (self.calibration.output_height - 1.0) * 0.5,
        )
        self.ideal_centers = {
            IDEAL_TARGET_ARROW: ideal_center,
            IDEAL_TARGET_BEACON: ideal_center,
        }
        self._ideal_drag_active = False

        self._build_ui()
        self._bind_events()
        self.scan_folder()
        self.root.protocol("WM_DELETE_WINDOW", self.close)

    def _build_ui(self) -> None:
        self.root.columnconfigure(1, weight=1)
        self.root.rowconfigure(1, weight=1)

        toolbar = ttk.Frame(self.root, padding=(8, 7))
        toolbar.grid(row=0, column=0, columnspan=3, sticky="ew")
        toolbar.columnconfigure(1, weight=1)
        ttk.Label(toolbar, text="图片目录").grid(row=0, column=0, padx=(0, 5))
        ttk.Entry(toolbar, textvariable=self.folder_var).grid(row=0, column=1, sticky="ew")
        ttk.Button(toolbar, text="选择", command=self.choose_folder).grid(row=0, column=2, padx=5)
        ttk.Button(toolbar, text="刷新", command=self.scan_folder).grid(row=0, column=3)
        ttk.Separator(toolbar, orient=tk.VERTICAL).grid(row=0, column=4, sticky="ns", padx=8)
        ttk.Button(toolbar, text="上一张", command=lambda: self.move_file(-1)).grid(row=0, column=5)
        ttk.Button(toolbar, text="下一张", command=lambda: self.move_file(1)).grid(row=0, column=6, padx=5)
        ttk.Button(toolbar, text="适应窗口", command=self.fit_image).grid(row=0, column=7)

        file_panel = ttk.Frame(self.root, padding=(8, 0, 4, 7), width=280)
        file_panel.grid(row=1, column=0, sticky="nsew")
        file_panel.rowconfigure(2, weight=1)
        file_panel.columnconfigure(0, weight=1)
        ttk.Label(file_panel, text="捕获图像").grid(row=0, column=0, sticky="w", pady=(0, 4))
        filter_entry = ttk.Entry(file_panel, textvariable=self.filter_var)
        filter_entry.grid(row=1, column=0, sticky="ew", pady=(0, 5))
        self.file_list = tk.Listbox(file_panel, activestyle="dotbox", exportselection=False)
        file_scroll = ttk.Scrollbar(file_panel, orient=tk.VERTICAL, command=self.file_list.yview)
        self.file_list.configure(yscrollcommand=file_scroll.set)
        self.file_list.grid(row=2, column=0, sticky="nsew")
        file_scroll.grid(row=2, column=1, sticky="ns")
        self.file_count_label = ttk.Label(file_panel, text="0 张")
        self.file_count_label.grid(row=3, column=0, sticky="w", pady=(5, 0))

        center = ttk.Frame(self.root, padding=(4, 0, 4, 7))
        center.grid(row=1, column=1, sticky="nsew")
        center.rowconfigure(1, weight=1)
        center.columnconfigure(0, weight=1)
        view_bar = ttk.Frame(center)
        view_bar.grid(row=0, column=0, sticky="ew", pady=(0, 5))
        ttk.Label(view_bar, text="显示").pack(side=tk.LEFT)
        view_box = ttk.Combobox(
            view_bar,
            textvariable=self.view_var,
            values=VIEW_NAMES,
            state="readonly",
            width=18,
        )
        view_box.pack(side=tk.LEFT, padx=(5, 12))
        ttk.Label(view_bar, text="滚轮缩放，右键拖动画面").pack(side=tk.LEFT)

        self.canvas = tk.Canvas(center, background="#151515", highlightthickness=0)
        self.canvas.grid(row=1, column=0, sticky="nsew")

        side = ttk.Frame(self.root, padding=(4, 0, 8, 7), width=360)
        side.grid(row=1, column=2, sticky="nsew")
        side.columnconfigure(0, weight=1)
        side.rowconfigure(3, weight=1)

        binary_controls = ttk.LabelFrame(side, text="单阈值二值化", padding=8)
        binary_controls.grid(row=0, column=0, sticky="ew")
        binary_controls.columnconfigure(1, weight=1)
        ttk.Label(binary_controls, text="阈值").grid(row=0, column=0, sticky="w")
        self.threshold_scale = ttk.Scale(
            binary_controls,
            from_=0,
            to=255,
            orient=tk.HORIZONTAL,
            command=self.threshold_scale_changed,
        )
        self.threshold_scale.set(self.threshold_var.get())
        self.threshold_scale.grid(row=0, column=1, sticky="ew", padx=7)
        threshold_spin = ttk.Spinbox(
            binary_controls,
            from_=0,
            to=255,
            width=5,
            textvariable=self.threshold_var,
            command=self.schedule_analysis,
        )
        threshold_spin.grid(row=0, column=2)
        ttk.Label(binary_controls, text="最小面积").grid(row=1, column=0, sticky="w", pady=(7, 0))
        min_area_spin = ttk.Spinbox(
            binary_controls,
            from_=1,
            to=90240,
            width=8,
            textvariable=self.min_area_var,
            command=self.schedule_analysis,
        )
        min_area_spin.grid(row=1, column=1, sticky="w", padx=7, pady=(7, 0))
        connectivity = ttk.Frame(binary_controls)
        connectivity.grid(row=1, column=2, pady=(7, 0))
        ttk.Radiobutton(
            connectivity,
            text="8邻域",
            value=8,
            variable=self.connectivity_var,
            command=self.schedule_analysis,
        ).pack(side=tk.LEFT)
        ttk.Radiobutton(
            connectivity,
            text="4邻域",
            value=4,
            variable=self.connectivity_var,
            command=self.schedule_analysis,
        ).pack(side=tk.LEFT)
        processing_modes = ttk.Frame(binary_controls)
        processing_modes.grid(
            row=2, column=0, columnspan=3, sticky="ew", pady=(7, 0)
        )
        processing_modes.columnconfigure(1, weight=1)
        processing_modes.columnconfigure(3, weight=1)
        ttk.Label(processing_modes, text="去畸变").grid(row=0, column=0, sticky="w")
        undistort_mode_box = ttk.Combobox(
            processing_modes,
            textvariable=self.undistort_mode_var,
            values=UNDISTORT_MODE_NAMES,
            state="readonly",
            width=11,
        )
        undistort_mode_box.grid(row=0, column=1, sticky="ew", padx=(5, 10))
        undistort_mode_box.bind(
            "<<ComboboxSelected>>", self.undistort_mode_changed
        )
        ttk.Label(processing_modes, text="候选").grid(row=0, column=2, sticky="w")
        candidate_mode_box = ttk.Combobox(
            processing_modes,
            textvariable=self.candidate_mode_var,
            values=CANDIDATE_MODE_NAMES,
            state="readonly",
            width=14,
        )
        candidate_mode_box.grid(row=0, column=3, sticky="ew", padx=(5, 0))
        candidate_mode_box.bind(
            "<<ComboboxSelected>>", self.candidate_mode_changed
        )

        pixel_frame = ttk.LabelFrame(side, text="局部灰度", padding=8)
        pixel_frame.grid(row=1, column=0, sticky="ew", pady=(7, 0))
        pixel_frame.columnconfigure(0, weight=1)
        self.magnifier_label = ttk.Label(pixel_frame, anchor="center")
        self.magnifier_label.grid(row=0, column=0, sticky="ew")
        ttk.Label(pixel_frame, textvariable=self.pixel_var, justify=tk.LEFT).grid(
            row=1, column=0, sticky="w", pady=(5, 0)
        )

        metadata_frame = ttk.LabelFrame(side, text="图像信息", padding=8)
        metadata_frame.grid(row=2, column=0, sticky="ew", pady=(7, 0))
        ttk.Label(metadata_frame, textvariable=self.metadata_var, justify=tk.LEFT).pack(anchor="w")

        analysis_notebook = ttk.Notebook(side)
        self.analysis_notebook = analysis_notebook
        analysis_notebook.grid(row=3, column=0, sticky="nsew", pady=(7, 0))
        ideal_tab = ttk.Frame(analysis_notebook, padding=4)
        candidate_tab = ttk.Frame(analysis_notebook, padding=4)
        analysis_notebook.add(ideal_tab, text="理想目标")
        analysis_notebook.add(candidate_tab, text="连通域候选")
        ideal_tab.columnconfigure(0, weight=1)
        candidate_tab.columnconfigure(0, weight=1)
        candidate_tab.rowconfigure(1, weight=1)

        ideal_controls = ttk.LabelFrame(ideal_tab, text="理想目标投影", padding=8)
        ideal_controls.grid(row=0, column=0, sticky="nsew")
        ideal_controls.columnconfigure(1, weight=1)
        ideal_controls.columnconfigure(3, weight=1)

        ttk.Label(ideal_controls, text="目标").grid(row=0, column=0, sticky="w")
        target_box = ttk.Combobox(
            ideal_controls,
            textvariable=self.ideal_target_var,
            values=IDEAL_TARGET_NAMES,
            state="readonly",
            width=9,
        )
        target_box.grid(row=0, column=1, sticky="ew", padx=(4, 8))
        target_box.bind("<<ComboboxSelected>>", self.ideal_target_changed)
        ttk.Label(ideal_controls, text="背景").grid(row=0, column=2, sticky="w")
        background_box = ttk.Combobox(
            ideal_controls,
            textvariable=self.ideal_background_var,
            values=IDEAL_BACKGROUND_NAMES,
            state="readonly",
            width=12,
        )
        background_box.grid(row=0, column=3, sticky="ew", padx=(4, 0))
        background_box.bind("<<ComboboxSelected>>", self.ideal_background_changed)

        ttk.Button(
            ideal_controls,
            text="重读JSON",
            command=self.reload_ideal_telemetry,
        ).grid(row=1, column=0, sticky="w", pady=(5, 0))
        ttk.Label(
            ideal_controls,
            textvariable=self.ideal_source_var,
            justify=tk.LEFT,
            wraplength=255,
        ).grid(row=1, column=1, columnspan=3, sticky="w", padx=(4, 0), pady=(5, 0))

        def parameter_entry(
            row: int,
            column: int,
            label: str,
            variable: tk.StringVar,
            callback,
        ) -> ttk.Entry:
            ttk.Label(ideal_controls, text=label).grid(
                row=row, column=column, sticky="w", pady=(4, 0)
            )
            entry = ttk.Entry(ideal_controls, textvariable=variable, width=8)
            entry.grid(
                row=row, column=column + 1, sticky="ew", padx=(4, 8 if column == 0 else 0),
                pady=(4, 0),
            )
            entry.bind("<KeyRelease>", callback)
            return entry

        parameter_entry(2, 0, "Roll°", self.roll_var, self.ideal_pose_edited)
        parameter_entry(2, 2, "Pitch°", self.pitch_var, self.ideal_pose_edited)
        parameter_entry(3, 0, "机Yaw°", self.uav_yaw_var, self.ideal_pose_edited)
        parameter_entry(3, 2, "高度cm", self.height_var, self.ideal_pose_edited)
        parameter_entry(4, 0, "车Yaw°", self.car_yaw_var, self.ideal_car_yaw_edited)
        ttk.Checkbutton(
            ideal_controls,
            text="相对机Yaw",
            variable=self.car_yaw_relative_var,
            command=self.ideal_car_yaw_edited,
        ).grid(row=4, column=2, columnspan=2, sticky="w", pady=(4, 0))
        ttk.Separator(ideal_controls).grid(
            row=5, column=0, columnspan=4, sticky="ew", pady=(6, 1)
        )
        mount_controls = ttk.Frame(ideal_controls)
        mount_controls.grid(row=6, column=0, columnspan=4, sticky="ew", pady=(4, 0))
        ttk.Label(mount_controls, text="安装外参°").pack(side=tk.LEFT)
        for label, variable in (
            ("R", self.mount_roll_var),
            ("P", self.mount_pitch_var),
            ("Y", self.mount_yaw_var),
        ):
            ttk.Label(mount_controls, text=label).pack(side=tk.LEFT, padx=(6, 2))
            entry = ttk.Entry(mount_controls, textvariable=variable, width=6)
            entry.pack(side=tk.LEFT)
            entry.bind("<KeyRelease>", self.ideal_mount_angles_changed)
        ttk.Button(
            mount_controls,
            text="归零",
            command=self.reset_mount_angles,
        ).pack(side=tk.RIGHT, padx=(6, 0))
        ttk.Separator(ideal_controls).grid(
            row=7, column=0, columnspan=4, sticky="ew", pady=(6, 1)
        )
        parameter_entry(8, 0, "箭头臂cm", self.arrow_arm_length_var, self.ideal_parameter_changed)
        parameter_entry(8, 2, "夹角°", self.arrow_angle_var, self.ideal_parameter_changed)
        parameter_entry(9, 0, "箭头高cm", self.arrow_plane_height_var, self.ideal_parameter_changed)
        parameter_entry(9, 2, "开口偏移°", self.arrow_opening_offset_var, self.ideal_parameter_changed)
        parameter_entry(10, 0, "信标直径cm", self.beacon_diameter_var, self.ideal_parameter_changed)
        parameter_entry(10, 2, "信标高cm", self.beacon_plane_height_var, self.ideal_parameter_changed)
        ttk.Separator(ideal_controls).grid(
            row=11, column=0, columnspan=4, sticky="ew", pady=(6, 1)
        )
        sampling_actions = ttk.Frame(ideal_controls)
        sampling_actions.grid(row=12, column=0, columnspan=4, sticky="ew", pady=(4, 0))
        ttk.Button(
            sampling_actions,
            text="采样参数",
            command=self.open_sampling_window,
        ).pack(side=tk.LEFT)
        ttk.Button(
            sampling_actions,
            text="识别阈值",
            command=self.open_matching_window,
        ).pack(side=tk.LEFT, padx=(6, 0))
        ttk.Button(
            sampling_actions,
            text="对齐候选质心",
            command=self.align_ideal_to_selected_component,
        ).pack(side=tk.LEFT, padx=(6, 0))
        ttk.Label(
            ideal_controls,
            textvariable=self.sampling_summary_var,
            justify=tk.LEFT,
            wraplength=330,
        ).grid(row=13, column=0, columnspan=4, sticky="w", pady=(5, 0))
        ttk.Label(
            ideal_controls,
            textvariable=self.ideal_status_var,
            justify=tk.LEFT,
            wraplength=330,
        ).grid(row=14, column=0, columnspan=4, sticky="w", pady=(5, 0))

        candidate_bar = ttk.Frame(candidate_tab)
        candidate_bar.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 4))
        candidate_bar.columnconfigure(0, weight=1)
        ttk.Label(candidate_bar, text="连通域候选").grid(row=0, column=0, sticky="w")
        recognition_controls = ttk.Frame(candidate_bar)
        recognition_controls.grid(row=1, column=0, sticky="ew", pady=(3, 0))
        ttk.Label(recognition_controls, text="识别引擎").pack(side=tk.LEFT)
        recognition_mode_box = ttk.Combobox(
            recognition_controls,
            textvariable=self.recognition_mode_var,
            values=RECOGNITION_MODE_NAMES,
            state="readonly",
            width=14,
        )
        recognition_mode_box.pack(side=tk.LEFT, padx=(5, 6))
        recognition_mode_box.bind(
            "<<ComboboxSelected>>", self.recognition_mode_changed
        )
        ttk.Button(
            recognition_controls,
            text="轮廓阈值",
            command=self.open_contour_matching_window,
        ).pack(side=tk.LEFT)
        ttk.Label(recognition_controls, text="信标上限").pack(
            side=tk.LEFT, padx=(10, 3)
        )
        beacon_limit_spin = ttk.Spinbox(
            recognition_controls,
            from_=BEACON_ACCEPT_LIMIT_MIN,
            to=BEACON_ACCEPT_LIMIT_MAX,
            width=3,
            textvariable=self.beacon_accept_limit_var,
            command=self.beacon_accept_limit_changed,
        )
        beacon_limit_spin.pack(side=tk.LEFT)
        beacon_limit_spin.bind("<KeyRelease>", self.beacon_accept_limit_changed)
        beacon_limit_spin.bind("<FocusOut>", self.beacon_accept_limit_changed)
        candidate_actions = ttk.Frame(candidate_bar)
        candidate_actions.grid(row=2, column=0, sticky="ew", pady=(3, 0))
        ttk.Button(
            candidate_actions,
            text="打开候选",
            command=self.open_selected_component,
        ).pack(side=tk.RIGHT)
        ttk.Button(
            candidate_actions,
            text="对齐理想目标",
            command=self.align_ideal_to_selected_component,
        ).pack(side=tk.RIGHT, padx=(0, 6))
        ttk.Button(
            candidate_actions,
            text="一键识别",
            command=self.recognize_current_image_targets,
        ).pack(side=tk.LEFT)
        ttk.Button(
            candidate_actions,
            text="仅测箭头",
            command=self.recognize_current_image_arrows_only,
        ).pack(side=tk.LEFT, padx=(6, 0))
        ttk.Button(
            candidate_actions,
            text="仅测信标",
            command=self.recognize_current_image_beacons_only,
        ).pack(side=tk.LEFT, padx=(6, 0))

        columns = (
            "area",
            "center",
            "box",
            "occupancy",
            "horizontal_run",
            "contour",
            "route",
            "beacon",
        )
        self.component_tree = ttk.Treeview(
            candidate_tab, columns=columns, show="tree headings", height=12
        )
        self.component_tree.heading("#0", text="#")
        self.component_tree.column("#0", width=36, stretch=False, anchor="center")
        headings = {
            "area": ("面积", 55),
            "center": ("质心", 82),
            "box": ("外接框", 85),
            "occupancy": ("占空比", 62),
            "horizontal_run": ("最长段比", 70),
            "contour": ("轮廓", 90),
            "route": ("识别顺序", 80),
            "beacon": ("识别结果", 260),
        }
        for name, (text, width) in headings.items():
            self.component_tree.heading(name, text=text)
            self.component_tree.column(name, width=width, anchor="center")
        component_scroll = ttk.Scrollbar(
            candidate_tab, orient=tk.VERTICAL, command=self.component_tree.yview
        )
        self.component_tree.configure(yscrollcommand=component_scroll.set)
        self.component_tree.grid(row=1, column=0, sticky="nsew")
        component_scroll.grid(row=1, column=1, sticky="ns")
        component_x_scroll = ttk.Scrollbar(
            candidate_tab, orient=tk.HORIZONTAL, command=self.component_tree.xview
        )
        self.component_tree.configure(xscrollcommand=component_x_scroll.set)
        component_x_scroll.grid(row=2, column=0, sticky="ew")

        status = ttk.Label(self.root, textvariable=self.status_var, relief=tk.SUNKEN, anchor="w", padding=(6, 3))
        status.grid(row=2, column=0, columnspan=3, sticky="ew")

    def _bind_events(self) -> None:
        self.file_list.bind("<<ListboxSelect>>", self.file_selected)
        self.filter_var.trace_add("write", lambda *_: self.apply_filter())
        self.view_var.trace_add("write", self.view_changed)
        self.threshold_var.trace_add("write", lambda *_: self.threshold_entry_changed())
        self.min_area_var.trace_add("write", lambda *_: self.schedule_analysis())
        self.canvas.bind("<Configure>", self.canvas_configured)
        self.canvas.bind("<Motion>", self.canvas_motion)
        self.canvas.bind("<MouseWheel>", self.canvas_wheel)
        self.canvas.bind("<Button-4>", lambda event: self.zoom_at(event.x, event.y, 1.25))
        self.canvas.bind("<Button-5>", lambda event: self.zoom_at(event.x, event.y, 0.8))
        self.canvas.bind("<ButtonPress-3>", self.pan_start)
        self.canvas.bind("<B3-Motion>", self.pan_move)
        self.canvas.bind("<ButtonPress-1>", self.ideal_drag_start)
        self.canvas.bind("<B1-Motion>", self.ideal_drag_move)
        self.canvas.bind("<ButtonRelease-1>", self.ideal_drag_stop)
        self.component_tree.bind("<<TreeviewSelect>>", self.component_selected)
        self.component_tree.bind("<Double-1>", lambda _event: self.open_selected_component())
        self.root.bind("<Left>", lambda _event: self.move_file(-1))
        self.root.bind("<Right>", lambda _event: self.move_file(1))
        self.root.bind("<Key-f>", lambda _event: self.fit_image())
        self.root.bind("<Control-o>", lambda _event: self.choose_folder())

    def _load_settings(self) -> dict:
        try:
            return json.loads(SETTINGS_PATH.read_text(encoding="utf-8"))
        except (OSError, UnicodeError, json.JSONDecodeError):
            return {}

    def close(self) -> None:
        matching_settings = {
            "near_negative_width_px": self.safe_float(
                self.near_negative_width_var, 2.0
            ),
        }
        sampling_settings = {}
        contour_threshold_settings = {}
        for target_kind in MATCHING_TARGET_KINDS:
            try:
                thresholds = self.current_matching_thresholds(target_kind)
            except ValueError:
                thresholds = self.matching_defaults[target_kind]
            matching_settings[target_kind] = thresholds.to_mapping()
            variables = self.sampling_vars[target_kind]
            sampling_settings[target_kind] = {
                "positive_step_px": self.safe_int(
                    variables["positive_step_px"], 2, 1, 32
                ),
                "negative_step_px": self.safe_int(
                    variables["negative_step_px"], 4, 1, 32
                ),
                "margin_px": self.safe_float(variables["margin_px"], 4.0),
                "positive_band_px": self.safe_float(
                    variables["positive_band_px"], 1.5
                ),
                "negative_guard_px": self.safe_float(
                    variables["negative_guard_px"], 3.0
                ),
            }
            try:
                contour_thresholds = self.current_contour_thresholds(target_kind)
            except ValueError:
                contour_thresholds = self.contour_defaults[target_kind]
            contour_threshold_settings[target_kind] = contour_thresholds.to_mapping()
        try:
            yaw_assist_enabled, yaw_tolerance = self.current_contour_yaw_assist()
        except ValueError:
            yaw_assist_enabled, yaw_tolerance = True, 20.0
        contour_threshold_settings["yaw_assist"] = {
            "enabled": yaw_assist_enabled,
            "tolerance_deg": yaw_tolerance,
        }
        matching_settings["arrow_shape_gate"] = {
            "enabled": bool(self.arrow_shape_gate_enabled_var.get()),
            "max_occupancy": self.safe_ratio(
                self.arrow_max_occupancy_var, ARROW_MAX_OCCUPANCY_DEFAULT
            ),
            "max_horizontal_run_ratio": self.safe_ratio(
                self.arrow_max_horizontal_run_var,
                ARROW_MAX_HORIZONTAL_RUN_DEFAULT,
            ),
        }
        settings = {
            "folder": self.folder_var.get(),
            "threshold": self.safe_int(self.threshold_var, 50, 0, 255),
            "min_area": self.safe_int(self.min_area_var, 6, 1, 90240),
            "connectivity": 8 if self.connectivity_var.get() == 8 else 4,
            "candidate_mode": self.candidate_mode_var.get(),
            "undistort_mode": self.undistort_mode_var.get(),
            "recognition_mode": self.recognition_mode_var.get(),
            "beacon_accept_limit": self.current_beacon_accept_limit(),
            "ideal_target": {
                "type": self.ideal_target_var.get(),
                "background": self.ideal_background_var.get(),
                "roll_deg": self.safe_float(self.roll_var, 0.0),
                "pitch_deg": self.safe_float(self.pitch_var, 0.0),
                "uav_yaw_deg": self.safe_float(self.uav_yaw_var, 0.0),
                "height_cm": self.safe_float(self.height_var, 100.0),
                "car_yaw_deg": self.safe_float(self.car_yaw_var, 0.0),
                "car_yaw_relative": bool(self.car_yaw_relative_var.get()),
                "mount_roll_deg": self.safe_float(self.mount_roll_var, 0.0),
                "mount_pitch_deg": self.safe_float(self.mount_pitch_var, 0.0),
                "mount_yaw_deg": self.safe_float(self.mount_yaw_var, 0.0),
                "arrow_arm_length_cm": self.safe_float(self.arrow_arm_length_var, 19.0),
                "arrow_angle_deg": self.safe_float(self.arrow_angle_var, 70.0),
                "arrow_plane_height_cm": self.safe_float(self.arrow_plane_height_var, 17.0),
                "arrow_opening_offset_deg": self.safe_float(
                    self.arrow_opening_offset_var, 180.0
                ),
                "beacon_diameter_cm": self.safe_float(self.beacon_diameter_var, 10.0),
                "beacon_plane_height_cm": self.safe_float(
                    self.beacon_plane_height_var, 0.0
                ),
                "near_shape_variance_enabled": bool(
                    self.near_shape_variance_enabled_var.get()
                ),
                "sampling": sampling_settings,
            },
            "matching_thresholds": matching_settings,
            "contour_thresholds": contour_threshold_settings,
        }
        try:
            SETTINGS_PATH.write_text(
                json.dumps(settings, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
        except OSError:
            pass
        self.root.destroy()

    @staticmethod
    def safe_int(variable: tk.Variable, default: int, minimum: int, maximum: int) -> int:
        try:
            value = int(variable.get())
        except (TypeError, ValueError, tk.TclError):
            value = default
        return max(minimum, min(maximum, value))

    @staticmethod
    def safe_float(variable: tk.Variable, default: float) -> float:
        try:
            return float(variable.get())
        except (TypeError, ValueError, tk.TclError):
            return default

    @staticmethod
    def safe_ratio(variable: tk.Variable, default: float) -> float:
        try:
            value = float(variable.get())
        except (TypeError, ValueError, tk.TclError):
            value = default
        return max(0.0, min(1.0, value))

    def current_beacon_accept_limit(self) -> int:
        return self.safe_int(
            self.beacon_accept_limit_var,
            BEACON_ACCEPT_LIMIT_DEFAULT,
            BEACON_ACCEPT_LIMIT_MIN,
            BEACON_ACCEPT_LIMIT_MAX,
        )

    def choose_folder(self) -> None:
        selected = filedialog.askdirectory(
            initialdir=self.folder_var.get() or str(DEFAULT_CAPTURE_DIR),
            title="选择捕获图像目录",
        )
        if selected:
            self.folder_var.set(selected)
            self.scan_folder()

    def scan_folder(self) -> None:
        folder = Path(self.folder_var.get())
        if not folder.is_dir():
            messagebox.showerror("目录无效", f"目录不存在：\n{folder}")
            return
        self.status_var.set("正在扫描图像……")
        self.root.update_idletasks()
        files = [
            path
            for path in folder.rglob("*")
            if path.is_file() and path.suffix.lower() in SUPPORTED_SUFFIXES
        ]
        # Capture produces PNG/RAW pairs. Prefer the viewable PNG and hide duplicate RAW files.
        file_set = {path.with_suffix("") for path in files if path.suffix.lower() != ".raw"}
        self.all_files = sorted(
            (path for path in files if path.suffix.lower() != ".raw" or path.with_suffix("") not in file_set),
            key=lambda path: str(path.relative_to(folder)).lower(),
        )
        self.apply_filter()
        self.status_var.set(f"扫描完成：{len(self.all_files)} 张图像")
        if self.visible_files and self.current_path not in self.visible_files:
            self.file_list.selection_set(0)
            self.file_list.activate(0)
            self.load_path(self.visible_files[0])

    def apply_filter(self) -> None:
        folder = Path(self.folder_var.get())
        keyword = self.filter_var.get().strip().lower()
        self.visible_files = [
            path
            for path in self.all_files
            if not keyword or keyword in str(path.relative_to(folder)).lower()
        ]
        self.file_list.delete(0, tk.END)
        for path in self.visible_files:
            self.file_list.insert(tk.END, str(path.relative_to(folder)))
        self.file_count_label.config(text=f"{len(self.visible_files)} 张")

    def file_selected(self, _event: object = None) -> None:
        selection = self.file_list.curselection()
        if selection:
            self.load_path(self.visible_files[selection[0]])

    def move_file(self, delta: int) -> None:
        if not self.visible_files:
            return
        selection = self.file_list.curselection()
        index = selection[0] if selection else 0
        index = max(0, min(len(self.visible_files) - 1, index + delta))
        self.file_list.selection_clear(0, tk.END)
        self.file_list.selection_set(index)
        self.file_list.activate(index)
        self.file_list.see(index)
        self.load_path(self.visible_files[index])

    def current_undistort_interpolation(self) -> str:
        return (
            UNDISTORT_NEAREST
            if self.undistort_mode_var.get() == UNDISTORT_MODE_NEAREST
            else UNDISTORT_BILINEAR
        )

    def apply_current_undistortion(self, image: Image.Image) -> Image.Image:
        started = perf_counter()
        result = self.undistorter.apply(
            image,
            interpolation=self.current_undistort_interpolation(),
        )
        self.undistort_elapsed_ms = (perf_counter() - started) * 1000.0
        return result

    def load_path(self, path: Path) -> None:
        if path == self.current_path:
            return
        try:
            self.status_var.set(f"正在加载并去畸变：{path.name}")
            self.root.update_idletasks()
            image = self.load_gray_image(path)
            undistorted = self.apply_current_undistortion(image)
        except (OSError, ValueError) as exc:
            messagebox.showerror("加载失败", str(exc))
            self.status_var.set(f"加载失败：{path.name}")
            return

        self.current_path = path
        self.original = image
        self.undistorted = undistorted
        self.selected_component = None
        ideal_center = (
            (self.calibration.output_width - 1.0) * 0.5,
            (self.calibration.output_height - 1.0) * 0.5,
        )
        self.ideal_centers[IDEAL_TARGET_ARROW] = ideal_center
        self.ideal_centers[IDEAL_TARGET_BEACON] = ideal_center
        self.update_metadata(path)
        self.run_analysis()
        self._fit_pending = True
        self.root.after_idle(self.fit_image)

    def load_gray_image(self, path: Path) -> Image.Image:
        if path.suffix.lower() == ".raw":
            expected = self.calibration.image_width * self.calibration.image_height
            payload = path.read_bytes()
            if len(payload) != expected:
                raise ValueError(f"RAW大小为 {len(payload)} B，预期 {expected} B")
            return Image.frombytes(
                "L",
                (self.calibration.image_width, self.calibration.image_height),
                payload,
            )
        with Image.open(path) as opened:
            return opened.convert("L").copy()

    def update_metadata(self, path: Path) -> None:
        metadata_path = path.with_suffix(".json")
        lines = [path.name]
        telemetry: dict = {}
        try:
            metadata = json.loads(metadata_path.read_text(encoding="utf-8-sig"))
            if not isinstance(metadata, dict):
                raise TypeError("JSON根节点不是对象")
            raw_telemetry = metadata.get("telemetry", metadata)
            if not isinstance(raw_telemetry, dict):
                raise TypeError("telemetry节点不是对象")
            telemetry = raw_telemetry
            if telemetry:
                roll_value = self.telemetry_number(telemetry, "roll_deg")
                pitch_value = self.telemetry_number(telemetry, "pitch_deg")
                yaw_value = self.telemetry_number(telemetry, "yaw_deg")
                height_value = self.telemetry_number(
                    telemetry, "tof_height_cm", "vision_tof_height_cm", "fused_height_cm"
                )
                if any(
                    value is not None
                    for value in (roll_value, pitch_value, yaw_value, height_value)
                ):
                    lines.extend(
                        (
                            f"Roll {(roll_value or 0.0):.2f}°  "
                            f"Pitch {(pitch_value or 0.0):.2f}°",
                            f"Yaw {(yaw_value or 0.0):.2f}°  "
                            f"ToF {(height_value or 0.0):.2f} cm",
                        )
                    )
                else:
                    lines.append("无有效遥测元数据")
            else:
                lines.append("无有效遥测元数据")
        except (OSError, UnicodeError, json.JSONDecodeError, TypeError, ValueError):
            lines.append("无有效遥测元数据")
        self.current_telemetry = telemetry
        self.apply_ideal_telemetry(telemetry)
        self.metadata_var.set("\n".join(lines))

    @staticmethod
    def telemetry_number(telemetry: dict, *names: str) -> float | None:
        for name in names:
            value = telemetry.get(name)
            if value is None:
                continue
            try:
                return float(value)
            except (TypeError, ValueError):
                continue
        return None

    def apply_ideal_telemetry(self, telemetry: dict) -> None:
        self.projection_calibration = self.base_projection_calibration
        self.target_projector.calibration = self.projection_calibration
        pose_fields = (
            (self.roll_var, ("roll_deg",)),
            (self.pitch_var, ("pitch_deg",)),
            (self.uav_yaw_var, ("yaw_deg",)),
            (
                self.height_var,
                ("tof_height_cm", "vision_tof_height_cm", "fused_height_cm"),
            ),
        )
        pose_loaded = False
        for variable, names in pose_fields:
            value = self.telemetry_number(telemetry, *names)
            if value is not None:
                variable.set(f"{value:.6f}")
                pose_loaded = True
        self._ideal_pose_source = "同名JSON" if pose_loaded else "手动（JSON无姿态）"

        mount_fields = (
            (self.mount_roll_var, "camera_mount_roll_deg"),
            (self.mount_pitch_var, "camera_mount_pitch_deg"),
            (self.mount_yaw_var, "camera_mount_yaw_deg"),
        )
        for variable, name in mount_fields:
            value = self.telemetry_number(telemetry, name)
            if value is not None:
                variable.set(f"{value:.6f}")
        offset_values = {
            field: self.telemetry_number(telemetry, json_name)
            for field, json_name in (
                ("offset_x_cm", "camera_offset_x_cm"),
                ("offset_y_cm", "camera_offset_y_cm"),
                ("offset_z_cm", "camera_offset_z_cm"),
            )
        }
        if any(value is not None for value in offset_values.values()):
            self.projection_calibration = replace(
                self.projection_calibration,
                **{
                    field: value
                    for field, value in offset_values.items()
                    if value is not None
                },
            )
            self.target_projector.calibration = self.projection_calibration

        yaw_valid = telemetry.get(
            "car_yaw_valid", telemetry.get("vehicle_yaw_valid", True)
        )
        relative_yaw = self.telemetry_number(telemetry, "car_yaw_rel_uav_deg")
        absolute_yaw = self.telemetry_number(
            telemetry, "car_yaw_deg", "vehicle_yaw_deg"
        )
        if not bool(yaw_valid):
            self._ideal_car_yaw_source = "手动（JSON标记无效）"
        elif relative_yaw is not None:
            self.car_yaw_var.set(f"{relative_yaw:.6f}")
            self.car_yaw_relative_var.set(True)
            self._ideal_car_yaw_source = "同名JSON相对Yaw"
        elif absolute_yaw is not None:
            self.car_yaw_var.set(f"{absolute_yaw:.6f}")
            self.car_yaw_relative_var.set(False)
            self._ideal_car_yaw_source = "同名JSON绝对Yaw"
        else:
            self._ideal_car_yaw_source = "手动（JSON无小车Yaw）"
        self.update_ideal_source_text()
        self.invalidate_beacon_recognition()

    def update_ideal_source_text(self) -> None:
        self.ideal_source_var.set(
            f"姿态：{self._ideal_pose_source}；小车Yaw：{self._ideal_car_yaw_source}"
        )

    def reload_ideal_telemetry(self) -> None:
        if self.current_path is None:
            self.ideal_status_var.set("尚未加载图像")
            return
        self.update_metadata(self.current_path)
        self.redraw_canvas()

    def ideal_pose_edited(self, _event: object = None) -> None:
        self._ideal_pose_source = "手动"
        self.update_ideal_source_text()
        self.invalidate_beacon_recognition()
        self.redraw_canvas()

    def ideal_car_yaw_edited(self, _event: object = None) -> None:
        self._ideal_car_yaw_source = "手动"
        self.update_ideal_source_text()
        self.redraw_canvas()

    def ideal_parameter_changed(self, _event: object = None) -> None:
        self.invalidate_beacon_recognition()
        self.redraw_canvas()

    def open_sampling_window(self) -> None:
        if self.sampling_window is not None and self.sampling_window.winfo_exists():
            self.sampling_window.deiconify()
            self.sampling_window.lift()
            return

        window = tk.Toplevel(self.root)
        window.title("正负形采样参数")
        window.resizable(False, False)
        self.sampling_window = window
        content = ttk.Frame(window, padding=10)
        content.grid(row=0, column=0, sticky="nsew")
        content.columnconfigure(0, weight=1)
        notebook = ttk.Notebook(content)
        notebook.grid(row=0, column=0, sticky="ew")
        for target_kind in MATCHING_TARGET_KINDS:
            tab = ttk.Frame(notebook, padding=8)
            notebook.add(
                tab,
                text="箭头" if target_kind == TARGET_ARROW else "信标",
            )
            tab.columnconfigure(1, weight=1)
            variables = self.sampling_vars[target_kind]
            fields = (
                ("正形步长 (px)", variables["positive_step_px"]),
                ("负形步长 (px)", variables["negative_step_px"]),
                ("采样框余量 (px)", variables["margin_px"]),
                ("正形轮廓带 (px)", variables["positive_band_px"]),
                ("负形最小距离 (px)", variables["negative_guard_px"]),
            )
            for row, (label, variable) in enumerate(fields):
                ttk.Label(tab, text=label).grid(row=row, column=0, sticky="w", pady=3)
                if row in (0, 1):
                    entry = ttk.Spinbox(
                        tab,
                        from_=1,
                        to=32,
                        width=9,
                        textvariable=variable,
                        command=self.sampling_parameter_changed,
                    )
                else:
                    entry = ttk.Entry(tab, width=11, textvariable=variable)
                entry.grid(row=row, column=1, sticky="ew", padx=(8, 0), pady=3)
                entry.bind("<KeyRelease>", self.sampling_parameter_changed)
        initial_target = self.ideal_target_var.get()
        notebook.select(1 if initial_target == IDEAL_TARGET_BEACON else 0)
        notebook.bind("<<NotebookTabChanged>>", self.sampling_tab_changed)
        if initial_target not in (IDEAL_TARGET_ARROW, IDEAL_TARGET_BEACON):
            self.ideal_target_var.set(IDEAL_TARGET_ARROW)
            self.view_var.set(self.ideal_background_var.get())
            self.redraw_canvas()

        legend = ttk.Frame(content)
        legend.grid(row=1, column=0, sticky="ew", pady=(6, 0))
        tk.Label(legend, text="● 正形点", fg="#168a45").pack(side=tk.LEFT)
        tk.Label(legend, text="● 负形点", fg="#d62f4b").pack(side=tk.LEFT, padx=(12, 0))
        tk.Label(legend, text="● 近形点", fg="#d6a62f").pack(side=tk.LEFT, padx=(12, 0))
        ttk.Checkbutton(
            legend,
            text="显示近形灰度方差",
            variable=self.near_shape_variance_enabled_var,
            command=self.sampling_parameter_changed,
        ).pack(side=tk.RIGHT)
        ttk.Label(
            content,
            textvariable=self.sampling_count_var,
            justify=tk.LEFT,
            wraplength=400,
        ).grid(row=2, column=0, sticky="w", pady=(7, 0))
        ttk.Label(
            content,
            textvariable=self.sampling_summary_var,
            justify=tk.LEFT,
            wraplength=400,
        ).grid(row=3, column=0, sticky="w", pady=(7, 0))
        actions = ttk.Frame(content)
        actions.grid(row=4, column=0, sticky="e", pady=(8, 0))
        ttk.Button(
            actions,
            text="恢复默认",
            command=self.reset_sampling_parameters,
        ).pack(side=tk.LEFT)
        ttk.Button(
            actions,
            text="关闭",
            command=self.close_sampling_window,
        ).pack(side=tk.LEFT, padx=(6, 0))
        window.protocol("WM_DELETE_WINDOW", self.close_sampling_window)

    def close_sampling_window(self) -> None:
        if self.sampling_window is not None and self.sampling_window.winfo_exists():
            self.sampling_window.destroy()
        self.sampling_window = None

    def reset_sampling_parameters(self) -> None:
        for variables in self.sampling_vars.values():
            variables["positive_step_px"].set("2")
            variables["negative_step_px"].set("4")
            variables["margin_px"].set("4")
            variables["positive_band_px"].set("1.5")
            variables["negative_guard_px"].set("3")
        self.near_shape_variance_enabled_var.set(False)
        self.invalidate_beacon_recognition()
        self.redraw_canvas()

    def sampling_tab_changed(self, event: tk.Event) -> None:
        notebook = event.widget
        try:
            target_kind = MATCHING_TARGET_KINDS[notebook.index("current")]
        except (IndexError, tk.TclError):
            return
        target = (
            IDEAL_TARGET_ARROW if target_kind == TARGET_ARROW else IDEAL_TARGET_BEACON
        )
        self.ideal_target_var.set(target)
        if self.view_var.get() not in IDEAL_BACKGROUND_NAMES:
            self.view_var.set(self.ideal_background_var.get())
        self.redraw_canvas()

    def sampling_parameter_changed(self, _event: object = None) -> None:
        self.invalidate_beacon_recognition()
        self.redraw_canvas()

    def current_matching_thresholds(self, target_kind: str) -> MatchingThresholds:
        if target_kind not in MATCHING_TARGET_KINDS:
            raise ValueError("识别目标类型无效")
        values: dict[str, object] = {}
        try:
            for key, attribute in THRESHOLD_ATTRIBUTES.items():
                raw_value = self.matching_value_vars[target_kind][key].get()
                values[attribute] = (
                    int(raw_value)
                    if key in (POSITIVE_POINTS, NEGATIVE_POINTS)
                    else float(raw_value)
                )
        except (TypeError, ValueError, tk.TclError) as exc:
            raise ValueError("识别门槛必须是有效数字") from exc
        values["enabled"] = tuple(
            key
            for key in THRESHOLD_KEYS
            if self.matching_enabled_vars[target_kind][key].get()
        )
        thresholds = MatchingThresholds(**values)
        thresholds.validate()
        return thresholds

    def current_near_negative_width(self) -> float:
        try:
            value = float(self.near_negative_width_var.get())
        except (TypeError, ValueError, tk.TclError) as exc:
            raise ValueError("近负形环宽度必须是数字") from exc
        if not 0.1 <= value <= 32.0:
            raise ValueError("近负形环宽度必须在0.1到32像素之间")
        return value

    def current_arrow_shape_gate(self) -> tuple[bool, float, float]:
        try:
            max_occupancy = float(self.arrow_max_occupancy_var.get())
            max_horizontal_run = float(
                self.arrow_max_horizontal_run_var.get()
            )
        except (TypeError, ValueError, tk.TclError) as exc:
            raise ValueError("箭头形状门控参数必须是数字") from exc
        if not 0.0 <= max_occupancy <= 1.0:
            raise ValueError("箭头占空比上限必须在0到1之间")
        if not 0.0 <= max_horizontal_run <= 1.0:
            raise ValueError("最长水平段比例上限必须在0到1之间")
        return (
            bool(self.arrow_shape_gate_enabled_var.get()),
            max_occupancy,
            max_horizontal_run,
        )

    def current_contour_thresholds(self, target_kind: str) -> ContourThresholds:
        if target_kind not in MATCHING_TARGET_KINDS:
            raise ValueError("轮廓识别目标类型无效")
        values: dict[str, float | int] = {}
        try:
            for field_name, _label in CONTOUR_THRESHOLD_FIELDS:
                raw_value = self.contour_threshold_vars[target_kind][field_name].get()
                values[field_name] = (
                    int(raw_value)
                    if field_name in {"min_contour_points", "direction_window_points"}
                    else float(raw_value)
                )
        except (TypeError, ValueError, tk.TclError) as exc:
            raise ValueError("轮廓识别阈值必须是有效数字") from exc
        thresholds = ContourThresholds(**values)
        thresholds.validate()
        return thresholds

    def current_contour_yaw_assist(self) -> tuple[bool, float]:
        try:
            tolerance = float(self.contour_yaw_tolerance_var.get())
        except (TypeError, ValueError, tk.TclError) as exc:
            raise ValueError("Yaw辅助方向误差必须是有效数字") from exc
        if not math.isfinite(tolerance) or not 1.0 <= tolerance <= 90.0:
            raise ValueError("Yaw辅助方向误差必须在1到90度之间")
        return bool(self.contour_yaw_assist_enabled_var.get()), tolerance

    def open_contour_matching_window(self) -> None:
        if (
            self.contour_matching_window is not None
            and self.contour_matching_window.winfo_exists()
        ):
            self.contour_matching_window.deiconify()
            self.contour_matching_window.lift()
            return

        window = tk.Toplevel(self.root)
        window.title("轮廓几何识别阈值")
        window.resizable(False, True)
        self.contour_matching_window = window
        content = ttk.Frame(window, padding=10)
        content.grid(row=0, column=0, sticky="nsew")
        ttk.Label(
            content,
            text=(
                "基础轮廓门控不依赖姿态或Yaw；只有方向点不完整时，"
                "可使用车辆Yaw投影进行有限补救。"
            ),
            justify=tk.LEFT,
            wraplength=450,
        ).grid(row=0, column=0, sticky="w", pady=(0, 7))

        yaw_controls = ttk.Frame(content)
        yaw_controls.grid(row=1, column=0, sticky="ew", pady=(0, 7))
        ttk.Checkbutton(
            yaw_controls,
            text="启用Yaw辅助箭头补救",
            variable=self.contour_yaw_assist_enabled_var,
            command=self.contour_parameter_changed,
        ).pack(side=tk.LEFT)
        ttk.Label(yaw_controls, text="方向误差上限(°)").pack(
            side=tk.LEFT, padx=(12, 4)
        )
        yaw_tolerance_entry = ttk.Entry(
            yaw_controls,
            width=7,
            textvariable=self.contour_yaw_tolerance_var,
        )
        yaw_tolerance_entry.pack(side=tk.LEFT)
        yaw_tolerance_entry.bind("<KeyRelease>", self.contour_parameter_changed)

        notebook = ttk.Notebook(content)
        notebook.grid(row=2, column=0, sticky="nsew")
        for target_kind in MATCHING_TARGET_KINDS:
            tab = ttk.Frame(notebook, padding=8)
            notebook.add(tab, text=MATCHING_TARGET_NAMES[target_kind])
            tab.columnconfigure(0, weight=1)
            for row, (field_name, label) in enumerate(CONTOUR_THRESHOLD_FIELDS):
                ttk.Label(tab, text=label).grid(
                    row=row, column=0, sticky="w", padx=(0, 18), pady=2
                )
                entry = ttk.Entry(
                    tab,
                    width=12,
                    textvariable=self.contour_threshold_vars[target_kind][field_name],
                )
                entry.grid(row=row, column=1, sticky="e", pady=2)
                entry.bind("<KeyRelease>", self.contour_parameter_changed)

        actions = ttk.Frame(content)
        actions.grid(row=3, column=0, sticky="e", pady=(9, 0))
        ttk.Button(
            actions,
            text="恢复宽松初值",
            command=self.reset_contour_thresholds,
        ).pack(side=tk.LEFT)
        ttk.Button(
            actions,
            text="关闭",
            command=self.close_contour_matching_window,
        ).pack(side=tk.LEFT, padx=(6, 0))
        window.protocol("WM_DELETE_WINDOW", self.close_contour_matching_window)

    def close_contour_matching_window(self) -> None:
        if (
            self.contour_matching_window is not None
            and self.contour_matching_window.winfo_exists()
        ):
            self.contour_matching_window.destroy()
        self.contour_matching_window = None

    def reset_contour_thresholds(self) -> None:
        for target_kind in MATCHING_TARGET_KINDS:
            defaults = self.contour_defaults[target_kind]
            for field_name, _label in CONTOUR_THRESHOLD_FIELDS:
                self.contour_threshold_vars[target_kind][field_name].set(
                    str(getattr(defaults, field_name))
                )
        self.contour_yaw_assist_enabled_var.set(True)
        self.contour_yaw_tolerance_var.set("20")
        self.invalidate_beacon_recognition()
        self.update_component_tree()
        self.redraw_canvas()

    def contour_parameter_changed(self, _event: object = None) -> None:
        self.invalidate_beacon_recognition()
        self.update_component_tree()
        self.redraw_canvas()

    def open_matching_window(self) -> None:
        if self.matching_window is not None and self.matching_window.winfo_exists():
            self.matching_window.deiconify()
            self.matching_window.lift()
            return

        window = tk.Toplevel(self.root)
        window.title("理想目标识别阈值")
        window.resizable(False, False)
        self.matching_window = window
        content = ttk.Frame(window, padding=10)
        content.grid(row=0, column=0, sticky="nsew")

        common = ttk.Frame(content)
        common.grid(row=0, column=0, sticky="ew", pady=(0, 7))
        ttk.Label(common, text="近负形环宽度 (px)").pack(side=tk.LEFT)
        near_entry = ttk.Entry(
            common,
            width=8,
            textvariable=self.near_negative_width_var,
        )
        near_entry.pack(side=tk.LEFT, padx=(7, 0))
        near_entry.bind("<KeyRelease>", self.matching_parameter_changed)
        ttk.Label(
            common,
            text="勾选表示该项参与试判",
        ).pack(side=tk.RIGHT, padx=(18, 0))

        notebook = ttk.Notebook(content)
        notebook.grid(row=1, column=0, sticky="nsew")
        for target_kind in MATCHING_TARGET_KINDS:
            tab = ttk.Frame(notebook, padding=8)
            notebook.add(tab, text=MATCHING_TARGET_NAMES[target_kind])
            tab.columnconfigure(1, weight=1)
            threshold_row_offset = 0
            if target_kind == TARGET_ARROW:
                ttk.Checkbutton(
                    tab,
                    text="启用箭头占空比/最长段门控",
                    variable=self.arrow_shape_gate_enabled_var,
                    command=self.matching_parameter_changed,
                ).grid(row=0, column=0, columnspan=3, sticky="w", pady=2)
                ttk.Label(tab, text="占空比上限").grid(
                    row=1, column=1, sticky="w", padx=(2, 16), pady=2
                )
                occupancy_entry = ttk.Entry(
                    tab,
                    width=10,
                    textvariable=self.arrow_max_occupancy_var,
                )
                occupancy_entry.grid(row=1, column=2, sticky="e", pady=2)
                occupancy_entry.bind(
                    "<KeyRelease>", self.matching_parameter_changed
                )
                ttk.Label(tab, text="最长水平段比例上限").grid(
                    row=2, column=1, sticky="w", padx=(2, 16), pady=2
                )
                run_entry = ttk.Entry(
                    tab,
                    width=10,
                    textvariable=self.arrow_max_horizontal_run_var,
                )
                run_entry.grid(row=2, column=2, sticky="e", pady=2)
                run_entry.bind("<KeyRelease>", self.matching_parameter_changed)
                threshold_row_offset = 3
            for row, key in enumerate(THRESHOLD_KEYS):
                label = MATCHING_FIELD_LABELS[key]
                if key == COMPLETENESS:
                    label = (
                        "左右臂最低完整度"
                        if target_kind == TARGET_ARROW
                        else "四象限最低完整度"
                    )
                ttk.Checkbutton(
                    tab,
                    variable=self.matching_enabled_vars[target_kind][key],
                    command=self.matching_parameter_changed,
                ).grid(row=row + threshold_row_offset, column=0, sticky="w", pady=2)
                ttk.Label(tab, text=label).grid(
                    row=row + threshold_row_offset,
                    column=1,
                    sticky="w",
                    padx=(2, 16),
                    pady=2,
                )
                entry = ttk.Entry(
                    tab,
                    width=10,
                    textvariable=self.matching_value_vars[target_kind][key],
                )
                entry.grid(row=row + threshold_row_offset, column=2, sticky="e", pady=2)
                entry.bind("<KeyRelease>", self.matching_parameter_changed)

        ttk.Label(
            content,
            text=(
                "初值由旧图像/2灯的6帧箭头和12个信标候选粗略拟合；"
                "远背景门槛默认不参与试判。"
            ),
            justify=tk.LEFT,
            wraplength=430,
        ).grid(row=2, column=0, sticky="w", pady=(7, 0))
        ttk.Label(
            content,
            textvariable=self.sampling_summary_var,
            justify=tk.LEFT,
            wraplength=430,
        ).grid(row=3, column=0, sticky="w", pady=(7, 0))
        actions = ttk.Frame(content)
        actions.grid(row=4, column=0, sticky="e", pady=(9, 0))
        ttk.Button(
            actions,
            text="恢复推荐初值",
            command=self.reset_matching_thresholds,
        ).pack(side=tk.LEFT)
        ttk.Button(
            actions,
            text="关闭",
            command=self.close_matching_window,
        ).pack(side=tk.LEFT, padx=(6, 0))
        window.protocol("WM_DELETE_WINDOW", self.close_matching_window)

    def close_matching_window(self) -> None:
        if self.matching_window is not None and self.matching_window.winfo_exists():
            self.matching_window.destroy()
        self.matching_window = None

    def reset_matching_thresholds(self) -> None:
        self.near_negative_width_var.set("2")
        self.arrow_shape_gate_enabled_var.set(ARROW_SHAPE_GATE_DEFAULT_ENABLED)
        self.arrow_max_occupancy_var.set(str(ARROW_MAX_OCCUPANCY_DEFAULT))
        self.arrow_max_horizontal_run_var.set(
            str(ARROW_MAX_HORIZONTAL_RUN_DEFAULT)
        )
        for target_kind in MATCHING_TARGET_KINDS:
            config = self.matching_defaults[target_kind]
            for key, attribute in THRESHOLD_ATTRIBUTES.items():
                self.matching_value_vars[target_kind][key].set(
                    str(getattr(config, attribute))
                )
                self.matching_enabled_vars[target_kind][key].set(
                    key in config.enabled
                )
        self.invalidate_beacon_recognition()
        self.redraw_canvas()

    def matching_parameter_changed(self, _event: object = None) -> None:
        self.invalidate_beacon_recognition()
        self.redraw_canvas()

    def ideal_mount_angles_changed(self, _event: object = None) -> None:
        self.invalidate_beacon_recognition()
        self.redraw_canvas()

    def reset_mount_angles(self) -> None:
        self.mount_roll_var.set("0")
        self.mount_pitch_var.set("0")
        self.mount_yaw_var.set("0")
        self.target_projector.set_mount_angles(0.0, 0.0, 0.0)
        self.invalidate_beacon_recognition()
        self.redraw_canvas()

    def ideal_target_changed(self, _event: object = None) -> None:
        if self.ideal_target_var.get() != IDEAL_TARGET_NONE:
            self.view_var.set(self.ideal_background_var.get())
        self.redraw_canvas()

    def ideal_background_changed(self, _event: object = None) -> None:
        if self.ideal_target_var.get() != IDEAL_TARGET_NONE:
            self.view_var.set(self.ideal_background_var.get())
        self.redraw_canvas()

    def view_changed(self, *_args: object) -> None:
        view = self.view_var.get()
        if view in IDEAL_BACKGROUND_NAMES:
            self.ideal_background_var.set(view)
        self.redraw_canvas()

    def undistort_mode_changed(self, _event: object = None) -> None:
        if self.original is None:
            return
        try:
            self.status_var.set(
                f"正在切换去畸变采样：{self.undistort_mode_var.get()}"
            )
            self.root.update_idletasks()
            self.undistorted = self.apply_current_undistortion(self.original)
        except ValueError as exc:
            messagebox.showerror("去畸变失败", str(exc))
            return
        self.run_analysis()

    def candidate_mode_changed(self, _event: object = None) -> None:
        if self.candidate_mode_var.get() == CANDIDATE_MODE_CONTOUR:
            self.connectivity_var.set(8)
        self.schedule_analysis()

    def recognition_mode_changed(self, _event: object = None) -> None:
        if (
            self.recognition_mode_var.get() == RECOGNITION_MODE_CONTOUR
            and self.candidate_mode_var.get() != CANDIDATE_MODE_CONTOUR
        ):
            self.candidate_mode_var.set(CANDIDATE_MODE_CONTOUR)
            self.connectivity_var.set(8)
            self.schedule_analysis()
            return
        self.invalidate_beacon_recognition()
        self.update_component_tree()
        self.redraw_canvas()
        self.status_var.set(f"识别引擎已切换为：{self.recognition_mode_var.get()}")

    def beacon_accept_limit_changed(self, _event: object = None) -> None:
        self.invalidate_beacon_recognition()
        self.update_component_tree()
        self.redraw_canvas()

    def threshold_scale_changed(self, value: str) -> None:
        self.threshold_var.set(int(round(float(value))))

    def threshold_entry_changed(self) -> None:
        value = self.safe_int(self.threshold_var, 50, 0, 255)
        if abs(self.threshold_scale.get() - value) >= 0.5:
            self.threshold_scale.set(value)
        self.schedule_analysis()

    def schedule_analysis(self) -> None:
        self.invalidate_beacon_recognition()
        if self._threshold_job is not None:
            self.root.after_cancel(self._threshold_job)
        self._threshold_job = self.root.after(80, self.run_analysis)

    def invalidate_beacon_recognition(self) -> None:
        had_results = bool(self.beacon_recognition_results)
        self.beacon_recognition_results.clear()
        if hasattr(self, "component_tree"):
            for item_id in self.component_tree.get_children():
                self.component_tree.set(item_id, "beacon", "未检测")
        if had_results:
            self.status_var.set("新方案识别结果已失效，请重新识别")

    def run_analysis(self) -> None:
        self._threshold_job = None
        if self.undistorted is None:
            return
        threshold = self.safe_int(self.threshold_var, 50, 0, 255)
        min_area = self.safe_int(self.min_area_var, 6, 1, self.undistorted.width * self.undistorted.height)
        threshold_started = perf_counter()
        self.binary = threshold_image(self.undistorted, threshold)
        self.threshold_elapsed_ms = (perf_counter() - threshold_started) * 1000.0
        candidate_started = perf_counter()
        if self.candidate_mode_var.get() == CANDIDATE_MODE_CONTOUR:
            self.connectivity_var.set(8)
            self.components = find_contour_components(
                self.binary,
                min_area=min_area,
            )
        else:
            self.components = find_components(
                self.binary,
                min_area=min_area,
                eight_connected=self.connectivity_var.get() == 8,
            )
        self.candidate_elapsed_ms = (perf_counter() - candidate_started) * 1000.0
        self.selected_component = None
        self.invalidate_beacon_recognition()
        self.update_component_tree()
        self.redraw_canvas()
        foreground = sum(1 for value in self.binary.tobytes() if value)
        ratio = 100.0 * foreground / (self.binary.width * self.binary.height)
        contour_failures = sum(
            component.extraction_method == "contour"
            and not component.contour_closed
            for component in self.components
        )
        failure_text = (
            f" | 未闭合 {contour_failures}" if contour_failures else ""
        )
        file_text = f"{self.current_path.name} | " if self.current_path else ""
        self.status_var.set(
            f"{file_text}{self.undistort_mode_var.get()} "
            f"{self.undistort_elapsed_ms:.1f} ms | 二值化 "
            f"{self.threshold_elapsed_ms:.1f} ms | {self.candidate_mode_var.get()} "
            f"{self.candidate_elapsed_ms:.1f} ms | 阈值 {threshold} | "
            f"前景 {foreground} ({ratio:.2f}%) | 候选 {len(self.components)}"
            f"{failure_text} | 最小面积 {min_area}"
        )

    def update_component_tree(self) -> None:
        self.component_tree.delete(*self.component_tree.get_children())
        for component in self.components:
            recognition = self.beacon_recognition_results.get(component.label)
            if recognition is None:
                recognition_text = "未检测"
            elif recognition.passed:
                target_text = (
                    "箭头通过" if recognition.target_kind == TARGET_ARROW
                    else "信标通过"
                )
                engine_text = (
                    "轮廓" if recognition.recognition_engine == RECOGNITION_MODE_CONTOUR
                    else "理想"
                )
                recognition_text = f"{engine_text}·{target_text}"
            else:
                first_failure = recognition.failures[0] if recognition.failures else "未通过"
                recognition_text = f"未通过：{first_failure}"
            coarse_area = max(1, (component.area + 2) // 4)
            if recognition is not None and recognition.route:
                route_text = recognition.route
            elif self.recognition_mode_var.get() == RECOGNITION_MODE_CONTOUR:
                route_text = "信标优先"
            else:
                route_text = (
                    "箭头优先" if coarse_area >= FIRMWARE_ROUTE_ARROW_MIN_AREA
                    else "信标优先"
                )
            if component.extraction_method == "contour":
                contour_state = "闭合" if component.contour_closed else "未闭合"
                if component.contour_overflow:
                    contour_state = "步数溢出"
                contour_text = f"{contour_state} {len(component.contour_directions)}"
            else:
                contour_text = "RLE"
            self.component_tree.insert(
                "",
                tk.END,
                iid=str(component.label),
                text=str(component.label),
                values=(
                    component.area,
                    f"{component.centroid_x:.1f},{component.centroid_y:.1f}",
                    f"{component.width}x{component.height}",
                    f"{component.occupancy:.3f}",
                    f"{component.horizontal_run_ratio:.3f}",
                    contour_text,
                    route_text,
                    recognition_text,
                ),
            )
        if (
            self.selected_component is not None
            and self.component_tree.exists(str(self.selected_component))
        ):
            self.component_tree.selection_set(str(self.selected_component))
            self.component_tree.see(str(self.selected_component))

    def recognize_current_image_beacons(self) -> None:
        if self.undistorted is None:
            messagebox.showinfo("信标识别", "请先加载一张图像。")
            return
        if not self.components:
            self.invalidate_beacon_recognition()
            self.update_component_tree()
            self.view_var.set("连通域候选")
            self.status_var.set("信标识别完成：当前图像无连通域候选")
            return

        try:
            roll = float(self.roll_var.get())
            pitch = float(self.pitch_var.get())
            height = float(self.height_var.get())
            plane_height = float(self.beacon_plane_height_var.get())
            diameter = float(self.beacon_diameter_var.get())
            mount_roll = float(self.mount_roll_var.get())
            mount_pitch = float(self.mount_pitch_var.get())
            mount_yaw = float(self.mount_yaw_var.get())
            numeric_values = (
                roll,
                pitch,
                height,
                plane_height,
                diameter,
                mount_roll,
                mount_pitch,
                mount_yaw,
            )
            if not all(math.isfinite(value) for value in numeric_values):
                raise ValueError("姿态、高度和信标参数必须是有限数值")
            if height <= 0.0:
                raise ValueError("无人机高度必须大于0")
            if plane_height < 0.0:
                raise ValueError("信标高度不能小于0")
            if plane_height >= height - self.projection_calibration.offset_z_cm:
                raise ValueError("信标平面必须低于相机")
            if diameter <= 0.0:
                raise ValueError("信标直径必须大于0")
            self.target_projector.set_mount_angles(
                mount_roll, mount_pitch, mount_yaw
            )
            sampling_config = self.current_sampling_config(TARGET_BEACON)
            thresholds = self.current_matching_thresholds(TARGET_BEACON)
            near_negative_width = self.current_near_negative_width()
            binary_threshold = self.safe_int(self.threshold_var, 50, 0, 255)
        except ValueError as exc:
            messagebox.showerror("信标识别参数错误", str(exc))
            self.status_var.set(f"信标识别失败：{exc}")
            return

        results: dict[int, BeaconRecognitionResult] = {}
        accepted = 0
        beacon_accept_limit = self.current_beacon_accept_limit()
        for component in self.components:
            if accepted >= beacon_accept_limit:
                results[component.label] = BeaconRecognitionResult(
                    metrics=None,
                    passed=False,
                    failures=(
                        f"信标跳过：已达到单帧上限 {beacon_accept_limit}",
                    ),
                    target_kind=TARGET_BEACON,
                    route="信标优先",
                    attempts=(
                        f"信标跳过（已达到上限 {beacon_accept_limit}）",
                    ),
                    binary_threshold=binary_threshold,
                )
                continue
            center_image = (component.centroid_x, component.centroid_y)
            try:
                center_plane = self.target_projector.image_to_plane(
                    center_image, roll, pitch, height, plane_height
                )
                if center_plane is None:
                    raise ValueError("候选质心无法投影到信标平面")
                projected_outline: list[tuple[float, float]] = []
                for point in build_ideal_beacon_points(center_plane, diameter):
                    projected = self.target_projector.plane_to_image(
                        point, roll, pitch, height, plane_height
                    )
                    if projected is None:
                        raise ValueError("信标轮廓位于相机后方")
                    projected_outline.append(projected)
                sampling = sample_projected_shape(
                    self.undistorted,
                    projected_outline,
                    closed=True,
                    filled_positive=True,
                    config=sampling_config,
                    is_valid_pixel=self.undistorter.is_valid_output_pixel,
                    collect_near_shape=self.near_shape_variance_enabled_var.get(),
                )
                metrics = calculate_matching_metrics(
                    self.undistorted,
                    projected_outline,
                    sampling,
                    target_kind=TARGET_BEACON,
                    center=center_image,
                    binary_threshold=binary_threshold,
                    negative_guard_px=sampling_config.negative_guard_px,
                    near_negative_width_px=near_negative_width,
                )
                decision = evaluate_matching_metrics(
                    metrics,
                    thresholds,
                    target_kind=TARGET_BEACON,
                )
                results[component.label] = BeaconRecognitionResult(
                    metrics=metrics,
                    passed=decision.passed,
                    failures=decision.failures,
                    box_clipped=sampling.box_clipped,
                    target_kind=TARGET_BEACON if decision.passed else None,
                    projected_outline=(
                        tuple(projected_outline) if decision.passed else ()
                    ),
                    sampling=sampling,
                    sampling_config=sampling_config,
                    binary_threshold=binary_threshold,
                )
                if decision.passed:
                    accepted += 1
            except (ValueError, OverflowError) as exc:
                results[component.label] = BeaconRecognitionResult(
                    metrics=None,
                    passed=False,
                    failures=(f"计算失败：{exc}",),
                )

        self.beacon_recognition_results = results
        self.update_component_tree()
        self.view_var.set("连通域候选")
        self.redraw_canvas()
        self.status_var.set(
            f"信标识别完成：{accepted}/{len(results)} 个候选通过 | "
            f"二值阈值 {binary_threshold} | 信标直径 {diameter:.1f} cm"
        )

    @staticmethod
    def _direction_error_deg(
        expected: tuple[float, float],
        observed: tuple[float, float],
    ) -> float | None:
        expected_length = math.hypot(*expected)
        observed_length = math.hypot(*observed)
        if expected_length <= 1e-9 or observed_length <= 1e-9:
            return None
        dot = expected[0] * observed[0] + expected[1] * observed[1]
        cross = expected[0] * observed[1] - expected[1] * observed[0]
        return math.degrees(math.atan2(abs(cross), dot))

    def contour_yaw_direction_error(
        self,
        component: ConnectedComponent,
        metrics: ContourMetrics,
    ) -> tuple[float, str]:
        if len(metrics.arm_tip_points) != 2:
            raise ValueError("需要两个端点")
        direction_points = (
            ("V", metrics.vertex_point),
            ("N", metrics.deepest_point),
        )
        available_points = [
            (name, point) for name, point in direction_points if point is not None
        ]
        if not available_points:
            raise ValueError("缺少外顶点V和内凹口N")

        try:
            roll = float(self.roll_var.get())
            pitch = float(self.pitch_var.get())
            height = float(self.height_var.get())
            uav_yaw = float(self.uav_yaw_var.get())
            car_yaw = float(self.car_yaw_var.get())
            plane_height = float(self.arrow_plane_height_var.get())
            arm_length = float(self.arrow_arm_length_var.get())
            arrow_angle = float(self.arrow_angle_var.get())
            opening_offset = float(self.arrow_opening_offset_var.get())
            mount_roll = float(self.mount_roll_var.get())
            mount_pitch = float(self.mount_pitch_var.get())
            mount_yaw = float(self.mount_yaw_var.get())
        except (TypeError, ValueError, tk.TclError) as exc:
            raise ValueError("姿态、Yaw或箭头参数不是有效数字") from exc
        values = (
            roll,
            pitch,
            height,
            uav_yaw,
            car_yaw,
            plane_height,
            arm_length,
            arrow_angle,
            opening_offset,
            mount_roll,
            mount_pitch,
            mount_yaw,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("姿态、Yaw和箭头参数必须是有限数值")
        if height <= 0.0 or plane_height < 0.0:
            raise ValueError("高度和箭头平面高度无效")
        if plane_height >= height - self.projection_calibration.offset_z_cm:
            raise ValueError("箭头平面必须低于相机")

        self.target_projector.set_mount_angles(mount_roll, mount_pitch, mount_yaw)
        center_image = (component.centroid_x, component.centroid_y)
        center_plane = self.target_projector.image_to_plane(
            center_image,
            roll,
            pitch,
            height,
            plane_height,
        )
        if center_plane is None:
            raise ValueError("候选质心无法投影到箭头平面")
        relative_yaw = wrap_degrees(
            car_yaw if self.car_yaw_relative_var.get() else car_yaw - uav_yaw
        )
        physical_points = build_ideal_chevron_points(
            center_plane,
            arm_length,
            arrow_angle,
            wrap_degrees(relative_yaw + opening_offset),
        )
        projected_apex = self.target_projector.plane_to_image(
            physical_points["apex"],
            roll,
            pitch,
            height,
            plane_height,
        )
        if projected_apex is None:
            raise ValueError("理论箭头顶点位于相机后方")
        expected = (
            projected_apex[0] - center_image[0],
            projected_apex[1] - center_image[1],
        )
        first_tip, second_tip = metrics.arm_tip_points
        endpoint_center = (
            (first_tip[0] + second_tip[0]) * 0.5,
            (first_tip[1] + second_tip[1]) * 0.5,
        )
        errors: list[tuple[float, str]] = []
        for name, point in available_points:
            assert point is not None
            error = self._direction_error_deg(
                expected,
                (point[0] - endpoint_center[0], point[1] - endpoint_center[1]),
            )
            if error is not None:
                errors.append((error, name))
        if not errors:
            raise ValueError("方向点与端点中点重合")
        return min(errors, key=lambda item: item[0])

    def evaluate_contour_arrow_with_yaw(
        self,
        component: ConnectedComponent,
        metrics: ContourMetrics,
        thresholds: ContourThresholds,
        yaw_assist_enabled: bool,
        yaw_tolerance_deg: float,
    ) -> tuple[bool, tuple[str, ...], bool, float | None, str]:
        decision = evaluate_contour_metrics(metrics, thresholds)
        if decision.passed:
            return True, (), False, None, ""
        if not yaw_assist_enabled or len(metrics.arm_tip_points) != 2:
            return False, decision.failures, False, None, ""
        if metrics.vertex_point is None and metrics.deepest_point is None:
            return False, decision.failures, False, None, ""

        direction_free_thresholds = replace(
            thresholds,
            min_vertex_angle_deg=0.0,
            max_vertex_angle_deg=180.0,
            min_arm_balance=0.0,
            min_endpoint_turn_deg=0.0,
            min_vertex_turn_deg=0.0,
            max_vertex_turn_deg=180.0,
            min_notch_turn_deg=0.0,
            max_negative_turn_deg=180.0,
        )
        base_decision = evaluate_contour_metrics(metrics, direction_free_thresholds)
        if not base_decision.passed:
            return False, decision.failures, False, None, ""
        try:
            direction_error, direction_source = self.contour_yaw_direction_error(
                component,
                metrics,
            )
        except ValueError as exc:
            return (
                False,
                (*decision.failures, f"Yaw辅助不可用：{exc}"),
                False,
                None,
                "",
            )
        if direction_error <= yaw_tolerance_deg:
            return True, (), True, direction_error, direction_source
        return (
            False,
            (
                *decision.failures,
                f"Yaw方向误差 {direction_error:.3f} > {yaw_tolerance_deg:.3f}",
            ),
            False,
            direction_error,
            direction_source,
        )

    def recognize_current_image_arrows_only(self) -> None:
        self.recognition_mode_var.set(RECOGNITION_MODE_CONTOUR)
        self.recognize_current_image_contours(TARGET_ARROW)

    def recognize_current_image_beacons_only(self) -> None:
        self.recognition_mode_var.set(RECOGNITION_MODE_CONTOUR)
        self.recognize_current_image_contours(TARGET_BEACON)

    def recognize_current_image_contours(self, target_only: str | None = None) -> None:
        if target_only not in (None, TARGET_ARROW, TARGET_BEACON):
            raise ValueError("轮廓单类型测试目标无效")
        if self.undistorted is None or self.binary is None:
            messagebox.showinfo("轮廓识别", "请先加载一张图像。")
            return
        if (
            self.candidate_mode_var.get() != CANDIDATE_MODE_CONTOUR
            or any(
                component.extraction_method != "contour"
                for component in self.components
            )
        ):
            self.candidate_mode_var.set(CANDIDATE_MODE_CONTOUR)
            self.connectivity_var.set(8)
            self.run_analysis()
        if not self.components:
            self.invalidate_beacon_recognition()
            self.update_component_tree()
            self.view_var.set("连通域候选")
            self.status_var.set("轮廓识别完成：当前图像无连通域候选")
            return

        try:
            thresholds = {
                target_kind: self.current_contour_thresholds(target_kind)
                for target_kind in MATCHING_TARGET_KINDS
            }
            yaw_assist_enabled, yaw_tolerance_deg = (
                self.current_contour_yaw_assist()
            )
        except ValueError as exc:
            messagebox.showerror("轮廓识别参数错误", str(exc))
            self.status_var.set(f"轮廓识别失败：{exc}")
            return

        results: dict[int, BeaconRecognitionResult] = {}
        arrow_found = False
        beacon_found_count = 0
        beacon_accept_limit = self.current_beacon_accept_limit()
        binary_threshold = self.safe_int(self.threshold_var, 50, 0, 255)
        for component in self.components:
            try:
                contour_metrics = calculate_contour_metrics(
                    component,
                    self.undistorted,
                    self.binary,
                    thresholds[TARGET_ARROW],
                    negative_turn_window_points=(
                        thresholds[TARGET_BEACON].direction_window_points
                    ),
                )
            except (ValueError, OverflowError) as exc:
                results[component.label] = BeaconRecognitionResult(
                    metrics=None,
                    passed=False,
                    failures=(f"轮廓指标计算失败：{exc}",),
                    route=(
                        f"仅测{MATCHING_TARGET_NAMES[target_only]}"
                        if target_only is not None
                        else "信标优先"
                    ),
                    attempts=(),
                    binary_threshold=binary_threshold,
                    recognition_engine=RECOGNITION_MODE_CONTOUR,
                )
                continue

            attempts: list[str] = []
            failures: list[str] = []
            accepted_kind: str | None = None
            yaw_assisted = False
            yaw_direction_error: float | None = None
            yaw_direction_source = ""

            if target_only == TARGET_ARROW:
                (
                    arrow_passed,
                    arrow_failures,
                    yaw_assisted,
                    yaw_direction_error,
                    yaw_direction_source,
                ) = self.evaluate_contour_arrow_with_yaw(
                    component,
                    contour_metrics,
                    thresholds[TARGET_ARROW],
                    yaw_assist_enabled,
                    yaw_tolerance_deg,
                )
                attempts.append(
                    "箭头Yaw辅助通过"
                    if yaw_assisted
                    else f"箭头{'通过' if arrow_passed else '失败'}"
                )
                if arrow_passed:
                    accepted_kind = TARGET_ARROW
                else:
                    failures.extend(arrow_failures)
            elif target_only == TARGET_BEACON:
                beacon_decision = evaluate_contour_metrics(
                    contour_metrics,
                    thresholds[TARGET_BEACON],
                )
                attempts.append(
                    f"信标{'通过' if beacon_decision.passed else '失败'}"
                )
                if beacon_decision.passed:
                    accepted_kind = TARGET_BEACON
                else:
                    failures.extend(beacon_decision.failures)
            else:
                if beacon_found_count < beacon_accept_limit:
                    beacon_decision = evaluate_contour_metrics(
                        contour_metrics,
                        thresholds[TARGET_BEACON],
                    )
                    attempts.append(
                        f"信标{'通过' if beacon_decision.passed else '失败'}"
                    )
                    if beacon_decision.passed:
                        accepted_kind = TARGET_BEACON
                        beacon_found_count += 1
                    else:
                        failures.extend(
                            f"信标：{failure}"
                            for failure in beacon_decision.failures
                        )
                else:
                    attempts.append(
                        f"信标跳过（已达到上限 {beacon_accept_limit}）"
                    )
                    failures.append(
                        f"信标：已达到单帧上限 {beacon_accept_limit}"
                    )

                if accepted_kind is None:
                    if arrow_found:
                        attempts.append("箭头跳过（本帧已有箭头）")
                        failures.append("箭头：本帧已有箭头")
                    else:
                        (
                            arrow_passed,
                            arrow_failures,
                            yaw_assisted,
                            yaw_direction_error,
                            yaw_direction_source,
                        ) = self.evaluate_contour_arrow_with_yaw(
                            component,
                            contour_metrics,
                            thresholds[TARGET_ARROW],
                            yaw_assist_enabled,
                            yaw_tolerance_deg,
                        )
                        attempts.append(
                            "箭头Yaw辅助通过"
                            if yaw_assisted
                            else f"箭头{'通过' if arrow_passed else '失败'}"
                        )
                        if arrow_passed:
                            accepted_kind = TARGET_ARROW
                            arrow_found = True
                        else:
                            failures.extend(
                                f"箭头：{failure}" for failure in arrow_failures
                            )

            results[component.label] = BeaconRecognitionResult(
                metrics=None,
                passed=accepted_kind is not None,
                failures=() if accepted_kind is not None else tuple(failures),
                box_clipped=contour_metrics.boundary_clipped,
                target_kind=accepted_kind,
                route=(
                    f"仅测{MATCHING_TARGET_NAMES[target_only]}"
                    if target_only is not None
                    else "信标优先"
                ),
                attempts=tuple(attempts),
                binary_threshold=binary_threshold,
                contour_metrics=contour_metrics,
                recognition_engine=RECOGNITION_MODE_CONTOUR,
                yaw_assisted=yaw_assisted,
                yaw_direction_error_deg=yaw_direction_error,
                yaw_direction_source=yaw_direction_source,
            )

        self.beacon_recognition_results = results
        self.update_component_tree()
        self.view_var.set("连通域候选")
        self.redraw_canvas()
        arrow_count = sum(
            result.passed and result.target_kind == TARGET_ARROW
            for result in results.values()
        )
        beacon_count = sum(
            result.passed and result.target_kind == TARGET_BEACON
            for result in results.values()
        )
        test_mode = (
            f"仅测{MATCHING_TARGET_NAMES[target_only]}"
            if target_only is not None
            else "混合识别"
        )
        yaw_status = (
            "不适用"
            if target_only == TARGET_BEACON
            else f"开({yaw_tolerance_deg:.1f}°)" if yaw_assist_enabled else "关"
        )
        self.status_var.set(
            f"轮廓几何识别完成[{test_mode}]：箭头 {arrow_count}，"
            f"信标 {beacon_count}，候选 {len(results)} | "
            f"固定阈值 {binary_threshold} | Yaw辅助 {yaw_status}"
        )

    def recognize_current_image_targets(self) -> None:
        if self.recognition_mode_var.get() == RECOGNITION_MODE_CONTOUR:
            self.recognize_current_image_contours()
            return
        if self.undistorted is None:
            messagebox.showinfo("新方案识别", "请先加载一张图像。")
            return
        if not self.components:
            self.invalidate_beacon_recognition()
            self.update_component_tree()
            self.view_var.set("连通域候选")
            self.status_var.set("新方案识别完成：当前图像无连通域候选")
            return

        try:
            roll = float(self.roll_var.get())
            pitch = float(self.pitch_var.get())
            height = float(self.height_var.get())
            uav_yaw = float(self.uav_yaw_var.get())
            car_yaw = float(self.car_yaw_var.get())
            arrow_plane_height = float(self.arrow_plane_height_var.get())
            beacon_plane_height = float(self.beacon_plane_height_var.get())
            arm_length = float(self.arrow_arm_length_var.get())
            arrow_angle = float(self.arrow_angle_var.get())
            opening_offset = float(self.arrow_opening_offset_var.get())
            beacon_diameter = float(self.beacon_diameter_var.get())
            mount_roll = float(self.mount_roll_var.get())
            mount_pitch = float(self.mount_pitch_var.get())
            mount_yaw = float(self.mount_yaw_var.get())
            values = (
                roll, pitch, height, uav_yaw, car_yaw,
                arrow_plane_height, beacon_plane_height,
                arm_length, arrow_angle, opening_offset, beacon_diameter,
                mount_roll, mount_pitch, mount_yaw,
            )
            if not all(math.isfinite(value) for value in values):
                raise ValueError("姿态、高度和目标参数必须是有限数值")
            if height <= 0.0:
                raise ValueError("无人机高度必须大于0")
            if min(arrow_plane_height, beacon_plane_height) < 0.0:
                raise ValueError("目标平面高度不能小于0")
            if max(arrow_plane_height, beacon_plane_height) >= (
                height - self.projection_calibration.offset_z_cm
            ):
                raise ValueError("目标平面必须低于相机")
            self.target_projector.set_mount_angles(
                mount_roll, mount_pitch, mount_yaw
            )
            sampling_configs = {
                target_kind: self.current_sampling_config(target_kind)
                for target_kind in MATCHING_TARGET_KINDS
            }
            near_negative_width = self.current_near_negative_width()
            binary_threshold = self.safe_int(self.threshold_var, 50, 0, 255)
            relative_yaw = wrap_degrees(
                car_yaw if self.car_yaw_relative_var.get() else car_yaw - uav_yaw
            )
        except ValueError as exc:
            messagebox.showerror("新方案参数错误", str(exc))
            self.status_var.set(f"新方案识别失败：{exc}")
            return

        def evaluate_target(
            component: ConnectedComponent,
            target_kind: str,
        ) -> BeaconRecognitionResult:
            center_image = (component.centroid_x, component.centroid_y)
            sampling_config = sampling_configs[target_kind]
            plane_height = (
                arrow_plane_height if target_kind == TARGET_ARROW
                else beacon_plane_height
            )
            try:
                if target_kind == TARGET_ARROW:
                    shape_gate_enabled, max_occupancy, max_horizontal_run = (
                        self.current_arrow_shape_gate()
                    )
                    if shape_gate_enabled:
                        shape_failures: list[str] = []
                        if component.occupancy > max_occupancy:
                            shape_failures.append(
                                f"箭头占空比 {component.occupancy:.3f} > "
                                f"{max_occupancy:.3f}"
                            )
                        if component.horizontal_run_ratio > max_horizontal_run:
                            shape_failures.append(
                                f"最长水平段比例 "
                                f"{component.horizontal_run_ratio:.3f} > "
                                f"{max_horizontal_run:.3f}"
                            )
                        if shape_failures:
                            return BeaconRecognitionResult(
                                metrics=None,
                                passed=False,
                                failures=tuple(shape_failures),
                                target_kind=TARGET_ARROW,
                            )
                center_plane = self.target_projector.image_to_plane(
                    center_image, roll, pitch, height, plane_height
                )
                if center_plane is None:
                    raise ValueError("候选质心无法投影到目标平面")
                if target_kind == TARGET_ARROW:
                    physical_points = build_ideal_chevron_points(
                        center_plane,
                        arm_length,
                        arrow_angle,
                        wrap_degrees(relative_yaw + opening_offset),
                    )
                    outline = []
                    for name in ("left_tip", "apex", "right_tip"):
                        projected = self.target_projector.plane_to_image(
                            physical_points[name], roll, pitch, height, plane_height
                        )
                        if projected is None:
                            raise ValueError("箭头端点位于相机后方")
                        outline.append(projected)
                    closed = False
                    filled_positive = False
                else:
                    physical_points = build_ideal_beacon_cardinal_points(
                        center_plane, beacon_diameter
                    )
                    projected_points: dict[str, tuple[float, float]] = {
                        "center": center_image,
                    }
                    for name in ("left", "right", "up", "down"):
                        projected = self.target_projector.plane_to_image(
                            physical_points[name], roll, pitch, height, plane_height
                        )
                        if projected is None:
                            raise ValueError("信标基准点位于相机后方")
                        projected_points[name] = projected
                    outline = list(build_projected_ellipse_outline(
                        projected_points["center"],
                        projected_points["left"],
                        projected_points["right"],
                        projected_points["up"],
                        projected_points["down"],
                    ))
                    closed = True
                    filled_positive = True
                sampling = sample_projected_shape(
                    self.undistorted,
                    outline,
                    closed=closed,
                    filled_positive=filled_positive,
                    config=sampling_config,
                    is_valid_pixel=self.undistorter.is_valid_output_pixel,
                    collect_near_shape=self.near_shape_variance_enabled_var.get(),
                )
                metrics = calculate_matching_metrics(
                    self.undistorted,
                    outline,
                    sampling,
                    target_kind=target_kind,
                    center=center_image,
                    binary_threshold=binary_threshold,
                    negative_guard_px=sampling_config.negative_guard_px,
                    near_negative_width_px=near_negative_width,
                )
                decision = evaluate_matching_metrics(
                    metrics,
                    self.current_matching_thresholds(target_kind),
                    target_kind=target_kind,
                )
                return BeaconRecognitionResult(
                    metrics=metrics,
                    passed=decision.passed,
                    failures=decision.failures,
                    box_clipped=sampling.box_clipped,
                    target_kind=target_kind if decision.passed else None,
                    projected_outline=tuple(outline) if decision.passed else (),
                    sampling=sampling,
                    sampling_config=sampling_config,
                    binary_threshold=binary_threshold,
                    recognition_engine=RECOGNITION_MODE_IDEAL,
                )
            except (ValueError, OverflowError) as exc:
                return BeaconRecognitionResult(
                    metrics=None,
                    passed=False,
                    failures=(f"计算失败：{exc}",),
                    recognition_engine=RECOGNITION_MODE_IDEAL,
                )

        arrow_priority = [
            component for component in self.components
            if max(1, (component.area + 2) // 4) >= FIRMWARE_ROUTE_ARROW_MIN_AREA
        ]
        beacon_priority = [
            component for component in self.components
            if component not in arrow_priority
        ]
        results: dict[int, BeaconRecognitionResult] = {}
        arrow_found = False
        beacon_found_count = 0
        beacon_accept_limit = self.current_beacon_accept_limit()
        for component in (*arrow_priority, *beacon_priority):
            route = (
                "箭头优先" if component in arrow_priority else "信标优先"
            )
            order = (
                (TARGET_ARROW, TARGET_BEACON)
                if route == "箭头优先"
                else (TARGET_BEACON, TARGET_ARROW)
            )
            attempts: list[str] = []
            failures: list[str] = []
            last_result: BeaconRecognitionResult | None = None
            for target_kind in order:
                target_name = "箭头" if target_kind == TARGET_ARROW else "信标"
                if target_kind == TARGET_ARROW and arrow_found:
                    attempts.append("箭头跳过（本帧已有箭头）")
                    continue
                if (
                    target_kind == TARGET_BEACON
                    and beacon_found_count >= beacon_accept_limit
                ):
                    attempts.append(
                        f"信标跳过（已达到上限 {beacon_accept_limit}）"
                    )
                    failures.append(
                        f"信标：已达到单帧上限 {beacon_accept_limit}"
                    )
                    continue
                current = evaluate_target(component, target_kind)
                last_result = current
                attempts.append(f"{target_name}{'通过' if current.passed else '失败'}")
                if current.passed:
                    if target_kind == TARGET_ARROW:
                        arrow_found = True
                    else:
                        beacon_found_count += 1
                    results[component.label] = BeaconRecognitionResult(
                        metrics=current.metrics,
                        passed=True,
                        failures=(),
                        box_clipped=current.box_clipped,
                        target_kind=target_kind,
                        route=route,
                        attempts=tuple(attempts),
                        projected_outline=current.projected_outline,
                        sampling=current.sampling,
                        sampling_config=current.sampling_config,
                        binary_threshold=current.binary_threshold,
                        recognition_engine=RECOGNITION_MODE_IDEAL,
                    )
                    break
                first_failure = current.failures[0] if current.failures else "未通过"
                failures.append(f"{target_name}：{first_failure}")
            else:
                results[component.label] = BeaconRecognitionResult(
                    metrics=last_result.metrics if last_result else None,
                    passed=False,
                    failures=tuple(failures) or ("没有可执行的识别分支",),
                    box_clipped=bool(last_result and last_result.box_clipped),
                    route=route,
                    attempts=tuple(attempts),
                    sampling=last_result.sampling if last_result else None,
                    sampling_config=(
                        last_result.sampling_config if last_result else None
                    ),
                    binary_threshold=(
                        last_result.binary_threshold if last_result else binary_threshold
                    ),
                    recognition_engine=RECOGNITION_MODE_IDEAL,
                )

        self.beacon_recognition_results = results
        self.update_component_tree()
        self.view_var.set("连通域候选")
        self.redraw_canvas()
        arrow_count = sum(
            result.passed and result.target_kind == TARGET_ARROW
            for result in results.values()
        )
        beacon_count = sum(
            result.passed and result.target_kind == TARGET_BEACON
            for result in results.values()
        )
        self.status_var.set(
            f"新方案识别完成：箭头 {arrow_count}，信标 {beacon_count}，"
            f"候选 {len(results)} | 固定阈值 {binary_threshold} | "
            f"面积分流线 {FIRMWARE_ROUTE_ARROW_MIN_AREA}（188x120等效）"
        )

    def component_selected(self, _event: object = None) -> None:
        selection = self.component_tree.selection()
        self.selected_component = int(selection[0]) if selection else None
        if self.view_var.get() == "连通域候选":
            self.redraw_canvas()

    def current_component(self) -> ConnectedComponent | None:
        if self.selected_component is None:
            return None
        return next(
            (component for component in self.components if component.label == self.selected_component),
            None,
        )

    def align_ideal_to_selected_component(self) -> None:
        target = self.ideal_target_var.get()
        if target not in (IDEAL_TARGET_ARROW, IDEAL_TARGET_BEACON):
            messagebox.showinfo("理想目标", "请先选择理想箭头或理想信标。")
            return
        component = self.current_component()
        if component is None:
            messagebox.showinfo("候选质心", "请先在连通域候选表中选择一个候选。")
            return
        self.ideal_centers[target] = (component.centroid_x, component.centroid_y)
        if self.view_var.get() not in IDEAL_BACKGROUND_NAMES:
            self.view_var.set(self.ideal_background_var.get())
        self.redraw_canvas()

    def open_selected_component(self) -> None:
        component = self.current_component()
        if component is None or self.undistorted is None or self.binary is None:
            messagebox.showinfo("候选区域", "请先在候选表中选择一个连通域。")
            return
        padding = 3
        box = (
            max(0, component.min_x - padding),
            max(0, component.min_y - padding),
            min(self.undistorted.width, component.max_x + padding + 1),
            min(self.undistorted.height, component.max_y + padding + 1),
        )
        gray_crop = self.undistorted.crop(box)
        binary_crop = self.binary.crop(box)
        scale = min(12, max(2, int(320 / max(1, gray_crop.width, gray_crop.height))))
        display_size = (gray_crop.width * scale, gray_crop.height * scale)
        gray_overlay = gray_crop.convert("RGB")
        if component.contour_points:
            contour_draw = ImageDraw.Draw(gray_overlay)
            local_points = [
                (point_x - box[0], point_y - box[1])
                for point_x, point_y in component.contour_points
            ]
            if len(local_points) >= 2:
                contour_draw.line(local_points, fill=(45, 205, 255), width=1)
            start_x, start_y = local_points[0]
            contour_draw.ellipse(
                (start_x - 1, start_y - 1, start_x + 1, start_y + 1),
                outline=(255, 230, 80),
            )
        gray_display = gray_overlay.resize(display_size, Image.Resampling.NEAREST)
        binary_display = binary_crop.resize(display_size, Image.Resampling.NEAREST)

        window = tk.Toplevel(self.root)
        window.title(f"候选 #{component.label}")
        window.resizable(True, True)
        window.geometry("960x760")
        window.minsize(760, 560)
        content = ttk.Frame(window, padding=8)
        content.pack(fill=tk.BOTH, expand=True)
        content.columnconfigure(0, weight=1)
        content.columnconfigure(1, weight=1)
        content.rowconfigure(2, weight=1)
        gray_photo = ImageTk.PhotoImage(gray_display)
        binary_photo = ImageTk.PhotoImage(binary_display)
        gray_title = (
            "去畸变灰度图 + 轮廓" if component.contour_points else "去畸变灰度图"
        )
        ttk.Label(content, text=gray_title).grid(row=0, column=0, padx=4)
        ttk.Label(content, text="单阈值二值图").grid(row=0, column=1, padx=4)
        ttk.Label(content, image=gray_photo).grid(row=1, column=0, padx=4, pady=4)
        ttk.Label(content, image=binary_photo).grid(row=1, column=1, padx=4, pady=4)
        coarse_area = max(1, (component.area + 2) // 4)
        details = (
            f"候选 #{component.label}\n"
            f"原图尺寸 {self.undistorted.width}x{self.undistorted.height}    "
            f"连通域面积 {component.area} px    188x120等效面积 {coarse_area} px\n"
            f"质心 x={component.centroid_x:.3f}, y={component.centroid_y:.3f}\n"
            f"外接框 min=({component.min_x}, {component.min_y})  "
            f"max=({component.max_x}, {component.max_y})  "
            f"尺寸 {component.width}x{component.height}  "
            f"框面积 {component.width * component.height} px\n"
            f"边界像素 {component.boundary_pixels}    "
            f"圆度 {component.circularity:.6f}    "
            f"占空比 {component.occupancy:.6f}\n"
            f"最长水平连续段 {component.max_horizontal_run} px    "
            f"最长水平段比例 {component.horizontal_run_ratio:.6f}\n"
        )
        if component.extraction_method == "contour":
            direction_names = ("上", "右上", "右", "右下", "下", "左下", "左", "左上")
            direction_text = "  ".join(
                f"{name}:{count}"
                for name, count in zip(direction_names, component.direction_counts)
            )
            contour_state = "闭合" if component.contour_closed else "未闭合"
            if component.contour_overflow:
                contour_state = "步数溢出"
            details += (
                f"候选提取 八邻域轮廓爬线    状态 {contour_state}    "
                f"轮廓点 {len(component.contour_points)}    "
                f"方向步数 {len(component.contour_directions)}    "
                f"近似周长 {component.contour_perimeter:.3f} px\n"
                f"八方向统计 {direction_text}\n"
                "面积说明：轮廓扫描线跨度近似值，内部封闭孔洞按白色填平。\n"
            )
        else:
            details += "候选提取 RLE连通域（对照模式）\n"
        recognition = self.beacon_recognition_results.get(component.label)
        if recognition is None:
            details += "\n新方案识别：未检测（尚未执行识别）"
        elif recognition.contour_metrics is not None:
            metrics = recognition.contour_metrics
            target_text = (
                "箭头" if recognition.target_kind == TARGET_ARROW
                else "信标" if recognition.target_kind == TARGET_BEACON
                else "未分类"
            )

            def optional_metric(value: float | None) -> str:
                return f"{value:.3f}" if value is not None else "--"

            details += (
                f"\n识别引擎：{RECOGNITION_MODE_CONTOUR}    "
                f"结果：{'通过' if recognition.passed else '未通过'}    "
                f"类型：{target_text}    分流：{recognition.route or '未记录'}\n"
                f"固定阈值：{recognition.binary_threshold if recognition.binary_threshold is not None else '--'}    "
                f"接触图像边缘：{'是' if metrics.boundary_clipped else '否'}\n"
                f"尝试：{' → '.join(recognition.attempts) or '无'}\n"
                f"轮廓点 {metrics.contour_points}    面积 {metrics.area} px    "
                f"周长 {metrics.perimeter:.3f} px\n"
                f"长短边比例 {metrics.aspect_ratio:.3f}    "
                f"圆度 {metrics.circularity:.3f}    "
                f"占空比 {metrics.occupancy:.3f}\n"
                f"凸包填充率 {metrics.solidity:.3f}    "
                f"径向波动 {metrics.radial_cv:.3f}    "
                f"最大凹陷比例 {metrics.concavity_ratio:.3f}    "
                f"方向熵 {metrics.direction_entropy:.3f}\n"
                f"两组主方向占比 {metrics.dominant_axis_ratio:.3f}    "
                f"方向转折率 {metrics.turn_transition_ratio:.3f}    "
                f"方向均值窗口 {metrics.direction_window_points} 点\n"
                f"端点方向差 "
                f"{'/'.join(f'{value:.1f}°' for value in metrics.endpoint_turn_angles) or '--'}    "
                f"外顶点方向差 {optional_metric(metrics.vertex_turn_angle)}°    "
                f"内凹口方向差 {optional_metric(metrics.notch_turn_angle)}°\n"
                f"最大反向转角 {metrics.max_negative_turn_deg:.3f}°"
                f"（窗口 {metrics.negative_turn_window_points} 点）    "
                f"箭头物理夹角 {optional_metric(metrics.vertex_angle_deg)}°    "
                f"候选两臂平衡度 {optional_metric(metrics.arm_balance)}\n"
                f"前景灰度 {optional_metric(metrics.foreground_mean)}"
                f"（{metrics.foreground_samples}点）    "
                f"背景灰度 {optional_metric(metrics.background_mean)}"
                f"（{metrics.background_samples}点）    "
                f"灰度差 {optional_metric(metrics.gray_contrast)}\n"
                "说明：凸包仅用于计算轮廓凹陷和填充率；目标面积仍采用候选轮廓扫描线面积。"
            )
            if recognition.yaw_assisted:
                details += (
                    f"\nYaw辅助：通过    方向点 {recognition.yaw_direction_source}    "
                    f"方向误差 {optional_metric(recognition.yaw_direction_error_deg)}°"
                )
            elif recognition.yaw_direction_error_deg is not None:
                details += (
                    f"\nYaw辅助：未通过    方向点 {recognition.yaw_direction_source}    "
                    f"方向误差 {recognition.yaw_direction_error_deg:.3f}°"
                )
            if recognition.failures:
                details += "\n原因：\n  - " + "\n  - ".join(recognition.failures)
        elif recognition.metrics is None:
            details += (
                f"\n新方案识别：未通过    类型："
                f"{recognition.target_kind or '未分类'}    "
                f"分流：{recognition.route or '未记录'}\n"
                f"固定阈值：{recognition.binary_threshold if recognition.binary_threshold is not None else '--'}\n"
                f"尝试：{' → '.join(recognition.attempts) or '无'}\n"
                "原因：\n  - "
            ) + "\n  - ".join(
                recognition.failures
            )
        else:
            metrics = recognition.metrics

            def metric_text(value: float | None, digits: int = 2) -> str:
                return f"{value:.{digits}f}" if value is not None else "--"

            target_text = (
                "箭头" if recognition.target_kind == TARGET_ARROW
                else "信标" if recognition.target_kind == TARGET_BEACON
                else "未分类"
            )
            details += (
                f"\n新方案识别：{'通过' if recognition.passed else '未通过'}    "
                f"类型：{target_text}    分流：{recognition.route or '未记录'}\n"
                f"固定阈值：{recognition.binary_threshold if recognition.binary_threshold is not None else '--'}    "
                f"采样框裁边：{'是' if recognition.box_clipped else '否'}\n"
                f"尝试：{' → '.join(recognition.attempts) or '无'}\n"
                f"正形均值 {metric_text(metrics.positive_mean, 3)}  "
                f"负形均值 {metric_text(metrics.negative_mean, 3)}  "
                f"近负形均值 {metric_text(metrics.near_negative_mean, 3)}  "
                f"远负形均值 {metric_text(metrics.far_negative_mean, 3)}\n"
                f"局部阈值 {metric_text(metrics.local_threshold, 3)}  "
                f"灰度差 {metric_text(metrics.gray_difference, 3)}  "
                f"标准化分离度 {metric_text(metrics.standardized_separation, 3)}\n"
                f"正形命中率 {metric_text(metrics.positive_hit_rate, 3)}  "
                f"负形命中率 {metric_text(metrics.negative_hit_rate, 3)}  "
                f"固定阈值一致率 {metric_text(metrics.binary_consistency, 3)}\n"
                f"完整度 {metric_text(metrics.completeness, 3)}  "
                f"近负形差 {metric_text(metrics.near_difference, 3)}  "
                f"远背景差 {metric_text(metrics.far_difference, 3)}\n"
                f"正形点 {metrics.positive_count}  负形点 {metrics.negative_count}  "
                f"近负形点 {metrics.near_negative_count}  远背景点 {metrics.far_negative_count}\n"
                f"近形点 {metrics.near_shape_count}  "
                f"近形灰度方差 {metric_text(metrics.near_shape_variance, 3)}"
            )
            if recognition.sampling is not None:
                sampling = recognition.sampling
                details += (
                    "\n\n采样统计："
                    f"\n  范围 {sampling.bounds or '--'}"
                    f"\n  正形 {len(sampling.positive_points)}"
                    f"  负形 {len(sampling.negative_points)}"
                    f"  近形 {len(sampling.near_shape_points)}"
                    f"  忽略 {sampling.ignored_count}"
                    f"  无效 {sampling.invalid_count}"
                    f"\n  采样框裁边：{'是' if sampling.box_clipped else '否'}"
                )
            if recognition.sampling_config is not None:
                config = recognition.sampling_config
                details += (
                    "\n采样参数："
                    f"\n  正形步长 {config.positive_step_px} px"
                    f"  负形步长 {config.negative_step_px} px"
                    f"  框余量 {config.margin_px:.3f} px"
                    f"\n  正形轮廓带 {config.positive_band_px:.3f} px"
                    f"  负形最小距离 {config.negative_guard_px:.3f} px"
                    f"  近形方差统计："
                    f"{'开启' if self.near_shape_variance_enabled_var.get() else '关闭'}"
                )
            if recognition.projected_outline:
                outline_text = " ".join(
                    f"({point[0]:.3f},{point[1]:.3f})"
                    for point in recognition.projected_outline
                )
                details += f"\n理想轮廓投影点：{outline_text}"
            if recognition.failures:
                details += "\n原因：\n  - " + "\n  - ".join(recognition.failures)
        details_frame = ttk.Frame(content)
        details_frame.grid(row=2, column=0, columnspan=2, sticky="nsew", padx=4, pady=(5, 0))
        details_frame.columnconfigure(0, weight=1)
        details_frame.rowconfigure(0, weight=1)
        details_text = tk.Text(
            details_frame,
            width=112,
            height=24,
            wrap=tk.NONE,
            font=("Microsoft YaHei UI", 10),
        )
        details_text.grid(row=0, column=0, sticky="nsew")
        details_y_scroll = ttk.Scrollbar(
            details_frame, orient=tk.VERTICAL, command=details_text.yview
        )
        details_y_scroll.grid(row=0, column=1, sticky="ns")
        details_x_scroll = ttk.Scrollbar(
            details_frame, orient=tk.HORIZONTAL, command=details_text.xview
        )
        details_x_scroll.grid(row=1, column=0, sticky="ew")
        details_text.configure(
            yscrollcommand=details_y_scroll.set,
            xscrollcommand=details_x_scroll.set,
        )
        details_text.insert("1.0", details)
        details_text.configure(state=tk.DISABLED)
        window.gray_photo = gray_photo  # type: ignore[attr-defined]
        window.binary_photo = binary_photo  # type: ignore[attr-defined]
        window.details_text = details_text  # type: ignore[attr-defined]

    def canvas_configured(self, _event: object = None) -> None:
        if self._fit_pending:
            self._fit_pending = False
            self.fit_image()

    def get_view_image(self) -> Image.Image | None:
        view = self.view_var.get()
        if view == "原始灰度图":
            return self.original
        if view == "去畸变灰度图":
            return self.undistorted
        if view == "单阈值二值图":
            return self.binary
        if view == "连通域候选" and self.undistorted is not None:
            return self.undistorted.convert("RGB")
        return None

    def get_gray_reference(self) -> Image.Image | None:
        return self.original if self.view_var.get() == "原始灰度图" else self.undistorted

    def redraw_canvas(self) -> None:
        image = self.get_view_image()
        if image is None:
            self.canvas.delete("all")
            return
        width = max(1, int(round(image.width * self.zoom)))
        height = max(1, int(round(image.height * self.zoom)))
        resized = image.resize((width, height), Image.Resampling.NEAREST)
        self.canvas_photo = ImageTk.PhotoImage(resized)
        self.canvas.delete("all")
        self.canvas.create_image(
            self.offset_x,
            self.offset_y,
            anchor=tk.NW,
            image=self.canvas_photo,
            tags="frame",
        )
        if self.view_var.get() == "连通域候选":
            self.draw_recognition_overlay()
        self.draw_ideal_target_overlay(image)

    def draw_recognition_overlay(self) -> None:
        arrow_color = "#ff9f1c"
        beacon_color = "#18c7a3"
        pending_color = "#6f8fa8"
        failed_color = "#7d858c"
        selected_color = "#ffd54f"
        occupied_labels: list[tuple[int, int, int, int]] = []
        image_bounds = (
            self.offset_x,
            self.offset_y,
            self.offset_x + self.calibration.output_width * self.zoom,
            self.offset_y + self.calibration.output_height * self.zoom,
        )

        def canvas_point(point: tuple[float, float]) -> tuple[float, float]:
            return (
                self.offset_x + point[0] * self.zoom,
                self.offset_y + point[1] * self.zoom,
            )

        def boxes_overlap(
            left: tuple[int, int, int, int],
            right: tuple[int, int, int, int],
            padding: int = 3,
        ) -> bool:
            return not (
                left[2] + padding < right[0]
                or right[2] + padding < left[0]
                or left[3] + padding < right[1]
                or right[3] + padding < left[1]
            )

        def draw_label(
            text: str,
            color: str,
            box: tuple[float, float, float, float],
        ) -> None:
            min_x, min_y, max_x, max_y = box
            placements = (
                (min_x, min_y - 4, tk.SW),
                (min_x, max_y + 4, tk.NW),
                (max_x + 5, min_y, tk.NW),
                (min_x - 5, min_y, tk.NE),
            )
            text_id = self.canvas.create_text(
                placements[0][0],
                placements[0][1],
                text=text,
                fill=color,
                anchor=placements[0][2],
                font=("Microsoft YaHei UI", 9, "bold"),
                tags="recognition_overlay",
            )
            chosen = placements[0]
            chosen_bounds = self.canvas.bbox(text_id)
            for placement in placements:
                self.canvas.coords(text_id, placement[0], placement[1])
                self.canvas.itemconfigure(text_id, anchor=placement[2])
                bounds = self.canvas.bbox(text_id)
                if bounds is None:
                    continue
                inside_image = (
                    bounds[0] >= image_bounds[0]
                    and bounds[1] >= image_bounds[1]
                    and bounds[2] <= image_bounds[2]
                    and bounds[3] <= image_bounds[3]
                )
                if inside_image and not any(
                    boxes_overlap(bounds, occupied) for occupied in occupied_labels
                ):
                    chosen = placement
                    chosen_bounds = bounds
                    break
            self.canvas.coords(text_id, chosen[0], chosen[1])
            self.canvas.itemconfigure(text_id, anchor=chosen[2])
            chosen_bounds = self.canvas.bbox(text_id) or chosen_bounds
            shadow_id = self.canvas.create_text(
                chosen[0] + 1,
                chosen[1] + 1,
                text=text,
                fill="#101418",
                anchor=chosen[2],
                font=("Microsoft YaHei UI", 9, "bold"),
                tags="recognition_overlay",
            )
            self.canvas.tag_lower(shadow_id, text_id)
            if chosen_bounds is not None:
                occupied_labels.append(chosen_bounds)

        for component in sorted(self.components, key=lambda item: (item.min_y, item.min_x)):
            recognition = self.beacon_recognition_results.get(component.label)
            accepted_arrow = bool(
                recognition
                and recognition.passed
                and recognition.target_kind == TARGET_ARROW
            )
            accepted_beacon = bool(
                recognition
                and recognition.passed
                and recognition.target_kind == TARGET_BEACON
            )
            if accepted_arrow:
                target_color = arrow_color
                label = f"箭头 #{component.label}"
            elif accepted_beacon:
                target_color = beacon_color
                label = f"信标 #{component.label}"
            elif recognition is None:
                target_color = pending_color
                label = f"#{component.label}"
            else:
                target_color = failed_color
                label = f"#{component.label}"

            selected = component.label == self.selected_component
            box_color = selected_color if selected else target_color
            min_x, min_y = canvas_point((component.min_x, component.min_y))
            max_x, max_y = canvas_point((component.max_x + 1, component.max_y + 1))
            dash = (5, 4) if recognition is not None and not recognition.passed else ()
            self.canvas.create_rectangle(
                min_x,
                min_y,
                max_x,
                max_y,
                outline=box_color,
                width=2 if selected or accepted_arrow or accepted_beacon else 1,
                dash=dash,
                tags="recognition_overlay",
            )

            if component.contour_points:
                contour_points = list(component.contour_points)
                contour_step = max(1, math.ceil(len(contour_points) / 512))
                contour_display = contour_points[::contour_step]
                if contour_display[-1] != contour_points[-1]:
                    contour_display.append(contour_points[-1])
                if len(contour_display) >= 2:
                    flat_contour = [
                        value
                        for point in contour_display
                        for value in canvas_point(point)
                    ]
                    self.canvas.create_line(
                        *flat_contour,
                        fill="#4ac6ff" if selected else "#3e7388",
                        width=2 if selected else 1,
                        joinstyle=tk.ROUND,
                        tags="recognition_overlay",
                    )
                if selected:
                    start_x, start_y = canvas_point(contour_points[0])
                    self.canvas.create_oval(
                        start_x - 5,
                        start_y - 5,
                        start_x + 5,
                        start_y + 5,
                        outline="#ffe66d",
                        width=2,
                        tags="recognition_overlay",
                    )
                    arrow_step = max(1, len(component.contour_directions) // 24)
                    for index in range(0, len(component.contour_directions), arrow_step):
                        if index + 1 >= len(contour_points):
                            break
                        start = canvas_point(contour_points[index])
                        end = canvas_point(contour_points[index + 1])
                        self.canvas.create_line(
                            start[0],
                            start[1],
                            end[0],
                            end[1],
                            fill="#ffe66d",
                            width=2,
                            arrow=tk.LAST,
                            tags="recognition_overlay",
                        )

            contour_metrics = (
                recognition.contour_metrics if recognition is not None else None
            )
            if contour_metrics is not None:
                tip_points = [
                    canvas_point(point) for point in contour_metrics.arm_tip_points
                ]
                vertex = (
                    canvas_point(contour_metrics.vertex_point)
                    if contour_metrics.vertex_point is not None
                    else None
                )
                if len(tip_points) == 2 and vertex is not None:
                    first_tip, second_tip = tip_points
                    self.canvas.create_line(
                        first_tip[0],
                        first_tip[1],
                        vertex[0],
                        vertex[1],
                        second_tip[0],
                        second_tip[1],
                        fill="#00d2d3",
                        width=2,
                        dash=(3, 2),
                        tags="recognition_overlay",
                    )

                def draw_direction_marker(
                    point: tuple[float, float],
                    marker: str,
                    color: str,
                    radius: int,
                ) -> None:
                    self.canvas.create_oval(
                        point[0] - radius,
                        point[1] - radius,
                        point[0] + radius,
                        point[1] + radius,
                        fill="#101418",
                        outline=color,
                        width=2,
                        tags="recognition_overlay",
                    )
                    self.canvas.create_text(
                        point[0] + radius + 2,
                        point[1] - radius - 1,
                        text=marker,
                        fill=color,
                        anchor=tk.SW,
                        font=("Microsoft YaHei UI", 8, "bold"),
                        tags="recognition_overlay",
                    )

                for index, point in enumerate(tip_points, start=1):
                    draw_direction_marker(point, f"E{index}", "#ff9f43", 3)
                if vertex is not None:
                    draw_direction_marker(vertex, "V", "#ff4d6d", 4)
                if contour_metrics.deepest_point is not None:
                    notch = canvas_point(contour_metrics.deepest_point)
                    draw_direction_marker(notch, "N", "#9b6cff", 3)

            outline = recognition.projected_outline if recognition is not None else ()
            if (accepted_arrow or accepted_beacon) and outline:
                outline_points = [canvas_point(point) for point in outline]
                if accepted_beacon:
                    outline_points.append(outline_points[0])
                flat_outline = [value for point in outline_points for value in point]
                self.canvas.create_line(
                    *flat_outline,
                    fill=target_color,
                    width=3 if accepted_arrow else 2,
                    joinstyle=tk.ROUND,
                    tags="recognition_overlay",
                )
                if accepted_arrow:
                    for index, (point_x, point_y) in enumerate(outline_points):
                        radius = 4 if index == 1 else 2.5
                        self.canvas.create_oval(
                            point_x - radius,
                            point_y - radius,
                            point_x + radius,
                            point_y + radius,
                            outline=target_color,
                            width=2,
                            tags="recognition_overlay",
                        )

            center_x, center_y = canvas_point(
                (component.centroid_x, component.centroid_y)
            )
            if accepted_beacon:
                radius = 4
                self.canvas.create_oval(
                    center_x - radius,
                    center_y - radius,
                    center_x + radius,
                    center_y + radius,
                    outline=target_color,
                    width=2,
                    tags="recognition_overlay",
                )
            else:
                radius = 4 if accepted_arrow else 2
                self.canvas.create_line(
                    center_x - radius,
                    center_y,
                    center_x + radius,
                    center_y,
                    fill=target_color,
                    width=2,
                    tags="recognition_overlay",
                )
                self.canvas.create_line(
                    center_x,
                    center_y - radius,
                    center_x,
                    center_y + radius,
                    fill=target_color,
                    width=2,
                    tags="recognition_overlay",
                )
            draw_label(label, target_color, (min_x, min_y, max_x, max_y))

    def draw_ideal_target_overlay(self, background: Image.Image) -> None:
        target = self.ideal_target_var.get()
        self.sampling_count_var.set("采样点总数：--")
        if target == IDEAL_TARGET_NONE:
            self.ideal_status_var.set("选择目标后可在图像中左键拖动")
            self.sampling_summary_var.set(
                "正形灰度 --（0点） | 负形灰度 --（0点） | 近形方差未启用"
            )
            self.sampling_count_var.set(
                "采样点总数：0 | 参与评分：0（正形 0 / 负形 0） | 近形 0"
            )
            return
        if self.view_var.get() not in IDEAL_BACKGROUND_NAMES:
            self.ideal_status_var.set("理想目标仅叠加在去畸变灰度图或单阈值二值图")
            self.sampling_summary_var.set("当前显示模式不执行正负形采样")
            return
        calibration = self.projection_calibration
        if background.size != (calibration.output_width, calibration.output_height):
            self.ideal_status_var.set("当前背景不是376x240去畸变图")
            return

        try:
            roll = float(self.roll_var.get())
            pitch = float(self.pitch_var.get())
            height = float(self.height_var.get())
            mount_roll = float(self.mount_roll_var.get())
            mount_pitch = float(self.mount_pitch_var.get())
            mount_yaw = float(self.mount_yaw_var.get())
            self.target_projector.set_mount_angles(
                mount_roll, mount_pitch, mount_yaw
            )
            if height <= 0.0:
                raise ValueError("无人机高度必须大于0")
            if target == IDEAL_TARGET_ARROW:
                plane_height = float(self.arrow_plane_height_var.get())
            elif target == IDEAL_TARGET_BEACON:
                plane_height = float(self.beacon_plane_height_var.get())
            else:
                raise ValueError("目标类型无效")
            if plane_height < 0.0:
                raise ValueError("目标平面高度不能小于0")
            if plane_height >= height - calibration.offset_z_cm:
                raise ValueError("目标平面必须低于相机")
        except ValueError as exc:
            self.ideal_status_var.set(f"参数错误：{exc}")
            return

        center_image = self.ideal_centers[target]
        center_plane = self.target_projector.image_to_plane(
            center_image, roll, pitch, height, plane_height
        )
        if center_plane is None:
            self.ideal_status_var.set("目标中心无法投影到物理平面")
            return

        projected_points: list[tuple[float, float]] = []
        status_detail = ""
        color = "#00e5ff"
        try:
            if target == IDEAL_TARGET_ARROW:
                arm_length = float(self.arrow_arm_length_var.get())
                opening_angle = float(self.arrow_angle_var.get())
                opening_offset = float(self.arrow_opening_offset_var.get())
                uav_yaw = float(self.uav_yaw_var.get())
                car_yaw = float(self.car_yaw_var.get())
                relative_yaw = (
                    car_yaw if self.car_yaw_relative_var.get() else car_yaw - uav_yaw
                )
                relative_yaw = wrap_degrees(relative_yaw)
                opening_direction = wrap_degrees(relative_yaw + opening_offset)
                points = build_ideal_chevron_points(
                    center_plane, arm_length, opening_angle, opening_direction
                )
                for name in ("left_tip", "apex", "right_tip"):
                    projected = self.target_projector.plane_to_image(
                        points[name], roll, pitch, height, plane_height
                    )
                    if projected is None:
                        raise ValueError("箭头端点位于相机后方")
                    projected_points.append(projected)
                status_detail = (
                    f"臂长 {arm_length:.1f}cm，夹角 {opening_angle:.1f}°，"
                    f"相对Yaw {relative_yaw:.1f}°"
                )
                color = "#ffb000"
            else:
                diameter = float(self.beacon_diameter_var.get())
                points = build_ideal_beacon_points(center_plane, diameter)
                for point in points:
                    projected = self.target_projector.plane_to_image(
                        point, roll, pitch, height, plane_height
                    )
                    if projected is None:
                        raise ValueError("信标轮廓位于相机后方")
                    projected_points.append(projected)
                projected_points.append(projected_points[0])
                status_detail = f"直径 {diameter:.1f}cm"
        except ValueError as exc:
            self.ideal_status_var.set(f"参数错误：{exc}")
            return

        sampling_outline = (
            projected_points
            if target == IDEAL_TARGET_ARROW
            else projected_points[:-1]
        )
        self.update_sampling_overlay(target, sampling_outline)

        canvas_points = [
            (
                self.offset_x + point[0] * self.zoom,
                self.offset_y + point[1] * self.zoom,
            )
            for point in projected_points
        ]
        flat_points = [coordinate for point in canvas_points for coordinate in point]
        self.canvas.create_line(
            *flat_points,
            fill=color,
            width=3,
            tags="ideal_target",
        )
        if target == IDEAL_TARGET_ARROW:
            for point_x, point_y in canvas_points:
                self.canvas.create_oval(
                    point_x - 4,
                    point_y - 4,
                    point_x + 4,
                    point_y + 4,
                    outline=color,
                    width=2,
                    tags="ideal_target",
                )

        center_canvas = (
            self.offset_x + center_image[0] * self.zoom,
            self.offset_y + center_image[1] * self.zoom,
        )
        self.canvas.create_line(
            center_canvas[0] - 6,
            center_canvas[1],
            center_canvas[0] + 6,
            center_canvas[1],
            fill=color,
            width=2,
            tags="ideal_target",
        )
        self.canvas.create_line(
            center_canvas[0],
            center_canvas[1] - 6,
            center_canvas[0],
            center_canvas[1] + 6,
            fill=color,
            width=2,
            tags="ideal_target",
        )
        self.canvas.create_text(
            center_canvas[0] + 8,
            center_canvas[1] - 8,
            text=target,
            fill=color,
            anchor=tk.SW,
            tags="ideal_target",
        )

        checked_points = (
            projected_points if target == IDEAL_TARGET_ARROW else projected_points[:-1]
        )
        outside_count = sum(
            not self.target_projector.is_inside_image(point)
            for point in checked_points
        )
        outside_text = f"，{outside_count}个轮廓点在画面外" if outside_count else ""
        self.ideal_status_var.set(
            f"中心物理坐标 ({center_plane[0]:.1f}, {center_plane[1]:.1f})cm；"
            f"{status_detail}；安装RPY "
            f"({mount_roll:.1f}°, {mount_pitch:.1f}°, {mount_yaw:.1f}°)"
            f"{outside_text}"
        )

    def current_sampling_config(self, target_kind: str) -> SamplingConfig:
        if target_kind not in MATCHING_TARGET_KINDS:
            raise ValueError("采样目标类型无效")
        variables = self.sampling_vars[target_kind]
        target_name = "箭头" if target_kind == TARGET_ARROW else "信标"
        try:
            positive_step = int(variables["positive_step_px"].get())
            negative_step = int(variables["negative_step_px"].get())
            margin = float(variables["margin_px"].get())
            positive_band = float(variables["positive_band_px"].get())
            negative_guard = float(variables["negative_guard_px"].get())
        except (TypeError, ValueError, tk.TclError) as exc:
            raise ValueError(f"{target_name}采样参数必须是数字") from exc
        config = SamplingConfig(
            positive_step_px=positive_step,
            negative_step_px=negative_step,
            margin_px=margin,
            positive_band_px=positive_band,
            negative_guard_px=negative_guard,
        )
        try:
            config.validate()
        except ValueError as exc:
            raise ValueError(f"{target_name}{exc}") from exc
        return config

    def update_sampling_overlay(
        self,
        target: str,
        outline: list[tuple[float, float]],
    ) -> None:
        if self.undistorted is None:
            self.sampling_summary_var.set("尚未生成去畸变灰度图")
            return
        target_kind = TARGET_ARROW if target == IDEAL_TARGET_ARROW else TARGET_BEACON
        try:
            sampling_config = self.current_sampling_config(target_kind)
            result = sample_projected_shape(
                self.undistorted,
                outline,
                closed=target == IDEAL_TARGET_BEACON,
                filled_positive=target == IDEAL_TARGET_BEACON,
                config=sampling_config,
                is_valid_pixel=self.undistorter.is_valid_output_pixel,
                collect_near_shape=self.near_shape_variance_enabled_var.get(),
            )
        except ValueError as exc:
            self.sampling_summary_var.set(f"采样参数错误：{exc}")
            return
        positive_count = len(result.positive_points)
        negative_count = len(result.negative_points)
        near_shape_count = len(result.near_shape_points)
        scored_count = positive_count + negative_count
        total_count = (
            scored_count
            + near_shape_count
            + result.ignored_count
            + result.invalid_count
        )
        self.sampling_count_var.set(
            f"采样点总数：{total_count} | 参与评分：{scored_count}"
            f"（正形 {positive_count} / 负形 {negative_count}） | "
            f"近形 {near_shape_count} | "
            f"忽略：{result.ignored_count} | 无效：{result.invalid_count} | "
            f"步长 正/负 {sampling_config.positive_step_px}/"
            f"{sampling_config.negative_step_px} px"
        )
        self.draw_sampling_result(result)

        try:
            metrics = calculate_matching_metrics(
                self.undistorted,
                outline,
                result,
                target_kind=target_kind,
                center=self.ideal_centers[target],
                binary_threshold=self.safe_int(self.threshold_var, 50, 0, 255),
                negative_guard_px=sampling_config.negative_guard_px,
                near_negative_width_px=self.current_near_negative_width(),
            )
            decision = evaluate_matching_metrics(
                metrics,
                self.current_matching_thresholds(target_kind),
                target_kind=target_kind,
            )
        except ValueError as exc:
            self.sampling_summary_var.set(f"识别参数错误：{exc}")
            return

        def metric_text(value: float | None, digits: int = 2) -> str:
            return f"{value:.{digits}f}" if value is not None else "--"

        if decision.passed:
            decision_text = "试判：通过"
        else:
            shown_failures = "；".join(decision.failures[:2])
            remaining = len(decision.failures) - 2
            suffix = f"；另{remaining}项" if remaining > 0 else ""
            decision_text = f"试判：未通过（{shown_failures}{suffix}）"
        completeness_name = "双臂" if target_kind == TARGET_ARROW else "四象限"
        extras = [f"忽略{result.ignored_count}点"]
        if self.near_shape_variance_enabled_var.get():
            extras.append(
                f"近形 {metrics.near_shape_count}点/方差 "
                f"{metric_text(metrics.near_shape_variance, 1)}"
            )
        else:
            extras.append("近形方差未启用")
        if result.invalid_count:
            extras.append(f"无效区{result.invalid_count}点")
        if result.box_clipped:
            extras.append("采样框已裁边")
        self.sampling_summary_var.set(
            f"{decision_text}\n"
            f"灰差 {metric_text(metrics.gray_difference, 1)} | "
            f"分离度 {metric_text(metrics.standardized_separation)} | "
            f"固定一致 {metric_text(metrics.binary_consistency)}\n"
            f"正/负命中 {metric_text(metrics.positive_hit_rate)}/"
            f"{metric_text(metrics.negative_hit_rate)} | "
            f"{completeness_name} {metric_text(metrics.completeness)} | "
            f"近/远差 {metric_text(metrics.near_difference, 1)}/"
            f"{metric_text(metrics.far_difference, 1)}\n"
            f"正/负形 {metric_text(metrics.positive_mean, 1)}/"
            f"{metric_text(metrics.negative_mean, 1)} | "
            f"近/远负形 {metric_text(metrics.near_negative_mean, 1)}/"
            f"{metric_text(metrics.far_negative_mean, 1)} | "
            f"近形点/方差 {metrics.near_shape_count}/"
            f"{metric_text(metrics.near_shape_variance, 1)} | "
            f"P/N {metrics.positive_count}/{metrics.negative_count} | "
            + "，".join(extras)
        )

    def draw_sampling_result(self, result: SamplingResult) -> None:
        if result.bounds is not None:
            min_x, min_y, max_x, max_y = result.bounds
            self.canvas.create_rectangle(
                self.offset_x + min_x * self.zoom,
                self.offset_y + min_y * self.zoom,
                self.offset_x + max_x * self.zoom,
                self.offset_y + max_y * self.zoom,
                outline="#ffe066",
                width=1,
                dash=(4, 3),
                tags="ideal_sampling",
            )

        def draw_points(points: tuple[tuple[int, int], ...], color: str) -> None:
            stride = max(1, (len(points) + 799) // 800)
            radius = 2.0
            for image_x, image_y in points[::stride]:
                canvas_x = self.offset_x + image_x * self.zoom
                canvas_y = self.offset_y + image_y * self.zoom
                self.canvas.create_oval(
                    canvas_x - radius,
                    canvas_y - radius,
                    canvas_x + radius,
                    canvas_y + radius,
                    fill=color,
                    outline="",
                    tags="ideal_sampling",
                )

        draw_points(result.positive_points, "#28d96f")
        draw_points(result.negative_points, "#f04464")
        if self.near_shape_variance_enabled_var.get():
            draw_points(result.near_shape_points, "#d6a62f")

    def ideal_drag_start(self, event: tk.Event) -> None:
        target = self.ideal_target_var.get()
        if target == IDEAL_TARGET_NONE or self.view_var.get() not in IDEAL_BACKGROUND_NAMES:
            return
        image_x = (float(event.x) - self.offset_x) / self.zoom
        image_y = (float(event.y) - self.offset_y) / self.zoom
        if not (
            0.0 <= image_x < self.calibration.output_width
            and 0.0 <= image_y < self.calibration.output_height
        ):
            return
        self._ideal_drag_active = True
        self.ideal_centers[target] = (image_x, image_y)
        self.redraw_canvas()

    def ideal_drag_move(self, event: tk.Event) -> None:
        if not self._ideal_drag_active:
            return
        target = self.ideal_target_var.get()
        if target not in (IDEAL_TARGET_ARROW, IDEAL_TARGET_BEACON):
            return
        image_x = (float(event.x) - self.offset_x) / self.zoom
        image_y = (float(event.y) - self.offset_y) / self.zoom
        self.ideal_centers[target] = (
            max(0.0, min(self.calibration.output_width - 1.0, image_x)),
            max(0.0, min(self.calibration.output_height - 1.0, image_y)),
        )
        self.redraw_canvas()

    def ideal_drag_stop(self, _event: object = None) -> None:
        self._ideal_drag_active = False

    def fit_image(self) -> None:
        image = self.get_view_image()
        if image is None:
            return
        self.root.update_idletasks()
        canvas_width = max(100, self.canvas.winfo_width())
        canvas_height = max(100, self.canvas.winfo_height())
        self.zoom = max(0.1, min(12.0, 0.94 * min(canvas_width / image.width, canvas_height / image.height)))
        self.offset_x = (canvas_width - image.width * self.zoom) * 0.5
        self.offset_y = (canvas_height - image.height * self.zoom) * 0.5
        self.redraw_canvas()

    def canvas_wheel(self, event: tk.Event) -> None:
        self.zoom_at(event.x, event.y, 1.25 if event.delta > 0 else 0.8)

    def zoom_at(self, canvas_x: int, canvas_y: int, factor: float) -> None:
        if self.get_view_image() is None:
            return
        old_zoom = self.zoom
        new_zoom = max(0.1, min(12.0, old_zoom * factor))
        image_x = (canvas_x - self.offset_x) / old_zoom
        image_y = (canvas_y - self.offset_y) / old_zoom
        self.zoom = new_zoom
        self.offset_x = canvas_x - image_x * new_zoom
        self.offset_y = canvas_y - image_y * new_zoom
        self.redraw_canvas()

    def pan_start(self, event: tk.Event) -> None:
        self._pan_anchor = (event.x, event.y, self.offset_x, self.offset_y)

    def pan_move(self, event: tk.Event) -> None:
        if self._pan_anchor is None:
            return
        start_x, start_y, offset_x, offset_y = self._pan_anchor
        self.offset_x = offset_x + event.x - start_x
        self.offset_y = offset_y + event.y - start_y
        self.redraw_canvas()

    def canvas_motion(self, event: tk.Event) -> None:
        gray = self.get_gray_reference()
        if gray is None:
            return
        x = int((event.x - self.offset_x) / self.zoom)
        y = int((event.y - self.offset_y) / self.zoom)
        if x < 0 or y < 0 or x >= gray.width or y >= gray.height:
            self.pixel_var.set("鼠标位于图像外")
            return
        value = int(gray.getpixel((x, y)))
        binary_value = None
        if gray is self.undistorted and self.binary is not None:
            binary_value = 1 if self.binary.getpixel((x, y)) else 0
        binary_text = f"  二值={binary_value}" if binary_value is not None else ""
        self.pixel_var.set(f"坐标 ({x}, {y})\n灰度={value}{binary_text}\n缩放={self.zoom:.2f}x")
        self.update_magnifier(gray, x, y)

    def update_magnifier(self, gray: Image.Image, x: int, y: int) -> None:
        radius = 5
        crop = gray.crop((x - radius, y - radius, x + radius + 1, y + radius + 1))
        scale = 14
        enlarged = crop.resize((crop.width * scale, crop.height * scale), Image.Resampling.NEAREST).convert("RGB")
        draw = ImageDraw.Draw(enlarged)
        center = radius * scale
        draw.rectangle(
            (center, center, center + scale - 1, center + scale - 1),
            outline=(255, 60, 60),
            width=2,
        )
        self.magnifier_photo = ImageTk.PhotoImage(enlarged)
        self.magnifier_label.config(image=self.magnifier_photo)


def self_test() -> int:
    analysis_self_test()
    projection_self_test(CALIBRATION_HEADER)
    sampling_self_test()
    matching_self_test()
    contour_matching_self_test()
    calibration = CameraCalibration.from_header(CALIBRATION_HEADER)
    assert (calibration.image_width, calibration.image_height) == (188, 120)
    assert (calibration.output_width, calibration.output_height) == (376, 240)
    undistorter = FisheyeUndistorter(calibration)
    assert undistorter.is_valid_output_pixel(
        calibration.output_width // 2,
        calibration.output_height // 2,
    )
    assert not undistorter.is_valid_output_pixel(-1, 0)
    sample = Image.new("L", (calibration.image_width, calibration.image_height), 127)
    assert undistorter.apply(sample, interpolation=UNDISTORT_BILINEAR).size == (
        calibration.output_width,
        calibration.output_height,
    )
    assert undistorter.apply(sample, interpolation=UNDISTORT_NEAREST).size == (
        calibration.output_width,
        calibration.output_height,
    )
    assert DEFAULT_CAPTURE_DIR.is_dir()
    return 0


def main() -> int:
    if "--self-test" in sys.argv:
        return self_test()
    root = tk.Tk()
    try:
        PictureViewerApp(root)
    except (OSError, UnicodeError, ValueError) as exc:
        root.withdraw()
        messagebox.showerror("启动失败", str(exc))
        root.destroy()
        return 1
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


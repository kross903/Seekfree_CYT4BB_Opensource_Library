#!/usr/bin/env python3
"""Standalone viewer for inspecting captured MT9V03 grayscale frames."""

from __future__ import annotations

import json
from pathlib import Path
import sys
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from PIL import Image, ImageDraw, ImageTk

from image_analysis import (
    CameraCalibration,
    ConnectedComponent,
    FisheyeUndistorter,
    find_components,
    self_test as analysis_self_test,
    threshold_image,
)
from ideal_projection import (
    ProjectionCalibration,
    TargetPlaneProjector,
    build_ideal_beacon_points,
    build_ideal_chevron_points,
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


class PictureViewerApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("MT9V03 图像特征查看器")
        self.root.geometry("1440x860")
        self.root.minsize(1050, 650)

        self.calibration = CameraCalibration.from_header(CALIBRATION_HEADER)
        self.undistorter = FisheyeUndistorter(self.calibration)
        self.projection_calibration = ProjectionCalibration.from_header(CALIBRATION_HEADER)
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
        self.sampling_step_var = tk.StringVar(
            value=str(ideal_settings.get("sampling_step_px", "2"))
        )
        self.sampling_margin_var = tk.StringVar(
            value=str(ideal_settings.get("sampling_margin_px", "4"))
        )
        self.sampling_positive_band_var = tk.StringVar(
            value=str(ideal_settings.get("sampling_positive_band_px", "1.5"))
        )
        self.sampling_negative_guard_var = tk.StringVar(
            value=str(ideal_settings.get("sampling_negative_guard_px", "3"))
        )
        self.near_negative_width_var = tk.StringVar(
            value=str(matching_settings.get("near_negative_width_px", "2"))
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
        self.ideal_source_var = tk.StringVar(value="姿态：手动；小车Yaw：手动")
        self.ideal_status_var = tk.StringVar(value="选择目标后可在图像中左键拖动")
        self.sampling_summary_var = tk.StringVar(
            value="正形灰度 --（0点） | 负形灰度 --（0点）"
        )
        self._ideal_pose_source = "手动"
        self._ideal_car_yaw_source = "手动"
        self.sampling_window: tk.Toplevel | None = None
        self.matching_window: tk.Toplevel | None = None

        self.all_files: list[Path] = []
        self.visible_files: list[Path] = []
        self.current_path: Path | None = None
        self.original: Image.Image | None = None
        self.undistorted: Image.Image | None = None
        self.binary: Image.Image | None = None
        self.components: list[ConnectedComponent] = []
        self.selected_component: int | None = None
        self.current_telemetry: dict = {}

        self.canvas_photo: ImageTk.PhotoImage | None = None
        self.magnifier_photo: ImageTk.PhotoImage | None = None
        self.zoom = 1.0
        self.offset_x = 0.0
        self.offset_y = 0.0
        self._fit_pending = False
        self._pan_anchor: tuple[int, int, float, float] | None = None
        self._threshold_job: str | None = None
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
        ttk.Label(candidate_bar, text="连通域候选").pack(side=tk.LEFT)
        ttk.Button(candidate_bar, text="打开候选", command=self.open_selected_component).pack(side=tk.RIGHT)
        ttk.Button(
            candidate_bar,
            text="对齐理想目标",
            command=self.align_ideal_to_selected_component,
        ).pack(side=tk.RIGHT, padx=(0, 6))

        columns = ("area", "center", "box", "circle", "fill")
        self.component_tree = ttk.Treeview(
            candidate_tab, columns=columns, show="tree headings", height=12
        )
        self.component_tree.heading("#0", text="#")
        self.component_tree.column("#0", width=36, stretch=False, anchor="center")
        headings = {
            "area": ("面积", 55),
            "center": ("质心", 82),
            "box": ("外接框", 85),
            "circle": ("圆度", 55),
            "fill": ("占空", 55),
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
        for target_kind in MATCHING_TARGET_KINDS:
            try:
                thresholds = self.current_matching_thresholds(target_kind)
            except ValueError:
                thresholds = self.matching_defaults[target_kind]
            matching_settings[target_kind] = thresholds.to_mapping()
        settings = {
            "folder": self.folder_var.get(),
            "threshold": self.safe_int(self.threshold_var, 50, 0, 255),
            "min_area": self.safe_int(self.min_area_var, 6, 1, 90240),
            "connectivity": 8 if self.connectivity_var.get() == 8 else 4,
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
                "sampling_step_px": self.safe_int(
                    self.sampling_step_var, 2, 1, 32
                ),
                "sampling_margin_px": self.safe_float(
                    self.sampling_margin_var, 4.0
                ),
                "sampling_positive_band_px": self.safe_float(
                    self.sampling_positive_band_var, 1.5
                ),
                "sampling_negative_guard_px": self.safe_float(
                    self.sampling_negative_guard_var, 3.0
                ),
            },
            "matching_thresholds": matching_settings,
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

    def load_path(self, path: Path) -> None:
        if path == self.current_path:
            return
        try:
            self.status_var.set(f"正在加载并去畸变：{path.name}")
            self.root.update_idletasks()
            image = self.load_gray_image(path)
            undistorted = self.undistorter.apply(image)
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
        self.status_var.set(
            f"{path.name} | 原图 {image.width}x{image.height} | "
            f"去畸变 {undistorted.width}x{undistorted.height}"
        )

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
        self.redraw_canvas()

    def ideal_car_yaw_edited(self, _event: object = None) -> None:
        self._ideal_car_yaw_source = "手动"
        self.update_ideal_source_text()
        self.redraw_canvas()

    def ideal_parameter_changed(self, _event: object = None) -> None:
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
        content.columnconfigure(1, weight=1)
        fields = (
            ("采样步长 (px)", self.sampling_step_var),
            ("采样框余量 (px)", self.sampling_margin_var),
            ("正形轮廓带 (px)", self.sampling_positive_band_var),
            ("负形最小距离 (px)", self.sampling_negative_guard_var),
        )
        for row, (label, variable) in enumerate(fields):
            ttk.Label(content, text=label).grid(row=row, column=0, sticky="w", pady=3)
            if row == 0:
                entry = ttk.Spinbox(
                    content,
                    from_=1,
                    to=32,
                    width=9,
                    textvariable=variable,
                    command=self.sampling_parameter_changed,
                )
            else:
                entry = ttk.Entry(content, width=11, textvariable=variable)
            entry.grid(row=row, column=1, sticky="ew", padx=(8, 0), pady=3)
            entry.bind("<KeyRelease>", self.sampling_parameter_changed)

        legend = ttk.Frame(content)
        legend.grid(row=4, column=0, columnspan=2, sticky="ew", pady=(6, 0))
        tk.Label(legend, text="● 正形点", fg="#168a45").pack(side=tk.LEFT)
        tk.Label(legend, text="● 负形点", fg="#d62f4b").pack(side=tk.LEFT, padx=(12, 0))
        ttk.Label(
            content,
            textvariable=self.sampling_summary_var,
            justify=tk.LEFT,
            wraplength=280,
        ).grid(row=5, column=0, columnspan=2, sticky="w", pady=(7, 0))
        actions = ttk.Frame(content)
        actions.grid(row=6, column=0, columnspan=2, sticky="e", pady=(8, 0))
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
        self.sampling_step_var.set("2")
        self.sampling_margin_var.set("4")
        self.sampling_positive_band_var.set("1.5")
        self.sampling_negative_guard_var.set("3")
        self.redraw_canvas()

    def sampling_parameter_changed(self, _event: object = None) -> None:
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
                ).grid(row=row, column=0, sticky="w", pady=2)
                ttk.Label(tab, text=label).grid(
                    row=row, column=1, sticky="w", padx=(2, 16), pady=2
                )
                entry = ttk.Entry(
                    tab,
                    width=10,
                    textvariable=self.matching_value_vars[target_kind][key],
                )
                entry.grid(row=row, column=2, sticky="e", pady=2)
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
        for target_kind in MATCHING_TARGET_KINDS:
            config = self.matching_defaults[target_kind]
            for key, attribute in THRESHOLD_ATTRIBUTES.items():
                self.matching_value_vars[target_kind][key].set(
                    str(getattr(config, attribute))
                )
                self.matching_enabled_vars[target_kind][key].set(
                    key in config.enabled
                )
        self.redraw_canvas()

    def matching_parameter_changed(self, _event: object = None) -> None:
        self.redraw_canvas()

    def ideal_mount_angles_changed(self, _event: object = None) -> None:
        self.redraw_canvas()

    def reset_mount_angles(self) -> None:
        self.mount_roll_var.set("0")
        self.mount_pitch_var.set("0")
        self.mount_yaw_var.set("0")
        self.target_projector.set_mount_angles(0.0, 0.0, 0.0)
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

    def threshold_scale_changed(self, value: str) -> None:
        self.threshold_var.set(int(round(float(value))))

    def threshold_entry_changed(self) -> None:
        value = self.safe_int(self.threshold_var, 50, 0, 255)
        if abs(self.threshold_scale.get() - value) >= 0.5:
            self.threshold_scale.set(value)
        self.schedule_analysis()

    def schedule_analysis(self) -> None:
        if self._threshold_job is not None:
            self.root.after_cancel(self._threshold_job)
        self._threshold_job = self.root.after(80, self.run_analysis)

    def run_analysis(self) -> None:
        self._threshold_job = None
        if self.undistorted is None:
            return
        threshold = self.safe_int(self.threshold_var, 50, 0, 255)
        min_area = self.safe_int(self.min_area_var, 6, 1, self.undistorted.width * self.undistorted.height)
        self.binary = threshold_image(self.undistorted, threshold)
        self.components = find_components(
            self.binary,
            min_area=min_area,
            eight_connected=self.connectivity_var.get() == 8,
        )
        self.selected_component = None
        self.update_component_tree()
        self.redraw_canvas()
        foreground = sum(1 for value in self.binary.tobytes() if value)
        ratio = 100.0 * foreground / (self.binary.width * self.binary.height)
        self.status_var.set(
            f"阈值 {threshold} | 前景 {foreground} ({ratio:.2f}%) | "
            f"连通域 {len(self.components)} | 最小面积 {min_area}"
        )

    def update_component_tree(self) -> None:
        self.component_tree.delete(*self.component_tree.get_children())
        for component in self.components:
            self.component_tree.insert(
                "",
                tk.END,
                iid=str(component.label),
                text=str(component.label),
                values=(
                    component.area,
                    f"{component.centroid_x:.1f},{component.centroid_y:.1f}",
                    f"{component.width}x{component.height}",
                    f"{component.circularity:.3f}",
                    f"{component.occupancy:.2f}",
                ),
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
        gray_display = gray_crop.resize(display_size, Image.Resampling.NEAREST)
        binary_display = binary_crop.resize(display_size, Image.Resampling.NEAREST)

        window = tk.Toplevel(self.root)
        window.title(f"候选 #{component.label}")
        window.resizable(True, True)
        content = ttk.Frame(window, padding=8)
        content.pack(fill=tk.BOTH, expand=True)
        gray_photo = ImageTk.PhotoImage(gray_display)
        binary_photo = ImageTk.PhotoImage(binary_display)
        ttk.Label(content, text="去畸变灰度图").grid(row=0, column=0, padx=4)
        ttk.Label(content, text="单阈值二值图").grid(row=0, column=1, padx=4)
        ttk.Label(content, image=gray_photo).grid(row=1, column=0, padx=4, pady=4)
        ttk.Label(content, image=binary_photo).grid(row=1, column=1, padx=4, pady=4)
        details = (
            f"面积 {component.area} px    质心 ({component.centroid_x:.2f}, {component.centroid_y:.2f})\n"
            f"外接框 ({component.min_x}, {component.min_y}) - ({component.max_x}, {component.max_y})    "
            f"尺寸 {component.width}x{component.height}\n"
            f"边界像素 {component.boundary_pixels}    圆度 {component.circularity:.4f}    "
            f"占空比 {component.occupancy:.4f}"
        )
        ttk.Label(content, text=details, justify=tk.LEFT).grid(
            row=2, column=0, columnspan=2, sticky="w", padx=4, pady=(5, 0)
        )
        window.gray_photo = gray_photo  # type: ignore[attr-defined]
        window.binary_photo = binary_photo  # type: ignore[attr-defined]

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
            overlay = self.undistorted.convert("RGB")
            draw = ImageDraw.Draw(overlay)
            for component in self.components:
                selected = component.label == self.selected_component
                color = (255, 210, 0) if selected else (0, 255, 80)
                width = 2 if selected else 1
                draw.rectangle(
                    (component.min_x, component.min_y, component.max_x, component.max_y),
                    outline=color,
                    width=width,
                )
                draw.text((component.min_x + 1, component.min_y + 1), str(component.label), fill=color)
                radius = 2 if selected else 1
                draw.ellipse(
                    (
                        component.centroid_x - radius,
                        component.centroid_y - radius,
                        component.centroid_x + radius,
                        component.centroid_y + radius,
                    ),
                    outline=(255, 70, 70),
                )
            return overlay
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
        self.draw_ideal_target_overlay(image)

    def draw_ideal_target_overlay(self, background: Image.Image) -> None:
        target = self.ideal_target_var.get()
        if target == IDEAL_TARGET_NONE:
            self.ideal_status_var.set("选择目标后可在图像中左键拖动")
            self.sampling_summary_var.set("正形灰度 --（0点） | 负形灰度 --（0点）")
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

    def current_sampling_config(self) -> SamplingConfig:
        try:
            step = int(self.sampling_step_var.get())
            margin = float(self.sampling_margin_var.get())
            positive_band = float(self.sampling_positive_band_var.get())
            negative_guard = float(self.sampling_negative_guard_var.get())
        except (TypeError, ValueError, tk.TclError) as exc:
            raise ValueError("采样参数必须是数字") from exc
        return SamplingConfig(
            step_px=step,
            margin_px=margin,
            positive_band_px=positive_band,
            negative_guard_px=negative_guard,
        )

    def update_sampling_overlay(
        self,
        target: str,
        outline: list[tuple[float, float]],
    ) -> None:
        if self.undistorted is None:
            self.sampling_summary_var.set("尚未生成去畸变灰度图")
            return
        try:
            sampling_config = self.current_sampling_config()
            result = sample_projected_shape(
                self.undistorted,
                outline,
                closed=target == IDEAL_TARGET_BEACON,
                filled_positive=target == IDEAL_TARGET_BEACON,
                config=sampling_config,
                is_valid_pixel=self.undistorter.is_valid_output_pixel,
            )
        except ValueError as exc:
            self.sampling_summary_var.set(f"采样参数错误：{exc}")
            return
        self.draw_sampling_result(result)

        target_kind = TARGET_ARROW if target == IDEAL_TARGET_ARROW else TARGET_BEACON
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
    calibration = CameraCalibration.from_header(CALIBRATION_HEADER)
    assert (calibration.image_width, calibration.image_height) == (188, 120)
    assert (calibration.output_width, calibration.output_height) == (376, 240)
    undistorter = FisheyeUndistorter(calibration)
    assert undistorter.is_valid_output_pixel(
        calibration.output_width // 2,
        calibration.output_height // 2,
    )
    assert not undistorter.is_valid_output_pixel(-1, 0)
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


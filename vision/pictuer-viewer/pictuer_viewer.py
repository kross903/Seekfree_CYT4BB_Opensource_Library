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


APP_DIR = Path(__file__).resolve().parent
VISION_DIR = APP_DIR.parent
REPO_ROOT = VISION_DIR.parent
DEFAULT_CAPTURE_DIR = VISION_DIR / "capture"
CALIBRATION_HEADER = REPO_ROOT / "project" / "code" / "vision" / "camera_calibration_params.h"
SETTINGS_PATH = APP_DIR / "viewer_settings.json"
SUPPORTED_SUFFIXES = {".png", ".pgm", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff", ".raw"}
VIEW_NAMES = ("原始灰度图", "去畸变灰度图", "单阈值二值图", "连通域候选")


class PictureViewerApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("MT9V03 图像特征查看器")
        self.root.geometry("1440x860")
        self.root.minsize(1050, 650)

        self.calibration = CameraCalibration.from_header(CALIBRATION_HEADER)
        self.undistorter = FisheyeUndistorter(self.calibration)
        settings = self._load_settings()

        self.folder_var = tk.StringVar(
            value=str(Path(settings.get("folder", DEFAULT_CAPTURE_DIR)))
        )
        self.filter_var = tk.StringVar()
        self.view_var = tk.StringVar(value="去畸变灰度图")
        self.threshold_var = tk.IntVar(value=int(settings.get("threshold", 50)))
        self.min_area_var = tk.IntVar(value=int(settings.get("min_area", 6)))
        self.connectivity_var = tk.IntVar(value=int(settings.get("connectivity", 8)))
        self.status_var = tk.StringVar(value="就绪")
        self.pixel_var = tk.StringVar(value="移动鼠标查看坐标和灰度")
        self.metadata_var = tk.StringVar(value="尚未加载图像")

        self.all_files: list[Path] = []
        self.visible_files: list[Path] = []
        self.current_path: Path | None = None
        self.original: Image.Image | None = None
        self.undistorted: Image.Image | None = None
        self.binary: Image.Image | None = None
        self.components: list[ConnectedComponent] = []
        self.selected_component: int | None = None

        self.canvas_photo: ImageTk.PhotoImage | None = None
        self.magnifier_photo: ImageTk.PhotoImage | None = None
        self.zoom = 1.0
        self.offset_x = 0.0
        self.offset_y = 0.0
        self._fit_pending = False
        self._pan_anchor: tuple[int, int, float, float] | None = None
        self._threshold_job: str | None = None

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
        side.rowconfigure(4, weight=1)

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

        candidate_bar = ttk.Frame(side)
        candidate_bar.grid(row=3, column=0, sticky="ew", pady=(8, 4))
        ttk.Label(candidate_bar, text="连通域候选").pack(side=tk.LEFT)
        ttk.Button(candidate_bar, text="打开候选", command=self.open_selected_component).pack(side=tk.RIGHT)

        columns = ("area", "center", "box", "circle", "fill")
        self.component_tree = ttk.Treeview(side, columns=columns, show="tree headings", height=12)
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
        component_scroll = ttk.Scrollbar(side, orient=tk.VERTICAL, command=self.component_tree.yview)
        self.component_tree.configure(yscrollcommand=component_scroll.set)
        self.component_tree.grid(row=4, column=0, sticky="nsew")
        component_scroll.grid(row=4, column=1, sticky="ns")

        status = ttk.Label(self.root, textvariable=self.status_var, relief=tk.SUNKEN, anchor="w", padding=(6, 3))
        status.grid(row=2, column=0, columnspan=3, sticky="ew")

    def _bind_events(self) -> None:
        self.file_list.bind("<<ListboxSelect>>", self.file_selected)
        self.filter_var.trace_add("write", lambda *_: self.apply_filter())
        self.view_var.trace_add("write", lambda *_: self.redraw_canvas())
        self.threshold_var.trace_add("write", lambda *_: self.threshold_entry_changed())
        self.min_area_var.trace_add("write", lambda *_: self.schedule_analysis())
        self.canvas.bind("<Configure>", self.canvas_configured)
        self.canvas.bind("<Motion>", self.canvas_motion)
        self.canvas.bind("<MouseWheel>", self.canvas_wheel)
        self.canvas.bind("<Button-4>", lambda event: self.zoom_at(event.x, event.y, 1.25))
        self.canvas.bind("<Button-5>", lambda event: self.zoom_at(event.x, event.y, 0.8))
        self.canvas.bind("<ButtonPress-3>", self.pan_start)
        self.canvas.bind("<B3-Motion>", self.pan_move)
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
        settings = {
            "folder": self.folder_var.get(),
            "threshold": self.safe_int(self.threshold_var, 50, 0, 255),
            "min_area": self.safe_int(self.min_area_var, 6, 1, 90240),
            "connectivity": 8 if self.connectivity_var.get() == 8 else 4,
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
        try:
            metadata = json.loads(metadata_path.read_text(encoding="utf-8-sig"))
            telemetry = metadata.get("telemetry", {})
            if telemetry:
                lines.extend(
                    (
                        f"Roll {telemetry.get('roll_deg', 0.0):.2f}°  "
                        f"Pitch {telemetry.get('pitch_deg', 0.0):.2f}°",
                        f"Yaw {telemetry.get('yaw_deg', 0.0):.2f}°  "
                        f"ToF {telemetry.get('tof_height_cm', 0.0):.2f} cm",
                    )
                )
        except (OSError, UnicodeError, json.JSONDecodeError, TypeError, ValueError):
            lines.append("无有效遥测元数据")
        self.metadata_var.set("\n".join(lines))

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
    calibration = CameraCalibration.from_header(CALIBRATION_HEADER)
    assert (calibration.image_width, calibration.image_height) == (188, 120)
    assert (calibration.output_width, calibration.output_height) == (376, 240)
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


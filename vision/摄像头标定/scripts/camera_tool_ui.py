#!/usr/bin/env python3
"""Desktop UI for camera frame capture, calibration, and header generation."""

from __future__ import annotations

import queue
import threading
import traceback
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, messagebox, scrolledtext, ttk

from PIL import Image, ImageTk

from calibrate_fisheye import (
    load_calibration,
    read_grayscale_image,
    run_calibration,
    undistort_fisheye_image,
)
from capture_seekfree_frames import capture_frames, list_serial_ports
from generate_firmware_headers import generate_headers


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_FRAMES_DIR = PROJECT_ROOT / "frames" / "uart_capture"
DEFAULT_OUTPUT_DIR = PROJECT_ROOT / "output"


class CameraToolApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("摄像头标定工具")
        self.root.geometry("1180x760")

        self.capture_stop_event = threading.Event()
        self.capture_running = False
        self.preview_image = None
        self.ui_queue: queue.Queue[tuple[str, object]] = queue.Queue()

        self.port_var = tk.StringVar()
        self.baud_var = tk.StringVar(value="115200")
        self.count_var = tk.StringVar(value="50")
        self.capture_out_var = tk.StringVar(value=str(DEFAULT_FRAMES_DIR))
        self.prefix_var = tk.StringVar(value="frame")
        self.timeout_var = tk.StringVar(value="1.0")

        self.capture_status_var = tk.StringVar(value="未开始")
        self.frame_info_var = tk.StringVar(value="暂无帧")

        self.frames_dir_var = tk.StringVar(value=str(DEFAULT_FRAMES_DIR))
        self.calib_output_var = tk.StringVar(value=str(DEFAULT_OUTPUT_DIR))
        self.pattern_cols_var = tk.StringVar(value="11")
        self.pattern_rows_var = tk.StringVar(value="8")
        self.square_size_var = tk.StringVar(value="2.0")
        self.valid_radius_var = tk.StringVar(value="0.0")
        self.calib_summary_var = tk.StringVar(value="未执行")

        self.header_calibration_var = tk.StringVar(value=str(DEFAULT_OUTPUT_DIR / "camera_calibration.json"))
        self.header_output_var = tk.StringVar(value=str(DEFAULT_OUTPUT_DIR))
        self.header_q_var = tk.StringVar(value="14")
        self.header_summary_var = tk.StringVar(value="未执行")

        self.build_layout()
        self.refresh_ports()
        self.root.after(100, self.process_ui_queue)

    def build_layout(self) -> None:
        root_frame = ttk.Frame(self.root, padding=10)
        root_frame.pack(fill=tk.BOTH, expand=True)
        root_frame.columnconfigure(0, weight=1)
        root_frame.columnconfigure(1, weight=1)
        root_frame.rowconfigure(0, weight=1)
        root_frame.rowconfigure(1, weight=1)

        capture_frame = ttk.LabelFrame(root_frame, text="1. 串口抓图", padding=10)
        capture_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 6), pady=(0, 6))
        capture_frame.columnconfigure(1, weight=1)
        capture_frame.columnconfigure(3, weight=1)
        capture_frame.rowconfigure(7, weight=1)
        self.build_capture_panel(capture_frame)

        preview_frame = ttk.LabelFrame(root_frame, text="最新预览", padding=10)
        preview_frame.grid(row=0, column=1, sticky="nsew", padx=(6, 0), pady=(0, 6))
        preview_frame.columnconfigure(0, weight=1)
        preview_frame.rowconfigure(0, weight=1)
        self.build_preview_panel(preview_frame)

        calib_frame = ttk.LabelFrame(root_frame, text="2. 标定", padding=10)
        calib_frame.grid(row=1, column=0, sticky="nsew", padx=(0, 6), pady=(6, 0))
        calib_frame.columnconfigure(1, weight=1)
        self.build_calibration_panel(calib_frame)

        header_frame = ttk.LabelFrame(root_frame, text="3. 生成固件头文件", padding=10)
        header_frame.grid(row=1, column=1, sticky="nsew", padx=(6, 0), pady=(6, 0))
        header_frame.columnconfigure(1, weight=1)
        self.build_header_panel(header_frame)

    def build_capture_panel(self, parent: ttk.LabelFrame) -> None:
        ttk.Label(parent, text="串口").grid(row=0, column=0, sticky="w")
        self.port_combo = ttk.Combobox(parent, textvariable=self.port_var)
        self.port_combo.grid(row=0, column=1, sticky="ew", padx=(8, 8))
        ttk.Button(parent, text="刷新串口", command=self.refresh_ports).grid(row=0, column=2, sticky="ew")

        ttk.Label(parent, text="波特率").grid(row=1, column=0, sticky="w", pady=(8, 0))
        ttk.Entry(parent, textvariable=self.baud_var).grid(row=1, column=1, sticky="ew", padx=(8, 8), pady=(8, 0))

        ttk.Label(parent, text="抓图数量").grid(row=1, column=2, sticky="w", pady=(8, 0))
        ttk.Entry(parent, textvariable=self.count_var).grid(row=1, column=3, sticky="ew", pady=(8, 0))

        ttk.Label(parent, text="输出目录").grid(row=2, column=0, sticky="w", pady=(8, 0))
        ttk.Entry(parent, textvariable=self.capture_out_var).grid(row=2, column=1, columnspan=2, sticky="ew", padx=(8, 8), pady=(8, 0))
        ttk.Button(parent, text="选择", command=lambda: self.choose_directory(self.capture_out_var)).grid(row=2, column=3, sticky="ew", pady=(8, 0))

        ttk.Label(parent, text="文件前缀").grid(row=3, column=0, sticky="w", pady=(8, 0))
        ttk.Entry(parent, textvariable=self.prefix_var).grid(row=3, column=1, sticky="ew", padx=(8, 8), pady=(8, 0))

        ttk.Label(parent, text="超时(s)").grid(row=3, column=2, sticky="w", pady=(8, 0))
        ttk.Entry(parent, textvariable=self.timeout_var).grid(row=3, column=3, sticky="ew", pady=(8, 0))

        button_bar = ttk.Frame(parent)
        button_bar.grid(row=4, column=0, columnspan=4, sticky="ew", pady=(12, 0))
        button_bar.columnconfigure(0, weight=1)
        button_bar.columnconfigure(1, weight=1)
        self.start_capture_button = ttk.Button(button_bar, text="开始抓图", command=self.start_capture)
        self.start_capture_button.grid(row=0, column=0, sticky="ew", padx=(0, 6))
        self.stop_capture_button = ttk.Button(button_bar, text="停止抓图", command=self.stop_capture, state=tk.DISABLED)
        self.stop_capture_button.grid(row=0, column=1, sticky="ew", padx=(6, 0))

        ttk.Label(parent, text="状态").grid(row=5, column=0, sticky="w", pady=(12, 0))
        ttk.Label(parent, textvariable=self.capture_status_var).grid(row=5, column=1, columnspan=3, sticky="w", padx=(8, 0), pady=(12, 0))

        ttk.Label(parent, text="帧信息").grid(row=6, column=0, sticky="w", pady=(8, 0))
        ttk.Label(parent, textvariable=self.frame_info_var).grid(row=6, column=1, columnspan=3, sticky="w", padx=(8, 0), pady=(8, 0))

        ttk.Label(parent, text="日志").grid(row=7, column=0, sticky="nw", pady=(8, 0))
        self.capture_log = scrolledtext.ScrolledText(parent, height=14, wrap=tk.WORD)
        self.capture_log.grid(row=7, column=1, columnspan=3, sticky="nsew", padx=(8, 0), pady=(8, 0))

    def build_preview_panel(self, parent: ttk.LabelFrame) -> None:
        self.preview_label = ttk.Label(parent, text="暂无预览", anchor="center")
        self.preview_label.grid(row=0, column=0, sticky="nsew")

    def build_calibration_panel(self, parent: ttk.LabelFrame) -> None:
        ttk.Label(parent, text="图像目录").grid(row=0, column=0, sticky="w")
        ttk.Entry(parent, textvariable=self.frames_dir_var).grid(row=0, column=1, sticky="ew", padx=(8, 8))
        ttk.Button(parent, text="选择", command=lambda: self.choose_directory(self.frames_dir_var)).grid(row=0, column=2, sticky="ew")

        ttk.Label(parent, text="输出目录").grid(row=1, column=0, sticky="w", pady=(8, 0))
        ttk.Entry(parent, textvariable=self.calib_output_var).grid(row=1, column=1, sticky="ew", padx=(8, 8), pady=(8, 0))
        ttk.Button(parent, text="选择", command=lambda: self.choose_directory(self.calib_output_var)).grid(row=1, column=2, sticky="ew", pady=(8, 0))

        ttk.Label(parent, text="内角点列数").grid(row=2, column=0, sticky="w", pady=(8, 0))
        ttk.Entry(parent, textvariable=self.pattern_cols_var).grid(row=2, column=1, sticky="ew", padx=(8, 8), pady=(8, 0))

        ttk.Label(parent, text="内角点行数").grid(row=3, column=0, sticky="w", pady=(8, 0))
        ttk.Entry(parent, textvariable=self.pattern_rows_var).grid(row=3, column=1, sticky="ew", padx=(8, 8), pady=(8, 0))

        ttk.Label(parent, text="方格边长(cm)").grid(row=4, column=0, sticky="w", pady=(8, 0))
        ttk.Entry(parent, textvariable=self.square_size_var).grid(row=4, column=1, sticky="ew", padx=(8, 8), pady=(8, 0))

        ttk.Label(parent, text="有效半径(px)").grid(row=5, column=0, sticky="w", pady=(8, 0))
        ttk.Entry(parent, textvariable=self.valid_radius_var).grid(row=5, column=1, sticky="ew", padx=(8, 8), pady=(8, 0))

        button_bar = ttk.Frame(parent)
        button_bar.grid(row=6, column=0, columnspan=3, sticky="ew", pady=(12, 0))
        button_bar.columnconfigure(0, weight=1)
        button_bar.columnconfigure(1, weight=1)
        ttk.Button(button_bar, text="开始标定", command=self.start_calibration).grid(
            row=0, column=0, sticky="ew", padx=(0, 6)
        )
        ttk.Button(button_bar, text="预览去畸变", command=self.preview_undistortion).grid(
            row=0, column=1, sticky="ew", padx=(6, 0)
        )

        ttk.Label(parent, text="结果").grid(row=7, column=0, sticky="nw", pady=(12, 0))
        ttk.Label(parent, textvariable=self.calib_summary_var, justify=tk.LEFT).grid(row=7, column=1, columnspan=2, sticky="w", padx=(8, 0), pady=(12, 0))

    def build_header_panel(self, parent: ttk.LabelFrame) -> None:
        ttk.Label(parent, text="标定 JSON").grid(row=0, column=0, sticky="w")
        ttk.Entry(parent, textvariable=self.header_calibration_var).grid(row=0, column=1, sticky="ew", padx=(8, 8))
        ttk.Button(parent, text="选择", command=lambda: self.choose_file(self.header_calibration_var, [("JSON", "*.json"), ("All files", "*.*")])).grid(row=0, column=2, sticky="ew")

        ttk.Label(parent, text="输出目录").grid(row=1, column=0, sticky="w", pady=(8, 0))
        ttk.Entry(parent, textvariable=self.header_output_var).grid(row=1, column=1, sticky="ew", padx=(8, 8), pady=(8, 0))
        ttk.Button(parent, text="选择", command=lambda: self.choose_directory(self.header_output_var)).grid(row=1, column=2, sticky="ew", pady=(8, 0))

        ttk.Label(parent, text="Q 值").grid(row=2, column=0, sticky="w", pady=(8, 0))
        ttk.Entry(parent, textvariable=self.header_q_var).grid(row=2, column=1, sticky="ew", padx=(8, 8), pady=(8, 0))

        ttk.Button(parent, text="生成头文件", command=self.start_generate_headers).grid(row=3, column=0, columnspan=3, sticky="ew", pady=(12, 0))

        ttk.Label(parent, text="结果").grid(row=4, column=0, sticky="nw", pady=(12, 0))
        ttk.Label(parent, textvariable=self.header_summary_var, justify=tk.LEFT).grid(row=4, column=1, columnspan=2, sticky="w", padx=(8, 0), pady=(12, 0))

    def choose_directory(self, variable: tk.StringVar) -> None:
        selected = filedialog.askdirectory(initialdir=variable.get() or str(PROJECT_ROOT))
        if selected:
            variable.set(selected)

    def choose_file(self, variable: tk.StringVar, filetypes: list[tuple[str, str]]) -> None:
        selected = filedialog.askopenfilename(initialdir=str(PROJECT_ROOT), filetypes=filetypes)
        if selected:
            variable.set(selected)

    def refresh_ports(self) -> None:
        ports = list_serial_ports()
        self.port_combo["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])
        if not ports:
            self.port_var.set("")
        self.log_capture(f"串口列表: {ports or '未发现'}")

    def log_capture(self, message: str) -> None:
        self.capture_log.insert(tk.END, message + "\n")
        self.capture_log.see(tk.END)

    def set_capture_running(self, running: bool) -> None:
        self.capture_running = running
        self.start_capture_button.config(state=tk.DISABLED if running else tk.NORMAL)
        self.stop_capture_button.config(state=tk.NORMAL if running else tk.DISABLED)

    def start_capture(self) -> None:
        if self.capture_running:
            return

        if not self.port_var.get():
            messagebox.showerror("参数错误", "未选择串口。")
            return

        try:
            baud = int(self.baud_var.get())
            count = int(self.count_var.get())
            timeout = float(self.timeout_var.get())
        except ValueError:
            messagebox.showerror("参数错误", "波特率、抓图数量、超时必须是数字。")
            return

        self.capture_stop_event.clear()
        self.set_capture_running(True)
        self.capture_status_var.set("抓图中")
        self.log_capture("开始抓图")

        worker = threading.Thread(
            target=self.capture_worker,
            args=(
                self.port_var.get(),
                baud,
                count,
                Path(self.capture_out_var.get()),
                self.prefix_var.get(),
                timeout,
            ),
            daemon=True,
        )
        worker.start()

    def stop_capture(self) -> None:
        if not self.capture_running:
            return
        self.capture_stop_event.set()
        self.capture_status_var.set("正在停止")
        self.log_capture("已请求停止抓图")

    def capture_worker(self, port_name: str, baud: int, count: int, out_dir: Path, prefix: str, timeout: float) -> None:
        try:
            frames = capture_frames(
                port_name=port_name,
                baud=baud,
                count=count,
                out=out_dir,
                prefix=prefix,
                timeout=timeout,
                on_log=lambda msg: self.ui_queue.put(("capture_log", msg)),
                on_frame=lambda frame: self.ui_queue.put(("frame", frame)),
                should_stop=self.capture_stop_event.is_set,
            )
            self.ui_queue.put(("capture_done", frames))
        except BaseException as exc:
            detail = "".join(traceback.format_exception_only(type(exc), exc)).strip()
            self.ui_queue.put(("capture_error", detail))

    def start_calibration(self) -> None:
        try:
            pattern_cols = int(self.pattern_cols_var.get())
            pattern_rows = int(self.pattern_rows_var.get())
            square_size = float(self.square_size_var.get())
            valid_radius = float(self.valid_radius_var.get())
        except ValueError:
            messagebox.showerror("参数错误", "标定参数必须是数字。")
            return

        self.calib_summary_var.set("标定中")
        worker = threading.Thread(
            target=self.calibration_worker,
            args=(
                Path(self.frames_dir_var.get()),
                Path(self.calib_output_var.get()),
                pattern_cols,
                pattern_rows,
                square_size,
                valid_radius,
            ),
            daemon=True,
        )
        worker.start()

    def preview_undistortion(self) -> None:
        image_path = filedialog.askopenfilename(
            initialdir=self.frames_dir_var.get() or str(DEFAULT_FRAMES_DIR),
            filetypes=[
                ("图像", "*.png *.jpg *.jpeg *.bmp *.pgm"),
                ("所有文件", "*.*"),
            ],
        )
        if not image_path:
            return

        calibration_path = Path(self.header_calibration_var.get())
        try:
            source = read_grayscale_image(Path(image_path))
            if source is None:
                raise ValueError(f"无法读取图像: {image_path}")
            calibration = load_calibration(calibration_path)
            undistorted = undistort_fisheye_image(source, calibration)
        except (OSError, RuntimeError, TypeError, ValueError) as exc:
            messagebox.showerror("去畸变预览失败", str(exc))
            return

        preview = tk.Toplevel(self.root)
        preview.title("去畸变预览")
        preview.geometry("1180x430")
        preview.columnconfigure(0, weight=1)
        preview.columnconfigure(1, weight=1)
        preview.rowconfigure(1, weight=1)

        ttk.Label(preview, text="原图", anchor="center").grid(
            row=0, column=0, sticky="ew", padx=8, pady=(8, 4)
        )
        ttk.Label(
            preview,
            text=f"去畸变 {undistorted.shape[1]}×{undistorted.shape[0]}（原焦距尺度）",
            anchor="center",
        ).grid(
            row=0, column=1, sticky="ew", padx=8, pady=(8, 4)
        )

        size = (564, 360)
        source_photo = ImageTk.PhotoImage(
            Image.fromarray(source).resize(size, Image.Resampling.NEAREST)
        )
        undistorted_photo = ImageTk.PhotoImage(
            Image.fromarray(undistorted).resize(size, Image.Resampling.NEAREST)
        )
        ttk.Label(preview, image=source_photo, anchor="center").grid(
            row=1, column=0, sticky="nsew", padx=8, pady=(4, 8)
        )
        ttk.Label(preview, image=undistorted_photo, anchor="center").grid(
            row=1, column=1, sticky="nsew", padx=8, pady=(4, 8)
        )
        preview.source_photo = source_photo
        preview.undistorted_photo = undistorted_photo

    def calibration_worker(
        self,
        frames_dir: Path,
        output_dir: Path,
        pattern_cols: int,
        pattern_rows: int,
        square_size: float,
        valid_radius: float,
    ) -> None:
        try:
            result = run_calibration(
                frames_dir=frames_dir,
                output_dir=output_dir,
                pattern_cols=pattern_cols,
                pattern_rows=pattern_rows,
                square_size_cm=square_size,
                valid_radius_px=valid_radius,
            )
            self.ui_queue.put(("calibration_done", result))
        except BaseException as exc:
            detail = "".join(traceback.format_exception_only(type(exc), exc)).strip()
            self.ui_queue.put(("calibration_error", detail))

    def start_generate_headers(self) -> None:
        try:
            q = int(self.header_q_var.get())
        except ValueError:
            messagebox.showerror("参数错误", "Q 值必须是整数。")
            return

        self.header_summary_var.set("生成中")
        worker = threading.Thread(
            target=self.header_worker,
            args=(Path(self.header_calibration_var.get()), Path(self.header_output_var.get()), q),
            daemon=True,
        )
        worker.start()

    def header_worker(self, calibration_path: Path, output_dir: Path, q: int) -> None:
        try:
            result = generate_headers(calibration_path, output_dir, q)
            self.ui_queue.put(("headers_done", result))
        except BaseException as exc:
            detail = "".join(traceback.format_exception_only(type(exc), exc)).strip()
            self.ui_queue.put(("headers_error", detail))

    def process_ui_queue(self) -> None:
        while True:
            try:
                event, payload = self.ui_queue.get_nowait()
            except queue.Empty:
                break

            if event == "capture_log":
                self.log_capture(str(payload))
            elif event == "frame":
                self.update_preview(payload)
            elif event == "capture_done":
                frames = payload
                self.set_capture_running(False)
                self.capture_status_var.set(f"完成，共保存 {len(frames)} 帧")
            elif event == "capture_error":
                self.set_capture_running(False)
                self.capture_status_var.set("失败")
                messagebox.showerror("抓图失败", str(payload))
            elif event == "calibration_done":
                result = payload
                self.header_calibration_var.set(str(result["json_path"]))
                self.calib_summary_var.set(
                    "成功\n"
                    f"RMS: {result['rms']:.6f}\n"
                    f"发现图像: {result['discovered_count']}\n"
                    f"读取成功: {result['readable_count']}\n"
                    f"使用图像: {result['used_count']}\n"
                    f"拒绝图像: {len(result['rejected_images'])}\n"
                    f"JSON: {result['json_path']}\n"
                    f"报告: {result['report_path']}\n"
                    f"角点诊断: {result['detection_report_path']}"
                )
            elif event == "calibration_error":
                self.calib_summary_var.set(f"失败\n{payload}")
                messagebox.showerror("标定失败", str(payload))
            elif event == "headers_done":
                result = payload
                self.header_summary_var.set(
                    "成功\n"
                    f"参数头文件: {result['params_path']}\n"
                    f"射线表: {result['ray_path']}\n"
                    f"去畸变表: {result['rect_path']}"
                )
            elif event == "headers_error":
                self.header_summary_var.set(f"失败\n{payload}")
                messagebox.showerror("生成失败", str(payload))

        self.root.after(100, self.process_ui_queue)

    def update_preview(self, frame: dict) -> None:
        self.frame_info_var.set(
            f"序号: {frame['index']}  尺寸: {frame['width']}x{frame['height']}  "
            f"类型: {frame['image_type']}  边界: {frame['boundary_num']}"
        )
        image_path = frame.get("image_path")
        if not image_path:
            self.preview_label.config(text="该帧未生成可预览图像", image="")
            return

        image = Image.open(image_path)
        image = image.resize((564, 360), Image.Resampling.NEAREST)
        self.preview_image = ImageTk.PhotoImage(image)
        self.preview_label.config(image=self.preview_image, text="")


def main() -> int:
    root = tk.Tk()
    CameraToolApp(root)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

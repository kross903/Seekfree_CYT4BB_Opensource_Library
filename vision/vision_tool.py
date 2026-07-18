#!/usr/bin/env python3
"""Desktop UI for running the firmware vision C code against saved camera frames."""

from __future__ import annotations

import json
import os
import queue
import shlex
import shutil
import subprocess
import sys
import threading
import time
from pathlib import Path
from tkinter import filedialog, messagebox, scrolledtext, ttk
import tkinter as tk

from PIL import Image, ImageDraw, ImageTk

from capture_serial import capture_frames, list_serial_ports


APP_ROOT = Path(__file__).resolve().parent
DEFAULT_SOURCE = Path(r"D:\code\project\Seekfree_CYT4BB_Opensource_Library\project\code\vision")
DEFAULT_FRAMES = Path(r"D:\code\project\Seekfree_CYT4BB_Opensource_Library\vision\capture")
SIM_ROOT = APP_ROOT / "sim"
BUILD_ROOT = APP_ROOT / "build"
RUN_ROOT = APP_ROOT / "runs"
WIDTH = 188
HEIGHT = 120


class Simulator:
    """Builds the selected firmware vision sources with the PC hardware shim."""

    def __init__(self, source_dir: Path, threshold: int = 150) -> None:
        self.source_dir = source_dir
        self.threshold = threshold

    def find_compiler(self) -> str | None:
        configured = os.environ.get("VISION_CC") or os.environ.get("CC")
        candidates = [configured] if configured else []
        candidates.extend(["gcc", "clang"])
        candidates.extend([
            r"C:\msys64\mingw64\bin\gcc.exe",
            r"C:\msys64\ucrt64\bin\gcc.exe",
            r"C:\msys64\clang64\bin\clang.exe",
            r"C:\msys64\usr\bin\gcc.exe",
        ])
        for candidate in candidates:
            if candidate:
                resolved = shutil.which(candidate) or (candidate if Path(candidate).is_file() else None)
                if resolved:
                    return resolved
        return None

    def source_files(self) -> list[Path]:
        names = [
            "vision_runtime.c",
            "beacon_detect.c",
            "vehicle_marker_localization.c",
            "ipm_interface.c",
            "drone_ipm_embedded.c",
            "camera_calibration.c",
        ]
        return [self.source_dir / name for name in names]

    def output_path(self) -> Path:
        return BUILD_ROOT / ("vision_sim.exe" if os.name == "nt" else "vision_sim")

    @staticmethod
    def _to_msys_path(value: str) -> str:
        if len(value) >= 3 and value[1] == ":" and value[2] in {"\\", "/"}:
            return "/" + value[0].lower() + value[2:].replace("\\", "/")
        return value

    def manifest_path(self) -> Path:
        return BUILD_ROOT / "vision_sim_source.txt"

    def build_inputs(self) -> list[Path]:
        inputs = list(self.source_dir.glob("*.c")) + list(self.source_dir.glob("*.h"))
        inputs.extend(path for path in (SIM_ROOT / "include").rglob("*") if path.is_file())
        inputs.append(SIM_ROOT / "vision_sim_main.c")
        return inputs

    def needs_rebuild(self) -> bool:
        executable = self.output_path()
        manifest = self.manifest_path()
        if not executable.exists() or not manifest.exists():
            return True
        if manifest.read_text(encoding="utf-8").strip() != str(self.source_dir.resolve()):
            return True
        executable_time = executable.stat().st_mtime
        return any(path.exists() and path.stat().st_mtime > executable_time for path in self.build_inputs())

    def build(self) -> tuple[bool, str]:
        compiler = self.find_compiler()
        if not compiler:
            return False, "No GCC/Clang compiler found. Install MinGW-w64 GCC or LLVM Clang, or set VISION_CC."
        missing = [str(path) for path in self.source_files() if not path.exists()]
        if missing:
            return False, "Vision source files are missing:\n" + "\n".join(missing)

        BUILD_ROOT.mkdir(parents=True, exist_ok=True)
        command = [
            compiler,
            "-std=c99",
            "-O2",
            "-Wall",
            "-Wextra",
            "-Wno-unused-parameter",
            "-DCY_CORE_CM7_1=1",
            "-DCY_CORE_CM7_0=0",
            "-I",
            str(SIM_ROOT / "include"),
            "-I",
            str(self.source_dir),
            "-I",
            str(self.source_dir.parent),
            *[str(path) for path in self.source_files()],
            str(SIM_ROOT / "vision_sim_main.c"),
            "-lm",
            "-o",
            str(self.output_path()),
        ]
        process_command = command
        compiler_lower = compiler.lower().replace("/", "\\")
        msys_root = Path(r"C:\msys64")
        if os.name == "nt" and "\\msys64\\" in compiler_lower and (msys_root / "usr" / "bin" / "bash.exe").exists():
            bash_command = ["gcc" if token == compiler else self._to_msys_path(token) for token in command]
            process_command = [
                str(msys_root / "usr" / "bin" / "bash.exe"),
                "-lc",
                "export PATH=/mingw64/bin:$PATH; " + shlex.join(bash_command),
            ]
        elif os.name == "nt" and Path(compiler).suffix.lower() == ".exe":
            process_command = ["cmd.exe", "/d", "/c", subprocess.list2cmdline(command)]
        result = subprocess.run(process_command, cwd=str(APP_ROOT), capture_output=True, text=True)
        log = "$ " + " ".join(process_command) + f"\n[returncode={result.returncode}]\n" + (result.stdout or "") + (result.stderr or "")
        if result.returncode != 0:
            return False, log
        self.manifest_path().write_text(str(self.source_dir.resolve()), encoding="utf-8", newline="\n")
        return True, log + "\nBuild succeeded: " + str(self.output_path())

    def run(
        self,
        raw_path: Path,
        output_dir: Path,
        roll: float,
        pitch: float,
        height: float,
        undistort: bool = False,
        birdview: bool = False,
    ) -> tuple[bool, str, Path | None, Path | None, Path | None]:
        executable = self.output_path()
        if self.needs_rebuild():
            ok, log = self.build()
            if not ok:
                return False, log, None, None, None
        output_dir.mkdir(parents=True, exist_ok=True)
        result_path = output_dir / "result.json"
        binary_path = output_dir / "binary.pgm"
        undistorted_path = output_dir / "undistorted.pgm"
        birdview_path = output_dir / "birdview.pgm"
        command = [
            str(executable),
            "--input", str(raw_path),
            "--output", str(result_path),
            "--binary", str(binary_path),
            "--roll", str(roll),
            "--pitch", str(pitch),
            "--height", str(height),
        ]
        if undistort:
            command.extend(("--undistorted", str(undistorted_path)))
        if birdview:
            command.extend(("--birdview", str(birdview_path)))
        result = subprocess.run(command, cwd=str(APP_ROOT), capture_output=True, text=True)
        log = "$ " + " ".join(command) + f"\n[returncode={result.returncode}]\n" + (result.stdout or "") + (result.stderr or "")
        if result.returncode != 0 or not result_path.exists():
            return False, log, None, None, None
        return (
            True,
            log,
            result_path,
            undistorted_path if undistorted_path.exists() else None,
            birdview_path if birdview_path.exists() else None,
        )


class VisionToolApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("视觉代码仿真工具")
        self.root.geometry("1500x900")
        self.root.minsize(1100, 700)

        self.source_var = tk.StringVar(value=str(DEFAULT_SOURCE))
        self.frames_var = tk.StringVar(value=str(DEFAULT_FRAMES))
        self.compiler_var = tk.StringVar(value="自动检测")
        self.roll_var = tk.StringVar(value="0")
        self.pitch_var = tk.StringVar(value="0")
        self.yaw_var = tk.StringVar(value="0")
        self.height_var = tk.StringVar(value="100")
        self.undistort_var = tk.BooleanVar(value=False)
        self.birdview_var = tk.BooleanVar(value=False)
        self.status_var = tk.StringVar(value="就绪")
        self.frame_info_var = tk.StringVar(value="未选择帧")
        self.frame_telemetry_var = tk.StringVar(value="帧姿态/高度：暂无")
        self.result_info_var = tk.StringVar(value="暂无仿真结果")
        self.preview_image: ImageTk.PhotoImage | None = None
        self.binary_image: ImageTk.PhotoImage | None = None
        self.current_source_image: Image.Image | None = None
        self.current_undistorted_image: Image.Image | None = None
        self.current_birdview_image: Image.Image | None = None
        self.processed_window: tk.Toplevel | None = None
        self.processed_label: ttk.Label | None = None
        self.processed_photo: ImageTk.PhotoImage | None = None
        self.current_frame: Path | None = None
        self.current_result: dict | None = None
        self.ui_queue: queue.Queue[tuple[str, object]] = queue.Queue()
        self.simulation_queue: queue.Queue[tuple[Path, float, float, float, bool, bool]] = queue.Queue()
        self.capture_stop_event = threading.Event()
        self.capture_running = False
        self.capture_out_var = tk.StringVar(value=str(APP_ROOT / "capture"))
        self.capture_port_var = tk.StringVar()
        self.capture_baud_var = tk.StringVar(value="115200")
        self.capture_count_var = tk.StringVar(value="10")
        self.capture_prefix_var = tk.StringVar(value="frame")
        self.capture_timeout_var = tk.StringVar(value="1.0")
        self.capture_status_var = tk.StringVar(value="未开始")
        self.editor_path: Path | None = None
        self.editor_dirty = False

        self.build_layout()
        self.refresh_sources()
        self.refresh_frames()
        self.refresh_capture_ports()
        threading.Thread(target=self.simulation_worker, daemon=True).start()
        self.root.after(100, self.process_queue)

    def build_layout(self) -> None:
        root_frame = ttk.Frame(self.root, padding=8)
        root_frame.pack(fill=tk.BOTH, expand=True)
        root_frame.rowconfigure(1, weight=1)
        root_frame.columnconfigure(0, weight=1)

        toolbar = ttk.Frame(root_frame)
        toolbar.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        toolbar.columnconfigure(1, weight=1)
        ttk.Label(toolbar, text="视觉源码").grid(row=0, column=0, sticky="w")
        ttk.Entry(toolbar, textvariable=self.source_var).grid(row=0, column=1, sticky="ew", padx=6)
        ttk.Button(toolbar, text="选择", command=self.choose_source).grid(row=0, column=2)
        ttk.Button(toolbar, text="打开目录", command=self.open_source).grid(row=0, column=3, padx=(6, 0))
        ttk.Button(toolbar, text="重新扫描", command=self.refresh_sources).grid(row=0, column=4, padx=(6, 0))
        ttk.Label(toolbar, text="编译器").grid(row=0, column=5, padx=(18, 4))
        ttk.Label(toolbar, textvariable=self.compiler_var).grid(row=0, column=6, sticky="w")

        main = ttk.PanedWindow(root_frame, orient=tk.HORIZONTAL)
        main.grid(row=1, column=0, sticky="nsew")
        left = ttk.Frame(main, padding=(0, 0, 6, 0))
        center = ttk.Frame(main, padding=(6, 0))
        right = ttk.Frame(main, padding=(6, 0, 0, 0))
        main.add(left, weight=1)
        main.add(center, weight=4)
        main.add(right, weight=2)
        self.build_source_panel(left)
        self.build_preview_panel(center)
        self.build_control_panel(right)

        log_frame = ttk.LabelFrame(root_frame, text="构建与运行日志", padding=4)
        log_frame.grid(row=2, column=0, sticky="ew", pady=(8, 0))
        self.log_text = scrolledtext.ScrolledText(log_frame, height=8, wrap=tk.NONE)
        self.log_text.pack(fill=tk.BOTH, expand=True)

    def build_source_panel(self, parent: ttk.Frame) -> None:
        parent.rowconfigure(1, weight=1)
        parent.rowconfigure(3, weight=2)
        parent.columnconfigure(0, weight=1)
        ttk.Label(parent, text="C/H 源文件").grid(row=0, column=0, sticky="w")
        self.source_list = tk.Listbox(parent, exportselection=False)
        self.source_list.grid(row=1, column=0, sticky="nsew", pady=(4, 8))
        self.source_list.bind("<<ListboxSelect>>", self.load_selected_source)
        ttk.Label(parent, text="源码编辑器").grid(row=2, column=0, sticky="w")
        self.editor = scrolledtext.ScrolledText(parent, wrap=tk.NONE, undo=True)
        self.editor.grid(row=3, column=0, sticky="nsew", pady=(4, 4))
        self.editor.bind("<<Modified>>", self.editor_modified)
        button_bar = ttk.Frame(parent)
        button_bar.grid(row=4, column=0, sticky="ew")
        button_bar.columnconfigure(0, weight=1)
        button_bar.columnconfigure(1, weight=1)
        ttk.Button(button_bar, text="保存源码", command=self.save_source).grid(row=0, column=0, sticky="ew", padx=(0, 3))
        ttk.Button(button_bar, text="放弃修改", command=self.reload_source).grid(row=0, column=1, sticky="ew", padx=(3, 0))

    def build_preview_panel(self, parent: ttk.Frame) -> None:
        parent.rowconfigure(2, weight=1)
        parent.columnconfigure(0, weight=1)
        ttk.Label(parent, textvariable=self.frame_info_var).grid(row=0, column=0, sticky="w")
        ttk.Label(parent, textvariable=self.frame_telemetry_var).grid(row=1, column=0, sticky="w", pady=(3, 0))
        image_frame = ttk.Frame(parent)
        image_frame.grid(row=2, column=0, sticky="nsew", pady=(6, 0))
        image_frame.columnconfigure(0, weight=1)
        image_frame.columnconfigure(1, weight=1)
        image_frame.rowconfigure(0, weight=1)
        self.preview_label = ttk.Label(image_frame, text="原图", anchor="center")
        self.preview_label.grid(row=0, column=0, sticky="nsew", padx=(0, 4))
        self.binary_label = ttk.Label(image_frame, text="二值图", anchor="center")
        self.binary_label.grid(row=0, column=1, sticky="nsew", padx=(4, 0))

    def build_control_panel(self, parent: ttk.Frame) -> None:
        parent.columnconfigure(1, weight=1)
        capture = ttk.LabelFrame(parent, text="串口抓图（自动仿真）", padding=6)
        capture.grid(row=0, column=0, columnspan=3, sticky="ew", pady=(0, 10))
        capture.columnconfigure(1, weight=1)
        ttk.Label(capture, text="串口").grid(row=0, column=0, sticky="w")
        self.capture_port_combo = ttk.Combobox(capture, textvariable=self.capture_port_var)
        self.capture_port_combo.grid(row=0, column=1, sticky="ew", padx=5)
        ttk.Button(capture, text="刷新", command=self.refresh_capture_ports).grid(row=0, column=2)
        ttk.Label(capture, text="波特率").grid(row=1, column=0, sticky="w", pady=3)
        ttk.Entry(capture, textvariable=self.capture_baud_var, width=10).grid(row=1, column=1, sticky="w", padx=5, pady=3)
        ttk.Label(capture, text="数量").grid(row=1, column=2, sticky="e")
        ttk.Entry(capture, textvariable=self.capture_count_var, width=7).grid(row=1, column=3, sticky="w", padx=5, pady=3)
        ttk.Label(capture, text="输出").grid(row=2, column=0, sticky="w", pady=3)
        ttk.Entry(capture, textvariable=self.capture_out_var).grid(row=2, column=1, columnspan=2, sticky="ew", padx=5, pady=3)
        ttk.Button(capture, text="选择", command=self.choose_capture_output).grid(row=2, column=3, pady=3)
        ttk.Label(capture, text="超时(s)").grid(row=3, column=0, sticky="w", pady=3)
        ttk.Entry(capture, textvariable=self.capture_timeout_var, width=10).grid(row=3, column=1, sticky="w", padx=5, pady=3)
        ttk.Label(capture, text="前缀").grid(row=3, column=2, sticky="e", pady=3)
        ttk.Entry(capture, textvariable=self.capture_prefix_var, width=10).grid(row=3, column=3, sticky="w", padx=5, pady=3)
        self.capture_start_button = ttk.Button(capture, text="开始抓图", command=self.start_capture)
        self.capture_start_button.grid(row=4, column=0, columnspan=2, sticky="ew", pady=(5, 0), padx=(0, 3))
        self.capture_stop_button = ttk.Button(capture, text="停止", command=self.stop_capture, state=tk.DISABLED)
        self.capture_stop_button.grid(row=4, column=2, columnspan=2, sticky="ew", pady=(5, 0), padx=(3, 0))
        ttk.Label(capture, textvariable=self.capture_status_var).grid(row=5, column=0, columnspan=4, sticky="w", pady=(5, 0))

        ttk.Label(parent, text="输入帧目录").grid(row=1, column=0, sticky="w")
        ttk.Entry(parent, textvariable=self.frames_var).grid(row=1, column=1, sticky="ew", padx=5)
        ttk.Button(parent, text="选择", command=self.choose_frames).grid(row=1, column=2)
        ttk.Label(parent, text="帧列表").grid(row=2, column=0, columnspan=3, sticky="w", pady=(12, 3))
        self.frame_list = tk.Listbox(parent, height=10, exportselection=False)
        self.frame_list.grid(row=3, column=0, columnspan=3, sticky="ew")
        self.frame_list.bind("<<ListboxSelect>>", self.select_frame)
        ttk.Button(parent, text="刷新帧", command=self.refresh_frames).grid(row=4, column=0, columnspan=3, sticky="ew", pady=(4, 12))

        params = ttk.LabelFrame(parent, text="仿真参数", padding=6)
        params.grid(row=5, column=0, columnspan=3, sticky="ew")
        params.columnconfigure(1, weight=1)
        for row, (label, variable) in enumerate((
            ("Roll (deg)", self.roll_var),
            ("Pitch (deg)", self.pitch_var),
            ("Yaw (deg, record)", self.yaw_var),
            ("ToF height (cm)", self.height_var),
        )):
            ttk.Label(params, text=label).grid(row=row, column=0, sticky="w", pady=3)
            ttk.Entry(params, textvariable=variable).grid(row=row, column=1, sticky="ew", pady=3)

        ttk.Checkbutton(
            parent,
            text="显示去畸变图（运行仿真生成）",
            variable=self.undistort_var,
            command=self.refresh_processed_preview,
        ).grid(row=6, column=0, columnspan=3, sticky="w", pady=(8, 0))
        ttk.Checkbutton(
            parent,
            text="显示逆透视图（运行仿真生成）",
            variable=self.birdview_var,
            command=self.refresh_processed_preview,
        ).grid(row=7, column=0, columnspan=3, sticky="w", pady=(4, 0))
        ttk.Button(parent, text="打开调整图像窗口", command=self.open_processed_window).grid(row=8, column=0, columnspan=3, sticky="ew", pady=(8, 0))
        ttk.Button(parent, text="构建 C 仿真", command=self.start_build).grid(row=9, column=0, columnspan=3, sticky="ew", pady=(12, 4))
        ttk.Button(parent, text="运行当前帧", command=self.start_run).grid(row=10, column=0, columnspan=3, sticky="ew", pady=4)
        ttk.Label(parent, textvariable=self.status_var, foreground="#1b6e3a", wraplength=280).grid(row=11, column=0, columnspan=3, sticky="w", pady=(10, 3))
        ttk.Label(parent, textvariable=self.result_info_var, justify=tk.LEFT, wraplength=280).grid(row=12, column=0, columnspan=3, sticky="w")

    def log(self, message: str) -> None:
        self.log_text.insert(tk.END, message.rstrip() + "\n")
        self.log_text.see(tk.END)

    def choose_source(self) -> None:
        selected = filedialog.askdirectory(initialdir=self.source_var.get() or str(APP_ROOT))
        if selected:
            self.source_var.set(selected)
            self.refresh_sources()

    def choose_frames(self) -> None:
        selected = filedialog.askdirectory(initialdir=self.frames_var.get() or str(APP_ROOT))
        if selected:
            self.frames_var.set(selected)
            self.refresh_frames()

    def choose_capture_output(self) -> None:
        selected = filedialog.askdirectory(initialdir=self.capture_out_var.get() or str(APP_ROOT))
        if selected:
            self.capture_out_var.set(selected)

    def refresh_capture_ports(self) -> None:
        ports = list_serial_ports()
        self.capture_port_combo["values"] = ports
        if ports and self.capture_port_var.get() not in ports:
            self.capture_port_var.set(ports[0])
        if not ports:
            self.capture_port_var.set("")
        self.log("serial ports: " + (", ".join(ports) if ports else "none found"))

    def start_capture(self) -> None:
        if self.capture_running:
            return
        if not self.capture_port_var.get():
            messagebox.showerror("Capture", "Select a serial port first.")
            return
        try:
            baud = int(self.capture_baud_var.get())
            count = int(self.capture_count_var.get())
            timeout = float(self.capture_timeout_var.get())
            if baud <= 0 or count <= 0 or timeout <= 0:
                raise ValueError
        except ValueError:
            messagebox.showerror("Capture", "Baud, count and timeout must be positive numbers.")
            return
        self.capture_stop_event.clear()
        self.capture_running = True
        output = Path(self.capture_out_var.get())
        self.frames_var.set(str(output))
        self.capture_start_button.config(state=tk.DISABLED)
        self.capture_stop_button.config(state=tk.NORMAL)
        self.capture_status_var.set("抓图中")
        threading.Thread(
            target=self.capture_worker,
            args=(self.capture_port_var.get(), baud, count, output, self.capture_prefix_var.get(), timeout),
            daemon=True,
        ).start()

    def stop_capture(self) -> None:
        if self.capture_running:
            self.capture_stop_event.set()
            self.capture_status_var.set("正在停止")

    def capture_worker(self, port: str, baud: int, count: int, output: Path, prefix: str, timeout: float) -> None:
        try:
            frames = capture_frames(
                port_name=port,
                baud=baud,
                count=count,
                out_dir=output,
                prefix=prefix,
                timeout=timeout,
                on_frame=lambda frame: self.ui_queue.put(("capture_frame", frame)),
                on_log=lambda message: self.ui_queue.put(("capture_log", message)),
                should_stop=self.capture_stop_event.is_set,
            )
            self.ui_queue.put(("capture_done", len(frames)))
        except Exception as exc:
            self.ui_queue.put(("capture_error", str(exc)))

    def enqueue_auto_simulation(self, frame_path: Path) -> None:
        try:
            roll = float(self.roll_var.get())
            pitch = float(self.pitch_var.get())
            height = float(self.height_var.get())
        except ValueError:
            self.ui_queue.put(("run", (False, "simulation parameters are invalid", None, APP_ROOT, None, None, None)))
            return
        self.simulation_queue.put((
            frame_path,
            roll,
            pitch,
            height,
            bool(self.undistort_var.get()),
            bool(self.birdview_var.get()),
        ))

    def simulation_worker(self) -> None:
        while True:
            frame_path, roll, pitch, height, undistort, birdview = self.simulation_queue.get()
            try:
                self.run_worker(frame_path, roll, pitch, height, undistort, birdview)
            except Exception as exc:
                self.ui_queue.put(("run", (False, str(exc), None, APP_ROOT, None, None, None)))

    def open_source(self) -> None:
        path = Path(self.source_var.get())
        if os.name == "nt":
            os.startfile(str(path))
        else:
            subprocess.Popen(["xdg-open", str(path)])

    def refresh_sources(self) -> None:
        source_dir = Path(self.source_var.get())
        self.source_list.delete(0, tk.END)
        if not source_dir.is_dir():
            self.compiler_var.set("源码目录不存在")
            return
        for path in sorted(source_dir.glob("*")):
            if path.suffix.lower() in {".c", ".h"}:
                self.source_list.insert(tk.END, path.name)
        compiler = Simulator(source_dir).find_compiler()
        self.compiler_var.set(compiler or "未找到 GCC/Clang")

    def refresh_frames(self) -> None:
        frame_dir = Path(self.frames_var.get())
        self.frame_list.delete(0, tk.END)
        if not frame_dir.is_dir():
            self.frame_info_var.set("帧目录不存在")
            return
        files = sorted([*frame_dir.glob("*.png"), *frame_dir.glob("*.raw"), *frame_dir.glob("*.pgm")])
        for path in files:
            self.frame_list.insert(tk.END, path.name)
        if files:
            self.frame_list.selection_set(0)
            self.select_frame()

    def select_frame(self, _event: object = None) -> None:
        selection = self.frame_list.curselection()
        if not selection:
            return
        self.current_frame = Path(self.frames_var.get()) / self.frame_list.get(selection[0])
        self.show_frame(self.current_frame)

    def show_frame(self, path: Path) -> None:
        try:
            image = self.load_image(path)
        except Exception as exc:
            self.status_var.set(f"读取帧失败: {exc}")
            return
        self.current_source_image = image
        self.current_undistorted_image = None
        self.current_birdview_image = None
        self.apply_frame_metadata(path)
        self.frame_info_var.set(f"{path.name}  {image.width}×{image.height}")
        self.refresh_preview()
        self.refresh_processed_preview()

    def refresh_preview(self) -> None:
        if self.current_source_image is None:
            self.preview_label.configure(image="", text="未选择帧")
            return
        image = self.current_source_image
        display = self.fit_image(image, self.preview_label.winfo_width() or 560, self.preview_label.winfo_height() or 360)
        self.preview_image = ImageTk.PhotoImage(display)
        self.preview_label.configure(image=self.preview_image, text="")

    def open_processed_window(self) -> None:
        if self.processed_window is not None and self.processed_window.winfo_exists():
            self.processed_window.deiconify()
            self.processed_window.lift()
            self.refresh_processed_preview()
            return

        self.processed_window = tk.Toplevel(self.root)
        self.processed_window.title("调整图像")
        self.processed_window.geometry("720x560")
        self.processed_window.minsize(360, 300)
        self.processed_label = ttk.Label(self.processed_window, text="请勾选处理选项并运行当前帧", anchor="center")
        self.processed_label.pack(fill=tk.BOTH, expand=True, padx=12, pady=12)
        self.processed_window.protocol("WM_DELETE_WINDOW", self.close_processed_window)
        self.refresh_processed_preview()

    def close_processed_window(self) -> None:
        if self.processed_window is not None and self.processed_window.winfo_exists():
            self.processed_window.destroy()
        self.processed_window = None
        self.processed_label = None
        self.processed_photo = None

    def refresh_processed_preview(self) -> None:
        if self.processed_label is None or self.processed_window is None or not self.processed_window.winfo_exists():
            return

        if self.birdview_var.get():
            image = self.current_birdview_image
            title = "去畸变 + 逆透视图" if self.undistort_var.get() else "逆透视图"
            missing_text = "请先运行当前帧生成逆透视图"
        elif self.undistort_var.get():
            image = self.current_undistorted_image
            title = "去畸变图"
            missing_text = "请先运行当前帧生成去畸变图"
        else:
            image = None
            title = "调整图像"
            missing_text = "请勾选去畸变或逆透视选项"

        if image is None:
            self.processed_photo = None
            self.processed_label.configure(image="", text=missing_text)
            self.processed_window.title(title)
            return

        display = self.fit_image(
            image,
            self.processed_label.winfo_width() or 680,
            self.processed_label.winfo_height() or 500,
        )
        self.processed_photo = ImageTk.PhotoImage(display)
        self.processed_label.configure(image=self.processed_photo, text="")
        self.processed_window.title(title)

    def apply_frame_metadata(self, path: Path) -> None:
        metadata_path = path.with_suffix(".json")
        try:
            payload = json.loads(metadata_path.read_text(encoding="utf-8"))
            telemetry = payload.get("telemetry")
            if not isinstance(telemetry, dict):
                self.frame_telemetry_var.set("帧姿态/高度：无元数据")
                return
            roll = float(telemetry["roll_deg"])
            pitch = float(telemetry["pitch_deg"])
            yaw = float(telemetry["yaw_deg"])
            tof_height = float(telemetry["tof_height_cm"])
            fused_height = float(telemetry["fused_height_cm"])
            self.roll_var.set(f"{roll:.6f}")
            self.pitch_var.set(f"{pitch:.6f}")
            self.yaw_var.set(f"{yaw:.6f}")
            self.height_var.set(f"{tof_height:.6f}")
            image_seq = telemetry.get("image_seq", "-")
            input_seq = telemetry.get("input_seq", "-")
            self.frame_telemetry_var.set(
                f"帧元数据 seq={image_seq}/{input_seq}  "
                f"R={roll:.2f}° P={pitch:.2f}° Y={yaw:.2f}°  "
                f"ToF={tof_height:.1f}cm Fusion={fused_height:.1f}cm"
            )
        except (OSError, ValueError, TypeError, KeyError, json.JSONDecodeError):
            self.frame_telemetry_var.set("帧姿态/高度：无元数据")

    @staticmethod
    def load_image(path: Path, target_size: tuple[int, int] | None = (WIDTH, HEIGHT)) -> Image.Image:
        if path.suffix.lower() == ".raw":
            payload = path.read_bytes()
            if len(payload) != WIDTH * HEIGHT:
                raise ValueError(f"RAW 大小为 {len(payload)}，需要 {WIDTH * HEIGHT}")
            return Image.frombytes("L", (WIDTH, HEIGHT), payload)
        image = Image.open(path).convert("L")
        if target_size is not None and image.size != target_size:
            image = image.resize(target_size, Image.Resampling.NEAREST)
        return image

    @staticmethod
    def fit_image(image: Image.Image, width: int, height: int) -> Image.Image:
        scale = min(max(width, 1) / image.width, max(height, 1) / image.height)
        size = (max(1, int(image.width * scale)), max(1, int(image.height * scale)))
        return image.resize(size, Image.Resampling.NEAREST)

    def start_build(self) -> None:
        source = Path(self.source_var.get())
        threshold = 150
        self.status_var.set("构建中")
        threading.Thread(target=self.build_worker, args=(source, threshold), daemon=True).start()

    def build_worker(self, source: Path, threshold: int) -> None:
        ok, log = Simulator(source, threshold).build()
        self.ui_queue.put(("build", (ok, log)))

    def start_run(self) -> None:
        if self.current_frame is None:
            messagebox.showerror("无法运行", "请先选择一帧图像。")
            return
        try:
            roll = float(self.roll_var.get())
            pitch = float(self.pitch_var.get())
            height = float(self.height_var.get())
        except ValueError:
            messagebox.showerror("参数错误", "Roll、Pitch、高度必须是数字。")
            return
        self.status_var.set("运行中")
        undistort = bool(self.undistort_var.get())
        birdview = bool(self.birdview_var.get())
        threading.Thread(
            target=self.run_worker,
            args=(self.current_frame, roll, pitch, height, undistort, birdview),
            daemon=True,
        ).start()

    def run_worker(
        self,
        frame_path: Path,
        roll: float,
        pitch: float,
        height: float,
        undistort: bool,
        birdview: bool,
    ) -> None:
        source = Path(self.source_var.get())
        run_dir = RUN_ROOT / time.strftime("run_%Y%m%d_%H%M%S")
        run_dir.mkdir(parents=True, exist_ok=True)
        try:
            image = self.load_image(frame_path)
            raw_path = run_dir / "input.raw"
            raw_path.write_bytes(image.tobytes())
            simulator = Simulator(source)
            ok, log, result_path, undistorted_path, birdview_path = simulator.run(
                raw_path, run_dir, roll, pitch, height, undistort, birdview
            )
            payload = json.loads(result_path.read_text(encoding="utf-8")) if result_path else None
            undistorted_image = self.load_image(undistorted_path) if undistorted_path else None
            birdview_image = self.load_image(birdview_path, target_size=None) if birdview_path else None
            self.ui_queue.put(("run", (ok, log, payload, run_dir, image, undistorted_image, birdview_image)))
        except Exception as exc:
            self.ui_queue.put(("run", (False, str(exc), None, run_dir, None, None, None)))

    def process_queue(self) -> None:
        while True:
            try:
                event, payload = self.ui_queue.get_nowait()
            except queue.Empty:
                break
            if event == "capture_log":
                self.log(str(payload))
            elif event == "capture_frame":
                frame = payload
                image_path = frame.get("image_path")
                if image_path:
                    self.current_frame = Path(image_path)
                    self.show_frame(self.current_frame)
                    self.enqueue_auto_simulation(self.current_frame)
                self.refresh_frames()
                self.capture_status_var.set(f"已保存 {frame['index'] + 1} 帧，正在仿真")
            elif event == "capture_done":
                self.capture_running = False
                self.capture_start_button.config(state=tk.NORMAL)
                self.capture_stop_button.config(state=tk.DISABLED)
                self.capture_status_var.set(f"完成，共保存 {payload} 帧")
                self.refresh_frames()
            elif event == "capture_error":
                self.capture_running = False
                self.capture_start_button.config(state=tk.NORMAL)
                self.capture_stop_button.config(state=tk.DISABLED)
                self.capture_status_var.set("失败")
                self.log("capture error: " + str(payload))
                messagebox.showerror("Capture", str(payload))
            elif event == "build":
                ok, log = payload
                self.log(log)
                self.status_var.set("构建成功" if ok else "构建失败")
            elif event == "run":
                ok, log, result, run_dir, image, undistorted_image, birdview_image = payload
                self.log(log)
                self.status_var.set("运行完成" if ok else "运行失败")
                if ok and result and image:
                    self.current_result = result
                    self.show_result(
                        image,
                        result,
                        Path(run_dir) / "binary.pgm",
                        undistorted_image,
                        birdview_image,
                    )
        self.root.after(100, self.process_queue)

    def show_result(
        self,
        image: Image.Image,
        result: dict,
        binary_path: Path,
        undistorted_image: Image.Image | None = None,
        birdview_image: Image.Image | None = None,
    ) -> None:
        original = image.convert("RGB")
        binary = Image.open(binary_path).convert("RGB") if binary_path.exists() else image.convert("RGB")
        draw_original = ImageDraw.Draw(original)
        draw_binary = ImageDraw.Draw(binary)
        for beacon in result.get("beacons", []):
            if not beacon.get("valid"):
                continue
            box = (beacon["min_col"], beacon["min_row"], beacon["max_col"], beacon["max_row"])
            draw_original.rectangle(box, outline=(0, 255, 255), width=1)
            draw_binary.rectangle(box, outline=(0, 255, 255), width=1)
            cx = (beacon["min_col"] + beacon["max_col"]) // 2
            cy = (beacon["min_row"] + beacon["max_row"]) // 2
            draw_original.ellipse((cx - 2, cy - 2, cx + 2, cy + 2), outline=(255, 0, 0), width=1)
            draw_binary.ellipse((cx - 2, cy - 2, cx + 2, cy + 2), outline=(255, 0, 0), width=1)
        chevron = result.get("chevron", {})
        if chevron.get("valid"):
            def point(name: str) -> tuple[int, int]:
                item = chevron.get(name, {})
                return int(round(item.get("u", 0))), int(round(item.get("v", 0)))

            def draw_chevron(draw: ImageDraw.ImageDraw) -> None:
                draw.line([point("left_line_start_img"), point("left_line_end_img")], fill=(0, 255, 0), width=1)
                draw.line([point("right_line_start_img"), point("right_line_end_img")], fill=(0, 255, 0), width=1)
                apex = point("apex_img")
                left_tip = point("left_tip_img")
                right_tip = point("right_tip_img")
                for px, py, color in ((apex[0], apex[1], (255, 0, 0)),
                                      (left_tip[0], left_tip[1], (0, 128, 255)),
                                      (right_tip[0], right_tip[1], (0, 128, 255))):
                    draw.ellipse((px - 2, py - 2, px + 2, py + 2), outline=color, width=1)

            draw_chevron(draw_original)
            draw_chevron(draw_binary)
        self.current_source_image = original
        self.current_undistorted_image = undistorted_image.convert("RGB") if undistorted_image is not None else None
        self.current_birdview_image = birdview_image.convert("RGB") if birdview_image is not None else None
        self.binary_image = ImageTk.PhotoImage(self.fit_image(binary, self.binary_label.winfo_width() or 560, self.binary_label.winfo_height() or 360))
        self.binary_label.configure(image=self.binary_image, text="")
        self.refresh_preview()
        self.refresh_processed_preview()
        self.result_info_var.set(
            f"连通域: {result.get('blob_count', 0)}\n"
            f"信标: {result.get('beacon_count', 0)}\n"
            f"小车标志有效: {result.get('vehicle', {}).get('valid', 0)}\n"
            f"运行目录: {Path(result.get('binary_path', '')).parent}"
        )
        if self.undistort_var.get() or self.birdview_var.get():
            self.open_processed_window()

    def editor_modified(self, _event: object = None) -> None:
        if self.editor.edit_modified():
            self.editor_dirty = True
            self.editor.edit_modified(False)

    def load_selected_source(self, _event: object = None) -> None:
        selection = self.source_list.curselection()
        if not selection:
            return
        path = Path(self.source_var.get()) / self.source_list.get(selection[0])
        self.editor_path = path
        self.editor.delete("1.0", tk.END)
        self.editor.insert("1.0", path.read_text(encoding="utf-8"))
        self.editor.edit_modified(False)
        self.editor_dirty = False

    def reload_source(self) -> None:
        if self.editor_path:
            self.editor.delete("1.0", tk.END)
            self.editor.insert("1.0", self.editor_path.read_text(encoding="utf-8"))
            self.editor.edit_modified(False)
            self.editor_dirty = False

    def save_source(self) -> None:
        if not self.editor_path:
            return
        self.editor_path.write_text(self.editor.get("1.0", tk.END).rstrip() + "\n", encoding="utf-8", newline="\n")
        self.editor_dirty = False
        self.status_var.set(f"已保存 {self.editor_path.name}，下次运行会重新编译")


def self_test() -> int:
    assert WIDTH == 188 and HEIGHT == 120
    assert (SIM_ROOT / "vision_sim_main.c").exists()
    assert Simulator(DEFAULT_SOURCE).source_files()
    print("vision tool self-test passed")
    return 0


def main() -> int:
    if "--self-test" in sys.argv:
        return self_test()
    root = tk.Tk()
    VisionToolApp(root)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

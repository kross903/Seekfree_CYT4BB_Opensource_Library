from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from .schema import AXIS_CHAINS


def _save_plot(fig: plt.Figure, path: Path) -> None:
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def plot_overview(df: pd.DataFrame, x_col: str, out_dir: Path) -> None:
    fig, axes = plt.subplots(4, 2, figsize=(15, 12), sharex=True)
    if len(df) == 0:
        for ax in axes.flat:
            ax.text(0.5, 0.5, "No data for selected segment", ha="center", va="center")
            ax.set_axis_off()
        _save_plot(fig, out_dir / "FP_overview.png")
        return
    sample = df[x_col]

    axes[0, 0].plot(sample, df["x_pos_fb_cm"], label="x pos fb cm")
    axes[0, 0].plot(sample, df["x_v_target_cms"], label="x v target cm/s")
    axes[0, 0].plot(sample, df["x_v_fb_cms"], label="x v fb cm/s")
    axes[0, 0].set_title("X/Pitch position and velocity")
    axes[0, 0].legend(loc="best")
    axes[0, 0].grid(True, alpha=0.3)

    axes[0, 1].plot(sample, df["y_pos_fb_cm"], label="y pos fb cm")
    axes[0, 1].plot(sample, df["y_v_target_cms"], label="y v target cm/s")
    axes[0, 1].plot(sample, df["y_v_fb_cms"], label="y v fb cm/s")
    axes[0, 1].set_title("Y/Roll position and velocity")
    axes[0, 1].legend(loc="best")
    axes[0, 1].grid(True, alpha=0.3)

    axes[1, 0].plot(sample, df["pitch_target_deg"], label="pitch target deg")
    axes[1, 0].plot(sample, df["pitch_current_deg"], label="pitch current deg")
    axes[1, 0].set_title("Pitch angle tracking")
    axes[1, 0].legend(loc="best")
    axes[1, 0].grid(True, alpha=0.3)

    axes[1, 1].plot(sample, df["roll_target_deg"], label="roll target deg")
    axes[1, 1].plot(sample, df["roll_current_deg"], label="roll current deg")
    axes[1, 1].set_title("Roll angle tracking")
    axes[1, 1].legend(loc="best")
    axes[1, 1].grid(True, alpha=0.3)

    axes[2, 0].plot(sample, df["pitch_rate_target_dps"], label="pitch rate target")
    axes[2, 0].plot(sample, df["pitch_rate_current_dps"], label="pitch rate current")
    axes[2, 0].set_title("Pitch rate tracking")
    axes[2, 0].legend(loc="best")
    axes[2, 0].grid(True, alpha=0.3)

    axes[2, 1].plot(sample, df["roll_rate_target_dps"], label="roll rate target")
    axes[2, 1].plot(sample, df["roll_rate_current_dps"], label="roll rate current")
    axes[2, 1].set_title("Roll rate tracking")
    axes[2, 1].legend(loc="best")
    axes[2, 1].grid(True, alpha=0.3)

    axes[3, 0].plot(df["x_pos_fb_cm"], df["y_pos_fb_cm"])
    axes[3, 0].scatter(df["x_pos_fb_cm"].iloc[0], df["y_pos_fb_cm"].iloc[0], s=35, label="start")
    axes[3, 0].scatter(df["x_pos_fb_cm"].iloc[-1], df["y_pos_fb_cm"].iloc[-1], s=35, label="end")
    axes[3, 0].set_title("XY position feedback path")
    axes[3, 0].set_xlabel("x pos fb cm")
    axes[3, 0].set_ylabel("y pos fb cm")
    axes[3, 0].axis("equal")
    axes[3, 0].legend(loc="best")
    axes[3, 0].grid(True, alpha=0.3)

    axes[3, 1].plot(sample, df["position_loop_active"], label="position loop active")
    axes[3, 1].set_title("Position loop active flag")
    axes[3, 1].set_ylim(-0.1, 1.1)
    axes[3, 1].legend(loc="best")
    axes[3, 1].grid(True, alpha=0.3)

    for ax in axes[-1, :]:
        ax.set_xlabel(x_col)

    _save_plot(fig, out_dir / "FP_overview.png")


def plot_axis_chain(df: pd.DataFrame, x_col: str, out_dir: Path, axis_name: str) -> None:
    axis_info = AXIS_CHAINS[axis_name]
    if axis_name == "x":
        series = [
            ("position/velocity", ["x_pos_fb_cm", "x_v_target_cms", "x_v_fb_cms"]),
            ("pitch angle", ["pitch_target_deg", "pitch_current_deg"]),
            ("pitch rate", ["pitch_rate_target_dps", "pitch_rate_current_dps"]),
        ]
        filename = "FP_x_pitch_chain.png"
    else:
        series = [
            ("position/velocity", ["y_pos_fb_cm", "y_v_target_cms", "y_v_fb_cms"]),
            ("roll angle", ["roll_target_deg", "roll_current_deg"]),
            ("roll rate", ["roll_rate_target_dps", "roll_rate_current_dps"]),
        ]
        filename = "FP_y_roll_chain.png"

    fig, axes = plt.subplots(len(series), 1, figsize=(13, 9), sharex=True)
    fig.suptitle(f"{axis_info['title']} control chain")
    if len(df) == 0:
        for ax in axes:
            ax.text(0.5, 0.5, "No data for selected segment", ha="center", va="center")
            ax.set_axis_off()
        _save_plot(fig, out_dir / filename)
        return
    sample = df[x_col]

    for ax, (subtitle, cols) in zip(axes, series):
        for col in cols:
            ax.plot(sample, df[col], label=col)
        ax.set_title(subtitle)
        ax.legend(loc="best")
        ax.grid(True, alpha=0.3)
    axes[-1].set_xlabel(x_col)

    _save_plot(fig, out_dir / filename)


def plot_axis_errors(df: pd.DataFrame, x_col: str, out_dir: Path, axis_name: str) -> None:
    if axis_name == "x":
        pairs = [
            ("velocity error", "x_v_target_cms", "x_v_fb_cms"),
            ("pitch angle error", "pitch_target_deg", "pitch_current_deg"),
            ("pitch rate error", "pitch_rate_target_dps", "pitch_rate_current_dps"),
        ]
        filename = "FP_x_errors.png"
    else:
        pairs = [
            ("velocity error", "y_v_target_cms", "y_v_fb_cms"),
            ("roll angle error", "roll_target_deg", "roll_current_deg"),
            ("roll rate error", "roll_rate_target_dps", "roll_rate_current_dps"),
        ]
        filename = "FP_y_errors.png"

    fig, axes = plt.subplots(len(pairs), 1, figsize=(13, 9), sharex=True)
    if len(df) == 0:
        for ax in axes:
            ax.text(0.5, 0.5, "No data for selected segment", ha="center", va="center")
            ax.set_axis_off()
        _save_plot(fig, out_dir / filename)
        return
    sample = df[x_col]
    for ax, (title, target_col, feedback_col) in zip(axes, pairs):
        error = df[target_col] - df[feedback_col]
        ax.plot(sample, error, label=title)
        ax.axhline(0.0, color="black", linewidth=0.8, alpha=0.5)
        ax.set_title(title)
        ax.legend(loc="best")
        ax.grid(True, alpha=0.3)
    axes[-1].set_xlabel(x_col)

    _save_plot(fig, out_dir / filename)


def plot_axis_spectrum(df: pd.DataFrame, out_dir: Path, axis_name: str, hz: float | None) -> None:
    if axis_name == "x":
        cols = ["x_v_fb_cms", "pitch_current_deg", "pitch_rate_current_dps"]
        filename = "FP_x_spectrum.png"
    else:
        cols = ["y_v_fb_cms", "roll_current_deg", "roll_rate_current_dps"]
        filename = "FP_y_spectrum.png"

    fig, axes = plt.subplots(len(cols), 1, figsize=(13, 9), sharex=True)
    if len(df) == 0:
        for ax in axes:
            ax.text(0.5, 0.5, "No data for selected segment", ha="center", va="center")
            ax.set_axis_off()
        _save_plot(fig, out_dir / filename)
        return
    for ax, col in zip(axes, cols):
        series = df[col].to_numpy(dtype=float)
        centered = series - np.mean(series) if len(series) else series
        spectrum = np.abs(np.fft.rfft(centered)) if len(series) else np.array([0.0])
        if hz:
            freq = np.fft.rfftfreq(len(series), d=1.0 / hz) if len(series) else np.array([0.0])
            ax.set_xlabel("Hz")
        else:
            freq = np.fft.rfftfreq(len(series), d=1.0) if len(series) else np.array([0.0])
            ax.set_xlabel("cycles/sample")
        ax.plot(freq, spectrum, label=col)
        ax.set_title(f"Spectrum - {col}")
        ax.legend(loc="best")
        ax.grid(True, alpha=0.3)

    _save_plot(fig, out_dir / filename)

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np
import pandas as pd

from .schema import AXIS_CHAINS, DEFAULT_SATURATION_HINTS, NUMERIC_EXCLUDE


@dataclass
class PairMetrics:
    stage: str
    target_col: str
    feedback_col: str
    active: bool
    count: int
    target_nonzero_pct: float
    feedback_nonzero_pct: float
    error_mean: float
    error_std: float
    error_mae: float
    error_rms: float
    error_peak_abs: float
    correlation: float | None
    lag_samples: int | None
    lag_seconds: float | None
    dominant_norm_freq: float | None
    dominant_hz: float | None
    dominant_period_samples: float | None
    dominant_period_s: float | None
    abnormal_segments: list[tuple[int, int]]
    saturation_hits: int
    target_all_zero: bool
    feedback_all_zero: bool


def _nonzero_pct(series: pd.Series) -> float:
    if len(series) == 0:
        return 0.0
    return float(series.abs().gt(1e-6).sum() / len(series) * 100.0)


def _safe_corr(a: pd.Series, b: pd.Series) -> float | None:
    if len(a) < 2 or a.std(ddof=0) < 1e-9 or b.std(ddof=0) < 1e-9:
        return None
    return float(a.corr(b))


def _xcorr_lag(a: np.ndarray, b: np.ndarray) -> int | None:
    if len(a) < 4 or np.std(a) < 1e-9 or np.std(b) < 1e-9:
        return None
    a = a - np.mean(a)
    b = b - np.mean(b)
    corr = np.correlate(a, b, mode="full")
    lag = int(np.argmax(corr) - (len(a) - 1))
    return lag


def _dominant_frequency(series: np.ndarray, hz: float | None) -> dict[str, float | None]:
    if len(series) < 8:
        return {
            "dominant_norm_freq": None,
            "dominant_hz": None,
            "dominant_period_samples": None,
            "dominant_period_s": None,
        }

    centered = series - np.mean(series)
    if np.std(centered) < 1e-9:
        return {
            "dominant_norm_freq": None,
            "dominant_hz": None,
            "dominant_period_samples": None,
            "dominant_period_s": None,
        }

    spectrum = np.fft.rfft(centered)
    amps = np.abs(spectrum)
    if len(amps) <= 1:
        return {
            "dominant_norm_freq": None,
            "dominant_hz": None,
            "dominant_period_samples": None,
            "dominant_period_s": None,
        }

    amps[0] = 0.0
    peak_index = int(np.argmax(amps))
    if peak_index <= 0 or amps[peak_index] <= 0.0:
        return {
            "dominant_norm_freq": None,
            "dominant_hz": None,
            "dominant_period_samples": None,
            "dominant_period_s": None,
        }

    freqs_norm = np.fft.rfftfreq(len(centered), d=1.0)
    dom_norm = float(freqs_norm[peak_index])
    dom_hz = float(dom_norm * hz) if hz else None
    dom_period_samples = (1.0 / dom_norm) if dom_norm > 1e-9 else None
    dom_period_s = (1.0 / dom_hz) if dom_hz and dom_hz > 1e-9 else None
    return {
        "dominant_norm_freq": dom_norm,
        "dominant_hz": dom_hz,
        "dominant_period_samples": dom_period_samples,
        "dominant_period_s": dom_period_s,
    }


def _find_abnormal_segments(error_abs: pd.Series) -> list[tuple[int, int]]:
    if len(error_abs) == 0:
        return []

    threshold = float(error_abs.mean() + 2.5 * error_abs.std(ddof=0))
    if threshold <= 1e-6:
        return []

    mask = error_abs > threshold
    segments: list[tuple[int, int]] = []
    start: int | None = None
    for idx, flagged in enumerate(mask.tolist()):
        if flagged and start is None:
            start = idx
        elif not flagged and start is not None:
            if idx - start >= 3:
                segments.append((start, idx - 1))
            start = None
    if start is not None and len(mask) - start >= 3:
        segments.append((start, len(mask) - 1))
    return segments[:8]


def compute_summary(df: pd.DataFrame) -> pd.DataFrame:
    numeric_cols = [col for col in df.columns if col not in NUMERIC_EXCLUDE]
    if len(df) == 0:
        return pd.DataFrame(
            {
                "count": [0.0 for _ in numeric_cols],
                "mean": [0.0 for _ in numeric_cols],
                "std": [0.0 for _ in numeric_cols],
                "min": [0.0 for _ in numeric_cols],
                "25%": [0.0 for _ in numeric_cols],
                "50%": [0.0 for _ in numeric_cols],
                "75%": [0.0 for _ in numeric_cols],
                "max": [0.0 for _ in numeric_cols],
                "abs_max": [0.0 for _ in numeric_cols],
                "nonzero_pct": [0.0 for _ in numeric_cols],
            },
            index=numeric_cols,
        )
    summary = df[numeric_cols].describe().T
    summary["abs_max"] = df[numeric_cols].abs().max()
    summary["nonzero_pct"] = (df[numeric_cols].abs().gt(1e-6).sum() / len(df) * 100.0) if len(df) else 0.0
    return summary


def compute_axis_metrics(df: pd.DataFrame, hz: float | None) -> dict[str, Any]:
    result: dict[str, Any] = {"axes": {}, "field_status": {}}

    for col in df.columns:
        if col in NUMERIC_EXCLUDE:
            continue
        result["field_status"][col] = {
            "all_zero": bool(df[col].abs().max() <= 1e-6) if len(df) else True,
            "nonzero_pct": _nonzero_pct(df[col]) if len(df) else 0.0,
        }

    for axis_name, axis_info in AXIS_CHAINS.items():
        pair_metrics: list[PairMetrics] = []
        for stage, target_col, feedback_col in axis_info["series"]:
            if feedback_col is None:
                continue

            target = df[target_col]
            feedback = df[feedback_col]
            error = target - feedback
            lag_samples = _xcorr_lag(target.to_numpy(dtype=float), feedback.to_numpy(dtype=float))
            dom = _dominant_frequency(feedback.to_numpy(dtype=float), hz)
            sat_range = DEFAULT_SATURATION_HINTS.get(feedback_col)
            sat_hits = 0
            if sat_range is not None and len(feedback):
                lo, hi = sat_range
                sat_hits = int(((feedback <= lo + 1e-6) | (feedback >= hi - 1e-6)).sum())

            pair_metrics.append(
                PairMetrics(
                    stage=stage,
                    target_col=target_col,
                    feedback_col=feedback_col,
                    active=bool(_nonzero_pct(target) > 0.0 or _nonzero_pct(feedback) > 0.0),
                    count=int(len(df)),
                    target_nonzero_pct=_nonzero_pct(target),
                    feedback_nonzero_pct=_nonzero_pct(feedback),
                    error_mean=float(error.mean()) if len(error) else 0.0,
                    error_std=float(error.std(ddof=0)) if len(error) else 0.0,
                    error_mae=float(error.abs().mean()) if len(error) else 0.0,
                    error_rms=float(np.sqrt(np.mean(np.square(error)))) if len(error) else 0.0,
                    error_peak_abs=float(error.abs().max()) if len(error) else 0.0,
                    correlation=_safe_corr(target, feedback),
                    lag_samples=lag_samples,
                    lag_seconds=(lag_samples / hz) if (lag_samples is not None and hz) else None,
                    dominant_norm_freq=dom["dominant_norm_freq"],
                    dominant_hz=dom["dominant_hz"],
                    dominant_period_samples=dom["dominant_period_samples"],
                    dominant_period_s=dom["dominant_period_s"],
                    abnormal_segments=_find_abnormal_segments(error.abs()),
                    saturation_hits=sat_hits,
                    target_all_zero=bool(target.abs().max() <= 1e-6) if len(target) else True,
                    feedback_all_zero=bool(feedback.abs().max() <= 1e-6) if len(feedback) else True,
                )
            )

        cross_target = sum(df[col].abs() for col in axis_info["cross_axis_targets"])
        cross_feedback = sum(df[col].abs() for col in axis_info["cross_axis_feedbacks"])
        primary_target_col = pair_metrics[0].target_col if pair_metrics else None
        primary_target = df[primary_target_col] if primary_target_col else pd.Series(dtype=float)
        cross_corr = _safe_corr(primary_target.abs(), cross_feedback) if primary_target_col else None

        result["axes"][axis_name] = {
            "title": axis_info["title"],
            "pairs": pair_metrics,
            "cross_axis_target_activity": float(cross_target.mean()) if len(df) else 0.0,
            "cross_axis_feedback_activity": float(cross_feedback.mean()) if len(df) else 0.0,
            "cross_axis_correlation": cross_corr,
        }

    result["position_loop_active_pct"] = _nonzero_pct(df["position_loop_active"]) if "position_loop_active" in df else 0.0
    result["rows"] = int(len(df))
    result["hz"] = hz
    return result

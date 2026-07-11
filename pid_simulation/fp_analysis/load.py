from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import pandas as pd

from .schema import FP_COLUMNS


@dataclass
class LoadedFPLog:
    df: pd.DataFrame
    x_col: str
    input_path: Path
    hz: float | None


def load_fp_csv(path: Path, hz: float | None = None, segment: str = "all") -> LoadedFPLog:
    df = pd.read_csv(path)
    missing = [col for col in FP_COLUMNS if col not in df.columns]
    if missing:
        raise ValueError(f"Input is missing FP columns: {missing}")

    df = df.rename(columns=FP_COLUMNS)
    df.insert(0, "sample", range(len(df)))

    x_col = "sample"
    resolved_hz = hz if hz and hz > 0.0 else None
    if resolved_hz:
        df.insert(1, "time_s", df["sample"] / resolved_hz)
        x_col = "time_s"

    if segment != "all":
        active_value = 1 if segment == "active" else 0
        df = df[df["position_loop_active"] == active_value].copy()
        df.reset_index(drop=True, inplace=True)
        df["sample"] = range(len(df))
        if resolved_hz:
            df["time_s"] = df["sample"] / resolved_hz

    return LoadedFPLog(df=df, x_col=x_col, input_path=path, hz=resolved_hz)


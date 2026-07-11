from __future__ import annotations

from pathlib import Path
from typing import Any


def _fmt_num(value: float | None, digits: int = 3) -> str:
    if value is None:
        return "n/a"
    return f"{value:.{digits}f}"


def build_markdown_report(
    *,
    title: str,
    input_name: str,
    segment: str,
    metrics: dict[str, Any],
    diagnostics: dict[str, Any],
) -> str:
    lines: list[str] = [f"# {title}", ""]
    lines.append("## Basic Info")
    lines.append(f"- input: `{input_name}`")
    lines.append(f"- rows: `{metrics['rows']}`")
    lines.append(f"- segment: `{segment}`")
    lines.append(f"- time axis: `{'time_s' if metrics['hz'] else 'sample'}`")
    lines.append(f"- sample rate: `{metrics['hz'] if metrics['hz'] else 'not provided'}`")
    lines.append("")

    lines.append("## Sample Validity")
    field_status = metrics["field_status"]
    inactive_fields = [name for name, status in field_status.items() if status["all_zero"]]
    if diagnostics["general"]:
        for item in diagnostics["general"]:
            lines.append(f"- {item}")
    else:
        lines.append("- No global validity issues found.")
    lines.append(f"- all-zero fields: `{', '.join(inactive_fields) if inactive_fields else 'none'}`")
    lines.append("")

    for axis_name, axis_data in metrics["axes"].items():
        lines.append(f"## {axis_data['title']} Diagnostics")
        for finding in diagnostics["axes"].get(axis_name, []):
            lines.append(f"- {finding}")
        lines.append("")
        lines.append("| Stage | MAE | RMS | Peak | Corr | Lag(samples) | Lag(s) | Dom Freq | Dom Period |")
        lines.append("|---|---:|---:|---:|---:|---:|---:|---:|---:|")
        for pair in axis_data["pairs"]:
            dom_freq = _fmt_num(pair.dominant_hz) if metrics["hz"] else _fmt_num(pair.dominant_norm_freq)
            dom_period = _fmt_num(pair.dominant_period_s) if metrics["hz"] else _fmt_num(pair.dominant_period_samples)
            lines.append(
                f"| {pair.stage} | {_fmt_num(pair.error_mae)} | {_fmt_num(pair.error_rms)} | {_fmt_num(pair.error_peak_abs)} | "
                f"{_fmt_num(pair.correlation)} | {pair.lag_samples if pair.lag_samples is not None else 'n/a'} | "
                f"{_fmt_num(pair.lag_seconds)} | {dom_freq} | {dom_period} |"
            )
        lines.append("")

    lines.append("## Abnormal Segment Summary")
    any_segments = False
    for axis_name, axis_data in metrics["axes"].items():
        for pair in axis_data["pairs"]:
            if pair.abnormal_segments:
                any_segments = True
                ranges = ", ".join(f"[{start}:{end}]" for start, end in pair.abnormal_segments)
                lines.append(f"- {axis_data['title']} {pair.stage}: {ranges}")
    if not any_segments:
        lines.append("- No strong abnormal segments detected by the current rule thresholds.")
    lines.append("")

    lines.append("## Suggested Additional Fields")
    for item in diagnostics["recommendations"]:
        lines.append(f"- {item}")
    lines.append("")
    return "\n".join(lines)


def write_markdown_report(path: Path, content: str) -> None:
    path.write_text(content, encoding="utf-8")


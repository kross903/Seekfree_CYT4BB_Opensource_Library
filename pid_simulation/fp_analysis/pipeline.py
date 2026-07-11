from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from .load import load_fp_csv
from .metrics import compute_axis_metrics, compute_summary
from .plots import plot_axis_chain, plot_axis_errors, plot_axis_spectrum, plot_overview
from .report import build_markdown_report, write_markdown_report
from .rules import build_diagnosis


@dataclass
class FPAnalysisOptions:
    csv: Path
    out_dir: Path
    hz: float | None
    title: str
    no_plots: bool
    report_only: bool
    segment: str
    report_format: str


def run_fp_analysis(options: FPAnalysisOptions) -> dict[str, str]:
    loaded = load_fp_csv(options.csv, hz=options.hz, segment=options.segment)
    df = loaded.df
    options.out_dir.mkdir(parents=True, exist_ok=True)

    readable_csv = options.out_dir / "FP_readable.csv"
    summary_csv = options.out_dir / "FP_summary.csv"
    report_md = options.out_dir / "FP_report.md"

    if not options.report_only:
        df.to_csv(readable_csv, index=False, encoding="utf-8-sig")
        compute_summary(df).to_csv(summary_csv, encoding="utf-8-sig")

    metrics = compute_axis_metrics(df, loaded.hz)
    diagnostics = build_diagnosis(metrics)

    report_content = build_markdown_report(
        title=options.title,
        input_name=options.csv.name,
        segment=options.segment,
        metrics=metrics,
        diagnostics=diagnostics,
    )
    write_markdown_report(report_md, report_content)

    if not options.no_plots:
        plot_overview(df, loaded.x_col, options.out_dir)
        for axis_name in ("x", "y"):
            plot_axis_chain(df, loaded.x_col, options.out_dir, axis_name)
            plot_axis_errors(df, loaded.x_col, options.out_dir, axis_name)
            plot_axis_spectrum(df, options.out_dir, axis_name, loaded.hz)

    return {
        "rows": str(len(df)),
        "readable_csv": str(readable_csv),
        "summary_csv": str(summary_csv),
        "report_md": str(report_md),
        "out_dir": str(options.out_dir),
    }


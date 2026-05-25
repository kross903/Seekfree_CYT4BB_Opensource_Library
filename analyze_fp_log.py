#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

from fp_analysis.pipeline import FPAnalysisOptions, run_fp_analysis


def main() -> int:
    parser = argparse.ArgumentParser(description="Analyze FP! VOFA 16-channel logs.")
    parser.add_argument("csv", nargs="?", default="FP.csv", help="Input FP CSV path.")
    parser.add_argument("--out-dir", default=None, help="Output directory. Default: <csv_dir>/FP_processed")
    parser.add_argument("--hz", type=float, default=0.0, help="Optional sample rate in Hz.")
    parser.add_argument("--title", default="FP Log Diagnostic Report", help="Markdown report title.")
    parser.add_argument("--no-plots", action="store_true", help="Skip PNG plot generation.")
    parser.add_argument("--report-only", action="store_true", help="Only generate the Markdown report.")
    parser.add_argument("--segment", choices=("active", "inactive", "all"), default="all", help="Analyze only selected position loop segments.")
    parser.add_argument("--format", choices=("md",), default="md", help="Report format.")
    args = parser.parse_args()

    input_path = Path(args.csv)
    out_dir = Path(args.out_dir) if args.out_dir else input_path.with_name("FP_processed")
    result = run_fp_analysis(
        FPAnalysisOptions(
            csv=input_path,
            out_dir=out_dir,
            hz=args.hz if args.hz > 0.0 else None,
            title=args.title,
            no_plots=args.no_plots,
            report_only=args.report_only,
            segment=args.segment,
            report_format=args.format,
        )
    )

    print(f"rows: {result['rows']}")
    print(f"readable csv: {result['readable_csv']}")
    print(f"summary csv: {result['summary_csv']}")
    print(f"report md: {result['report_md']}")
    print(f"outputs: {result['out_dir']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

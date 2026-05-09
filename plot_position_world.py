import csv
import math
import sys
from pathlib import Path


def read_csv_rows(path):
    rows = []
    with path.open("r", encoding="utf-8-sig", newline="") as file:
        reader = csv.reader(file)
        next(reader, None)
        for row in reader:
            if len(row) < 8:
                continue
            try:
                rows.append([float(value) for value in row[:8]])
            except ValueError:
                continue
    return rows


def nice_ticks(vmin, vmax, count=6):
    if vmax <= vmin:
        return [vmin]
    raw_step = (vmax - vmin) / max(count - 1, 1)
    magnitude = 10 ** math.floor(math.log10(raw_step))
    normalized = raw_step / magnitude
    if normalized <= 1:
        step = 1 * magnitude
    elif normalized <= 2:
        step = 2 * magnitude
    elif normalized <= 5:
        step = 5 * magnitude
    else:
        step = 10 * magnitude
    start = math.floor(vmin / step) * step
    ticks = []
    value = start
    while value <= vmax + step * 0.5:
        if value >= vmin - step * 0.5:
            ticks.append(value)
        value += step
    return ticks


def escape_xml(text):
    return str(text).replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def world_stats(xs, ys, valid_flags):
    valid_idx = [i for i, ok in enumerate(valid_flags) if ok]
    invalid_idx = [i for i, ok in enumerate(valid_flags) if not ok]
    xmin, xmax = min(xs), max(xs)
    ymin, ymax = min(ys), max(ys)
    path_len = sum(math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1]) for i in range(1, len(xs)))
    valid_ratio = 100.0 * len(valid_idx) / len(xs)
    summary = [
        f"samples: {len(xs)}",
        f"valid: {len(valid_idx)} ({valid_ratio:.1f}%)",
        f"x range: {xmin:.1f} to {xmax:.1f} cm",
        f"y range: {ymin:.1f} to {ymax:.1f} cm",
        f"path length: {path_len:.1f} cm",
        f"end: ({xs[-1]:.1f}, {ys[-1]:.1f}) cm",
    ]
    return {
        "xmin": xmin,
        "xmax": xmax,
        "ymin": ymin,
        "ymax": ymax,
        "path_len": path_len,
        "valid_idx": valid_idx,
        "invalid_idx": invalid_idx,
        "summary": summary,
    }


def build_single_svg(xs, ys, valid_flags, title, out_path):
    stats = world_stats(xs, ys, valid_flags)
    width, height = 1000, 1000
    margin = 95
    plot_width = width - 2 * margin
    plot_height = height - 2 * margin
    x_range = stats["xmax"] - stats["xmin"] if stats["xmax"] > stats["xmin"] else 1.0
    y_range = stats["ymax"] - stats["ymin"] if stats["ymax"] > stats["ymin"] else 1.0
    scale = min(plot_width / x_range, plot_height / y_range)
    used_width = x_range * scale
    used_height = y_range * scale
    x_offset = margin + (plot_width - used_width) / 2.0
    y_offset = margin + (plot_height - used_height) / 2.0

    def sx(v):
        return x_offset + (v - stats["xmin"]) * scale

    def sy(v):
        return y_offset + used_height - (v - stats["ymin"]) * scale

    points = " ".join(f"{sx(xs[i]):.2f},{sy(ys[i]):.2f}" for i in range(len(xs)))
    svg = []
    svg.append(f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">')
    svg.append('<rect width="100%" height="100%" fill="#f7f5ef"/>')
    svg.append(f'<text x="500" y="38" text-anchor="middle" font-family="Arial, sans-serif" font-size="24" font-weight="700" fill="#202020">{escape_xml(title)}</text>')
    svg.append(f'<rect x="{margin}" y="{margin}" width="{plot_width}" height="{plot_height}" fill="#ffffff" stroke="#d0c8b8" stroke-width="1.2"/>')

    for tick in nice_ticks(stats["xmin"], stats["xmax"]):
        px = sx(tick)
        svg.append(f'<line x1="{px:.2f}" y1="{margin}" x2="{px:.2f}" y2="{height - margin}" stroke="#d9d9d9" stroke-dasharray="5,5" stroke-width="1"/>')
        svg.append(f'<text x="{px:.2f}" y="{height - margin + 24}" text-anchor="middle" font-family="Arial" font-size="12" fill="#555">{tick:.0f}</text>')

    for tick in nice_ticks(stats["ymin"], stats["ymax"]):
        py = sy(tick)
        svg.append(f'<line x1="{margin}" y1="{py:.2f}" x2="{width - margin}" y2="{py:.2f}" stroke="#d9d9d9" stroke-dasharray="5,5" stroke-width="1"/>')
        svg.append(f'<text x="{margin - 12}" y="{py + 4:.2f}" text-anchor="end" font-family="Arial" font-size="12" fill="#555">{tick:.0f}</text>')

    svg.append(f'<polyline points="{points}" fill="none" stroke="#255fbd" stroke-width="3" stroke-linejoin="round" stroke-linecap="round" opacity="0.9"/>')

    for index in stats["invalid_idx"]:
        svg.append(f'<circle cx="{sx(xs[index]):.2f}" cy="{sy(ys[index]):.2f}" r="4" fill="#888888" opacity="0.75"/>')
    step = max(1, len(stats["valid_idx"]) // 180)
    for index in stats["valid_idx"][::step]:
        svg.append(f'<circle cx="{sx(xs[index]):.2f}" cy="{sy(ys[index]):.2f}" r="2.3" fill="#1f9d55" opacity="0.45"/>')

    svg.append(f'<circle cx="{sx(xs[0]):.2f}" cy="{sy(ys[0]):.2f}" r="8" fill="#18a999" stroke="#111" stroke-width="1.5"/>')
    svg.append(f'<text x="{sx(xs[0]) + 12:.2f}" y="{sy(ys[0]) - 10:.2f}" font-family="Arial" font-size="14" font-weight="700" fill="#222">start</text>')
    svg.append(f'<circle cx="{sx(xs[-1]):.2f}" cy="{sy(ys[-1]):.2f}" r="9" fill="#e4572e" stroke="#111" stroke-width="1.5"/>')
    svg.append(f'<text x="{sx(xs[-1]) + 12:.2f}" y="{sy(ys[-1]) - 10:.2f}" font-family="Arial" font-size="14" font-weight="700" fill="#222">end</text>')

    svg.append(f'<text x="500" y="{height - 22}" text-anchor="middle" font-family="Arial" font-size="16" fill="#222">X world position (cm)</text>')
    svg.append('<text transform="translate(28,500) rotate(-90)" text-anchor="middle" font-family="Arial" font-size="16" fill="#222">Y world position (cm)</text>')

    box_x, box_y, line_height = 112, 112, 22
    svg.append(f'<rect x="{box_x}" y="{box_y}" width="280" height="{len(stats["summary"]) * line_height + 24}" rx="8" fill="#ffffff" stroke="#c8c0b0" opacity="0.94"/>')
    for index, line in enumerate(stats["summary"]):
        svg.append(f'<text x="{box_x + 14}" y="{box_y + 26 + index * line_height}" font-family="Arial" font-size="14" fill="#222">{escape_xml(line)}</text>')

    legend_x, legend_y = width - 310, height - 150
    svg.append(f'<rect x="{legend_x}" y="{legend_y}" width="215" height="80" rx="8" fill="#ffffff" stroke="#c8c0b0" opacity="0.94"/>')
    svg.append(f'<line x1="{legend_x + 18}" y1="{legend_y + 24}" x2="{legend_x + 58}" y2="{legend_y + 24}" stroke="#255fbd" stroke-width="3"/>')
    svg.append(f'<text x="{legend_x + 70}" y="{legend_y + 29}" font-family="Arial" font-size="13" fill="#222">trajectory</text>')
    svg.append(f'<circle cx="{legend_x + 38}" cy="{legend_y + 48}" r="4" fill="#1f9d55" opacity="0.6"/>')
    svg.append(f'<text x="{legend_x + 70}" y="{legend_y + 53}" font-family="Arial" font-size="13" fill="#222">valid sample</text>')
    svg.append(f'<circle cx="{legend_x + 38}" cy="{legend_y + 68}" r="4" fill="#888888" opacity="0.75"/>')
    svg.append(f'<text x="{legend_x + 70}" y="{legend_y + 73}" font-family="Arial" font-size="13" fill="#222">invalid / gated</text>')
    svg.append("</svg>")

    out_path.write_text("\n".join(svg), encoding="utf-8")
    return stats["summary"]


def build_compare_svg(legacy_x, legacy_y, flow_x, flow_y, valid_flags, title, out_path):
    all_x = legacy_x + flow_x
    all_y = legacy_y + flow_y
    xmin, xmax = min(all_x), max(all_x)
    ymin, ymax = min(all_y), max(all_y)
    width, height = 1000, 1000
    margin = 95
    plot_width = width - 2 * margin
    plot_height = height - 2 * margin
    x_range = xmax - xmin if xmax > xmin else 1.0
    y_range = ymax - ymin if ymax > ymin else 1.0
    scale = min(plot_width / x_range, plot_height / y_range)
    used_width = x_range * scale
    used_height = y_range * scale
    x_offset = margin + (plot_width - used_width) / 2.0
    y_offset = margin + (plot_height - used_height) / 2.0

    def sx(v):
        return x_offset + (v - xmin) * scale

    def sy(v):
        return y_offset + used_height - (v - ymin) * scale

    legacy_points = " ".join(f"{sx(legacy_x[i]):.2f},{sy(legacy_y[i]):.2f}" for i in range(len(legacy_x)))
    flow_points = " ".join(f"{sx(flow_x[i]):.2f},{sy(flow_y[i]):.2f}" for i in range(len(flow_x)))

    valid_idx = [i for i, ok in enumerate(valid_flags) if ok]
    invalid_idx = [i for i, ok in enumerate(valid_flags) if not ok]
    summary = [
        f"samples: {len(legacy_x)}",
        f"valid: {len(valid_idx)} ({100.0 * len(valid_idx) / len(valid_flags):.1f}%)",
        f"legacy end: ({legacy_x[-1]:.1f}, {legacy_y[-1]:.1f}) cm",
        f"flow end: ({flow_x[-1]:.1f}, {flow_y[-1]:.1f}) cm",
    ]

    svg = []
    svg.append(f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">')
    svg.append('<rect width="100%" height="100%" fill="#f7f5ef"/>')
    svg.append(f'<text x="500" y="38" text-anchor="middle" font-family="Arial, sans-serif" font-size="24" font-weight="700" fill="#202020">{escape_xml(title)}</text>')
    svg.append(f'<rect x="{margin}" y="{margin}" width="{plot_width}" height="{plot_height}" fill="#ffffff" stroke="#d0c8b8" stroke-width="1.2"/>')

    for tick in nice_ticks(xmin, xmax):
        px = sx(tick)
        svg.append(f'<line x1="{px:.2f}" y1="{margin}" x2="{px:.2f}" y2="{height - margin}" stroke="#d9d9d9" stroke-dasharray="5,5" stroke-width="1"/>')
        svg.append(f'<text x="{px:.2f}" y="{height - margin + 24}" text-anchor="middle" font-family="Arial" font-size="12" fill="#555">{tick:.0f}</text>')
    for tick in nice_ticks(ymin, ymax):
        py = sy(tick)
        svg.append(f'<line x1="{margin}" y1="{py:.2f}" x2="{width - margin}" y2="{py:.2f}" stroke="#d9d9d9" stroke-dasharray="5,5" stroke-width="1"/>')
        svg.append(f'<text x="{margin - 12}" y="{py + 4:.2f}" text-anchor="end" font-family="Arial" font-size="12" fill="#555">{tick:.0f}</text>')

    svg.append(f'<polyline points="{legacy_points}" fill="none" stroke="#255fbd" stroke-width="3" stroke-linejoin="round" stroke-linecap="round" opacity="0.9"/>')
    svg.append(f'<polyline points="{flow_points}" fill="none" stroke="#d83b7d" stroke-width="2.4" stroke-linejoin="round" stroke-linecap="round" opacity="0.9"/>')

    step = max(1, len(valid_idx) // 180)
    for index in valid_idx[::step]:
        svg.append(f'<circle cx="{sx(legacy_x[index]):.2f}" cy="{sy(legacy_y[index]):.2f}" r="2.2" fill="#1f9d55" opacity="0.35"/>')
        svg.append(f'<circle cx="{sx(flow_x[index]):.2f}" cy="{sy(flow_y[index]):.2f}" r="2.2" fill="#d83b7d" opacity="0.25"/>')
    for index in invalid_idx:
        svg.append(f'<circle cx="{sx(legacy_x[index]):.2f}" cy="{sy(legacy_y[index]):.2f}" r="3.4" fill="#888888" opacity="0.6"/>')

    svg.append(f'<circle cx="{sx(legacy_x[0]):.2f}" cy="{sy(legacy_y[0]):.2f}" r="8" fill="#18a999" stroke="#111" stroke-width="1.5"/>')
    svg.append(f'<circle cx="{sx(flow_x[-1]):.2f}" cy="{sy(flow_y[-1]):.2f}" r="9" fill="#e4572e" stroke="#111" stroke-width="1.5"/>')

    svg.append(f'<text x="500" y="{height - 22}" text-anchor="middle" font-family="Arial" font-size="16" fill="#222">X world position (cm)</text>')
    svg.append('<text transform="translate(28,500) rotate(-90)" text-anchor="middle" font-family="Arial" font-size="16" fill="#222">Y world position (cm)</text>')

    box_x, box_y, line_height = 112, 112, 22
    svg.append(f'<rect x="{box_x}" y="{box_y}" width="330" height="{len(summary) * line_height + 24}" rx="8" fill="#ffffff" stroke="#c8c0b0" opacity="0.94"/>')
    for index, line in enumerate(summary):
        svg.append(f'<text x="{box_x + 14}" y="{box_y + 26 + index * line_height}" font-family="Arial" font-size="14" fill="#222">{escape_xml(line)}</text>')

    legend_x, legend_y = width - 320, height - 160
    svg.append(f'<rect x="{legend_x}" y="{legend_y}" width="225" height="92" rx="8" fill="#ffffff" stroke="#c8c0b0" opacity="0.94"/>')
    svg.append(f'<line x1="{legend_x + 18}" y1="{legend_y + 24}" x2="{legend_x + 58}" y2="{legend_y + 24}" stroke="#255fbd" stroke-width="3"/>')
    svg.append(f'<text x="{legend_x + 70}" y="{legend_y + 29}" font-family="Arial" font-size="13" fill="#222">legacy position</text>')
    svg.append(f'<line x1="{legend_x + 18}" y1="{legend_y + 44}" x2="{legend_x + 58}" y2="{legend_y + 44}" stroke="#d83b7d" stroke-width="2.4"/>')
    svg.append(f'<text x="{legend_x + 70}" y="{legend_y + 49}" font-family="Arial" font-size="13" fill="#222">flow position</text>')
    svg.append(f'<circle cx="{legend_x + 38}" cy="{legend_y + 68}" r="4" fill="#1f9d55" opacity="0.6"/>')
    svg.append(f'<text x="{legend_x + 70}" y="{legend_y + 73}" font-family="Arial" font-size="13" fill="#222">valid sample</text>')
    svg.append("</svg>")

    out_path.write_text("\n".join(svg), encoding="utf-8")
    return summary


def main():
    input_path = Path(sys.argv[1]) if len(sys.argv) >= 2 else Path("position.csv")
    output_path = Path(sys.argv[2]) if len(sys.argv) >= 3 else input_path.with_name("position_world_plot.svg")
    rows = read_csv_rows(input_path)
    if not rows:
        raise SystemExit(f"No valid rows found in {input_path}")

    stem = input_path.stem.lower()
    if stem == "positionc":
        legacy_x = [row[0] for row in rows]
        legacy_y = [row[1] for row in rows]
        flow_x = [row[2] for row in rows]
        flow_y = [row[3] for row in rows]
        valid_flags = [abs(row[7] - 1.0) < 1e-5 for row in rows]
        summary = build_compare_svg(
            legacy_x, legacy_y, flow_x, flow_y, valid_flags,
            "World-frame XY comparison from FC data",
            output_path,
        )
    else:
        xs = [row[5] for row in rows]
        ys = [row[6] for row in rows]
        valid_flags = [abs(row[7] - 1.0) < 1e-5 for row in rows]
        summary = build_single_svg(
            xs, ys, valid_flags,
            "World-frame XY trajectory from FB data",
            output_path,
        )

    print(output_path.resolve())
    for line in summary:
        print(line)


if __name__ == "__main__":
    main()

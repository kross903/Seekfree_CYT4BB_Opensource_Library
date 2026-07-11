from __future__ import annotations

from typing import Any


def _fmt_pair(stage: str, msg: str) -> str:
    return f"{stage}: {msg}"


def build_diagnosis(metrics: dict[str, Any]) -> dict[str, Any]:
    diagnostics: dict[str, Any] = {"general": [], "axes": {}, "missing_fields": []}

    if metrics["rows"] == 0:
        diagnostics["general"].append("筛选后的日志为空，无法分析该分段。")
        diagnostics["recommendations"] = [
            "检查 `--segment` 选择是否与当前日志匹配。",
            "若要分析位置环区段，请重新采集 `position_loop_active=1` 的日志。",
        ]
        return diagnostics

    if metrics["position_loop_active_pct"] <= 0.0:
        diagnostics["general"].append("位置环未参与：`position_loop_active` 全程为 0，本次日志不支持位置环闭环性能判断。")

    for axis_name, axis_data in metrics["axes"].items():
        findings: list[str] = []
        pairs = {pair.stage: pair for pair in axis_data["pairs"]}
        velocity = pairs.get("velocity")
        angle = pairs.get("angle")
        rate = pairs.get("rate")

        if rate:
            if rate.saturation_hits > 0:
                findings.append(_fmt_pair("角速度环", f"反馈出现 {rate.saturation_hits} 次接近饱和值/极限值。"))
            if rate.target_all_zero and rate.feedback_nonzero_pct > 20.0 and rate.error_rms > 5.0:
                findings.append(_fmt_pair("角速度环", "目标基本为 0，但反馈持续抖动，存在静态抖动或零偏嫌疑。"))
            if rate.error_rms > 15.0 or (rate.correlation is not None and rate.correlation < 0.5):
                findings.append(_fmt_pair("角速度环", "跟踪较差，优先检查速率环 PID、滤波和 IMU 噪声。"))

        if angle:
            if angle.saturation_hits > 0:
                findings.append(_fmt_pair("角度环", f"反馈出现 {angle.saturation_hits} 次接近姿态极限值。"))
            if rate and (rate.error_rms <= 8.0 and angle.error_rms > 5.0):
                findings.append(_fmt_pair("角度环", "角度环跟踪差，但速率环相对正常，优先检查姿态环参数或目标角映射。"))
            if angle.target_all_zero and angle.feedback_nonzero_pct > 20.0 and angle.error_rms > 3.0:
                findings.append(_fmt_pair("角度环", "目标基本为 0，但姿态反馈明显摆动，存在配平、估计或耦合问题。"))

        if velocity:
            if velocity.target_all_zero and velocity.feedback_nonzero_pct > 20.0 and velocity.error_rms > 8.0:
                findings.append(_fmt_pair("速度环", "目标基本为 0，但速度反馈持续波动，优先检查光流/融合速度估计。"))
            if angle and rate and rate.error_rms <= 8.0 and angle.error_rms <= 4.0 and velocity.error_rms > 10.0:
                findings.append(_fmt_pair("速度环", "速度环跟踪差，但内环基本正常，优先检查速度环参数或速度反馈链。"))

        if axis_data["cross_axis_correlation"] is not None and axis_data["cross_axis_correlation"] > 0.6:
            findings.append("双轴耦合嫌疑：本轴目标活动与另一轴反馈活动相关性较高。")

        abnormal_ranges = []
        for pair in axis_data["pairs"]:
            for start, end in pair.abnormal_segments:
                abnormal_ranges.append(f"{pair.stage}[{start}:{end}]")
        if abnormal_ranges:
            findings.append(f"异常段摘要：{', '.join(abnormal_ranges[:8])}")

        if not findings:
            findings.append("未发现显著规则异常；建议结合图表人工复核。")

        diagnostics["axes"][axis_name] = findings

    diagnostics["recommendations"] = [
        "补采位置目标 `x/y setpoint`，否则无法严格判断位置环误差。",
        "补采最终执行器输出或 PWM/油门，用于判断限幅与混控问题。",
        "补采限幅标志、视觉有效标志和实际发送频率，提升自动诊断准确性。",
    ]
    return diagnostics

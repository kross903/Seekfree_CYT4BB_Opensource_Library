from __future__ import annotations

import csv
import shutil
import sys
from dataclasses import asdict, dataclass
from enum import Enum
from pathlib import Path
from typing import Dict, List, Tuple

import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation

from .config import SimulationConfig
from .firmware_controller import FirmwarePIDController


class SimExitStatus(Enum):
    TIMEOUT = "Timeout: simulation reached t_final."


@dataclass
class SimulationOutput:
    rows: List[Dict[str, float]]
    rotorpy_result: Dict[str, Dict[str, np.ndarray]]


def ensure_rotorpy_path(cfg: SimulationConfig) -> None:
    root = Path(cfg.rotorpy_root)
    if not root.exists():
        raise FileNotFoundError(f"RotorPy root not found: {root}")
    root_str = str(root)
    if root_str not in sys.path:
        sys.path.insert(0, root_str)


def make_vehicle(cfg: SimulationConfig):
    ensure_rotorpy_path(cfg)
    from rotorpy.vehicles.hummingbird_params import quad_params
    from rotorpy.vehicles.multirotor import Multirotor

    initial_state = {
        "x": np.array([cfg.initial_x_cm / 100.0, cfg.initial_y_cm / 100.0, cfg.initial_height_cm / 100.0]),
        "v": np.zeros(3),
        "q": np.array([0.0, 0.0, 0.0, 1.0]),
        "w": np.zeros(3),
        "wind": np.zeros(3),
        "rotor_speeds": np.full(4, np.sqrt((quad_params["mass"] * 9.81) / (4.0 * quad_params["k_eta"]))),
    }
    vehicle = Multirotor(
        quad_params,
        initial_state=initial_state,
        control_abstraction="cmd_motor_speeds",
        aero=cfg.aero,
        enable_ground=cfg.enable_ground,
        integrator_kwargs={"method": "RK45", "rtol": 1e-5, "atol": 1e-7, "max_step": cfg.sim_dt_s},
    )
    return vehicle, quad_params


def make_wind_profile(cfg: SimulationConfig):
    ensure_rotorpy_path(cfg)
    profile = cfg.wind_profile.lower()
    from rotorpy.wind.default_winds import ConstantWind, NoWind, SinusoidWind

    if profile in {"none", "no", "off", "nowind"}:
        return NoWind()
    if profile == "constant":
        return ConstantWind(wx=1.0, wy=0.0, wz=0.0)
    if profile == "sinusoid":
        return SinusoidWind()
    raise ValueError(f"Unsupported wind_profile: {cfg.wind_profile}")


def _merge_dict_list(items: List[Dict[str, np.ndarray]]) -> Dict[str, np.ndarray]:
    if not items:
        return {}
    return {key: np.stack([np.asarray(item[key]) for item in items], axis=0) for key in items[0].keys()}


def _empty_flat_output(state: Dict[str, np.ndarray], cfg: SimulationConfig) -> Dict[str, np.ndarray | float]:
    return {
        "x": np.array([cfg.target_x_cm / 100.0, cfg.target_y_cm / 100.0, cfg.target_height_cm / 100.0]),
        "x_dot": np.zeros(3),
        "x_ddot": np.zeros(3),
        "x_dddot": np.zeros(3),
        "x_ddddot": np.zeros(3),
        "yaw": 0.0,
        "yaw_dot": 0.0,
    }


def _control_for_plot(control: Dict[str, np.ndarray], state: Dict[str, np.ndarray], quad_params: Dict[str, float]) -> Dict[str, np.ndarray | float]:
    cmd_motor_speeds = np.asarray(control["cmd_motor_speeds"], dtype=float)
    thrusts = quad_params["k_eta"] * cmd_motor_speeds * cmd_motor_speeds
    k = quad_params["k_m"] / quad_params["k_eta"]
    rotor_pos = quad_params["rotor_pos"]
    rotor_dir = quad_params["rotor_directions"]
    f_to_tm = np.vstack(
        (
            np.ones((1, quad_params["num_rotors"])),
            np.hstack([np.cross(rotor_pos[key], np.array([0, 0, 1])).reshape(-1, 1)[0:2] for key in rotor_pos]),
            (k * rotor_dir).reshape(1, -1),
        )
    )
    tm = f_to_tm @ thrusts
    return {
        "cmd_motor_speeds": cmd_motor_speeds,
        "cmd_motor_thrusts": thrusts,
        "cmd_thrust": float(tm[0]),
        "cmd_moment": np.array([tm[1], tm[2], tm[3]], dtype=float),
        "cmd_q": np.asarray(state["q"], dtype=float),
        "cmd_w": np.zeros(3),
        "cmd_v": np.zeros(3),
        "cmd_acc": np.zeros(3),
    }


def _fake_sensor_packets(state: Dict[str, np.ndarray], statedot: Dict[str, np.ndarray]) -> Tuple[Dict[str, np.ndarray], Dict[str, np.ndarray], Dict[str, np.ndarray], Dict[str, np.ndarray]]:
    accel = np.asarray(statedot.get("vdot", np.zeros(3)), dtype=float)
    gyro = np.asarray(state["w"], dtype=float)
    imu = {"accel": accel, "gyro": gyro}
    imu_gt = {"accel": accel.copy()}
    mocap = {
        "x": np.asarray(state["x"], dtype=float),
        "v": np.asarray(state["v"], dtype=float),
        "q": np.asarray(state["q"], dtype=float),
        "w": np.asarray(state["w"], dtype=float),
    }
    state_estimate = {
        "filter_state": np.zeros(0),
        "covariance": np.zeros((0, 0)),
    }
    return imu, imu_gt, mocap, state_estimate


def run_simulation_with_result(cfg: SimulationConfig) -> SimulationOutput:
    vehicle, quad_params = make_vehicle(cfg)
    wind_profile = make_wind_profile(cfg)
    controller = FirmwarePIDController(cfg, quad_params)
    state = {k: np.array(v, dtype=float) for k, v in vehicle.initial_state.items()}
    rows: List[Dict[str, float]] = []
    times: List[float] = []
    states: List[Dict[str, np.ndarray]] = []
    controls: List[Dict[str, np.ndarray]] = []
    flats: List[Dict[str, np.ndarray | float]] = []
    imu_packets: List[Dict[str, np.ndarray]] = []
    imu_gt_packets: List[Dict[str, np.ndarray]] = []
    mocap_packets: List[Dict[str, np.ndarray]] = []
    estimate_packets: List[Dict[str, np.ndarray]] = []
    steps = int(round(cfg.duration_s * cfg.sim_rate_hz))

    for step in range(steps + 1):
        t = step * cfg.sim_dt_s
        state["wind"] = np.asarray(wind_profile.update(t, state["x"]), dtype=float)
        control = controller.update(t, state)
        statedot = vehicle.statedot(state, control, cfg.sim_dt_s)
        diag = asdict(controller.last_diag)
        diag.update(
            {
                "state_x_m": float(state["x"][0]),
                "state_y_m": float(state["x"][1]),
                "state_z_m": float(state["x"][2]),
                "state_vx_mps": float(state["v"][0]),
                "state_vy_mps": float(state["v"][1]),
                "state_vz_mps": float(state["v"][2]),
            }
        )
        rows.append(diag)
        times.append(t)
        states.append({k: np.asarray(v, dtype=float).copy() for k, v in state.items()})
        controls.append(_control_for_plot(control, state, quad_params))
        flats.append(_empty_flat_output(state, cfg))
        imu, imu_gt, mocap, state_estimate = _fake_sensor_packets(state, statedot)
        imu_packets.append(imu)
        imu_gt_packets.append(imu_gt)
        mocap_packets.append(mocap)
        estimate_packets.append(state_estimate)
        if step < steps:
            state = vehicle.step(state, control, cfg.sim_dt_s)

    result = {
        "time": np.asarray(times),
        "state": _merge_dict_list(states),
        "control": _merge_dict_list(controls),
        "flat": _merge_dict_list(flats),
        "imu_measurements": _merge_dict_list(imu_packets),
        "imu_gt": _merge_dict_list(imu_gt_packets),
        "mocap_measurements": _merge_dict_list(mocap_packets),
        "state_estimate": _merge_dict_list(estimate_packets),
        "exit": SimExitStatus.TIMEOUT,
    }
    return SimulationOutput(rows=rows, rotorpy_result=result)


def run_simulation(cfg: SimulationConfig) -> List[Dict[str, float]]:
    return run_simulation_with_result(cfg).rows


def save_csv(rows: List[Dict[str, float]], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        return
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def plot_results(rows: List[Dict[str, float]], path: Path | None = None, show: bool = False) -> None:
    if not rows:
        return
    t = np.array([r["t"] for r in rows])

    fig, axes = plt.subplots(5, 1, figsize=(12, 14), sharex=True)
    fig.suptitle("RotorPy firmware PID simulation")

    axes[0].plot([r["pos_x_cm"] for r in rows], [r["pos_y_cm"] for r in rows], label="xy path")
    axes[0].scatter([rows[0]["pos_x_cm"]], [rows[0]["pos_y_cm"]], label="start")
    axes[0].scatter([rows[-1]["pos_x_cm"]], [rows[-1]["pos_y_cm"]], label="end")
    axes[0].set_ylabel("Y cm")
    axes[0].set_xlabel("X cm")
    axes[0].axis("equal")
    axes[0].grid(True)
    axes[0].legend()

    axes[1].plot(t, [r["height_cm"] for r in rows], label="height")
    axes[1].plot(t, [r["target_height_cm"] for r in rows], label="target height")
    axes[1].plot(t, [r["height_pid_output"] for r in rows], label="height pid out")
    axes[1].set_ylabel("cm / output")
    axes[1].grid(True)
    axes[1].legend()

    axes[2].plot(t, [r["vx_cms"] for r in rows], label="vx")
    axes[2].plot(t, [r["vx_target_cms"] for r in rows], label="vx target")
    axes[2].plot(t, [r["vy_cms"] for r in rows], label="vy")
    axes[2].plot(t, [r["vy_target_cms"] for r in rows], label="vy target")
    axes[2].set_ylabel("cm/s")
    axes[2].grid(True)
    axes[2].legend()

    axes[3].plot(t, [r["roll_deg"] for r in rows], label="roll")
    axes[3].plot(t, [r["target_roll_deg"] for r in rows], label="target roll")
    axes[3].plot(t, [r["pitch_deg"] for r in rows], label="pitch")
    axes[3].plot(t, [r["target_pitch_deg"] for r in rows], label="target pitch")
    axes[3].set_ylabel("deg")
    axes[3].grid(True)
    axes[3].legend()

    for idx in range(4):
        axes[4].plot(t, [r[f"motor{idx}"] for r in rows], label=f"motor{idx}")
    axes[4].set_ylabel("firmware throttle")
    axes[4].set_xlabel("time s")
    axes[4].grid(True)
    axes[4].legend()

    fig.tight_layout()
    if path is not None:
        path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(path, dpi=160)
    if show:
        plt.show()
    else:
        plt.close(fig)


def _empty_world(cfg: SimulationConfig):
    ensure_rotorpy_path(cfg)
    from rotorpy.world import World

    return World.empty((-4, 4, -4, 4, -1, 4))


def _rotorpy_data_out(cfg: SimulationConfig) -> Path:
    ensure_rotorpy_path(cfg)
    return Path(cfg.rotorpy_root) / "rotorpy" / "data_out"


def _copy_rotorpy_outputs_to_project(cfg: SimulationConfig, prefix: str) -> None:
    source_dir = _rotorpy_data_out(cfg)
    if not source_dir.exists():
        return
    cfg.output_dir.mkdir(parents=True, exist_ok=True)
    for source in source_dir.glob(f"{prefix}*"):
        if source.is_file():
            shutil.copy2(source, cfg.output_dir / source.name)


def show_rotorpy_plot(result: Dict[str, Dict[str, np.ndarray]], cfg: SimulationConfig, fname: str | None = None) -> None:
    ensure_rotorpy_path(cfg)
    from rotorpy.utils.plotter import Plotter

    Plotter(result, _empty_world(cfg)).plot_results(
        fname=fname,
        plot_mocap=False,
        plot_estimator=False,
        plot_imu=True,
    )
    if fname is not None:
        _copy_rotorpy_outputs_to_project(cfg, fname)


def show_rotorpy_animation(
    result: Dict[str, Dict[str, np.ndarray]],
    cfg: SimulationConfig,
    fname: str | None = None,
    animate_wind: bool = False,
) -> None:
    ensure_rotorpy_path(cfg)
    from rotorpy.utils.plotter import Plotter

    Plotter(result, _empty_world(cfg)).animate_results(fname=fname, animate_wind=animate_wind)
    if fname is not None:
        _copy_rotorpy_outputs_to_project(cfg, fname)

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Tuple


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_ROTORPY_ROOT = Path(r"D:\Aa-app\rotorpy-main")
OUTPUT_DIR = Path(__file__).resolve().parent / "outputs"


@dataclass
class PIDConfig:
    kp: float
    ki: float
    kd: float
    output_limit: float
    integral_limit: float
    integral_threshold: float
    alpha: float


def default_pid_configs() -> Dict[str, PIDConfig]:
    """Defaults copied from project/code/common/All_Init.c."""
    return {
        "gyro_roll": PIDConfig(2.0, 0.0, 0.0, 2000.0, 100.0, 10.0, 0.8),
        "gyro_pitch": PIDConfig(2.5, 0.0, 0.0, 2000.0, 100.0, 10.0, 0.8),
        "gyro_yaw": PIDConfig(5.5, 0.0, 0.0, 400.0, 10.0, 0.0, 0.0),
        "angle_roll": PIDConfig(8.5, 0.0001, 0.0, 300.0, 60000.0, 10000.0, 0.30),
        "angle_pitch": PIDConfig(7.5, 0.0001, 0.0, 300.0, 60000.0, 10000.0, 0.30),
        "angle_yaw": PIDConfig(3.0, 0.0, 0.0, 100.0, 20.0, 0.0, 0.0),
        "height": PIDConfig(2.0, 0.0001, 0.1, 80.0, 30.0, 10000.0, 0.3),
        "vel_x": PIDConfig(0.50, 0.0001, 0.0, 40.0, 20.0, 10000.0, 0.8),
        "vel_y": PIDConfig(0.50, 0.0001, 0.0, 40.0, 20.0, 10000.0, 0.8),
        "pos_x": PIDConfig(2.0, 0.001, 0.0, 100.0, 30.0, 10000.0, 1.0),
        "pos_y": PIDConfig(2.0, 0.001, 0.0, 100.0, 30.0, 10000.0, 1.0),
    }


@dataclass
class SimulationConfig:
    rotorpy_root: Path = DEFAULT_ROTORPY_ROOT
    output_dir: Path = OUTPUT_DIR
    duration_s: float = 8.0
    sim_rate_hz: int = 500
    control_rate_hz: int = 4000
    height_rate_hz: int = 100
    target_height_cm: float = 110.0
    target_x_cm: float = 0.0
    target_y_cm: float = 0.0
    initial_x_cm: float = 50.0
    initial_y_cm: float = 30.0
    initial_height_cm: float = 110.0
    target_yaw_deg: float = 0.0
    base_throttle: float = 770.0
    hover_roll_deg: float = -0.5
    hover_pitch_deg: float = 0.65
    rotorpy_pitch_axis_sign: float = -1.0
    rotorpy_roll_axis_sign: float = -1.0
    motor_output_scale_max: float = 2000.0
    safe_min_throttle: float = 300.0
    tilt_thrust_comp_max: float = 1.30
    throttle_max_effective_omega_ratio: float = 1.8
    # RotorPy order [r1, r2, r3, r4] = firmware motor indices [2, 0, 1, 3].
    firmware_to_rotorpy_order: Tuple[int, int, int, int] = (2, 0, 1, 3)
    position_deadzone_cm: float = 3.0
    position_noise_cm: float = 0.0
    position_dropout_prob: float = 0.0
    position_update_hz: int = 100
    velocity_ready: bool = True
    wind_profile: str = "sinusoid"
    aero: bool = False
    enable_ground: bool = False
    pid: Dict[str, PIDConfig] = field(default_factory=default_pid_configs)

    @property
    def sim_dt_s(self) -> float:
        return 1.0 / float(self.sim_rate_hz)

    @property
    def control_dt_s(self) -> float:
        return 1.0 / float(self.control_rate_hz)

    @property
    def control_ticks_per_step(self) -> int:
        return max(1, int(round(self.control_rate_hz / self.sim_rate_hz)))

    @property
    def height_tick_divider(self) -> int:
        return max(1, int(round(self.control_rate_hz / self.height_rate_hz)))

    @property
    def position_tick_divider(self) -> int:
        return max(1, int(round(self.control_rate_hz / self.position_update_hz)))

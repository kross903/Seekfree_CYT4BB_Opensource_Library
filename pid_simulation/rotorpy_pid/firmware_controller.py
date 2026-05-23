from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Tuple

import numpy as np
from scipy.spatial.transform import Rotation

from .config import SimulationConfig
from .pid_model import PIDController


DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi
GRAVITY_CMS2 = 981.0


@dataclass
class ControlDiag:
    t: float = 0.0
    pos_x_cm: float = 0.0
    pos_y_cm: float = 0.0
    target_x_cm: float = 0.0
    target_y_cm: float = 0.0
    pos_error_x_cm: float = 0.0
    pos_error_y_cm: float = 0.0
    vx_target_cms: float = 0.0
    vy_target_cms: float = 0.0
    vx_cms: float = 0.0
    vy_cms: float = 0.0
    ax_cmd_cms2: float = 0.0
    ay_cmd_cms2: float = 0.0
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0
    target_roll_deg: float = 0.0
    target_pitch_deg: float = 0.0
    target_yaw_deg: float = 0.0
    target_roll_rate_dps: float = 0.0
    target_pitch_rate_dps: float = 0.0
    target_yaw_rate_dps: float = 0.0
    gyro_roll_dps: float = 0.0
    gyro_pitch_dps: float = 0.0
    gyro_yaw_dps: float = 0.0
    roll_output: float = 0.0
    pitch_output: float = 0.0
    yaw_output: float = 0.0
    height_cm: float = 0.0
    target_height_cm: float = 0.0
    height_error_cm: float = 0.0
    height_pid_output: float = 0.0
    base_throttle: float = 0.0
    motor0: float = 0.0
    motor1: float = 0.0
    motor2: float = 0.0
    motor3: float = 0.0
    rotor0_rad_s: float = 0.0
    rotor1_rad_s: float = 0.0
    rotor2_rad_s: float = 0.0
    rotor3_rad_s: float = 0.0


class FirmwarePIDController:
    def __init__(self, cfg: SimulationConfig, quad_params: Dict[str, float]):
        self.cfg = cfg
        self.quad_params = quad_params
        self.pid = {name: PIDController.from_config(p) for name, p in cfg.pid.items()}

        self.height_pid_output = 0.0
        self.roll_output = 0.0
        self.pitch_output = 0.0
        self.yaw_output = 0.0
        self.target_roll_rate_lpf = 0.0
        self.target_pitch_rate_lpf = 0.0
        self.angle_lpf_initialized = False
        self.tick_count = 0
        self.last_position_obs = (cfg.initial_x_cm, cfg.initial_y_cm)
        self.last_diag = ControlDiag()

        self.hover_omega = math.sqrt(
            (quad_params["mass"] * 9.81) / (quad_params["num_rotors"] * quad_params["k_eta"])
        )
        max_omega = min(
            quad_params["rotor_speed_max"],
            self.hover_omega * self.cfg.throttle_max_effective_omega_ratio,
        )
        denom = max(1.0, cfg.motor_output_scale_max - cfg.base_throttle)
        self.omega_per_throttle = max(1e-6, (max_omega - self.hover_omega) / denom)

    def update(self, t: float, state: Dict[str, np.ndarray]) -> Dict[str, np.ndarray]:
        control = None
        for _ in range(self.cfg.control_ticks_per_step):
            control = self._update_one_control_tick(t, state)
            self.tick_count += 1
        assert control is not None
        return control

    def _state_feedback(self, state: Dict[str, np.ndarray]) -> Tuple[float, ...]:
        euler = Rotation.from_quat(state["q"]).as_euler("xyz", degrees=True)
        roll_deg, pitch_deg, yaw_deg = float(euler[0]), float(euler[1]), float(euler[2])
        gyro_roll_dps, gyro_pitch_dps, gyro_yaw_dps = (state["w"] * RAD2DEG).astype(float)
        # Firmware position loop consumes target/car position relative to the drone
        # (ir_pose_estimate.x/y), not the drone world position. RotorPy state['x']
        # is drone world position, so convert it to the same relative observation.
        pos_x_cm = float(self.cfg.target_x_cm - state["x"][0] * 100.0)
        pos_y_cm = float(self.cfg.target_y_cm - state["x"][1] * 100.0)
        height_cm = float(state["x"][2] * 100.0)
        vx_cms = float(state["v"][0] * 100.0)
        vy_cms = float(state["v"][1] * 100.0)
        return (
            roll_deg,
            pitch_deg,
            yaw_deg,
            gyro_roll_dps,
            gyro_pitch_dps,
            gyro_yaw_dps,
            pos_x_cm,
            pos_y_cm,
            height_cm,
            vx_cms,
            vy_cms,
        )

    def _position_observation(self, true_x_cm: float, true_y_cm: float) -> Tuple[bool, float, float]:
        if self.tick_count % self.cfg.position_tick_divider != 0:
            return True, self.last_position_obs[0], self.last_position_obs[1]

        if self.cfg.position_dropout_prob > 0.0 and np.random.random() < self.cfg.position_dropout_prob:
            return False, self.last_position_obs[0], self.last_position_obs[1]

        noise_x = np.random.normal(0.0, self.cfg.position_noise_cm) if self.cfg.position_noise_cm > 0 else 0.0
        noise_y = np.random.normal(0.0, self.cfg.position_noise_cm) if self.cfg.position_noise_cm > 0 else 0.0
        self.last_position_obs = (true_x_cm + noise_x, true_y_cm + noise_y)
        return True, self.last_position_obs[0], self.last_position_obs[1]

    def _apply_position_deadzone(self, value_cm: float) -> float:
        if -self.cfg.position_deadzone_cm <= value_cm <= self.cfg.position_deadzone_cm:
            return 0.0
        return value_cm

    def _update_position_loop(self, pos_x_cm: float, pos_y_cm: float, vx_cms: float, vy_cms: float) -> Tuple[float, float, float, float, float, float]:
        _ = vx_cms, vy_cms
        valid, obs_x_cm, obs_y_cm = self._position_observation(pos_x_cm, pos_y_cm)
        if not valid or not self.cfg.velocity_ready:
            self.pid["pos_x"].reset()
            self.pid["pos_y"].reset()
            return obs_x_cm, obs_y_cm, 0.0, 0.0, 0.0, 0.0

        pos_error_x = self._apply_position_deadzone(obs_x_cm - self.cfg.target_x_cm)
        pos_error_y = self._apply_position_deadzone(obs_y_cm - self.cfg.target_y_cm)

        if pos_error_x == 0.0:
            self.pid["pos_x"].reset()
        if pos_error_y == 0.0:
            self.pid["pos_y"].reset()

        vx_cmd = self.pid["pos_x"].calculate(pos_error_x, 0.0, self.cfg.control_dt_s)
        vy_cmd = self.pid["pos_y"].calculate(pos_error_y, 0.0, self.cfg.control_dt_s)
        return obs_x_cm, obs_y_cm, pos_error_x, pos_error_y, vx_cmd, vy_cmd

    def _update_height_loop(self, height_cm: float) -> float:
        if self.tick_count % self.cfg.height_tick_divider != 0:
            return self.height_pid_output

        error_cm = self.cfg.target_height_cm - height_cm
        error_cm = max(-40.0, min(40.0, error_cm))
        self.height_pid_output = self.pid["height"].calculate(height_cm + error_cm, height_cm, self.cfg.control_dt_s)
        return self.height_pid_output

    def _update_angle_rate_lpf(self, roll_rate_raw: float, pitch_rate_raw: float) -> Tuple[float, float]:
        alpha = 0.04
        if not self.angle_lpf_initialized:
            self.target_roll_rate_lpf = roll_rate_raw
            self.target_pitch_rate_lpf = pitch_rate_raw
            self.angle_lpf_initialized = True
        else:
            self.target_roll_rate_lpf += alpha * (roll_rate_raw - self.target_roll_rate_lpf)
            self.target_pitch_rate_lpf += alpha * (pitch_rate_raw - self.target_pitch_rate_lpf)
        return self.target_roll_rate_lpf, self.target_pitch_rate_lpf

    def _mix_motor_outputs(self, base_throttle: float, roll_deg: float, pitch_deg: float) -> List[float]:
        tilt_cos = math.cos(roll_deg * DEG2RAD) * math.cos(pitch_deg * DEG2RAD)
        tilt_cos = max(1.0 / self.cfg.tilt_thrust_comp_max, tilt_cos)
        thrust_base_comp = (base_throttle + self.height_pid_output) / tilt_cos

        motor = [0.0, 0.0, 0.0, 0.0]
        motor[3] = thrust_base_comp + self.roll_output + self.pitch_output - self.yaw_output
        motor[2] = thrust_base_comp + self.roll_output - self.pitch_output + self.yaw_output
        motor[1] = thrust_base_comp - self.roll_output + self.pitch_output + self.yaw_output
        motor[0] = thrust_base_comp - self.roll_output - self.pitch_output - self.yaw_output

        limited = []
        for value in motor:
            if thrust_base_comp >= self.cfg.safe_min_throttle and value < self.cfg.safe_min_throttle:
                value = self.cfg.safe_min_throttle
            value = max(0.0, min(self.cfg.motor_output_scale_max, value))
            limited.append(value)
        return limited

    def _throttle_to_omega(self, throttle: float) -> float:
        omega = self.hover_omega + (throttle - self.cfg.base_throttle) * self.omega_per_throttle
        return max(self.quad_params["rotor_speed_min"], min(self.quad_params["rotor_speed_max"], omega))

    def _update_one_control_tick(self, t: float, state: Dict[str, np.ndarray]) -> Dict[str, np.ndarray]:
        (
            roll_deg,
            pitch_deg,
            yaw_deg,
            gyro_roll_dps,
            gyro_pitch_dps,
            gyro_yaw_dps,
            pos_x_cm,
            pos_y_cm,
            height_cm,
            vx_cms,
            vy_cms,
        ) = self._state_feedback(state)

        obs_x, obs_y, pos_err_x, pos_err_y, vx_target, vy_target = self._update_position_loop(
            pos_x_cm, pos_y_cm, vx_cms, vy_cms
        )

        ax_cmd = self.pid["vel_x"].calculate(vx_target, vx_cms, self.cfg.control_dt_s)
        ay_cmd = self.pid["vel_y"].calculate(vy_target, vy_cms, self.cfg.control_dt_s)
        pitch_corr = self.cfg.rotorpy_pitch_axis_sign * (-(ax_cmd / GRAVITY_CMS2) * RAD2DEG)
        roll_corr = self.cfg.rotorpy_roll_axis_sign * ((ay_cmd / GRAVITY_CMS2) * RAD2DEG)
        target_pitch = self.cfg.hover_pitch_deg + pitch_corr
        target_roll = self.cfg.hover_roll_deg + roll_corr
        target_yaw = self.cfg.target_yaw_deg

        roll_rate_raw = self.pid["angle_roll"].calculate(target_roll, roll_deg, self.cfg.control_dt_s)
        pitch_rate_raw = self.pid["angle_pitch"].calculate(target_pitch, pitch_deg, self.cfg.control_dt_s)
        target_roll_rate, target_pitch_rate = self._update_angle_rate_lpf(roll_rate_raw, pitch_rate_raw)
        target_yaw_rate = self.pid["angle_yaw"].calculate(target_yaw, yaw_deg, self.cfg.control_dt_s)

        self.roll_output = self.pid["gyro_roll"].calculate(target_roll_rate, gyro_roll_dps, self.cfg.control_dt_s)
        self.pitch_output = self.pid["gyro_pitch"].calculate(target_pitch_rate, gyro_pitch_dps, self.cfg.control_dt_s)
        self.yaw_output = -self.pid["gyro_yaw"].calculate(target_yaw_rate, gyro_yaw_dps, self.cfg.control_dt_s)

        self._update_height_loop(height_cm)
        motor = self._mix_motor_outputs(self.cfg.base_throttle, roll_deg, pitch_deg)
        rotorpy_motor = [motor[idx] for idx in self.cfg.firmware_to_rotorpy_order]
        rotor_omega = np.array([self._throttle_to_omega(v) for v in rotorpy_motor])

        self.last_diag = ControlDiag(
            t=t,
            pos_x_cm=obs_x,
            pos_y_cm=obs_y,
            target_x_cm=self.cfg.target_x_cm,
            target_y_cm=self.cfg.target_y_cm,
            pos_error_x_cm=pos_err_x,
            pos_error_y_cm=pos_err_y,
            vx_target_cms=vx_target,
            vy_target_cms=vy_target,
            vx_cms=vx_cms,
            vy_cms=vy_cms,
            ax_cmd_cms2=ax_cmd,
            ay_cmd_cms2=ay_cmd,
            roll_deg=roll_deg,
            pitch_deg=pitch_deg,
            yaw_deg=yaw_deg,
            target_roll_deg=target_roll,
            target_pitch_deg=target_pitch,
            target_yaw_deg=target_yaw,
            target_roll_rate_dps=target_roll_rate,
            target_pitch_rate_dps=target_pitch_rate,
            target_yaw_rate_dps=target_yaw_rate,
            gyro_roll_dps=gyro_roll_dps,
            gyro_pitch_dps=gyro_pitch_dps,
            gyro_yaw_dps=gyro_yaw_dps,
            roll_output=self.roll_output,
            pitch_output=self.pitch_output,
            yaw_output=self.yaw_output,
            height_cm=height_cm,
            target_height_cm=self.cfg.target_height_cm,
            height_error_cm=self.cfg.target_height_cm - height_cm,
            height_pid_output=self.height_pid_output,
            base_throttle=self.cfg.base_throttle,
            motor0=motor[0],
            motor1=motor[1],
            motor2=motor[2],
            motor3=motor[3],
            rotor0_rad_s=rotor_omega[0],
            rotor1_rad_s=rotor_omega[1],
            rotor2_rad_s=rotor_omega[2],
            rotor3_rad_s=rotor_omega[3],
        )
        return {"cmd_motor_speeds": rotor_omega}

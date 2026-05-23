from __future__ import annotations

from dataclasses import dataclass

from .config import PIDConfig


@dataclass
class PIDController:
    kp: float
    ki: float
    kd: float
    output_limit: float
    integral_limit: float
    integral_threshold: float
    alpha: float
    integral: float = 0.0
    derivative: float = 0.0
    prev_error: float = 0.0

    @classmethod
    def from_config(cls, cfg: PIDConfig) -> "PIDController":
        return cls(
            cfg.kp,
            cfg.ki,
            cfg.kd,
            cfg.output_limit,
            cfg.integral_limit,
            cfg.integral_threshold,
            cfg.alpha,
        )

    def calculate(self, target: float, current: float, dt: float) -> float:
        """Match project/code/control/pid_controller.c semantics.

        The firmware currently ignores dt in integral and derivative scaling.
        """
        _ = dt
        error = target - current
        integral_factor = 1.0
        if abs(error) > 0.0 and self.integral_threshold > 0.0:
            if abs(error) >= self.integral_threshold:
                integral_factor = self.integral_threshold / abs(error)

        self.integral += integral_factor * error
        self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))

        raw_derivative = error - self.prev_error
        self.derivative = self.alpha * raw_derivative + (1.0 - self.alpha) * self.derivative

        output = self.kp * error + self.ki * self.integral + self.kd * self.derivative
        output = max(-self.output_limit, min(self.output_limit, output))
        self.prev_error = error
        return output

    def reset(self) -> None:
        self.integral = 0.0
        self.prev_error = 0.0

    @property
    def i_out(self) -> float:
        return self.ki * self.integral


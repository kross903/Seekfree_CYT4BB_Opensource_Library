from __future__ import annotations

import argparse
import shlex
from pathlib import Path
from typing import Optional

try:
    from .config import SimulationConfig
    from .simulation_engine import (
        SimulationOutput,
        plot_results,
        run_simulation_with_result,
        save_csv,
        show_rotorpy_animation,
        show_rotorpy_plot,
    )
except ImportError:
    import sys

    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
    from pid_simulation.rotorpy_pid.config import SimulationConfig
    from pid_simulation.rotorpy_pid.simulation_engine import (
        SimulationOutput,
        plot_results,
        run_simulation_with_result,
        save_csv,
        show_rotorpy_animation,
        show_rotorpy_plot,
    )


PID_FIELDS = {"kp", "ki", "kd", "output_limit", "integral_limit", "integral_threshold", "alpha"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Terminal PID tuning for RotorPy firmware PID simulation.")
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--sim-rate", type=int, default=300)
    parser.add_argument("--rotorpy-root", type=Path, default=None)
    return parser.parse_args()


def print_help() -> None:
    print(
        """
Commands:
  run                         run simulation with current parameters
  show                        show custom PID overview plot
  rotorpy_plot                show RotorPy Plotter figures
  rotorpy_animate             show RotorPy animation
  common                      run, save, show RotorPy figures, and show wind-arrow animation
  run_common                  alias of common
  save                        save CSV and overview PNG to outputs/
  list                        list current PID parameters
  set <pid>.<field> <value>   set PID parameter, e.g. set pos_x.kp 1.5
  setcfg <name> <value>       set scalar config, e.g. setcfg initial_x_cm 80
  help                        print this help
  quit                        exit

PID names:
  pos_x pos_y vel_x vel_y angle_roll angle_pitch angle_yaw gyro_roll gyro_pitch gyro_yaw height

PID fields:
  kp ki kd output_limit integral_limit integral_threshold alpha
"""
    )


def print_summary(output: SimulationOutput) -> None:
    if not output.rows:
        print("No simulation rows.")
        return
    last = output.rows[-1]
    max_motor = max(max(row[f"motor{i}"] for i in range(4)) for row in output.rows)
    print(
        "Final: "
        f"x={last['pos_x_cm']:.1f}cm y={last['pos_y_cm']:.1f}cm "
        f"h={last['height_cm']:.1f}cm vx={last['vx_cms']:.1f}cm/s vy={last['vy_cms']:.1f}cm/s "
        f"max_motor={max_motor:.0f}"
    )


def list_pid(cfg: SimulationConfig) -> None:
    for name, pid in cfg.pid.items():
        print(
            f"{name:12s} "
            f"kp={pid.kp:g} ki={pid.ki:g} kd={pid.kd:g} "
            f"out={pid.output_limit:g} int={pid.integral_limit:g} "
            f"thr={pid.integral_threshold:g} alpha={pid.alpha:g}"
        )


def set_pid_value(cfg: SimulationConfig, key: str, value_text: str) -> None:
    if "." not in key:
        raise ValueError("Use format <pid>.<field>, e.g. pos_x.kp")
    pid_name, field = key.split(".", 1)
    field = field.lower()
    if pid_name not in cfg.pid:
        raise ValueError(f"Unknown PID name: {pid_name}")
    if field not in PID_FIELDS:
        raise ValueError(f"Unknown PID field: {field}")
    setattr(cfg.pid[pid_name], field, float(value_text))
    print(f"Set {pid_name}.{field} = {float(value_text):g}")


def set_config_value(cfg: SimulationConfig, key: str, value_text: str) -> None:
    if not hasattr(cfg, key):
        raise ValueError(f"Unknown config name: {key}")
    old_value = getattr(cfg, key)
    if isinstance(old_value, bool):
        value = value_text.lower() in {"1", "true", "yes", "on"}
    elif isinstance(old_value, int):
        value = int(float(value_text))
    elif isinstance(old_value, float):
        value = float(value_text)
    else:
        raise ValueError(f"Config is not scalar-tunable: {key}")
    setattr(cfg, key, value)
    print(f"Set cfg.{key} = {value}")


def save_output(cfg: SimulationConfig, output: Optional[SimulationOutput]) -> None:
    if output is None:
        print("No result yet. Run simulation first.")
        return
    cfg.output_dir.mkdir(parents=True, exist_ok=True)
    save_csv(output.rows, cfg.output_dir / "rotorpy_pid_log.csv")
    plot_results(output.rows, cfg.output_dir / "rotorpy_pid_plot.png", show=False)
    print(f"Saved CSV: {cfg.output_dir / 'rotorpy_pid_log.csv'}")
    print(f"Saved plot: {cfg.output_dir / 'rotorpy_pid_plot.png'}")


def run_common_workflow(cfg: SimulationConfig) -> SimulationOutput:
    output = run_simulation_with_result(cfg)
    print_summary(output)
    save_output(cfg, output)
    show_rotorpy_plot(output.rotorpy_result, cfg, fname="rotorpy_pid")
    show_rotorpy_animation(output.rotorpy_result, cfg, fname="rotorpy_pid", animate_wind=True)
    return output


def main() -> None:
    args = parse_args()
    cfg = SimulationConfig(duration_s=args.duration, sim_rate_hz=args.sim_rate)
    if args.rotorpy_root is not None:
        cfg.rotorpy_root = args.rotorpy_root

    output: Optional[SimulationOutput] = None
    print_help()

    while True:
        try:
            command_line = input("rotorpy_pid> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break

        if not command_line:
            continue

        try:
            parts = shlex.split(command_line)
            cmd = parts[0].lower()
            if cmd in {"quit", "exit", "q"}:
                break
            if cmd == "help":
                print_help()
            elif cmd == "list":
                list_pid(cfg)
            elif cmd == "set":
                if len(parts) != 3:
                    raise ValueError("Usage: set <pid>.<field> <value>")
                set_pid_value(cfg, parts[1], parts[2])
            elif cmd == "setcfg":
                if len(parts) != 3:
                    raise ValueError("Usage: setcfg <name> <value>")
                set_config_value(cfg, parts[1], parts[2])
            elif cmd == "run":
                output = run_simulation_with_result(cfg)
                print_summary(output)
            elif cmd == "show":
                if output is None:
                    output = run_simulation_with_result(cfg)
                    print_summary(output)
                plot_results(output.rows, show=True)
            elif cmd == "rotorpy_plot":
                if output is None:
                    output = run_simulation_with_result(cfg)
                    print_summary(output)
                show_rotorpy_plot(output.rotorpy_result, cfg)
            elif cmd == "rotorpy_animate":
                if output is None:
                    output = run_simulation_with_result(cfg)
                    print_summary(output)
                show_rotorpy_animation(output.rotorpy_result, cfg, animate_wind=True)
            elif cmd in {"common", "run_common"}:
                output = run_common_workflow(cfg)
            elif cmd == "save":
                if output is None:
                    output = run_simulation_with_result(cfg)
                    print_summary(output)
                save_output(cfg, output)
            else:
                print(f"Unknown command: {cmd}. Type help.")
        except Exception as exc:
            print(f"Error: {exc}")


if __name__ == "__main__":
    main()

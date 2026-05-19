from __future__ import annotations

import argparse
from pathlib import Path

try:
    from .config import SimulationConfig
    from .simulation_engine import (
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
        plot_results,
        run_simulation_with_result,
        save_csv,
        show_rotorpy_animation,
        show_rotorpy_plot,
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run RotorPy firmware PID simulation.")
    parser.add_argument("--duration", type=float, default=8.0, help="Simulation duration in seconds.")
    parser.add_argument("--x0-cm", type=float, default=50.0, help="Initial X relative position in cm.")
    parser.add_argument("--y0-cm", type=float, default=30.0, help="Initial Y relative position in cm.")
    parser.add_argument("--height-cm", type=float, default=110.0, help="Target height in cm.")
    parser.add_argument("--z0-cm", type=float, default=110.0, help="Initial height in cm.")
    parser.add_argument("--base-throttle", type=float, default=770.0, help="Firmware base throttle.")
    parser.add_argument("--sim-rate", type=int, default=500, help="RotorPy vehicle step rate in Hz.")
    parser.add_argument("--rotorpy-root", type=Path, default=None, help="Path to rotorpy-main.")
    parser.add_argument("--show", action="store_true", help="Show matplotlib plot window.")
    parser.add_argument("--rotorpy-plot", action="store_true", help="Show RotorPy Plotter windows.")
    parser.add_argument("--rotorpy-animate", action="store_true", help="Show RotorPy animation.")
    parser.add_argument("--animate-wind", action="store_true", help="Show the RotorPy wind arrow in the animation.")
    parser.add_argument("--save-rotorpy-prefix", type=str, default=None, help="Save RotorPy Plotter figures with this prefix.")
    parser.add_argument("--common", action="store_true", help="Run the common workflow: save RotorPy plots and show wind-arrow animation.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    cfg = SimulationConfig(
        duration_s=args.duration,
        initial_x_cm=args.x0_cm,
        initial_y_cm=args.y0_cm,
        initial_height_cm=args.z0_cm,
        target_height_cm=args.height_cm,
        base_throttle=args.base_throttle,
        sim_rate_hz=args.sim_rate,
    )
    if args.rotorpy_root is not None:
        cfg.rotorpy_root = args.rotorpy_root

    if args.common and args.save_rotorpy_prefix is None:
        args.save_rotorpy_prefix = "rotorpy_pid"
    if args.common:
        args.rotorpy_plot = True
        args.rotorpy_animate = True
        args.animate_wind = True

    output = run_simulation_with_result(cfg)
    rows = output.rows
    out_dir = cfg.output_dir
    save_csv(rows, out_dir / "rotorpy_pid_log.csv")
    plot_results(rows, out_dir / "rotorpy_pid_plot.png", show=args.show)
    if args.rotorpy_plot or args.save_rotorpy_prefix:
        show_rotorpy_plot(output.rotorpy_result, cfg, fname=args.save_rotorpy_prefix)
    if args.rotorpy_animate:
        show_rotorpy_animation(output.rotorpy_result, cfg, fname=args.save_rotorpy_prefix, animate_wind=args.animate_wind)
    print(f"Saved CSV: {out_dir / 'rotorpy_pid_log.csv'}")
    print(f"Saved plot: {out_dir / 'rotorpy_pid_plot.png'}")
    if rows:
        last = rows[-1]
        print(
            "Final: "
            f"x={last['pos_x_cm']:.1f}cm y={last['pos_y_cm']:.1f}cm "
            f"h={last['height_cm']:.1f}cm vx={last['vx_cms']:.1f}cm/s vy={last['vy_cms']:.1f}cm/s"
        )


if __name__ == "__main__":
    main()

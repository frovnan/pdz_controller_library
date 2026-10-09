#!/usr/bin/env python3

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd


def main():
    RESULTS_DIR = (
        Path.home()
        / "franka_ros2_ws"
        / "src"
        / "pdz_controller_library"
        / "results"
    )

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "controller",
        help="Controller name",
    )
    parser.add_argument(
        "trajectory",
        choices=["1", "2", "3"],
        help="Trajectory type",
    )
    parser.add_argument(
        "timestamp",
        nargs="?",
        help="Specific recording timestamp",
    )
    args = parser.parse_args()

    # Example:
    # ros2 run pdz_controller_library plot_pose_error.py --controller joint_impedance_ik_controller --trajectory 3 --timestamp 20261009_110617

    controller_dir = RESULTS_DIR / args.controller

    if args.timestamp:
        prefix = f"{args.controller}_{args.trajectory}_{args.timestamp}"
        real_csv = controller_dir / f"{prefix}_real.csv"
        desired_csv = controller_dir / f"{prefix}_desired.csv"
    else:
        real_files = sorted(
            controller_dir.glob(f"{args.controller}_*_real.csv"),
            key=lambda p: p.stat().st_mtime,
        )

        if not real_files:
            parser.error(f"No real CSV files found in {controller_dir}")

        real_csv = real_files[-1]
        desired_csv = real_csv.with_name(
            real_csv.name.replace("_real.csv", "_desired.csv")
        )

    if not real_csv.exists():
        parser.error(f"Real CSV not found: {real_csv}")

    if not desired_csv.exists():
        parser.error(f"Matching desired CSV not found: {desired_csv}")

    print(f"Real data:    {real_csv}")
    print(f"Desired data: {desired_csv}")

    # Load CSV data
    real = pd.read_csv(real_csv)
    desired = pd.read_csv(desired_csv)

    # Pose components: CSV column, plot label, vertical axis label.
    pose_plots = [
        ("x", "x", "x [m]"),
        ("y", "y", "y [m]"),
        ("z", "z", "z [m]"),
        ("rx", "rx", "rx [rad]"),
        ("ry", "ry", "ry [rad]"),
        ("rz", "rz", "rz [rad]"),
    ]

    fig, axes = plt.subplots(8, 1, figsize=(10, 14), sharex=True)

    for ax, (column, label, ylabel) in zip(axes[:6], pose_plots):
        ax.plot(real["time_s"], real[column], label=f"{label} measured")
        ax.plot(
            desired["time_s"],
            desired[column],
            label=f"{label} desired",
            linestyle="--",
        )
        ax.set_ylabel(ylabel)
        ax.legend(loc="upper right")
        ax.grid(True, alpha=0.3)

    axes[6].plot(
        real["time_s"],
        real["position_error_norm"],
        label="Position error norm",
    )
    axes[6].set_ylabel("Position error [m]")
    axes[6].legend(loc="upper right")
    axes[6].grid(True, alpha=0.3)

    axes[7].plot(
        real["time_s"],
        real["geodesic_error"],
        label="Orientation error",
    )
    axes[7].set_ylabel("Orientation error [rad]")
    axes[7].set_xlabel("Time [s]")
    axes[7].legend(loc="upper right")
    axes[7].grid(True, alpha=0.3)

    fig.suptitle(real_csv.stem.replace("_real", ""), fontsize=14)
    fig.tight_layout(rect=[0, 0, 1, 0.98])

    # Save next to the CSV files, then display the figure.
    output_path = real_csv.with_name(
        real_csv.stem.replace("_real", "") + "_plot.png"
    )
    fig.savefig(output_path, dpi=300, bbox_inches="tight")
    print(f"Saved plot to: {output_path}")

    plt.show()


if __name__ == "__main__":
    main()
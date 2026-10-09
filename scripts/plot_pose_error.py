#!/usr/bin/env python3

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd


def main():
    parser = argparse.ArgumentParser(
        description="Plot measured and desired robot poses from CSV logs."
    )
    parser.add_argument(
        "real_csv",
        type=Path,
        help="Path to the *_real.csv file",
    )
    parser.add_argument(
        "desired_csv",
        type=Path,
        help="Path to the *_desired.csv file",
    )
    args = parser.parse_args()

    real = pd.read_csv(args.real_csv)
    desired = pd.read_csv(args.desired_csv)

    # Pose components: CSV column, plot label, vertical axis label.
    pose_plots = [
        ("x", "x", "x [m]"),
        ("y", "y", "y [m]"),
        ("z", "z", "z [m]"),
        ("rx", "rx", "rx [rad]"),
        ("ry", "ry", "ry [rad]"),
        ("rz", "rz", "rz [rad]"),
    ]

    fig, axes = plt.subplots(8, 1, figsize=(12, 18), sharex=True)

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
        label="Geodesic orientation error",
    )
    axes[7].set_ylabel("Orientation error [rad]")
    axes[7].set_xlabel("Time [s]")
    axes[7].legend(loc="upper right")
    axes[7].grid(True, alpha=0.3)

    fig.suptitle(args.real_csv.stem.replace("_real", ""), fontsize=14)
    fig.tight_layout(rect=[0, 0, 1, 0.98])

    # Save next to the CSV files, then display the figure.
    output_path = args.real_csv.with_name(
        args.real_csv.stem.replace("_real", "") + "_plot.png"
    )
    fig.savefig(output_path, dpi=300, bbox_inches="tight")
    print(f"Saved plot to: {output_path}")

    plt.show()


if __name__ == "__main__":
    main()
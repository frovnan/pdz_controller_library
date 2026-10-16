#!/usr/bin/env python3

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd


DOFS = [
    ("x", "X position", "m"),
    ("y", "Y position", "m"),
    ("z", "Z position", "m"),
    ("rx", "Roll", "rad"),
    ("ry", "Pitch", "rad"),
    ("rz", "Yaw", "rad"),
]


def main():
    parser = argparse.ArgumentParser(
        description="Plot six frequency sweeps, one for each DoF."
    )
    parser.add_argument(
        "controller",
        help="Controller name, matching the results subfolder",
    )
    parser.add_argument(
        "timestamps",
        nargs=6,
        help="Six run timestamps in order: X Y Z Roll Pitch Yaw",
    )
    args = parser.parse_args()

    results_dir = (
        Path(__file__).resolve().parent.parent
        / "results"
        / args.controller
    )

    if not results_dir.is_dir():
        raise FileNotFoundError(
            f"Results directory not found: {results_dir}"
        )

    fig, axes = plt.subplots(6, 1, figsize=(15, 11))
    axes = axes.flatten()

    for i, (timestamp, (column, title, unit)) in enumerate(
        zip(args.timestamps, DOFS)
    ):
        stem = f"{args.controller}_3_{timestamp}"
        real_file = results_dir / f"{stem}_real.csv"
        desired_file = results_dir / f"{stem}_desired.csv"

        if not real_file.is_file():
            raise FileNotFoundError(f"Missing file: {real_file}")
        if not desired_file.is_file():
            raise FileNotFoundError(f"Missing file: {desired_file}")

        real_df = pd.read_csv(real_file)
        desired_df = pd.read_csv(desired_file)

        real_df = real_df[real_df["time_s"] <= 125]
        desired_df = desired_df[desired_df["time_s"] <= 125]

        for df, path in (
            (real_df, real_file),
            (desired_df, desired_file),
        ):
            if not {"time_s", column}.issubset(df.columns):
                raise ValueError(
                    f"{path.name} must contain 'time_s' and '{column}'"
                )

        ax = axes[i]

        ax.plot(
            desired_df["time_s"],
            desired_df[column],
            color="orange",
            alpha=0.75,
            linewidth=1.0,
            label="Desired",
            zorder=1,
        )
        ax.plot(
            real_df["time_s"],
            real_df[column],
            color="tab:blue",
            linewidth=1.0,
            label="Measured",
            zorder=2,
        )

        ax.set_title(f"{title}")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel(f"{title} ({unit})")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right")

    fig.suptitle(
        f"Frequency sweep — {args.controller}", fontsize=15
    )
    fig.tight_layout()

    output_file = results_dir / "frequency_sweep_all_dofs.png"
    fig.savefig(output_file, dpi=200, bbox_inches="tight")

    print(f"Saved plot to: {output_file}")
    plt.show()


if __name__ == "__main__":
    main()

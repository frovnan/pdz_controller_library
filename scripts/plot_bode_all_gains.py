#!/usr/bin/env python3

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd

DOFS = [
    ("x", "X position"),
    ("y", "Y position"),
    ("z", "Z position"),
    ("rx", "Roll"),
    ("ry", "Pitch"),
    ("rz", "Yaw"),
]

GAINS = {
    "low": "Low",
    "mid": "Mid",
    "high": "High",
}

COLORS = {
    "low": "tab:blue",
    "mid": "tab:orange",
    "high": "tab:green",
}

REQUIRED_COLUMNS = {
    "dof",
    "frequency_hz",
    "gain_db",
    "phase_deg",
}


def load_results(results_dir, controller, gain):

    filename = (
        f"{controller}_bode_results_{gain}_impedance.csv"
    )

    filepath = results_dir / filename

    if not filepath.is_file():
        raise FileNotFoundError(f"Missing file: {filepath}")

    df = pd.read_csv(filepath)

    missing = REQUIRED_COLUMNS - set(df.columns)

    if missing:
        raise ValueError(
            f"{filepath.name} is missing columns: {missing}"
        )

    return df


def main():

    parser = argparse.ArgumentParser(
        description="Plot Bode responses for different impedance gains."
    )

    parser.add_argument(
        "controller",
        help="Controller name matching the results folder and CSV prefix",
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

    # Load the three gain configurations.
    results = {
        gain: load_results(results_dir, args.controller, gain)
        for gain in GAINS
    }

    # Create 3x2 grid.
    fig, axes = plt.subplots(
        3, 2,
        figsize=(15, 13),
        squeeze=False,
    )

    for i, (dof, title) in enumerate(DOFS):

        row = i % 3
        col = i // 3

        parent_ax = axes[row, col]
        parent_ax.remove()

        subgrid = parent_ax.get_subplotspec().subgridspec(
            2, 1,
            hspace=0.12,
        )

        ax_mag = fig.add_subplot(subgrid[0])
        ax_phase = fig.add_subplot(
            subgrid[1],
            sharex=ax_mag,
        )

        ax_mag.set_title(title)
        ax_mag.set_ylabel("Gain (dB)")
        ax_phase.set_ylabel("Phase (°)")
        ax_phase.set_xlabel("Frequency (Hz)")

        ax_mag.grid(True, which="both", alpha=0.3)
        ax_phase.grid(True, which="both", alpha=0.3)

        ax_mag.set_xscale("log")

        # Plot all three gain configurations.
        for gain, df in results.items():

            subset = (
                df[df["dof"] == dof]
                .sort_values("frequency_hz")
            )

            if subset.empty:
                continue

            ax_mag.semilogx(
                subset["frequency_hz"],
                subset["gain_db"],
                "o-",
                color=COLORS[gain],
                markersize=3,
                label=GAINS[gain],
            )

            ax_phase.semilogx(
                subset["frequency_hz"],
                subset["phase_deg"],
                "o-",
                color=COLORS[gain],
                markersize=3,
                label=GAINS[gain],
            )

        ax_mag.tick_params(axis="x", labelbottom=False)

        if col == 0:
            ax_mag.legend(fontsize=8)

    fig.suptitle(
        f"Bode plots — {args.controller}",
        fontsize=16,
    )

    fig.tight_layout(rect=[0, 0, 1, 0.97])

    output_file = results_dir / f"{args.controller}_bode_all_gains.png"

    fig.savefig(
        output_file,
        dpi=250,
        bbox_inches="tight",
    )

    print(f"Saved plots to: {output_file}")

    plt.show()


if __name__ == "__main__":
    main()
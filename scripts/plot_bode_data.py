#!/usr/bin/env python3

import argparse
from pathlib import Path

import numpy as np
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

# Start time of each frequency interval, in seconds.

change_times = np.array([
    5.0, 25.0,
    28.0, 44.6667,
    47.6667, 61.5556,
    64.5556, 76.1296,
    79.1296, 88.7747,
    91.7747, 99.812,
    102.812, 109.510,
    112.510, 118.092,
    121.092, 126.092,
    129.092, 134.092,
    137.092, 142.092,
    145.092, 150.092,
    153.092, 158.092,
    161.092, 166.092,
    169.092, 174.092,
    177.092, 182.092,
    185.092, 190.092,
])

omega_steps = np.array([
    1.5708,
    1.88496,
    2.26195,
    2.71434,
    3.2572,
    3.90864,
    4.69037,
    5.62845,
    6.75414,
    8.10496,
    9.72596,
    11.6711,
    14.0054,
    16.8065,
    20.1677,
    24.2013,
    25.1327,
])




def fit_sinusoid(time, signal, omega):
    """Fit y(t) = c + a*sin(omega*t) + b*cos(omega*t)."""
    time = np.asarray(time, dtype=float)
    signal = np.asarray(signal, dtype=float)

    valid = np.isfinite(time) & np.isfinite(signal)
    time = time[valid]
    signal = signal[valid]

    if len(time) < 4:
        return None

    # Center time for numerical conditioning.
    t_ref = time[0]
    t = time - t_ref

    X = np.column_stack([
        np.ones_like(t),
        np.sin(omega * t),
        np.cos(omega * t),
    ])

    coeffs, _, rank, _ = np.linalg.lstsq(X, signal, rcond=None)

    if rank < 3:
        return None

    c, a, b = coeffs
    amplitude = np.hypot(a, b)

    # Phase for the convention A*sin(omega*t + phase).
    phase_local = np.arctan2(b, a)
    phase = phase_local - omega * t_ref
    phase = (phase + np.pi) % (2 * np.pi) - np.pi

    return {
        "offset": c,
        "amplitude": amplitude,
        "phase_rad": phase,
    }


def analyze_run(real_df, desired_df, column):
    """Calculate gain and phase for one DoF across the sweep."""
    results = []

    # Use the shorter recording as the end of the final interval.
    recording_end = min(
        real_df["time_s"].max(),
        desired_df["time_s"].max(),
    )

    interval_ends = np.append(change_times[1:], recording_end)

    for i, omega in enumerate(omega_steps):
        t_start = change_times[2 * i]
        t_end = interval_ends[2 * i + 1]

        if t_end <= t_start:
            continue

        real_segment = real_df[
            (real_df["time_s"] >= t_start)
            & (real_df["time_s"] < t_end)
        ]

        desired_segment = desired_df[
            (desired_df["time_s"] >= t_start)
            & (desired_df["time_s"] < t_end)
        ]

        real_fit = fit_sinusoid(
            real_segment["time_s"],
            real_segment[column],
            omega,
        )

        desired_fit = fit_sinusoid(
            desired_segment["time_s"],
            desired_segment[column],
            omega,
        )

        if real_fit is None or desired_fit is None:
            continue

        A_real = real_fit["amplitude"]
        A_desired = desired_fit["amplitude"]

        # Ignore intervals with insufficient reference excitation.
        if A_desired < 1e-8:
            continue

        gain = A_real / A_desired
        gain_db = 20 * np.log10(max(gain, 1e-12))

        phase_diff = real_fit["phase_rad"] - desired_fit["phase_rad"]
        phase_diff = (phase_diff + np.pi) % (2 * np.pi) - np.pi

        results.append({
            "dof": column,
            "frequency_hz": omega / (2 * np.pi),
            "gain_db": gain_db,
            "phase_deg": np.degrees(phase_diff),
            "n_real": len(real_segment),
            "n_desired": len(desired_segment),
            "duration_s": t_end - t_start,
        })

    return results


def main():
    parser = argparse.ArgumentParser(
        description="Calculate Bode plots from six frequency-sweep runs."
    )
    parser.add_argument(
        "controller",
        help="Controller name matching the results subfolder",
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
        raise FileNotFoundError(f"Results directory not found: {results_dir}")

    all_results = []

    for timestamp, (column, title, unit) in zip(args.timestamps, DOFS):
        stem = f"{args.controller}_3_{timestamp}"
        real_file = results_dir / f"{stem}_real.csv"
        desired_file = results_dir / f"{stem}_desired.csv"

        if not real_file.is_file():
            raise FileNotFoundError(f"Missing file: {real_file}")
        if not desired_file.is_file():
            raise FileNotFoundError(f"Missing file: {desired_file}")

        real_df = pd.read_csv(real_file)
        desired_df = pd.read_csv(desired_file)

        for df, path in (
            (real_df, real_file),
            (desired_df, desired_file),
        ):
            if not {"time_s", column}.issubset(df.columns):
                raise ValueError(
                    f"{path.name} must contain 'time_s' and '{column}'"
                )

        # Restrict to the recorded sweep.
        #real_df = real_df[real_df["time_s"] <= 165]
        #desired_df = desired_df[desired_df["time_s"] <= 165]

        run_results = analyze_run(real_df, desired_df, column)
        all_results.extend(run_results)

        print(f"{title}: {len(run_results)} frequency points fitted")

    results = pd.DataFrame(all_results)

    if results.empty:
        raise RuntimeError("No valid frequency-response points were fitted.")

    # Save numerical results for later controller comparisons.
    csv_file = results_dir / "bode_results.csv"
    results.to_csv(csv_file, index=False)
    print(f"Saved numerical results to: {csv_file}")

    
    
    # 3x2 grid of DoFs, each containing magnitude and phase subplots
    fig, axes = plt.subplots(
        3, 2,
        figsize=(15, 15),
        squeeze=False,
    )

    for i, (column, title, unit) in enumerate(DOFS):
        row = i % 3
        col = i // 3
        parent_ax = axes[row, col]

        # Use a nested GridSpec to create two axes inside this grid cell.
        parent_ax.remove()
        subgrid = parent_ax.get_subplotspec().subgridspec(
            2, 1, hspace=0.15
        )

        ax_mag = fig.add_subplot(subgrid[0])
        ax_phase = fig.add_subplot(subgrid[1], sharex=ax_mag)

        subset = results[results["dof"] == column].sort_values(
            "frequency_hz"
        )

        ax_mag.set_title(title)
        ax_mag.set_ylabel("Gain (dB)")
        ax_mag.grid(True, which="both", alpha=0.3)
        ax_mag.tick_params(axis="x", labelbottom=False)

        ax_phase.set_xlabel("Frequency (Hz)")
        ax_phase.set_ylabel("Phase (°)")
        ax_phase.grid(True, which="both", alpha=0.3)

        if subset.empty:
            ax_mag.text(
                0.5, 0.5, "No valid frequency points",
                ha="center", va="center",
                transform=ax_mag.transAxes,
            )
            continue

        ax_mag.semilogx(
            subset["frequency_hz"],
            subset["gain_db"],
            "o-",
            markersize=3,
        )

        ax_phase.semilogx(
            subset["frequency_hz"],
            subset["phase_deg"],
            "o-",
            markersize=3,
        )

    fig.suptitle(
        f"Bode plots — {args.controller}",
        fontsize=16,
    )
    fig.tight_layout(rect=[0, 0, 1, 0.97])

    output_file = results_dir / "bode_all_dofs.png"
    fig.savefig(output_file, dpi=200, bbox_inches="tight")

    print(f"Saved plots to: {output_file}")
    plt.show()




if __name__ == "__main__":
    main()

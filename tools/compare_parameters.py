"""
Parameter Comparison Tool for NexDynIPC Simulations.

Compares two simulation CSV outputs (e.g. different mu or gamma values)
and generates a divergence analysis plot similar to divergence_analysis.png.

Usage:
    python tools/compare_parameters.py <csv_a> <csv_b> [--output <output_png>]

Example:
    python tools/compare_parameters.py \
        output/double_pendulum_mu_1000_gamma_0.5.csv \
        output/double_pendulum_mu_10000_gamma_0.5.csv \
        --output output/mu_comparison.png
"""

import argparse
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl

# Use Times New Roman font globally
mpl.rcParams['font.family'] = 'serif'
mpl.rcParams['font.serif'] = ['Times New Roman']
mpl.rcParams['mathtext.fontset'] = 'stix'
import os
import sys


def load_body_data(csv_path, body_name):
    """Load data for a specific body from the CSV."""
    df = pd.read_csv(csv_path)
    return df[df["body_name"] == body_name].sort_values("time").reset_index(drop=True)


def compute_divergence(df_a, df_b):
    """Compute position and acceleration divergence between two datasets."""
    # Ensure same length
    n = min(len(df_a), len(df_b))
    df_a = df_a.iloc[:n]
    df_b = df_b.iloc[:n]

    time = df_a["time"].values

    # Position divergence (Euclidean)
    pos_diff = np.sqrt(
        (df_a["pos_x"].values - df_b["pos_x"].values) ** 2
        + (df_a["pos_y"].values - df_b["pos_y"].values) ** 2
        + (df_a["pos_z"].values - df_b["pos_z"].values) ** 2
    )

    # Acceleration divergence (Euclidean)
    acc_diff = np.sqrt(
        (df_a["ax"].values - df_b["ax"].values) ** 2
        + (df_a["ay"].values - df_b["ay"].values) ** 2
        + (df_a["az"].values - df_b["az"].values) ** 2
    )

    # Velocity divergence (Euclidean)
    vel_diff = np.sqrt(
        (df_a["vx"].values - df_b["vx"].values) ** 2
        + (df_a["vy"].values - df_b["vy"].values) ** 2
        + (df_a["vz"].values - df_b["vz"].values) ** 2
    )

    return time, pos_diff, acc_diff, vel_diff


def rmse(arr):
    return np.sqrt(np.mean(arr ** 2))


def extract_label(filepath):
    """Extract a human-readable label from the filename."""
    basename = os.path.splitext(os.path.basename(filepath))[0]
    # e.g. "double_pendulum_mu_1000_gamma_0.5" -> "mu=1000, gamma=0.5"
    parts = basename.split("_")
    label_parts = []
    i = 0
    while i < len(parts):
        if parts[i] == "mu" and i + 1 < len(parts):
            label_parts.append(f"μ={parts[i+1]}")
            i += 2
        elif parts[i] == "gamma" and i + 1 < len(parts):
            label_parts.append(f"γ={parts[i+1]}")
            i += 2
        else:
            i += 1
    return ", ".join(label_parts) if label_parts else basename


def main():
    parser = argparse.ArgumentParser(description="Compare two NexDynIPC simulation outputs")
    parser.add_argument("csv_a", help="Path to first CSV file")
    parser.add_argument("csv_b", help="Path to second CSV file")
    parser.add_argument("--output", "-o", default="output/parameter_comparison.png",
                        help="Output PNG path (default: output/parameter_comparison.png)")
    parser.add_argument("--bodies", nargs="+", default=["Link1", "Link2"],
                        help="Body names to compare (default: Link1 Link2)")
    args = parser.parse_args()

    label_a = extract_label(args.csv_a)
    label_b = extract_label(args.csv_b)

    bodies = args.bodies
    n_bodies = len(bodies)

    # Create figure: 3 rows (pos, vel, acc) x n_bodies columns
    fig, axes = plt.subplots(3, n_bodies, figsize=(6 * n_bodies, 12))
    if n_bodies == 1:
        axes = axes.reshape(-1, 1)

    fig.suptitle(f"Parameter Comparison: [{label_a}] vs [{label_b}]",
                 fontsize=14, fontweight="bold", y=0.98)

    colors = {"pos": "#1f77b4", "vel": "#2ca02c", "acc": "#ff7f0e"}

    for col, body_name in enumerate(bodies):
        df_a = load_body_data(args.csv_a, body_name)
        df_b = load_body_data(args.csv_b, body_name)

        if len(df_a) == 0 or len(df_b) == 0:
            print(f"Warning: No data for body '{body_name}' in one of the files, skipping.")
            continue

        time, pos_diff, acc_diff, vel_diff = compute_divergence(df_a, df_b)

        # Row 0: Position Divergence
        ax = axes[0, col]
        pos_rmse = rmse(pos_diff)
        ax.plot(time, pos_diff, color=colors["pos"], linewidth=1.2)
        ax.set_title(f"{body_name} Position Divergence (RMSE: {pos_rmse:.2e})")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Euclidean Distance (m)")
        ax.grid(True, alpha=0.3)

        # Row 1: Velocity Divergence
        ax = axes[1, col]
        vel_rmse = rmse(vel_diff)
        ax.plot(time, vel_diff, color=colors["vel"], linewidth=1.2)
        ax.set_title(f"{body_name} Velocity Divergence (RMSE: {vel_rmse:.2e})")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Diff Magnitude (m/s)")
        ax.grid(True, alpha=0.3)

        # Row 2: Acceleration Divergence
        ax = axes[2, col]
        acc_rmse = rmse(acc_diff)
        ax.plot(time, acc_diff, color=colors["acc"], linewidth=1.2)
        ax.set_title(f"{body_name} Acceleration Divergence (RMSE: {acc_rmse:.2e})")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Diff Magnitude (m/s²)")
        ax.grid(True, alpha=0.3)

    plt.tight_layout(rect=[0, 0, 1, 0.96])

    os.makedirs(os.path.dirname(args.output) or ".", exist_ok=True)
    plt.savefig(args.output, dpi=150, bbox_inches="tight")
    print(f"Comparison plot saved to: {args.output}")
    plt.close()


if __name__ == "__main__":
    main()

"""
Reference Comparison Tool for NexDynIPC.

Compares simulation results (single CSV) against multi-file reference data
(as found in assets/reference_data/double_pendulum/).

Usage:
    python tools/compare_with_reference.py <sim_csv> <ref_dir> [--output <output_png>]

Example:
    python tools/compare_with_reference.py \
        output/double_pendulum_mu_1000_gamma_0.5.csv \
        assets/reference_data/double_pendulum/ \
        --output output/reference_comparison.png
"""

import argparse
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
import os
import glob

# Style settings
mpl.rcParams['font.family'] = 'serif'
mpl.rcParams['font.serif'] = ['Times New Roman']
mpl.rcParams['mathtext.fontset'] = 'stix'

def load_sim_data(csv_path, body_name):
    """Load simulation data for a specific body."""
    df = pd.read_csv(csv_path)
    # Filter by body name
    body_df = df[df["body_name"] == body_name].sort_values("time").reset_index(drop=True)
    return body_df

def load_ref_file(filepath):
    """
    Load a reference CSV file.
    Assumes format: Time, X, Y, Z, ...
    The first column is assumed to be time if it's monotonic.
    """
    try:
        # Reference CSVs often have complex headers, skip them or try to parse
        # Based on user view: "X:Pos_TX-Body1...", "0.000, 0.500..."
        # It looks like the file has a header.
        df = pd.read_csv(filepath)
        
        # Identify coordinate columns.
        # Usually checking for X/Y/Z in header or assuming columns 1,2,3 are X,Y,Z
        # after time (col 0).
        
        # Let's clean up column names
        df.columns = [c.strip() for c in df.columns]
        
        # Rename first column to 'time'
        df.rename(columns={df.columns[0]: 'time'}, inplace=True)
        
        # Identify spatial components. 
        # Typically the file has specific columns. 
        # For Link1_Pos.csv: Time, X, Y, Z (technically headers are X:Pos..., Y:Pos..., Y:Pos..., Y:Pos...)
        # Wait, lines 1 shows: X:Pos..., Y:Pos..., Y:Pos..., Y:Pos... 
        # Actually line 1 is header.
        # Column 0: Time (implied? No, looks like X:Pos is X coordinate? Wait.)
        
        # Let's re-examine Link1_Pos.csv content from previous turn:
        # 1: X:Pos_TX-Body1-Model1(m),Y:Pos_TX-Body1-Model1(m),Y:Pos_TY-Body1-Model1(m),Y:Pos_TZ-Body1-Model1(m)
        # 2: 0.0000000,0.50000000,0.0000000,0.0000000
        # 3: 0.0010130000,0.50000000,-1.5959431e-06,0.0000000
        
        # Column 0 (values 0, 0.001...) looks like TIME.
        # Column 1 (values 0.5...) looks like X position?
        # Column 2 (values 0, -1.5e-6...) looks like Y position.
        # Column 3 (values 0...) looks like Z position.
        
        # So structure is likely: Time, X, Y, Z
        return df.iloc[:, [0, 1, 2, 3]]
    except Exception as e:
        print(f"Error loading {filepath}: {e}")
        return None

def rmse(arr):
    return np.sqrt(np.mean(arr ** 2))

def main():
    parser = argparse.ArgumentParser(description="Compare Simulation vs Reference")
    parser.add_argument("sim_csv", help="Path to simulation result CSV")
    parser.add_argument("ref_dir", help="Directory containing reference CSVs")
    parser.add_argument("--output", "-o", default="output/reference_comparison.png")
    args = parser.parse_args()

    bodies = ["Link1", "Link2"]
    metrics = ["Pos", "Vel", "Acc"]
    
    # Map metric names to sim columns
    sim_cols = {
        "Pos": ["pos_x", "pos_y", "pos_z"],
        "Vel": ["vx", "vy", "vz"],
        "Acc": ["ax", "ay", "az"]
    }
    
    units = {"Pos": "m", "Vel": "m/s", "Acc": "m/s²"}
    colors = {"Pos": "#1f77b4", "Vel": "#2ca02c", "Acc": "#ff7f0e"} # Blue, Green, Orange

    fig, axes = plt.subplots(3, 2, figsize=(12, 12))
    fig.suptitle(f"Simulation vs Reference Divergence", fontsize=16, fontweight="bold")

    for col_idx, body in enumerate(bodies):
        sim_df = load_sim_data(args.sim_csv, body)
        if sim_df.empty:
            print(f"Warning: No simulation data for {body}")
            continue
            
        sim_time = sim_df["time"].values

        for row_idx, metric in enumerate(metrics):
            ax = axes[row_idx, col_idx]
            color = colors[metric]
            
            # Find reference file
            # Pattern example: Link1_Pos.csv
            pattern = os.path.join(args.ref_dir, f"{body}_{metric}.csv")
            matches = glob.glob(pattern)
            
            if not matches:
                ax.text(0.5, 0.5, f"Missing Reference\n{pattern}", 
                        ha='center', va='center', transform=ax.transAxes)
                continue
                
            ref_df = load_ref_file(matches[0])
            if ref_df is None: continue

            # Interpolate reference data to simulation time points
            ref_time = ref_df.iloc[:, 0].values
            
            # Extract ref vectors
            ref_vecs = []
            for i in range(1, 4): # cols 1, 2, 3 are x, y, z
                val = np.interp(sim_time, ref_time, ref_df.iloc[:, i].values)
                ref_vecs.append(val)
            
            ref_vecs = np.array(ref_vecs).T # (N, 3)
            
            # Extract sim vectors
            sim_vec_cols = sim_cols[metric]
            sim_vecs = sim_df[sim_vec_cols].values # (N, 3)
            
            # Compute Euclidean distance
            diff = sim_vecs - ref_vecs
            dist = np.linalg.norm(diff, axis=1)
            
            err_rmse = rmse(dist)
            
            # Plot
            ax.plot(sim_time, dist, linewidth=1.2, label=f"Error (RMSE={err_rmse:.2e})", color=color)
            ax.set_title(f"{body} {metric} Divergence")
            ax.set_xlabel("Time (s)")
            ax.set_ylabel(f"Error ({units[metric]})")
            ax.grid(True, alpha=0.3)
            ax.legend(loc='upper right')

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    
    os.makedirs(os.path.dirname(args.output) or ".", exist_ok=True)
    plt.savefig(args.output, dpi=150, bbox_inches="tight")
    print(f"Comparison plot saved to: {args.output}")

if __name__ == "__main__":
    main()

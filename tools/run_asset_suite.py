import argparse
import glob
import subprocess
from pathlib import Path

import pandas as pd


def run_case(exe_path: Path, scene_file: Path, figures_root: Path) -> dict:
    case_name = scene_file.stem
    print(f"\n=== Running case: {case_name} ===")

    run_cmd = [str(exe_path), str(scene_file)]
    sim = subprocess.run(run_cmd, cwd=exe_path.parent.parent.parent)
    if sim.returncode != 0:
        return {"case": case_name, "status": "simulation_failed"}

    out_dir = exe_path.parent.parent.parent / "output" / case_name
    state_csv = out_dir / f"{case_name}.csv"
    kpi_csv = out_dir / f"{case_name}_kpi.csv"

    if not state_csv.exists() or not kpi_csv.exists():
        return {"case": case_name, "status": "missing_output"}

    fig_dir = figures_root / case_name
    viz_cmd = [
        "python",
        "tools/csv_visualizer.py",
        str(state_csv),
        str(kpi_csv),
        "-o",
        str(fig_dir),
        "--exact-output",
    ]
    viz = subprocess.run(viz_cmd, cwd=exe_path.parent.parent.parent)
    if viz.returncode != 0:
        return {"case": case_name, "status": "visualization_failed"}

    kpi_df = pd.read_csv(kpi_csv)
    result = {
        "case": case_name,
        "status": "ok",
        "frames": int(len(kpi_df)),
        "max_V_w": float(kpi_df["V_w"].max()),
        "max_T_sat": float(kpi_df["T_sat"].max()),
        "max_R_dual": float(kpi_df["R_dual"].max()),
        "max_constraint_violation": float(kpi_df["max_constraint_violation"].max()),
        "output_dir": str(out_dir),
        "figure_dir": str(fig_dir),
    }
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description="Run all asset cases and generate figures.")
    parser.add_argument(
        "--exe",
        type=str,
        default="build/Debug/NexDynIPC.exe",
        help="Path to simulation executable",
    )
    parser.add_argument(
        "--assets-glob",
        type=str,
        default="assets/*/*.json",
        help="Glob for asset JSON cases",
    )
    parser.add_argument(
        "--figures-root",
        type=str,
        default="figures",
        help="Root directory for generated figures",
    )
    parser.add_argument(
        "--summary",
        type=str,
        default="output/asset_suite_summary.csv",
        help="Summary CSV output path",
    )
    args = parser.parse_args()

    exe_path = Path(args.exe)
    if not exe_path.exists():
        print(f"Executable not found: {exe_path}")
        return 1

    case_files = [Path(p) for p in glob.glob(args.assets_glob) if "reference_data" not in p]
    case_files = sorted(case_files)
    if not case_files:
        print("No asset JSON cases found")
        return 1

    figures_root = Path(args.figures_root)
    figures_root.mkdir(parents=True, exist_ok=True)

    rows = []
    for scene_file in case_files:
        rows.append(run_case(exe_path, scene_file, figures_root))

    df = pd.DataFrame(rows)
    summary_path = Path(args.summary)
    summary_path.parent.mkdir(parents=True, exist_ok=True)
    df.to_csv(summary_path, index=False)

    ok_count = int((df["status"] == "ok").sum())
    print("\n=== Asset Suite Summary ===")
    print(f"Cases: {len(df)}, Success: {ok_count}, Failed: {len(df) - ok_count}")
    print(f"Summary: {summary_path}")

    return 0 if ok_count == len(df) else 2


if __name__ == "__main__":
    raise SystemExit(main())

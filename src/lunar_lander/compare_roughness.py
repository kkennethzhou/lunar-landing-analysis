"""Compare the nominal terrain with a harsher preset on the same lander and seed.

This is the short probe named in docs/known-issues.md OPEN-1. Adjacent feet
are 4.0 m apart; the older ~2.5 m figure was wrong. Prefer
`python -m lunar_lander.open1_campaigns` for the full 500-trial harsh
campaign on the compression-leg model. This module does not replace that.
"""

from __future__ import annotations

import argparse
import json
import tempfile
from pathlib import Path

import mujoco
import numpy as np
from scipy import stats

from lunar_lander.generate_world import make_heightmap, slope_degrees, write_world
from lunar_lander.paths import PROJECT_ROOT
from lunar_lander.run_monte_carlo import run_campaign


def _corr_landed(records, slope):
    landed = [row for row in records if row["outcome"] == "landed"]
    if len(landed) < 5:
        return None, None, len(landed)
    forces = np.array([row["peak_strut_force_N"] for row in landed])
    slopes = np.array([
        float(slope[int(round(row["y"])), int(round(row["x"]))])
        for row in landed
    ])
    r, p = stats.pearsonr(forces, slopes)
    return float(r), float(p), len(landed)


def run_preset(preset: str, n_trials: int, seed: int, output_dir: Path) -> dict:
    world = write_world(preset, output_dir)
    model = mujoco.MjModel.from_xml_path(str(output_dir / "lunar_world.xml"))
    records = run_campaign(
        model,
        n_trials=n_trials,
        seed=seed,
        csv_path=output_dir / "results.csv",
        record_summary=False,
    )
    height = make_heightmap(preset)
    slope = slope_degrees(height)
    r, p, n_landed = _corr_landed(records, slope)
    n_fail = sum(1 for row in records if row["outcome"] != "landed")
    return {
        "preset": preset,
        "relief_m": world["relief_m"],
        "slope_mean_deg": world["slope_mean_deg"],
        "slope_p95_deg": world["slope_p95_deg"],
        "slope_max_deg": world["slope_max_deg"],
        "n_trials": n_trials,
        "n_landed": n_landed,
        "failure_rate_pct": 100.0 * n_fail / n_trials,
        "corr_force_slope_r": r,
        "corr_force_slope_p": p,
        "stroke_max_m": max(row["peak_stroke_m"] for row in records),
    }


def compare(n_trials: int = 40, seed: int = 42) -> dict:
    """Run nominal and harsh presets in temporary directories."""
    summary = {"n_trials": n_trials, "seed": seed, "presets": {}}
    for preset in ("nominal", "harsh"):
        with tempfile.TemporaryDirectory() as tmp:
            summary["presets"][preset] = run_preset(preset, n_trials, seed, Path(tmp))
    nominal = summary["presets"]["nominal"]
    harsh = summary["presets"]["harsh"]
    summary["note"] = (
        "Same seed and same lander. A correlation here is a probe, not a "
        "replacement for the 500-trial validation_stats.json result."
    )
    summary["harsh_relief_over_nominal"] = harsh["relief_m"] / nominal["relief_m"]
    return summary


def main(argv=None) -> dict:
    parser = argparse.ArgumentParser(description="Nominal vs harsher terrain probe")
    parser.add_argument("--trials", type=int, default=40)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--output", default=str(PROJECT_ROOT / "docs" / "roughness_comparison.json"))
    args = parser.parse_args(argv)
    result = compare(args.trials, args.seed)
    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, indent=2))
    for name, row in result["presets"].items():
        print(
            f"{name}: relief {row['relief_m']:.2f} m, "
            f"mean slope {row['slope_mean_deg']:.2f}°, "
            f"failure {row['failure_rate_pct']:.1f}%, "
            f"r(force, slope)={row['corr_force_slope_r']}"
        )
    print(f"Wrote {out}")
    return result


if __name__ == "__main__":
    main()

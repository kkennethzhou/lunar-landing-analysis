"""OPEN-1 experimental campaigns for issue #3.

Runs into an isolated output directory. Never writes over published
docs/summary_stats.json or figures/optimal_landing_zones.png.

Campaigns (approved design order):
  0 / nominal-remeasure — same seed-42 nominal lander with touchdown logging,
    so spawn vs contact terrain can be compared.
  1 / harsh — 500 trials, seed 42, harsh heightmap, unchanged lander.
  2 / low-drift — 500 trials, seed 42, nominal terrain, vx=vy=0 after draws.

A null result (|r| < 0.2 on every documented condition) is a valid outcome.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
from pathlib import Path

import mujoco

from lunar_lander.generate_world import write_world
from lunar_lander.paths import PROJECT_ROOT
from lunar_lander.run_monte_carlo import run_campaign
from lunar_lander.validate_results import load_rows, run_validation

DEFAULT_ROOT = PROJECT_ROOT / "output" / "open1"
PUBLISHED_SUMMARY = PROJECT_ROOT / "docs" / "summary_stats.json"
PUBLISHED_FIGURE = PROJECT_ROOT / "figures" / "optimal_landing_zones.png"


def _assert_published_untouched(before_summary: bytes | None, before_figure: bytes | None):
    after_summary = PUBLISHED_SUMMARY.read_bytes() if PUBLISHED_SUMMARY.exists() else None
    after_figure = PUBLISHED_FIGURE.read_bytes() if PUBLISHED_FIGURE.exists() else None
    if before_summary is not None and after_summary != before_summary:
        raise RuntimeError("OPEN-1 run mutated published docs/summary_stats.json")
    if before_figure is not None and after_figure != before_figure:
        raise RuntimeError("OPEN-1 run mutated published figures/optimal_landing_zones.png")


def run_one_campaign(
    name: str,
    root: Path,
    *,
    preset: str,
    n_trials: int,
    seed: int,
    zero_drift: bool,
) -> dict:
    camp_dir = root / name
    if camp_dir.exists():
        shutil.rmtree(camp_dir)
    camp_dir.mkdir(parents=True, exist_ok=True)

    print(f"\n=== OPEN-1 campaign: {name} (preset={preset}, trials={n_trials}, "
          f"zero_drift={zero_drift}) ===")
    world = write_world(preset, camp_dir)
    xml_path = camp_dir / "lunar_world.xml"
    csv_path = camp_dir / "results.csv"
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    records = run_campaign(
        model,
        n_trials=n_trials,
        seed=seed,
        csv_path=csv_path,
        record_summary=False,
        zero_horizontal_velocity=zero_drift,
    )
    rows = load_rows(csv_path)
    _code, stats = run_validation(
        rows,
        preset=preset,
        stats_out_path=camp_dir / "validation_stats.json",
        merge_summary=True,
        summary_path=camp_dir / "summary_stats.json",
    )
    # Also stash terrain stats from world generation.
    (camp_dir / "terrain_stats.json").write_text(json.dumps(world, indent=2))

    summary = {
        "campaign": name,
        "preset": preset,
        "n_trials": n_trials,
        "seed": seed,
        "zero_horizontal_velocity": zero_drift,
        "output_dir": str(camp_dir),
        "relief_m": world["relief_m"],
        "failure_rate_pct": stats["failure_rate_pct"],
        "n_bottomed_out": stats["n_bottomed_out"],
        "stroke_max_m": stats["stroke_max_m"],
        "corr_force_spawn_slope_r": stats["corr_force_slope_r"],
        "corr_force_spawn_slope_p": stats["corr_force_slope_p"],
        "corr_force_touchdown_slope_r": stats["corr_force_touchdown_slope_r"],
        "corr_force_touchdown_slope_p": stats["corr_force_touchdown_slope_p"],
        "corr_force_feet_mean_slope_r": stats["corr_force_feet_mean_slope_r"],
        "corr_force_feet_mean_slope_p": stats["corr_force_feet_mean_slope_p"],
        "corr_force_feet_plane_residual_r": stats["corr_force_feet_plane_residual_r"],
        "corr_force_drift_r": stats["corr_force_drift_r"],
        "mean_spawn_touchdown_offset_m": stats["mean_spawn_touchdown_offset_m"],
        "open1_signal_ok": stats["open1_signal_ok"],
        "open1_conditions": stats["open1_conditions"],
        "drift_quartile_bins": stats["drift_quartile_bins"],
        "n_records": len(records),
    }
    (camp_dir / "campaign_summary.json").write_text(json.dumps(summary, indent=2))
    return summary


def run_open1(
    root: Path | None = None,
    n_trials: int = 500,
    seed: int = 42,
    campaigns: list[str] | None = None,
) -> dict:
    root = Path(root) if root is not None else DEFAULT_ROOT
    root.mkdir(parents=True, exist_ok=True)

    before_summary = PUBLISHED_SUMMARY.read_bytes() if PUBLISHED_SUMMARY.exists() else None
    before_figure = PUBLISHED_FIGURE.read_bytes() if PUBLISHED_FIGURE.exists() else None

    wanted = campaigns or ["nominal-remeasure", "harsh", "low-drift"]
    specs = {
        "nominal-remeasure": dict(preset="nominal", zero_drift=False),
        "harsh": dict(preset="harsh", zero_drift=False),
        "low-drift": dict(preset="nominal", zero_drift=True),
    }

    results = {}
    for name in wanted:
        if name not in specs:
            raise ValueError(f"unknown campaign {name!r}; expected one of {sorted(specs)}")
        results[name] = run_one_campaign(
            name, root, n_trials=n_trials, seed=seed, **specs[name]
        )
        _assert_published_untouched(before_summary, before_figure)

    any_hit = any(r["open1_signal_ok"] for r in results.values())
    report = {
        "design": "OPEN-1 approved experimental plan (issue #3)",
        "n_trials": n_trials,
        "seed": seed,
        "output_root": str(root),
        "adjacent_foot_spacing_m": 4.0,
        "signal_r_threshold": 0.2,
        "published_artifacts_unchanged": True,
        "open1_any_condition_hit": any_hit,
        "verdict": (
            "HIT: at least one documented condition reached |r| >= 0.2"
            if any_hit
            else "NULL: no documented terrain/controller condition reached |r| >= 0.2"
        ),
        "campaigns": results,
        "commands": {
            "all": (
                f"LUNAR_OUTPUT_DIR={root} python -m lunar_lander.open1_campaigns "
                f"--trials {n_trials} --seed {seed}"
            ),
            "smoke": (
                f"python -m lunar_lander.open1_campaigns --trials 2 --seed 42 "
                f"--campaigns nominal-remeasure,harsh,low-drift --output-root {root}"
            ),
        },
        "notes": [
            "Validator always loads the matching heightmap preset (harsh campaign "
            "uses the harsh grid).",
            "Low-drift zeros vx/vy AFTER the seed-42 RNG draws so spawn points match.",
            "Stiffness, damping, stroke limit, and controller gains were not retuned.",
            "Adjacent feet are 4.0 m apart in the MJCF (not 2.5 m).",
        ],
    }
    (root / "open1_report.json").write_text(json.dumps(report, indent=2))
    print(f"\nWrote {root / 'open1_report.json'}")
    print(f"Verdict: {report['verdict']}")
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description="OPEN-1 experimental campaigns")
    parser.add_argument("--trials", type=int, default=500)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument(
        "--output-root",
        type=Path,
        default=None,
        help="isolated output root (default: output/open1 or $LUNAR_OUTPUT_DIR/open1)",
    )
    parser.add_argument(
        "--campaigns",
        default="nominal-remeasure,harsh,low-drift",
        help="comma-separated campaign names",
    )
    args = parser.parse_args(argv)

    if args.output_root is not None:
        root = args.output_root
    elif os.environ.get("LUNAR_OUTPUT_DIR"):
        root = Path(os.environ["LUNAR_OUTPUT_DIR"]).expanduser().resolve() / "open1"
    else:
        root = DEFAULT_ROOT

    names = [c.strip() for c in args.campaigns.split(",") if c.strip()]
    return run_open1(root=root, n_trials=args.trials, seed=args.seed, campaigns=names)


if __name__ == "__main__":
    main()

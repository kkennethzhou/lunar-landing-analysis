"""OPEN-1 measurement, validator preset, and output-isolation checks."""

from __future__ import annotations

import csv
import json
from pathlib import Path

import mujoco
import numpy as np
import pytest

from lunar_lander.generate_world import make_heightmap, slope_degrees, write_world
from lunar_lander.run_monte_carlo import ADJACENT_FOOT_SPACING_M, run_campaign
from lunar_lander.terrain_metrics import (
    feet_terrain,
    local_roughness,
    sample_grid,
    spawn_touchdown_offset_m,
)
from lunar_lander.validate_results import load_terrain, run_validation

PROJECT = Path(__file__).resolve().parents[1]
PUBLISHED_SUMMARY = PROJECT / "docs" / "summary_stats.json"
PUBLISHED_FIGURE = PROJECT / "figures" / "optimal_landing_zones.png"


def test_adjacent_feet_are_four_metres():
    assert ADJACENT_FOOT_SPACING_M == 4.0


def test_sample_grid_and_roughness_shapes():
    H = make_heightmap("nominal")
    slope = slope_degrees(H)
    assert sample_grid(slope, 50.2, 60.7) == float(slope[61, 50])
    assert local_roughness(H, 50.0, 60.0) >= 0.0
    ft = feet_terrain(H, slope, [(48.0, 60.0), (52.0, 60.0), (48.0, 64.0), (52.0, 64.0)])
    assert "mean_slope_deg" in ft
    assert ft["plane_residual_rms_m"] >= 0.0


def test_harsh_load_terrain_differs_from_nominal():
    H_n, S_n = load_terrain("nominal")
    H_h, S_h = load_terrain("harsh")
    assert H_h.max() > H_n.max() * 1.5
    assert not np.array_equal(S_n, S_h)


@pytest.mark.slow
def test_touchdown_columns_and_spawn_offset(tmp_path):
    write_world("nominal", tmp_path)
    model = mujoco.MjModel.from_xml_path(str(tmp_path / "lunar_world.xml"))
    out = tmp_path / "results.csv"
    run_campaign(
        model, n_trials=3, seed=42, csv_path=out,
        record_summary=False, progress=False,
    )
    with open(out) as f:
        rows = list(csv.DictReader(f))
    assert len(rows) == 3
    for row in rows:
        assert "touchdown_x" in row and "foot1_x" in row
        assert "drawn_vx" in row
        offset = spawn_touchdown_offset_m(row)
        assert offset >= 0.0
        # With up to 2.3 m/s drift over ~10 s, offset can be large; just
        # require the logged touchdown to be a finite number on the map.
        assert 0.0 <= float(row["touchdown_x"]) <= 127.0
        assert 0.0 <= float(row["touchdown_y"]) <= 127.0


@pytest.mark.slow
def test_zero_drift_preserves_spawn_and_zeros_velocity(tmp_path):
    write_world("nominal", tmp_path)
    model = mujoco.MjModel.from_xml_path(str(tmp_path / "lunar_world.xml"))
    base = tmp_path / "base.csv"
    zero = tmp_path / "zero.csv"
    run_campaign(
        model, n_trials=5, seed=42, csv_path=base,
        record_summary=False, progress=False,
    )
    run_campaign(
        model, n_trials=5, seed=42, csv_path=zero,
        record_summary=False, progress=False,
        zero_horizontal_velocity=True,
    )
    with open(base) as f:
        base_rows = list(csv.DictReader(f))
    with open(zero) as f:
        zero_rows = list(csv.DictReader(f))
    for b, z in zip(base_rows, zero_rows, strict=True):
        assert b["x"] == z["x"] and b["y"] == z["y"]
        assert float(z["vx"]) == 0.0 and float(z["vy"]) == 0.0
        assert abs(float(z["drawn_vx"]) - float(b["vx"])) < 1e-9
        assert abs(float(z["drawn_vy"]) - float(b["vy"])) < 1e-9


@pytest.mark.slow
def test_validator_harsh_preset_and_published_isolation(tmp_path):
    before_summary = PUBLISHED_SUMMARY.read_bytes()
    before_figure = PUBLISHED_FIGURE.read_bytes() if PUBLISHED_FIGURE.exists() else None

    write_world("harsh", tmp_path)
    model = mujoco.MjModel.from_xml_path(str(tmp_path / "lunar_world.xml"))
    csv_path = tmp_path / "results.csv"
    run_campaign(
        model, n_trials=3, seed=42, csv_path=csv_path,
        record_summary=False, progress=False,
    )
    with open(csv_path) as f:
        rows = list(csv.DictReader(f))
    code, stats = run_validation(
        rows,
        preset="harsh",
        stats_out_path=tmp_path / "validation_stats.json",
        merge_summary=True,
        summary_path=tmp_path / "summary_stats.json",
    )
    assert stats["terrain_preset"] == "harsh"
    assert stats["has_touchdown_columns"] is True
    assert "open1_signal_ok" in stats
    assert (tmp_path / "validation_stats.json").exists()
    assert (tmp_path / "summary_stats.json").exists()
    # Exit code may be 1 (null signal); that is fine.
    assert code in (0, 1)

    assert PUBLISHED_SUMMARY.read_bytes() == before_summary
    if before_figure is not None:
        assert PUBLISHED_FIGURE.read_bytes() == before_figure


def test_open1_runner_smoke_does_not_touch_published(tmp_path):
    from lunar_lander.open1_campaigns import run_open1

    before_summary = PUBLISHED_SUMMARY.read_bytes()
    before_figure = PUBLISHED_FIGURE.read_bytes() if PUBLISHED_FIGURE.exists() else None
    report = run_open1(
        root=tmp_path / "open1",
        n_trials=1,
        seed=42,
        campaigns=["nominal-remeasure"],
    )
    assert report["published_artifacts_unchanged"] is True
    assert (tmp_path / "open1" / "open1_report.json").exists()
    assert PUBLISHED_SUMMARY.read_bytes() == before_summary
    if before_figure is not None:
        assert PUBLISHED_FIGURE.read_bytes() == before_figure
    # Ensure published JSON was not rewritten by merge_summary.
    published = json.loads(before_summary)
    assert published["n_trials"] == 500

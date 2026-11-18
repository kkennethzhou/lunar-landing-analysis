"""A 10-trial seed-42 campaign stays on the recorded trajectory.

Marked slow: it steps the physics. CI runs it on one Python version.
Forces are compared with a 1 N tolerance so a different CPU can differ in
the last bit without hiding a real model change.
"""

from pathlib import Path

import mujoco
import pytest

from lunar_lander.paths import XML_PATH
from lunar_lander.run_monte_carlo import run_campaign

FIXTURE = Path(__file__).resolve().parent / "fixtures" / "trials_10_seed42.csv"


def _rows(path):
    import csv
    with open(path) as handle:
        return list(csv.DictReader(handle))


@pytest.mark.slow
def test_ten_trials_match_fixture(tmp_path):
    if not XML_PATH.exists():
        pytest.skip("run python -m lunar_lander.generate_world first")
    model = mujoco.MjModel.from_xml_path(str(XML_PATH))
    out = tmp_path / "results.csv"
    run_campaign(model, n_trials=10, seed=42, csv_path=out, record_summary=False, progress=False)
    got = _rows(out)
    expected = _rows(FIXTURE)
    assert len(got) == len(expected) == 10
    for left, right in zip(got, expected, strict=True):
        assert left["outcome"] == right["outcome"]
        assert left["x"] == right["x"]
        assert left["y"] == right["y"]
        assert abs(float(left["peak_strut_force_N"]) - float(right["peak_strut_force_N"])) < 1.0
        assert abs(float(left["peak_stroke_m"]) - float(right["peak_stroke_m"])) < 0.001

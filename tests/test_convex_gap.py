"""Convex-hull collision erases crater floors; the heightfield does not."""

from lunar_lander.convex_gap import crater_floor_gaps, measure


def test_geometric_hull_bridges_craters():
    gaps = crater_floor_gaps("nominal")
    assert len(gaps) == 7
    assert max(row["gap_m"] for row in gaps) > 0.3


def test_contact_agrees_with_the_hull():
    result = measure("nominal")
    mesh_gap = result["contact"]["mesh"]["gap_above_true_floor_m"]
    hfield_gap = result["contact"]["hfield"]["gap_above_true_floor_m"]
    assert mesh_gap > 0.3
    assert abs(hfield_gap) < 0.05
    assert mesh_gap > abs(hfield_gap) * 5

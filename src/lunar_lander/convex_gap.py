"""Measure the crater-floor error of a convex-hull mesh against the heightfield.

MuJoCo collides mesh geoms against the convex hull. On this terrain the hull
is a lid over every crater. The heightfield is the corrected collision geom.
This module records both the geometric hull gap and a one-degree-of-freedom
contact check, and writes them to JSON.
"""

from __future__ import annotations

import json
import tempfile
from pathlib import Path

import mujoco
import numpy as np
from scipy.spatial import ConvexHull

from lunar_lander.generate_world import (
    CRATERS,
    GRID,
    SPACING,
    angle_to_world,
    make_heightmap,
    write_heightfield_png,
    write_obj,
)
from lunar_lander.paths import PROJECT_ROOT

PROBE_RADIUS_M = 0.12


def _upper_hull_z(hull: ConvexHull, x: float, y: float) -> float:
    """Height of the convex hull's upper surface at (x, y).

    Each hull equation is n·p + c <= 0 inside the hull. Equations with
    nz > 0 cap z from above. The lid is the lowest of those caps.
    """
    upper = np.inf
    for nx, ny, nz, offset in hull.equations:
        if abs(nz) < 1e-12:
            if nx * x + ny * y + offset > 1e-4:
                raise RuntimeError(f"({x:.2f}, {y:.2f}) is outside the hull footprint")
            continue
        bound = -(nx * x + ny * y + offset) / nz
        if nz > 0:
            upper = min(upper, bound)
    if not np.isfinite(upper):
        raise RuntimeError(f"no upper hull bound at ({x:.2f}, {y:.2f})")
    return float(upper)


def crater_floor_gaps(preset: str = "nominal") -> list[dict]:
    """Geometric gap between each crater sample and the convex hull, in metres."""
    height = make_heightmap(preset)
    n = height.shape[0]
    xs = np.arange(n) * SPACING
    # meshgrid indexing="xy": xx[j, i] = i*spacing, yy[j, i] = j*spacing,
    # matching write_obj where H[j, i] is the vertex at that (x, y).
    xx, yy = np.meshgrid(xs, xs, indexing="xy")
    points = np.column_stack([xx.ravel(), yy.ravel(), height.ravel()])
    hull = ConvexHull(points)
    rows = []
    for cx, cy, _radius, _depth in CRATERS:
        x = angle_to_world(cx)
        y = angle_to_world(cy)
        ix = min(max(int(round(x / SPACING)), 0), n - 1)
        iy = min(max(int(round(y / SPACING)), 0), n - 1)
        true_z = float(height[iy, ix])
        hull_z = _upper_hull_z(hull, x, y)
        rows.append({
            "x_m": x,
            "y_m": y,
            "true_z_m": true_z,
            "hull_z_m": hull_z,
            "gap_m": hull_z - true_z,
        })
    return rows


def _probe_xml(kind: str, x: float, y: float, relief: float) -> str:
    if kind == "mesh":
        asset = '<mesh name="terrain" file="lunar_surface.obj"/>'
        ground = '<geom name="ground" type="mesh" mesh="terrain" contype="1" conaffinity="1"/>'
    elif kind == "hfield":
        half = (GRID - 1) * SPACING / 2.0
        asset = (
            f'<hfield name="terrain" file="lunar_heightfield.png" '
            f'size="{half:.4f} {half:.4f} {relief:.6f} 1"/>'
        )
        ground = (
            f'<geom name="ground" type="hfield" hfield="terrain" '
            f'pos="{half:.4f} {half:.4f} 0" contype="1" conaffinity="1"/>'
        )
    else:
        raise ValueError(kind)
    return f"""<mujoco>
  <compiler meshdir="physics_mesh"/>
  <option gravity="0 0 -1.62" timestep="0.001"/>
  <asset>{asset}</asset>
  <worldbody>
    {ground}
    <body name="probe" pos="{x:.4f} {y:.4f} 0">
      <joint name="z" type="slide" axis="0 0 1" damping="40"/>
      <geom type="sphere" size="{PROBE_RADIUS_M}" mass="20" pos="0 0 0"/>
    </body>
  </worldbody>
</mujoco>
"""


def contact_surface_z(xml_path: Path, start_z: float) -> float:
    """Drop a vertically constrained sphere and return the surface it comes to rest on."""
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    data.qpos[0] = start_z
    data.qvel[0] = 0.0
    for _ in range(8000):
        mujoco.mj_step(model, data)
    return float(data.qpos[0] - PROBE_RADIUS_M)


def measure(preset: str = "nominal") -> dict:
    """Geometric hull gaps, plus a contact check at the worst crater."""
    gaps = crater_floor_gaps(preset)
    worst = max(gaps, key=lambda row: row["gap_m"])
    height = make_heightmap(preset)
    relief = float(height.max() - height.min())
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        mesh = root / "physics_mesh"
        mesh.mkdir()
        write_obj(str(mesh / "lunar_surface.obj"), height, SPACING)
        write_heightfield_png(mesh / "lunar_heightfield.png", height)
        start_z = worst["hull_z_m"] + 1.0
        contact = {}
        for kind in ("mesh", "hfield"):
            xml_path = root / f"{kind}.xml"
            xml_path.write_text(_probe_xml(kind, worst["x_m"], worst["y_m"], relief))
            surface_z = contact_surface_z(xml_path, start_z)
            contact[kind] = {
                "surface_z_m": surface_z,
                "gap_above_true_floor_m": surface_z - worst["true_z_m"],
            }
    return {
        "preset": preset,
        "n_craters": len(gaps),
        "max_geometric_gap_m": worst["gap_m"],
        "mean_geometric_gap_m": float(np.mean([row["gap_m"] for row in gaps])),
        "worst_crater": worst,
        "contact": contact,
        "craters": gaps,
        "hfield_quantization_m": relief / 255.0,
    }


def main(argv=None) -> dict:
    import argparse
    parser = argparse.ArgumentParser(description="Measure convex-hull crater flattening")
    parser.add_argument("--preset", default="nominal")
    parser.add_argument(
        "--output",
        default=str(PROJECT_ROOT / "docs" / "convex_gap.json"),
    )
    args = parser.parse_args(argv)
    result = measure(args.preset)
    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, indent=2))
    print(
        f"Max geometric gap {result['max_geometric_gap_m']:.3f} m; "
        f"contact mesh {result['contact']['mesh']['gap_above_true_floor_m']:.3f} m, "
        f"hfield {result['contact']['hfield']['gap_above_true_floor_m']:.3f} m"
    )
    print(f"Wrote {out}")
    return result


if __name__ == "__main__":
    main()

"""
Procedural lunar world generator.

Importing this module does not write files. Call main() or write_world().
"""

import argparse
import math
from pathlib import Path

import mujoco
import numpy as np

from lunar_lander.paths import (
    HEIGHTFIELD_PATH,
    OBJ_PATH,
    XML_PATH,
)
from lunar_lander.paths import (
    TERRAIN_STATS as TERRAIN_STATS_PATH,
)

GRID      = 128          # vertices per side
SPACING   = 1.0         # metres between grid points  →  127 m × 127 m terrain

# (cx, cy, radius, depth) in the layered_noise angle coordinates.
CRATERS = [
    (1.1, 1.4, 0.35, 1.8),
    (3.8, 2.1, 0.25, 1.2),
    (5.2, 4.7, 0.40, 2.0),
    (2.5, 5.3, 0.20, 0.9),
    (4.1, 0.9, 0.30, 1.5),
    (0.6, 3.6, 0.18, 0.7),
    (6.0, 1.8, 0.28, 1.3),
]

# 1. Heightmap

def layered_noise(N: int, crater_scale: float = 1.0, ridge_scale: float = 1.0) -> np.ndarray:
    """
    Combine several sine/cosine octaves + circular crater dips.
    Returns an (N, N) float array, values roughly in [-3, 3] metres.
    """
    x = np.linspace(0, 2 * math.pi, N, endpoint=False)
    y = np.linspace(0, 2 * math.pi, N, endpoint=False)
    xg, yg = np.meshgrid(x, y)          # shape (N, N)

    h  = np.zeros((N, N), dtype=np.float64)

    # broad undulation
    h += 1.2 * np.sin(1.3 * xg) * np.cos(0.9 * yg)
    h += 0.8 * np.cos(2.1 * xg + 0.4) * np.sin(1.7 * yg + 1.1)

    # medium ridges
    h += 0.4 * ridge_scale * np.sin(4.3 * xg + 0.7) * np.cos(3.8 * yg + 2.3)
    h += 0.3 * ridge_scale * np.cos(5.1 * xg + 1.2) * np.sin(4.6 * yg + 0.5)

    # fine pebble texture
    h += 0.10 * np.sin(11.0 * xg + 2.1) * np.cos(9.7 * yg + 1.3)
    h += 0.07 * np.cos(13.0 * xg + 0.3) * np.sin(12.0 * yg + 0.8)

    # crater dips  (cx, cy in [0,2π], radius r, depth d)
    # bowl profile: a central Gaussian pit of depth d, plus a raised annular
    # rim centred at 1.2*r (an ejecta ring, not a second concentric pit).
    # At dist=0 the rim term is ~exp(-9)≈0 so centre depth stays ≈ d.
    # At dist=1.2r the rim term peaks, partially cancelling the (already
    # small) pit contribution there and producing a genuine raised lip.
    for cx, cy, r, d in CRATERS:
        d = d * crater_scale
        dist = np.sqrt((xg - cx) ** 2 + (yg - cy) ** 2)
        pit = d * np.exp(-((dist / r) ** 2))
        rim = 0.3 * d * np.exp(-(((dist - 1.2 * r) / (0.4 * r)) ** 2))
        bowl = pit - rim
        h -= bowl

    # boulder bumps
    boulders = [
        (2.0, 4.0, 0.10, 0.5),
        (4.8, 2.8, 0.08, 0.4),
        (1.5, 0.7, 0.07, 0.3),
        (5.5, 5.1, 0.09, 0.45),
    ]
    for bx, by, r, bump in boulders:
        dist = np.sqrt((xg - bx) ** 2 + (yg - by) ** 2)
        h += bump * np.exp(-((dist / r) ** 2))

    return h

# 2. OBJ export

def write_obj(path: str, H: np.ndarray, spacing: float) -> None:
    N     = H.shape[0]
    lines = []

    # vertices
    for j in range(N):
        for i in range(N):
            x = i * spacing
            y = j * spacing
            z = H[j, i]
            lines.append(f"v {x:.4f} {y:.4f} {z:.4f}")

    # per-vertex normals (central differences)
    Nx_arr = np.zeros_like(H)
    Ny_arr = np.zeros_like(H)
    Nz_arr = np.ones_like(H)
    Nx_arr[:, 1:-1] = (H[:, :-2] - H[:, 2:]) / (2 * spacing)
    Ny_arr[1:-1, :] = (H[:-2, :] - H[2:, :]) / (2 * spacing)
    mag = np.sqrt(Nx_arr**2 + Ny_arr**2 + Nz_arr**2)
    Nx_arr /= mag; Ny_arr /= mag; Nz_arr /= mag

    for j in range(N):
        for i in range(N):
            lines.append(f"vn {Nx_arr[j,i]:.4f} {Ny_arr[j,i]:.4f} {Nz_arr[j,i]:.4f}")

    # faces (two triangles per quad), 1-based index
    def idx(i, j):
        return j * N + i + 1

    for j in range(N - 1):
        for i in range(N - 1):
            a = idx(i,   j);   b = idx(i+1, j)
            c = idx(i+1, j+1); d = idx(i,   j+1)
            lines.append(f"f {a}//{a} {b}//{b} {c}//{c}")
            lines.append(f"f {a}//{a} {c}//{c} {d}//{d}")

    with open(path, "w") as f:
        f.write("# Procedural lunar surface\n")
        f.write(f"# Grid {N}x{N}, spacing {spacing} m\n\n")
        f.write("\n".join(lines))
        f.write("\n")

PRESETS = {
    "nominal": {"crater_scale": 1.0, "ridge_scale": 1.0},
    # Deeper craters and steeper ridges for compare_roughness.py.
    # Nominal scales are exactly 1, so the shipped terrain is unchanged.
    "harsh": {"crater_scale": 3.0, "ridge_scale": 2.0},
}

def make_heightmap(preset: str = "nominal", n: int = GRID) -> np.ndarray:
    """Elevation grid with its minimum shifted to z=0."""
    if preset not in PRESETS:
        raise ValueError(f"unknown preset {preset!r}; expected one of {sorted(PRESETS)}")
    h = layered_noise(n, **PRESETS[preset])
    return h - h.min()

def slope_degrees(H: np.ndarray, spacing: float = SPACING) -> np.ndarray:
    """Per-cell slope in degrees from the grid gradient, not relief/width."""
    gy, gx = np.gradient(H, spacing)
    return np.degrees(np.arctan(np.hypot(gx, gy)))

def angle_to_world(angle: float, n: int = GRID, spacing: float = SPACING) -> float:
    """Map a layered_noise angle in [0, 2 pi) to a world coordinate in metres."""
    return angle / (2.0 * math.pi) * n * spacing

def write_heightfield_png(path: str, H: np.ndarray) -> None:
    from PIL import Image
    relief = H.max() - H.min()
    norm = ((H - H.min()) / relief * 255.0).round().astype(np.uint8) if relief > 0 else np.zeros_like(H, dtype=np.uint8)
    # MuJoCo hfield row 0 = local -Y edge, and PNG row 0 = image top.
    # H[j, i] has j increasing with world Y (per write_obj: y = j*spacing),
    # so flip vertically here to match MuJoCo's expected row order.
    img = Image.fromarray(np.flipud(norm), mode="L")
    img.save(path)

def _subtree_ids(model, root_id):
    ids = [root_id]
    for bid in range(model.nbody):
        parent = bid
        while parent != 0:
            parent = model.body_parentid[parent]
            if parent == root_id:
                ids.append(bid)
                break
    return sorted(set(ids))

def write_world(preset: str = "nominal", output_dir=None) -> dict:
    """Write the OBJ, the heightfield PNG, and the MJCF.

    output_dir defaults to the package output directory. Pass another
    directory to leave the nominal world in place.
    """
    if output_dir is None:
        xml_path = XML_PATH
        obj_path = OBJ_PATH
        png_path = HEIGHTFIELD_PATH
    else:
        out = Path(output_dir)
        mesh = out / "physics_mesh"
        mesh.mkdir(parents=True, exist_ok=True)
        xml_path = out / "lunar_world.xml"
        obj_path = mesh / "lunar_surface.obj"
        png_path = mesh / "lunar_heightfield.png"

    print(f"Generating heightmap ({preset}) …")
    H = make_heightmap(preset)
    relief_m = float(H.max() - H.min())
    slope_deg = slope_degrees(H)
    slope_mean = float(slope_deg.mean())
    slope_median = float(np.median(slope_deg))
    slope_p95 = float(np.percentile(slope_deg, 95))
    slope_max = float(slope_deg.max())
    pct_over_5 = float((slope_deg > 5).mean() * 100)
    pct_over_10 = float((slope_deg > 10).mean() * 100)
    pct_over_15 = float((slope_deg > 15).mean() * 100)
    print(f"  Terrain z range: {H.min():.3f} - {H.max():.3f} m")
    domain_tilt = math.degrees(math.atan(relief_m / ((GRID - 1) * SPACING)))
    print(
        f"  Slope statistics (degrees from horizontal):\n"
        f"    mean {slope_mean:.2f}  median {slope_median:.2f}  "
        f"p95 {slope_p95:.2f}  max {slope_max:.2f}\n"
        f"    >5° {pct_over_5:.1f}%  >10° {pct_over_10:.1f}%  >15° {pct_over_15:.1f}%\n"
        f"  relief/width = {domain_tilt:.2f}° is the domain tilt, not a slope."
    )

    print("Writing OBJ …")
    write_obj(str(obj_path), H, SPACING)
    size_mb = obj_path.stat().st_size / 1e6
    print(f"  Saved: {obj_path}  ({size_mb:.1f} MB)")

    # 2b. Heightfield PNG for collision
    # MuJoCo's type="mesh" geoms collide against the mesh's CONVEX HULL, not
    # the true (concave) surface: verified empirically: inside a crater, the
    # physics contact solver placed the surface ~0.5 m above the actual mesh
    # vertex height there. That silently invalidates any local-terrain-slope
    # correlation (Phase 4), since the collision surface doesn't match the
    # generated terrain wherever it's concave. type="hfield" gives MuJoCo's
    # native, exact (non-convex) heightfield collision, which is the correct
    # primitive for terrain generated from a heightmap in the first place.
    # The OBJ mesh is still written above (visual reference / triangle-count
    # provenance); the PNG below drives actual collision + rendering.

    print("Writing heightfield PNG …")
    write_heightfield_png(png_path, H)
    print(f"  Saved: {png_path}")

    HFIELD_HALF_X = (GRID - 1) * SPACING / 2.0
    HFIELD_HALF_Y = (GRID - 1) * SPACING / 2.0
    HFIELD_ELEV_Z = relief_m
    HFIELD_BASE_Z = 1.0   # skirt thickness below z=0, closes off the collision solid

    # 3. MJCF XML

    # Centre of terrain (for lander spawn)
    cx = (GRID - 1) * SPACING / 2.0
    cy = (GRID - 1) * SPACING / 2.0

    # Height directly under spawn point (grid centre)
    mid = GRID // 2
    terrain_z_at_spawn = float(H[mid, mid])
    lander_z = terrain_z_at_spawn + 30.0   # 30 m above terrain

    # Lander mass budget
    # Target total = 1000.000 kg exactly. No <inertial> override on lander_base.
    # MuJoCo derives mass/inertia per-body from geom primitives (inertiafromgeom
    # default "auto"), which also fixes the previously wrong hand-picked yaw
    # inertia (diaginertia Izz was 5x too low for an 800 kg 2.4x2.4x1.2 m box).
    # Fixed masses: engine 50, tank 200, 4 struts x 30 = 120, 4 feet x 10 = 40
    # Sum of fixed = 410 kg -> bus must be 1000 - 410 = 590 kg to hit 1000.000 kg.
    ENGINE_MASS = 50.0
    TANK_MASS   = 200.0
    STRUT_MASS  = 30.0   # each, x4
    FOOT_MASS   = 10.0   # each, x4
    BUS_MASS    = 1000.0 - ENGINE_MASS - TANK_MASS - 4 * STRUT_MASS - 4 * FOOT_MASS  # 590.0

    # Descent engine
    # Static lunar weight = 1000 kg x 1.62 m/s^2 = 1620 N.
    # Max thrust set to ~3x static weight so the controller has authority to
    # both hover and brake a meaningful descent rate.
    ENGINE_MAX_THRUST = 5000.0   # N

    # Landing leg spring-damper sizing (BLOCKING-1, docs/known-issues.md)
    # The previous sizing (320 N/m) was derived from landing kinetic energy
    # alone and never checked whether the legs could hold the vehicle up at
    # rest. Static per-leg load is F = m*g/4 = 1000*1.62/4 = 405 N; at 320 N/m
    # that requires 1.27 m of spring compression against a 0.8 m stroke limit.
    # physically impossible. Both constraints below must hold simultaneously.
    #
    # Static constraint: compression at rest should use <=20% of stroke, so the
    # rest is available to absorb an actual landing:
    #   k >= 405 / (0.20 * 0.8) = 2531 N/m  ->  round up to 2500 N/m minimum
    #
    # Dynamic constraint: at the 1.5 m/s design touchdown speed, per-leg energy
    # balance (KE_total split between 4 springs, weight doing work W*x as the
    # legs compress by x) is
    #   2*k*x^2 - W*x - KE = 0,   W = m*g = 1620 N,   KE = 0.5*m*v^2 = 1125 J
    # solved for x and required to stay under ~75% of the 0.8 m stroke.
    #
    # Trade study (static x = 405/k, dynamic x solves the quadratic above):
    #   k=2000: static 0.203 m (25%), dynamic 0.770 m (96%)  -- dynamic too tight
    #   k=3000: static 0.135 m (17%), dynamic 0.589 m (74%)  -- satisfies both
    #   k=4000: static 0.101 m (13%), dynamic 0.490 m (61%)
    #
    # k=3000 N/m satisfies both constraints with margin. Damping for zeta~=0.7
    # (near-critical: suppresses rebound/tip-over without over-damping):
    #   c = 0.7 * 2*sqrt(k * m_eff) = 0.7*2*sqrt(3000*250) = 1212 N.s/m -> 1200
    #
    # Regression: tests/test_static_equilibrium.py asserts both constraints
    # directly and drops the lander with the engine off to confirm each leg
    # settles at 405/k without bottoming out. It's the test whose absence let
    # 320 N/m ship in the first place.
    LEG_STIFFNESS = 3000   # N/m
    LEG_DAMPING   = 1200   # N·s/m

    xml = f"""<mujoco model="lunar_world">

      <compiler angle="radian" meshdir="physics_mesh" autolimits="true"/>

      <option gravity="0 0 -1.62" timestep="0.002"/>

      <asset>
        <mesh name="lunar_surface_mesh" file="lunar_surface.obj" scale="1 1 1"/>
        <hfield name="terrain_hf" file="lunar_heightfield.png"
                size="{HFIELD_HALF_X:.2f} {HFIELD_HALF_Y:.2f} {HFIELD_ELEV_Z:.4f} {HFIELD_BASE_Z:.2f}"/>
        <texture name="moon_tex" type="2d" builtin="checker"
                 rgb1="0.55 0.55 0.55" rgb2="0.40 0.40 0.40"
                 width="512" height="512"/>
        <material name="moon_mat" texture="moon_tex" texrepeat="8 8"
                  specular="0.1" shininess="0.05"/>
        <material name="lander_mat" rgba="0.85 0.85 0.75 1" specular="0.3"/>
        <material name="leg_mat"    rgba="0.6  0.6  0.5  1" specular="0.2"/>
      </asset>

      <worldbody>

        <!-- Sky / ambient lighting -->
        <light name="sun" pos="60 60 120" dir="-0.4 -0.4 -1"
               diffuse="0.9 0.9 0.85" specular="0.3 0.3 0.3" castshadow="true"/>
        <light name="fill" pos="-40 20 80" dir="0.3 -0.1 -0.8"
               diffuse="0.25 0.25 0.35" specular="0 0 0" castshadow="false"/>

        <!-- Static terrain (collision + visual). hfield, not mesh: MuJoCo mesh
             geoms collide against their convex hull, which is wrong for
             concave terrain (craters). hfield gives exact, non-convex
             collision that actually matches the generated heightmap. Centred
             at (cx, cy) so its local [-half,+half] span covers world [0,127]. -->
        <body name="terrain" pos="{cx:.2f} {cy:.2f} 0">
          <geom name="ground_col"
                type="hfield" hfield="terrain_hf"
                material="moon_mat"
                contype="1" conaffinity="1"
                friction="0.8 0.005 0.0001"
                solimp="0.99 0.999 0.001" solref="0.02 1"/>
        </body>

        <!-- Lunar lander -->
        <body name="lander_base" pos="{cx:.2f} {cy:.2f} {lander_z:.2f}">

          <!-- 6-DOF free joint so the lander can fall freely -->
          <freejoint name="lander_free"/>

          <!-- Main bus (box approximation of descent stage). No explicit
               <inertial> here on purpose: mass and inertia are both derived
               from this geom + the engine/tank/leg geoms below, so total body
               mass and inertia are physically consistent with the geometry
               (see BUS_MASS derivation above). -->
          <geom name="bus" type="box" size="1.2 1.2 0.6"
                material="lander_mat"
                contype="1" conaffinity="1"
                mass="{BUS_MASS:.1f}" group="1"/>

          <!-- Engine bell (visual only) -->
          <geom name="engine" type="cylinder" size="0.25 0.35" pos="0 0 -0.95"
                material="lander_mat" contype="0" conaffinity="0" mass="{ENGINE_MASS:.1f}" group="1"/>

          <!-- Fuel tank dome -->
          <geom name="tank" type="sphere" size="0.5" pos="0 0 0.7"
                material="lander_mat" contype="0" conaffinity="0" mass="{TANK_MASS:.1f}" group="1"/>

          <!-- Descent engine thrust site: force applied along body -z -->
          <site name="engine_site" pos="0 0 -0.95" size="0.05"/>

          <!-- 4 landing legs -->
          <!-- Each leg: upper strut (rigid) + lower footpad (slide joint with
               spring-damper to simulate shock absorber)                      -->

          <!-- Leg 1 - +X +Y corner -->
          <body name="leg1_upper" pos="1.1 1.1 -0.6">
            <joint name="leg1_slide" type="slide" axis="0 0 -1"
                   range="0 0.8" stiffness="{LEG_STIFFNESS}" damping="{LEG_DAMPING}"/>
            <geom name="leg1_strut" type="capsule"
                  fromto="0 0 0  0.9 0.9 -1.4" size="0.06"
                  material="leg_mat" mass="{STRUT_MASS:.1f}" group="1"/>
            <body name="leg1_foot" pos="0.9 0.9 -1.4">
              <geom name="foot1" type="sphere" size="0.12"
                    material="leg_mat"
                    contype="1" conaffinity="1"
                    friction="0.9 0.005 0.0001"
                    solimp="0.99 0.999 0.001" solref="0.02 1"
                    mass="{FOOT_MASS:.1f}" group="1"/>
            </body>
          </body>

          <!-- Leg 2 - -X +Y corner -->
          <body name="leg2_upper" pos="-1.1 1.1 -0.6">
            <joint name="leg2_slide" type="slide" axis="0 0 -1"
                   range="0 0.8" stiffness="{LEG_STIFFNESS}" damping="{LEG_DAMPING}"/>
            <geom name="leg2_strut" type="capsule"
                  fromto="0 0 0  -0.9 0.9 -1.4" size="0.06"
                  material="leg_mat" mass="{STRUT_MASS:.1f}" group="1"/>
            <body name="leg2_foot" pos="-0.9 0.9 -1.4">
              <geom name="foot2" type="sphere" size="0.12"
                    material="leg_mat"
                    contype="1" conaffinity="1"
                    friction="0.9 0.005 0.0001"
                    solimp="0.99 0.999 0.001" solref="0.02 1"
                    mass="{FOOT_MASS:.1f}" group="1"/>
            </body>
          </body>

          <!-- Leg 3 - -X -Y corner -->
          <body name="leg3_upper" pos="-1.1 -1.1 -0.6">
            <joint name="leg3_slide" type="slide" axis="0 0 -1"
                   range="0 0.8" stiffness="{LEG_STIFFNESS}" damping="{LEG_DAMPING}"/>
            <geom name="leg3_strut" type="capsule"
                  fromto="0 0 0  -0.9 -0.9 -1.4" size="0.06"
                  material="leg_mat" mass="{STRUT_MASS:.1f}" group="1"/>
            <body name="leg3_foot" pos="-0.9 -0.9 -1.4">
              <geom name="foot3" type="sphere" size="0.12"
                    material="leg_mat"
                    contype="1" conaffinity="1"
                    friction="0.9 0.005 0.0001"
                    solimp="0.99 0.999 0.001" solref="0.02 1"
                    mass="{FOOT_MASS:.1f}" group="1"/>
            </body>
          </body>

          <!-- Leg 4 - +X -Y corner -->
          <body name="leg4_upper" pos="1.1 -1.1 -0.6">
            <joint name="leg4_slide" type="slide" axis="0 0 -1"
                   range="0 0.8" stiffness="{LEG_STIFFNESS}" damping="{LEG_DAMPING}"/>
            <geom name="leg4_strut" type="capsule"
                  fromto="0 0 0  0.9 -0.9 -1.4" size="0.06"
                  material="leg_mat" mass="{STRUT_MASS:.1f}" group="1"/>
            <body name="leg4_foot" pos="0.9 -0.9 -1.4">
              <geom name="foot4" type="sphere" size="0.12"
                    material="leg_mat"
                    contype="1" conaffinity="1"
                    friction="0.9 0.005 0.0001"
                    solimp="0.99 0.999 0.001" solref="0.02 1"
                    mass="{FOOT_MASS:.1f}" group="1"/>
            </body>
          </body>

        </body>
        <!-- end lander_base -->

      </worldbody>

      <actuator>
        <motor name="descent_engine" site="engine_site" gear="0 0 1 0 0 0"
               ctrlrange="0 {ENGINE_MAX_THRUST:.0f}" ctrllimited="true"/>
      </actuator>

      <!-- Sensors for telemetry -->
      <sensor>
        <framepos   name="lander_pos"    objtype="body" objname="lander_base"/>
        <framequat  name="lander_quat"   objtype="body" objname="lander_base"/>
        <framelinvel name="lander_linvel" objtype="body" objname="lander_base"/>
        <frameangvel name="lander_angvel" objtype="body" objname="lander_base"/>
        <jointpos   name="leg1_compress" joint="leg1_slide"/>
        <jointpos   name="leg2_compress" joint="leg2_slide"/>
        <jointpos   name="leg3_compress" joint="leg3_slide"/>
        <jointpos   name="leg4_compress" joint="leg4_slide"/>
      </sensor>

    </mujoco>
    """

    print("Writing MJCF XML …")
    with open(xml_path, "w") as f:
        f.write(xml)
    print(f"  Saved: {xml_path}")

    # 4. Verify total mass

    model = mujoco.MjModel.from_xml_path(str(xml_path))

    # Sum mass over the lander's kinematic subtree only (lander_base + legs).
    # model.body_mass.sum() over the WHOLE model also includes the static
    # terrain body, whose mesh geom has no explicit density and is therefore
    # many orders of magnitude heavier: not part of the "lander mass" spec.
    _lander_root = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "lander_base")

    _lander_body_ids = _subtree_ids(model, _lander_root)
    total_mass = float(model.body_mass[_lander_body_ids].sum())
    print(f"  Verifying total lander mass (lander_base + legs subtree): {total_mass:.6f} kg")
    assert abs(total_mass - 1000.0) < 1e-6, (
        f"Total lander mass {total_mass:.6f} kg != 1000.000 kg: mass budget is wrong"
    )
    print("   Mass assertion passed: lander subtree mass == 1000.000 kg")

    # 8-bit PNG: pixel 0 maps to z=0 and pixel 255 maps to the relief.
    hfield_quantization_m = float(relief_m / 255.0) if relief_m > 0 else 0.0

    # 5. Summary

    total_verts = GRID * GRID
    total_faces = (GRID - 1) * (GRID - 1) * 2

    print("World written")
    print(f"  grid: {GRID}x{GRID} vertices, {total_verts:,} verts, {total_faces:,} triangles")
    print(f"  z range: {H.min():.2f} to {H.max():.2f} m")
    print("  craters: 7, boulders: 4")
    print(f"  spawn: ({cx:.1f}, {cy:.1f}, {lander_z:.1f}) m")
    print(f"  lander mass: {total_mass:.3f} kg")
    print("  gravity: -1.62 m/s^2")
    print(f"  leg stiffness: {LEG_STIFFNESS} N/m, damping: {LEG_DAMPING} N s/m")
    print(f"  engine max thrust: {ENGINE_MAX_THRUST:.0f} N")
    print(f"  OBJ: {obj_path}")
    print(f"  MJCF: {xml_path}")
    print(f"  heightfield PNG: 256 levels ({hfield_quantization_m * 100:.2f} cm per level)")

    # 6. Terrain stats for downstream scripts (summary_stats.json)

    TERRAIN_STATS = dict(
        relief_m=relief_m,
        slope_mean_deg=slope_mean,
        slope_median_deg=slope_median,
        slope_p95_deg=slope_p95,
        slope_max_deg=slope_max,
        pct_cells_over_5deg=pct_over_5,
        pct_cells_over_10deg=pct_over_10,
        pct_cells_over_15deg=pct_over_15,
        grid=GRID,
        spacing_m=SPACING,
        domain_m=(GRID - 1) * SPACING,
        lander_mass_kg=total_mass,
        gravity_mps2=1.62,
        timestep_s=0.002,
        leg_stiffness_Npm=LEG_STIFFNESS,
        leg_damping_Nspm=LEG_DAMPING,
        engine_max_thrust_N=ENGINE_MAX_THRUST,
        hfield_levels=256,
        hfield_quantization_m=hfield_quantization_m,
        preset=preset,
    )
    return TERRAIN_STATS

def main(argv=None):
    parser = argparse.ArgumentParser(description="Write the lunar MuJoCo world")
    parser.add_argument("--preset", default="nominal", choices=sorted(PRESETS))
    parser.add_argument(
        "--output-dir",
        default=None,
        help="write XML and meshes here (required for any preset other than nominal)",
    )
    args = parser.parse_args(argv)
    if args.preset != "nominal" and args.output_dir is None:
        parser.error(
            "a non-nominal preset would replace the nominal world; pass --output-dir"
        )
    stats = write_world(args.preset, args.output_dir)
    if args.preset == "nominal" and args.output_dir is None:
        import json
        with open(TERRAIN_STATS_PATH, "w") as f:
            json.dump(stats, f, indent=2)
        print(f"  Saved terrain stats: {TERRAIN_STATS_PATH}")
        from lunar_lander.summary_stats import merge_summary_stats
        merge_summary_stats(stats)
        print("  Merged into summary_stats.json")
    return stats

if __name__ == "__main__":
    main()

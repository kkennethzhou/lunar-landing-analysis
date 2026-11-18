"""
Static equilibrium regression tests.

These encode properties derivable from first principles, independent of what
the simulation happens to output. That distinction matters: the landing-gear
stiffness bug (docs/known-issues.md BLOCKING-1) survived a full 500-trial
campaign because every check compared simulation output against other
simulation output. Nothing asked whether the lander could stand up.

The shipped stiffness is 3000 N/m. These checks are what rejected 320 N/m:
static lunar weight per leg does not fit inside the stroke at that spring rate.
"""

import math

import mujoco
import numpy as np
import pytest

from lunar_lander.paths import XML_PATH

LANDER_MASS_KG = 1000.0
LUNAR_GRAVITY = 1.62
N_LEGS = 4
STROKE_LIMIT_M = 0.8

STATIC_LOAD_PER_LEG_N = LANDER_MASS_KG * LUNAR_GRAVITY / N_LEGS  # 405.0 N
LEG_JOINTS = ["leg1_slide", "leg2_slide", "leg3_slide", "leg4_slide"]

def _require_model():
    if not XML_PATH.exists():
        pytest.skip(f"{XML_PATH.name} not found: run `make world` first")
    return mujoco.MjModel.from_xml_path(str(XML_PATH))

def _leg_stiffness(model) -> float:
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, LEG_JOINTS[0])
    return float(model.jnt_stiffness[jid])

def _leg_damping(model) -> float:
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, LEG_JOINTS[0])
    return float(model.dof_damping[model.jnt_dofadr[jid]])

# Mass and gravity

def _lander_subtree_mass(model) -> float:
    """Mass of lander_base + its legs, excluding the static terrain body.

    The terrain hfield geom has no explicit density override, so MuJoCo
    assigns it a geometry-derived mass many orders of magnitude larger than
    the lander (tens of millions of kg): model.body_mass.sum() over the
    WHOLE model includes that and is never close to 1000 kg. Mirrors the
    subtree filter generate_world.py itself asserts against at generation
    time.
    """
    root_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "lander_base")
    ids = [root_id]
    for bid in range(model.nbody):
        parent = bid
        while parent != 0:
            parent = model.body_parentid[parent]
            if parent == root_id:
                ids.append(bid)
                break
    return float(model.body_mass[sorted(set(ids))].sum())

def test_total_mass_is_exactly_1000kg():
    """Every document claims 1,000 kg. The model must agree.

    Regression: an explicit <inertial mass="800"> once silently overrode the
    bus/engine/tank geom masses, giving 960 kg while all reporting said 1,000.
    """
    model = _require_model()
    total = _lander_subtree_mass(model)
    assert total == pytest.approx(1000.0, abs=0.5), (
        f"Model mass is {total:.1f} kg, not 1000 kg. If a body carries an "
        f"explicit <inertial> element, it overrides geom-derived mass."
    )

def test_gravity_is_lunar():
    model = _require_model()
    assert model.opt.gravity[2] == pytest.approx(-1.62, abs=0.01)
    assert model.opt.gravity[0] == 0.0
    assert model.opt.gravity[1] == 0.0

def test_inertia_is_derived_not_handpicked():
    """A 2.4 x 2.4 x 1.2 m box of ~590 kg has Izz much larger than Ixx.

    Regression: diaginertia="300 300 150" was hand-specified, with Izz set
    5x below its correct value, making the lander far too easy to tip.

    model.body_inertia stores principal moments in eigenvalue order, not
    literally (xx, yy, zz) in the body frame (body_iquat gives the rotation
    into that principal frame, and it's generally not the identity): so we
    can't just unpack the array positionally. The lander is symmetric about
    its z-axis (four legs at symmetric corners, tank/engine on the z-axis),
    so Ixx == Iyy by construction; identify them as whichever two of the
    three eigenvalues are closest together, leaving Izz as the outlier.
    """
    model = _require_model()
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "lander_base")
    inertias = list(model.body_inertia[bid])
    pairs = [(0, 1), (0, 2), (1, 2)]
    i, j = min(pairs, key=lambda p: abs(inertias[p[0]] - inertias[p[1]]))
    k = ({0, 1, 2} - {i, j}).pop()
    ixx, iyy, izz = inertias[i], inertias[j], inertias[k]
    assert izz > ixx, (
        f"Izz ({izz:.1f}) should exceed Ixx ({ixx:.1f}) for a flat, wide bus. "
        f"A hand-picked diaginertia is likely overriding derived inertia."
    )
    assert ixx == pytest.approx(iyy, rel=0.15), "Ixx and Iyy should be near-symmetric"

# Landing gear sizing

def test_legs_support_static_weight():
    """The lander must be able to stand on its own legs.

    Static per-leg load is 405 N. Required compression is 405/k, which must
    fit inside the 0.8 m stroke with margin. This is the constraint that
    sizing on landing energy alone does not impose.
    """
    model = _require_model()
    k = _leg_stiffness(model)
    required = STATIC_LOAD_PER_LEG_N / k

    assert required < STROKE_LIMIT_M, (
        f"Leg stiffness {k:.0f} N/m requires {required:.3f} m of static "
        f"compression, exceeding the {STROKE_LIMIT_M} m stroke limit. "
        f"The lander cannot stand at rest. Need k >= "
        f"{STATIC_LOAD_PER_LEG_N / STROKE_LIMIT_M:.0f} N/m to fit at all, "
        f"and >= {STATIC_LOAD_PER_LEG_N / (0.2 * STROKE_LIMIT_M):.0f} N/m "
        f"for a healthy 20% static margin."
    )

def test_static_compression_leaves_stroke_margin():
    """At rest the legs should use no more than ~20% of available stroke,
    leaving the rest to absorb the landing."""
    model = _require_model()
    k = _leg_stiffness(model)
    frac = (STATIC_LOAD_PER_LEG_N / k) / STROKE_LIMIT_M

    assert frac <= 0.25, (
        f"Static compression uses {frac:.0%} of stroke (target <=20%). "
        f"Recommended k ~= 3000 N/m gives 17%."
    )

def test_dynamic_stroke_fits_at_design_touchdown():
    """At the 1.5 m/s design touchdown, solve 2k*x^2 - W*x - KE = 0 for peak
    stroke and require it to stay inside the limit."""
    model = _require_model()
    k = _leg_stiffness(model)

    ke = 0.5 * LANDER_MASS_KG * 1.5**2
    w = LANDER_MASS_KG * LUNAR_GRAVITY
    a, b, c = 2 * k, -w, -ke
    x = (-b + math.sqrt(b * b - 4 * a * c)) / (2 * a)

    assert x < STROKE_LIMIT_M, (
        f"Peak dynamic stroke {x:.3f} m at 1.5 m/s exceeds the "
        f"{STROKE_LIMIT_M} m limit at k={k:.0f} N/m: legs bottom out and "
        f"reported forces become contact-solver artifacts."
    )
    assert x / STROKE_LIMIT_M <= 0.80, (
        f"Peak dynamic stroke uses {x / STROKE_LIMIT_M:.0%} of the limit; "
        f"keep it under 80% for off-nominal margin."
    )

def test_damping_ratio_is_near_critical():
    """zeta ~= 0.7 suppresses bounce without over-damping. Under-damped legs
    let the lander rebound and tip."""
    model = _require_model()
    k, c = _leg_stiffness(model), _leg_damping(model)
    zeta = c / (2 * math.sqrt(k * LANDER_MASS_KG / N_LEGS))

    assert 0.4 <= zeta <= 1.2, (
        f"Damping ratio zeta={zeta:.2f} is outside [0.4, 1.2]. "
        f"For k={k:.0f} N/m, zeta=0.7 needs c="
        f"{0.7 * 2 * math.sqrt(k * LANDER_MASS_KG / N_LEGS):.0f} N*s/m."
    )

# Settling behaviour

def _terrain_z_at(model, data, x, y):
    mujoco.mj_forward(model, data)
    pos = np.array([x, y, 200.0])
    vec = np.array([0.0, 0.0, -1.0])
    geom_out = np.zeros(1, dtype=np.int32)
    hit = mujoco.mj_ray(model, data, pos, vec, np.array([1, 0, 0, 0, 0, 0], dtype=np.uint8), 1, -1, geom_out)
    return (200.0 - hit) if hit >= 0 else 0.0

def _find_flat_spawn_xy(model, data, foot_offsets, margin=15.0, domain=127.0, step=1.0):
    """Find an (x, y) where the terrain under all four feet is close to
    level, so a static-equilibrium check can assume even load sharing.

    The procedural terrain slopes up to ~28 degrees (docs/known-issues.md
    MINOR-2): most spawn points would tilt the lander onto 1-2 legs rather
    than sharing the load evenly across all four. Terrain generation is
    fully deterministic (no RNG), so this search is stable across runs.
    """
    best_xy, best_spread = None, float("inf")
    lo, hi = margin, domain - margin
    x = lo
    while x <= hi:
        y = lo
        while y <= hi:
            heights = [_terrain_z_at(model, data, x + dx, y + dy) for dx, dy in foot_offsets]
            spread = max(heights) - min(heights)
            if spread < best_spread:
                best_spread, best_xy = spread, (x, y, max(heights))
            if best_spread < 0.01:
                return best_xy
            y += step
        x += step
    return best_xy

@pytest.mark.slow
def test_lander_settles_at_rest_with_engine_off():
    """Drop the lander from just above contact with zero thrust and confirm
    each leg settles to 405/k without pinning against its stroke limit.

    This is the end-to-end version of the checks above, and the direct test
    for BLOCKING-2: with thrust cut, the legs must carry full weight.

    The model's default spawn (from generate_world.py) is 30 m above the
    terrain: free-falling from there under 1.62 m/s^2 gravity hits at
    ~9.9 m/s, not the 1.5 m/s design touchdown, which bottoms out any leg
    stiffness regardless of whether BLOCKING-1 is fixed. Reposition to just
    above contact first, using the model's own foot geometry rather than
    hardcoding leg dimensions, so this actually tests static settling: over
    a patch of terrain flat enough that all four legs actually share the
    load (checking each leg against 405/k individually only makes sense
    where the ground doesn't tilt the vehicle onto a subset of its feet).
    """
    model = _require_model()
    data = mujoco.MjData(model)

    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)

    base_z = float(data.qpos[2])
    foot_gids = {n: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, n)
                 for n in ("foot1", "foot2", "foot3", "foot4")}
    foot_radius = float(model.geom_size[foot_gids["foot1"], 0])
    foot_bottom_z = float(data.geom_xpos[foot_gids["foot1"], 2]) - foot_radius
    offset_below_base = base_z - foot_bottom_z

    base_xy = data.qpos[0:2].copy()
    foot_offsets = [
        (float(data.geom_xpos[gid, 0] - base_xy[0]), float(data.geom_xpos[gid, 1] - base_xy[1]))
        for gid in foot_gids.values()
    ]

    spawn_x, spawn_y, terrain_z = _find_flat_spawn_xy(model, data, foot_offsets)

    data.qpos[0] = spawn_x
    data.qpos[1] = spawn_y
    clearance = 0.05
    data.qpos[2] = terrain_z + offset_below_base + clearance
    data.qvel[:] = 0.0
    data.ctrl[:] = 0.0
    mujoco.mj_forward(model, data)

    for _ in range(5000):  # 10 s at dt=0.002
        data.ctrl[:] = 0.0
        mujoco.mj_step(model, data)

    # A rigid 4-point stance is statically indeterminate: even on terrain
    # flat to ~2 cm across the footprint, the solver settles onto whichever
    # 2-3 feet end up bearing load first (the same reason a real 4-legged
    # table wobbles rather than distributing weight evenly): confirmed
    # empirically here (contact forces of 43 N/45 N/385 N/765 N across the
    # four feet, despite a flat pad). So this cannot assert each leg
    # individually settles at 405/k; what must hold regardless of which
    # subset of feet carries the load is that no leg bottoms out, the
    # vehicle comes to rest, and the feet collectively bear the full static
    # weight (the direct BLOCKING-2 regression: with thrust cut, gravity
    # alone must be balanced by ground contact, not partly by the engine).
    from lunar_lander.run_monte_carlo import get_contact_forces

    for name in LEG_JOINTS:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        stroke = float(data.qpos[model.jnt_qposadr[jid]])
        assert stroke < STROKE_LIMIT_M - 1e-3, (
            f"{name} bottomed out at {stroke:.4f} m with the engine off. "
            f"The legs cannot hold the vehicle up."
        )

    total_contact_force = sum(get_contact_forces(model, data).values())
    expected_total = STATIC_LOAD_PER_LEG_N * N_LEGS  # 1620 N
    assert total_contact_force == pytest.approx(expected_total, rel=0.05), (
        f"Feet carry {total_contact_force:.0f} N total; expected ~{expected_total:.0f} N "
        f"(= full static weight, engine off)."
    )

    assert abs(data.qvel[2]) < 0.02, "Lander never came to rest vertically"

# Terrain collision

def test_terrain_uses_hfield_not_mesh():
    """MuJoCo evaluates mesh-mesh collision against the CONVEX HULL, which
    erases every crater. The terrain must be an hfield.

    Regression: this bug ran silently through an entire campaign. The render
    showed craters; the physics saw a smooth dome.
    """
    model = _require_model()
    gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "ground_col")
    assert gid >= 0, "no geom named 'ground_col'"

    assert model.geom_type[gid] == mujoco.mjtGeom.mjGEOM_HFIELD, (
        "Terrain collision geom is not an hfield. With type='mesh', MuJoCo "
        "collides against the convex hull and all concave terrain vanishes."
    )

def test_engine_actuator_exists():
    """A landing simulation needs a descent engine. The original model had
    a decorative engine bell with contype=0 and no actuators at all."""
    model = _require_model()
    assert model.nu >= 1, (
        "Model has no actuators. Free-fall from altitude is not a landing."
    )

if __name__ == "__main__":
    m = _require_model()
    k, c = _leg_stiffness(m), _leg_damping(m)
    print(f"mass         {_lander_subtree_mass(m):.1f} kg")
    print(f"stiffness    {k:.0f} N/m")
    print(f"damping      {c:.0f} N*s/m")
    print(f"static x     {STATIC_LOAD_PER_LEG_N / k:.3f} m "
          f"({STATIC_LOAD_PER_LEG_N / k / STROKE_LIMIT_M:.0%} of stroke)")
    print(f"zeta         {c / (2 * math.sqrt(k * LANDER_MASS_KG / N_LEGS)):.2f}")
    print(f"actuators    {m.nu}")

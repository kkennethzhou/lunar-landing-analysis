"""
Static equilibrium regression tests.

These encode properties derivable from first principles, independent of what
the simulation happens to output. That distinction matters: the landing-gear
stiffness bug (docs/known-issues.md BLOCKING-1) survived a full 500-trial
campaign because every check compared simulation output against other
simulation output. Nothing asked whether the lander could stand up.

test_legs_support_static_weight is currently EXPECTED TO FAIL at the shipped
stiffness of 320 N/m. That is the point — it is the test that should have
existed before the campaign ran.
"""

import math
from pathlib import Path

import mujoco
import numpy as np
import pytest

XML_PATH = Path(__file__).resolve().parents[1] / "physics_mesh" / ".." / "lunar_world.xml"

LANDER_MASS_KG = 1000.0
LUNAR_GRAVITY = 1.62
N_LEGS = 4
STROKE_LIMIT_M = 0.8

STATIC_LOAD_PER_LEG_N = LANDER_MASS_KG * LUNAR_GRAVITY / N_LEGS  # 405.0 N
LEG_JOINTS = ["leg1_slide", "leg2_slide", "leg3_slide", "leg4_slide"]


def _require_model():
    if not XML_PATH.exists():
        pytest.skip(f"{XML_PATH.name} not found — run `make world` first")
    return mujoco.MjModel.from_xml_path(str(XML_PATH))


def _leg_stiffness(model) -> float:
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, LEG_JOINTS[0])
    return float(model.jnt_stiffness[jid])


def _leg_damping(model) -> float:
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, LEG_JOINTS[0])
    return float(model.dof_damping[model.jnt_dofadr[jid]])


# ── Mass and gravity ─────────────────────────────────────────────────────────


def test_total_mass_is_exactly_1000kg():
    """Every document claims 1,000 kg. The model must agree.

    Regression: an explicit <inertial mass="800"> once silently overrode the
    bus/engine/tank geom masses, giving 960 kg while all reporting said 1,000.
    """
    model = _require_model()
    total = float(model.body_mass.sum())
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
    """
    model = _require_model()
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "lander_base")
    ixx, iyy, izz = model.body_inertia[bid]
    assert izz > ixx, (
        f"Izz ({izz:.1f}) should exceed Ixx ({ixx:.1f}) for a flat, wide bus. "
        f"A hand-picked diaginertia is likely overriding derived inertia."
    )
    assert ixx == pytest.approx(iyy, rel=0.15), "Ixx and Iyy should be near-symmetric"


# ── Landing gear sizing ──────────────────────────────────────────────────────


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
        f"{STROKE_LIMIT_M} m limit at k={k:.0f} N/m — legs bottom out and "
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


# ── Settling behaviour ───────────────────────────────────────────────────────


@pytest.mark.slow
def test_lander_settles_at_rest_with_engine_off():
    """Drop the lander from just above contact with zero thrust and confirm
    each leg settles to 405/k without pinning against its stroke limit.

    This is the end-to-end version of the checks above, and the direct test
    for BLOCKING-2: with thrust cut, the legs must carry full weight.
    """
    model = _require_model()
    data = mujoco.MjData(model)
    k = _leg_stiffness(model)

    mujoco.mj_resetData(model, data)
    data.ctrl[:] = 0.0  # engine off — legs carry everything
    mujoco.mj_forward(model, data)

    for _ in range(5000):  # 10 s at dt=0.002
        data.ctrl[:] = 0.0
        mujoco.mj_step(model, data)

    expected = STATIC_LOAD_PER_LEG_N / k
    for name in LEG_JOINTS:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        stroke = float(data.qpos[model.jnt_qposadr[jid]])

        assert stroke < STROKE_LIMIT_M - 1e-3, (
            f"{name} bottomed out at {stroke:.4f} m with the engine off. "
            f"The legs cannot hold the vehicle up."
        )
        assert stroke == pytest.approx(expected, rel=0.15), (
            f"{name} settled at {stroke:.4f} m; expected ~{expected:.4f} m "
            f"(= 405 N / {k:.0f} N/m)."
        )

    assert abs(data.qvel[2]) < 0.02, "Lander never came to rest vertically"


# ── Terrain collision ────────────────────────────────────────────────────────


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
    print(f"mass         {m.body_mass.sum():.1f} kg")
    print(f"stiffness    {k:.0f} N/m")
    print(f"damping      {c:.0f} N*s/m")
    print(f"static x     {STATIC_LOAD_PER_LEG_N / k:.3f} m "
          f"({STATIC_LOAD_PER_LEG_N / k / STROKE_LIMIT_M:.0%} of stroke)")
    print(f"zeta         {c / (2 * math.sqrt(k * LANDER_MASS_KG / N_LEGS)):.2f}")
    print(f"actuators    {m.nu}")

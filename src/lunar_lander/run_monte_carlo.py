"""
Monte Carlo landing simulation: powered descent, 200+ drops.

Each trial:
  - Random X/Y spawn within safe terrain bounds
  - Random horizontal velocity kick (±1.5 m/s each axis)
  - Closed-loop descent controller tapers target vertical speed from
    ~10 m/s at altitude down to 1.5 m/s at touchdown (PD on velocity error)
  - Physics stepped until landed (stable), tipped, over-force, over-stroke,
    over-g, or timed out
  - Logs: trial, x, y, vx, vy, strut force (physical) + contact force
    (raw, for comparison), peak stroke, peak accel, outcome, failure reason,
    plus touchdown body/foot positions (OPEN-1 measurement fix: spawn x,y
    are not contact coordinates)
"""

import csv
import math

import mujoco
import numpy as np

from lunar_lander.paths import RESULTS_CSV as CSV_PATH
from lunar_lander.paths import XML_PATH

# Adjacent feet sit 4.0 m apart in the MJCF (upper attachment ±1.1 m plus
# strut tip ±0.9 m). Older docs called this ~2.5 m; that was wrong.
ADJACENT_FOOT_SPACING_M = 4.0

N_TRIALS     = 500
MAX_STEPS    = 12000       # 12000 x 0.002 s = 24 s max per trial
SPAWN_HEIGHT = 30.0        # metres above terrain  (matches generator)
TIP_THRESH   = math.radians(15)
VEL_RANGE    = 2.3         # m/s horizontal kick (±), widened from 1.5
GRID_MARGIN  = 15          # stay this many metres from terrain edge

# Phase 7: envelope widening
# A 100%-success Monte Carlo can't discriminate between landing sites (see
# validate_results.py's signal test: confirmed empirically at VEL_RANGE=1.5,
# no initial tilt/spin: corr(force, slope) ~ 0, because nothing ever got
# close enough to its limits for terrain to matter). These ranges are scaled
# up from the original nominal levels toward (not to) interactive_landing.py's
# stress-test levels (5.5-9.0 m/s drift, 2-5 rad/s tumble): enough to
# produce a genuine failure population without guaranteeing tip-over, which
# would saturate the test in the other direction.
INIT_TILT_MAX_DEG      = 2.0     # initial body tilt off vertical at spawn
# The lander has no attitude-control torque actuator (Phase 1 only added a
# single vertical thrust actuator), so any initial angular velocity is
# uncorrected for the whole ~9 s descent: at 0.35 rad/s that integrates to
# ~180 degrees of tilt by touchdown, guaranteeing a tip-over on every trial
# (verified empirically: 60/60 failures at that setting). Capped low enough
# that its worst-case contribution over a ~10 s flight stays a few degrees.
INIT_ANGVEL_MAX_RADPS  = 0.02    # initial per-axis angular velocity at spawn
FRICTION_SCALE_MIN     = 0.7     # regolith-patch friction variability
FRICTION_SCALE_MAX     = 1.3
TOUCHDOWN_VZ_MIN        = 1.0    # guidance/altimeter touchdown-speed dispersion
TOUCHDOWN_VZ_MAX        = 2.3

# Terrain spans 0-127 m; keep lander inside margin
SPAWN_MIN = GRID_MARGIN
SPAWN_MAX = 127.0 - GRID_MARGIN

# Descent controller
# Target vertical speed profile: a constant-deceleration reference (not a
# straight line in altitude) so the target hits TOUCHDOWN_VZ exactly at
# agl=0 and CRUISE_VZ exactly at agl=TAPER_START_M:
#   v_target(agl) = -sqrt(TOUCHDOWN_VZ^2 + 2*A_DES*agl)
# PD control alone lags a ramping/curving target under constant gravity
# bias (classic Type-1 steady-state error): a pure PD taper was found
# empirically to still be falling at ~2.6-2.8 m/s at touchdown instead of
# 1.5, which then exceeded the legs' designed energy budget and bounced.
# Fixed by adding a velocity feed-forward term: a_ff = d(v_target)/dt,
# estimated by finite-differencing vz_target between steps and commanding
# thrust = m*(g + a_ff) + PD correction. This tracks the ramp/curve with
# near-zero steady-state error, closing the gap that PD alone couldn't.
TOUCHDOWN_VZ   = 1.5       # m/s, design touchdown speed (matches leg sizing)
CRUISE_VZ      = 10.0      # m/s, target descent rate at high altitude
TAPER_START_M  = 25.0      # begin tapering toward touchdown speed below this AGL
A_DES          = (CRUISE_VZ**2 - TOUCHDOWN_VZ**2) / (2 * TAPER_START_M)
KP_VZ          = 4000.0    # N per (m/s) velocity error (now just trims residual error)
KD_VZ          = 400.0     # N per (m/s^2): damps rate-of-change of error
LANDER_MASS    = 1000.0
GRAVITY        = 1.62
HOVER_THRUST   = LANDER_MASS * GRAVITY   # feed-forward weight compensation

def vz_target_profile(agl, touchdown_vz=TOUCHDOWN_VZ):
    if agl > TAPER_START_M:
        return -CRUISE_VZ
    a_des = (CRUISE_VZ**2 - touchdown_vz**2) / (2 * TAPER_START_M)
    return -math.sqrt(touchdown_vz**2 + 2 * a_des * max(0.0, agl))

# Failure criteria
# Structural leg-force limit: ~50 kN per leg. Static per-leg load on the
# Moon is 1000*1.62/4 = 405 N; 50 kN is a ~125x margin on static load,
# comparable in spirit to Apollo LM's <10 g design envelope translated to
# strut load for a 1000 kg vehicle decelerating at up to ~10 g over a
# short stroke (F = m*a/4legs ~ 1000*10*1.62/4 ~= 4 kN nominal, so 50 kN
# leaves headroom for genuine off-nominal impacts while still being a real
# structural ceiling, not a number that lets every trial pass).
LEG_FORCE_LIMIT_N = 50_000.0
STROKE_LIMIT_M     = 0.8      # joint range upper bound: bottoming out
ACCEL_LIMIT_G      = 10.0

def quat_to_tilt(q):
    """Return tilt angle (rad) from a MuJoCo wxyz quaternion."""
    w, x, y, z = q
    zz = w*w - x*x - y*y + z*z
    return math.acos(max(-1.0, min(1.0, zz)))

_TERRAIN_GROUP_FILTER = np.array([1, 0, 0, 0, 0, 0], dtype=np.uint8)

def terrain_z_at(model, data, x, y):
    """
    Ray-cast straight down from (x, y, 200) to find terrain height.
    mj_forward must run first so geom_xpos is populated: a raw
    mj_resetData() zeroes it, and this only "worked" before because the
    terrain body happens to sit at the origin with identity orientation.

    Filtered to geom group 0 (terrain only; all lander geoms are group 1
    in the generator). bodyexclude alone is insufficient here: it can only
    exclude a single body id, but the lander's legs/feet are separate
    child bodies extending ~2 m below lander_base. This function runs
    every step (for the descent controller's altitude-above-ground
    taper), so once the lander gets close to the ground an unfiltered or
    single-body-excluded ray starts hitting the lander's own leg/foot
    geoms instead of the terrain underneath: freezing the computed
    altitude and corrupting the descent profile right at touchdown.
    """
    mujoco.mj_forward(model, data)
    pos   = np.array([x, y, 200.0])
    vec   = np.array([0.0, 0.0, -1.0])
    geom_out = np.zeros(1, dtype=np.int32)
    hit = mujoco.mj_ray(model, data, pos, vec, _TERRAIN_GROUP_FILTER, 1, -1, geom_out)
    return (200.0 - hit) if hit >= 0 else 0.0

LEG_JOINTS = ["leg1_slide", "leg2_slide", "leg3_slide", "leg4_slide"]
FOOT_GEOMS = {"foot1", "foot2", "foot3", "foot4"}
BASE_FOOT_FRICTION = 0.9   # matches lunar_world.xml's foot geom friction[0]
LEG_TO_FOOT = {"leg1_slide": "foot1", "leg2_slide": "foot2",
               "leg3_slide": "foot3", "leg4_slide": "foot4"}

def get_contact_forces(model, data):
    """
    Sum raw contact normal forces on each foot geom. Kept only as a
    labelled comparison column: on a bottomed-out leg this measures the
    solver's joint-limit/contact stiffness (solref), not the physical
    load the landing gear carries. See get_strut_forces for the real one.
    """
    forces = {n: 0.0 for n in FOOT_GEOMS}
    for i in range(data.ncon):
        c = data.contact[i]
        cf = np.zeros(6)
        mujoco.mj_contactForce(model, data, i, cf)
        normal_force = abs(cf[0])
        g1 = model.geom(c.geom1).name
        g2 = model.geom(c.geom2).name
        if g1 in forces:
            forces[g1] += normal_force
        if g2 in forces:
            forces[g2] += normal_force
    return forces

def get_strut_forces(model, data, dof_adrs):
    """
    Physical axial load carried by each leg strut: passive (spring+damper)
    force plus constraint force (joint-limit reaction when bottomed out),
    at the slide joint's DOF. This is what the landing gear actually
    carries, independent of contact-solver stiffness.
    """
    return {name: abs(data.qfrc_passive[adr] + data.qfrc_constraint[adr])
            for name, adr in dof_adrs.items()}

def get_strokes(data, qpos_adrs):
    return {name: float(data.qpos[adr]) for name, adr in qpos_adrs.items()}

def descent_thrust(vz_current, vz_target, prev_err, vz_target_prev, dt):
    """
    PD on vertical velocity error, plus feed-forward hover thrust and a
    feed-forward term for the reference profile's own acceleration
    (finite-differenced), so the controller doesn't lag a moving target.
    """
    err = vz_target - vz_current   # both signed negative-down; error > 0 means falling too fast
    derr = (err - prev_err) / dt if dt > 0 else 0.0
    a_ff = (vz_target - vz_target_prev) / dt if dt > 0 else 0.0   # negative = accelerating downward
    thrust = HOVER_THRUST + LANDER_MASS * a_ff + KP_VZ * err + KD_VZ * derr
    return max(0.0, min(thrust, ENGINE_MAX_THRUST)), err

ENGINE_MAX_THRUST = 5000.0

def run_trial(model, data, x, y, vx, vy,
              tilt_axis=(0.0, 0.0), tilt_deg=0.0,
              wx=0.0, wy=0.0, wz=0.0,
              friction_scale=1.0, touchdown_vz_target=None):
    """
    Reset lander to (x, y, terrain_z + SPAWN_HEIGHT) with velocity (vx, vy, 0),
    optional initial tilt (tilt_axis, tilt_deg), optional initial angular
    velocity (wx, wy, wz), an optional per-trial friction multiplier
    applied to the foot geoms (simulates regolith patch variability), and
    an optional per-trial touchdown-speed target overriding TOUCHDOWN_VZ
    (simulates altimeter/guidance dispersion instead of a perfectly
    known, identical target every trial). These are Phase 7's envelope
    widening: nominal Monte Carlo levels saturate at ~100% success, which
    can't discriminate between landing sites (see validate_results.py's
    signal test): widening toward (but not to) interactive_landing.py's
    stress-test levels is what makes force start to depend on where and
    how the lander lands.
    """
    mujoco.mj_resetData(model, data)

    tz = terrain_z_at(model, data, x, y)
    spawn_z = tz + SPAWN_HEIGHT

    data.qpos[0] = x
    data.qpos[1] = y
    data.qpos[2] = spawn_z

    if tilt_deg > 0.0:
        ax, ay = tilt_axis
        norm = math.hypot(ax, ay)
        if norm > 0:
            ax, ay = ax / norm, ay / norm
        half = math.radians(tilt_deg) / 2.0
        data.qpos[3] = math.cos(half)             # w
        data.qpos[4] = ax * math.sin(half)         # x
        data.qpos[5] = ay * math.sin(half)         # y
        data.qpos[6] = 0.0                         # z
    else:
        data.qpos[3] = 1.0
        data.qpos[4] = 0.0
        data.qpos[5] = 0.0
        data.qpos[6] = 0.0

    data.qvel[0] = vx
    data.qvel[1] = vy
    data.qvel[2] = 0.0
    data.qvel[3] = wx
    data.qvel[4] = wy
    data.qvel[5] = wz

    for gname in FOOT_GEOMS:
        gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, gname)
        model.geom_friction[gid, 0] = BASE_FOOT_FRICTION * friction_scale

    vz_target_override = touchdown_vz_target if touchdown_vz_target is not None else TOUCHDOWN_VZ

    mujoco.mj_forward(model, data)

    dof_adrs = {}
    qpos_adrs = {}
    for jname in LEG_JOINTS:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jname)
        dof_adrs[jname] = model.jnt_dofadr[jid]
        qpos_adrs[jname] = model.jnt_qposadr[jid]

    foot_geom_ids = {n: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, n) for n in FOOT_GEOMS}
    FOOT_RADIUS = 0.12

    def foot_clearance(model, data):
        """
        Minimum ground clearance of any footpad (foot bottom above local
        terrain). Used as the descent controller's altitude reference
        instead of body height, since the feet hang ~2 m below
        lander_base: targeting TOUCHDOWN_VZ at body-AGL=0 meant the
        actual touchdown (when a foot first reaches the ground) happened
        while the profile still called for ~3.6-3.7 m/s, not 1.5.
        """
        best = None
        for gid in foot_geom_ids.values():
            fx, fy, fz = data.geom_xpos[gid]
            tz_foot = terrain_z_at(model, data, fx, fy)
            clearance = (fz - FOOT_RADIUS) - tz_foot
            if best is None or clearance < best:
                best = clearance
        return best

    max_strut_force   = {n: 0.0 for n in LEG_JOINTS}
    max_contact_force = {n: 0.0 for n in FOOT_GEOMS}
    max_stroke        = {n: 0.0 for n in LEG_JOINTS}
    max_accel_g       = 0.0

    tipped   = False
    outcome  = "timeout"
    fail_reason = ""

    VEL_WINDOW    = 150
    STABLE_VZ     = 0.02
    vz_history    = []
    feet_down_run = 0     # consecutive-step counter for >=3 feet in contact
    ACCEL_WINDOW  = 10     # ~20 ms smoothing window for peak_accel_g
    accel_history = []

    prev_vz_err = 0.0
    dt = model.opt.timestep
    engine_act = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "descent_engine")
    vz_target_prev = vz_target_profile(SPAWN_HEIGHT, vz_target_override)

    # Thrust cutoff at touchdown (BLOCKING-2, docs/known-issues.md). Without
    # this, HOVER_THRUST keeps applying m*g feed-forward after contact, so
    # the settle detector below fires while the engine is still holding up
    # most of the vehicle's weight: legs never carry their designed static
    # load and every reported force understates the real one. Apollo cut
    # off on the contact-probe signal; mirrored here by ramping thrust to
    # zero over ~0.2 s once >=3 feet register contact, using the previous
    # step's contact_forces (initialised from the pre-loop state, then kept
    # current by the post-step recomputation below) so the decision is made
    # from already-measured state rather than an extra contact query.
    TOUCHDOWN_RAMP_S = 0.2
    ramp_steps = max(1, round(TOUCHDOWN_RAMP_S / dt))
    steps_since_touchdown = None
    contact_forces = get_contact_forces(model, data)

    # OPEN-1: spawn (x, y) is not the contact site. With no horizontal
    # thruster, drift up to VEL_RANGE over a ~10 s descent can put the
    # vehicle tens of metres away. Capture body + foot XY at first
    # contact (>=1 foot > 1 N); fall back to final pose if never contacted.
    touchdown_logged = False
    touchdown_x = float(data.qpos[0])
    touchdown_y = float(data.qpos[1])
    foot_xy = {n: (float(data.geom_xpos[gid][0]), float(data.geom_xpos[gid][1]))
               for n, gid in foot_geom_ids.items()}

    def _capture_touchdown_pose():
        nonlocal touchdown_x, touchdown_y, foot_xy, touchdown_logged
        touchdown_x = float(data.qpos[0])
        touchdown_y = float(data.qpos[1])
        foot_xy = {
            n: (float(data.geom_xpos[gid][0]), float(data.geom_xpos[gid][1]))
            for n, gid in foot_geom_ids.items()
        }
        touchdown_logged = True

    step = 0
    for step in range(MAX_STEPS):  # noqa: B007  read after the loop as the step count
        # AGL (body height above local terrain) for crash detection.
        tz_here = terrain_z_at(model, data, data.qpos[0], data.qpos[1])

        feet_down_now = sum(1 for f in contact_forces.values() if f > 1.0)
        if steps_since_touchdown is None and feet_down_now >= 3:
            steps_since_touchdown = 0

        # Foot clearance (min over all 4 feet) drives the descent profile,
        # not body AGL: see foot_clearance() docstring.
        clearance = foot_clearance(model, data)
        vz_target = vz_target_profile(clearance, vz_target_override)
        thrust, prev_vz_err = descent_thrust(data.qvel[2], vz_target, prev_vz_err, vz_target_prev, dt)
        vz_target_prev = vz_target

        if steps_since_touchdown is not None:
            ramp_frac = max(0.0, 1.0 - steps_since_touchdown / ramp_steps)
            thrust *= ramp_frac
            steps_since_touchdown += 1

        data.ctrl[engine_act] = thrust

        mujoco.mj_step(model, data)

        tilt = quat_to_tilt(data.qpos[3:7])
        if tilt > TIP_THRESH:
            tipped = True
            outcome = "tipped"
            fail_reason = "tilt_exceeded"
            break

        contact_forces = get_contact_forces(model, data)
        strut_forces = get_strut_forces(model, data, dof_adrs)
        strokes = get_strokes(data, qpos_adrs)

        if not touchdown_logged and any(f > 1.0 for f in contact_forces.values()):
            _capture_touchdown_pose()

        for n in LEG_JOINTS:
            if strut_forces[n] > max_strut_force[n]:
                max_strut_force[n] = strut_forces[n]
            if strokes[n] > max_stroke[n]:
                max_stroke[n] = strokes[n]
        for n in FOOT_GEOMS:
            if contact_forces[n] > max_contact_force[n]:
                max_contact_force[n] = contact_forces[n]

        # Smoothed (windowed-average) acceleration, not raw per-step qacc.
        # A single 0.002s timestep's instantaneous qacc during a stiff
        # contact-onset event is a discretization spike, not something a
        # real bandwidth-limited accelerometer would report: averaging
        # over ACCEL_WINDOW steps (~20 ms) gives a physically meaningful
        # "g-load" instead of letting one timestep dominate the metric.
        accel_mag = np.linalg.norm(data.qacc[0:3])
        accel_history.append(accel_mag)
        if len(accel_history) > ACCEL_WINDOW:
            accel_history.pop(0)
        accel_g = (sum(accel_history) / len(accel_history)) / 9.80665
        if accel_g > max_accel_g:
            max_accel_g = accel_g

        # Failure checks (structural / bottoming-out / over-g), evaluated
        # every step so the run stops as soon as a limit is crossed.
        peak_strut_now = max(strut_forces.values())
        if peak_strut_now > LEG_FORCE_LIMIT_N:
            outcome = "failed"
            fail_reason = "leg_force_exceeded"
            break
        if max(strokes.values()) >= STROKE_LIMIT_M:
            outcome = "failed"
            fail_reason = "leg_bottomed_out"
            break
        if accel_g > ACCEL_LIMIT_G:
            outcome = "failed"
            fail_reason = "accel_exceeded"
            break

        body_z = data.qpos[2]
        if body_z < tz_here - 0.5:
            outcome = "crashed"
            fail_reason = "crashed"
            break

        # Settle detection: >=3 feet down for VEL_WINDOW *consecutive* steps,
        # not just at the current step (the original bug only windowed |vz|).
        vz_history.append(abs(data.qvel[2]))
        if len(vz_history) > VEL_WINDOW:
            vz_history.pop(0)

        feet_down = sum(1 for f in contact_forces.values() if f > 1.0)
        if feet_down >= 3:
            feet_down_run += 1
        else:
            feet_down_run = 0

        if (feet_down_run >= VEL_WINDOW
                and len(vz_history) == VEL_WINDOW
                and max(vz_history) < STABLE_VZ
                and abs(data.qvel[0]) < 0.05
                and abs(data.qvel[1]) < 0.05):
            outcome = "landed"
            fail_reason = ""
            break

    if not touchdown_logged:
        _capture_touchdown_pose()

    peak_strut = max(max_strut_force.values())
    peak_contact = max(max_contact_force.values())
    peak_stroke = max(max_stroke.values())

    return dict(
        peak_strut_force_N=peak_strut,
        peak_contact_force_N=peak_contact,
        peak_stroke_m=peak_stroke,
        peak_accel_g=max_accel_g,
        tipped=tipped,
        outcome=outcome,
        fail_reason=fail_reason,
        steps=step + 1,
        touchdown_x=touchdown_x,
        touchdown_y=touchdown_y,
        foot1_x=foot_xy["foot1"][0],
        foot1_y=foot_xy["foot1"][1],
        foot2_x=foot_xy["foot2"][0],
        foot2_y=foot_xy["foot2"][1],
        foot3_x=foot_xy["foot3"][0],
        foot3_y=foot_xy["foot3"][1],
        foot4_x=foot_xy["foot4"][0],
        foot4_y=foot_xy["foot4"][1],
    )

# Main

def run_campaign(model, n_trials=N_TRIALS, seed=42, csv_path=CSV_PATH,
                 progress=True, record_summary=True,
                 zero_horizontal_velocity=False):
    """Run the seeded descent Monte Carlo and return one dict per trial.

    The draw order matches the historical seed-42 campaign: positions, then
    horizontal velocity, tilt, spin, friction, and the touchdown target.

    If zero_horizontal_velocity is True, vx/vy are still drawn (so later RNG
    draws keep the same spawn points as the published campaign) and then
    overwritten to 0 before each trial runs.
    """
    rng = np.random.default_rng(seed)
    data = mujoco.MjData(model)
    trial_count = n_trials
    out_csv = csv_path

    fieldnames = ["trial", "x", "y", "vx", "vy",
                  "peak_strut_force_N", "peak_contact_force_N",
                  "peak_stroke_m", "peak_accel_g",
                  "tipped", "outcome", "fail_reason", "steps",
                  # legacy column name kept for any downstream script that
                  # still reads max_leg_force_N: now maps to the physical
                  # strut force, not the old solver-spike contact force.
                  "max_leg_force_N",
                  "touchdown_x", "touchdown_y",
                  "foot1_x", "foot1_y", "foot2_x", "foot2_y",
                  "foot3_x", "foot3_y", "foot4_x", "foot4_y",
                  "drawn_vx", "drawn_vy"]

    xs  = rng.uniform(SPAWN_MIN, SPAWN_MAX, trial_count)
    ys  = rng.uniform(SPAWN_MIN, SPAWN_MAX, trial_count)
    vxs = rng.uniform(-VEL_RANGE, VEL_RANGE, trial_count)
    vys = rng.uniform(-VEL_RANGE, VEL_RANGE, trial_count)

    tilt_degs    = rng.uniform(0.0, INIT_TILT_MAX_DEG, trial_count)
    tilt_angles  = rng.uniform(0.0, 2 * math.pi, trial_count)
    tilt_axes    = np.stack([np.cos(tilt_angles), np.sin(tilt_angles)], axis=1)
    wxs = rng.uniform(-INIT_ANGVEL_MAX_RADPS, INIT_ANGVEL_MAX_RADPS, trial_count)
    wys = rng.uniform(-INIT_ANGVEL_MAX_RADPS, INIT_ANGVEL_MAX_RADPS, trial_count)
    wzs = rng.uniform(-INIT_ANGVEL_MAX_RADPS, INIT_ANGVEL_MAX_RADPS, trial_count)
    friction_scales = rng.uniform(FRICTION_SCALE_MIN, FRICTION_SCALE_MAX, trial_count)
    touchdown_targets = rng.uniform(TOUCHDOWN_VZ_MIN, TOUCHDOWN_VZ_MAX, trial_count)

    # OPEN-1 low-drift control: zero AFTER the seeded draws so spawn x,y
    # (and every later draw) still match the published seed-42 campaign.
    drawn_vxs = np.array(vxs, copy=True)
    drawn_vys = np.array(vys, copy=True)
    if zero_horizontal_velocity:
        vxs = np.zeros(trial_count)
        vys = np.zeros(trial_count)

    outcomes_count = {"landed": 0, "tipped": 0, "crashed": 0, "failed": 0, "timeout": 0}
    fail_reason_count = {}

    records = []
    with open(out_csv, "w", newline="") as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()

        for i in range(trial_count):
            x, y, vx, vy = xs[i], ys[i], vxs[i], vys[i]

            r = run_trial(model, data, x, y, vx, vy,
                          tilt_axis=tuple(tilt_axes[i]), tilt_deg=tilt_degs[i],
                          wx=wxs[i], wy=wys[i], wz=wzs[i],
                          friction_scale=friction_scales[i],
                          touchdown_vz_target=touchdown_targets[i])
            outcomes_count[r["outcome"]] += 1
            if r["fail_reason"]:
                fail_reason_count[r["fail_reason"]] = fail_reason_count.get(r["fail_reason"], 0) + 1

            record = {
                "trial": i + 1,
                "x": float(x), "y": float(y), "vx": float(vx), "vy": float(vy),
                "peak_strut_force_N": float(r["peak_strut_force_N"]),
                "peak_stroke_m": float(r["peak_stroke_m"]),
                "outcome": r["outcome"],
                "fail_reason": r["fail_reason"],
                "touchdown_x": float(r["touchdown_x"]),
                "touchdown_y": float(r["touchdown_y"]),
                "foot1_x": float(r["foot1_x"]), "foot1_y": float(r["foot1_y"]),
                "foot2_x": float(r["foot2_x"]), "foot2_y": float(r["foot2_y"]),
                "foot3_x": float(r["foot3_x"]), "foot3_y": float(r["foot3_y"]),
                "foot4_x": float(r["foot4_x"]), "foot4_y": float(r["foot4_y"]),
                "drawn_vx": float(drawn_vxs[i]), "drawn_vy": float(drawn_vys[i]),
            }
            records.append(record)
            writer.writerow({
                "trial":  i + 1,
                "x":      f"{x:.2f}",
                "y":      f"{y:.2f}",
                "vx":     f"{vx:.3f}",
                "vy":     f"{vy:.3f}",
                "peak_strut_force_N":   f"{r['peak_strut_force_N']:.1f}",
                "peak_contact_force_N": f"{r['peak_contact_force_N']:.1f}",
                "peak_stroke_m":        f"{r['peak_stroke_m']:.4f}",
                "peak_accel_g":         f"{r['peak_accel_g']:.2f}",
                "tipped":    int(r["tipped"]),
                "outcome":   r["outcome"],
                "fail_reason": r["fail_reason"],
                "steps":     r["steps"],
                "max_leg_force_N": f"{r['peak_strut_force_N']:.1f}",
                "touchdown_x": f"{r['touchdown_x']:.2f}",
                "touchdown_y": f"{r['touchdown_y']:.2f}",
                "foot1_x": f"{r['foot1_x']:.2f}", "foot1_y": f"{r['foot1_y']:.2f}",
                "foot2_x": f"{r['foot2_x']:.2f}", "foot2_y": f"{r['foot2_y']:.2f}",
                "foot3_x": f"{r['foot3_x']:.2f}", "foot3_y": f"{r['foot3_y']:.2f}",
                "foot4_x": f"{r['foot4_x']:.2f}", "foot4_y": f"{r['foot4_y']:.2f}",
                "drawn_vx": f"{drawn_vxs[i]:.3f}",
                "drawn_vy": f"{drawn_vys[i]:.3f}",
            })
            csvfile.flush()

            if progress:
                bar_filled = int(30 * (i + 1) / trial_count)
                bar = "#" * bar_filled + "-" * (30 - bar_filled)
                print(f"\r[{bar}] {i+1:3d}/{trial_count}  {r['outcome']:<8}  "
                      f"strut={r['peak_strut_force_N']:7.1f} N  stroke={r['peak_stroke_m']:.3f} m",
                      end="", flush=True)

    if progress:
        print(f"\n\nDone. Results written to {out_csv}")

    # Summary

    rows = []
    with open(out_csv) as f:
        for r in csv.DictReader(f):
            rows.append(r)

    forces    = [float(r["peak_strut_force_N"]) for r in rows]
    strokes   = [float(r["peak_stroke_m"]) for r in rows]
    steps_all = [int(r["steps"]) for r in rows]
    n_bottomed = sum(1 for s in strokes if s >= STROKE_LIMIT_M)

    if progress:
        def _pct(key):
            return 100 * outcomes_count[key] / trial_count
        print(f"\nMonte Carlo results (n={trial_count})")
        print(f"  landed:  {outcomes_count['landed']:3d} ({_pct('landed'):5.1f}%)")
        print(f"  tipped:  {outcomes_count['tipped']:3d} ({_pct('tipped'):5.1f}%)")
        print(f"  crashed: {outcomes_count['crashed']:3d} ({_pct('crashed'):5.1f}%)")
        print(f"  failed:  {outcomes_count['failed']:3d} ({_pct('failed'):5.1f}%)")
        print(f"  timeout: {outcomes_count['timeout']:3d} ({_pct('timeout'):5.1f}%)")
        print(f"  failure reasons: {fail_reason_count}")
        print("  peak strut force (N): "
              f"min {min(forces):.1f}, max {max(forces):.1f}, "
              f"mean {sum(forces)/len(forces):.1f}")
        print(f"  max stroke: {max(strokes):.4f} m (limit {STROKE_LIMIT_M} m), "
              f"bottomed out: {n_bottomed}")
        print("  steps: "
              f"min {min(steps_all)} ({min(steps_all)*0.002:.2f} s), "
              f"max {max(steps_all)} ({max(steps_all)*0.002:.2f} s), "
              f"mean {int(sum(steps_all)/len(steps_all))} "
              f"({int(sum(steps_all)/len(steps_all))*0.002:.2f} s)")

    if record_summary:
        from lunar_lander.summary_stats import merge_summary_stats
        merge_summary_stats(dict(
        n_trials=trial_count,
        outcome_counts=outcomes_count,
        fail_reason_counts=fail_reason_count,
        failure_rate_pct=100 * (trial_count - outcomes_count["landed"]) / trial_count,
        touchdown_design_vz_mps=TOUCHDOWN_VZ,
        force_min_N=min(forces),
        force_max_N=max(forces),
        force_mean_N=sum(forces) / len(forces),
        stroke_max_m=max(strokes),
        stroke_limit_m=STROKE_LIMIT_M,
        n_bottomed_out=n_bottomed,
        leg_force_limit_N=LEG_FORCE_LIMIT_N,
        accel_limit_g=ACCEL_LIMIT_G,
        descent_time_min_s=min(steps_all) * model.opt.timestep,
        descent_time_max_s=max(steps_all) * model.opt.timestep,
        descent_time_mean_s=(sum(steps_all) / len(steps_all)) * model.opt.timestep,
        vel_range_mps=VEL_RANGE,
        grid_margin_m=GRID_MARGIN,
        spawn_min_m=SPAWN_MIN,
            spawn_max_m=SPAWN_MAX,
        ))
        print("  Merged into summary_stats.json")
    return records

def main(argv=None):
    import argparse
    from pathlib import Path

    parser = argparse.ArgumentParser(description="Monte Carlo lunar landing trials")
    parser.add_argument("--trials", type=int, default=N_TRIALS,
                        help="number of trials to run (default: %(default)s)")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--csv", type=Path, default=None,
                        help="results CSV path (default: paths.RESULTS_CSV)")
    parser.add_argument("--xml", type=Path, default=None,
                        help="MJCF path (default: paths.XML_PATH)")
    parser.add_argument(
        "--zero-drift",
        action="store_true",
        help="draw horizontal velocities then overwrite vx=vy=0 (OPEN-1 control)",
    )
    parser.add_argument("--no-summary", action="store_true",
                        help="do not merge into summary_stats.json")
    args = parser.parse_args(argv)
    xml_path = args.xml if args.xml is not None else XML_PATH
    csv_path = args.csv if args.csv is not None else CSV_PATH
    print(f"Loading model: {xml_path}")
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    return run_campaign(
        model,
        n_trials=args.trials,
        seed=args.seed,
        csv_path=csv_path,
        record_summary=not args.no_summary,
        zero_horizontal_velocity=args.zero_drift,
    )

if __name__ == "__main__":
    main()

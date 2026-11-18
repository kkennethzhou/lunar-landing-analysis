"""Interactive lunar lander viewer.

Usage: python -m lunar_lander.interactive_landing

1. Stable flight: you pick the XY target and an optional small Vx/Vy.
2. Stress test: tumble and cross-drift are filled in automatically.
"""

import math
import sys
import time

import mujoco
import mujoco.viewer
import numpy as np

from lunar_lander.paths import XML_PATH

# Physics constants

SPAWN_ALTITUDE  = 30.0          # m above terrain
TIP_THRESH_DEG  = 15.0
FORCE_CRITICAL  = 1_100_000.0   # N: structural failure threshold
STABLE_VZ       = 0.02          # m/s: vertical quiet threshold
STABLE_HVEL     = 0.05          # m/s: horizontal quiet threshold
STABLE_WINDOW   = 150           # consecutive steps to confirm stable
MAX_STEPS       = 14_000        # 28 s hard cap (stress test needs more time to settle)

# Stress-test injection ranges
STRESS_DRIFT_MIN  =  5.5        # m/s : minimum cross-drift speed
STRESS_DRIFT_MAX  =  9.0        # m/s
STRESS_TUMBLE_MIN =  2.0        # rad/s: per-axis angular velocity
STRESS_TUMBLE_MAX =  5.0        # rad/s

COORD_MIN, COORD_MAX = 15.0, 112.0
FOOT_GEOMS = {"foot1", "foot2", "foot3", "foot4"}

# ANSI palette

G   = "\033[92m"    # green
Y   = "\033[93m"    # yellow
R   = "\033[91m"    # red
C   = "\033[96m"    # cyan
M   = "\033[95m"    # magenta
W   = "\033[97m"    # bright white
B   = "\033[1m"     # bold
DIM = "\033[2m"
RST = "\033[0m"

BOLD_RED = "\033[1;91m"

# Helpers

def banner():
    print("Lunar lander viewer")
    print("MuJoCo, gravity -1.62 m/s^2")

def mode_menu() -> int:
    """Return 1 (stable) or 2 (stress test)."""
    print("Flight mode")
    print(f"  {W}1{RST}  Stable flight. You pick the coordinates.")
    print(f"  {W}2{RST}  Stress test. Tumble and cross-drift are filled in.")
    print()
    while True:
        raw = input(f"  {W}Select mode [1/2]:{RST} ").strip()
        if raw in ("1", "2"):
            return int(raw)
        print(f"  {R}Enter 1 or 2.{RST}")

def prompt_float(label: str, lo: float, hi: float, default=None) -> float:
    dflt_str = f"  {DIM}[default {default}]{RST}" if default is not None else ""
    while True:
        raw = input(f"  {W}{label}{RST}{dflt_str}: ").strip()
        if raw == "" and default is not None:
            return float(default)
        try:
            v = float(raw)
            if lo <= v <= hi:
                return v
            print(f"  {R}Must be between {lo} and {hi}.{RST}")
        except ValueError:
            print(f"  {R}Please enter a number.{RST}")

def quat_tilt_deg(q) -> float:
    """Tilt from vertical (degrees) given wxyz quaternion."""
    w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    zz = w*w - x*x - y*y + z*z      # world z-component of body z-axis
    return math.degrees(math.acos(max(-1.0, min(1.0, zz))))

def body_zaxis_tilt_deg(model, data) -> float:
    """
    Cross-check tilt using the full 3×3 body orientation matrix (xmat).
    xmat is stored row-major; column 2 = body z-axis in world frame.
    """
    bid  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "lander_base")
    xmat = data.xmat[bid].reshape(3, 3)
    body_z_world = xmat[:, 2]          # third column
    cos_angle = float(np.clip(body_z_world[2], -1.0, 1.0))
    return math.degrees(math.acos(cos_angle))

_TERRAIN_GROUP_FILTER = np.array([1, 0, 0, 0, 0, 0], dtype=np.uint8)

def terrain_z(model, data, tx, ty) -> float:
    """
    Raycast straight down from z=200 to find terrain elevation.
    Filtered to geom group 0 (terrain only: all lander geoms are group 1
    in the generator) so the ray can't hit the lander's own leg/foot
    geoms, which extend well below lander_base's origin.
    """
    mujoco.mj_forward(model, data)
    pos      = np.array([tx, ty, 200.0])
    vec      = np.array([0.0, 0.0, -1.0])
    geom_out = np.zeros(1, dtype=np.int32)
    hit      = mujoco.mj_ray(model, data, pos, vec, _TERRAIN_GROUP_FILTER, 1, -1, geom_out)
    return (200.0 - hit) if hit >= 0 else 0.0

def contact_forces(model, data) -> dict:
    """Return {geom_name: total normal force (N)} for all foot geoms."""
    forces = {n: 0.0 for n in FOOT_GEOMS}
    cf     = np.zeros(6)
    for i in range(data.ncon):
        c  = data.contact[i]
        mujoco.mj_contactForce(model, data, i, cf)
        nf = abs(cf[0])
        g1 = model.geom(c.geom1).name
        g2 = model.geom(c.geom2).name
        if g1 in forces: forces[g1] += nf
        if g2 in forces: forces[g2] += nf
    return forces

def feet_on_ground(forces: dict, threshold: float = 1.0) -> int:
    return sum(1 for f in forces.values() if f > threshold)

def print_report(mode: int, tx: float, ty: float, tz: float,
                 vx: float, vy: float,
                 wx: float, wy: float, wz: float,
                 peak_force: float, final_tilt_quat: float,
                 final_tilt_xmat: float, report_step: int,
                 timestep: float, tipped: bool):
    """Print the full structural evaluation report to the terminal."""

    # Determine verdict
    # Use the stricter of the two tilt readings
    final_tilt = max(final_tilt_quat, final_tilt_xmat)

    tip_over         = tipped or (final_tilt > TIP_THRESH_DEG)
    impact_failure   = (not tip_over) and (peak_force >= FORCE_CRITICAL)
    successful       = not tip_over and not impact_failure

    if successful:
        verdict_str = f"{G}{B}landed{RST}"
    elif impact_failure:
        verdict_str = f"{BOLD_RED}impact failure{RST}"
    else:
        verdict_str = f"{BOLD_RED}tip-over{RST}"

    force_col = R if peak_force  >= FORCE_CRITICAL else G
    tilt_col  = R if final_tilt  >  TIP_THRESH_DEG  else G
    mode_str  = (f"{G}Stable Flight{RST}" if mode == 1
                 else f"{M}{B}Unstable Stress Test{RST}")

    print(f"\n{W}{B}Landing report{RST}")
    print(f"  Flight mode          : {mode_str}")
    print(f"  Target coordinates   : ({tx:.2f}, {ty:.2f}) m")
    print(f"  Terrain elevation    : {tz:.3f} m  {DIM}(raycast){RST}")
    print(f"  Linear disturbance   : Vx={vx:+.2f}  Vy={vy:+.2f} m/s")
    if mode == 2:
        print(f"  Angular injection    : ωx={wx:+.3f}  ωy={wy:+.3f}  ωz={wz:+.3f} rad/s")
        mag = math.sqrt(wx**2 + wy**2 + wz**2)
        print(f"  Tumble magnitude     : {mag:.3f} rad/s")
    print(f"  Sim time at event    : {report_step * timestep:.2f} s  ({report_step} steps)")
    print(f"  Peak landing force   : {force_col}{peak_force/1000:.1f} kN{RST}  "
          f"{DIM}(failure ≥ {FORCE_CRITICAL/1000:.0f} kN){RST}")
    print(f"  Final tilt (quat)    : {tilt_col}{final_tilt_quat:.2f}°{RST}")
    print(f"  Final tilt (xmat)    : {tilt_col}{final_tilt_xmat:.2f}°{RST}  "
          f"{DIM}(independent cross-check){RST}")
    print(f"  Result               : {verdict_str}")

    if tip_over:
        print(f"{BOLD_RED}Tip-over.{RST}")

    print(f"\n{DIM}Viewer is open. Close the window to continue.{RST}\n")

# Mode 1: Stable Flight

def get_stable_params() -> tuple[float, float, float, float]:
    """Prompt user for coordinates and optional gentle drift. Returns tx,ty,vx,vy."""
    print("\nTarget coordinates")
    tx = prompt_float("X position (15 - 112 m)", COORD_MIN, COORD_MAX)
    ty = prompt_float("Y position (15 - 112 m)", COORD_MIN, COORD_MAX)
    print("\nDisturbance velocities (optional)")
    vx = prompt_float("Vx  horizontal kick  (−5 to +5 m/s)", -5.0, 5.0, default=0.0)
    vy = prompt_float("Vy  horizontal kick  (−5 to +5 m/s)", -5.0, 5.0, default=0.0)
    return tx, ty, vx, vy

# Mode 2: Unstable Stress Test

def get_stress_params(rng: np.random.Generator) -> tuple[float, float, float, float, float, float, float]:
    """
    Prompt for target XY then inject randomised extreme velocities.
    Returns tx, ty, vx, vy, wx, wy, wz.
    """
    print("\nTarget coordinates")
    tx = prompt_float("X position (15 - 112 m)", COORD_MIN, COORD_MAX)
    ty = prompt_float("Y position (15 - 112 m)", COORD_MIN, COORD_MAX)

    # Cross-drift: random direction, magnitude 5.5 - 9.0 m/s
    drift_mag   = rng.uniform(STRESS_DRIFT_MIN, STRESS_DRIFT_MAX)
    drift_angle = rng.uniform(0, 2 * math.pi)
    vx = drift_mag * math.cos(drift_angle)
    vy = drift_mag * math.sin(drift_angle)

    # Tumble: each axis independently randomised in magnitude and sign
    def rand_ang():
        mag  = rng.uniform(STRESS_TUMBLE_MIN, STRESS_TUMBLE_MAX)
        sign = rng.choice([-1.0, 1.0])
        return float(mag * sign)

    wx, wy, wz = rand_ang(), rand_ang(), rand_ang()

    print(f"""
Stress injection
  Cross-drift      : Vx={vx:+.2f}  Vy={vy:+.2f} m/s  {DIM}(|V|={drift_mag:.2f} m/s){RST}
  Tumble (angular) : ωx={wx:+.3f}  ωy={wy:+.3f}  ωz={wz:+.3f} rad/s
  Tumble magnitude : {math.sqrt(wx**2+wy**2+wz**2):.3f} rad/s
""")

    return tx, ty, vx, vy, wx, wy, wz

# Simulation core

def run_simulation(model, data, mode: int,
                   tx: float, ty: float, tz: float, spawn_z: float,
                   vx: float, vy: float,
                   wx: float = 0.0, wy: float = 0.0, wz: float = 0.0):
    """
    Inject state, step physics, sync viewer, return simulation results.
    Returns: peak_force, final_tilt_quat, final_tilt_xmat, tipped, report_step
    """
    # State injection
    mujoco.mj_resetData(model, data)

    data.qpos[0] = tx
    data.qpos[1] = ty
    data.qpos[2] = spawn_z
    data.qpos[3] = 1.0          # quaternion w (identity: upright)
    data.qpos[4:7] = 0.0        # quaternion xyz

    data.qvel[0] = vx           # linear x
    data.qvel[1] = vy           # linear y
    data.qvel[2] = 0.0          # linear z
    data.qvel[3] = wx           # angular x  ← tumble injection
    data.qvel[4] = wy           # angular y
    data.qvel[5] = wz           # angular z

    mujoco.mj_forward(model, data)

    # Header
    mode_label = (f"{G}STABLE FLIGHT{RST}" if mode == 1
                  else f"{M}{B}STRESS TEST{RST}")
    print(f"\nRunning ({mode_label})")
    print(f"  {'Step':>6}  {'Time':>6}  {'Alt':>7}  {'Tilt':>7}  "
          f"{'Peak F':>9}  {'Feet':>5}  Status")
    print(f"  {'─'*6}  {'─'*6}  {'─'*7}  {'─'*7}  {'─'*9}  {'─'*5}  {'─'*20}")

    peak_force  = 0.0
    tipped      = False
    outcome     = "timeout"
    vz_history  = []
    report_step = 0

    # Stress test: allow longer tipped run-out so the viewer shows the full tumble
    tip_runout = 500 if mode == 2 else 200

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.lookat[:]  = [tx, ty, tz + 5]
        viewer.cam.distance   = 70.0
        viewer.cam.elevation  = -18.0
        viewer.cam.azimuth    = 135.0

        for step in range(MAX_STEPS):
            mujoco.mj_step(model, data)
            viewer.sync()

            alt      = data.qpos[2] - tz
            tilt_deg = quat_tilt_deg(data.qpos[3:7])
            forces   = contact_forces(model, data)
            peak_f   = max(forces.values())
            if peak_f > peak_force:
                peak_force = peak_f
            feet = feet_on_ground(forces)

            # Tilt detection
            if tilt_deg > TIP_THRESH_DEG and not tipped:
                tipped      = True
                outcome     = "tipped"
                report_step = step

            # Settle detection (only if not yet tipped)
            vz_history.append(abs(data.qvel[2]))
            if len(vz_history) > STABLE_WINDOW:
                vz_history.pop(0)

            if (not tipped
                    and feet >= 3
                    and len(vz_history) == STABLE_WINDOW
                    and max(vz_history) < STABLE_VZ
                    and abs(data.qvel[0]) < STABLE_HVEL
                    and abs(data.qvel[1]) < STABLE_HVEL):
                outcome     = "landed"
                report_step = step
                break

            # Console readout every 100 steps
            if step % 100 == 0:
                t_s      = step * model.opt.timestep
                force_kN = peak_f / 1000.0

                if tipped:
                    status_str = f"{BOLD_RED}TUMBLING{RST}"
                elif feet > 0:
                    status_str = f"{Y}CONTACT{RST}"
                else:
                    ang_mag = math.sqrt(sum(data.qvel[3+i]**2 for i in range(3)))
                    if ang_mag > 0.5:
                        status_str = f"{M}TUMBLING{RST}" if mode == 2 else f"{DIM}FALLING{RST}"
                    else:
                        status_str = f"{DIM}FALLING{RST}"

                tilt_col = (R if tilt_deg > TIP_THRESH_DEG
                            else (Y if tilt_deg > 8 else G))
                print(f"  {step:>6}  {t_s:>5.1f}s  {alt:>6.2f}m  "
                      f"{tilt_col}{tilt_deg:>6.1f}°{RST}  "
                      f"{force_kN:>7.1f}kN  {feet:>5}  {status_str}")

            # Let tipped lander play out so the viewer shows the full event
            if tipped and step > report_step + tip_runout:
                break

        # Final tilt readings from two independent sources
        final_tilt_quat = quat_tilt_deg(data.qpos[3:7])
        final_tilt_xmat = body_zaxis_tilt_deg(model, data)

        print_report(
            mode, tx, ty, tz, vx, vy, wx, wy, wz,
            peak_force, final_tilt_quat, final_tilt_xmat,
            report_step, model.opt.timestep, tipped,
        )

        # Hold viewer open until user closes it
        while viewer.is_running():
            viewer.sync()
            time.sleep(0.016)

    return peak_force, final_tilt_quat, final_tilt_xmat, tipped, outcome, report_step

# Session orchestrator

def run_session(model, data, rng: np.random.Generator):
    banner()

    mode = mode_menu()

    # Collect parameters for chosen mode
    wx = wy = wz = 0.0
    if mode == 1:
        tx, ty, vx, vy = get_stable_params()
    else:
        tx, ty, vx, vy, wx, wy, wz = get_stress_params(rng)

    # Terrain elevation at target (raycast)
    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)
    tz      = terrain_z(model, data, tx, ty)
    spawn_z = tz + SPAWN_ALTITUDE

    mode_str = "Stable Flight" if mode == 1 else "Unstable Stress Test"
    print(f"""
Run setup ({mode_str})
  Target XY        : ({tx:.2f}, {ty:.2f}) m
  Terrain elevation: {tz:.3f} m   {DIM}(raycast){RST}
  Spawn altitude   : {spawn_z:.3f} m   {DIM}(terrain + {SPAWN_ALTITUDE} m){RST}
  Linear injection : Vx={vx:+.2f}  Vy={vy:+.2f} m/s""")

    if mode == 2:
        print(f"  Angular injection: ωx={wx:+.3f}  ωy={wy:+.3f}  ωz={wz:+.3f} rad/s")

    print()

    run_simulation(
        model, data, mode,
        tx, ty, tz, spawn_z,
        vx, vy, wx, wy, wz,
    )

# Entry point

if __name__ == "__main__":
    print(f"\n{DIM}Loading model: {XML_PATH}{RST}")
    try:
        model = mujoco.MjModel.from_xml_path(str(XML_PATH))
        data  = mujoco.MjData(model)
    except Exception as e:
        print(f"{R}Failed to load model: {e}{RST}")
        sys.exit(1)

    rng = np.random.default_rng()   # unseeded: different tumble every run

    while True:
        run_session(model, data, rng)
        again = input(f"\n{C}Run another landing? (y/n):{RST} ").strip().lower()
        if again != "y":
            print(f"\n{DIM}Session closed.{RST}\n")
            break

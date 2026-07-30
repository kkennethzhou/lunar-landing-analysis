# Known Issues

Current as of the initial public release. Two blocking defects, several smaller ones.

Each entry states the symptom, the evidence, the root cause, and the fix — in that order, because the symptom is usually what someone else will notice first.

---

## 🔴 BLOCKING-1 — Landing gear cannot support the vehicle at rest

**Symptom.** Leg stiffness is `LEG_STIFFNESS = 320` N/m. The lander appears to land successfully, and `peak_stroke_m` reports 0.6385 m against a 0.8 m limit, so nothing looks wrong.

**Evidence.** Static per-leg load is

```
F = m·g / 4 = 1000 × 1.62 / 4 = 405 N
```

At 320 N/m that requires **1.27 m** of spring compression. The joint is `range="0 0.8"`. The lander cannot stand on its own legs — at rest, all four should be pinned against their stroke limits.

**Root cause.** The springs were sized on landing kinetic energy (1125 J at 1.5 m/s) without checking the static-support constraint. Energy sizing alone admits arbitrarily soft springs.

**Why it isn't caught at runtime.** See BLOCKING-2 — the engine is still firing, so the legs never carry full weight.

**Fix.** Satisfy both constraints:

| Constraint | Requirement | Implies |
|---|---|---|
| Static support | compression ≤ 20% of stroke at rest | k ≥ 405 / 0.16 ≈ **2,500 N/m** |
| Dynamic stroke | ≤ 75% of stroke at 1.5 m/s touchdown | solve `2k·x² − W·x − KE = 0` |

Trade study:

| k (N/m) | static x | % stroke | dynamic x @ 1.5 m/s | ζ at c=360 |
|---|---|---|---|---|
| 320 (current) | 1.266 m | **158% — impossible** | 3.099 m | 0.64 |
| 2000 | 0.203 m | 25% | 0.770 m | 0.25 |
| **3000** | **0.135 m** | **17%** | **0.589 m** | 0.21 |
| 4000 | 0.101 m | 13% | 0.490 m | 0.18 |
| 8000 | 0.051 m | 6% | 0.321 m | 0.13 |

**Recommended: k ≈ 3,000 N/m.** For ζ ≈ 0.7 (near-critical, suppresses bounce without over-damping):

```
c = 0.7 × 2√(k · m/4) = 0.7 × 2√(3000 × 250) ≈ 1,200 N·s/m
```

The current 360 N·s/m gives ζ = 0.21 — badly under-damped.

**Regression test.** `tests/test_static_equilibrium.py` spawns the lander at rest on flat terrain with the engine off, steps 5 s, and asserts each leg settles to `405/k` ± 10% without reaching the stroke limit. This is the test whose absence let 320 N/m through.

---

## 🔴 BLOCKING-2 — Thrust is never cut at touchdown

**Symptom.** `n_bottomed_out: 0` and `bottom_out_ok: true` across all 500 trials, despite BLOCKING-1 making that impossible.

**Evidence.** `descent_thrust()` applies

```python
HOVER_THRUST = LANDER_MASS * GRAVITY   # 1620 N
thrust = HOVER_THRUST + LANDER_MASS * a_ff + KP_VZ * err + KD_VZ * derr
```

on **every** step, including after contact. The settle detector fires while the engine is holding most of the vehicle's weight, so the legs only ever compress to 0.638 m.

**Consequence.** The simulation declares "landed" with the engine running. Every peak-force and stroke measurement is taken from a partially supported vehicle, so the bottom-out check passes for the wrong reason and force magnitudes are understated.

**Fix.** Cut thrust on touchdown: when ≥3 footpads register contact > 1 N, ramp thrust to zero over ~0.2 s (Apollo cut off on the contact-probe signal). Continue stepping until the vehicle is static on its legs, and only then evaluate settling.

**Ordering matters.** Fixing this before BLOCKING-1 will correctly cause every trial to bottom out and fail. Fix stiffness first.

---

## 🟠 OPEN-1 — No terrain signal in the safety map

**Symptom.** The core deliverable — a recommended landing coordinate — is not statistically defensible.

**Evidence** (500 trials, 16% failure rate):

```
corr(force, local_slope)     = +0.022  (p = 0.65)   ← no signal
corr(force, elevation)       = -0.024  (p = 0.63)   ← no signal
corr(force, |v_horizontal|)  = +0.495               ← dominates
corr(failed, local_slope)    = +0.054  (t = 1.20)   ← not significant
```

Failure rate by slope quartile is flat:

| Slope range | Failure rate |
|---|---|
| 0.4–3.8° | 14.4% |
| 3.8–5.9° | 14.4% |
| 5.9–8.1° | 18.4% |
| 8.1–21.6° | 16.8% |

Bootstrap 95% spread on the recommended coordinate: **58.7 m** across a 97 m domain.

**Interpretation.** The simulation is measuring approach conditions (drift speed), not site quality. Any "optimal site" extracted from this data is an artifact of where trials happened to cluster.

**Status: genuinely open.** Three things could still change it, in order of promise:

1. **Partial correlation of force against slope controlling for `|v|`.** Drift variance may be masking a real terrain effect. This is the single most likely route to recovering a signal and has not been run.
2. Fixing BLOCKING-1 and BLOCKING-2, so the legs actually carry load through touchdown and terrain has a mechanism to matter.
3. Widening terrain roughness — the current heightfield may be too benign for a 2.5 m leg span to notice.

If none of these produce a signal, the correct conclusion is that this experimental design cannot resolve site quality, and the negative result stands.

---

## 🟡 MINOR-1 — Two disagreeing density-confound implementations

`validate_results.py` reports **98.58 m**; `plot_landing_zones.py` reports **11.72 m**. One takes `argmax` of a smoothed count field; the other calls `recommended_site()` with constant forces.

**Fix.** One definition in a shared helper, imported by both.

**Also.** The test currently "passes" degenerately — it compares two maps that are both noise, and a large distance between two random fields is not evidence of signal. When the signal test fails, this should report `INCONCLUSIVE`, not `PASS`.

---

## 🟡 MINOR-2 — Whole-domain tilt reported as a slope statistic

`relief / width = 4.44 / 127 ≈ 2°` describes the end-to-end tilt of the entire terrain. It is not a slope, and it is not an average of anything.

Real per-cell gradient statistics: mean **6.46°**, median 5.94°, p95 **12.73°**, max **27.77°**, with 63% of cells above 5° and 3.4% above the 15° tip-over threshold.

The substitution understates terrain difficulty by more than 3× and is easy to make by accident.

---

## ✅ FIXED — Mesh collision was silently convex

The terrain was loaded as `<geom type="mesh">`. **MuJoCo evaluates mesh–mesh collision against the convex hull**, so every crater and depression was invisible to the physics engine while remaining visible in the render. The lander was touching down on a smooth dome.

No error, no warning — the simulation modelled different terrain than it displayed, and this partly explains the original zero terrain correlation.

**Fixed** by switching to `<hfield>`, which supports exact non-convex geometry.

If you take one thing from this repo: this class of bug produces plausible numbers forever. The only defense is validating outputs against physical expectations, not reading code.

---

## ✅ FIXED — Peak forces were solver artifacts

With under-sized gear, legs bottomed out on their joint limits every trial. Reported loads of 356–1,203 kN (up to 128 g on a 1,000 kg vehicle — Apollo LM design limit was under 10 g) were the response of `solref="0.005 1"`, not landing-gear load. Changing that one solver parameter changed every published number.

**Fixed** by logging strut load from the joint DOF and asserting no leg reaches its stroke limit. Contact force is retained as a separate, clearly labelled column.

---

## ✅ FIXED — Mass discrepancy

An explicit `<inertial mass="800">` on `lander_base` silently overrode the bus (400) + engine (50) + tank (200) geom masses. Actual model mass was 960 kg while every document claimed 1,000 kg, and `diaginertia="300 300 150"` was hand-picked against a correct Izz of 768.

**Fixed** by removing the `<inertial>` element, deriving mass and inertia from geometry, and asserting `model.body_mass.sum() == 1000.0` at generation time.

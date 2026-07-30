# Known Issues

Current as of the round-3 public release. Both blocking defects from round 2 are fixed; the open question about whether a terrain signal exists has been tested, not just hypothesized about, and the negative result stands.

Each entry states the symptom, the evidence, the root cause, and the fix — in that order, because the symptom is usually what someone else will notice first.

---

## 🟠 OPEN-1 — No terrain signal in the safety map

**Symptom.** The core deliverable — a recommended landing coordinate — is not statistically defensible.

**Evidence** (500 trials, seed 42, 15.6% failure rate, both BLOCKING-1 and BLOCKING-2 fixed):

```
corr(force, local_slope)                          = +0.025  (p = 0.61)   <- no signal
corr(force, elevation)                             = -0.022  (p = 0.65)   <- no signal
corr(force, |v_horizontal|)                        = +0.525  (p = 3e-31)  <- dominates
corr(force, local_slope | |v_horizontal|)          = +0.005  (p = 0.92)   <- partial correlation: still no signal
corr(failed, local_slope)                          = +0.070  (p = 0.12)   <- not significant
```

Failure rate by slope quartile is flat:

| Slope range | Failure rate |
|---|---|
| 0.4–3.8° | 14.4% |
| 3.8–5.9° | 16.8% |
| 5.9–8.1° | 14.4% |
| 8.1–21.6° | 16.8% |

Bootstrap 95% spread on the recommended coordinate: **58.4 m** across a 97 m domain.

**Interpretation.** The simulation is measuring approach conditions (drift speed), not site quality. Any "optimal site" extracted from this data is an artifact of where trials happened to cluster.

**Status: negative result, now well-supported.** Round 2 listed three things that could still change this, in order of promise. All three that were physics/analysis fixes have now been tried:

1. ~~Partial correlation of force against slope controlling for `|v|`.~~ **Tested — no signal.** `r=+0.005, p=0.92`, flatter than the raw correlation, not masked by it.
2. ~~Fixing BLOCKING-1 and BLOCKING-2, so the legs actually carry load through touchdown.~~ **Fixed.** See below. Forces are now physically real (5.7–23.2 kN range, no bottom-outs) and the engine genuinely cuts at touchdown — and the signal is still absent.
3. **Widening terrain roughness** — the current heightfield may be too benign for a 2.5 m leg span to notice. This is the one remaining untested hypothesis, and it requires redesigning the terrain generator (larger craters, steeper local gradients relative to the lander's footprint), not another physics or analysis fix.

With the two plausible physics confounds eliminated and the most promising analysis (partial correlation) tested directly, the correct conclusion for this experimental design, as built, is that it does not resolve site quality — the negative result stands. It would take a deliberately harsher terrain generator, not a physics fix, to test hypothesis 3.

---

## 🟡 MINOR-2 — Whole-domain tilt reported as a slope statistic

`relief / width = 4.44 / 127 ≈ 2°` describes the end-to-end tilt of the entire terrain. It is not a slope, and it is not an average of anything.

Real per-cell gradient statistics: mean **6.46°**, median 5.94°, p95 **12.73°**, max **27.77°**, with 63% of cells above 5° and 3.4% above the 15° tip-over threshold.

The substitution understates terrain difficulty by more than 3× and is easy to make by accident.

---

## ✅ FIXED — Landing gear could not support the vehicle at rest (BLOCKING-1)

**Symptom.** Leg stiffness was `LEG_STIFFNESS = 320` N/m. The lander appeared to land successfully, and `peak_stroke_m` reported 0.6385 m against a 0.8 m limit, so nothing looked wrong.

**Root cause.** The springs were sized on landing kinetic energy (1125 J at 1.5 m/s) without checking the static-support constraint. Static per-leg load is `F = m·g/4 = 1000×1.62/4 = 405 N`; at 320 N/m that requires **1.27 m** of spring compression against a 0.8 m joint range — physically impossible. Energy sizing alone admits arbitrarily soft springs.

**Why it wasn't caught at runtime.** The engine was still firing at touchdown (see the thrust-cutoff fix below), so the legs never carried full weight and the impossible compression never had to happen.

**Fix.** `LEG_STIFFNESS = 3000` N/m, satisfying both constraints:

| Constraint | Requirement | Value at k=3000 |
|---|---|---|
| Static support | compression ≤ 20% of stroke at rest | 0.135 m (17%) |
| Dynamic stroke | ≤ 75% of stroke at 1.5 m/s touchdown | 0.589 m (74%) |

Damping raised from 360 to `LEG_DAMPING = 1200` N·s/m for ζ≈0.7 (was ζ=0.21, badly under-damped):

```
c = 0.7 × 2√(k · m/4) = 0.7 × 2√(3000 × 250) ≈ 1,200 N·s/m
```

**Regression.** `tests/test_static_equilibrium.py` asserts both constraints directly from first principles, and drops the lander with the engine off to confirm it settles without bottoming out. It's the test whose absence let 320 N/m through in the first place — it was written before this fix landed and initially failed by design.

---

## ✅ FIXED — Thrust was never cut at touchdown (BLOCKING-2)

**Symptom.** `n_bottomed_out: 0` and `bottom_out_ok: true` across all 500 trials, despite BLOCKING-1 making that impossible.

**Root cause.** `descent_thrust()` applied `HOVER_THRUST = m·g` feed-forward on **every** step, including after contact. The settle detector fired while the engine was still holding most of the vehicle's weight, so the legs only ever compressed to 0.638 m.

**Consequence.** The simulation declared "landed" with the engine running. Every peak-force and stroke measurement was taken from a partially supported vehicle, so the bottom-out check passed for the wrong reason and force magnitudes were understated.

**Fix.** Thrust ramps to zero over ~0.2 s once ≥3 footpads register contact >1 N (Apollo cut off on the same contact-probe signal), then the sim keeps stepping until the vehicle is genuinely static on its legs before evaluating settling. Ordering mattered: this had to land after BLOCKING-1, since fixing it first would have correctly caused every trial to bottom out against the undersized gear.

**Result.** Forces are now physically real: 5.7–23.2 kN range across 500 trials (mean 13.1 kN), no bottomed-out legs, and a single-trial check confirms `ctrl=0` with total foot contact force matching the vehicle's full 1,620 N static weight at rest.

---

## ✅ FIXED — Two disagreeing density-confound implementations

`validate_results.py` used to report **98.58 m**; `plot_landing_zones.py` reported **11.72 m**. One took `argmax` of a smoothed count field; the other called `recommended_site()` with constant forces — two different comparisons, not two measurements of the same thing.

**Fix.** One definition (`lunar_lander.site_selection.recommended_site`, constant-force argmin), imported by both scripts.

**Also fixed.** The test used to "pass" degenerately whenever the signal test failed — comparing two maps that are both noise, where a large distance between them isn't evidence of anything. It now reports `INCONCLUSIVE` instead of `PASS`/`FAIL` whenever the signal test above it fails.

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

# Known issues

Current as of the 0.4.0 release. Both blocking defects are fixed, including the slide axis, which used to extend the leg under load. The open question, whether a terrain signal shows up once the physics is fixed, has been run again on that model. It does not.

## OPEN-1. No terrain signal in the safety map

The recommended landing coordinate is not statistically defensible.

Evidence from 500 trials, seed 42, on the compression legs. 405 landed, 91 tipped, 4 timed out (19.0% not landed). Recorded in `docs/summary_stats.json`.

```
corr(force, local_slope)                  = +0.060  (p = 0.23)
corr(force, elevation)                    = -0.013  (p = 0.80)
corr(force, |v_horizontal|)               = +0.539  (p = 7e-32)
corr(force, local_slope | |v_horizontal|) = +0.041  (p = 0.41)
corr(failed, local_slope)                 = +0.099  (p = 0.027)
```

The failure correlation has p under 0.05 and `|r|` under the 0.2 bar the validator requires, so that check still fails.

Failure rate by slope quartile:

| Slope range | Failure rate |
|---|---|
| 0.4-3.8° | 17.6% |
| 3.8-5.9° | 16.8% |
| 5.9-8.1° | 19.2% |
| 8.1-21.6° | 22.4% |

Bootstrap 95% spread on the recommended coordinate: 55.0 m across a 97 m domain.

The simulation is mostly measuring approach conditions. A coordinate pulled out of this map is where the trials happened to cluster.

Three follow-ups were listed before this release:

1. Partial correlation of force against slope, controlling for `|v|`. Tested on the compression-leg campaign. r=+0.041, p=0.41. Flatter than the raw correlation.
2. Fix the leg stiffness, the slide axis, and the thrust cutoff so the legs carry the vehicle. Done. See below. Peak strut force on the 500-trial run is 6.0-23.4 kN (mean 13.2 kN). Maximum stroke is 0.797 m. No leg bottomed out. The slope correlation is still absent.
3. Rougher terrain and measurement at contact. Adjacent feet are 4.0 m apart (not 2.5 m). The published CSV `x,y` are spawn coordinates, not touchdown; with drift up to 2.3 m/s the lander can be tens of metres away at contact. The approved OPEN-1 experiment logs touchdown and foot positions, runs a 500-trial harsh campaign on the compression-leg model, and a low-drift control. See [open1_experiment.md](open1_experiment.md). `python -m lunar_lander.open1_campaigns` writes only under an isolated output directory.

The committed 40-trial probe (`docs/roughness_comparison.json`, seed 42) is from before the 0.4.0 compression-leg campaign (max stroke 0.14–0.16 m versus 0.797 m on v0.4.0). It raises relief from 4.44 m to 8.77 m and the failure rate from 12.5% to 37.5%. Force-slope correlation on the harsher grid is r=0.039, p=0.85. That file does not replace the 500-trial result and is stale relative to the current lander.

OPEN-1 experiment results (500 trials, seed 42) are in [open1_experiment.md](open1_experiment.md) and `docs/open1/`. Peak force still does not track slope at `|r| ≥ 0.2`. Measuring failure at touchdown instead of spawn on the same nominal campaign yields `r=+0.268`. Leave this issue open until peak load meets the bar.

## MINOR-2. Whole-domain tilt quoted as a slope

`relief / width = 4.44 / 127` is about 2°. That is the end-to-end tilt of the patch. It is not a slope and it is not an average.

Per-cell gradient: mean 6.46°, median 5.94°, p95 12.73°, max 27.77°. 63% of cells are steeper than 5°. 3.4% are steeper than the 15° tip threshold.

Using relief/width understates the terrain by more than 3x.

## Fixed. Landing gear could not hold the vehicle (BLOCKING-1)

`LEG_STIFFNESS` was 320 N/m. Trials looked successful. `peak_stroke_m` was 0.6385 m against a 0.8 m limit, so nothing in the log looked broken.

The springs were sized on landing kinetic energy (1125 J at 1.5 m/s) and not checked at rest. Static load per leg is `F = m*g/4 = 1000*1.62/4 = 405 N`. At 320 N/m that is 1.27 m of compression. The joint only has 0.8 m.

It did not fail at runtime because the engine was still firing at touchdown (next section). The legs never carried full weight, so the impossible compression never had to happen.

Fix: `LEG_STIFFNESS = 3000` N/m. The slide axis was also `0 0 -1`, so positive q extended the foot and the spring force was zero with the vehicle parked. Raising k did not create a ride height until the axis became `0 0 1`. q = 0 is the extended leg, positive q compresses it, and the spring pushes the bus up.

| Constraint | Requirement | At k=3000 |
|---|---|---|
| Static support | compression at most 20% of stroke at rest | 0.135 m (17%) |
| Dynamic stroke | at most 75% of stroke at 1.5 m/s | 0.589 m (74%) |

Damping went from 360 to `LEG_DAMPING = 1200` N s/m, about zeta 0.7. It was zeta 0.21.

```
c = 0.7 * 2 * sqrt(k * m/4) = 0.7 * 2 * sqrt(3000 * 250) = 1200 N s/m
```

`tests/test_static_equilibrium.py` checks both constraints and drops the lander with the engine off. That test is what would have caught 320 N/m. It was written against the old spring and failed until the stiffness changed.

## Fixed. Thrust never cut at touchdown (BLOCKING-2)

`n_bottomed_out` was 0 on all 500 trials even though BLOCKING-1 says that should have been impossible.

`descent_thrust()` applied `HOVER_THRUST = m*g` on every step, including after contact. The settle check fired while the engine still held most of the weight. Legs only compressed to 0.638 m.

The log said landed with the engine running. Peak force and stroke were measured on a vehicle that was still being held up. The bottom-out check passed for that reason, and the forces were low.

Thrust now ramps to zero over about 0.2 s once at least three footpads register contact above 1 N. The sim keeps stepping until the vehicle is sitting on its legs. This had to land after the stiffness fix. Doing it first would have bottomed out every trial on the old springs.

On the 500-trial rerun with the compression legs, peak strut force is 6.0-23.4 kN (mean 13.2 kN) and the maximum stroke is 0.797 m. No leg bottoms out. Thrust is off once the vehicle has settled.

## Fixed. Two different density-confound numbers

`validate_results.py` reported 98.58 m. `plot_landing_zones.py` reported 11.72 m. One took `argmax` of a smoothed count field. The other called `recommended_site()` with constant forces. Those are different comparisons.

Both scripts now import `lunar_lander.site_selection.recommended_site` and use a constant-force argmin.

The old test also passed whenever the signal test failed. Two noisy maps being far apart is not evidence. If the signal test fails, this check is now `INCONCLUSIVE` instead of pass or fail.

## Fixed. Mesh collision was the convex hull

Terrain was `<geom type="mesh">`. MuJoCo mesh-mesh contact uses the convex hull, so craters were visible and not colliding. The lander touched a dome.

There was no error. The render and the contact model were different surfaces. That is part of why the early terrain correlation was zero.

Switched to `<hfield>`. `python -m lunar_lander.convex_gap` writes the gap at the crater floors.

## Fixed. Peak forces were solver artifacts

With the undersized gear, legs hit the joint limit every trial. Reported loads of 356-1,203 kN are up to about 128 g on a 1,000 kg vehicle. The Apollo LM design limit was under 10 g. Those numbers followed `solref="0.005 1"`, not the landing gear. Changing that one solver parameter changed every published force.

Strut load now comes from the joint DOF, and a leg that reaches its stroke fails the check. Contact force is still logged, in its own column.

## Fixed. Mass was 960 kg, not 1,000 kg

An explicit `<inertial mass="800">` on `lander_base` overrode the bus (400), engine (50), and tank (200) geom masses. The model was 960 kg while the docs said 1,000 kg. `diaginertia="300 300 150"` was typed in against a calculated Izz of 768.

The `<inertial>` element is gone. Mass and inertia come from the geoms. The check is the lander subtree mass, not `model.body_mass.sum()`. The heightfield body gets a huge geometry mass and is not part of the lander.

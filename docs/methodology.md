# Methodology

## 1. Imagery acquisition

40 LROC Narrow Angle Camera EDR strips requested from the NASA PDS archive; 38 retrieved. All are Left NAC (`LE`) products spanning 2011–2020.

Each strip is up to 2,532 × 52,224 px (~132 MP) in the 2× crosstrack summation mode. NAC is a **pushbroom line scanner** — it builds an image line by line as the orbiter flies, so every scanline has a different camera centre. This matters in step 2.

PDS3 files are a text header followed by raw pixel data. Pixel offset is:

```
offset = (^IMAGE - 1) × RECORD_BYTES
```

16-bit little-endian values, contrast-stretched at the 2nd/98th percentile to 8-bit PNG.

## 2. Why COLMAP failed

3 of 38 images registered, producing **one** 3D point. Full post-mortem in [colmap-failure.md](colmap-failure.md). Three independent causes:

1. **Pushbroom geometry** violates COLMAP's single-centre-of-projection pinhole model outright
2. **No overlap** — strips span 2011–2020 across different orbits, so different footprints and sun angles
3. **`--data_type video`** assumes short-baseline sequential frames

Also: only `LE` products were downloaded. NAC left/right pairs are the natural stereo geometry, so the one configuration with a chance of working was never attempted.

## 3. Procedural terrain

128×128 vertex heightfield at 1 m spacing → 127 × 127 m.

```
Z(x, y) = Σ sinusoidal octaves − Σ craters + Σ boulders
```

Four sinusoidal octaves (broad undulation, medium ridges, fine texture), seven craters, four boulders. Craters use an inverted Gaussian basin plus an **annular** rim term — an earlier difference-of-centred-Gaussians formulation produced a rim of only 0.034 × depth (6.9 cm on the deepest crater) while also reducing centre depth to 0.75 × the nominal parameter.

**Slope statistics** (per-cell gradient, not `relief/width`):

| Statistic | Value |
|---|---|
| Relief | 4.44 m |
| Mean slope | 6.46° |
| Median | 5.94° |
| p95 | 12.73° |
| Max | 27.77° |
| Cells > 5° | 63.4% |
| Cells > 15° (tip threshold) | 3.4% |

`relief / width = 4.44/127 ≈ 2°` is the whole-domain tilt and understates difficulty by 3×.

**Collision uses `<hfield>`, not `<geom type="mesh">`.** MuJoCo evaluates mesh collision against the convex hull, which erases every crater while leaving them visible in the render. `python -m lunar_lander.convex_gap` writes the gap at each crater floor to `docs/convex_gap.json`.

The heightfield file is an 8-bit PNG. Pixel 0 is z = 0 and pixel 255 is the full relief, so the vertical step is relief/255 (about 1.7 cm on the 4.44 m nominal grid). That is the collision surface. It is not a centimetre-accurate elevation model.

## 4. Lander model

| Parameter | Value |
|---|---|
| Total mass | 1,000 kg (asserted at generation) |
| Gravity | 1.62 m/s² |
| Timestep | 0.002 s (500 Hz) |
| Legs | 4 × independent spring-damper, spherical footpads |
| Stroke | 0.8 m |
| Engine | single thrust actuator, 5 kN max |

Mass and inertia are derived from geometry — no explicit `<inertial>` element, which would silently override geom masses.

**Leg sizing must satisfy two constraints**, not one:

- Static: `405 N / k ≤ 0.2 × 0.8 m` ⇒ `k ≥ 2,500 N/m`
- Dynamic: solve `2k·x² − W·x − KE = 0` at 1.5 m/s ⇒ `x ≤ 0.6 m`

The shipped stiffness is 3,000 N/m and the damping is 1,200 N·s/m. An earlier 320 N/m spring could not hold the lander up inside the stroke. See known-issues BLOCKING-1.

## 5. Descent control

Velocity profile targeting 1.5 m/s at touchdown:

```
v_target(agl) = −√(v_touchdown² + 2·a·agl)     for agl < 25 m
v_target(agl) = −10 m/s                        otherwise
```

PD on velocity error plus feed-forward `m·(g + a_ff)`. Pure PD lagged the ramping target under constant gravity bias (Type-1 steady-state error) and arrived at 2.6–2.8 m/s; the feed-forward term closes that gap.

Once three feet are in contact, thrust ramps to zero over 0.2 s. An earlier controller kept the hover feed-forward on after touchdown, so the legs never carried the vehicle's weight (known-issues BLOCKING-2).

## 6. Monte Carlo

500 trials, `seed = 42`. Randomized: spawn position (uniform over 15–112 m), horizontal drift (±2.3 m/s per axis), initial tilt, initial angular velocity, terrain friction, touchdown speed target (1.0–2.3 m/s).

**Failure criteria:** tilt > 15°, leg force > 50 kN, stroke ≥ 0.8 m (bottomed out), body acceleration > 10 g.

**Success:** ≥3 footpads in contact > 1 N for 150 *consecutive* steps, |vz| < 0.02 m/s, |vx|,|vy| < 0.05 m/s.

Result, from `output/summary_stats.json`: 422 landed, 43 tipped, 16 over-acceleration, 19 timeouts (15.6% not landed). Peak stroke 0.144 m. A separate roughness probe, `python -m lunar_lander.compare_roughness`, reruns a shorter campaign on a deeper crater field and writes `docs/roughness_comparison.json`. It does not replace the 500-trial result.

## 7. Site selection

**Nadaraya-Watson kernel regression** — a local weighted *mean*:

```
force_field = gaussian_filter(force_sum, σ) / gaussian_filter(count, σ)
```

Cells below 5 effective samples are masked; the recommendation is the global **minimum** of the masked field. σ = 8 cells = 3.90 m over the 97 × 97 m spawn domain.

An earlier version accumulated with `+=`, so three mediocre trials outranked one good one. Removing all force data moved that answer by only 2.2 m — it was a density map.

## 8. Validation

Four checks, all run automatically:

1. **Signal** — `corr(force, slope)` with p-value, plus the partial correlation controlling for horizontal drift. Below |r| = 0.2, the map has no terrain information.
2. **Failure vs terrain** — whether failures track slope.
3. **Density confound** — how far the answer moves when forces are replaced by a constant. Reported inconclusive when the signal test has already failed.
4. **Bootstrap** — 1000 resamples; spread of the recommended coordinate.
5. **Bottom-out** — hard fail if any leg reached its stroke limit.

Current status: the signal and bootstrap checks fail. See [known-issues.md](known-issues.md) OPEN-1.

# Methodology

## 1. Imagery

40 LROC Narrow Angle Camera EDR strips were requested from the NASA PDS archive. 38 came back. All are Left NAC (`LE`) products from 2011-2020.

Each strip is up to 2,532 by 52,224 px (about 132 MP) in the 2x crosstrack summation mode. NAC is a pushbroom line scanner. It builds the image one line at a time as the orbiter moves, so each scanline has its own camera centre. That is why step 2 failed.

PDS3 files are a text header and then raw pixels. The pixel offset is:

```
offset = (^IMAGE - 1) * RECORD_BYTES
```

Pixels are 16-bit little-endian, contrast-stretched at the 2nd and 98th percentile into an 8-bit PNG.

## 2. Why COLMAP failed

3 of 38 images registered. The model had one 3D point. The writeup is [colmap-failure.md](colmap-failure.md).

1. Pushbroom geometry does not fit COLMAP's pinhole model (one centre of projection per image).
2. The strips do not overlap. They span 2011-2020 and different orbits, so the footprints and sun angles differ.
3. `--data_type video` assumes short-baseline frames in order.

Only `LE` products were downloaded. The left/right NAC pair is the stereo geometry, and that pair was never requested.

## 3. Procedural terrain

128 by 128 vertices at 1 m spacing, so 127 m by 127 m.

```
Z(x, y) = sum of sinusoidal octaves - sum of craters + sum of boulders
```

Four octaves (broad undulation, medium ridges, fine texture), seven craters, four boulders. Craters are an inverted Gaussian basin plus an annular rim. An earlier difference of two centred Gaussians made a rim of only 0.034 times depth (6.9 cm on the deepest crater) and cut the centre depth to 0.75 times the parameter.

Slopes below are per-cell gradients, not `relief/width`.

| Statistic | Value |
|---|---|
| Relief | 4.44 m |
| Mean slope | 6.46° |
| Median | 5.94° |
| p95 | 12.73° |
| Max | 27.77° |
| Cells steeper than 5° | 63.4% |
| Cells steeper than 15° | 3.4% |

`relief / width = 4.44/127` is about 2°. That is the tilt of the whole patch, and it understates the cell slopes by about 3x.

Contact uses `<hfield>`, not `<geom type="mesh">`. MuJoCo mesh collision uses the convex hull, which covers the craters while the render still shows them. `python -m lunar_lander.convex_gap` writes `docs/convex_gap.json`.

The heightfield file is an 8-bit PNG. Pixel 0 is z = 0 and pixel 255 is the full relief, so the vertical step is relief/255 (about 1.7 cm on the 4.44 m grid). That quantized surface is what the contact solver sees.

## 4. Lander

| Parameter | Value |
|---|---|
| Total mass | 1,000 kg (checked when the model is built) |
| Gravity | 1.62 m/s^2 |
| Timestep | 0.002 s |
| Legs | 4 independent spring-dampers, spherical feet |
| Stroke | 0.8 m |
| Engine | one actuator, 5 kN max |

There is no `<inertial>` element. One of those had been overriding the geom masses.

The legs have to meet two constraints:

- Static: `405 N / k <= 0.2 * 0.8 m`, so `k >= 2,500 N/m`
- Dynamic: solve `2k*x^2 - W*x - KE = 0` at 1.5 m/s, with `x <= 0.6 m`

Shipped stiffness is 3,000 N/m. Damping is 1,200 N s/m. The slide axis is `0 0 1`: positive q compresses the leg. The old 320 N/m spring, on a downward axis, could not hold the lander inside the stroke. See BLOCKING-1 in [known-issues.md](known-issues.md).

## 5. Descent control

Target speed is 1.5 m/s at touchdown:

```
v_target(agl) = -sqrt(v_touchdown^2 + 2*a*agl)    for agl < 25 m
v_target(agl) = -10 m/s                            otherwise
```

Control is PD on velocity error plus feed-forward `m*(g + a_ff)`. PD alone lagged the ramp under a constant gravity bias and arrived at 2.6-2.8 m/s. The feed-forward term closes that.

Once three feet are in contact, thrust ramps to zero over 0.2 s. The earlier controller left the hover feed-forward on, so the legs never carried the weight (BLOCKING-2).

## 6. Monte Carlo

500 trials, `seed = 42`. Each trial draws spawn position (uniform on 15-112 m), horizontal drift (up to 2.3 m/s per axis), initial tilt, initial angular velocity, terrain friction, and a touchdown speed target (1.0-2.3 m/s).

A trial fails on tilt over 15°, leg force over 50 kN, stroke at the 0.8 m limit, or body acceleration over 10 g.

A landing counts if at least 3 footpads stay in contact above 1 N for 150 consecutive steps, with `|vz| < 0.02` m/s and `|vx|`, `|vy| < 0.05` m/s.

From `docs/summary_stats.json`, the same file the pipeline writes to `output/summary_stats.json`: 405 landed, 91 tipped, 4 timeouts (19.0% not landed). Peak stroke 0.797 m. No trial exceeded the acceleration limit.

`python -m lunar_lander.compare_roughness` reruns a shorter campaign on a deeper crater field and writes `docs/roughness_comparison.json`. It does not replace the 500-trial result. The approved OPEN-1 follow-up (touchdown logging, 500-trial harsh campaign, low-drift control) is `python -m lunar_lander.open1_campaigns`; methods are in [open1_experiment.md](open1_experiment.md).

Adjacent feet are 4.0 m apart in the MJCF.

## 7. Site selection

Nadaraya-Watson kernel regression, a local weighted mean:

```
force_field = gaussian_filter(force_sum, sigma) / gaussian_filter(count, sigma)
```

Cells under 5 effective samples are masked. The recommendation is the minimum of what remains. Sigma is 8 cells, 3.90 m, over the 97 m by 97 m spawn window.

An earlier version accumulated with `+=`, so three ordinary trials beat one good one. Removing the force data only moved that answer by 2.2 m.

## 8. Validation

The script runs these and writes a verdict:

1. Signal. `corr(force, slope)` and the partial correlation controlling for horizontal drift. Below `|r| = 0.2` the map has no terrain information.
2. Whether failures track slope.
3. How far the recommended site moves when every force is replaced by a constant. Inconclusive if the signal test already failed.
4. Bootstrap. 1000 resamples. Spread of the recommended coordinate.
5. Bottom-out. Fails if any leg reached its stroke limit.

On the stored campaign the signal and bootstrap checks fail. See OPEN-1 in [known-issues.md](known-issues.md).

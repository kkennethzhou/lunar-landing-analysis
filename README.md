# Lunar landing site analysis

MuJoCo Monte Carlo landings on a procedural 127 m patch. The code picks a touchdown coordinate from the trials that landed, then checks whether that coordinate is about the terrain. On the 500-trial run it is not.

License: [MIT](LICENSE). Python 3.10+.

The pipeline runs, and the two physics bugs that could have faked a null result are fixed (see [Known issues](#known-issues)). The site-selection checks still fail. Peak leg load does not track slope, the partial correlation controlling for drift is flat, and the recommended point moves tens of metres under bootstrap.

## What it does

The original plan was to build terrain from NASA LROC Narrow Angle Camera strips. That reconstruction did not work. Details are in [docs/colmap-failure.md](docs/colmap-failure.md). The simulation uses a procedural heightfield instead: layered sinusoids, Gaussian craters, and a few boulders.

From there:

1. Build the terrain and the MuJoCo model.
2. Drop a 1,000 kg lander 500 times with randomized position, drift, tilt, and touchdown speed.
3. Estimate a safest coordinate with kernel regression on the landings that succeeded.
4. Test whether that map has a terrain signal.

## Result

500 trials, seed 42, on the compression legs (slide axis `0 0 1`, 3,000 N/m). 405 landed, 91 tipped, 4 timed out (19.0% not landed). No leg bottomed out (max stroke 0.797 m of 0.8 m). Thrust is cut at touchdown. The record is `docs/summary_stats.json`.

```
corr(peak_leg_force, local_terrain_slope)                     = +0.060  (p = 0.23)
corr(peak_leg_force, horizontal_drift)                        = +0.539  (p = 7e-32)
corr(peak_leg_force, local_terrain_slope | horizontal_drift)  = +0.041  (p = 0.41)
corr(trial_failed, local_terrain_slope)                       = +0.099  (p = 0.027)
```

Landing force tracks drift speed. It does not track where the lander touched down. Controlling for drift does not uncover a slope effect. The partial correlation is flatter than the raw one.

Bootstrap resampling moves the recommended coordinate by 55.0 m (95% spread) inside a 97 m spawn window. The heatmap is a smooth picture of noise.

A 40-trial probe on a rougher preset (relief 8.77 m instead of 4.44 m) raises the failure rate from 12.5% to 37.5% and still has no force-slope correlation (`docs/roughness_comparison.json`). That file is a short probe. The 500-trial numbers above are the campaign result.

The convex-hull gap is measured in `docs/convex_gap.json`. At the deepest sampled crater the hull and the mesh contact sit 4.10 m above the floor. The heightfield contact sits 2.6 cm below it. The heightfield PNG has 256 levels, about 1.7 cm per step on this relief.

## Bugs that were in earlier runs

These showed up by checking the outputs against a hand calculation.

**Mesh collision used the convex hull.** The terrain was a `<geom type="mesh">`. MuJoCo collides meshes on the convex hull, so craters were in the render and not in the contact. The lander was landing on a dome. The model now uses `<hfield>`.

**Reported peak forces were solver output.** With the old soft legs, every trial hit the joint limit. Loads of 356-1,203 kN came from `solref="0.005 1"`. Changing that parameter changed the published numbers. Strut load is now read from the joint, and a trial that reaches the stroke limit fails the bottom-out check.

**The legs could not hold the lander at rest.** Stiffness was 320 N/m, sized from impact energy only. Static load is `1000 * 1.62 / 4 = 405 N` per leg, which needs 1.27 m of compression. Stroke is 0.8 m. Stiffness is now 3,000 N/m and damping is 1,200 N·s/m. The slide axis was `0 0 -1`, so increasing q extended the foot and the spring force was zero with the vehicle parked. The axis is now `0 0 1`: q = 0 is the extended leg, positive q compresses it, and the spring pushes the bus up. The spring sits above the strut and foot, so it carries the 840 kg bus, engine, and tank. Parked sag on flat ground is 0.113 m. Charging each spring with the full 405 N is 0.135 m, still inside the 0.8 m stop.

**Thrust stayed on after contact.** The controller kept applying `m*g` after the feet touched, so the "landed" flag fired while the engine was still holding most of the weight. Thrust now ramps to zero over 0.2 s once at least three feet see contact.

Fixing the thrust cutoff before the stiffness would have bottomed out every trial. Both had to change together.

## Quick start

```bash
git clone https://github.com/kkennethzhou/lunar-landing-analysis.git
cd lunar-landing-analysis
python -m venv .venv && source .venv/bin/activate
pip install -e ".[dev]"

# terrain, 500 trials, site map, validation
make all

python -m lunar_lander.generate_world
python -m lunar_lander.run_monte_carlo
python -m lunar_lander.plot_landing_zones
python -m lunar_lander.validate_results

python -m lunar_lander.convex_gap
python -m lunar_lander.compare_roughness --trials 40

# OPEN-1 follow-up (isolated output/open1/; does not overwrite published docs/)
python -m lunar_lander.open1_campaigns --trials 2   # smoke
python -m lunar_lander.open1_campaigns --trials 500 # full campaigns
```

Runs are deterministic from `seed = 42`. No GPU. A 500-trial run is about 10 minutes on an M-series Mac.

```bash
python -m lunar_lander.interactive_landing
```

### NASA images

The 38 LROC strips are about 1.9 GB and are not in the repo.

```bash
python scripts/download_lunar.py
python scripts/convert_nasa_images.py
```

The simulation does not read them. They document the reconstruction attempt.

## Validation

`validate_results.py` runs after the Monte Carlo. Numbers are from `summary_stats.json` for the current 500-trial run.

| Check | Question | Result |
|---|---|---|
| Signal | Does landing force correlate with slope? | fail, r=+0.060, p=0.23 |
| Partial correlation | Does controlling for drift reveal a slope effect? | fail, r=+0.041, p=0.41 |
| Failure vs terrain | Do failures track slope? | fail, r=+0.099, p=0.027, under the \|r\|=0.2 bar |
| Density confound | Does the site move if every force is replaced by a constant? | inconclusive, because the signal test failed |
| Bootstrap | Is the recommended site stable? | fail, 55.0 m spread |
| Bottom-out | Did any leg hit the stroke limit? | pass, max 0.797 m of 0.8 m |

The density test is inconclusive on purpose. If both maps are noise, the distance between them does not mean anything. It only counts once the signal test passes.

## Known issues

Full notes: [docs/known-issues.md](docs/known-issues.md).

Landing gear is 3,000 N/m, up from 320 N/m, and the slide axis points up so compression loads the spring. Thrust cuts at touchdown. Drift is still the main correlate of landing force (r=+0.539). Rougher terrain is the remaining thing to try. The 40-trial probe above made more trials fail and did not create a slope correlation. See OPEN-1.

## Layout

```
src/lunar_lander/
  generate_world.py       terrain, heightfield, MJCF
  run_monte_carlo.py      trials, descent control, failure criteria
  plot_landing_zones.py   kernel-regression map
  validate_results.py     the checks above
  site_selection.py       shared regression
  convex_gap.py           hull vs heightfield at crater floors
  compare_roughness.py    short rerun on a rougher preset
  open1_campaigns.py      OPEN-1 isolated 500-trial experiments (issue #3)
  terrain_metrics.py      slope/roughness at spawn, touchdown, and feet
  interactive_landing.py  viewer
  paths.py                output paths
  summary_stats.py        numbers the docs quote

scripts/   NASA download and PDS3 conversion
tests/     includes the static-equilibrium check
docs/      methodology, COLMAP notes, known issues
figures/   plots, regenerated by the pipeline
```

Reported numbers are written to JSON and read back. An earlier banner claimed 1,000 kg while the model was 960 kg, because the mass was a literal in the print string.

## Model, short version

Terrain is a 128 by 128 heightfield at 1 m spacing (127 m by 127 m). Four sinusoidal octaves, seven Gaussian craters with rims, four boulders. Relief 4.44 m. Cell slopes: mean 6.46°, p95 12.73°, max 27.77°, 63% of cells steeper than 5°. `relief/width` is about 2°. That is the tilt of the whole patch, not a slope.

The lander is 1,000 kg, four spring-damper legs, spherical feet. Mass and inertia come from the geoms. One thruster, PD velocity control plus feed-forward, 1.5 m/s touchdown target.

Site selection is Nadaraya-Watson regression: local mean force, `gaussian_filter(force_sum, sigma) / gaussian_filter(count, sigma)`, cells under 5 effective samples masked out. An older version accumulated scores with `+=`. Deleting the force values only moved that answer by 2.2 m, so it was mostly a map of where the trials landed.

Derivations are in [docs/methodology.md](docs/methodology.md).

## Contributing

Issues and pull requests are fine, especially on OPEN-1. Please do not change a threshold so a failing check turns green.

`pytest` and `ruff check .` before a pull request. See [CONTRIBUTING.md](CONTRIBUTING.md).

## Citation

```bibtex
@software{zhou_lunar_landing_2026,
  author = {Zhou, Kenneth},
  title  = {Lunar Landing Site Analysis: A MuJoCo Monte Carlo Pipeline},
  year   = {2026},
  url    = {https://github.com/kkennethzhou/lunar-landing-analysis}
}
```

## Data and license

Code is [MIT](LICENSE). LROC images are not redistributed. Citation notes are in [DATA_LICENSE.md](DATA_LICENSE.md).

NASA/GSFC/Arizona State University for LROC, via the [PDS Imaging Node](https://pds-imaging.jpl.nasa.gov/). Physics is [MuJoCo](https://mujoco.org/). The failed reconstruction used [COLMAP](https://colmap.github.io/).

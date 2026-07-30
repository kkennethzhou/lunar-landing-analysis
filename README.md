# Lunar Landing Site Analysis

**A MuJoCo Monte Carlo pipeline for lunar landing site selection — and an honest account of why its headline result doesn't hold up.**

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![Status: negative result (confirmed)](https://img.shields.io/badge/status-negative%20result%20(confirmed)-blue.svg)](#known-issues)
[![Python 3.11](https://img.shields.io/badge/python-3.11-blue.svg)](https://www.python.org/)

> **Status: investigation complete.** The pipeline runs end to end and is reproducible, and the two physics defects that could have explained a null result are fixed (see [Known Issues](#known-issues)). The site-selection result is still not statistically supported — the validation suite fails its signal, partial-correlation, and bootstrap checks even with both defects fixed. This is no longer "broken and unresolved"; it's a negative result that survived the two most plausible explanations for it.

---

## What this is

Given a patch of lunar terrain, where should a 1,000 kg lander touch down? This project builds a virtual moon, drops a spacecraft on it hundreds of times under randomized conditions, and tries to let the data pick the safest coordinates.

The pipeline:

1. **Acquire** — fetch NASA LROC Narrow Angle Camera strips from the PDS archive
2. **Reconstruct** — attempt photogrammetric 3D terrain via COLMAP *(fails — see [docs/colmap-failure.md](docs/colmap-failure.md))*
3. **Generate** — build procedural lunar terrain instead: layered sinusoids, Gaussian craters, boulders
4. **Simulate** — MuJoCo rigid-body lander with closed-loop descent control under lunar gravity
5. **Test** — Monte Carlo across randomized position, drift, tilt, and touchdown speed
6. **Validate** — test whether the resulting safety map means anything

Step 6 is the part most projects like this skip, and it's the reason this one knows it isn't finished.

## The interesting result

After 500 trials with a 15.6% failure rate, legs never bottoming out (max stroke 0.144 m of 0.8 m), and the engine genuinely cut at touchdown:

```
corr(peak_leg_force, local_terrain_slope)                          = +0.025  (p = 0.61)
corr(peak_leg_force, horizontal_drift)                             = +0.525  (p = 3e-31)
corr(peak_leg_force, local_terrain_slope | horizontal_drift)       = +0.005  (p = 0.92)   <- partial correlation
corr(trial_failed, local_terrain_slope)                            = +0.070  (p = 0.12)
```

**Landing force does not depend on where you land. It depends on how fast you were drifting** — and controlling for drift speed doesn't recover a hidden terrain effect either; the partial correlation is, if anything, flatter than the raw one.

The recommended coordinate moves by tens of metres under bootstrap resampling (95% spread: 58.4 m across a 97 m domain). A "safety map" built from this data is smoothed noise — it looks convincing and means nothing.

This is a negative result, and it now stands on firmer ground than the first version of this claim: the two physics defects that could have produced it (undersized landing gear, thrust never cutting at touchdown) are both fixed, and the most promising remaining hypothesis — that drift variance was masking a real terrain effect — was tested directly and didn't pan out. See [Known Issues](#known-issues) for what was fixed and what that leaves open.

## Bugs worth knowing about

All four were found by auditing outputs against first principles rather than by reading code — the kind that produce plausible-looking numbers indefinitely if nobody checks.

**Mesh collision was silently convex.** The terrain was loaded as `<geom type="mesh">`. MuJoCo evaluates mesh–mesh collision against the **convex hull**, so every crater and depression was invisible to the physics — the lander was touching down on a smooth dome. Fixed by switching to `<hfield>`, which supports exact non-convex geometry. Nothing errored; the simulation just quietly modelled different terrain than it rendered.

**Peak forces were solver artifacts.** With under-sized landing gear, the legs bottomed out on their joint limits every single trial. The reported 356–1,203 kN "leg loads" were the response of MuJoCo's contact solver parameter `solref="0.005 1"` — change that one number and every result changed. Fixed by logging strut load from the joint DOF and asserting no leg reaches its stroke limit.

**Landing gear couldn't support the vehicle at rest.** Leg stiffness was sized on landing impact energy alone (320 N/m), which required 1.27 m of static compression against a 0.8 m stroke — physically impossible. Fixed to 3,000 N/m, satisfying both a static-support constraint and a dynamic-stroke constraint at the 1.5 m/s design touchdown.

**Thrust never cut at touchdown.** The descent controller kept applying `m·g` feed-forward after the legs made contact, so the "landed" detector fired while the engine was still holding up most of the vehicle's weight — legs never carried their designed load, and every force reading understated the real one. Fixed by ramping thrust to zero over ~0.2 s once ≥3 feet register contact, mirroring how Apollo cut off on the contact-probe signal.

## Quick start

```bash
git clone https://github.com/kkennethzhou/lunar-landing-analysis.git
cd lunar-landing-analysis
python -m venv .venv && source .venv/bin/activate
pip install -e ".[dev]"

# Full pipeline — terrain, 500 trials, site selection, validation
make all

# Or step by step
python -m lunar_lander.generate_world      # terrain + MJCF
python -m lunar_lander.run_monte_carlo     # 500 trials -> results.csv
python -m lunar_lander.plot_landing_zones  # safety map
python -m lunar_lander.validate_results    # does any of it mean anything?
```

Everything runs offline and deterministically from `seed = 42`. No GPU required; a 500-trial run takes roughly 10 minutes on an M-series Mac.

Interactive viewer, if you have a display:

```bash
python -m lunar_lander.interactive_landing
```

### Reproducing the NASA imagery step

The 38 LROC strips (~1.9 GB) are **not** in this repo — they're regenerated from the public archive:

```bash
python scripts/download_lunar.py       # 40 requested, 38 retrieved
python scripts/convert_nasa_images.py  # PDS3 -> PNG
```

The simulation does not depend on these; the terrain is procedural. They're here to document the reconstruction attempt.

## Validation suite

`validate_results.py` runs a battery of checks after every Monte Carlo and refuses to endorse a result that fails them. Numbers below are read from `summary_stats.json` after the current 500-trial run:

| Check | What it asks | Current |
|---|---|---|
| **Signal test** | Does landing force correlate with terrain slope? | ❌ `r=+0.025, p=0.61` |
| **Partial correlation** | Does controlling for drift speed reveal a hidden terrain effect? | ❌ `r=+0.005, p=0.92` |
| **Failure vs. terrain** | Does failure rate correlate with terrain slope? | ❌ `r=+0.070, p=0.12` |
| **Density confound** | Does the answer change if you ignore forces entirely? | ⚠️ INCONCLUSIVE (signal test failed) |
| **Bootstrap** | Is the recommended site stable under resampling? | ❌ 58.4 m spread |
| **Bottom-out** | Did any leg exceed its stroke and invalidate force readings? | ✅ max 0.144 m of 0.8 m, engine genuinely off |

The bottom-out check is a clean pass now — the descent engine actually cuts at touchdown, so this measures the real physics, not an artifact of the engine still firing. The density-confound test is reported as inconclusive, not pass or fail: comparing two maps that are both noise proves nothing, so it's only meaningful once the signal test above it passes.

## Known issues

Tracked in [docs/known-issues.md](docs/known-issues.md).

**Fixed.** Landing gear now supports the vehicle at rest (stiffness raised from 320 to 3,000 N/m, satisfying both the static-support and dynamic-stroke constraints), and thrust genuinely cuts at touchdown (ramped to zero over ~0.2 s once ≥3 feet register contact, instead of continuing to apply `m·g` feed-forward after landing). Both were blocking: fixing thrust cutoff without fixing leg stiffness first would have made every trial bottom out.

**Still open.** Whether fixing both recovers a terrain signal has now been tested, not just hypothesized about, and the answer is no: the partial correlation of force against slope controlling for drift speed (the single most promising untested analysis in the previous round) comes back flatter than the raw correlation, and failure rate doesn't track terrain slope either. Drift speed remains the dominant correlate of landing force (`r=+0.525`). The negative result stands on firmer ground now that both plausible physics explanations have been eliminated — see [docs/known-issues.md](docs/known-issues.md) OPEN-1 for what's left to try (mainly: terrain roughness may simply be too benign for a 2.5 m leg span to notice, which would require redesigning the terrain generator, not the physics).

## Repository layout

```
src/lunar_lander/
├── generate_world.py       procedural terrain -> heightfield + MJCF
├── run_monte_carlo.py      trial loop, descent control, failure criteria
├── plot_landing_zones.py   kernel-regression safety map
├── validate_results.py     the checks above
├── site_selection.py       shared kernel-regression site-selection logic
├── interactive_landing.py  real-time viewer
├── paths.py                centralised output paths
└── summary_stats.py        single source of truth for reported numbers

scripts/        NASA download + PDS3 conversion
tests/          unit tests, including static-equilibrium check
docs/           methodology, COLMAP post-mortem, known issues
figures/        generated plots (regenerable)
```

Every number this project reports is written to `summary_stats.json` and read back from there. Hardcoded values in print statements are how the original version came to claim a 1,000 kg lander while simulating 960 kg.

## Methodology notes

**Terrain.** 128×128 vertex heightfield at 1 m spacing (127×127 m), from four sinusoidal octaves plus seven Gaussian craters with annular rims and four boulders. Relief 4.44 m. Real per-cell slope statistics: mean 6.46°, p95 12.73°, max 27.77°, with 63% of cells above 5°. Note that `relief/width` ≈ 2° is a whole-domain tilt, not a slope statistic — an easy and misleading substitution.

**Lander.** 1,000 kg, four independent spring-damper legs with spherical footpads, mass and inertia derived from geometry rather than hand-specified. Single thrust actuator with PD velocity control plus feed-forward, targeting 1.5 m/s at touchdown.

**Site selection.** Nadaraya-Watson kernel regression — the local *mean* force per cell, `gaussian_filter(force_sum, σ) / gaussian_filter(count, σ)`, masked below 5 effective samples. An earlier version accumulated scores with `+=`, which let trial density masquerade as safety: removing all force data moved the answer by only 2.2 m.

Full derivations in [docs/methodology.md](docs/methodology.md).

## Contributing

Issues and PRs welcome, particularly on the open questions above. If you find another silent-wrong-answer bug like the convex hull one, that's the most valuable contribution possible here.

Run `pytest` and `ruff check .` before submitting. See [CONTRIBUTING.md](CONTRIBUTING.md).

## Citation

```bibtex
@software{zhou_lunar_landing_2026,
  author = {Zhou, Kenneth},
  title  = {Lunar Landing Site Analysis: A MuJoCo Monte Carlo Pipeline},
  year   = {2026},
  url    = {https://github.com/kkennethzhou/lunar-landing-analysis}
}
```

## Data & license

Code is [MIT](LICENSE). NASA LROC imagery is public domain but carries citation expectations — see [DATA_LICENSE.md](DATA_LICENSE.md).

## Acknowledgements

NASA/GSFC/Arizona State University for LROC data via the [PDS Imaging Node](https://pds-imaging.jpl.nasa.gov/). Built with [MuJoCo](https://mujoco.org/) and [COLMAP](https://colmap.github.io/).

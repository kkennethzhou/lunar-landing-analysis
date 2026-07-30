# Lunar Landing Site Analysis

**A MuJoCo Monte Carlo pipeline for lunar landing site selection — and an honest account of why its headline result doesn't hold up yet.**

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![Status: WIP](https://img.shields.io/badge/status-work%20in%20progress-orange.svg)](#known-issues)
[![Python 3.11](https://img.shields.io/badge/python-3.11-blue.svg)](https://www.python.org/)

> **Status: work in progress.** The pipeline runs end to end and is reproducible. The *site-selection result it produces is not yet trustworthy* — the validation suite currently fails two of its four checks, and two known physics defects are unfixed. Details in [Known Issues](#known-issues). This is published in that state deliberately: the validation suite that catches the problem is the most useful part of the repo.

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

After 500 trials with a 16% failure rate:

```
corr(peak_leg_force, local_terrain_slope) = +0.022  (p = 0.65)
corr(peak_leg_force, horizontal_drift)    = +0.495
```

**Landing force does not depend on where you land. It depends on how fast you were drifting.**

The recommended coordinate moves by tens of metres under bootstrap resampling (95% spread: 58.7 m across a 97 m domain). A "safety map" built from this data is smoothed noise — it looks convincing and means nothing.

This is a negative result, and it survived several rounds of trying to make it go away. Two physics defects that could still explain it are documented below and not yet fixed.

## Two bugs worth knowing about

Both were found by auditing outputs against first principles rather than by reading code. Both are the kind that produce plausible-looking numbers indefinitely.

**Mesh collision was silently convex.** The terrain was loaded as `<geom type="mesh">`. MuJoCo evaluates mesh–mesh collision against the **convex hull**, so every crater and depression was invisible to the physics — the lander was touching down on a smooth dome. Fixed by switching to `<hfield>`, which supports exact non-convex geometry. Nothing errored; the simulation just quietly modelled different terrain than it rendered.

**Peak forces were solver artifacts.** With under-sized landing gear, the legs bottomed out on their joint limits every single trial. The reported 356–1,203 kN "leg loads" were the response of MuJoCo's contact solver parameter `solref="0.005 1"` — change that one number and every result changed. Fixed by logging strut load from the joint DOF and asserting no leg reaches its stroke limit.

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

`validate_results.py` runs four checks after every Monte Carlo and refuses to endorse a result that fails them:

| Check | What it asks | Current |
|---|---|---|
| **Signal test** | Does landing force correlate with terrain slope? | ❌ `r=+0.022, p=0.65` |
| **Density confound** | Does the answer change if you ignore forces entirely? | ⚠️ inconclusive |
| **Bootstrap** | Is the recommended site stable under resampling? | ❌ 58.7 m spread |
| **Bottom-out** | Did any leg exceed its stroke and invalidate force readings? | ⚠️ passes for the wrong reason |

The two ⚠️ rows are honest about *why* they aren't clean passes. The density-confound test compares two maps that are both noise, so the large distance between them proves nothing. The bottom-out check passes only because the descent engine is still firing when the sim declares touchdown.

## Known issues

Tracked in [docs/known-issues.md](docs/known-issues.md). The two blocking ones:

**1. Landing gear cannot support the vehicle at rest.** Leg stiffness is 320 N/m. Static load is 405 N per leg, requiring 1.27 m of compression against a 0.8 m stroke limit. The design point should satisfy both a static constraint (≤20% stroke at rest ⇒ k ≥ 2,500 N/m) and a dynamic one (≤75% stroke at 1.5 m/s touchdown); k ≈ 3,000 N/m with c ≈ 1,200 N·s/m satisfies both. Sizing on impact energy alone produced a spring that can't hold the lander up.

**2. Thrust is never cut at touchdown.** The descent controller applies `HOVER_THRUST = m·g` feed-forward on every step, so the engine holds most of the vehicle's weight while the settle detector fires. The sim reports "landed" with the engine running, which is why no leg ever bottoms out.

These interact: fixing (2) without (1) will correctly cause every trial to bottom out. Fix stiffness first.

Whether fixing both recovers a terrain signal is genuinely open. The most promising untested analysis is a **partial correlation of force against slope controlling for drift speed** — drift variance (r = +0.495) may be masking a real terrain effect.

## Repository layout

```
src/lunar_lander/
├── generate_world.py       procedural terrain -> heightfield + MJCF
├── run_monte_carlo.py      trial loop, descent control, failure criteria
├── plot_landing_zones.py   kernel-regression safety map
├── validate_results.py     the four checks above
├── interactive_landing.py  real-time viewer
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

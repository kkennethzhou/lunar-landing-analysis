# Push Checklist

This folder is a **staging area**, not a git repo. Your existing `LunarProject/.git` is 1.9 GB because the NASA PNGs are committed: GitHub warns above 1 GB and blocks at 2 GB, so that history should stay private.

## Before you push

- [ ] **Wire up `paths.py`.** Source files still use `os.path.dirname(__file__)`, which writes results into the installed package directory. Replace those constants with imports from `lunar_lander.paths`. See "Known gap" below.
- [x] ~~Replace `CHANGEME` placeholders~~: done, username `kkennethzhou` is filled in throughout
- [ ] Confirm you're comfortable publishing a work-in-progress with a failing validation suite: the README leads with it, which is the honest framing but it is a choice
- [ ] `pytest`: expect `test_static_equilibrium` failures until BLOCKING-1 is fixed. That's intended; the test documents the defect
- [ ] `ruff check .`

## Push

```bash
cd github-release
git init
git add -A
git commit -m "Initial release: MuJoCo lunar landing analysis with validation suite

Monte Carlo landing site selection pipeline. Published as work in progress:
the validation suite currently fails its signal and bootstrap checks, and two
physics defects are documented but unfixed. See docs/known-issues.md."

git branch -M main
git remote add origin https://github.com/kkennethzhou/lunar-landing-analysis.git
git push -u origin main
```

Verify the push is small: should be well under 1 MB. If git reports hundreds of MB, `.gitignore` isn't matching; check that `lunar_images/` and `*.pptx` are excluded.

## Known gap

`paths.py` exists but the modules don't import from it yet. Until they do, running from a pip-installed package will try to write into `site-packages`. Running from a clone works fine.

Mechanical fix: in each of `generate_world.py`, `run_monte_carlo.py`, `plot_landing_zones.py`, `validate_results.py`, `summary_stats.py`:

```python
# replace
XML_PATH = os.path.join(os.path.dirname(__file__), "lunar_world.xml")
# with
from lunar_lander.paths import XML_PATH
```

Worth doing before the first release tag.

## Repo settings after push

- Description: *MuJoCo Monte Carlo pipeline for lunar landing site selection, with a validation suite that checks whether the result means anything*
- Topics: `mujoco` `simulation` `monte-carlo` `lunar` `nasa` `robotics` `physics-simulation`
- Enable Issues; open one each for BLOCKING-1, BLOCKING-2, OPEN-1 and label them `good first issue` where they're well-specified
- Consider pinning an issue for OPEN-1 (the no-signal question): it's the most interesting open problem here

## Not included, deliberately

`lunar_images/` (1.9 GB, regenerable), `colmap_workspace/` (regenerable), `*.pptx` (132 MB), `training_frames/` and `raw_video/` (the indoor 3DGS test, unrelated to the lunar work), `3dgs_output/` (empty).

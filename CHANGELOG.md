# Changelog

## 0.4.0 (2026-11-27)

- Landing-leg slides use axis `0 0 1`. Positive q compresses the leg, so the spring holds the bus at rest.
- The 500-trial seed-42 campaign was rerun on that model. 19.0% did not land, maximum stroke was 0.797 m, and no leg bottomed out. The slope correlation is still absent. Numbers are in `docs/summary_stats.json`.
- CI installs `libgl1`, `libglib2.0-0`, and `libosmesa6` before generating terrain.
- The 10-trial fixture matches the compression-leg model.

## 0.3.0 (2026-11-18)

- Pipeline modules expose `main()` and no longer run when imported. `lunar-generate`, `lunar-simulate`, and `lunar-plot` call those functions.
- The validator samples the generator's heightmap instead of a second copy.
- `lunar-convex-gap` records the crater-floor error of a convex hull against the heightfield.
- `lunar-roughness` repeats a short seeded campaign on a rougher terrain preset.
- CI runs the engine-off settle test and the 10-trial fixture on Python 3.11. A failing smoke step fails the job.
- Docs match the 3,000 N/m gear and the thrust cutoff.

## 0.2.0 (2026-07-30)

- Leg stiffness raised to 3,000 N/m and thrust cut at touchdown.
- Partial correlation of landing force against slope, controlling for drift.
- Heightfield collision replaces the convex mesh collider.

# Changelog

## 0.3.0 — 2026-11-18

- Pipeline modules expose `main()` and no longer run when imported. `lunar-generate`, `lunar-simulate`, and `lunar-plot` match those functions.
- The validator samples the generator's heightmap instead of a second copy of it.
- `lunar-convex-gap` records the crater-floor error of a convex hull against the heightfield.
- `lunar-roughness` repeats a short seeded campaign on a harsher terrain preset.
- CI runs the engine-off settle test and the 10-trial fixture on Python 3.11. A failing smoke step fails the job.
- Documentation matches the 3,000 N/m gear and the thrust cutoff.

## 0.2.0 — 2026-07-30

- Leg stiffness raised to 3,000 N/m and thrust cut at touchdown.
- Partial correlation of landing force against slope, controlling for drift.
- Heightfield collision replaces the convex mesh collider.

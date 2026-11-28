# OPEN-1 experiment (issue #3)

Approved plan for whether peak leg load can track terrain once measurement
and two controlled campaigns are fixed. The `|r| = 0.2` success bar is
unchanged. Stiffness, damping, stroke limit, and controller gains are not
retuned to make the check green.

Adjacent feet are **4.0 m** apart in the MJCF (upper attachment ±1.1 m plus
strut tip ±0.9 m). Older notes that said ~2.5 m were wrong.

## Hard constraints

- Do not overwrite published `docs/summary_stats.json` or
  `figures/optimal_landing_zones.png`.
- All campaign outputs go under an isolated directory (`output/open1/` by
  default, or `$LUNAR_OUTPUT_DIR/open1`).
- A null result is valid. Leave issue #3 open unless a documented condition
  hits `|r| ≥ 0.2` on a full seeded campaign.

## Step 0 — Measurement

CSV `x,y` are spawn coordinates. The lander has no horizontal thruster; with
drift up to 2.3 m/s over a ~10 s descent it can be tens of metres away at
contact. Each trial now logs:

- `touchdown_x`, `touchdown_y` — body XY at first foot contact (>1 N)
- `foot1_x`…`foot4_y` — foot geom XY at that instant
- `drawn_vx`, `drawn_vy` — RNG draws (kept even when `--zero-drift` zeros the
  actual velocities)

The validator reports force/failure correlations against spawn slope
(historical), touchdown slope, mean slope under the feet, local roughness at
touchdown, and plane-residual roughness under the feet. It also bins trials
by drift quartile.

## Step 1 — Harsh campaign

500 trials, seed 42, `harsh` preset (crater×3, ridge×2), same compression-leg
lander. The validator loads the **harsh** heightmap for that run. If legs
bottom out, report it; do not raise the stroke limit or stiffness.

## Step 2 — Low-drift control

500 trials, seed 42, nominal terrain. RNG draw order matches the published
campaign; `vx` and `vy` are set to 0 **after** those draws so spawn points
still match. Nominal microdata (from the remeasure campaign) is binned by
drift quartile inside the validation report.

## Commands

```bash
# Smoke (seconds)
python -m lunar_lander.open1_campaigns --trials 2 --seed 42

# Full approved campaigns (~tens of minutes each on typical hardware)
python -m lunar_lander.open1_campaigns --trials 500 --seed 42

# One campaign only
python -m lunar_lander.open1_campaigns --trials 500 --campaigns harsh
python -m lunar_lander.open1_campaigns --trials 500 --campaigns low-drift
python -m lunar_lander.open1_campaigns --trials 500 --campaigns nominal-remeasure

# Explicit isolation
LUNAR_OUTPUT_DIR=/tmp/lunar_open1 python -m lunar_lander.open1_campaigns --trials 500
```

Outputs:

| Path | Contents |
|---|---|
| `output/open1/open1_report.json` | Aggregate verdict |
| `output/open1/<campaign>/results.csv` | Trial microdata + touchdown columns |
| `output/open1/<campaign>/validation_stats.json` | Full validation including OPEN-1 metrics |
| `output/open1/<campaign>/summary_stats.json` | Campaign-local summary (not `docs/`) |

## Success criteria

1. Touchdown / feet logging implemented and tested.
2. Validator can evaluate the harsh campaign against the harsh heightmap.
3. New outputs only under the isolated directory; published artifacts unchanged.
4. If a full 500-trial run completes, report correlations (spawn vs
   touchdown/feet), bottom-out counts, drift-quartile bins, and whether
   `|r| ≥ 0.2` was met under any documented condition. Otherwise smoke-test
   only and do not invent results.

Documented conditions for the `|r| ≥ 0.2` bar (with `p < 0.05`):

- force vs spawn slope (raw and drift-controlled)
- failed vs spawn slope
- force vs touchdown slope (raw and drift-controlled)
- force vs touchdown local roughness
- force vs mean feet slope
- force vs feet plane-residual roughness
- failed vs touchdown slope

## Results (500 trials, seed 42; this PR)

Committed digests: [`docs/open1/report.json`](open1/report.json). Live CSVs stay under `output/open1/` (gitignored). Published `docs/summary_stats.json` and `figures/optimal_landing_zones.png` SHA-256 unchanged.

| Campaign | Fail % | Max stroke | Mean spawn→TD offset | Force vs spawn slope | Force vs TD slope | Force vs feet slope | Failed vs spawn slope | Failed vs TD slope |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| nominal-remeasure | 19.0 | 0.797 m | 12.34 m | +0.060 | −0.043 | −0.027 | +0.099 | **+0.268** |
| harsh | 43.2 | **0.800 m** (1 bottom-out) | 12.49 m | +0.047 | −0.045 | −0.069 | +0.192 | **+0.406** |
| low-drift | 6.4 | 0.644 m | 1.16 m | +0.030 | +0.079 | +0.172 | **+0.545** | **+0.632** |

**Peak leg load** does not hit `|r| ≥ 0.2` against spawn, touchdown, or feet slope on any campaign. Issue #3 stays open for that load-tracking question.

**Measurement finding:** the published failure-vs-spawn-slope check (`r=+0.099`) was looking ~12 m away from contact. At touchdown on the same seed-42 landings, failure-vs-slope is `r=+0.268`. That is a documented HIT for the failure metric, not a reason to move the force bar.

**Harsh:** one trial bottomed out. Stroke limit and stiffness were not changed.

**Low-drift:** zeroing `vx,vy` after the seed-42 draws drops failure to 6.4% and makes failure track slope strongly; force vs feet mean slope rises only to `r=+0.172`.

Drift-quartile bins for the nominal remeasure are in `docs/open1/nominal_remeasure.json`.

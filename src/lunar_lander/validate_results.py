"""
Post-hoc validation for the Monte Carlo + site-selection pipeline.

Run this after every run_monte_carlo.py + plot_landing_zones.py pass. It
is the check whose absence caused the original problem: the deck reported
a confident "optimal landing site" without ever testing whether the
underlying force measurements carried any real terrain signal, or
whether the recommended site was stable/meaningful at all. This script
runs that test every time and prints a pass/fail report, refusing to
stay quiet about a null result.

Checks:
  1. Signal test    - corr(peak_strut_force, local_slope) and
                       corr(peak_strut_force, elevation), with p-values.
  2. Density-confound test - does the recommended site move if every
                       force is replaced by a constant?
  3. Bootstrap      - resample trials 1000x with replacement; how much
                       does the recommended coordinate move?
  4. Bottom-out assertion - hard fail if any trial's peak leg stroke
                       reached the 0.8 m joint limit.
  5. Failure breakdown by reason.
"""

import csv
import math
import sys
import json
import numpy as np
from scipy.ndimage import gaussian_filter
from scipy import stats

from lunar_lander.paths import RESULTS_CSV as CSV_PATH, VALIDATION_STATS as STATS_OUT_PATH

STROKE_LIMIT_M = 0.8
GRID_MARGIN = 15.0
DOMAIN_MIN, DOMAIN_MAX = GRID_MARGIN, 127.0 - GRID_MARGIN
GRID = 128

SIGNAL_R_THRESHOLD = 0.2
DENSITY_CONFOUND_THRESHOLD_M = 5.0
BOOTSTRAP_SPREAD_THRESHOLD_M = 15.0
MIN_EFFECTIVE_SAMPLES = 5.0
SIGMA_CELLS = 8
KERNEL_RES = 200


# ── Terrain heightmap (duplicated from generate_world.py's layered_noise) ───
# Duplicated rather than imported: generate_world.py is a top-level script
# that regenerates the OBJ/XML/PNG as a side effect of being imported.
# Keep this in sync with generate_world.py's layered_noise() + crater loop
# if those parameters ever change.
def _layered_noise(N: int) -> np.ndarray:
    x = np.linspace(0, 2 * math.pi, N, endpoint=False)
    y = np.linspace(0, 2 * math.pi, N, endpoint=False)
    xg, yg = np.meshgrid(x, y)
    h = np.zeros((N, N), dtype=np.float64)
    h += 1.2 * np.sin(1.3 * xg) * np.cos(0.9 * yg)
    h += 0.8 * np.cos(2.1 * xg + 0.4) * np.sin(1.7 * yg + 1.1)
    h += 0.4 * np.sin(4.3 * xg + 0.7) * np.cos(3.8 * yg + 2.3)
    h += 0.3 * np.cos(5.1 * xg + 1.2) * np.sin(4.6 * yg + 0.5)
    h += 0.10 * np.sin(11.0 * xg + 2.1) * np.cos(9.7 * yg + 1.3)
    h += 0.07 * np.cos(13.0 * xg + 0.3) * np.sin(12.0 * yg + 0.8)
    craters = [
        (1.1, 1.4, 0.35, 1.8), (3.8, 2.1, 0.25, 1.2), (5.2, 4.7, 0.40, 2.0),
        (2.5, 5.3, 0.20, 0.9), (4.1, 0.9, 0.30, 1.5), (0.6, 3.6, 0.18, 0.7),
        (6.0, 1.8, 0.28, 1.3),
    ]
    for cx, cy, r, d in craters:
        dist = np.sqrt((xg - cx) ** 2 + (yg - cy) ** 2)
        pit = d * np.exp(-((dist / r) ** 2))
        rim = 0.3 * d * np.exp(-(((dist - 1.2 * r) / (0.4 * r)) ** 2))
        h -= (pit - rim)
    boulders = [
        (2.0, 4.0, 0.10, 0.5), (4.8, 2.8, 0.08, 0.4),
        (1.5, 0.7, 0.07, 0.3), (5.5, 5.1, 0.09, 0.45),
    ]
    for bx, by, r, bump in boulders:
        dist = np.sqrt((xg - bx) ** 2 + (yg - by) ** 2)
        h += bump * np.exp(-((dist / r) ** 2))
    return h


def load_terrain():
    H = _layered_noise(GRID)
    H -= H.min()
    gy, gx = np.gradient(H, 1.0)
    slope_deg = np.degrees(np.arctan(np.hypot(gx, gy)))
    return H, slope_deg


def sample_grid(field, x, y):
    ix = int(round(x))
    iy = int(round(y))
    ix = min(max(ix, 0), GRID - 1)
    iy = min(max(iy, 0), GRID - 1)
    return field[iy, ix]


def load_rows():
    with open(CSV_PATH) as f:
        return list(csv.DictReader(f))


def recommended_site(xs, ys, forces, x_min=DOMAIN_MIN, x_max=DOMAIN_MAX,
                      y_min=DOMAIN_MIN, y_max=DOMAIN_MAX, res=KERNEL_RES,
                      sigma_cells=SIGMA_CELLS, min_eff=MIN_EFFECTIVE_SAMPLES):
    """
    Same Nadaraya-Watson kernel-regression site selection as
    plot_landing_zones.py, factored out here so it can be re-run cheaply
    (no matplotlib) for the density-confound test and the bootstrap.
    Returns (opt_x, opt_y) or (None, None) if no cell has enough samples.
    """
    xi = np.linspace(x_min, x_max, res)
    yi = np.linspace(y_min, y_max, res)
    force_sum = np.zeros((res, res))
    count = np.zeros((res, res))
    for x, y, f in zip(xs, ys, forces):
        ix = int((x - x_min) / (x_max - x_min) * (res - 1))
        iy = int((y - y_min) / (y_max - y_min) * (res - 1))
        if 0 <= ix < res and 0 <= iy < res:
            force_sum[iy, ix] += f
            count[iy, ix] += 1

    smoothed_sum = gaussian_filter(force_sum, sigma=sigma_cells)
    smoothed_count = gaussian_filter(count, sigma=sigma_cells)
    with np.errstate(invalid="ignore", divide="ignore"):
        force_field = smoothed_sum / smoothed_count

    kernel_norm = 2 * math.pi * sigma_cells ** 2
    effective_n = smoothed_count * kernel_norm
    valid = effective_n >= min_eff
    if not np.any(valid):
        return None, None

    masked = np.where(valid, force_field, np.inf)
    iy, ix = np.unravel_index(np.argmin(masked), masked.shape)
    return xi[ix], yi[iy]


def main():
    rows = load_rows()
    n_total = len(rows)

    print("=" * 70)
    print(f"  VALIDATION REPORT  (n={n_total} trials)")
    print("=" * 70)

    # ── 4. Bottom-out assertion (checked first: everything downstream is
    #    meaningless if the legs ran out of stroke) ───────────────────────
    strokes = [float(r["peak_stroke_m"]) for r in rows]
    n_bottomed = sum(1 for s in strokes if s >= STROKE_LIMIT_M)
    print(f"\n[4] Bottom-out check: max stroke = {max(strokes):.4f} m "
          f"(limit {STROKE_LIMIT_M} m)")
    if n_bottomed > 0:
        print(f"  FAIL: {n_bottomed}/{n_total} trials bottomed out — "
              f"forces on those trials are solver artifacts, not physical "
              f"loads. The whole pipeline is invalid until this is fixed.")
        bottom_out_ok = False
    else:
        print("  PASS: no leg bottomed out in any trial.")
        bottom_out_ok = True

    # ── 5. Failure breakdown ─────────────────────────────────────────────
    from collections import Counter
    outcome_counts = Counter(r["outcome"] for r in rows)
    reason_counts = Counter(r["fail_reason"] for r in rows if r["fail_reason"])
    failure_rate = 100 * sum(v for k, v in outcome_counts.items() if k != "landed") / n_total
    print(f"\n[5] Outcome breakdown: {dict(outcome_counts)}")
    print(f"    Failure reasons: {dict(reason_counts)}")
    print(f"    Overall failure rate: {failure_rate:.1f}%")

    # ── Safe-landing data for site selection / signal tests ──────────────
    safe_rows = [r for r in rows if r["outcome"] == "landed" and r["tipped"] == "0"]
    xs = np.array([float(r["x"]) for r in safe_rows])
    ys = np.array([float(r["y"]) for r in safe_rows])
    forces = np.array([float(r["peak_strut_force_N"]) for r in safe_rows])

    H, slope_deg = load_terrain()
    slopes_at = np.array([sample_grid(slope_deg, x, y) for x, y in zip(xs, ys)])
    elevs_at = np.array([sample_grid(H, x, y) for x, y in zip(xs, ys)])

    # ── 1. Signal test ────────────────────────────────────────────────────
    r_slope, p_slope = stats.pearsonr(forces, slopes_at)
    r_elev, p_elev = stats.pearsonr(forces, elevs_at)
    print(f"\n[1] Signal test:")
    print(f"    corr(peak_strut_force, local_slope)     r={r_slope:+.4f}  p={p_slope:.4g}")
    print(f"    corr(peak_strut_force, elevation)        r={r_elev:+.4f}  p={p_elev:.4g}")
    signal_ok = abs(r_slope) >= SIGNAL_R_THRESHOLD and p_slope < 0.05
    if not signal_ok:
        print(f"  WARNING: |r|={abs(r_slope):.3f} < {SIGNAL_R_THRESHOLD} threshold "
              f"(or p >= 0.05) — the safety map carries NO measurable terrain "
              f"signal. The recommended landing site is NOT physically "
              f"meaningful; it is, at best, describing where forces happened "
              f"to be lower in this particular random sample, not a real "
              f"terrain-dependent safety property.")
    else:
        print("  PASS: force carries a statistically significant relationship "
              "with local terrain slope.")

    # ── 2. Density-confound test ─────────────────────────────────────────
    real_x, real_y = recommended_site(xs, ys, forces)
    const_x, const_y = recommended_site(xs, ys, np.ones_like(forces))
    if real_x is None or const_x is None:
        print("\n[2] Density-confound test: SKIPPED (not enough effective "
              "samples anywhere in the domain to select a site at all).")
        confound_dist = None
        confound_ok = False
    else:
        confound_dist = math.hypot(real_x - const_x, real_y - const_y)
        print(f"\n[2] Density-confound test:")
        print(f"    Recommended site (real forces):     ({real_x:.2f}, {real_y:.2f})")
        print(f"    Recommended site (constant force):  ({const_x:.2f}, {const_y:.2f})")
        print(f"    Distance moved: {confound_dist:.2f} m")
        confound_ok = confound_dist >= DENSITY_CONFOUND_THRESHOLD_M
        if not confound_ok:
            print(f"  WARNING: moved less than {DENSITY_CONFOUND_THRESHOLD_M} m — "
                  f"the recommended site is still substantially DENSITY-driven, "
                  f"not force-driven. Where trials happened to cluster is doing "
                  f"most of the work, not where they landed gently.")
        else:
            print("  PASS: recommended site is not just reproducing trial density.")

    # ── 3. Bootstrap stability ───────────────────────────────────────────
    print(f"\n[3] Bootstrap (1000 resamples with replacement):")
    rng = np.random.default_rng(42)
    n = len(xs)
    boot_x, boot_y = [], []
    for _ in range(1000):
        idx = rng.integers(0, n, n)
        bx, by = recommended_site(xs[idx], ys[idx], forces[idx])
        if bx is not None:
            boot_x.append(bx)
            boot_y.append(by)
    boot_x = np.array(boot_x)
    boot_y = np.array(boot_y)
    if len(boot_x) < 50:
        print(f"    SKIPPED: only {len(boot_x)}/1000 resamples produced a valid "
              f"site (too few effective samples per resample).")
        bootstrap_spread = None
        bootstrap_ok = False
    else:
        centroid_x, centroid_y = boot_x.mean(), boot_y.mean()
        dists = np.hypot(boot_x - centroid_x, boot_y - centroid_y)
        bootstrap_spread = float(np.percentile(dists, 95))
        print(f"    Valid resamples: {len(boot_x)}/1000")
        print(f"    Bootstrap centroid: ({centroid_x:.2f}, {centroid_y:.2f})")
        print(f"    95th percentile radius from centroid: {bootstrap_spread:.2f} m")
        bootstrap_ok = bootstrap_spread <= BOOTSTRAP_SPREAD_THRESHOLD_M
        if not bootstrap_ok:
            print(f"  WARNING: 95% region spans more than "
                  f"{BOOTSTRAP_SPREAD_THRESHOLD_M} m — the recommended "
                  f"coordinate is not stable; a different random sample of "
                  f"trials would likely have recommended a different site.")
        else:
            print("  PASS: recommended site is reasonably stable under resampling.")

    # ── Overall verdict ───────────────────────────────────────────────────
    overall_ok = bottom_out_ok and signal_ok and confound_ok and bootstrap_ok
    print("\n" + "=" * 70)
    print(f"  OVERALL: {'PASS' if overall_ok else 'FAIL / NOT MEANINGFUL'}")
    print("=" * 70)

    out = dict(
        n_total=n_total,
        outcome_counts=dict(outcome_counts),
        fail_reason_counts=dict(reason_counts),
        failure_rate_pct=failure_rate,
        n_bottomed_out=n_bottomed,
        bottom_out_ok=bottom_out_ok,
        corr_force_slope_r=float(r_slope),
        corr_force_slope_p=float(p_slope),
        corr_force_elevation_r=float(r_elev),
        corr_force_elevation_p=float(p_elev),
        signal_ok=bool(signal_ok),
        recommended_site_x=float(real_x) if real_x is not None else None,
        recommended_site_y=float(real_y) if real_y is not None else None,
        density_confound_distance_m=confound_dist,
        density_confound_ok=confound_ok,
        bootstrap_spread_95pct_m=bootstrap_spread,
        bootstrap_ok=bootstrap_ok,
        overall_ok=bool(overall_ok),
    )
    with open(STATS_OUT_PATH, "w") as f:
        json.dump(out, f, indent=2)
    print(f"\nSaved: {STATS_OUT_PATH}")

    from lunar_lander.summary_stats import merge_summary_stats
    merge_summary_stats(dict(
        corr_force_slope_r=out["corr_force_slope_r"],
        corr_force_slope_p=out["corr_force_slope_p"],
        corr_force_elevation_r=out["corr_force_elevation_r"],
        corr_force_elevation_p=out["corr_force_elevation_p"],
        signal_ok=out["signal_ok"],
        density_confound_distance_m=out["density_confound_distance_m"],
        density_confound_ok=out["density_confound_ok"],
        bootstrap_spread_95pct_m=out["bootstrap_spread_95pct_m"],
        bootstrap_ok=out["bootstrap_ok"],
        n_bottomed_out=out["n_bottomed_out"],
        bottom_out_ok=out["bottom_out_ok"],
        validation_overall_ok=out["overall_ok"],
    ))
    print("Merged into summary_stats.json")

    return 0 if overall_ok else 1


if __name__ == "__main__":
    sys.exit(main())

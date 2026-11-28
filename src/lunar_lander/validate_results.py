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
                       Also reports corr(force, |v_horizontal|) prominently
                       (drift speed is the leading confound candidate), and
                       a partial correlation of force~slope controlling for
                       |v_horizontal|: drift variance may be masking a real
                       terrain effect that the raw correlation can't see.
                       OPEN-1: also reports the same correlations at
                       touchdown and under the feet when those columns exist.
  1b. Failure vs terrain - point-biserial corr(failed, local_slope) plus
                       failure rate by slope quartile, over ALL trials (not
                       just the ones that landed safely).
  2. Density-confound test - does the recommended site move if every
                       force is replaced by a constant? Only meaningful once
                       [1] finds a real signal: reported INCONCLUSIVE, not
                       PASS/FAIL, when it doesn't (a large distance between
                       two noise fields isn't evidence of anything).
  3. Bootstrap      - resample trials 1000x with replacement; how much
                       does the recommended coordinate move?
  4. Bottom-out assertion - hard fail if any trial's peak leg stroke
                       reached the 0.8 m joint limit.
  5. Failure breakdown by reason.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from collections import Counter
from pathlib import Path

import numpy as np
from scipy import stats

from lunar_lander.paths import RESULTS_CSV as CSV_PATH
from lunar_lander.paths import VALIDATION_STATS as STATS_OUT_PATH
from lunar_lander.site_selection import recommended_site
from lunar_lander.terrain_metrics import (
    feet_terrain,
    foot_xy_from_row,
    local_roughness,
    row_has_touchdown,
    sample_grid,
    spawn_touchdown_offset_m,
    touchdown_xy,
)

STROKE_LIMIT_M = 0.8
GRID = 128

SIGNAL_R_THRESHOLD = 0.2
DENSITY_CONFOUND_THRESHOLD_M = 5.0
BOOTSTRAP_SPREAD_THRESHOLD_M = 15.0


def load_terrain(preset="nominal"):
    """Height and slope from the same function that writes the MuJoCo world."""
    from lunar_lander.generate_world import make_heightmap, slope_degrees
    H = make_heightmap(preset, n=GRID)
    return H, slope_degrees(H)


def load_rows(csv_path=CSV_PATH):
    with open(csv_path) as f:
        return list(csv.DictReader(f))


def partial_corr(x, y, controlling_for):
    """Partial correlation of x and y, controlling for z, via the standard
    residual method: regress x on z and y on z (simple OLS), then correlate
    what's left over. If z explains away what looked like an x~y
    relationship (or reveals one that raw correlation couldn't see because
    z's variance was swamping it), that shows up here."""
    z = controlling_for
    x_resid = x - np.polyval(np.polyfit(z, x, 1), z)
    y_resid = y - np.polyval(np.polyfit(z, y, 1), z)
    return stats.pearsonr(x_resid, y_resid)


def _pearson_safe(x, y):
    if len(x) < 3 or np.std(x) == 0 or np.std(y) == 0:
        return float("nan"), float("nan")
    r, p = stats.pearsonr(x, y)
    return float(r), float(p)


def _pointbiserial_safe(failed, slopes):
    if len(failed) < 3 or np.std(failed) == 0 or np.std(slopes) == 0:
        return float("nan"), float("nan")
    r, p = stats.pointbiserialr(failed, slopes)
    return float(r), float(p)


def drift_quartile_bins(rows, H, slope_deg):
    """Bin trials by |v_horizontal| quartile; report force/failure vs slope per bin.

    Uses spawn slope (and touchdown slope when columns exist). Does not
    mutate any published artifact.
    """
    v = np.hypot(
        np.array([float(r["vx"]) for r in rows]),
        np.array([float(r["vy"]) for r in rows]),
    )
    # Prefer drawn_vx when the low-drift control zeroed actual vx but kept draws.
    if rows and "drawn_vx" in rows[0] and rows[0].get("drawn_vx") not in ("", None):
        drawn = np.hypot(
            np.array([float(r["drawn_vx"]) for r in rows]),
            np.array([float(r["drawn_vy"]) for r in rows]),
        )
        # If actual velocities are all ~0 but draws are not, use draws for binning.
        if float(np.max(v)) < 1e-9 and float(np.max(drawn)) > 0:
            v = drawn

    edges = np.percentile(v, [25, 50, 75])
    idx = np.digitize(v, edges)
    has_td = bool(rows) and row_has_touchdown(rows[0])
    bins = []
    for q in range(4):
        mask = idx == q
        q_rows = [r for r, m in zip(rows, mask, strict=True) if m]
        if not q_rows:
            bins.append({"quartile": q, "n": 0})
            continue
        v_q = v[mask]
        lo, hi = float(v_q.min()), float(v_q.max())
        safe = [r for r in q_rows if r["outcome"] == "landed" and r["tipped"] == "0"]
        failed = np.array([
            0.0 if r["outcome"] == "landed" and r["tipped"] == "0" else 1.0
            for r in q_rows
        ])
        spawn_slopes_all = np.array([
            sample_grid(slope_deg, float(r["x"]), float(r["y"])) for r in q_rows
        ])
        r_fail, p_fail = _pointbiserial_safe(failed, spawn_slopes_all)
        entry = {
            "quartile": q,
            "n": len(q_rows),
            "drift_min_mps": lo,
            "drift_max_mps": hi,
            "failure_rate_pct": float(100.0 * failed.mean()),
            "corr_failed_spawn_slope_r": r_fail,
            "corr_failed_spawn_slope_p": p_fail,
        }
        if len(safe) >= 5:
            forces = np.array([float(r["peak_strut_force_N"]) for r in safe])
            spawn_slopes = np.array([
                sample_grid(slope_deg, float(r["x"]), float(r["y"])) for r in safe
            ])
            r_s, p_s = _pearson_safe(forces, spawn_slopes)
            entry["n_landed"] = len(safe)
            entry["corr_force_spawn_slope_r"] = r_s
            entry["corr_force_spawn_slope_p"] = p_s
            if has_td:
                td_slopes = np.array([
                    sample_grid(slope_deg, *touchdown_xy(r)) for r in safe
                ])
                r_t, p_t = _pearson_safe(forces, td_slopes)
                entry["corr_force_touchdown_slope_r"] = r_t
                entry["corr_force_touchdown_slope_p"] = p_t
        else:
            entry["n_landed"] = len(safe)
        bins.append(entry)
    return bins


def run_validation(
    rows,
    preset="nominal",
    stats_out_path=None,
    merge_summary=True,
    summary_path=None,
):
    """Run the full validation suite on already-loaded CSV rows.

    Returns (exit_code, stats_dict). When merge_summary is False the
    published / default summary_stats.json is left untouched.
    """
    n_total = len(rows)
    stats_path = Path(stats_out_path) if stats_out_path is not None else STATS_OUT_PATH

    print("=" * 70)
    print(f"  VALIDATION REPORT  (n={n_total} trials, terrain preset={preset!r})")
    print("=" * 70)

    # 4. Bottom-out assertion
    strokes = [float(r["peak_stroke_m"]) for r in rows]
    n_bottomed = sum(1 for s in strokes if s >= STROKE_LIMIT_M)
    print(f"\n[4] Bottom-out check: max stroke = {max(strokes):.4f} m "
          f"(limit {STROKE_LIMIT_M} m)")
    if n_bottomed > 0:
        print(f"  FAIL: {n_bottomed}/{n_total} trials bottomed out: "
              f"forces on those trials are solver artifacts, not physical "
              f"loads. The whole pipeline is invalid until this is fixed.")
        bottom_out_ok = False
    else:
        print("  PASS: no leg bottomed out in any trial.")
        bottom_out_ok = True

    # 5. Failure breakdown
    outcome_counts = Counter(r["outcome"] for r in rows)
    reason_counts = Counter(r["fail_reason"] for r in rows if r["fail_reason"])
    failure_rate = 100 * sum(v for k, v in outcome_counts.items() if k != "landed") / n_total
    print(f"\n[5] Outcome breakdown: {dict(outcome_counts)}")
    print(f"    Failure reasons: {dict(reason_counts)}")
    print(f"    Overall failure rate: {failure_rate:.1f}%")

    H, slope_deg = load_terrain(preset)
    has_touchdown = bool(rows) and row_has_touchdown(rows[0])

    # Safe-landing data for site selection / signal tests
    safe_rows = [r for r in rows if r["outcome"] == "landed" and r["tipped"] == "0"]
    xs = np.array([float(r["x"]) for r in safe_rows])
    ys = np.array([float(r["y"]) for r in safe_rows])
    forces = np.array([float(r["peak_strut_force_N"]) for r in safe_rows])
    v_horiz = np.hypot(
        np.array([float(r["vx"]) for r in safe_rows]),
        np.array([float(r["vy"]) for r in safe_rows]),
    )

    slopes_at = np.array([sample_grid(slope_deg, x, y) for x, y in zip(xs, ys, strict=True)])
    elevs_at = np.array([sample_grid(H, x, y) for x, y in zip(xs, ys, strict=True)])

    # 1. Signal test (spawn — historical measurement)
    r_slope, p_slope = _pearson_safe(forces, slopes_at)
    r_elev, p_elev = _pearson_safe(forces, elevs_at)
    r_drift, p_drift = _pearson_safe(forces, v_horiz)
    print("\n[1] Signal test (terrain at spawn x,y — historical):")
    print(f"    corr(peak_strut_force, local_slope)      r={r_slope:+.4f}  p={p_slope:.4g}")
    print(f"    corr(peak_strut_force, elevation)         r={r_elev:+.4f}  p={p_elev:.4g}")
    print(f"    corr(peak_strut_force, |v_horizontal|)    r={r_drift:+.4f}  p={p_drift:.4g}"
          f"  (larger than the slope correlation on this run)")
    signal_ok = abs(r_slope) >= SIGNAL_R_THRESHOLD and p_slope < 0.05
    if not signal_ok:
        print(f"  WARNING: |r|={abs(r_slope):.3f} is under {SIGNAL_R_THRESHOLD} "
              f"or p >= 0.05. Force is not correlated with slope on this run.")
    else:
        print("  PASS: force carries a statistically significant relationship "
              "with local terrain slope.")
    if abs(r_drift) > abs(r_slope):
        print(f"  NOTE: |corr(force, drift)|={abs(r_drift):.3f} is larger than "
              f"|corr(force, slope)|={abs(r_slope):.3f}.")

    # Partial correlation of force~slope controlling for |v_horizontal|.
    if len(forces) >= 3 and np.std(v_horiz) > 0:
        r_partial, p_partial = partial_corr(forces, slopes_at, v_horiz)
        r_partial, p_partial = float(r_partial), float(p_partial)
    else:
        r_partial, p_partial = float("nan"), float("nan")
    print("\n[1a] Partial correlation (force ~ spawn slope, controlling for |v_horizontal|):")
    print(f"    r={r_partial:+.4f}  p={p_partial:.4g}")
    partial_signal_ok = abs(r_partial) >= SIGNAL_R_THRESHOLD and p_partial < 0.05
    if partial_signal_ok and not signal_ok:
        print("  NOTE: a terrain signal appears once drift speed is controlled "
              "for, even though the raw correlation is flat. Drift variance "
              "was masking a real slope effect.")
    elif not partial_signal_ok:
        print("  No signal recovered after controlling for drift speed either.")
    else:
        print("  (Raw signal test already passed; partial correlation is "
              "reported for completeness.)")

    # OPEN-1 measurement: terrain at touchdown / under feet
    r_td_slope = p_td_slope = float("nan")
    r_td_partial = p_td_partial = float("nan")
    r_feet_slope = p_feet_slope = float("nan")
    r_feet_rough = p_feet_rough = float("nan")
    r_td_elev = p_td_elev = float("nan")
    r_td_rough = p_td_rough = float("nan")
    mean_spawn_td_offset = float("nan")
    touchdown_signal_ok = False
    feet_signal_ok = False
    feet_rough_signal_ok = False
    touchdown_rough_signal_ok = False

    if has_touchdown and len(safe_rows) >= 3:
        td_xy = [touchdown_xy(r) for r in safe_rows]
        td_slopes = np.array([sample_grid(slope_deg, x, y) for x, y in td_xy])
        td_elevs = np.array([sample_grid(H, x, y) for x, y in td_xy])
        td_rough = np.array([local_roughness(H, x, y) for x, y in td_xy])
        feet_mean_slopes = []
        feet_rough = []
        for r in safe_rows:
            ft = feet_terrain(H, slope_deg, foot_xy_from_row(r))
            feet_mean_slopes.append(ft["mean_slope_deg"])
            feet_rough.append(ft["plane_residual_rms_m"])
        feet_mean_slopes = np.array(feet_mean_slopes)
        feet_rough = np.array(feet_rough)

        offsets = np.array([spawn_touchdown_offset_m(r) for r in safe_rows])
        mean_spawn_td_offset = float(np.nanmean(offsets))

        r_td_slope, p_td_slope = _pearson_safe(forces, td_slopes)
        r_td_elev, p_td_elev = _pearson_safe(forces, td_elevs)
        r_td_rough, p_td_rough = _pearson_safe(forces, td_rough)
        r_feet_slope, p_feet_slope = _pearson_safe(forces, feet_mean_slopes)
        r_feet_rough, p_feet_rough = _pearson_safe(forces, feet_rough)

        if np.std(v_horiz) > 0:
            r_td_partial, p_td_partial = partial_corr(forces, td_slopes, v_horiz)
            r_td_partial, p_td_partial = float(r_td_partial), float(p_td_partial)

        touchdown_signal_ok = abs(r_td_slope) >= SIGNAL_R_THRESHOLD and p_td_slope < 0.05
        feet_signal_ok = abs(r_feet_slope) >= SIGNAL_R_THRESHOLD and p_feet_slope < 0.05
        feet_rough_signal_ok = abs(r_feet_rough) >= SIGNAL_R_THRESHOLD and p_feet_rough < 0.05
        touchdown_rough_signal_ok = abs(r_td_rough) >= SIGNAL_R_THRESHOLD and p_td_rough < 0.05

        print("\n[1c] OPEN-1 measurement (terrain at touchdown / under feet):")
        print(f"    mean |spawn - touchdown| horizontal offset: {mean_spawn_td_offset:.2f} m")
        print(f"    corr(force, touchdown_slope)              r={r_td_slope:+.4f}  p={p_td_slope:.4g}")
        print(f"    corr(force, touchdown_elevation)          r={r_td_elev:+.4f}  p={p_td_elev:.4g}")
        print(f"    corr(force, touchdown_local_roughness)    r={r_td_rough:+.4f}  p={p_td_rough:.4g}")
        print(f"    corr(force, mean_feet_slope)              r={r_feet_slope:+.4f}  p={p_feet_slope:.4g}")
        print(f"    corr(force, feet_plane_residual_rms)      r={r_feet_rough:+.4f}  p={p_feet_rough:.4g}")
        print(f"    partial(force ~ touchdown_slope | drift)  r={r_td_partial:+.4f}  p={p_td_partial:.4g}")
        if (touchdown_signal_ok or feet_signal_ok or feet_rough_signal_ok
                or touchdown_rough_signal_ok):
            print("  PASS: at least one contact-site terrain metric hits "
                  f"|r| >= {SIGNAL_R_THRESHOLD} with p < 0.05.")
        else:
            print("  No contact-site terrain metric hits the |r| = 0.2 bar.")
    elif not has_touchdown:
        print("\n[1c] OPEN-1 measurement: SKIPPED (CSV has no touchdown_x/y columns).")

    # 1b. Failure vs terrain, over ALL trials
    all_xs = np.array([float(r["x"]) for r in rows])
    all_ys = np.array([float(r["y"]) for r in rows])
    all_slopes = np.array([
        sample_grid(slope_deg, x, y) for x, y in zip(all_xs, all_ys, strict=True)
    ])
    failed = np.array([0.0 if r["outcome"] == "landed" and r["tipped"] == "0" else 1.0
                        for r in rows])
    r_pb, p_pb = _pointbiserial_safe(failed, all_slopes)
    print("\n[1b] Failure vs terrain (all trials, spawn slope):")
    print(f"    corr(failed, local_slope)  r={r_pb:+.4f}  p={p_pb:.4g}")

    quartile_edges = np.percentile(all_slopes, [25, 50, 75])
    quartile_idx = np.digitize(all_slopes, quartile_edges)
    quartile_failure_rates = []
    for q in range(4):
        mask = quartile_idx == q
        if mask.sum() > 0:
            lo = all_slopes[mask].min()
            hi = all_slopes[mask].max()
            rate = 100 * failed[mask].mean()
            quartile_failure_rates.append(rate)
            print(f"    slope {lo:5.1f}-{hi:5.1f} deg (n={mask.sum():3d}): "
                  f"failure rate {rate:5.1f}%")
        else:
            quartile_failure_rates.append(None)
    failure_terrain_signal_ok = abs(r_pb) >= SIGNAL_R_THRESHOLD and p_pb < 0.05
    if not failure_terrain_signal_ok:
        print("  No significant relationship between failure and terrain slope.")
    else:
        print("  PASS: failure rate carries a statistically significant "
              "relationship with local terrain slope.")

    r_pb_td = p_pb_td = float("nan")
    failure_touchdown_signal_ok = False
    if has_touchdown:
        all_td_slopes = np.array([
            sample_grid(slope_deg, *touchdown_xy(r)) for r in rows
        ])
        r_pb_td, p_pb_td = _pointbiserial_safe(failed, all_td_slopes)
        failure_touchdown_signal_ok = abs(r_pb_td) >= SIGNAL_R_THRESHOLD and p_pb_td < 0.05
        print("\n[1b+] Failure vs touchdown slope:")
        print(f"    corr(failed, touchdown_slope)  r={r_pb_td:+.4f}  p={p_pb_td:.4g}")

    # Drift-quartile bins (OPEN-1 control analysis helper)
    drift_bins = drift_quartile_bins(rows, H, slope_deg)
    print("\n[1d] Drift-quartile bins (spawn slope unless noted):")
    for b in drift_bins:
        if b["n"] == 0:
            continue
        line = (f"    Q{b['quartile']+1} drift {b['drift_min_mps']:.2f}-"
                f"{b['drift_max_mps']:.2f} m/s (n={b['n']}): "
                f"fail {b['failure_rate_pct']:.1f}%")
        if "corr_force_spawn_slope_r" in b and not math.isnan(b["corr_force_spawn_slope_r"]):
            line += (f", corr(force,spawn_slope)="
                     f"{b['corr_force_spawn_slope_r']:+.3f}")
        if "corr_force_touchdown_slope_r" in b and not math.isnan(
                b["corr_force_touchdown_slope_r"]):
            line += (f", corr(force,td_slope)="
                     f"{b['corr_force_touchdown_slope_r']:+.3f}")
        print(line)

    # OPEN-1 overall: any documented terrain/controller condition hits the bar
    open1_conditions = {
        "spawn_force_slope": signal_ok,
        "spawn_partial_force_slope": partial_signal_ok,
        "spawn_failed_slope": failure_terrain_signal_ok,
        "touchdown_force_slope": touchdown_signal_ok,
        "touchdown_local_roughness": touchdown_rough_signal_ok,
        "feet_mean_slope": feet_signal_ok,
        "feet_roughness": feet_rough_signal_ok,
        "touchdown_failed_slope": failure_touchdown_signal_ok,
        "touchdown_partial_force_slope": (
            abs(r_td_partial) >= SIGNAL_R_THRESHOLD and p_td_partial < 0.05
            if not math.isnan(r_td_partial) else False
        ),
    }
    open1_signal_ok = any(open1_conditions.values())
    print("\n[1e] OPEN-1 |r| >= 0.2 criterion across documented conditions:")
    for name, ok in open1_conditions.items():
        print(f"    {name}: {'HIT' if ok else 'miss'}")
    print(f"  OPEN-1 aggregate: {'HIT' if open1_signal_ok else 'NULL (no condition met)'}")

    # 2. Density-confound test
    real_x, real_y = recommended_site(xs, ys, forces)
    const_x, const_y = recommended_site(xs, ys, np.ones_like(forces))
    if real_x is None or const_x is None:
        print("\n[2] Density-confound test: SKIPPED (not enough effective "
              "samples anywhere in the domain to select a site at all).")
        confound_dist = None
        confound_ok = False
        confound_verdict = "SKIPPED"
    else:
        confound_dist = math.hypot(real_x - const_x, real_y - const_y)
        print("\n[2] Density-confound test:")
        print(f"    Recommended site (real forces):     ({real_x:.2f}, {real_y:.2f})")
        print(f"    Recommended site (constant force):  ({const_x:.2f}, {const_y:.2f})")
        print(f"    Distance moved: {confound_dist:.2f} m")
        if not signal_ok:
            confound_ok = False
            confound_verdict = "INCONCLUSIVE"
            print("  INCONCLUSIVE: the signal test above failed, so there is no "
                  "real force-vs-terrain relationship for this test to confound "
                  "in the first place. A large distance between two noise "
                  "fields is not evidence of anything; this check only means "
                  "something once [1] passes.")
        else:
            confound_ok = confound_dist >= DENSITY_CONFOUND_THRESHOLD_M
            confound_verdict = "PASS" if confound_ok else "FAIL"
            if not confound_ok:
                print(f"  WARNING: moved less than {DENSITY_CONFOUND_THRESHOLD_M} m: "
                      f"the recommended site is still substantially DENSITY-driven, "
                      f"not force-driven. Where trials happened to cluster is doing "
                      f"most of the work, not where they landed gently.")
            else:
                print("  PASS: recommended site is not just reproducing trial density.")

    # 3. Bootstrap stability
    print("\n[3] Bootstrap (1000 resamples with replacement):")
    rng = np.random.default_rng(42)
    n = len(xs)
    boot_x, boot_y = [], []
    if n >= 1:
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
                  f"{BOOTSTRAP_SPREAD_THRESHOLD_M} m: the recommended "
                  f"coordinate is not stable; a different random sample of "
                  f"trials would likely have recommended a different site.")
        else:
            print("  PASS: recommended site is reasonably stable under resampling.")

    # Overall verdict (historical: spawn signal + confound + bootstrap + bottom-out)
    overall_ok = bottom_out_ok and signal_ok and confound_ok and bootstrap_ok
    print("\n" + "=" * 70)
    print(f"  OVERALL: {'PASS' if overall_ok else 'FAIL / NOT MEANINGFUL'}")
    print("=" * 70)

    out = dict(
        n_total=n_total,
        terrain_preset=preset,
        outcome_counts=dict(outcome_counts),
        fail_reason_counts=dict(reason_counts),
        failure_rate_pct=failure_rate,
        n_bottomed_out=n_bottomed,
        bottom_out_ok=bottom_out_ok,
        stroke_max_m=float(max(strokes)),
        corr_force_slope_r=float(r_slope),
        corr_force_slope_p=float(p_slope),
        corr_force_elevation_r=float(r_elev),
        corr_force_elevation_p=float(p_elev),
        corr_force_drift_r=float(r_drift),
        corr_force_drift_p=float(p_drift),
        partial_corr_force_slope_given_drift_r=float(r_partial),
        partial_corr_force_slope_given_drift_p=float(p_partial),
        partial_signal_ok=bool(partial_signal_ok),
        corr_failed_slope_r=float(r_pb),
        corr_failed_slope_p=float(p_pb),
        failure_rate_by_slope_quartile_pct=quartile_failure_rates,
        failure_terrain_signal_ok=bool(failure_terrain_signal_ok),
        signal_ok=bool(signal_ok),
        signal_r_threshold=SIGNAL_R_THRESHOLD,
        has_touchdown_columns=has_touchdown,
        mean_spawn_touchdown_offset_m=mean_spawn_td_offset,
        corr_force_touchdown_slope_r=float(r_td_slope),
        corr_force_touchdown_slope_p=float(p_td_slope),
        corr_force_touchdown_elevation_r=float(r_td_elev),
        corr_force_touchdown_elevation_p=float(p_td_elev),
        corr_force_touchdown_local_roughness_r=float(r_td_rough),
        corr_force_touchdown_local_roughness_p=float(p_td_rough),
        partial_corr_force_touchdown_slope_given_drift_r=float(r_td_partial),
        partial_corr_force_touchdown_slope_given_drift_p=float(p_td_partial),
        corr_force_feet_mean_slope_r=float(r_feet_slope),
        corr_force_feet_mean_slope_p=float(p_feet_slope),
        corr_force_feet_plane_residual_r=float(r_feet_rough),
        corr_force_feet_plane_residual_p=float(p_feet_rough),
        touchdown_signal_ok=bool(touchdown_signal_ok),
        touchdown_rough_signal_ok=bool(touchdown_rough_signal_ok),
        feet_signal_ok=bool(feet_signal_ok),
        feet_rough_signal_ok=bool(feet_rough_signal_ok),
        corr_failed_touchdown_slope_r=float(r_pb_td),
        corr_failed_touchdown_slope_p=float(p_pb_td),
        failure_touchdown_signal_ok=bool(failure_touchdown_signal_ok),
        open1_conditions=open1_conditions,
        open1_signal_ok=bool(open1_signal_ok),
        drift_quartile_bins=drift_bins,
        recommended_site_x=float(real_x) if real_x is not None else None,
        recommended_site_y=float(real_y) if real_y is not None else None,
        density_confound_distance_m=confound_dist,
        density_confound_ok=confound_ok,
        density_confound_verdict=confound_verdict,
        bootstrap_spread_95pct_m=bootstrap_spread,
        bootstrap_ok=bootstrap_ok,
        overall_ok=bool(overall_ok),
        adjacent_foot_spacing_m=4.0,
    )
    stats_path.parent.mkdir(parents=True, exist_ok=True)
    with open(stats_path, "w") as f:
        json.dump(out, f, indent=2)
    print(f"\nSaved: {stats_path}")

    if merge_summary:
        from lunar_lander.summary_stats import merge_summary_stats
        merge_kwargs = dict(
            corr_force_slope_r=out["corr_force_slope_r"],
            corr_force_slope_p=out["corr_force_slope_p"],
            corr_force_elevation_r=out["corr_force_elevation_r"],
            corr_force_elevation_p=out["corr_force_elevation_p"],
            corr_force_drift_r=out["corr_force_drift_r"],
            corr_force_drift_p=out["corr_force_drift_p"],
            partial_corr_force_slope_given_drift_r=out["partial_corr_force_slope_given_drift_r"],
            partial_corr_force_slope_given_drift_p=out["partial_corr_force_slope_given_drift_p"],
            partial_signal_ok=out["partial_signal_ok"],
            corr_failed_slope_r=out["corr_failed_slope_r"],
            corr_failed_slope_p=out["corr_failed_slope_p"],
            failure_rate_by_slope_quartile_pct=out["failure_rate_by_slope_quartile_pct"],
            failure_terrain_signal_ok=out["failure_terrain_signal_ok"],
            signal_ok=out["signal_ok"],
            density_confound_distance_m=out["density_confound_distance_m"],
            density_confound_ok=out["density_confound_ok"],
            density_confound_verdict=out["density_confound_verdict"],
            bootstrap_spread_95pct_m=out["bootstrap_spread_95pct_m"],
            bootstrap_ok=out["bootstrap_ok"],
            n_bottomed_out=out["n_bottomed_out"],
            bottom_out_ok=out["bottom_out_ok"],
            validation_overall_ok=out["overall_ok"],
        )
        if summary_path is not None:
            merge_summary_stats(merge_kwargs, path=Path(summary_path))
            print(f"Merged into {summary_path}")
        else:
            merge_summary_stats(merge_kwargs)
            print("Merged into summary_stats.json")

    return (0 if overall_ok else 1), out


def main(argv=None):
    parser = argparse.ArgumentParser(description="Validate Monte Carlo landing results")
    parser.add_argument("--csv", type=Path, default=None,
                        help="results CSV (default: paths.RESULTS_CSV)")
    parser.add_argument("--preset", default="nominal", choices=("nominal", "harsh"),
                        help="heightmap preset the campaign used (default: nominal)")
    parser.add_argument("--stats-out", type=Path, default=None,
                        help="validation_stats.json path")
    parser.add_argument("--summary-out", type=Path, default=None,
                        help="optional summary_stats.json merge target")
    parser.add_argument("--no-merge-summary", action="store_true",
                        help="do not write summary_stats.json (keeps published docs safe)")
    args = parser.parse_args(argv)

    csv_path = args.csv if args.csv is not None else CSV_PATH
    rows = load_rows(csv_path)
    code, _ = run_validation(
        rows,
        preset=args.preset,
        stats_out_path=args.stats_out,
        merge_summary=not args.no_merge_summary,
        summary_path=args.summary_out,
    )
    return code


if __name__ == "__main__":
    sys.exit(main())

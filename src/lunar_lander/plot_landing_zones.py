"""Read results.csv and write optimal_landing_zones.png.

The map is a Nadaraya-Watson local mean of peak strut force. An earlier
version added scores with +=, so a pile of ordinary landings could rank
above one good one. Dividing the smoothed force sum by the smoothed count
avoids that. Cells with fewer than 5 effective samples are left out. The
recommended site is the minimum of what remains.
"""

import csv
import math

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.colors as mcolors
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

from lunar_lander.paths import LANDING_ZONES_PNG as OUT_PATH
from lunar_lander.paths import RESULTS_CSV as CSV_PATH
from lunar_lander.site_selection import kernel_regression_field, recommended_site


def main():
    # Load data

    rows = []
    with open(CSV_PATH) as f:
        for r in csv.DictReader(f):
            rows.append({
                "x":       float(r["x"]),
                "y":       float(r["y"]),
                "force":   float(r["peak_strut_force_N"]),
                "tipped":  int(r["tipped"]),
                "outcome": r["outcome"],
            })

    safe    = [r for r in rows if r["outcome"] == "landed" and not r["tipped"]]
    unsafe  = [r for r in rows if r["outcome"] != "landed" or r["tipped"]]

    sx = np.array([r["x"] for r in safe])
    sy = np.array([r["y"] for r in safe])
    sf = np.array([r["force"] for r in safe])

    ux = np.array([r["x"] for r in unsafe]) if unsafe else np.array([])
    uy = np.array([r["y"] for r in unsafe]) if unsafe else np.array([])

    # Kernel-regression safety field (safe points only)
    # GRID_MARGIN=15 in run_monte_carlo.py confines spawns to [15, 112] on a
    # 0-127 m terrain -> the analysis domain is 97x97 m, not the full
    # 127x127 m terrain. Label this correctly rather than implying full
    # coverage. Site-selection procedure (kernel_regression_field /
    # recommended_site) lives in lunar_lander.site_selection, shared with
    # validate_results.py so both scripts agree on what "recommended site"
    # and "density confound" mean (docs/known-issues.md MINOR-1: they used
    # to disagree, 98.58 m vs 11.72 m, because they compared against
    # different things: a constant-force argmin here now, not a pure-density
    # argmax, matching validate_results.py's definition).
    GRID_RES = 200
    x_min, x_max = 15.0, 112.0   # terrain spawn bounds (GRID_MARGIN to 127-GRID_MARGIN)
    y_min, y_max = 15.0, 112.0
    DOMAIN_SIDE_M = x_max - x_min   # 97 m
    CELL_SIZE_M = (x_max - x_min) / (GRID_RES - 1)
    SIGMA_CELLS = 8
    SIGMA_M = SIGMA_CELLS * CELL_SIZE_M
    MIN_EFFECTIVE_SAMPLES = 5.0

    field_kwargs = dict(x_min=x_min, x_max=x_max, y_min=y_min, y_max=y_max,
                         res=GRID_RES, sigma_cells=SIGMA_CELLS,
                         min_eff=MIN_EFFECTIVE_SAMPLES)

    xi, yi, force_field, valid_mask = kernel_regression_field(sx, sy, sf, **field_kwargs)

    if not np.any(valid_mask):
        raise RuntimeError("No grid cell reached the minimum effective sample "
                            "count; site selection is not meaningful with this "
                            "few trials.")

    opt_x, opt_y = recommended_site(sx, sy, sf, **field_kwargs)
    opt_ix = int(round((opt_x - x_min) / (x_max - x_min) * (GRID_RES - 1)))
    opt_iy = int(round((opt_y - y_min) / (y_max - y_min) * (GRID_RES - 1)))
    opt_local_force = force_field[opt_iy, opt_ix]

    # For the display heatmap, show the (masked) local mean force field.
    display_field = np.where(valid_mask, force_field, np.nan)

    # Cluster radius: circle enclosing points within 12 m of optimal site
    cluster_pts = [(x, y, f) for x, y, f in zip(sx, sy, sf)
                   if math.hypot(x - opt_x, y - opt_y) < 12]
    cluster_radius = 12.0
    mean_cluster_force = (sum(f for _, _, f in cluster_pts) / len(cluster_pts)
                          if cluster_pts else 0.0)

    # Density-confound check (same constant-force-argmin definition as
    #    validate_results.py; Phase 4 validation is the authoritative check,
    #    this is kept here too so the printed summary is self-contained)
    const_x, const_y = recommended_site(sx, sy, np.ones_like(sf), **field_kwargs)
    density_confound_dist_m = (math.hypot(opt_x - const_x, opt_y - const_y)
                                if const_x is not None else None)

    # Plot

    fig, ax = plt.subplots(figsize=(11, 10))
    fig.patch.set_facecolor("#0d0d1a")
    ax.set_facecolor("#0d0d1a")

    # Background heatmap: local mean force (lower = safer), reversed colormap
    # so low force (safe) reads as cool/green-ish and high force (risky) reads hot.
    extent = [x_min, x_max, y_min, y_max]
    ax.imshow(
        display_field,
        origin="lower", extent=extent,
        cmap="YlOrRd", alpha=0.45,
        aspect="auto", interpolation="bilinear",
    )

    # Safe landings: colour by impact force
    sc = ax.scatter(
        sx, sy,
        c=sf / 1e3,           # kN
        cmap="Greens_r",
        norm=mcolors.Normalize(vmin=sf.min() / 1e3, vmax=sf.max() / 1e3),
        s=55, alpha=0.82, edgecolors="#00ff88", linewidths=0.4,
        zorder=3, label=f"Safe landing ({len(safe)})",
    )

    # Unsafe landings
    if len(ux):
        ax.scatter(
            ux, uy,
            c="#ff3333", s=80, marker="X", alpha=0.9,
            edgecolors="#ff0000", linewidths=0.6,
            zorder=4, label=f"Tip / Crash / Failed ({len(unsafe)})",
        )

    # Cluster ring
    ring = Circle((opt_x, opt_y), cluster_radius,
                  fill=False, edgecolor="#00e5ff",
                  linewidth=1.8, linestyle="--", zorder=5)
    ax.add_patch(ring)

    # Optimal site star
    ax.scatter(opt_x, opt_y, marker="*", s=600,
               color="#ffe066", edgecolors="#ff9900",
               linewidths=1.0, zorder=6)

    # Label
    ax.annotate(
        f"* Recommended Landing Site (min local mean force)\n"
        f"   ({opt_x:.1f} m, {opt_y:.1f} m)\n"
        f"   local mean {opt_local_force/1e3:.1f} kN · {len(cluster_pts)} pts in {cluster_radius:.0f} m",
        xy=(opt_x, opt_y),
        xytext=(opt_x + 14, opt_y + 8),
        fontsize=9.5, color="#ffe066",
        fontweight="bold",
        arrowprops=dict(arrowstyle="->", color="#ffe066", lw=1.4),
        bbox=dict(boxstyle="round,pad=0.4", fc="#1a1a2e", ec="#ffe066", lw=1.2),
        zorder=7,
    )

    # Colourbar for safe scatter
    cbar = fig.colorbar(sc, ax=ax, pad=0.02, fraction=0.035)
    cbar.set_label("Peak strut force (kN)", color="white", fontsize=9)
    cbar.ax.yaxis.set_tick_params(color="white")
    plt.setp(cbar.ax.yaxis.get_ticklabels(), color="white")
    cbar.outline.set_edgecolor("#555577")

    # Stats box
    n_total = len(rows)
    success_pct = 100 * len(safe) / n_total
    stats_text = (
        f"Monte Carlo  n={n_total}\n"
        f"Safe landings : {len(safe)} ({success_pct:.0f}%)\n"
        f"Tip / Crash / Failed : {len(unsafe)} ({100-success_pct:.0f}%)\n"
        f"Force range   : {sf.min()/1e3:.1f}-{sf.max()/1e3:.1f} kN\n"
        f"sigma = {SIGMA_CELLS} cells = {SIGMA_M:.2f} m\n"
        f"Domain: {DOMAIN_SIDE_M:.0f}x{DOMAIN_SIDE_M:.0f} m ({x_min:.0f}-{x_max:.0f} m), "
        f"not full 127x127 m\n"
        f"Density-confound check: recommended site\n"
        f"  moves {density_confound_dist_m:.1f} m if forces are ignored\n"
        f"Moon gravity  : -1.62 m/s^2"
    )
    ax.text(0.015, 0.985, stats_text,
            transform=ax.transAxes, fontsize=8.5, color="#ccccdd",
            verticalalignment="top",
            bbox=dict(boxstyle="round,pad=0.5", fc="#0d0d2e", ec="#334466", lw=1),
            zorder=8)

    # Axes styling
    ax.set_xlim(x_min - 2, x_max + 2)
    ax.set_ylim(y_min - 2, y_max + 2)
    ax.set_xlabel("Terrain X (m)", color="#aaaacc", fontsize=11)
    ax.set_ylabel("Terrain Y (m)", color="#aaaacc", fontsize=11)
    ax.set_title("Lunar Lander - Landing Site Safety Map\n"
                 "Kernel-regression local mean strut force (min = safest) · "
                 f"sigma={SIGMA_M:.2f} m · domain {DOMAIN_SIDE_M:.0f}x{DOMAIN_SIDE_M:.0f} m",
                 color="white", fontsize=13, fontweight="bold", pad=14)
    ax.tick_params(colors="#888899")
    for spine in ax.spines.values():
        spine.set_edgecolor("#334466")

    ax.legend(loc="lower right", fontsize=9,
              facecolor="#1a1a2e", edgecolor="#334466",
              labelcolor="white", framealpha=0.9)

    ax.grid(color="#222244", linewidth=0.4, linestyle=":")

    plt.tight_layout()
    plt.savefig(OUT_PATH, dpi=160, bbox_inches="tight",
                facecolor=fig.get_facecolor())
    print(f"Saved: {OUT_PATH}")
    print(f"Recommended site: ({opt_x:.2f}, {opt_y:.2f}) m, "
          f"local mean force {opt_local_force/1e3:.2f} kN  "
          f"({len(cluster_pts)} pts in {cluster_radius:.0f} m radius, "
          f"raw cluster avg {mean_cluster_force/1e3:.2f} kN)")
    print(f"sigma = {SIGMA_CELLS} cells = {SIGMA_M:.3f} m")
    print(f"Domain: [{x_min}, {x_max}] x [{y_min}, {y_max}] = "
          f"{DOMAIN_SIDE_M:.0f}x{DOMAIN_SIDE_M:.0f} m (not the full 127x127 m terrain)")
    print(f"Density-confound check: recommended site moves "
          f"{density_confound_dist_m:.2f} m if every force is replaced by a "
          f"constant (constant-force site at ({const_x:.1f}, {const_y:.1f}))")

    # Export for summary_stats.json / verify_deck.py
    SITE_SELECTION_STATS = dict(
        recommended_x_m=float(opt_x),
        recommended_y_m=float(opt_y),
        recommended_local_mean_force_N=float(opt_local_force),
        cluster_radius_m=cluster_radius,
        cluster_n_points=len(cluster_pts),
        cluster_mean_force_N=float(mean_cluster_force),
        sigma_cells=SIGMA_CELLS,
        sigma_m=float(SIGMA_M),
        domain_min_m=x_min,
        domain_max_m=x_max,
        domain_side_m=DOMAIN_SIDE_M,
        min_effective_samples=MIN_EFFECTIVE_SAMPLES,
        density_confound_distance_m=(float(density_confound_dist_m)
                                      if density_confound_dist_m is not None else None),
        n_safe_landings=len(safe),
        n_unsafe=len(unsafe),
        force_min_N=float(sf.min()),
        force_max_N=float(sf.max()),
        force_mean_N=float(sf.mean()),
    )
    import json

    from lunar_lander.paths import SITE_SELECTION_STATS as SITE_SELECTION_STATS_PATH
    with open(SITE_SELECTION_STATS_PATH, "w") as f:
        json.dump(SITE_SELECTION_STATS, f, indent=2)
    print(f"Saved site-selection stats: {SITE_SELECTION_STATS_PATH}")

    from lunar_lander.summary_stats import merge_summary_stats
    merge_summary_stats(SITE_SELECTION_STATS)
    print("Merged into summary_stats.json")

if __name__ == "__main__":
    main()

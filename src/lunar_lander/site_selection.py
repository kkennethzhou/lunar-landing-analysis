"""
Shared site-selection procedure: Nadaraya-Watson kernel regression (a local
weighted MEAN of some per-trial value over x/y), masked by an effective-
sample-count threshold, with the recommended site being the argmin (safest
= lowest force).

validate_results.py and plot_landing_zones.py each defined their own copy
of this (docs/known-issues.md MINOR-1), and disagreed about what the
density-confound test should even compare against: one used a pure-density
argmax, the other a constant-force argmin. Constant-force argmin is the
more principled comparison: it holds the site-selection procedure fixed
and only changes the input (real per-trial force vs. an identical value
for every trial), rather than swapping in a different procedure (raw
density) entirely. Both scripts now import from here.
"""

import math

import numpy as np
from scipy.ndimage import gaussian_filter

GRID_MARGIN = 15.0
DOMAIN_MIN, DOMAIN_MAX = GRID_MARGIN, 127.0 - GRID_MARGIN
KERNEL_RES = 200
SIGMA_CELLS = 8
MIN_EFFECTIVE_SAMPLES = 5.0


def kernel_regression_field(xs, ys, values, x_min=DOMAIN_MIN, x_max=DOMAIN_MAX,
                             y_min=DOMAIN_MIN, y_max=DOMAIN_MAX, res=KERNEL_RES,
                             sigma_cells=SIGMA_CELLS, min_eff=MIN_EFFECTIVE_SAMPLES):
    """Nadaraya-Watson local weighted mean of `values` over (xs, ys).

    Returns (xi, yi, field, valid_mask): xi/yi are the grid axes, field is
    the local weighted mean at each cell, and valid_mask marks cells with
    enough effective samples (scipy.ndimage.gaussian_filter normalises its
    kernel to sum to 1 in pixel space, so the smoothed count is a KDE-style
    density, not a raw count: undo that normalisation before thresholding).
    """
    xi = np.linspace(x_min, x_max, res)
    yi = np.linspace(y_min, y_max, res)
    value_sum = np.zeros((res, res))
    count = np.zeros((res, res))
    for x, y, v in zip(xs, ys, values):
        ix = int((x - x_min) / (x_max - x_min) * (res - 1))
        iy = int((y - y_min) / (y_max - y_min) * (res - 1))
        if 0 <= ix < res and 0 <= iy < res:
            value_sum[iy, ix] += v
            count[iy, ix] += 1

    smoothed_sum = gaussian_filter(value_sum, sigma=sigma_cells)
    smoothed_count = gaussian_filter(count, sigma=sigma_cells)
    with np.errstate(invalid="ignore", divide="ignore"):
        field = smoothed_sum / smoothed_count

    kernel_norm = 2 * math.pi * sigma_cells ** 2
    effective_n = smoothed_count * kernel_norm
    valid_mask = effective_n >= min_eff
    return xi, yi, field, valid_mask


def recommended_site(xs, ys, values, **kwargs):
    """Argmin of the kernel-regression field (safest = lowest value).

    Returns (opt_x, opt_y), or (None, None) if no cell anywhere in the
    domain reached the minimum effective sample count.
    """
    xi, yi, field, valid_mask = kernel_regression_field(xs, ys, values, **kwargs)
    if not np.any(valid_mask):
        return None, None
    masked = np.where(valid_mask, field, np.inf)
    iy, ix = np.unravel_index(np.argmin(masked), masked.shape)
    return xi[ix], yi[iy]

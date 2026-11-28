"""Sample slope and roughness at spawn, touchdown, and under the feet.

OPEN-1: the validator used to read slope at CSV x,y (spawn). With no
horizontal thruster the lander can drift tens of metres before contact, so
spawn terrain is the wrong place to look for a force-slope signal.
"""

from __future__ import annotations

import numpy as np

GRID = 128


def sample_grid(field: np.ndarray, x: float, y: float) -> float:
    ix = int(round(x))
    iy = int(round(y))
    ix = min(max(ix, 0), GRID - 1)
    iy = min(max(iy, 0), GRID - 1)
    return float(field[iy, ix])


def local_roughness(H: np.ndarray, x: float, y: float, half_window: int = 2) -> float:
    """Std of elevation in a (2*half_window+1)^2 neighbourhood around (x, y)."""
    ix = int(round(x))
    iy = int(round(y))
    ix0 = max(ix - half_window, 0)
    ix1 = min(ix + half_window + 1, GRID)
    iy0 = max(iy - half_window, 0)
    iy1 = min(iy + half_window + 1, GRID)
    patch = H[iy0:iy1, ix0:ix1]
    if patch.size < 2:
        return 0.0
    return float(np.std(patch))


def plane_residual_rms(xs: np.ndarray, ys: np.ndarray, zs: np.ndarray) -> float:
    """RMS residual of z from the least-squares plane fit through the points."""
    if len(xs) < 3:
        return 0.0
    A = np.column_stack([xs, ys, np.ones(len(xs))])
    try:
        coef, *_ = np.linalg.lstsq(A, zs, rcond=None)
    except np.linalg.LinAlgError:
        return float(np.std(zs))
    pred = A @ coef
    resid = zs - pred
    return float(np.sqrt(np.mean(resid ** 2)))


def feet_terrain(H: np.ndarray, slope_deg: np.ndarray, foot_xy: list[tuple[float, float]]) -> dict:
    """Slope and roughness metrics under the four foot positions."""
    elevs = np.array([sample_grid(H, x, y) for x, y in foot_xy])
    slopes = np.array([sample_grid(slope_deg, x, y) for x, y in foot_xy])
    xs = np.array([p[0] for p in foot_xy])
    ys = np.array([p[1] for p in foot_xy])
    return {
        "mean_slope_deg": float(slopes.mean()),
        "max_slope_deg": float(slopes.max()),
        "elev_std_m": float(np.std(elevs)),
        "plane_residual_rms_m": plane_residual_rms(xs, ys, elevs),
        "elevs_m": elevs,
        "slopes_deg": slopes,
    }


def row_has_touchdown(row: dict) -> bool:
    return "touchdown_x" in row and row["touchdown_x"] not in ("", None)


def touchdown_xy(row: dict) -> tuple[float, float]:
    return float(row["touchdown_x"]), float(row["touchdown_y"])


def foot_xy_from_row(row: dict) -> list[tuple[float, float]]:
    return [
        (float(row["foot1_x"]), float(row["foot1_y"])),
        (float(row["foot2_x"]), float(row["foot2_y"])),
        (float(row["foot3_x"]), float(row["foot3_y"])),
        (float(row["foot4_x"]), float(row["foot4_y"])),
    ]


def spawn_touchdown_offset_m(row: dict) -> float:
    """Horizontal distance from spawn (x,y) to touchdown body position."""
    if not row_has_touchdown(row):
        return float("nan")
    return float(np.hypot(float(row["touchdown_x"]) - float(row["x"]),
                          float(row["touchdown_y"]) - float(row["y"])))

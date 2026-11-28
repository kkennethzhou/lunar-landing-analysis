"""Output paths.

Modules used to build paths with `os.path.dirname(__file__)`, which wrote
results.csv and the stats JSON into the source tree. After a pip install
that directory is site-packages, which is the wrong place for results.

Everything is written under a project-root `output/` directory instead,
overridable with the LUNAR_OUTPUT_DIR environment variable.
"""

from __future__ import annotations

import os
from pathlib import Path

PACKAGE_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = PACKAGE_DIR.parents[1]


def _output_root() -> Path:
    env = os.environ.get("LUNAR_OUTPUT_DIR")
    root = Path(env).expanduser().resolve() if env else PROJECT_ROOT / "output"
    root.mkdir(parents=True, exist_ok=True)
    return root


OUTPUT_DIR = _output_root()
MESH_DIR = OUTPUT_DIR / "physics_mesh"
# Default published pipeline writes under project-root figures/. When
# LUNAR_OUTPUT_DIR is set, keep figures inside that tree so experimental
# campaigns cannot overwrite figures/optimal_landing_zones.png.
FIGURES_DIR = (
    OUTPUT_DIR / "figures"
    if os.environ.get("LUNAR_OUTPUT_DIR")
    else PROJECT_ROOT / "figures"
)

for _d in (MESH_DIR, FIGURES_DIR):
    _d.mkdir(parents=True, exist_ok=True)

# Model
XML_PATH = OUTPUT_DIR / "lunar_world.xml"
OBJ_PATH = MESH_DIR / "lunar_surface.obj"
HEIGHTFIELD_PATH = MESH_DIR / "lunar_heightfield.png"

# Results
RESULTS_CSV = OUTPUT_DIR / "results.csv"

# Stats: every reported number lives in one of these
SUMMARY_STATS = OUTPUT_DIR / "summary_stats.json"
TERRAIN_STATS = OUTPUT_DIR / "terrain_stats.json"
VALIDATION_STATS = OUTPUT_DIR / "validation_stats.json"
SITE_SELECTION_STATS = OUTPUT_DIR / "site_selection_stats.json"

# Figures
LANDING_ZONES_PNG = FIGURES_DIR / "optimal_landing_zones.png"

__all__ = [
    "PROJECT_ROOT", "OUTPUT_DIR", "MESH_DIR", "FIGURES_DIR",
    "XML_PATH", "OBJ_PATH", "HEIGHTFIELD_PATH", "RESULTS_CSV",
    "SUMMARY_STATS", "TERRAIN_STATS", "VALIDATION_STATS",
    "SITE_SELECTION_STATS", "LANDING_ZONES_PNG",
]

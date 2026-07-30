"""
Shared helper: every number the presentation quotes must trace back to
summary_stats.json. generate_world.py, run_monte_carlo.py,
plot_landing_zones.py, and validate_results.py each merge their own
numbers into the same file (rather than overwriting it) so the full
pipeline can be re-run stage by stage without losing earlier results.
"""

import json
import os

SUMMARY_PATH = os.path.join(os.path.dirname(__file__), "summary_stats.json")


def merge_summary_stats(new_data: dict, path: str = SUMMARY_PATH) -> dict:
    existing = {}
    if os.path.exists(path):
        with open(path) as f:
            existing = json.load(f)
    existing.update(new_data)
    with open(path, "w") as f:
        json.dump(existing, f, indent=2, sort_keys=True)
    return existing

"""Lunar landing site analysis.

Order: generate_world, run_monte_carlo, plot_landing_zones, validate_results.
validate_results fails the signal and bootstrap checks on the stored campaign.
See docs/known-issues.md before using a recommended coordinate.
"""

__version__ = "0.4.0"
__all__ = [
    "generate_world",
    "run_monte_carlo",
    "plot_landing_zones",
    "validate_results",
    "interactive_landing",
    "summary_stats",
]

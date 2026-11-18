"""
Lunar landing site analysis — MuJoCo Monte Carlo pipeline.

Pipeline order:
    generate_world     procedural terrain -> heightfield + MJCF
    run_monte_carlo    randomized trials -> results.csv
    plot_landing_zones kernel-regression safety map
    validate_results   does any of it mean anything?

Note: validate_results currently FAILS its signal and bootstrap checks.
See docs/known-issues.md before trusting any recommended coordinate.
"""

__version__ = "0.3.0"
__all__ = [
    "generate_world",
    "run_monte_carlo",
    "plot_landing_zones",
    "validate_results",
    "interactive_landing",
    "summary_stats",
]

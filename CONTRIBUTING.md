# Contributing

Issues and pull requests are welcome. Use GitHub issues for bugs, questions, and support.

## Useful changes

The mesh-collision bug is the shape of thing worth reporting. MuJoCo evaluates `type="mesh"` contact on the convex hull, so an earlier lander was touching a dome while the render still showed craters. Nothing raised. A bug like that is more useful than a new feature.

The safety map still has no terrain signal. See OPEN-1 in [docs/known-issues.md](docs/known-issues.md) and the approved experiment in [docs/open1_experiment.md](docs/open1_experiment.md). Do not change the `|r| = 0.2` bar or retune the legs/controller to make the check green. Put experimental campaign outputs under an isolated directory; never overwrite published `docs/summary_stats.json` or `figures/optimal_landing_zones.png`.

## Rules

Do not tune a threshold to make validation pass. If the signal test fails, that is the result. A negative result is fine.

Numbers that get quoted have to come from a JSON file the pipeline writes. An earlier banner said 1,000 kg while the model was 960 kg, because the figure was hardcoded in the print.

Physics changes need a regression test. A number that looks reasonable is not a test. `tests/test_static_equilibrium.py` checks a quantity you can derive on paper.

## Setup

```bash
git clone https://github.com/kkennethzhou/lunar-landing-analysis.git
cd lunar-landing-analysis
python -m venv .venv && source .venv/bin/activate
pip install -e ".[dev]"
pytest -m "not slow" && ruff check .
```

`pytest -m slow` drops the lander with the engine off and reruns the 10-trial fixture. Keep `seed = 42` so runs can be compared. If a change moves the results, say by how much and why in the pull request.

## Bug reports

Say what you expected from a hand calculation, what the simulation produced, and the difference.

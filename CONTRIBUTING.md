# Contributing

Issues and pull requests are welcome. Open a GitHub issue for bugs, questions, and support. There is no separate support channel.

## Most valuable contributions

**Silent-wrong-answer bugs.** MuJoCo evaluates `type="mesh"` collision against the convex hull, so an earlier lander was touching down on a smooth dome while the render still showed craters. Nothing errored. A bug of that shape is worth more than a feature.

**The open question.** The safety map has no terrain signal. See [docs/known-issues.md](docs/known-issues.md) OPEN-1. On the 500-trial run, `corr(force, slope) = +0.025` and the partial correlation controlling for drift is `+0.005`. `python -m lunar_lander.compare_roughness` is the harsher-terrain probe. Do not treat a short probe as a replacement for that 500-trial result.

## Ground rules

**Don't tune thresholds to make validation pass.** If the signal test fails, that's a result, not a bug in the test. A negative result is an acceptable outcome.

**Every reported number traces to a JSON file the pipeline writes.** No hardcoded values in print statements. An earlier version claimed a 1,000 kg lander while simulating 960 kg because of a literal in a banner string.

**Physics changes need a regression test.** A number that looks reasonable is not evidence. `tests/test_static_equilibrium.py` is the model: it asserts a property derivable from first principles.

## Workflow

```bash
git clone https://github.com/kkennethzhou/lunar-landing-analysis.git
cd lunar-landing-analysis
python -m venv .venv && source .venv/bin/activate
pip install -e ".[dev]"
pytest -m "not slow" && ruff check .
```

`pytest -m slow` also drops the lander with the engine off and reruns the 10-trial fixture. Keep `seed = 42` so runs stay comparable. If your change alters results, say by how much and why in the pull request.

## Reporting bugs

Include what you expected from first principles, what the simulation produced, and the gap. For physics issues, show the hand calculation.

# Contributing

Thanks for looking. This project is a work in progress with two documented blocking defects, so there's real work available.

## Most valuable contributions

**Silent-wrong-answer bugs.** The highest-impact find in this repo's history was that MuJoCo evaluates `type="mesh"` collision against the convex hull, so the lander had been landing on a smooth dome with every crater erased. Nothing errored. If you find another bug of that shape, it's worth more than any feature.

**The open question.** `corr(force, terrain_slope) = +0.022` — the safety map has no terrain signal. See [docs/known-issues.md](docs/known-issues.md) OPEN-1. The most promising untested analysis is a partial correlation controlling for drift speed.

**The blocking defects.** BLOCKING-1 (leg stiffness) and BLOCKING-2 (thrust cutoff) are specified in detail with recommended values. Fix stiffness first — the other ordering makes every trial fail.

## Ground rules

**Don't tune thresholds to make validation pass.** If the signal test fails, that's a result, not a bug in the test. A validated negative result is the acceptable outcome here.

**Every reported number traces to `summary_stats.json`.** No hardcoded values in print statements — the original version claimed a 1,000 kg lander while simulating 960 kg because of a literal in a banner string.

**Physics changes need a regression test.** A number that looks reasonable is not evidence. `tests/test_static_equilibrium.py` is the model: it asserts a property derivable from first principles.

## Workflow

```bash
git clone https://github.com/kkennethzhou/lunar-landing-analysis.git
cd lunar-landing-analysis
python -m venv .venv && source .venv/bin/activate
pip install -e ".[dev]"
pytest && ruff check .
```

Keep `seed = 42` so runs stay comparable. If your change alters results, say by how much and why in the PR description.

## Reporting bugs

Include: what you expected from first principles, what the simulation produced, and the gap. For physics issues, show the hand calculation — that's how both fixed bugs in this repo were found.

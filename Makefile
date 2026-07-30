.PHONY: help install world simulate plot validate all test lint clean

help:
	@echo "make install   - install package + dev dependencies"
	@echo "make all       - full pipeline: world -> simulate -> plot -> validate"
	@echo "make world     - generate procedural terrain + MJCF"
	@echo "make simulate  - run Monte Carlo trials"
	@echo "make plot      - build safety map"
	@echo "make validate  - run the four validation checks"
	@echo "make test      - pytest"
	@echo "make lint      - ruff"
	@echo "make clean     - remove generated artifacts"

install:
	pip install -e ".[dev]"

world:
	python -m lunar_lander.generate_world

simulate: world
	python -m lunar_lander.run_monte_carlo

plot: simulate
	python -m lunar_lander.plot_landing_zones

validate: plot
	python -m lunar_lander.validate_results

all: validate
	@echo ""
	@echo "Pipeline complete. Check validation output above --"
	@echo "a failing signal test means the recommended site is not trustworthy."

test:
	pytest

lint:
	ruff check .

clean:
	rm -rf physics_mesh/ figures/*.png *_stats.json results.csv
	find . -type d -name __pycache__ -exec rm -rf {} + 2>/dev/null || true

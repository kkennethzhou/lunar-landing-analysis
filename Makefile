.PHONY: help install world simulate plot validate all test lint clean open1 open1-smoke

help:
	@echo "make install   - install package + dev dependencies"
	@echo "make all       - full pipeline: world -> simulate -> plot -> validate"
	@echo "make world     - generate procedural terrain + MJCF"
	@echo "make simulate  - run Monte Carlo trials"
	@echo "make plot      - build safety map"
	@echo "make validate  - run the four validation checks"
	@echo "make open1     - OPEN-1 500-trial campaigns into output/open1/"
	@echo "make open1-smoke - OPEN-1 2-trial smoke into output/open1/"
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

open1:
	python -m lunar_lander.open1_campaigns --trials 500 --seed 42

open1-smoke:
	python -m lunar_lander.open1_campaigns --trials 2 --seed 42

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

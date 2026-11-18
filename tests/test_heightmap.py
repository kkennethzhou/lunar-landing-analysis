"""The validator and the world generator share one heightmap."""

import numpy as np

from lunar_lander.generate_world import make_heightmap
from lunar_lander.validate_results import load_terrain


def test_nominal_relief_is_the_shipped_grid():
    height = make_heightmap("nominal")
    assert height.shape == (128, 128)
    assert height.min() == 0.0
    assert abs(float(height.max()) - 4.443909476770371) < 1e-9


def test_validator_uses_the_generator_grid():
    height, slope = load_terrain("nominal")
    assert np.array_equal(height, make_heightmap("nominal"))
    assert slope.shape == height.shape


def test_harsh_preset_is_rougher_and_does_not_replace_nominal():
    nominal = make_heightmap("nominal")
    harsh = make_heightmap("harsh")
    assert harsh.max() > nominal.max() * 1.5
    assert np.array_equal(make_heightmap("nominal"), nominal)

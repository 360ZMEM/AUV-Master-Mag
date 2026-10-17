import pytest

from auv_mag_tracking.v2.robustness_pipeline import DirectionalRejoinContract


@pytest.mark.parametrize('offset,proxy,expected', [
    (6., 0., 20.), (6., .03, 20.), (6., -.03, 5.),
    (-6., 0., 20.), (-6., -.03, 20.), (-6., .03, 5.),
])
def test_only_opposing_observed_proxy_shortens_the_nominal_runin(offset, proxy, expected):
    assert DirectionalRejoinContract().choose(offset, proxy) == expected


def test_invalid_observation_is_not_silently_a_nominal_direction():
    with pytest.raises(ValueError):
        DirectionalRejoinContract().choose(6., float('nan'))

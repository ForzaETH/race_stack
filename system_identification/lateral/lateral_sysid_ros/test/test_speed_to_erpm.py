import numpy as np

from lateral_sysid_ros.speed_to_erpm_from_bag import fit_gain, window_mean


def test_fit_gain_with_fixed_offset_and_outliers():
    rng = np.random.default_rng(0)
    v = rng.uniform(-1.5, 1.5, 2000)
    erpm = 3500.0 * v - 5.0 + rng.normal(0.0, 30.0, v.size)
    erpm[:50] += 3000.0  # wheel spin
    gain, keep = fit_gain(v, erpm, offset=-5.0)
    assert abs(gain - 3500.0) < 5.0
    assert not keep[:50].any()


def test_window_mean():
    t_samples = np.arange(0.0, 10.0, 0.1)
    values = 2.0 * t_samples
    np.testing.assert_allclose(window_mean(t_samples, values, np.array([5.0]), 1.0), [10.0], atol=0.11)

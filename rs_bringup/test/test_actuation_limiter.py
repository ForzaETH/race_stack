"""Tests for physical-unit actuation limiting."""

import math

import pytest

from rs_bringup.actuation_limiter import ActuationLimiter
from rs_bringup.actuation_limiter import ActuationLimits
from rs_bringup.actuation_limiter import select_command_source
from rs_bringup.actuation_limiter import vesc_range_to_physical


def test_vesc_range_inverts_positive_gain():
    assert vesc_range_to_physical(0.15, 0.85, 1.131, 0.57) == pytest.approx(
        (-0.3713528, 0.2475685))


def test_vesc_range_orders_negative_gain():
    assert vesc_range_to_physical(0.15, 0.85, -1.131, 0.458) == pytest.approx(
        (-0.3465959, 0.2723254))


def test_vesc_range_converts_erpm_to_speed():
    assert vesc_range_to_physical(-69000.0, 69000.0, 4800.0, 0.0) == (
        pytest.approx(-14.375), pytest.approx(14.375))


@pytest.mark.parametrize('gain', [0.0, math.nan])
def test_vesc_range_rejects_invalid_gain(gain):
    with pytest.raises(ValueError):
        vesc_range_to_physical(0.15, 0.85, gain, 0.5)


@pytest.fixture
def limiter():
    """Create representative forward-driving limits."""
    return ActuationLimiter(ActuationLimits(
        speed_min=0.0,
        speed_max=8.0,
        max_acceleration=2.0,
        max_deceleration=4.0,
        steering_min=-0.3,
        steering_max=0.3,
        max_steering_rate=1.0,
    ))


def test_acceleration_and_steering_are_rate_limited(limiter):
    state = limiter.step(5.0, 0.2, 0.1)

    assert state.speed == pytest.approx(0.2)
    assert state.steering_angle == pytest.approx(0.1)


def test_deceleration_uses_its_own_limit(limiter):
    limiter.step(5.0, 0.0, 1.0)
    state = limiter.step(0.0, 0.0, 0.25)

    assert state.speed == pytest.approx(1.0)


def test_targets_are_bounded_before_rate_limiting(limiter):
    state = limiter.step(20.0, -2.0, 10.0)

    assert state.speed == pytest.approx(8.0)
    assert state.steering_angle == pytest.approx(-0.3)


def test_disabled_state_stops_and_centres_safely(limiter):
    limiter.step(2.0, 0.3, 1.0)
    state = limiter.step(2.0, 0.3, 0.1, enabled=False)

    assert state.speed == 0.0
    assert state.steering_angle == pytest.approx(0.2)


def test_zero_dt_does_not_advance_output(limiter):
    state = limiter.step(2.0, 0.3, 0.0)

    assert state.speed == 0.0
    assert state.steering_angle == 0.0


@pytest.mark.parametrize('value', [math.inf, -math.inf, math.nan])
def test_non_finite_commands_are_rejected(limiter, value):
    with pytest.raises(ValueError):
        limiter.step(value, 0.0, 0.1)


def test_invalid_limits_are_rejected():
    with pytest.raises(ValueError):
        ActuationLimits(
            speed_min=2.0,
            speed_max=1.0,
            max_acceleration=1.0,
            max_deceleration=1.0,
            steering_min=-0.3,
            steering_max=0.3,
            max_steering_rate=1.0,
        )


def select_source(human=False, autonomous=False, teleop=9.9,
                  navigation=9.9, joy=9.9):
    return select_command_source(
        10.0, joy, human, autonomous, teleop, navigation, 0.2, 0.2)


def test_human_deadman_selects_only_teleop():
    assert select_source(human=True) == 'teleop'
    assert select_source(human=True, teleop=None) is None


def test_autonomous_deadman_selects_only_navigation():
    assert select_source(autonomous=True) == 'navigation'
    assert select_source(autonomous=True, navigation=None) is None


def test_human_has_priority_when_both_deadmen_are_held():
    assert select_source(human=True, autonomous=True) == 'teleop'
    assert select_source(
        human=True, autonomous=True, teleop=None) is None


@pytest.mark.parametrize(
    'kwargs',
    [
        {},
        {'human': True, 'joy': None},
        {'human': True, 'joy': 9.7},
        {'human': True, 'teleop': 9.7},
        {'autonomous': True, 'navigation': 9.7},
    ],
)
def test_missing_stale_or_released_inputs_disable_actuation(kwargs):
    assert select_source(**kwargs) is None

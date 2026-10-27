"""Deterministic physical-unit limits for Ackermann commands."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class ActuationLimits:
    """Limits expressed in vehicle units rather than VESC units."""

    speed_min: float
    speed_max: float
    max_acceleration: float
    max_deceleration: float
    steering_min: float
    steering_max: float
    max_steering_rate: float

    def __post_init__(self):
        values = vars(self).values()
        if not all(math.isfinite(value) for value in values):
            raise ValueError('actuation limits must be finite')
        if self.speed_min > self.speed_max:
            raise ValueError('speed_min must not exceed speed_max')
        if not self.speed_min <= 0.0 <= self.speed_max:
            raise ValueError('speed limits must include the safe zero command')
        if self.steering_min > self.steering_max:
            raise ValueError('steering_min must not exceed steering_max')
        if not self.steering_min <= 0.0 <= self.steering_max:
            raise ValueError('steering limits must include the centred command')
        if self.max_acceleration <= 0.0:
            raise ValueError('max_acceleration must be positive')
        if self.max_deceleration <= 0.0:
            raise ValueError('max_deceleration must be positive')
        if self.max_steering_rate <= 0.0:
            raise ValueError('max_steering_rate must be positive')


@dataclass(frozen=True)
class ActuationState:
    """Physical command most recently allowed by the limiter."""

    speed: float = 0.0
    steering_angle: float = 0.0


def _clip(value, lower, upper):
    return min(max(value, lower), upper)


def _move_towards(current, target, maximum_change):
    difference = target - current
    return current + _clip(difference, -maximum_change, maximum_change)


class ActuationLimiter:
    """Apply bounds and slew limits while retaining the applied state."""

    def __init__(self, limits):
        self.limits = limits
        self.state = ActuationState()

    def step(self, requested_speed, requested_steering, dt, enabled=True):
        """Advance the applied command by ``dt`` seconds.

        Acceleration is the positive speed-reference slew rate and
        deceleration is the negative slew rate. A disabled command stops the
        motor immediately and returns steering toward zero at the configured
        steering rate.
        """
        if not all(math.isfinite(value) for value in
                   (requested_speed, requested_steering, dt)):
            raise ValueError('actuation inputs and dt must be finite')
        if dt < 0.0:
            raise ValueError('dt must not be negative')

        steering_target = 0.0
        if enabled:
            speed_target = _clip(
                requested_speed, self.limits.speed_min, self.limits.speed_max)
            steering_target = _clip(
                requested_steering,
                self.limits.steering_min,
                self.limits.steering_max)

            speed_rate = self.limits.max_acceleration
            if speed_target < self.state.speed:
                speed_rate = self.limits.max_deceleration
            speed = _move_towards(
                self.state.speed, speed_target, speed_rate * dt)
        else:
            speed = 0.0

        steering = _move_towards(
            self.state.steering_angle,
            steering_target,
            self.limits.max_steering_rate * dt)
        self.state = ActuationState(speed=speed, steering_angle=steering)
        return self.state


def select_command_source(now, last_joy, human_deadman,
                          autonomous_deadman, last_teleop,
                          last_navigation, command_timeout, joy_timeout):
    """Select the fresh command explicitly authorized by the deadman.

    Human control has priority when both deadman buttons are held. It does not
    fall back to navigation when its command is stale, because that would let
    the human deadman authorize an autonomous command.
    """
    if last_joy is None or not 0.0 <= now - last_joy <= joy_timeout:
        return None
    if human_deadman:
        if (last_teleop is not None
                and 0.0 <= now - last_teleop <= command_timeout):
            return 'teleop'
        return None
    if autonomous_deadman:
        if (last_navigation is not None
                and 0.0 <= now - last_navigation <= command_timeout):
            return 'navigation'
    return None

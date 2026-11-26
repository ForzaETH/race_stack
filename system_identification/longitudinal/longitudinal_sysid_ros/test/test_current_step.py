from longitudinal_sysid_ros.current_step_node import CurrentStep


def make():
    # binary fractions, so that the phase boundaries are exact
    return CurrentStep(hold_time=0.25, accel_time=1.0, brake_time=2.0)


def step(exp, now, held):
    """Step with a fresh joystick that reports the button `held` or released."""
    return exp.step(now, held, not held)


def timed_out(exp, now):
    """Step with a stale joystick: no deadman, and no release either."""
    return exp.step(now, False, False)


def run(exp, start, end, deadman):
    """Step every 0.125 s from `start` to `end` inclusive, return the last command."""
    n = round((end - start) / 0.125)
    for i in range(n + 1):
        command = step(exp, start + i * 0.125, deadman)
    return command


def test_waits_for_deadman_then_holds_drives_brakes_and_ends():
    exp = make()
    assert step(exp, 0.0, False) is None
    assert step(exp, 5.0, True) == 'hold'
    assert step(exp, 5.125, True) == 'hold'
    assert step(exp, 5.25, True) == 'current'
    assert step(exp, 6.125, True) == 'current'
    assert step(exp, 6.25, True) == 'brake'
    assert step(exp, 8.125, True) == 'brake'
    assert step(exp, 8.25, True) == 'done'


def test_releasing_the_deadman_brakes_and_never_drives_again_in_that_run():
    exp = make()
    assert step(exp, 0.0, True) == 'hold'
    assert step(exp, 0.5, True) == 'current'
    assert step(exp, 0.625, False) == 'brake'
    assert step(exp, 0.75, True) == 'brake'
    assert step(exp, 2.625, True) == 'done'
    assert step(exp, 2.75, True) is None


def test_releasing_during_the_hold_cancels_without_braking():
    exp = make()
    assert step(exp, 0.0, True) == 'hold'
    assert step(exp, 0.125, False) is None
    assert step(exp, 0.5, True) == 'hold'


def test_holding_through_the_end_does_not_restart():
    exp = make()
    assert run(exp, 0.0, 3.25, True) == 'done'
    assert run(exp, 3.375, 10.0, True) is None


def test_release_and_press_again_repeats_the_experiment():
    exp = make()
    assert run(exp, 0.0, 3.25, True) == 'done'
    assert step(exp, 4.0, False) is None
    assert step(exp, 5.0, True) == 'hold'
    assert step(exp, 5.25, True) == 'current'
    assert step(exp, 6.25, True) == 'brake'
    assert step(exp, 8.25, True) == 'done'


def test_a_press_during_the_brake_does_not_arm_the_next_run():
    exp = make()
    step(exp, 0.0, True)
    assert step(exp, 0.5, False) == 'brake'
    assert run(exp, 0.625, 2.375, True) == 'brake'
    assert step(exp, 2.5, True) == 'done'
    assert step(exp, 2.625, True) is None


def test_a_joystick_timeout_after_a_run_does_not_arm_the_next_one():
    exp = make()
    assert run(exp, 0.0, 3.25, True) == 'done'
    assert timed_out(exp, 4.0) is None
    assert step(exp, 5.0, True) is None
    assert step(exp, 5.5, False) is None
    assert step(exp, 6.0, True) == 'hold'


def test_a_joystick_timeout_brakes():
    exp = make()
    assert run(exp, 0.0, 0.5, True) == 'current'
    assert timed_out(exp, 0.625) == 'brake'


def test_a_joystick_timeout_during_the_hold_cancels_and_does_not_arm():
    exp = make()
    assert step(exp, 0.0, True) == 'hold'
    assert timed_out(exp, 0.125) is None
    assert step(exp, 0.5, True) is None
    assert step(exp, 0.625, False) is None
    assert step(exp, 0.75, True) == 'hold'

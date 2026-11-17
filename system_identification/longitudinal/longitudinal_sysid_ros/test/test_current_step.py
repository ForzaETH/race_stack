from longitudinal_sysid_ros.current_step_node import CurrentStep


def test_waits_for_deadman_then_drives_brakes_and_ends():
    exp = CurrentStep(accel_time=1.0, brake_time=2.0)
    assert exp.step(0.0, False) is None
    assert exp.step(5.0, True) == 'current'
    assert exp.step(5.9, True) == 'current'
    assert exp.step(6.0, True) == 'brake'
    assert exp.step(7.9, True) == 'brake'
    assert exp.step(8.0, True) == 'done'


def test_releasing_the_deadman_brakes_and_never_drives_again():
    exp = CurrentStep(accel_time=1.0, brake_time=2.0)
    assert exp.step(0.0, True) == 'current'
    assert exp.step(0.3, False) == 'brake'
    assert exp.step(0.4, True) == 'brake'
    assert exp.step(2.4, True) == 'done'
    assert exp.step(2.5, True) == 'done'

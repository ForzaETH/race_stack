#!/usr/bin/env python3
"""The referee's rules on made-up numbers. Usage:  python3 test_bench_race.py"""
from bench_referee import Race

L = 100.0
APART = 5.0     # [m] too far to count as engaged
CLOSE = 1.0     # [m]


def run(rows):
    race = Race(L)
    for row in rows:
        outcome = race.update(*row)
        if outcome:
            return outcome
    return None


def test_contact_and_walls():
    assert run([(0.0, 10, 12, 2.0, [1, 1])]) == "contact"
    assert run([(0.0, 10, 12, 1.0, [1, 0])]) == "wall_ego"
    assert run([(0.0, 10, 12, 1.0, [0, 1])]) == "wall_opp"


def test_timeout():
    ticks = [k / 10 for k in range(302)]
    assert run([(t, 10, 12, APART, [0, 0]) for t in ticks]) == "invalid"
    assert run([(t, 10, 11, CLOSE, [0, 0]) for t in ticks]) == "held_off"


def test_pass_across_the_lap_line():
    assert run([(k / 10, 0.5, 99.5, CLOSE, [0, 0]) for k in range(40)]) == "pass"


def test_dropping_back_restarts_the_hold():
    ahead = [(k / 10, 11, 10, CLOSE, [0, 0]) for k in range(20)]
    behind = [(2.0, 10, 11, CLOSE, [0, 0])]
    ahead_again = [(2 + k / 10, 11, 10, CLOSE, [0, 0]) for k in range(1, 20)]
    assert run(ahead + behind + ahead_again) is None


def test_pass_under_way_finishes_after_the_limit():
    behind = [(29 + k / 10, 10, 11, CLOSE, [0, 0]) for k in range(10)]
    ahead = [(29.9 + k / 10, 11, 10, CLOSE, [0, 0]) for k in range(40)]
    assert run(behind + ahead) == "pass"


if __name__ == "__main__":
    for name, test in list(globals().items()):
        if name.startswith("test_"):
            test()
    print("rule tests passed")

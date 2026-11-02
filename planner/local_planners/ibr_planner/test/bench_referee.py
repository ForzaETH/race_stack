#!/usr/bin/env python3
"""Referee and logger for one benchmark race, for any pair of planners.

The ego car is the attacker and the opp car is the defender. Logs both cars at 20 Hz, stops at
the first decisive event and writes that outcome as the last line of the log:

    pass      attacker stayed ahead by at least --margin for --hold seconds
    held_off  --t-max seconds passed with no pass, after the cars came within --engage;
              an attacker already ahead at --t-max gets the rest of its --hold first
    invalid   --t-max seconds passed and the cars never came within --engage
    contact   the car bodies touched; the gym lets them pass through, so stop here
    wall_ego, wall_opp   that car hit a wall; the gym freezes it for good
    interrupted  the odometry stopped mid-race, so this race has no result; run it again


Usage:  python3 bench_referee.py run.jsonl global_waypoints.json [--t-max 30]
"""
import argparse
import json
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray

CARS = ("ego", "opp")
KEEP_EVERY = 5     # the bridge publishes at 100 Hz; 20 Hz is plenty for plots and metrics
MOVING = 0.1       # [m/s] the race clock starts when a car first exceeds this
MAX_GAP = 1.0      # [s] a longer silence means the sim stopped or restarted mid-race


class Race:
    """The rules, with no ROS in them so they can be tested on made-up numbers."""

    def __init__(self, L, t_max=30.0, hold=3.0, margin=0.58, engage=2.0):
        self.L, self.t_max, self.hold, self.margin, self.engage = L, t_max, hold, margin, engage
        self.t_ahead = None                           # when the attacker last got ahead
        self.engaged = False
        self.min_dist = math.inf

    def lead(self, s_ego, s_opp):
        """Attacker minus defender along the raceline, wrapped so the lap line is harmless."""
        return (s_ego - s_opp + self.L / 2) % self.L - self.L / 2

    def update(self, t, s_ego, s_opp, dist, flags):
        """t counts from the race start. Returns the outcome, or None while still racing."""
        self.min_dist = min(self.min_dist, dist)
        if flags[0] and flags[1]:                     # the gym sets both only for car-car contact
            return "contact"
        if flags[0]:
            return "wall_ego"
        if flags[1]:
            return "wall_opp"
        self.engaged = self.engaged or dist < self.engage
        if self.lead(s_ego, s_opp) < self.margin:
            self.t_ahead = None
        elif self.t_ahead is None:
            self.t_ahead = t
        elif t - self.t_ahead >= self.hold:
            return "pass"
        if t >= self.t_max and self.t_ahead is None:  # let a pass under way finish its hold
            return "held_off" if self.engaged else "invalid"
        return None


class Referee(Node):
    def __init__(self, path, race):
        super().__init__("bench_referee")
        self.f = open(path, "w")
        self.t_last = 0.0                             # race time of the last step [s]
        self.waited = False                           # cars seen standing still yet?
        self.race = race
        self.outcome = None
        self.t0 = None                                # race start [s]
        self.n = 0
        self.flags = [0, 0]
        self.odom = {c: None for c in CARS}           # latest t, x, y, v
        self.s = {c: None for c in CARS}              # latest raceline s
        for c in CARS:
            self.create_subscription(Odometry, f"/{c}/car_state/odom",
                                     lambda m, c=c: self.odom_cb(c, m), 50)
            self.create_subscription(Odometry, f"/{c}/car_state/frenet/odom",
                                     lambda m, c=c: self.frenet_cb(c, m), 50)
        self.create_subscription(Int8MultiArray, "/sim/collisions", self.collision_cb, 50)

    def collision_cb(self, msg):
        # Latched: a brief touch between two odometry messages must not go unnoticed.
        self.flags = [max(a, b) for a, b in zip(self.flags, msg.data)]

    def frenet_cb(self, car, msg):
        self.s[car] = msg.pose.pose.position.x

    def odom_cb(self, car, msg):
        p = msg.pose.pose.position
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.odom[car] = dict(t=t, x=p.x, y=p.y, v=msg.twist.twist.linear.x)
        if car == "ego" and None not in (*self.odom.values(), *self.s.values()):
            self.step()

    def step(self):
        e, o = self.odom["ego"], self.odom["opp"]
        if self.t0 is None:
            if max(abs(e["v"]), abs(o["v"])) < MOVING:
                self.waited = True                    # saw the cars standing at the start
                return
            if not self.waited:                       # joined a race already under way
                self.outcome = "interrupted"
                self.write(dict(outcome=self.outcome, t_end=0.0))
                return
            self.t0 = e["t"]
        t = e["t"] - self.t0
        if t - self.t_last > MAX_GAP:     # the data after a gap belongs to another sim
            self.outcome = "interrupted"
            self.write(dict(outcome=self.outcome, t_end=self.t_last, min_dist=self.race.min_dist,
                            engaged=self.race.engaged))
            return
        self.t_last = t
        dist = math.hypot(e["x"] - o["x"], e["y"] - o["y"])
        self.outcome = self.race.update(t, self.s["ego"], self.s["opp"], dist, self.flags)
        self.n += 1
        if self.n % KEEP_EVERY == 0 or self.outcome:
            self.write(dict(t=t, ego=[e["x"], e["y"], e["v"], self.s["ego"]],
                            opp=[o["x"], o["y"], o["v"], self.s["opp"]], dist=dist))
        if self.outcome:
            self.write(dict(outcome=self.outcome, t_end=t, min_dist=self.race.min_dist,
                            engaged=self.race.engaged))

    def write(self, row):
        self.f.write(json.dumps(row) + "\n")
        self.f.flush()


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("log", help="jsonl file to write")
    ap.add_argument("map", help="stack_master/maps/<map>/global_waypoints.json")
    ap.add_argument("--t-max", type=float, default=30.0, help="race length limit [s]")
    args = ap.parse_args()

    with open(args.map) as f:
        L = json.load(f)["global_traj_wpnts_iqp"]["wpnts"][-1]["s_m"]
    rclpy.init()
    node = Referee(args.log, Race(L, t_max=args.t_max))
    try:
        while rclpy.ok() and node.outcome is None:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    print(node.outcome)


if __name__ == "__main__":
    main()

from __future__ import annotations
from typing import Callable

import rclpy
import numpy as np
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from rs_msgs.msg import FrenetState, Obstacle, ObstacleArray, OvertakingSectors, OvertakingSpline, RaceState, Wpnt, WpntArray

from rs_state_machine.parameters_codegen import state_machine as state_machine_parameters

# latched: late subscribers still get the last message (publisher and subscriber must both use it)
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


# State machine transition definitions
# Keep in mind the guards are evaluated in order!
def timetrials_transitions() -> Transitions:
    sm = StateMachine
    return {
        RaceState.GB_TRACK: [(sm.always,                                        RaceState.GB_TRACK)],
    }

def head_to_head_transitions() -> Transitions:
    sm = StateMachine
    return {
        RaceState.GB_TRACK: [(not_(sm.raceline_clear),                          RaceState.TRAILING)],

        RaceState.TRAILING: [(all_of(sm.raceline_clear, sm.near_raceline),      RaceState.GB_TRACK),
                             (all_of(not_(sm.raceline_clear), sm.in_overtaking_sector,
                                     sm.overtake_line_available,
                                     sm.overtake_line_clear),                   RaceState.OVERTAKE)],

        RaceState.OVERTAKE: [(not_(sm.overtake_line_clear),                     RaceState.TRAILING),
                             (all_of(sm.overtake_complete, sm.raceline_clear),  RaceState.GB_TRACK),
                             (all_of(not_(sm.overtake_complete),
                                     sm.overtake_line_available),               RaceState.OVERTAKE),
                             (sm.always,                                        RaceState.TRAILING)],
    }


# RaceState constant names by value, for readable logs
STATE_NAMES = {getattr(RaceState, name): name for name in ('GB_TRACK', 'TRAILING', 'OVERTAKE')}

Guard = Callable[["StateMachine"], bool]
Transitions = dict[int, list[tuple[Guard, int]]]

# Guard combinators, a guard is any callable taking the state machine
def all_of(*guards: Guard) -> Guard:
    def g(c): return all(guard(c) for guard in guards)
    g.__name__ = " and ".join(guard.__name__ for guard in guards)
    return g

def not_(guard: Guard) -> Guard:
    def g(c): return not guard(c)
    g.__name__ = f"not {guard.__name__}"
    return g


class IllegalStateError(RuntimeError):
    def __init__(self, state: int):
        self.state = state
        super().__init__(f"Illegal state {STATE_NAMES.get(state, state)}!")


class StateMachine(Node):

    def always(self) -> bool:
        return True

    def near_raceline(self) -> bool:
        return np.abs(self.cur_d) < self.params.gb_ego_width_m 

    def in_overtaking_sector(self) -> bool:
        idx = int(self.cur_s / self.waypoints_dist)
        return idx < len(self.ot_allowed) and bool(self.ot_allowed[idx])

    def overtake_line_clear(self) -> bool:
        """No obstacle within overtaking_horizon_m ahead whose center is within lateral_width_ot_m of the overtaking spline."""
        spline = self.last_valid_avoidance_wpnts
        if not spline:
            return True
        return self._blocking_obstacle(lambda s: self._d_at(spline, s), self.params.overtaking_horizon_m, self.params.lateral_width_ot_m) is None


    def raceline_clear(self) -> bool:
        """No obstacle within gb_horizon_m ahead whose center is within lateral_width_gb_m of the raceline."""
        # the raceline is the reference of the Frenet frame, its d is 0 everywhere
        return self._blocking_obstacle(lambda _s: 0.0, self.params.gb_horizon_m, self.params.lateral_width_gb_m) is None

    def _blocking_obstacle(self, line_d: Callable[[float], float], horizon: float, width: float) -> Obstacle | None:
        """First obstacle within `horizon` ahead whose center is within `width` of the line given by its d(s)."""
        return next(
            (obs for obs in self.obstacles
             if self._ahead_within(obs.s_center, horizon) and abs(line_d(obs.s_center) - obs.d_center) < width),
            None,
        )

    @staticmethod
    def _d_at(wpnts: list[Wpnt], s: float) -> float:
        """Lateral position of a line given by waypoints at `s` (nearest waypoint)."""
        return wpnts[int(np.argmin([abs(wpnt.s_m - s) for wpnt in wpnts]))].d_m

    def _ahead_within(self, s: float, horizon: float) -> bool:
        """`s` is at most `horizon` meters ahead of the car, across the start/finish line too."""
        return (s - self.cur_s) % self.track_length < horizon

    def overtake_line_available(self) -> bool:
        if ((self.avoidance_wpnts is None) or 
            not self._past_switch_hysteresis(self.avoidance_wpnts)): # Switched too recently
            return False
        return True

    def overtake_complete(self) -> bool:
        """The car has passed the end of the overtaking spline, where it rejoins the raceline."""
        if not self.last_valid_avoidance_wpnts:
            return True
        end_s = self.last_valid_avoidance_wpnts[-1].s_m
        # signed distance to the spline end, wrapped to [-track_length/2, track_length/2)
        half = self.track_length / 2
        remaining = (end_s - self.cur_s + half) % self.track_length - half
        return remaining <= 0

    def _past_switch_hysteresis(self, avoidance_wpnts: OvertakingSpline) -> bool:
        since_switch = Time.from_msg(avoidance_wpnts.header.stamp) - Time.from_msg(avoidance_wpnts.last_switch_time)
        return since_switch >= Duration(seconds=self.params.splini_hyst_timer_sec)

    def next_state(self) -> tuple[int, Guard | None]:
        """Return the next state and the guard that triggered it (None if no guard held)."""
        rules = self.transitions.get(self.state)
        if rules is None:
            raise IllegalStateError(self.state)
        for guard, target in rules:
            if guard(self):
                return target, guard
        return self.state, None

    def __init__(self):
        super().__init__('state_machine')

        self.param_listener = state_machine_parameters.ParamListener(self)
        self.params = self.param_listener.get_params()
        self.waypoints_dist = 0.1

        # The state machine waits for information before running. At first callback, each topic
        # discards itself from the list. Only once the pending_inputs are empty does state machine
        # start.
        self.pending_inputs = {"/global_waypoints_scaled", "/car_state/frenet"}

        self.track_length = 0.0
        self.create_subscription(WpntArray, "/global_waypoints_scaled", self.glb_wpnts_scaled_cb, LATCHED)

        self.cur_s = 0.0
        self.cur_d = 0.0
        self.create_subscription(FrenetState, '/car_state/frenet', self.car_state_frenet_cb, 10)

        if self.params.mode == 'head_to_head':
            self.last_valid_avoidance_wpnts = None
            self.avoidance_wpnts = None
            self.create_subscription(OvertakingSpline, '/planner/avoidance/otwpnts', self.avoidance_cb, 10)

            self.obstacles = []
            self.create_subscription(ObstacleArray, "/perception/obstacles", self.obstacles_cb, 10)

            # The ot_interpolator publishes the sectors once and on every change
            self.pending_inputs.add("/overtaking_sectors")
            self.ot_allowed = np.zeros(0, dtype=bool)
            sectors_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(OvertakingSectors, '/overtaking_sectors', self.ot_sectors_cb, sectors_qos)
        
        # Construct state machine (timetrials or head to head)
        self.state = RaceState.GB_TRACK

        if self.params.mode == 'timetrials':
            self.transitions = timetrials_transitions()
        elif self.params.mode == 'head_to_head':
            self.transitions = head_to_head_transitions()
        else:
            raise NotImplementedError(f"Mode {self.params.mode} not recognized")

        # RaceState Publisher
        self.state_pub = self.create_publisher(RaceState, 'state', 10)

        # main loop
        self.main_loop = self.create_timer(1/self.params.rate_hz, self.main_loop_callback)

    def glb_wpnts_scaled_cb(self, data: WpntArray):
        self.track_length = list(data.wpnts)[-1].s_m # last waypoint's is the track length
        self.pending_inputs.discard("/global_waypoints_scaled")

    def car_state_frenet_cb(self, msg: FrenetState):
        self.cur_s = msg.s
        self.cur_d = msg.d
        self.pending_inputs.discard("/car_state/frenet")

    def avoidance_cb(self, data: OvertakingSpline):
        if len(data.wpnts) > 0:
            self.avoidance_wpnts = data
            # cache the latest spline that is past the side-switch hysteresis, used by overtake_line_clear
            if self._past_switch_hysteresis(data):
                self.last_valid_avoidance_wpnts = list(data.wpnts)
        else:
        # If empty we don't overwrite the avoidance waypoints
            pass

    def ot_sectors_cb(self, msg: OvertakingSectors):
        ot_allowed = np.zeros(max(msg.end, default=-1) + 1, dtype=bool)
        for start, end, allowed in zip(msg.start, msg.end, msg.allowed):
            if allowed:
                ot_allowed[start:end + 1] = True
        self.ot_allowed = ot_allowed
        self.pending_inputs.discard("/overtaking_sectors")
        self.get_logger().info(f"Overtaking allowed in sectors {[i for i, allowed in enumerate(msg.allowed) if allowed]}")

    def obstacles_cb(self, data):
        self.obstacles = data.obstacles

    def main_loop_callback(self):
        
        # Only start once all infomration has arrived
        if self.pending_inputs:
            self.get_logger().debug(f"Waiting for {', '.join(sorted(self.pending_inputs))}", throttle_duration_sec=2.0)
            return
        
        # Apply runtime parameter changes
        if self.param_listener.is_old(self.params):
            self.params = self.param_listener.get_params()
            
        # Determine the next state from the state machine
        new_state, guard = self.next_state()
        if guard is not None and new_state != self.state:
            self.get_logger().info(f"{STATE_NAMES[self.state]} -> {STATE_NAMES[new_state]} ({guard.__name__})")
        
        self.state = new_state
        msg = RaceState()
        msg.state = self.state
        self.state_pub.publish(msg)


# defined as entry point in setup.py:
def main(args=None):
    rclpy.init(args=args)

    state_machine = StateMachine()

    rclpy.spin(state_machine)

    state_machine.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()

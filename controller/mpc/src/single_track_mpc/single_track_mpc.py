#! /usr/bin/env python3
from typing import Tuple
import numpy as np
import rospy
from scipy.integrate import solve_ivp
from frenet_converter.frenet_converter import FrenetConverter
from f110_msgs.msg import WpntArray
from single_track_mpc.acados_settings import acados_settings
from single_track_mpc.utils.splinify import SplineTrack
from single_track_mpc.utils.indicies import StateIndex
from pbl_config import STMPCConfig, CarConfig, PacejkaTireConfig, TrailingConfig


class Single_track_MPC_Controller:
    def __init__(self, pose_frenet, racecar_version: str,
                 stmpc_config: STMPCConfig, car_config: CarConfig,
                 tire_config: PacejkaTireConfig,
                 trailing_config: TrailingConfig,
                 controller_frequency: float,
                 using_gokart: bool) -> None:
        """
        Initialise MPC object.
        """

        # Init the parameters
        self.racecar_version = racecar_version
        self.stmpc_config: STMPCConfig = stmpc_config
        self.car_config: CarConfig = car_config
        self.tire_config: PacejkaTireConfig = tire_config
        self.trailing_config: TrailingConfig = trailing_config
        self.controller_frequency = controller_frequency
        self.using_gokart = using_gokart
        self.mpc_init_params()
        if self.stmpc_config.MPC_freq != self.controller_frequency:
            rospy.logwarn("[MPC Controller] Controller frequency and MPC frequency are not equal. Warm start update will not be applied.")

        # Init the solver
        self.mpc_initialize_solver(pose_frenet)

    def mpc_init_params(self) -> None:
        # upper layer parameters: MPC
        self.t_MPC = 1 / self.stmpc_config.MPC_freq

        rospy.loginfo(
            f"[MPC Controller] Steps Delay set to {self.stmpc_config.steps_delay}. Equivalent to a delay of {1000*self.stmpc_config.steps_delay*self.t_MPC:3.2f} milliseconds.")
        # time delay propagation
        self.t_delay = self.stmpc_config.t_delay + self.t_MPC

        # seering angle buffer size
        buf_size = 2
        self.steering_angle_buf = np.zeros(buf_size)

        # Initial state
        self.mpc_sd = np.zeros((self.stmpc_config.N + 1, 2))
        self.u0 = np.zeros(2)
        self.fre_s = 0
        self.previous_frenet_s = 0
        # Initial time
        self.comp_time = 0
        # Initial overtake lateral distance
        self.overtake_d = self.stmpc_config.overtake_d
        self.loop_rate = 40
        self.gap = None
        self.gap_should = None
        self.gap_error = None
        self.gap_actual = None
        self.v_diff = None
        self.i_gap = 0

    def mpc_initialize_solver(self, pose_frenet: np.ndarray) -> None:
        """Initialises the controller. Global waypoints are stored in a SplineTrack. All necessary parameters are stored. """

        rospy.loginfo(f"[MPC Controller] Waiting for global waypoints")
        mincurv_raceline = rospy.wait_for_message("/global_waypoints", WpntArray)
        rospy.loginfo(f"[MPC Controller] Global waypoints obtained")

        x, y = self._transform_waypoints_to_cartesian(mincurv_raceline.wpnts)
        self.fren_conv = FrenetConverter(x, y)

        d_left, coords_path, d_right = self._transform_waypoints_to_coords(mincurv_raceline.wpnts)

        self.spline = SplineTrack(coords_direct=coords_path)

        self.nr_laps = 0

        kapparef = [x.kappa_radpm for x in mincurv_raceline.wpnts]
        vx_ref = [x.vx_mps for x in mincurv_raceline.wpnts]
        self.s_ref = np.array([x.s_m for x in mincurv_raceline.wpnts])
        (
            self.constraint, self.model, self.acados_solver, self.model_params
        ) = acados_settings(self.s_ref, kapparef, d_left, d_right,
                            self.stmpc_config, self.car_config, self.tire_config)

        # create warmstart
        self.apply_warm_start(pose_frenet=pose_frenet)  # TODO uniform general mpc input to a MPCstate

        self.kappa = kapparef
        self.prev_acc = 0

    def apply_warm_start(self, pose_frenet: np.ndarray):
        """Applies a warm start to the MPC solver.

        Args:
            pose_frenet (np.ndarray): _description_
        """
        warm_start_trajectory = self.get_warm_start(pose_frenet=pose_frenet, const_acc=1, const_steer_vel=0.0)
        for i in range(self.stmpc_config.N + 1):
            self.acados_solver.set(i, "x", warm_start_trajectory[i][:self.model.n_x])
            if i < self.stmpc_config.N:
                self.acados_solver.set(i, "u", warm_start_trajectory[i][self.model.n_x:])

     # main loop

    def main_loop(
            self,
            state,
            position_in_map,
            waypoint_array_in_map,
            speed_now,
            opponent,
            position_in_map_frenet,
            single_track_state,
            track_length,
            compute_time):
        # TODO: possibly rewrite
        # Updating parameters from manager
        self.state = state
        self.position_in_map = position_in_map
        self.waypoint_array_in_map = waypoint_array_in_map
        self.speed_now = speed_now
        self.opponent = opponent
        self.position_in_map_frenet = position_in_map_frenet
        self.vel_y = single_track_state[0]
        self.yaw_rate = single_track_state[1]
        if self.using_gokart:
            self.measured_acc = single_track_state[2]
            self.measured_steer = single_track_state[3]
        else:
            self.measured_steer = self.steering_angle_buf[-1]
            # TODO: improve acceleration estimation for f110 car, eg lowpass filter
            self.measured_acc = self.prev_acc
        self.track_length = track_length
        self.comp_time = compute_time
        # update initial state from sensors
        track_length = self.spline.track_length - 0.1  # Needed as values are not exact
        center_car_s = self.position_in_map_frenet[0] + self.car_config.lr * np.cos(self.position_in_map_frenet[2])
        deriv_center = self.spline.get_derivative(center_car_s)
        alpha_center = self.position_in_map[0, 2] - np.arctan2(deriv_center[1], deriv_center[0])
        # make aplha_center between -pi and pi
        alpha_center = alpha_center % (2 * np.pi)
        if alpha_center > np.pi:
            alpha_center = alpha_center - 2 * np.pi

        if self.position_in_map_frenet[0] < self.previous_frenet_s - \
                1:  # - 1 for rejecting noise or moving slightly backwards
            self.nr_laps += 1
        self.previous_frenet_s = self.position_in_map_frenet[0]

        current_pos_s = self.position_in_map_frenet[0] + self.nr_laps * self.spline.track_length

        self.fre_s = current_pos_s
        self.fre_d = self.position_in_map_frenet[1]
        self.fre_alpha = alpha_center

        x0 = np.array([self.fre_s, self.fre_d, self.fre_alpha, speed_now, self.vel_y,
                      self.measured_steer, self.yaw_rate, self.measured_acc], dtype=np.float64)

        # time delat compensation
        self.t_delay = self.comp_time
        self.t_delay = 0.00

        # print("t_delay",self.t_delay)
        # if x0[3] >= 0.1:
        #     propagated_x = self.propagate_time_delay(x0, self.u0)
        # else:
        propagated_x = x0

        # set the initial state for the mpc
        self.acados_solver.set(0, "lbx", propagated_x)
        self.acados_solver.set(0, "ubx", propagated_x)

        # dynamically change the target speed, weight parameters and constraints
        for i in range(self.stmpc_config.N + 1):
            # head to head racing
            # for the trailing, MPC just needs to change the reference speed (get from the trailing controller)
            if (self.state == "TRAILING" and (self.opponent is not None)):  # Trailing controller
                target_v_speed = self.trailing_controller(self.waypoint_array_in_map[0, 2])
            else:
                idx = np.abs(self.waypoint_array_in_map[:,4]  - self.mpc_sd[i,0]%self.spline.track_length).argmin()
                target_v_speed=self.waypoint_array_in_map[idx,2]
                self.overtake_d=0
                # head to head overtaking
                if (self.state == "OVERTAKE" and (self.opponent is not None)):
                    self.overtake_d = self.waypoint_array_in_map[idx, 3]

            if i < self.stmpc_config.steps_delay:
                # NOTE: increase the weight of the the steering angle cost, to effectively keep it constant
                multiplier = 1e6
                online_parameters = np.array([target_v_speed,
                                              self.stmpc_config.qadv,
                                              self.stmpc_config.qv,
                                              self.stmpc_config.qn,
                                              self.stmpc_config.qalpha,
                                              self.stmpc_config.qjerk,
                                              multiplier * self.stmpc_config.qddelta,
                                              self.stmpc_config.alat_max,
                                              self.stmpc_config.a_min,
                                              self.stmpc_config.a_max,
                                              self.stmpc_config.track_safety_margin,
                                              self.overtake_d],
                                             dtype=np.float64)
            else:
                online_parameters = np.array([target_v_speed,
                                              self.stmpc_config.qadv,
                                              self.stmpc_config.qv,
                                              self.stmpc_config.qn,
                                              self.stmpc_config.qalpha,
                                              self.stmpc_config.qjerk,
                                              self.stmpc_config.qddelta,
                                              self.stmpc_config.alat_max,
                                              self.stmpc_config.a_min,
                                              self.stmpc_config.a_max,
                                              self.stmpc_config.track_safety_margin,
                                              self.overtake_d],
                                             dtype=np.float64)

            self.acados_solver.set(i, "p", online_parameters)

            if i < self.stmpc_config.N:
                self.acados_solver.set(i, "lbu", np.array([-50, self.stmpc_config.ddelta_min]))
                self.acados_solver.set(i, "ubu", np.array([50, self.stmpc_config.ddelta_max]))
                # do not change i+1 to i since mpc needs the initial state
                self.acados_solver.set(
                    i + 1, "lbx", np.array([self.stmpc_config.v_min, self.stmpc_config.delta_min, self.stmpc_config.a_min]))
                self.acados_solver.set(
                    i + 1, "ubx", np.array([self.stmpc_config.v_max, self.stmpc_config.delta_max, self.stmpc_config.a_max]))

        # Update Warm Start
        # if self.stmpc_config.MPC_freq == self.controller_frequency:
        #     self.update_warm_start()

        # Solve OCP
        status = self.acados_solver.solve()
        if status != 0:
            # if the solver fails, apply warm start again
            # the initial warm start is a bit rough, but it does the job
            self.apply_warm_start(pose_frenet=[self.fre_s, self.fre_d, self.fre_alpha])

        # get solution
        self.u0 = self.acados_solver.get(0, "u")
        self.prev_acc = self.acados_solver.get(0, "x")[StateIndex.ACCEL.value]
        delayed_index = self.stmpc_config.steps_delay + 1
        self.pred_x = self.acados_solver.get(delayed_index, "x")
        self.steering_angle = self.pred_x[5]  # propagated_x[5] + self.u0[1]/40 #
        self.speed = self.pred_x[3]  # propagated_x[3] + self.u0[0]/40 #
        self.acceleration = self.prev_acc + self.u0[0] / self.controller_frequency # NOTE: Euler integration for accounting different frequencies controller/mpc time step
        self.prev_acc = self.acceleration
        self.jerk = self.u0[0]

        # steering buffer
        # shift the elements in the buffer to the right and add the predicted steering angle
        self.steering_angle_buf[1:] = self.steering_angle_buf[:-1]
        self.steering_angle_buf[0] = self.pred_x[5]
        self.states = None

        ##### visualization #####
        # Creating waypoint array with predicted positions
        self.mpc_sd = np.array([
            self.acados_solver.get(j, "x")[:2] for j in range(self.stmpc_config.N + 1)
        ])
        if status == 0:
            states = []
            for node in range(self.stmpc_config.N + 1):
                result = self.acados_solver.get(node, "x").tolist()
                for item in result:
                    states.append(item)
            self.states = states
            return self.speed, self.acceleration, self.jerk, self.steering_angle, self.states, status
        else:
            return 0, 0, 0, self.measured_steer , self.states, status

    #############
    # Utilities #
    #############
    def get_warm_start(self, pose_frenet: np.ndarray, const_acc: float, const_steer_vel: float) -> np.array:
        """
        Returns a warm start trajectory for the MPC. This is done by propagating the current state with a constant acceleration and steering angle.

        Input:  const_acc   : Constant acceleration
                const_steer : Constant steering angle

        Returns: np.array    : Warm start trajectory
        """
        warm_start = np.zeros((self.stmpc_config.N + 1, 10)) # TODO: hardcoded state space dimension
        warm_start[0] = np.array([pose_frenet[0], pose_frenet[1], pose_frenet[2],
                                 1, 0, 0, 0, const_acc, 0, const_steer_vel]) # TODO setting the velocity to current actual velocity might remove slowing down
        for i in range(1, self.stmpc_config.N + 1):
            xdot = self._dynamics_of_car(0, warm_start[i - 1])
            warm_start[i] = warm_start[i-1] + np.array(xdot) / self.stmpc_config.MPC_freq  # ugly euler integration
        return warm_start

    def update_warm_start(self) -> np.array:
        """
        Updates the warm start trajectory for the MPC. This is done by propagating the current states with the given inputs and reapplying the last input.
        """
        for i in range(self.stmpc_config.N + 1):
            # Get the current state and input
            current_state = self.acados_solver.get(i, "x")
            if i < self.stmpc_config.N:
                current_input = self.acados_solver.get(i, "u")
            else:
                current_input = np.zeros(self.model.n_u)  # Use zeros for the last step input

            # Concatenate state and input for dynamics computation
            state_with_input = np.concatenate((current_state, current_input))

            # Compute the state derivative
            xdot = self._dynamics_of_car(0, state_with_input)

            # Update the state using Euler integration
            updated_state = current_state + np.array(xdot)[:self.model.n_x] / self.controller_frequency
            self.acados_solver.set(i, "x", updated_state)

            # Update the input for the next state
            if i < self.stmpc_config.N - 1:
                next_inputs = self.acados_solver.get(i + 1, "u")
                self.acados_solver.set(i, "u", next_inputs)

    def _transform_waypoints_to_coords(self, data: WpntArray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Helper function to store the received waypoints into the right format such that they can be used for initialisation of SplineTrack.

        Input: data: WpntArray: Holds the data received from the globalwaypoints.

        Returns: Tuple: Stores the waypoints as follows: Left boundaries, reference path, right boundaries
        """
        waypoints = np.zeros((len(data), 2))
        d_left = np.zeros(len(data))
        d_right = np.zeros(len(data))
        boundaries = np.zeros((len(data), 2))
        for idx, wpnt in enumerate(data):
            waypoints[idx] = [wpnt.x_m, wpnt.y_m]
            d_left[idx] = wpnt.d_left              # Fix for boundaries due to frenet cartesian conversion
            d_right[idx] = wpnt.d_right              # Fix for boundaries due to frenet cartesian conversion
        res_coords = np.array([boundaries[:-1], waypoints[:-1], boundaries[:-1]])
        return d_left, res_coords, d_right

    def _transform_waypoints_to_cartesian(self, data: WpntArray) -> Tuple[np.ndarray, np.ndarray]:
        """Helper function to store the received waypoints into the right format such that they can be used for initialisation of SplineTrack.

        Input: data: WpntArray: Holds the data received from the globalwaypoints.

        Returns: Tuple: Stores the waypoints as follows: Left boundaries, reference path, right boundaries
        """
        x = np.zeros(len(data))
        y = np.zeros(len(data))
        for idx, wpnt in enumerate(data):
            x[idx] = wpnt.x_m
            y[idx] = wpnt.y_m
        return np.array(x), np.array(y)

    def propagate_time_delay(self, states: np.array, inputs: np.array) -> np.array:
        """
        RK45 forward propagation. Calls the function _dynamics_of_car which contains the dynamics of the car.

        Input:  np.array    : Holds the current state which needs to be propagated.
                inputs      : Holds the current inputs from the MPC.

        Returns: np.array   : Propagated state without input.

        """

        # Initial condition on the ODE
        x0 = np.concatenate((states, [inputs[0]], [0]), axis=0)
        # forward propagation
        self.t_delay = 0.0
        # DEBUG print nicely formatted aligned state
        # print("BEFORE state:", end=" ")
        # for x in x0:
        #     print(f"{x:5.2f}", end=" ")
        # print("\n")

        solution = solve_ivp(
            self._dynamics_of_car,
            t_span=[
                0,
                self.t_delay],
            y0=x0,
            method='RK45',
            atol=1e-4,
            rtol=1e-4)
        # NOTE: can't really converge? seen with negative acceleration

        # DEBUG print nicely formatted aligned state
        # print("AFTER  state:", end=" ")
        # for x in solution.y[:,-1]:
        #     print(f"{x:5.2f}", end=" ")
        # print("\n")

        solution = [x[-1] for x in solution.y]
        # Constraint on max. steering angle
        if abs(solution[5]) > self.constraint.delta_max:
            solution[5] = np.sign(solution[5]) * self.constraint.delta_max

        # Constraint on max. vel
        if abs(solution[3]) > self.constraint.v_x_max:
            solution[3] = np.sign(solution[3]) * self.constraint.v_x_max

        # On minimum speed
        if abs(solution[3]) < self.constraint.v_x_min:
            solution[3] = self.constraint.v_x_min

        # Only get the state as solution of where the car will be in t_delay seconds
        # not including the input
        # states[5] = solution[5]
        # states[6] = solution[6]
        # return states
        return np.array(solution)[:-2]

    def _dynamics_of_car(self, t, x0) -> list:
        '''
        Used for forward propagation. This function takes the dynamics from the acados model.
        '''

        s = x0[0]
        n = x0[1]
        theta = x0[2]
        v_x = x0[3]
        v_y = x0[4]
        delta = x0[5]
        yaw_rate = x0[6]
        accel = x0[7]
        derDelta = x0[8]
        jerk = x0[9]
        xdot = self.model.f_expl_func(
            s, n, theta, v_x, v_y, delta, yaw_rate, accel, derDelta, jerk, self.model_params.p)

        xdot = [float(xdot[0]), float(xdot[1]), float(xdot[2]), float(xdot[3]),
                float(xdot[4]), float(xdot[5]), float(xdot[6]), float(xdot[7]), derDelta, jerk]

        return xdot

    def trailing_controller(self, global_speed):

        self.gap = (self.opponent[0] - self.position_in_map_frenet[0]) % self.track_length  # gap to opponent
        # prev_gap = self.gap if self.gap_actual is None else self.gap_actual
        self.gap_actual = self.gap
        if self.trailing_config.trailing_mode:
            self.gap_should = self.trailing_config.trailing_gap
        else:
            self.gap_should = self.position_in_map_frenet[2] * self.trailing_config.trailing_gap

        self.gap_error = self.gap_should - self.gap_actual
        self.v_diff = self.position_in_map_frenet[2] - self.opponent[2]
        self.i_gap = np.clip(self.i_gap + self.gap_error / self.loop_rate, -10, 10)

        p_value = self.gap_error * self.trailing_config.trailing_p_gain
        d_value = self.v_diff * self.trailing_config.trailing_d_gain
        i_value = self.i_gap * self.trailing_config.trailing_i_gain

        self.trailing_command = np.clip(self.opponent[2] - p_value - i_value - d_value, 0, global_speed)
        if not self.opponent[4] and self.gap_actual > self.gap_should:
            self.trailing_command = max(self.trailing_config.blind_trailing_speed, self.trailing_command)

        return self.trailing_command

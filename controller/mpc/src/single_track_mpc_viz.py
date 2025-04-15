#! /usr/bin/env python3
import os
from typing import Tuple

import numpy as np
import rospkg
import rospy
import yaml
from dynamic_reconfigure.client import Client
from dynamic_reconfigure.msg import Config
from f110_msgs.msg import WpntArray
from frenet_converter.frenet_converter import FrenetConverter
from dynamic_reconfigure.client import Client
from dynamic_reconfigure.msg import Config
from nav_msgs.msg import Odometry
from pbl_config import (load_car_config_ros, load_pacejka_tire_config_ros,
                        load_STMPC_config_ros)
from plotting_fnc import *
from single_track_mpc.bicycle_model import bicycle_model
from single_track_mpc.utils.indicies import StateIndex
from single_track_mpc.utils.splinify import SplineTrack
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker, MarkerArray


class tireNode:
    def __init__(self,) -> None:
        """
        Initialise MPC object.

        Input:  conf_file   : String containing the path to the param_config.yaml file
        """

        rospy.init_node("tire_force_node", anonymous=True, log_level=rospy.DEBUG)

        self._init_params()

        self.vel_x = 0
        # self.glob2frenet = rospy.ServiceProxy('convert_glob2frenet_service', Glob2Frenet)
        # self.frenet2glob = rospy.ServiceProxy('convert_frenet2glob_service', Frenet2Glob)

        rospy.Subscriber("/mpc_controller/states", Float64MultiArray, self._states)

        self.v_x_pub = rospy.Publisher("/mpc_controller/v_x", Float64MultiArray, queue_size=10)
        self.tireforce__pub = rospy.Publisher("/mpc_controller/tire_force_viz", Float64MultiArray, queue_size=10)
        self.map_left_bound_pub = rospy.Publisher("/mpc_controller/map_left_bound", MarkerArray, queue_size=10)
        self.map_right_bound_pub = rospy.Publisher("/mpc_controller/map_right_bound", MarkerArray, queue_size=10)
        self.d_left_pub = rospy.Publisher("/mpc_controller/d_left", MarkerArray, queue_size=10)
        self.d_mid_pub = rospy.Publisher("/mpc_controller/d_mid", MarkerArray, queue_size=10)
        self.d_right_pub = rospy.Publisher("/mpc_controller/d_right", MarkerArray, queue_size=10)
        self.pred_pos_pub = rospy.Publisher("/mpc_controller/predicted_position", MarkerArray, queue_size=10)

        self.acc_x_pub = rospy.Publisher("/mpc_controller/acc_x", Float64MultiArray, queue_size=10)
        self.acc_y_pub = rospy.Publisher("/mpc_controller/acc_y", Float64MultiArray, queue_size=10)
        self.ellipse_pub = rospy.Publisher("/mpc_controller/ellipse", Float64MultiArray, queue_size=10)
        self.horizon_pub = rospy.Publisher("/mpc_controller/horizon", Float64MultiArray, queue_size=10)

        self.mpc_dyn_rec_client = Client("/mpc_param_tuner", config_callback=self.stmpc_config_cb)

        self._initialize_path()

    def _init_params(self,) -> None:
        dir_path = rospkg.RosPack().get_path("stack_master")
        self.racecar_version = rospy.get_param("/racecar_version")
        self.floor = rospy.get_param("/floor")
        self.stmpc_config = load_STMPC_config_ros(self.racecar_version)
        self.car_config = load_car_config_ros(self.racecar_version)
        self.tire_config = load_pacejka_tire_config_ros(self.racecar_version, self.floor)

        self.stmpc_config.N = self.stmpc_config.N

        self.car_config.lf = self.car_config.lf

        # Just for cost function testing
        self.stmpc_config.track_safety_margin
        t_delay = self.stmpc_config.t_delay
        self.t_MPC = 1 / self.stmpc_config.MPC_freq

        # time delay propagation
        self.t_delay = t_delay + self.t_MPC

    def stmpc_config_cb(self, params: Config):
        """
        Here the mpc parameters are updated if changed with rqt (dyn reconfigure)
        Values from .yaml file are set in mpc_online_params_server
        """
        for k, v in params.items():
            if k != "groups":
                setattr(self.stmpc_config, k, v)

        self.stmpc_config = self.stmpc_config

    def _initialize_path(self) -> None:
        """Initialises the controller. Global waypoints are stored in a SplineTrack. All necessary parameters are stored. """

        rospy.loginfo(f"[MPC Controller] Waiting for global waypoints")
        raceline = rospy.wait_for_message("/global_waypoints", WpntArray)
        rospy.loginfo(f"[MPC Controller] Global waypoints obtained")

        x, y = self._transform_waypoints_to_cartesian(raceline.wpnts)
        self.fren_conv = FrenetConverter(x, y)

        d_left, coords_path, d_right = self._transform_waypoints_to_coords(raceline.wpnts) # on f track trajectory is 81.803 m long.

        self.spline = SplineTrack(coords_direct=coords_path)

        # create the mpc contro ller struct to use the function

        kapparef = [x.kappa_radpm for x in raceline.wpnts]
        vx_ref =[x.vx_mps for x in raceline.wpnts]
        self.max_v_target = max(vx_ref)
        self.s_ref = np.array([x.s_m for x in raceline.wpnts])
        self.pathlength = self.s_ref[-1]


        self.model, self.constraint, self.params = bicycle_model(self.s_ref, kapparef ,d_left, d_right, self.stmpc_config, self.car_config, self.tire_config)

        self.states=np.zeros(self.model.n_x*(self.stmpc_config.N+1))
        self.states_k_1=np.zeros(self.model.n_x*(self.stmpc_config.N+1))
        self.acc_x = np.zeros(self.stmpc_config.N)
        self.acc_y = np.zeros(self.stmpc_config.N)
        self.ellipse = np.zeros(self.stmpc_config.N)

        # visualize the boundaries of the map
        map_boundaries = np.zeros((len(self.s_ref),2,2))
        for stage in range(len(self.s_ref)):
            map_boundaries[stage,0,:] = self.fren_conv.get_cartesian(self.s_ref[stage], -self.model.right_bound_s(self.s_ref[stage])).reshape(2)
            map_boundaries[stage,1,:] = self.fren_conv.get_cartesian(self.s_ref[stage],  self.model.left_bound_s(self.s_ref[stage])).reshape(2)

        self.map_boundaries=map_boundaries

    #############
    # Callbacks #
    #############
    def _states(self, states: Float64MultiArray) -> None:
        """slip angle callback callback. Normally not used."""
        self.states_k_1 = self.states
        self.states= states.data


    #############
    # Utilities #
    #############
    def _transform_waypoints_to_coords(self, data: WpntArray)-> Tuple[np.ndarray, np.ndarray, np.ndarray]:
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
        res_coords = np.array([boundaries[:-1],waypoints[:-1],boundaries[:-1]])
        return d_left, res_coords, d_right

    def _transform_waypoints_to_cartesian(self, data: WpntArray)-> Tuple[np.ndarray, np.ndarray]:
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

    #############
    # Main Loop #
    #############
    def viz_loop(self) -> None:
        """Main loop. The control procedure is implemented here."""

        rate = rospy.Rate(1/self.t_MPC)

        while not rospy.is_shutdown():


            # publish mpc param and map boundaries
            # self.mpc_param_pub.publish(self.mpc_param)
            self.publish_waypoint_markers(self.map_boundaries[:,0,:], "map_b_right")
            self.publish_waypoint_markers(self.map_boundaries[:,1,:], "map_b_left")

            # # publish trajectory bounds
            boundaries = np.zeros((self.stmpc_config.N, 3, 3))
            mpc_sd = []
            for stage in range(self.stmpc_config.N ):
                x_ = self.states[8*stage:8*(stage+1)-1] # TODO: hardcoded nx
                s_ = x_[0]
                s_mod = s_ % self.spline.track_length
                # store predicted position
                mpc_sd.append(x_[:2])

                boundaries[stage,0,0:2] = self.fren_conv.get_cartesian(s_mod, -self.model.right_bound_s(s_mod) + self.stmpc_config.track_safety_margin).reshape(2)
                boundaries[stage,1,0:2] = self.fren_conv.get_cartesian(s_mod, 0).reshape(2)
                boundaries[stage,2,0:2] = self.fren_conv.get_cartesian(s_mod, self.model.left_bound_s(s_mod) - self.stmpc_config.track_safety_margin).reshape(2)

            self.publish_waypoint_markers(boundaries[:,0,:], "d_right")
            self.publish_waypoint_markers(boundaries[:,1,:], "d_mid")
            self.publish_waypoint_markers(boundaries[:,2,:], "d_left")
            mpc_s_coords = np.array([el[0]%self.spline.track_length for el in mpc_sd])
            mpc_d_coords = np.array([el[1] for el in mpc_sd])
            pred_waypoints_x, pred_waypoints_y = self.fren_conv.get_cartesian(mpc_s_coords, mpc_d_coords)
            pred_waypoints = [np.array([x, y]) for x, y in zip(pred_waypoints_x, pred_waypoints_y)]

            self.publish_waypoint_markers(pred_waypoints, type="pred")

            self.pub_acc()

            rate.sleep()
    ##########################
    # Publishing/Visualizing #
    ##########################
    def publish_current_pos(self, coord: np.array) -> None:
        """Publishes current position into /mpc_controller/current_position"""
        waypoint_marker = Marker()
        waypoint_marker.header.frame_id = "map"
        waypoint_marker.header.stamp = rospy.Time.now()
        waypoint_marker.type = 2
        waypoint_marker.scale.x = 0.2
        waypoint_marker.scale.y = 0.2
        waypoint_marker.scale.z = 0.2
        waypoint_marker.color.r = 0.0
        waypoint_marker.color.g = 1.0
        waypoint_marker.color.b = 0.0
        waypoint_marker.color.a = 1.0
        waypoint_marker.pose.position.x = coord[0]
        waypoint_marker.pose.position.y = coord[1]
        waypoint_marker.pose.position.z = 0
        waypoint_marker.pose.orientation.x = 0
        waypoint_marker.pose.orientation.y = 0
        waypoint_marker.pose.orientation.z = 0
        waypoint_marker.pose.orientation.w = 1
        waypoint_marker.id = 1
        self.pos_pub.publish(waypoint_marker)

    def publish_states(self, states: list) -> None:
        """Publishes states into /mpc_controller/states"""
        msg = Float64MultiArray()
        msg.data = states
        self.states_pub.publish(msg)

    def publish_propagated_states(self, states: list) -> None:
        """Publishes propagated state into /mpc_controller/states_propagated"""
        msg = Float64MultiArray()
        msg.data = states
        self.states_propagated_pub.publish(msg)

    def publish_waypoint_markers(self, waypoints: np.ndarray, type:str) -> None:
        """Publishes predicted positions, right, left boundaries and points on reference path."""
        waypoint_markers = MarkerArray()
        wpnt_id = 0

        for waypoint in waypoints:
            waypoint_marker = Marker()
            waypoint_marker.header.frame_id = "map"
            waypoint_marker.header.stamp = rospy.Time.now()
            waypoint_marker.type = 2

            waypoint_marker.scale.x = 0.15
            waypoint_marker.scale.y = 0.15
            waypoint_marker.scale.z = 0.15

            #values
            waypoint_marker.pose.position.x = waypoint[0]
            waypoint_marker.pose.position.y = waypoint[1]
            waypoint_marker.pose.position.z = 0
            waypoint_marker.pose.orientation.x = 0
            waypoint_marker.pose.orientation.y = 0
            waypoint_marker.pose.orientation.z = 0
            waypoint_marker.pose.orientation.w = 1
            waypoint_marker.id = wpnt_id + 1
            wpnt_id += 1
            waypoint_markers.markers.append(waypoint_marker)

            if type == "pred":
                waypoint_marker.color.r = 1.0
                waypoint_marker.color.g = 0.0
                waypoint_marker.color.b = 1.05
                waypoint_marker.color.a = 1.0

            elif type == "target":
                waypoint_marker.color.r = 1.0
                waypoint_marker.color.g = 1.0
                waypoint_marker.color.b = 0.0
                waypoint_marker.color.a = 1.0
            elif type == "d_mid":
                waypoint_marker.color.r = 0.0
                waypoint_marker.color.g = 1.0
                waypoint_marker.color.b = 0.0
                waypoint_marker.color.a = 1.0
                #predicted velocity viz
                waypoint_marker.pose.position.z = waypoint[2]
            else:
                waypoint_marker.color.r = 1.0
                waypoint_marker.color.g = 0.0
                waypoint_marker.color.b = 0.0
                waypoint_marker.color.a = 1.0



        if type == "pred":
            self.pred_pos_pub.publish(waypoint_markers)
        elif type == "map_b_left":
            self.map_left_bound_pub.publish(waypoint_markers)
        elif type == "map_b_right":
            self.map_right_bound_pub.publish(waypoint_markers)
        elif type == "d_left":
            self.d_left_pub.publish(waypoint_markers)
        elif type == "d_mid":
            self.d_mid_pub.publish(waypoint_markers)
        elif type == "d_right":
            self.d_right_pub.publish(waypoint_markers)
        else:
            self.target_pos_pub.publish(waypoint_markers)

    def publish_current_pos_n(self,state: np.array) -> None:
        """Publishes current distance from reference path in /mpc_controller/current_pos_n. Not used."""
        position = Odometry()
        position.header.stamp = rospy.Time.now()
        position.header.frame_id = "base_link"
        position.pose.pose.position.y = state[StateIndex.MIN_DIST_TO_CENTER_LINE_N]
        self.pos_n_pub.publish(position)

    def publish_tireforce(self, states: list) -> None:
        """Publishes states into /mpc_controller/states"""
        msg = Float64MultiArray()
        msg.data = states
        self.tireforce__pub.publish(msg)

    def pub_acc(self):
        dummy_params = np.zeros(self.model.p.size()[0])
        n = self.model.n_x
        for i in range(self.stmpc_config.N):
            theta = self.states[n*i+StateIndex.ORIENTATION_THETA]
            v_x = self.states[n*i+StateIndex.VELOCITY_V_X]
            v_y = self.states[n*i+StateIndex.VELOCITY_V_Y]
            delta = self.states[n*i+StateIndex.STEERING_ANGLE_DELTA]
            yaw_rate = self.states[n*i+StateIndex.YAW_RATE]
            a_x = self.states[n*i+StateIndex.ACCEL]
            x_dot = self.model.f_expl_func(0, 0, theta, v_x, v_y, delta, yaw_rate, a_x, 0, 0, dummy_params)
            self.acc_x[i] = x_dot[3]
            self.acc_y[i] = x_dot[4]
            self.ellipse[i] = (x_dot[3]/self.stmpc_config.a_max)**2 + (x_dot[4]/self.stmpc_config.alat_max)**2

        msg = Float64MultiArray()
        msg.data = self.acc_x
        self.acc_x_pub.publish(msg)
        msg.data = self.acc_y
        self.acc_y_pub.publish(msg)
        msg.data = self.ellipse
        self.ellipse_pub.publish(msg)
if __name__ == "__main__":
    controller = tireNode()

    controller.viz_loop()

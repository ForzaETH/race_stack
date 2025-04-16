#! /usr/bin/env python3
import os
from typing import Tuple
import numpy as np
import rospkg
import rospy
import yaml
from ackermann_msgs.msg import AckermannDriveStamped
from f110_msgs.msg import WpntArray
from frenet_conversion.srv import Frenet2Glob, Glob2Frenet
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from plotting_fnc import *
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32,Float64MultiArray
from tf.transformations import euler_from_quaternion
from kinematic_mpc.utils.indicies import StateIndex
from kinematic_mpc.utils.splinify import SplineTrack
from visualization_msgs.msg import Marker, MarkerArray
from dynamic_reconfigure.msg import Config
from dynamic_reconfigure.client import Client

from pbl_config import KMPCConfig, load_KMPC_config_ros, CarConfig, load_car_config_ros
from frenet_converter.frenet_converter import FrenetConverter
from kinematic_mpc.bicycle_model import bicycle_model


class tireNode:
    def __init__(self,) -> None:
        """
        Initialise MPC object.

        Input:  conf_file   : String containing the path to the param_config.yaml file
        """

        rospy.init_node("tire_force_node", anonymous=True, log_level=rospy.DEBUG)

        self._init_params()

        self.vel_x = 0
        buf_size = 2
        self.steering_angle_buf = np.zeros(buf_size)
        self.steering_angle_buf[-1]= 0

        self.glob2frenet = rospy.ServiceProxy('convert_glob2frenet_service', Glob2Frenet)
        self.frenet2glob = rospy.ServiceProxy('convert_frenet2glob_service', Frenet2Glob)

        rospy.Subscriber("/car_state/odom", Odometry, self._odom_cb)
        rospy.Subscriber("/car_state/odom_frenet", Odometry, self._odom_frenet_cb)
        rospy.Subscriber("/car_state/pose", PoseStamped, self._pose_cb)
        rospy.Subscriber("/drive", AckermannDriveStamped, self._input_cb)
        rospy.Subscriber("/vesc/sensors/imu/raw", Imu, self._imu_cb)
        rospy.Subscriber("/slip_angle", Float32, self._slip_angle)
        rospy.Subscriber("/mpc_controller/states", Float64MultiArray, self._states)
        #TODO: adapt dyn reconf to Client
        # rospy.Subscriber("/mpc_param_tuner/parameter_updates", Config, self.mpc_params_cb)
        self.mpc_dyn_rec_client = Client("/mpc_param_tuner", config_callback=self.mpc_config_cb)

        self.tireforce__pub = rospy.Publisher("/mpc_controller/tire_force_viz", Float64MultiArray, queue_size=10)
        self.map_left_bound_pub = rospy.Publisher("/mpc_controller/map_left_bound", MarkerArray, queue_size=10)
        self.map_right_bound_pub = rospy.Publisher("/mpc_controller/map_right_bound", MarkerArray, queue_size=10)
        self.d_left_pub = rospy.Publisher("/mpc_controller/d_left", MarkerArray, queue_size=10)
        self.d_mid_pub = rospy.Publisher("/mpc_controller/d_mid", MarkerArray, queue_size=10)
        self.d_right_pub = rospy.Publisher("/mpc_controller/d_right", MarkerArray, queue_size=10)
        self.pred_pos_pub = rospy.Publisher("/mpc_controller/predicted_position", MarkerArray, queue_size=10)


        self._initialize_path()

    def _init_params(self,) -> None:
        self.racecar_version = rospy.get_param("/racecar_version")
        self.kmpc_config: KMPCConfig = load_KMPC_config_ros(racecar_version=self.racecar_version)
        self.car_config: CarConfig = load_car_config_ros(racecar_version=self.racecar_version)

        # Just for cost function testing
        t_delay = self.kmpc_config.t_delay
        self.t_MPC = 1 / self.kmpc_config.MPC_freq

        # time delay propagation
        self.t_delay = t_delay + self.t_MPC
        self.states=np.zeros(5*(self.kmpc_config.N))
        self.states_k_1=np.zeros(5*(self.kmpc_config.N))

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


        self.model, self.constraint, self.params = bicycle_model(self.s_ref, kapparef,vx_ref ,d_left, d_right, self.kmpc_config, self.car_config)

        # visualize the boundaries of the map
        map_boundaries = np.zeros((len(self.s_ref),2,2))
        for stage in range(len(self.s_ref)):
            map_boundaries[stage,0,:] = self.fren_conv.get_cartesian(self.s_ref[stage], -self.model.right_bound_s(self.s_ref[stage])).reshape(2)
            map_boundaries[stage,1,:] = self.fren_conv.get_cartesian(self.s_ref[stage],  self.model.left_bound_s(self.s_ref[stage])).reshape(2)

        self.map_boundaries=map_boundaries

    #############
    # Callbacks #
    #############
    def _odom_cb(self, data: Odometry) -> None:
        """Odom callback"""
        self.vel_x = data.twist.twist.linear.x
        self.vel_y = data.twist.twist.linear.y
        self.r = data.twist.twist.angular.z

    def _odom_frenet_cb(self, data: Odometry) -> None:
        """Odom frenet callback"""
        self.pos_s = data.pose.pose.position.x
        self.pos_n = data.pose.pose.position.y

        self.alpha = data.pose.pose.orientation.z

    def _pose_cb(self, data: PoseStamped) -> None:
        """Position callback"""
        self.pos_x = data.pose.position.x
        self.pos_y = data.pose.position.y
        self.theta = euler_from_quaternion([data.pose.orientation.x, data.pose.orientation.y,
                                    data.pose.orientation.z, data.pose.orientation.w])[2]

    def _input_cb(self, data: AckermannDriveStamped) -> None:
        """Steering angle callback. Normally not used since the steering angle of the MPC is taken."""

        self.steering_angle = data.drive.steering_angle

    def _imu_cb(self, data: Imu) -> None:
        """acceleration callback. Normally not used."""
       # self.acceleration = data.linear_acceleration.x

    def mpc_config_cb(self, params:Config):
        """
        Here the mpc parameters are updated if changed with rqt (dyn reconfigure)
        Values from .yaml file are set in mpc_online_params_server
        """
        for k, v in params.items():
            if k != "groups":
                setattr(self.kmpc_config, k, v)

    def _slip_angle(self, data: Float32) -> None:
        """slip angle callback callback. Normally not used."""
        self.slip_angle= data

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
            self.publish_waypoint_markers(self.map_boundaries[:,0,:], "map_b_right")
            self.publish_waypoint_markers(self.map_boundaries[:,1,:], "map_b_left")

            # # publish trajectory bounds
            boundaries = np.zeros((self.kmpc_config.N+1, 3, 2))
            mpc_sd = []
            for stage in range(self.kmpc_config.N ):
                x_ = self.states[5*stage:5*(stage+1)-1]
                s_ = x_[StateIndex.POS_ON_CENTER_LINE_S]
                s_mod = s_ % self.spline.track_length
                n_ = x_[StateIndex.MIN_DIST_TO_CENTER_LINE_N]
                v_=x_[StateIndex.VELOCITY_V_X]
                # store predicted position
                mpc_sd.append(x_[:2])
                # distance to border
                # consider left_bound_s and right_bound_s are positive. They are the distance from the centerline to the left and right boundary
                # bound_inflation is positive, it is the dynamic inflation of the boundary
                boundaries[stage,0,0:2] = self.fren_conv.get_cartesian(s_mod, -self.model.right_bound_s(s_mod) + self.kmpc_config.track_safety_margin).reshape(2)
                boundaries[stage,1,0:2] = self.fren_conv.get_cartesian(s_mod, 0).reshape(2)
                boundaries[stage,2,0:2] = self.fren_conv.get_cartesian(s_mod, self.model.left_bound_s(s_mod) - self.kmpc_config.track_safety_margin).reshape(2)

            self.publish_waypoint_markers(boundaries[:-1,0,:], "d_right") # skip the last point because they seemed to be buggy in visualization
            self.publish_waypoint_markers(boundaries[:-1,1,:], "d_mid")
            self.publish_waypoint_markers(boundaries[:-1,2,:], "d_left")

            mpc_s_coords = np.array([el[0]%self.spline.track_length for el in mpc_sd])
            mpc_d_coords = np.array([el[1] for el in mpc_sd])
            pred_waypoints_x, pred_waypoints_y = self.fren_conv.get_cartesian(mpc_s_coords, mpc_d_coords)
            pred_waypoints = [np.array([x, y]) for x, y in zip(pred_waypoints_x, pred_waypoints_y)]


            self.publish_waypoint_markers(pred_waypoints, type="pred")

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

    def publish_ackermann_msg(self , state: np.ndarray, next_input: bool = True) -> None:
        ack_msg = AckermannDriveStamped()
        ack_msg.header.stamp = rospy.Time.now()
        ack_msg.header.frame_id = "base_link"

        ack_msg.drive.steering_angle = state[StateIndex.STEERING_ANGLE_DELTA]
        ack_msg.drive.acceleration = state[StateIndex.ACCEL]
        # ack_msg.drive.speed = state[State.VELOCITY_V_X]
        #ack_msg.drive.jerk
        self.steering_velocity.publish(ack_msg)
        # Managing of contol actions sent to controller_node.py
        if next_input:
            self.drive_next_pub.publish(ack_msg)
        else:
            self.drive_pub.publish(ack_msg)

    def publish_tireforce(self, states: list) -> None:
        """Publishes states into /mpc_controller/states"""
        msg = Float64MultiArray()
        msg.data = states
        self.tireforce__pub.publish(msg)

if __name__ == "__main__":
    controller = tireNode()

    controller.viz_loop()

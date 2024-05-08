#!/usr/bin/env python3
from __future__ import annotations

import math
import time
import numpy as np

from numpy.typing import NDArray
from typing import List, Dict, Tuple, Optional

from filterpy.common import Q_discrete_white_noise
from filterpy.kalman import ExtendedKalmanFilter as EKF
from scipy.linalg import block_diag

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, FloatingPointRange, IntegerRange, SetParametersResult
from rclpy.parameter import Parameter

from std_msgs.msg import Float32
from builtin_interfaces.msg import Time
from f110_msgs.msg import Wpnt, WpntArray, ObstacleArray, Obstacle
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray

from frenet_conversion.frenet_converter import FrenetConverter
from tf_transformations import euler_from_quaternion

def normalize_s(s: float, track_length: float) -> float:
    """Normalizes S coordinate around the track length.
    """
    new_s = s % (track_length)
    if new_s > track_length/2:
        new_s -= track_length

    return new_s


class Opponent_state:
    """
    This class implements the opponent with a kalman filter
    """
    track_length = None
    waypoints = None
    rate = None #hz
    dt = None
    ttl = None
    P_vs = None
    P_d = None
    P_vd = None
    measurment_var_s = None
    measurment_var_d = None
    measurment_var_vs = None
    measurment_var_vd = None
    process_var_vs = None
    process_var_vd = None
    ratio_to_glob_path = None
    def __init__(self):
        """
        Initializes the kalman filter
        """
        self.id = None
        self.size = None
        self.isInitialised = False
        self.vs_list = []
        self.avg_vs = 0
        self.useTargetVel = False

        #----------------------------------------------------------------
        # --- initialization of the kalman filter ---
        # the state of the kalman filter is :
        # X = [s, v_s, d, v_d]
        #
        # the measurment of the kalman filter is :
        # Z = [s, d]
        #
        # the model of the kalman filter is constant velocity in the 
        # frenet frame added to that:
        # - A target velocity for the s direction to be proportional to 
        # the precaculated velocity of the path 
        # - A target velocity in the d direction that bring it back to 0
        # - A target position in the d direction that bring it back to 0
        #---------------------------------------------------------------

        self.dynamic_kf = EKF(dim_x=4, dim_z=2)
        self.dynamic_kf.F = np.array([[1., Opponent_state.dt, 0., 0.],
                                      [0., 1, 0, 0.],
                                      [0., 0., 1, Opponent_state.dt],
                                      [0., 0., 0., 1]])
        q1 =Q_discrete_white_noise(dim= 2, dt= 1./Opponent_state.rate ,var=Opponent_state.process_var_vs)
        q2 =Q_discrete_white_noise(dim= 2, dt=1./Opponent_state.rate, var=Opponent_state.process_var_vd)        
        self.dynamic_kf.Q = block_diag(q1, q2)
        self.dynamic_kf.H = np.identity(4)
        self.dynamic_kf.R = np.diag([Opponent_state.measurment_var_s, Opponent_state.measurment_var_vs, 
                                     Opponent_state.measurment_var_d, Opponent_state.measurment_var_vd])
        self.dynamic_kf.P = np.array([[Opponent_state.measurment_var_s, 0., 0., 0.],
                                      [0., Opponent_state.process_var_vs, 0., 0.],
                                      [0., 0., Opponent_state.measurment_var_d, 0.],
                                      [0., 0., 0., Opponent_state.process_var_vd]])
        self.dynamic_kf.B = np.identity(4)

        filter_length = 5
        self.vs_filt = np.zeros(filter_length)
        self.vd_filt = np.zeros(filter_length)

    # --- defining some utility functions ---
    def residual_h(a,b):
        y = a-b
        y[0] = normalize_s(y[0],Opponent_state.track_length)
        return y
    
    def Hjac(self):
        return np.identity(4)

    def hx(x):
        return np.array([normalize_s(x[0],
                         Opponent_state.track_length),x[1], x[2], x[3]])

    def target_velocity(self) :
        idx_closest_waypoint =  int((self.dynamic_kf.x[0]*10)%Opponent_state.track_length)
        return Opponent_state.ratio_to_glob_path*Opponent_state.waypoints[idx_closest_waypoint].vx_mps

    # ---------------------------------------
    #     defining the predict and update 
    #     functions for the kalman filter 
    # ---------------------------------------

    def predict (self):
        if self.useTargetVel:
            self.dynamic_kf.predict(u=[0,Opponent_state.P_vs*(self.target_velocity()-self.dynamic_kf.x[1]),
                                    -Opponent_state.P_d*self.dynamic_kf.x[2],-Opponent_state.P_vd*self.dynamic_kf.x[3]])
        else:
            self.dynamic_kf.predict(u=[0, 0,
                                   -Opponent_state.P_d*self.dynamic_kf.x[2],-Opponent_state.P_vd*self.dynamic_kf.x[3]])
        self.dynamic_kf.x[0] = normalize_s(self.dynamic_kf.x[0],Opponent_state.track_length)

    def update(self, tracked_obstacle: ObstacleSD):

        vs = ((2/3 * (tracked_obstacle.measurments_s[-1] - tracked_obstacle.measurments_s[-2])*self.rate) 
              + (1/3 * (tracked_obstacle.measurments_s[-2] - tracked_obstacle.measurments_s[-3])*self.rate))

        if not (vs > -1 and vs < 8):
            self.isInitialised = False
            return

        z = np.array([
            tracked_obstacle.measurments_s[-1],
            vs,
            tracked_obstacle.measurments_d[-1],
            (tracked_obstacle.measurments_d[-1] - tracked_obstacle.measurments_d[-2])*self.rate,
        ])

        self.dynamic_kf.update(
            np.array([
                normalize_s(z[0],Opponent_state.track_length), z[1], z[2], z[3]
            ]),
            Opponent_state.Hjac,
            Opponent_state.hx, 
            residual=Opponent_state.residual_h
        )
        self.dynamic_kf.x[0] = normalize_s(self.dynamic_kf.x[0],Opponent_state.track_length)

        self.vs_list.append(self.dynamic_kf.x[1])
        if(len(self.vs_list) > 20):
            self.vs_list = self.vs_list[-10:]

        self.avg_vs = 0
        for v in self.vs_list:
            self.avg_vs += v
        self.avg_vs /= len(self.vs_list)

        self.vs_filt[0] = self.dynamic_kf.x[1]
        self.vs_filt[1:] = self.vs_filt[:-1]  
        self.vd_filt[0] = self.dynamic_kf.x[3]
        self.vd_filt[1:] = self.vd_filt[:-1]

        if(len(self.vs_list) >=10):
            self.vs_list.pop(0)
        self.vs_list.append(self.dynamic_kf.x[1])

# ---------------------------------------------------------------- 
# create a class to define what are the properties of an obstacle
# ----------------------------------------------------------------
class ObstacleSD:
    """
    This Class implements the propoerties of the static/dynamic obstacles
    """
    min_nb_meas = None
    ttl = None
    min_std = None
    max_std = None

    def __init__(self, id, s_meas, d_meas, lap, size, isVisible):
        """
        Initialize the static/dynamic obstacle
        """
        # --- variable --- 
        self.id = id
        self.measurments_s = [s_meas]
        self.measurments_d = [d_meas]
        self.mean = [s_meas,d_meas] # [mean_s. mean_d] 
        self.static_count = 0
        self.total_count = 0
        self.nb_meas = 0
        self.ttl = ObstacleSD.ttl
        self.isInFront = True
        self.current_lap = lap
        self.staticFlag = None
        self.size = size
        self.nb_detection = 0
        self.isVisible = isVisible

    def update_mean(self,track_length):
        if (self.nb_meas == 0):
            self.mean = [self.measurments_s[-1],self.measurments_d[-1]]
        else:
            # ------------------------------------------------------------------------------------
            # since we know the number of measurments and the previous mean, to not loop 
            # through all the data the current mean is just a weighted sum between the 
            # previous mean weighted by the number of measurments and the new measurment 
            # ------------------------------------------------------------------------------------
            
            self.mean[1] = (self.mean[1]*self.nb_meas+self.measurments_d[-1])/(self.nb_meas+1)

            # ------------------------------------------------------------------------------------
            # to account for the wrapping in the process we can transform the s measurments into
            # angles ranging from 0 to 2 pi and do the weighted sum over unitary vectors with those 
            # angles and then convert the resulting angle back to an s measurment 
            # ------------------------------------------------------------------------------------

            previous_mean_rad = self.mean[0]*2*math.pi/track_length
            current_meas_rad = self.measurments_s[-1]*2*math.pi/track_length
            cos_mean_angle = (math.cos(previous_mean_rad)*self.nb_meas+math.cos(current_meas_rad))/(self.nb_meas+1)
            sin_mean_angle = (math.sin(previous_mean_rad)*self.nb_meas+math.sin(current_meas_rad))/(self.nb_meas+1)
            mean_angle = math.atan2(sin_mean_angle,cos_mean_angle)
            mean_s = mean_angle*track_length/2/math.pi
            self.mean[0] = mean_s if mean_s>=0 else mean_s+track_length

    def std_s (self,track_length):
        sum = 0
        mean_s = self.mean[0]
        for s in self.measurments_s:
            sum += normalize_s(s-mean_s,track_length)**2
        return (sum/len(self.measurments_s))**0.5

    def std_d (self):
        return np.std(self.measurments_d)

    def isStatic(self,track_length):
        # --- get a representative data set for the obstacle ---
        if self.nb_meas > ObstacleSD.min_nb_meas:
            std_s = self.std_s(track_length)
            std_d = self.std_d()
            # --- create a voting system so that the outliers don't affect much the result ---
            if (std_s < ObstacleSD.min_std and std_d < ObstacleSD.min_std):
                self.static_count = self.static_count +1
            # --- assert for sure that an obstacle is dynamic and not static ---
            elif (std_s > ObstacleSD.max_std or std_d > ObstacleSD.max_std):
                self.static_count = 0
            self.total_count = self.total_count +1
            self.staticFlag = self.static_count/self.total_count >= 0.5
        else:
            self.staticFlag = None

class StaticDynamic(Node):
    """
    This class implements a ROS node that classifies an publishes obstacles.

    It subscribes to the following topics:
        - `/raw_obstacles`: the raw obstacle data.
        - `/global_waypoints`: the global waypoints.
        - `/odom_frenet`: the car state in frenet frame.
        - `/odom`: the car state in glob frame.
        - `/scan`: the lidar scans.

    It publishes the following topics:
        - `/static_dynamic_marker_pub`: Publishes the obstacle markers
        - `/obstacles`: Publishes the obstacles
        - `/raw_obstacles`: Publishes the obstacles without Kalman Filtering
    """

    def __init__(self):
        """
        Initialize the node, subscribe to topics, and create publishers and service proxies.
        """
        super().__init__('tracking',
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)  # type: ignore

        # ------------ Tunable Parameters ------------
        self.declare_parameter("rate", 40, descriptor=ParameterDescriptor(
            description="rate at which the node is running"))
        self.declare_parameter("P_vs", 0.2, descriptor=ParameterDescriptor(
            description="proportional gain for the vs"))
        self.declare_parameter("P_d", 0.02, descriptor=ParameterDescriptor(
            description="proportional gain for the d"))
        self.declare_parameter("P_vd", 0.2, descriptor=ParameterDescriptor(
            description="proportional gain for the vd"))
        self.declare_parameter("measurment_var_s", 0.002, descriptor=ParameterDescriptor(
            description="the variance of the measurment noise in the s direction"))
        self.declare_parameter("measurment_var_d", 0.002, descriptor=ParameterDescriptor(
            description="the variance of the measurment noise in the d direction"))
        self.declare_parameter("measurment_var_vs", 0.2, descriptor=ParameterDescriptor(
            description="the variance of the measurment noise in vs"))
        self.declare_parameter("measurment_var_vd", 0.2, descriptor=ParameterDescriptor(
            description="the variance of the measurment noise in vs"))
        self.declare_parameter("process_var_vs", 2, descriptor=ParameterDescriptor(
            description="the variance of the process velocity noise in the s direction"))
        self.declare_parameter("process_var_vd", 8, descriptor=ParameterDescriptor(
            description="the variance of the process velocity noise in the d direction"))
        self.declare_parameter("max_dist", 0.5, descriptor=ParameterDescriptor(
            description="max distance for association"))
        self.declare_parameter("var_pub", 1, descriptor=ParameterDescriptor(
            description="obstacles with bigger variance are not published"))

        # dyn params sub
        Opponent_state.ttl = 40
        Opponent_state.ratio_to_glob_path = 0.6
        ObstacleSD.ttl = 3
        ObstacleSD.min_nb_meas = 5
        ObstacleSD.min_std = 0.16
        ObstacleSD.max_std = 0.2
        self.dist_deletion = 7
        self.dist_infront = 8
        self.vs_reset = 0.1
        self.aggro_multiplier = 2
        self.debug_mode = False
        self.publish_static = True
        self.noMemoryMode = False

        # ------------ Variables ------------
        # === Obstacle Variables
        self.meas_obstacles: List[Obstacle] = []
        '''The latest measured obstacles from detection'''
        self.tracked_obstacles: List[ObstacleSD] = []

        # === Track Bounds Variables
        self._initialized_track_bounds: bool = False
        self.waypoints: NDArray = np.zeros((3, 1))
        '''3xN numpy array with x,y,psi coordinates of waypoints'''
        self.globalpath: List[Wpnt] = []
        '''Global waypoints as a list of Wpnt'''
        self.track_length: float = -1.0
        '''Track length'''

        # === Car State variables
        self.car_s: Optional[float] = None
        '''Current S position of car from state estimation'''
        self.last_car_s: Optional[float] = None
        '''Previous S position of car in prev. loop'''
        self.car_position: NDArray = np.zeros(2)
        '''Car position in cartesian frame as 2-vector [x, y]'''
        self.car_orientation: NDArray = np.array([1., 0.])
        '''Car orientation as [cos(theta), sin(theta)]'''

        # === Perception Input variables
        self.scans: List[float] = []
        '''Range information from the lidar'''
        # scan limit params
        self.scan_max_angle: float = 0.0
        self.scan_min_angle: float = 0.0
        self.scan_increment: float = 0.0

        # === Misc. variables
        self.current_lap: int = 0
        '''Current lap of the car'''
        self.current_stamp: Time = self.get_clock().now().to_msg()
        '''Timestamp of most recent obstacle'''
        self.current_id: int = 1
        '''TODO'''
        self.update_rate: int = self.get_parameter(
            "rate").get_parameter_value().integer_value
        '''Update rate of tracking node'''
        self.get_logger().info(f"Update rate: {self.update_rate}")
        self.max_dist = self.get_parameter(
            "max_dist").get_parameter_value().double_value
        self.var_pub = self.get_parameter(
            "var_pub").get_parameter_value().integer_value

        self.from_bag = self.get_parameter(
            "/from_bag").get_parameter_value().bool_value
        self.measuring = self.get_parameter(
            "/measure").get_parameter_value().bool_value
        
        # Opponent State varibles
        Opponent_state.rate = self.update_rate
        Opponent_state.dt = 1/self.update_rate
        Opponent_state.P_vs = self.get_parameter(
            "P_vs").get_parameter_value().double_value
        Opponent_state.P_d = self.get_parameter(
            "P_d").get_parameter_value().double_value
        Opponent_state.P_vd = self.get_parameter(
            "P_vd").get_parameter_value().double_value
        Opponent_state.measurment_var_s = self.get_parameter(
            "measurment_var_s").get_parameter_value().double_value
        Opponent_state.measurment_var_d = self.get_parameter(
            "measurment_var_d").get_parameter_value().double_value
        Opponent_state.measurment_var_vs = self.get_parameter(
            "measurment_var_vs").get_parameter_value().double_value
        Opponent_state.measurment_var_vd = self.get_parameter(
            "measurment_var_vd").get_parameter_value().double_value
        Opponent_state.process_var_vs = self.get_parameter(
            "process_var_vs").get_parameter_value().integer_value
        Opponent_state.process_var_vd = self.get_parameter(
            "process_var_vd").get_parameter_value().integer_value
        
        
        # dyn params
        param_dicts = [{'name' : 'ttl_dynamic',
                        'default' : Opponent_state.ttl,
                        'descriptor' : ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER, integer_range=[IntegerRange(from_value=5, to_value=300, step=1)])},
                       {'name' : 'ratio_to_glob_path',
                        'default' : Opponent_state.ratio_to_glob_path,
                        'descriptor' : ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE, floating_point_range=[FloatingPointRange(from_value=0.1, to_value=2.0, step=0.1)])},
                       {'name' : 'ttl_static',
                        'default' : ObstacleSD.ttl,
                        'descriptor' : ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER, integer_range=[IntegerRange(from_value=1, to_value=20, step=1)])},
                       {'name' : 'min_nb_meas',
                        'default' : ObstacleSD.min_nb_meas,
                        'descriptor' : ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER, integer_range=[IntegerRange(from_value=3, to_value=10, step=1)])},
                       {'name' : 'min_std',
                        'default' : ObstacleSD.min_std,
                        'descriptor' : ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE, floating_point_range=[FloatingPointRange(from_value=0.05, to_value=0.3, step=0.01)])},
                       {'name' : 'max_std',
                        'default' : ObstacleSD.max_std,
                        'descriptor' : ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE, floating_point_range=[FloatingPointRange(from_value=0.15, to_value=0.5, step=0.01)])},
                       {'name' : 'dist_deletion',
                        'default' : self.dist_deletion,
                        'descriptor' : ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE, floating_point_range=[FloatingPointRange(from_value=3.0, to_value=10.0, step=0.1)])},
                       {'name' : 'dist_infront',
                        'default' : self.dist_infront,
                        'descriptor' : ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE, floating_point_range=[FloatingPointRange(from_value=3.0, to_value=10.0, step=0.1)])},
                       {'name' : 'vs_reset',
                        'default' : self.vs_reset,
                        'descriptor' : ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE, floating_point_range=[FloatingPointRange(from_value=0.0, to_value=2.0, step=0.1)])},
                       {'name' : 'aggro_multi',
                        'default' : self.aggro_multiplier,
                        'descriptor' : ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE, floating_point_range=[FloatingPointRange(from_value=0.01, to_value=0.5, step=0.01)])},
                       {'name' : 'debug_mode',
                        'default' : self.debug_mode,
                        'descriptor' : ParameterDescriptor(type=ParameterType.PARAMETER_BOOL)},
                       {'name' : 'publish_static',
                        'default' : self.publish_static,
                        'descriptor' : ParameterDescriptor(type=ParameterType.PARAMETER_BOOL)},
                       {'name' : 'noMemoryMode',
                        'default' : self.noMemoryMode,
                        'descriptor' : ParameterDescriptor(type=ParameterType.PARAMETER_BOOL)}]
        self.delcare_dyn_parameters(param_dicts)
        self.add_on_set_parameters_callback(self.dyn_param_cb)
        
        self.opponent_obstacle = Opponent_state()

        # ------------ Initialize main logic loop ------------
        self.timer = self.create_timer(1/self.update_rate, self.loop)

        # ------------ Subscribers ------------
        self.obs_sub = self.create_subscription(
            ObstacleArray, '/perception/detection/raw_obstacles', self.obstacleCallback, 10)
        self.glob_wpts_sub = self.create_subscription(
            WpntArray, '/global_waypoints', self.pathCallback, 10)
        self.cs_frenet = self.create_subscription(
            Odometry, '/car_state/frenet/odom', self.carStateCallback, 10)
        self.cs_odom_sub = self.create_subscription(
            Odometry, '/car_state/odom', self.carStateGlobCallback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scansCallback, 10)

        # ------------ Publishers ------------
        self.static_dynamic_marker_pub = self.create_publisher(
            MarkerArray, '/perception/static_dynamic_marker_pub', 5)
        self.estimated_obstacles_pub = self.create_publisher(
            ObstacleArray, '/perception/obstacles', 5)
        self.raw_opponent_pub = self.create_publisher(
            ObstacleArray, '/perception/raw_obstacles', 5)
        if self.measuring:
            self.latency_pub = self.create_publisher(
                Float32, '/perception/tracking/latency', 10)
            
        self.get_logger().info('[Tracking]: Tracking initialized')
            
    def delcare_dyn_parameters(self, param_dicts: List[dict]):
        params = []
        for param_dict in param_dicts:
            param = self.declare_parameter(
                param_dict['name'], param_dict['default'], param_dict['descriptor'])
            params.append(param)
        return params

    def loop(self):
        """Performs the main logic loop.
        """
        if self.track_length is None:
            self.get_logger().info("did not get path yet")
            return
        
        if self.measuring:
            start = time.perf_counter()

        if self.opponent_obstacle.isInitialised:
            self.opponent_obstacle.predict()

        self.update()

        if self.measuring:
            end = time.perf_counter()
            self.latency_pub.publish(end-start)

        self.publishObstacles()
        self.publish_Marker()

    # ------------ Callbacks ------------
    
    def dyn_param_cb(self, params: List[Parameter]):
        """
        Notices the change in the parameters and alters the tracking params accordingly
        """
        for param in params:
            param_name = param.name
            if param_name == 'ttl_dynamic':
                Opponent_state.ttl = param.value
            elif param_name == 'ratio_to_glob_path':
                Opponent_state.ratio_to_glob_path = param.value
            elif param_name == 'ttl_static':
                ObstacleSD.ttl = param.value
            elif param_name == 'min_nb_meas':
                ObstacleSD.min_nb_meas = param.value
            elif param_name == 'min_std':
                ObstacleSD.min_std = param.value
            elif param_name == 'max_std':
                ObstacleSD.max_std = param.value
            elif param_name == 'dist_deletion':
                self.dist_deletion = param.value
            elif param_name == 'dist_infront':
                self.dist_infront = param.value
            elif param_name == 'vs_reset':
                self.vs_reset = param.value
            elif param_name == 'aggro_multi':
                self.aggro_multi = param.value
            elif param_name == 'debug_mode':
                self.debug_mode = param.value
            elif param_name == 'publish_static':
                self.publish_static = param.value
            elif param_name == 'noMemoryMode':
                self.noMemoryMode = param.value
        
        obstacle_params = [ObstacleSD.ttl, ObstacleSD.min_nb_meas, ObstacleSD.min_std, ObstacleSD.max_std]
        self.get_logger().info(
            f'[Tracking] Dynamic reconf triggered new tracking params: Tracking TTL: {Opponent_state.ttl}, Ratio to glob path: {Opponent_state.ratio_to_glob_path},\n'
            f'ObstacleSD ttl, min_nb_meas, min_std, max_std: {obstacle_params},\n'
            f'dist_deletion: {self.dist_deletion} [m], dist_infront: {self.dist_infront} [m], vs_reset: {self.vs_reset},\n'
            f'Publish static obstacles: {self.publish_static}, no memory mode: {self.noMemoryMode}'
              )
        
        return SetParametersResult(successful=True)

    def obstacleCallback(self, data: ObstacleArray) -> None:
        """Populates the meas_obstacles and current_stamp member variables with data from the ObstacleArray.
        """
        self.meas_obstacles = data.obstacles  # type: ignore
        self.current_stamp = data.header.stamp

    def pathCallback(self, data: WpntArray) -> None:
        """Updates the track bounds variables.

            Also initializes the FrenetConverter object.
        """
        if self._initialized_track_bounds:
            return

        self.get_logger().info('[Tracking] received global path')

        self.waypoints = np.array([[wpnt.x_m, wpnt.y_m, wpnt.psi_rad]
                                  for wpnt in data.wpnts])
        self.globalpath = data.wpnts    # type: ignore
        self.track_length = data.wpnts[-1].s_m  # type: ignore

        Opponent_state.track_length = self.track_length
        Opponent_state.waypoints = self.globalpath  # type: ignore

        # Initialize the FrenetConverter object
        self.get_logger().info("[Tracking] initialized FrenetConverter object")
        self.converter = FrenetConverter(
            self.waypoints[:, 0], self.waypoints[:, 1], self.waypoints[:, 2])
        '''Cartesian to Frenet converter that is initialized with the first pathCallback.'''

        self._initialized_track_bounds = True

    def carStateCallback(self, data: Odometry) -> None:
        """Update the car S state."""
        self.car_s = data.pose.pose.position.x
        if self.last_car_s is None:
            self.last_car_s = data.pose.pose.position.x

    def carStateGlobCallback(self, data: Odometry) -> None:
        """Update the car state in Cartesian coordinates."""
        self.car_position = np.array(
            [data.pose.pose.position.x, data.pose.pose.position.y])

        car_yaw = euler_from_quaternion([data.pose.pose.orientation.x,
                                        data.pose.pose.orientation.y,
                                        data.pose.pose.orientation.z,
                                        data.pose.pose.orientation.w])[2]

        self.car_orientation = np.array([np.cos(car_yaw), np.sin(car_yaw)])

    def scansCallback(self, data: LaserScan) -> None:
        '''Get the latest range data from the lidar.'''
        self.scans = data.ranges    # type: ignore
        self.scan_max_angle = data.angle_max
        self.scan_min_angle = data.angle_min
        self.scan_increment = data.angle_increment

    # ------------ Logic Functions ------------
    def update(self):
        """update tracked obstacles, add new obstacles and remove unecessary
        """

        # Don't update if we have not received relevant msgs
        if self.car_s is None:
            return
        if not self._initialized_track_bounds:
            return

        meas_obstacles_copy = self.meas_obstacles.copy()
        car_s_copy = self.car_s
        car_position_copy = np.copy(self.car_position)  # type: ignore
        car_orientation_copy = np.copy(self.car_orientation)  # type: ignore

        # Update the current lap status
        self.lap_update(car_s_copy)

        removal_list = []
        for tracked_obstacle in self.tracked_obstacles:
            # --- verify if the obstacle is tracked by position and update the associated obstacle ---

            meas_obstacle = self.verify_position(
                tracked_obstacle, meas_obstacles_copy)
            is_tracked = meas_obstacle is not None

            if is_tracked:
                tracked_obstacle = self.update_tracked_obstacle(
                    tracked_obstacle, meas_obstacle)

                # obstacle is classified as moving
                if tracked_obstacle.staticFlag == False:
                    if self.opponent_obstacle.isInitialised:
                        self.opponent_obstacle.useTargetVel = False
                        if (self.opponent_obstacle.avg_vs < self.vs_reset and len(self.opponent_obstacle.vs_list) > 10 and self.publish_static):
                            self.opponent_obstacle.isInitialised = False
                            tracked_obstacle.staticFlag = True
                            tracked_obstacle.static_count = 0
                            tracked_obstacle.total_count = 0
                            tracked_obstacle.nb_meas = 0
                        else:
                            self.opponent_obstacle.update(tracked_obstacle)
                            self.opponent_obstacle.id = tracked_obstacle.id
                            self.opponent_obstacle.ttl = Opponent_state.ttl
                            self.opponent_obstacle.size = tracked_obstacle.size
                    else:
                        self.initialize_dynamic_obstacle(tracked_obstacle)

                # This obstacle is assigned. Remove it from further computation.
                meas_obstacles_copy.remove(meas_obstacle)

            else:
                # --- remove obstacle with dead ttl ---
                if tracked_obstacle.ttl <= 0:
                    if (tracked_obstacle.staticFlag == False):
                        self.opponent_obstacle.useTargetVel = True
                    removal_list.append(tracked_obstacle)
                elif tracked_obstacle.staticFlag is None:
                    tracked_obstacle.ttl -= 1
                else:
                    tracked_obstacle.isInFront = self.check_in_front(
                        tracked_obstacle, car_s_copy)
                    distance_obstacle_car = self.calc_distance_obs_car(
                        tracked_obstacle, car_s_copy)

                    if (tracked_obstacle.staticFlag and self.noMemoryMode):
                        tracked_obstacle.ttl -= 1
                    # --- if obstacle is near enough check if we can see it ---
                    elif (distance_obstacle_car < self.dist_deletion and tracked_obstacle.staticFlag):
                        try:
                            resp = self.converter.get_cartesian(
                                tracked_obstacle.mean[0], tracked_obstacle.mean[1])
                        except:
                            continue
                        vec_car_to_obs = resp - car_position_copy
                        if (self.check_in_field_of_view(vec_car_to_obs, car_orientation_copy)):
                            tracked_obstacle.ttl -= 1
                            tracked_obstacle.isVisible = True
                        else:
                            tracked_obstacle.isVisible = False
                    # update ttl of moving obstacles
                    elif (not tracked_obstacle.staticFlag):
                        tracked_obstacle.ttl -= 1
                    else:
                        tracked_obstacle.isVisible = False

        # update ttl of dynamic obstacle
        if (self.opponent_obstacle.isInitialised):
            if (self.opponent_obstacle.ttl <= 0):
                self.opponent_obstacle.isInitialised = False
                self.opponent_obstacle.useTargetVel = False
            else:
                self.opponent_obstacle.ttl -= 1

        for el in removal_list:
            self.tracked_obstacles.remove(el)

        for meas_obstacle in meas_obstacles_copy:
            # update the init function and append a new obstacle to the new _obstacles
            self.tracked_obstacles.append(ObstacleSD(
                id=self.current_id,
                s_meas=meas_obstacle.s_center,
                d_meas=meas_obstacle.d_center,
                lap=self.current_lap,
                size=meas_obstacle.size,
                isVisible=True
            ))
            self.current_id += 1

    # ------------ Utility Functions ------------

    def lap_update(self, car_s: float) -> None:
        """Updates the lap counter by observing the change in the car's S position.

            Increments the current_lap variable if necessary. Also updates the last_car_s variable.

        Args:
            car_s (float): current car S position
        """

        if self.last_car_s is None:
            return

        if car_s - self.last_car_s < -self.track_length/2:
            self.current_lap += 1

        self.last_car_s = car_s

    def get_closest_pos(self, max_dist: float, obstacle_position: Tuple[float, float], meas_obstacles_copy: List[Obstacle]) -> Tuple[List[Obstacle], List[float]]:
        """Among all obstacles in the query list meas_obstacles_copy, return all that are less than max_dist from obstacle_position.

        Args:
            max_dist (float): distance threshold
            obstacle_position (Tuple[float, float]): query obs position
            meas_obstacles_copy (List[Obstacle]): all input obstacles

        Returns:
            List[Obstacle]: List of potential matches
            List[float]]: List of distances of matches

            These lists have the same size.
        """

        potential_obs = []
        dists = []

        for meas_obstacle in meas_obstacles_copy:
            meas_obstacle_position = (meas_obstacle.s_center,
                                      meas_obstacle.d_center)
            dist = math.dist(obstacle_position, meas_obstacle_position)

            if (dist < max_dist):
                potential_obs.append(meas_obstacle)
                dists.append(dist)

        return potential_obs, dists

    def verify_position(self, obstacle: ObstacleSD, meas_obstacles_copy: List[Obstacle]) -> Optional[Obstacle]:
        """
        Among all the measured obstacles, checks if there is at least one that is witin a distance threshold
        from the KF prediction of the tracked obstacle. If so, return that obstacle. Else, return None.
        """

        # for dynamic obstacles we use the predicted position to get better accuracy
        max_dist = self.max_dist

        if obstacle.staticFlag == False:
            obstacle_position = (
                self.opponent_obstacle.dynamic_kf.x[0] % self.track_length,
                self.opponent_obstacle.dynamic_kf.x[2])
            max_dist *= self.aggro_multiplier
        else:
            obstacle_position = obstacle.mean   # s, d position

        potential_obs, dists = self.get_closest_pos(
            max_dist, obstacle_position, meas_obstacles_copy)

        # We found a successful match. Return the most likely elem.
        if (len(dists) > 0):
            min_idx = np.argmin(dists)
            return potential_obs[min_idx]

        # maybe kalman was wrong, the obstacles can't just be gone.
        # Try again with the obstacle mean.
        elif (obstacle.staticFlag == False):
            obstacle_position = obstacle.mean
            potential_obs, dists = self.get_closest_pos(
                max_dist, obstacle_position, meas_obstacles_copy)

            if (len(dists) > 0):
                min_idx = np.argmin(dists)
                return potential_obs[min_idx]

        return None

    def angle_to_obs(self, vec_to_obstacle: np.array, car_orientation: np.array) -> float:
        """Given the vector from car to obstacle in map frame and the car's orientation, return the angle of the obstacle
        in car frame in RADIANS.

        Args:
            vec_to_obstacle (np.array): 2-vector from obstacle to car in map frame
            car_orientation (np.array): 2-vector of np.cos(theta), np.sin(theta) where theta is the car's heading in map frame.

        Returns:
            float: bearing angle of the obstacle in car frame in RADIANS.
        """
        # Rotate the relative vec_
        rot = np.array([[ car_orientation[0], car_orientation[1]],
                        [-car_orientation[1], car_orientation[0]]])
        vec_to_obs_rot = np.dot(rot, vec_to_obstacle)

        return np.arctan2(vec_to_obs_rot[1], vec_to_obs_rot[0])

    def update_tracked_obstacle(self, tracked_obstacle: ObstacleSD, meas_obstacle: Obstacle) -> ObstacleSD:
        """Calls update methods on the tracked obstacle based on the observations from meas_obstacle.

        Args:
            tracked_obstacle (ObstacleSD): input tracked_obstacle
            meas_obstacle (Obstacle): associated measurement obstacle

        Returns:
            ObstacleSD: Updated tracked_obstacle variable
        """

        tracked_obstacle.measurments_s.append(meas_obstacle.s_center)
        tracked_obstacle.measurments_d.append(meas_obstacle.d_center)

        # handle list length
        if len(tracked_obstacle.measurments_s) > 30:
            tracked_obstacle.measurments_s = tracked_obstacle.measurments_s[-20:]
            tracked_obstacle.measurments_d = tracked_obstacle.measurments_d[-20:]

        tracked_obstacle.update_mean(self.track_length)
        tracked_obstacle.nb_meas += 1
        tracked_obstacle.isInFront = True
        tracked_obstacle.isVisible = True
        tracked_obstacle.current_lap = self.current_lap
        tracked_obstacle.size = meas_obstacle.size
        tracked_obstacle.isStatic(self.track_length)
        tracked_obstacle.ttl = ObstacleSD.ttl

        return tracked_obstacle

    def initialize_dynamic_obstacle(self, tracked_obstacle: ObstacleSD):
        """Initializes the opponent_obstacle variable as a Dynamic obstacle.

        Args:
            tracked_obstacle (ObstacleSD): Input tracked obstacle
        """

        self.opponent_obstacle.dynamic_kf.x = np.array([
            tracked_obstacle.measurments_s[-1],
            (tracked_obstacle.measurments_s[-1] -
             tracked_obstacle.measurments_s[-2])*Opponent_state.rate,
            tracked_obstacle.measurments_d[-1],
            (tracked_obstacle.measurments_d[-1] -
             tracked_obstacle.measurments_d[-2])*Opponent_state.rate
        ])

        self.opponent_obstacle.isInitialised = True
        self.opponent_obstacle.id = tracked_obstacle.id
        self.opponent_obstacle.ttl = Opponent_state.ttl
        self.opponent_obstacle.size = tracked_obstacle.size
        self.opponent_obstacle.avg_vs = 0
        self.opponent_obstacle.vs_list = []

    def check_in_front(self, tracked_obstacle: ObstacleSD, car_s: float) -> bool:
        """Check if tracked_obstacle is indeed in front of the current s of the car.
        """

        obj_dist_in_front = normalize_s(
            tracked_obstacle.measurments_s[-1] - car_s,
            self.track_length)

        return 0 < obj_dist_in_front < self.dist_infront

    def calc_distance_obs_car(self, tracked_obstacle: ObstacleSD, car_s: float):
        """Calculates the distance in S to the obstacle car in the S coordinate.
        """
        distance_obs_car = (
            tracked_obstacle.measurments_s[-1] - car_s) % self.track_length

        if self.debug_mode:
            self.get_logger().info("DIST CAR TO OBS")
            self.get_logger().info(f"{car_s=}")
            self.get_logger().info(
                f"obstacle s: {tracked_obstacle.measurments_s[-1]}")
            self.get_logger().info(f"track length: {self.track_length}")
            self.get_logger().info(f"current lap: {self.current_lap}")
            self.get_logger().info(
                f"obstacle lap: {tracked_obstacle.current_lap}")
            self.get_logger().info(f"distance: {distance_obs_car}")
            self.get_logger().info("----------------------------------")

        return distance_obs_car

    def check_in_field_of_view(self, vec_car_to_obs: np.array, car_orientation_copy: np.array) -> bool:
        """
        Checks if an obstacle is in the field of view by checking the corresponding lidar beams.
        """
        dist_to_obs = np.linalg.norm(vec_car_to_obs)
        bearing_angle = self.angle_to_obs(vec_car_to_obs, car_orientation_copy)

        # Check if bearing angle is outside of the visible range.
        if bearing_angle > self.scan_max_angle or bearing_angle < self.scan_min_angle:
            return False

        # Get the scan index at the bearing angle
        obstacle_scan_idx = (bearing_angle - self.scan_min_angle) / self.scan_increment
        obstacle_scan_idx = int(round(obstacle_scan_idx))

        # Catch scan index out of bounds
        largest_scan_idx = len(self.scans)
        if obstacle_scan_idx < 0 or obstacle_scan_idx >= largest_scan_idx:
            return False

        if self.debug_mode:
            print("FIELD OF VIEW:")
            print("index: ", obstacle_scan_idx)
            print("angle: ", np.degres(bearing_angle))
            print("Other lidar scans: ",
                  str([f"{sc:.2f}" for sc in self.scans[obstacle_scan_idx - 10:obstacle_scan_idx + 10]]))
            print("dist: ", dist_to_obs)
            print("----------------------------------")

        # Take a slice of the scan ranges where the obstacle should have been to see if it was obscured.
        low_index = max(0, obstacle_scan_idx - 4)  # bounds checks
        high_index = min(obstacle_scan_idx + 4, largest_scan_idx)
        scan_dists_of_interest = self.scans[low_index:high_index]
        if dist_to_obs < min(scan_dists_of_interest):
            return True

        # Obstacle is obscured, so not in field of view.
        return False

    def clearmarkers(self) -> MarkerArray:
        """Return a Marker that clears all markers."""
        marker = Marker()
        marker.action = Marker.DELETEALL
        return MarkerArray(markers=[marker])

    def publish_Marker(self) -> None:
        """Publishes perception markers for visualization purposes."""

        if not self._initialized_track_bounds:
            return


        markers_array = []

        # Visualize Static obstacles
        for tracked_obstacle in self.tracked_obstacles:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.current_stamp
            marker.id = tracked_obstacle.id
            marker.type = Marker.SPHERE

            if tracked_obstacle.isInFront:
                marker.scale.x = 0.5
                marker.scale.y = 0.5
                marker.scale.z = 0.5
            else:
                marker.scale.x = 0.25
                marker.scale.y = 0.25
                marker.scale.z = 0.25

            marker.color.a = 0.5
            marker.pose.orientation.w = 1.

            if tracked_obstacle.staticFlag is None and self.publish_static:
                marker.type = Marker.SPHERE
                marker.color.g = 0.
                marker.color.r = 1.
                marker.color.b = 1.
                x, y = self.converter.get_cartesian(
                    tracked_obstacle.measurments_s[-1], tracked_obstacle.measurments_d[-1])

                marker.pose.position.x = x
                marker.pose.position.y = y

                markers_array.append(marker)

            elif tracked_obstacle.staticFlag and self.publish_static:
                marker.color.g = 1.
                marker.color.r = 0.
                marker.color.b = 0.

                x, y = self.converter.get_cartesian(
                    tracked_obstacle.mean[0], tracked_obstacle.mean[1])
                marker.pose.position.x = x
                marker.pose.position.y = y

                markers_array.append(marker)

        # Visualize Tracked Opponent
        if self.opponent_obstacle.isInitialised:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.current_stamp
            marker.id = self.opponent_obstacle.id
            marker.type = Marker.SPHERE

            if self.opponent_obstacle.dynamic_kf.P[0][0] < self.var_pub:
                marker.scale.x = 0.5
                marker.scale.y = 0.5
                marker.scale.z = 0.5
            else:
                marker.scale.x = 0.25
                marker.scale.y = 0.25
                marker.scale.z = 0.25

            marker.color.a = 0.5
            marker.color.g = 0.
            marker.color.r = 1.
            marker.color.b = 0.

            x, y = self.converter.get_cartesian(
                self.opponent_obstacle.dynamic_kf.x[0] % self.track_length, self.opponent_obstacle.dynamic_kf.x[2])
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.orientation.w = 1.
            markers_array.append(marker)

        markers_array = MarkerArray(markers=markers_array)

        # Clear all markers and publish new ones
        self.static_dynamic_marker_pub.publish(self.clearmarkers())
        self.static_dynamic_marker_pub.publish(markers_array)

    def publishObstacles(self):
        """Publishes all collated obstacles
        """

        obstaclearray_temp = ObstacleArray()
        obstaclearray_temp.header.frame_id = 'map'
        obstaclearray_temp.header.stamp = self.current_stamp
        obstacle_array = []
        raw_opponent_array = []

        for obs in self.tracked_obstacles:
            obs_msg = Obstacle()

            obs_msg.id = obs.id
            obs_msg.size = obs.size
            obs_msg.vs = 0.
            obs_msg.vd = 0.
            obs_msg.is_static = True
            obs_msg.is_actually_a_gap = False
            obs_msg.is_visible = obs.isVisible

            if obs.staticFlag is None:
                obs_msg.s_center = obs.measurments_s[-1] % self.track_length
                obs_msg.d_center = obs.measurments_d[-1]
            elif obs.staticFlag:
                obs_msg.s_center = obs.mean[0]
                obs_msg.d_center = obs.mean[1]
            else:
                obs_msg.s_center = obs.measurments_s[-1] % self.track_length
                obs_msg.d_center = obs.measurments_d[-1]

            obs_msg.s_start = obs_msg.s_center-obs_msg.size/2 % self.track_length
            obs_msg.s_end = obs_msg.s_center+obs_msg.size/2 % self.track_length
            obs_msg.d_right = obs_msg.d_center-obs_msg.size/2
            obs_msg.d_left = obs_msg.d_center+obs_msg.size/2

            if obs.staticFlag is None and self.publish_static:
                obstacle_array.append(obs_msg)
            elif obs.staticFlag and self.publish_static:
                obstacle_array.append(obs_msg)
            else:
                raw_opponent_array.append(obs_msg)
        if self.opponent_obstacle.isInitialised:
            if self.opponent_obstacle.dynamic_kf.P[0][0] < self.var_pub:
                obs_msg = Obstacle()

                obs_msg.id = self.opponent_obstacle.id
                obs_msg.size = self.opponent_obstacle.size
                obs_msg.vs = np.mean(self.opponent_obstacle.vs_filt)
                obs_msg.vd = np.mean(self.opponent_obstacle.vd_filt)
                obs_msg.is_static = False
                obs_msg.is_actually_a_gap = False
                obs_msg.is_visible = True
                obs_msg.s_center = self.opponent_obstacle.dynamic_kf.x[0] % self.track_length
                obs_msg.d_center = self.opponent_obstacle.dynamic_kf.x[2]
                obs_msg.s_start = obs_msg.s_center-obs_msg.size/2 % self.track_length
                obs_msg.s_end = obs_msg.s_center+obs_msg.size/2 % self.track_length
                obs_msg.d_right = obs_msg.d_center-obs_msg.size/2
                obs_msg.d_left = obs_msg.d_center+obs_msg.size/2

                obstacle_array.append(obs_msg)
        obstaclearray_temp.obstacles = obstacle_array
        self.estimated_obstacles_pub.publish(obstaclearray_temp)
        obstaclearray_temp.obstacles = raw_opponent_array
        self.raw_opponent_pub.publish(obstaclearray_temp)

def main():
    rclpy.init()
    tracker = StaticDynamic()
    rclpy.spin(tracker)
    tracker.destroy_node()
    rclpy.shutdown()

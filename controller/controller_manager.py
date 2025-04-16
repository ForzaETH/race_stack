#!/usr/bin/env python3

import threading
import time

import numpy as np
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from dynamic_reconfigure.msg import Config
from dynamic_reconfigure.client import Client
from f110_msgs.msg import ObstacleArray, PidData, WpntArray
from sensor_msgs.msg import LaserScan
from frenet_converter.frenet_converter import FrenetConverter
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64, String, Float32, Float64MultiArray
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray
from map.src.MAP_Controller import MAP_Controller
from pp.src.PP_Controller import PP_Controller
from ftg.ftg import FTG
from single_track_mpc import Single_track_MPC_Controller
from kinematic_mpc import Kinematic_MPC_Controller
from pbl_config import (CarConfig, KMPCConfig, PacejkaTireConfig, STMPCConfig,
                        load_car_config_ros, load_KMPC_config_ros,
                        load_pacejka_tire_config_ros, load_STMPC_config_ros,
                        load_trailing_config_ros, TrailingConfig)
#TODO kmpc

class Controller_manager:
    """This class is the main controller manager for the car. It is responsible for selecting the correct controller $
    and publishing the corresponding commands to the actuators.
    
    It subscribes to the following topics:
    - /car_state/odom:  get ego car speed
    - /car_state/pose:  get ego car position (x, y, theta)
    - /local_waypoints: get waypoints starting at car's position in map frame
    - /vesc/sensors/imu/raw: get acceleration for steer scaling
    - /car_state/odom_frenet: get ego car frenet coordinates
    - /perception/obstacles: get opponent information (position, speed, static/dynamic)
    - /state_machine: get state of the car
    - /scan: get lidar scan data

    It publishes the following topics:
    - /lookahead_point: publish the lookahead point for visualization
    - /trailing_opponent_marker: publish the trailing opponent marker for visualization
    - /my_waypoints: publish the waypoints for visualization
    - /l1_distance: publish the l1 distance from the MAP Controller for visualization
    - /trailing/gap_data: publish the PID data of the Trailing controller for tuning if flag1 is True
    - /vesc/high_level/ackermann_cmd_mux/input/nav_1: publish the steering and speed command
    - /controller/latency: publish the latency of the controller for measuring if launched with measure:=true

    """
    def __init__(self):
        self.name = "control_node"
        rospy.init_node(self.name, anonymous=True)
        self.lock = threading.Lock()
        self.loop_rate = 40 # rate in hertz
        self.ros_time = rospy.Time()
        self.scan = None
        
        self.mapping = rospy.get_param('controller_manager/mapping', False)
        if self.mapping:
            self.init_mapping()
        else:
            self.init_controller()


    def init_controller(self):
        self.racecar_version = rospy.get_param('/racecar_version') # NUCX
        self.car_config: CarConfig = load_car_config_ros(self.racecar_version)
        self.LUT_name = rospy.get_param('controller_manager/LU_table') # name of lookup table
        self.ctrl_algo = rospy.get_param('controller_manager/ctrl_algo', 'MAP') # default controller
        self.l1_params = rospy.get_param('L1_controller')
        self.use_sim = rospy.get_param('/sim')
        self.wheelbase = self.car_config.lr + self.car_config.lf
        rospy.loginfo(f"[{self.name}] Using {self.LUT_name}")
        self.measuring = rospy.get_param('/measure', False)
        self.trailing_config: TrailingConfig = load_trailing_config_ros(self.racecar_version)
        
        self.state_machine_rate = rospy.get_param('state_machine/rate') #rate in hertz
        self.position_in_map = [] # current position in map frame
        self.position_in_map_frenet = [] # current position in frenet coordinates
        self.waypoint_list_in_map = [] # waypoints starting at car's position in map frame
        self.speed_now = 0 # current speed
        self.alpha = 0 # current yaw angle with respect to the race line
        self.yaw_rate = 0 # current yaw rate
        self.acc_now = np.zeros(5) # last 5 accleration values
        self.waypoint_safety_counter = 0

        # Trailing related variables
        self.opponent = [0,0,0,False, True] #s, d, vs, is_static
        self.state = ""
        self.gap_actual = None
        self.trailing_command = 2
        self.i_gap = 0

        self.converter = None

        # initializing l1 parameter
        # This step could be removed with rospy.wait_for_message() in control loop
        self.t_clip_min = 1
        self.t_clip_max = 5
        self.m_l1 = 0.3
        self.q_l1 = 0.15
        self.speed_lookahead = 0
        self.lat_err_coeff = 1
        self.acc_scaler_for_steer = 1.0
        self.dec_scaler_for_steer = 1.0 
        self.start_scale_speed = 7.0
        self.end_scale_speed = 8.0
        self.downscale_factor = 0.2
        self.speed_lookahead_for_steer = 0 

        self.prioritize_dyn = True

        self.trailing_gap = 1.5 
        self.trailing_p_gain = 0.5
        self.trailing_i_gain = 0.001
        self.trailing_d_gain = 0.2
        self.blind_trailing_speed = 1.5
        
        # buffers for improved computation
        self.waypoint_array_buf = MarkerArray()
        self.markers_buf = [Marker() for _ in range(1000)]

        # Parameters
        for i in range(5):
            # waiting for this message twice, as the republisher needs it first to compute the wanted param
            waypoints = rospy.wait_for_message('/global_waypoints', WpntArray)
        self.waypoints = np.array([[wpnt.x_m, wpnt.y_m] for wpnt in waypoints.wpnts])

        self.track_length = rospy.get_param("/global_republisher/track_length")


        # FTG
        self.ftg_controller = FTG()
        #  initialize MAP controller
        self.map_controller = MAP_Controller(
            self.t_clip_min, 
            self.t_clip_max, 
            self.m_l1, 
            self.q_l1, 
            self.speed_lookahead, 
            self.lat_err_coeff, 
            self.acc_scaler_for_steer, 
            self.dec_scaler_for_steer, 
            self.start_scale_speed, 
            self.end_scale_speed, 
            self.downscale_factor, 
            self.speed_lookahead_for_steer,

            self.prioritize_dyn,
            self.trailing_gap,
            self.trailing_p_gain,
            self.trailing_i_gain,
            self.trailing_d_gain,
            self.blind_trailing_speed,

            self.loop_rate,
            self.LUT_name,
            self.state_machine_rate,
            
            logger_info=rospy.loginfo,
            logger_warn=rospy.logwarn
        )

        #  initialize PP controller
        self.pp_controller = PP_Controller(
            self.t_clip_min, 
            self.t_clip_max, 
            self.m_l1, 
            self.q_l1, 
            self.speed_lookahead, 
            self.lat_err_coeff, 
            self.acc_scaler_for_steer, 
            self.dec_scaler_for_steer, 
            self.start_scale_speed, 
            self.end_scale_speed, 
            self.downscale_factor, 
            self.speed_lookahead_for_steer,

            self.prioritize_dyn,
            self.trailing_gap,
            self.trailing_p_gain,
            self.trailing_i_gain,
            self.trailing_d_gain,
            self.blind_trailing_speed,

            self.loop_rate,
            self.wheelbase,
            self.state_machine_rate,
            
            logger_info=rospy.loginfo,
            logger_warn=rospy.logwarn
        )
        
        if self.ctrl_algo == "STMPC":
            # configs
            odom_msg = rospy.wait_for_message('/car_state/odom_frenet', Odometry) # TODO rethink init to avoid these things
            self.car_state_frenet_cb(odom_msg)
            
            self.stmpc_config: STMPCConfig = load_STMPC_config_ros(self.racecar_version)
            self.floor = "dubi"
            self.tire_config: PacejkaTireConfig = load_pacejka_tire_config_ros(
                self.racecar_version, self.floor)
            # dyn reconfigure client
            # state
            pose_frenet = [self.position_in_map_frenet[0], self.position_in_map_frenet[1], self.alpha]
            rospy.loginfo(f"[{self.name}] Initializing Single Track MPC Controller")
            self.stmpc_controller = Single_track_MPC_Controller(pose_frenet=pose_frenet,
                                                                racecar_version=self.racecar_version,
                                                                stmpc_config=self.stmpc_config,
                                                                car_config=self.car_config,
                                                                tire_config=self.tire_config,
                                                                trailing_config=self.trailing_config,
                                                                controller_frequency=self.loop_rate,
                                                                using_gokart=False)
            self.mpc_dyn_rec_client = Client("/mpc_param_tuner", config_callback=self.stmpc_config_cb)
        elif self.ctrl_algo == "KMPC":
            self.kmpc_config: KMPCConfig = load_KMPC_config_ros(self.racecar_version)
            # dyn reconfigure client
            self.kmpc_controller = Kinematic_MPC_Controller(self.racecar_version, self.kmpc_config, self.car_config, self.trailing_config)
            self.mpc_dyn_rec_client = Client("/mpc_param_tuner", config_callback=self.kmpc_config_cb)
            self.compute_time = 0  # init mpc compute time


        # Publishers to view data
        self.lookahead_pub = rospy.Publisher('lookahead_point', Marker, queue_size=10)
        self.trailing_pub = rospy.Publisher('trailing_opponent_marker', Marker, queue_size=10)
        self.waypoint_pub = rospy.Publisher('my_waypoints', MarkerArray, queue_size=10)
        self.l1_pub = rospy.Publisher('l1_distance', Point, queue_size=10)
        self.gap_data = rospy.Publisher('/trailing/gap_data', PidData, queue_size=10)
        self.mpc_states_pub = rospy.Publisher("/mpc_controller/states", Float64MultiArray, queue_size=10)

        # Publisher for steering and speed command
        self.publish_topic = '/vesc/high_level/ackermann_cmd_mux/input/nav_1'
        self.drive_pub = rospy.Publisher(self.publish_topic, AckermannDriveStamped, queue_size=10)
        if self.measuring:
            self.measure_pub = rospy.Publisher('/controller/latency', Float32, queue_size=10)

        
        # Subscribers
        rospy.Subscriber('/car_state/odom', Odometry, self.odom_cb) # car speed
        rospy.Subscriber('/car_state/pose', PoseStamped, self.car_state_cb) # car position (x, y, theta)
        rospy.Subscriber('/local_waypoints', WpntArray, self.local_waypoint_cb) # waypoints (x, y, v, norm trackbound, s, kappa)
        rospy.Subscriber('/vesc/sensors/imu/raw', Imu, self.imu_cb) # acceleration subscriber for steer change
        rospy.Subscriber('/car_state/odom_frenet', Odometry, self.car_state_frenet_cb) # car frenet coordinates
        rospy.Subscriber("/l1_param_tuner/parameter_updates", Config, self.l1_params_cb) #l1 param tuning/updating
        rospy.Subscriber("/perception/obstacles", ObstacleArray, self.obstacle_cb)
        rospy.Subscriber("/state_machine", String, self.state_cb)
        rospy.Subscriber("/scan", LaserScan, self.scan_cb)

        self.converter = FrenetConverter(self.waypoints[:, 0], self.waypoints[:, 1])
        rospy.loginfo(f"[{self.name}] initialized FrenetConverter object")
        
    def init_mapping(self):
        rospy.logwarn(f"[{self.name}] Initializing for mapping")
        # Use FTG for mapping
        self.ftg_controller = FTG(mapping=False)
        
        # Publisher
        self.publish_topic = '/vesc/high_level/ackermann_cmd_mux/input/nav_1'
        self.drive_pub = rospy.Publisher(self.publish_topic, AckermannDriveStamped, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('/car_state/odom', Odometry, self.odom_mapping_cb) # car speed
        rospy.Subscriber("/scan", LaserScan, self.scan_cb)
        
        
        rospy.loginfo(f"[{self.name}] initialized for mapping")

    ############################################CALLBACKS############################################
    def scan_cb(self, data: LaserScan):
        self.scan = data

    def obstacle_cb(self, data:ObstacleArray):
        if len(data.obstacles) > 0 and len(self.position_in_map_frenet):
            self.opponent_s = None
            static_flag = False # If we have a Static and a Dynamic obstacle we prefer the dynamic
            closest_opp = self.track_length
            for obstacle in data.obstacles: 
                opponent_dist = (obstacle.s_start - self.position_in_map_frenet[0]) % self.track_length
                if opponent_dist < closest_opp or (static_flag and not obstacle.is_static): 
                    closest_opp = opponent_dist
                    opponent_static = obstacle.is_static
                    opponent_s = obstacle.s_center
                    opponent_d = obstacle.d_center
                    opponent_vs = obstacle.vs
                    opponent_visible = obstacle.is_visible
                    if opponent_static:
                        static_flag = self.prioritize_dyn # Chosen Obstacle is static
                    else: 
                        static_flag = False # Chosen obstacle is dynamic
                    self.opponent = [opponent_s, opponent_d, opponent_vs, opponent_static, opponent_visible]
        else:
            self.opponent = None

    def state_cb(self, data):
        self.state = data.data
          
    def l1_params_cb(self, params:Config):
        """
        Here the l1 parameters are updated if changed with rqt (dyn reconfigure)
        Values from .yaml file are set in l1_params_server.py      
        """
        
        ## Updating params for map and pp controller
        ## Lateral Control Parameters
        self.map_controller.t_clip_min = params.doubles[0].value
        self.map_controller.t_clip_max = params.doubles[1].value   
        self.map_controller.m_l1 = params.doubles[2].value
        self.map_controller.q_l1 = params.doubles[3].value
        self.map_controller.speed_lookahead = params.doubles[4].value
        self.map_controller.lat_err_coeff = params.doubles[5].value
        self.map_controller.acc_scaler_for_steer = params.doubles[6].value
        self.map_controller.dec_scaler_for_steer = params.doubles[7].value
        self.map_controller.start_scale_speed = params.doubles[8].value
        self.map_controller.end_scale_speed = params.doubles[9].value
        self.map_controller.downscale_factor = params.doubles[10].value
        self.map_controller.speed_lookahead_for_steer = params.doubles[11].value

        self.pp_controller.t_clip_min = params.doubles[0].value
        self.pp_controller.t_clip_max = params.doubles[1].value   
        self.pp_controller.m_l1 = params.doubles[2].value
        self.pp_controller.q_l1 = params.doubles[3].value
        self.pp_controller.speed_lookahead = params.doubles[4].value
        self.pp_controller.lat_err_coeff = params.doubles[5].value
        self.pp_controller.acc_scaler_for_steer = params.doubles[6].value
        self.pp_controller.dec_scaler_for_steer = params.doubles[7].value
        self.pp_controller.start_scale_speed = params.doubles[8].value
        self.pp_controller.end_scale_speed = params.doubles[9].value
        self.pp_controller.downscale_factor = params.doubles[10].value
        self.pp_controller.speed_lookahead_for_steer = params.doubles[11].value
        ## Trailing Control Parameters
        self.map_controller.prioritize_dyn = params.bools[0].value #True, prioritize dynamic obstacles always
        self.map_controller.trailing_gap = params.doubles[12].value # Distance in meters
        self.map_controller.trailing_p_gain = params.doubles[13].value
        self.map_controller.trailing_i_gain = params.doubles[14].value
        self.map_controller.trailing_d_gain = params.doubles[15].value
        self.map_controller.blind_trailing_speed = params.doubles[16].value

        self.pp_controller.prioritize_dyn = params.bools[0].value #True, prioritize dynamic obstacles always
        self.pp_controller.trailing_gap = params.doubles[12].value # Distance in meters
        self.pp_controller.trailing_p_gain = params.doubles[13].value
        self.pp_controller.trailing_i_gain = params.doubles[14].value
        self.pp_controller.trailing_d_gain = params.doubles[15].value
        self.pp_controller.blind_trailing_speed = params.doubles[16].value

    def odom_mapping_cb(self, data: Odometry):
        # velocity for follow the gap (needed to set gap radius)
        self.ftg_controller.set_vel(data.twist.twist.linear.x)

    def odom_cb(self, data: Odometry):
        self.vel_y = data.twist.twist.linear.y
        self.yaw_rate = data.twist.twist.angular.z
        self.speed_now = data.twist.twist.linear.x
        self.map_controller.speed_now = self.speed_now
        self.pp_controller.speed_now = self.speed_now
        
        # velocity for follow the gap (needed to set gap radius)
        self.ftg_controller.set_vel(data.twist.twist.linear.x)

    def car_state_cb(self, data: PoseStamped):
        x = data.pose.position.x
        y = data.pose.position.y
        theta = euler_from_quaternion([data.pose.orientation.x, data.pose.orientation.y, 
                                       data.pose.orientation.z, data.pose.orientation.w])[2]
        self.position_in_map = np.array([x, y, theta])[np.newaxis]

    # This function is currently not used
    def car_state_frenet_cb(self, data: Odometry):
        s = data.pose.pose.position.x
        d = data.pose.pose.position.y
        vs = data.twist.twist.linear.x
        vd = data.twist.twist.linear.y
        self.position_in_map_frenet = np.array([s,d,vs,vd])
        self.alpha = data.pose.pose.orientation.z

    def local_waypoint_cb(self, data: WpntArray):
        self.waypoint_list_in_map = []
        for waypoint in data.wpnts:
            waypoint_in_map = [waypoint.x_m, waypoint.y_m]
            speed = waypoint.vx_mps
            if waypoint.d_right + waypoint.d_left != 0:
                self.waypoint_list_in_map.append([waypoint_in_map[0],
                                                  waypoint_in_map[1], 
                                                  speed, 
                                                  min(waypoint.d_left, waypoint.d_right)/(waypoint.d_right + waypoint.d_left), 
                                                  waypoint.s_m, waypoint.kappa_radpm, waypoint.psi_rad, waypoint.ax_mps2]
                                                )
            else:
                self.waypoint_list_in_map.append([waypoint_in_map[0], waypoint_in_map[1], speed, 0, waypoint.s_m, waypoint.kappa_radpm, waypoint.psi_rad, waypoint.ax_mps2])
        self.waypoint_array_in_map = np.array(self.waypoint_list_in_map)
        self.waypoint_safety_counter = 0

    def imu_cb(self, data):
        self.acc_now[1:] = self.acc_now[:-1]
        self.acc_now[0] = -data.linear_acceleration.y # vesc is rotated 90 deg, so (-acc_y) == (long_acc)

    def stmpc_config_cb(self, params: Config):
        """
        Here the mpc parameters are updated if changed with rqt (dyn reconfigure)
        Values from .yaml file are set in mpc_online_params_server
        """
        for k, v in params.items():
            if k != "groups":
                setattr(self.stmpc_config, k, v)

        self.stmpc_controller.stmpc_config = self.stmpc_config

    def kmpc_config_cb(self, params: Config):
        """
        Here the mpc parameters are updated if changed with rqt (dyn reconfigure)
        Values from .yaml file are set in mpc_online_params_server
        """
        for k, v in params.items():
            if k != "groups":
                setattr(self.kmpc_config, k, v)

        self.kmpc_controller.kmpc_config = self.kmpc_config
    
    ############################################MAIN LOOP############################################

    def control_loop(self):
        rate = rospy.Rate(self.loop_rate)  
        
        if self.mapping:
            self.mapping_loop(rate)
        else:
            self.controller_loop(rate)
    
    def mapping_loop(self, rate: rospy.Rate):
        rospy.wait_for_message('/scan', LaserScan)
        rospy.wait_for_message('/car_state/odom', Odometry)
        rospy.loginfo(f"[{self.name}] Ready for mapping!")
        
        while not rospy.is_shutdown():
            speed, acceleration, jerk, steering_angle = 0, 0, 0, 0
            speed, steering_angle = self.ftg_controller.process_lidar(self.scan.ranges)
            ack_msg = self.create_ack_msg(speed, acceleration, jerk, steering_angle)
            self.drive_pub.publish(ack_msg)
            rate.sleep()
    
    def controller_loop(self, rate: rospy.Rate):
        rospy.loginfo(f"[{self.name}] Waiting for local Waypoints")
        rospy.wait_for_message('/local_waypoints', WpntArray)
        rospy.wait_for_message('/global_waypoints', WpntArray)
        rospy.wait_for_message('/car_state/odom', Odometry)
        rospy.wait_for_service("convert_glob2frenet_service")
        rospy.loginfo(f"[{self.name}] Local Waypoints received")
        rospy.loginfo(f"[{self.name}] Waiting for car_state/pose")
        rospy.wait_for_message('/car_state/pose', PoseStamped)
        self.track_length = rospy.get_param("/global_republisher/track_length")   
        rospy.loginfo(f"[{self.name}] Ready!")

        while not rospy.is_shutdown():
            if self.measuring:
                start = time.perf_counter()
            #lock wpnts to not get changed trough loop
            with self.lock:
                self.set_waypoint_markers(self.waypoint_array_in_map)
            #initializing ackermann variables
            speed, acceleration, jerk, steering_angle = 0, 0, 0, 0

            #Logic to select controller
            if self.state == "FTGONLY":
                speed, steering_angle = self.ftg_cycle()
            
            else: 
                if self.ctrl_algo == "MAP":
                    speed, acceleration, jerk, steering_angle = self.map_cycle()

                elif self.ctrl_algo == "PP":
                    speed, acceleration, jerk, steering_angle = self.pp_cycle()
                    
                elif self.ctrl_algo == "STMPC":
                    speed, acceleration, jerk, steering_angle = self.stmpc_cycle()

                elif self.ctrl_algo == "KMPC":
                    speed, acceleration, jerk, steering_angle = self.kmpc_cycle()
                else:
                    rospy.logwarn(f"[{self.name}] No valid controller selected")
                
            if self.measuring:
                end = time.perf_counter()
                self.measure_pub.publish(end-start)
            ack_msg = self.create_ack_msg(speed, acceleration, jerk, steering_angle)
            self.drive_pub.publish(ack_msg)
            self.visualize_steering(steering_angle)
            rate.sleep()


############################################HELPERS############################################
    def map_cycle(self):
        speed, acceleration, jerk, steering_angle, L1_point, L1_distance, idx_nearest_waypoint = self.map_controller.main_loop(self.state, 
                                                                                                                    self.position_in_map, 
                                                                                                                    self.waypoint_array_in_map, 
                                                                                                                    self.speed_now, 
                                                                                                                    self.opponent, 
                                                                                                                    self.position_in_map_frenet, 
                                                                                                                    self.acc_now,
                                                                                                                    self.track_length)
                
        self.set_lookahead_marker(L1_point, 100)
        self.visualize_trailing_opponent()
        self.l1_pub.publish(Point(x=idx_nearest_waypoint, y=L1_distance))
        
        
        self.waypoint_safety_counter += 1
        if self.waypoint_safety_counter >= self.loop_rate/self.state_machine_rate* 10: #we can use the same waypoints for 5 cycles
            rospy.logerr_throttle(0.5, f"[{self.name}] Received no local wpnts. STOPPING!!") 
            speed = 0
            steering_angle = 0
        self.map_controller.flag1 = False
        
        # Publish PID data of the Trailing controller for tuning
        if self.map_controller.flag1 == True:
            pid_msg = self.create_pid_msg(self.map_controller.gap_should, 
                                            self.map_controller.gap, 
                                            self.map_controller.gap_error, 
                                            self.map_controller.v_diff, 
                                            self.map_controller.i_gap, 
                                            self.map_controller.trailing_command)
            self.gap_data.publish(pid_msg)

        return speed, acceleration, jerk, steering_angle
    
    def pp_cycle(self):
        speed, acceleration, jerk, steering_angle, L1_point, L1_distance, idx_nearest_waypoint = self.pp_controller.main_loop(self.state, 
                                                                                                                    self.position_in_map, 
                                                                                                                    self.waypoint_array_in_map, 
                                                                                                                    self.speed_now, 
                                                                                                                    self.opponent, 
                                                                                                                    self.position_in_map_frenet, 
                                                                                                                    self.acc_now,
                                                                                                                    self.track_length)
                
        self.set_lookahead_marker(L1_point, 100)
        self.visualize_trailing_opponent()
        self.l1_pub.publish(Point(x=idx_nearest_waypoint, y=L1_distance))
        
        
        self.waypoint_safety_counter += 1
        if self.waypoint_safety_counter >= self.loop_rate/self.state_machine_rate* 10: #we can use the same waypoints for 5 cycles
            rospy.logerr_throttle(0.5, f"[{self.name}] Received no local wpnts. STOPPING!!") 
            speed = 0
            steering_angle = 0
        self.pp_controller.flag1 = False
        
        # Publish PID data of the Trailing controller for tuning
        if self.pp_controller.flag1 == True:
            pid_msg = self.create_pid_msg(self.pp_controller.gap_should, 
                                            self.pp_controller.gap, 
                                            self.pp_controller.gap_error, 
                                            self.pp_controller.v_diff, 
                                            self.pp_controller.i_gap, 
                                            self.pp_controller.trailing_command)
            self.gap_data.publish(pid_msg)

        return speed, acceleration, jerk, steering_angle

    def ftg_cycle(self):
        speed, steer = self.ftg_controller.process_lidar(self.scan.ranges)
        rospy.logwarn(f"[{self.name}] FTGONLY!!!")
        return speed, steer 
    
    def create_pid_msg(self, should, actual, error, d_value, i_value, input):
            pid_msg = PidData()
            pid_msg.header.stamp = rospy.Time.now()
            pid_msg.should = should
            pid_msg.actual = actual
            pid_msg.error = error
            pid_msg.d_value = d_value
            pid_msg.i_value = i_value
            pid_msg.input = input
            return pid_msg
        
    def create_ack_msg(self, speed, acceleration, jerk, steering_angle):
        ack_msg = AckermannDriveStamped()
        ack_msg.header.stamp = self.ros_time.now()
        ack_msg.header.frame_id = 'base_link'
        ack_msg.drive.steering_angle = steering_angle
        ack_msg.drive.speed = speed
        ack_msg.drive.jerk = jerk
        ack_msg.drive.acceleration = acceleration
        return ack_msg

    def stmpc_cycle(self):
        self.mpc_fre_pos = self.position_in_map_frenet
        self.mpc_fre_pos[2] = self.alpha
        self.single_track_state = np.array([self.vel_y, self.yaw_rate, self.acc_now[0], 0])
        
        d = self.stmpc_controller.main_loop(
            self.state,
            self.position_in_map,
            self.waypoint_array_in_map,
            self.speed_now,
            self.opponent,
            self.mpc_fre_pos,
            self.single_track_state,
            self.track_length,
            0) # TODO currently compute time not used
        speed, acceleration, jerk, steering_angle, states, status= d
        if status != 0: #Solver failed
            rospy.logerr(f"[{self.name}] Solver failed with {status = }, stopping")
            return 0, 0, 0, 0

        self.waypoint_safety_counter += 1
        if self.waypoint_safety_counter >= self.loop_rate / self.state_machine_rate * 10:
            rospy.logerr_throttle(0.5, "[Controller] Received no local wpnts. STOPPING!!")
            speed = 0
            steering_angle = 0

        # publish mpc states to vizualize
        if states is not None:
            self.publish_mpc_states(states)

        return speed, acceleration, jerk, steering_angle
    
    def kmpc_cycle(self):
        self.mpc_fre_pos = self.position_in_map_frenet
        self.mpc_fre_pos[2] = self.alpha
        d = self.kmpc_controller.main_loop(
            self.state,
            self.position_in_map,
            self.waypoint_array_in_map,
            self.speed_now,
            self.opponent,
            self.mpc_fre_pos,
            self.acc_now,
            self.track_length,
            0.0) # TODO currently compute time not used
        speed, acceleration, jerk, steering_angle, states = d

        self.waypoint_safety_counter += 1
        if self.waypoint_safety_counter >= self.loop_rate / self.state_machine_rate * 10:
            rospy.logerr_throttle(0.5, "[Controller] Received no local wpnts. STOPPING!!")
            speed = 0
            steering_angle = 0

        # publish mpc states to vizualize
        if states is not None:
            self.publish_mpc_states(states)
        return speed, acceleration, jerk, steering_angle
    
############################################MSG CREATION############################################
# visualization utilities
    def visualize_steering(self, theta):

        quaternions = quaternion_from_euler(0, 0, theta)

        lookahead_marker = Marker()
        lookahead_marker.header.frame_id = "base_link"
        lookahead_marker.header.stamp = self.ros_time.now()
        lookahead_marker.type = Marker.ARROW
        lookahead_marker.id = 50
        lookahead_marker.scale.x = 0.6
        lookahead_marker.scale.y = 0.05
        lookahead_marker.scale.z = 0
        lookahead_marker.color.r = 1.0
        lookahead_marker.color.g = 0.0
        lookahead_marker.color.b = 0.0
        lookahead_marker.color.a = 1.0
        lookahead_marker.lifetime = rospy.Duration()
        lookahead_marker.pose.position.x = 0
        lookahead_marker.pose.position.y = 0
        lookahead_marker.pose.position.z = 0
        lookahead_marker.pose.orientation.x = quaternions[0]
        lookahead_marker.pose.orientation.y = quaternions[1]
        lookahead_marker.pose.orientation.z = quaternions[2]
        lookahead_marker.pose.orientation.w = quaternions[3]
        self.lookahead_pub.publish(lookahead_marker)

    def set_waypoint_markers(self, waypoints):
        wpnt_id = 0

        for waypoint in waypoints:
            waypoint_marker = self.markers_buf[wpnt_id]
            waypoint_marker.header.frame_id = "map"
            waypoint_marker.header.stamp = self.ros_time.now()
            waypoint_marker.type = 2
            waypoint_marker.scale.x = 0.1
            waypoint_marker.scale.y = 0.1
            waypoint_marker.scale.z = 0.1
            waypoint_marker.color.r = 0.0
            waypoint_marker.color.g = 0.0
            waypoint_marker.color.b = 1.0
            waypoint_marker.color.a = 1.0
            waypoint_marker.pose.position.x = waypoint[0]
            waypoint_marker.pose.position.y = waypoint[1]
            waypoint_marker.pose.position.z = 0
            waypoint_marker.pose.orientation.x = 0
            waypoint_marker.pose.orientation.y = 0
            waypoint_marker.pose.orientation.z = 0
            waypoint_marker.pose.orientation.w = 1
            waypoint_marker.id = wpnt_id + 1
            wpnt_id += 1
        self.waypoint_array_buf.markers = self.markers_buf[:wpnt_id]
        self.waypoint_pub.publish(self.waypoint_array_buf)

    def set_lookahead_marker(self, lookahead_point, id):
        lookahead_marker = Marker()
        lookahead_marker.header.frame_id = "map"
        lookahead_marker.header.stamp = self.ros_time.now()
        lookahead_marker.type = 2
        lookahead_marker.id = id
        lookahead_marker.scale.x = 0.15
        lookahead_marker.scale.y = 0.15
        lookahead_marker.scale.z = 0.15
        lookahead_marker.color.r = 1.0
        lookahead_marker.color.g = 0.0
        lookahead_marker.color.b = 0.0
        lookahead_marker.color.a = 1.0
        lookahead_marker.pose.position.x = lookahead_point[0]
        lookahead_marker.pose.position.y = lookahead_point[1]
        lookahead_marker.pose.position.z = 0
        lookahead_marker.pose.orientation.x = 0
        lookahead_marker.pose.orientation.y = 0
        lookahead_marker.pose.orientation.z = 0
        lookahead_marker.pose.orientation.w = 1
        self.lookahead_pub.publish(lookahead_marker)

    def visualize_trailing_opponent(self):
        if(self.state == "TRAILING" and (self.opponent is not None)):
            on = True
        else:
            on = False
        opponent_marker = Marker()
        opponent_marker.header.frame_id = "map"
        opponent_marker.header.stamp = self.ros_time.now()
        opponent_marker.type = 2
        opponent_marker.scale.x = 0.3
        opponent_marker.scale.y = 0.3
        opponent_marker.scale.z = 0.3
        opponent_marker.color.r = 1.0
        opponent_marker.color.g = 0.0
        opponent_marker.color.b = 0.0
        opponent_marker.color.a = 1.0
        if self.opponent is not None:
            pos = self.converter.get_cartesian([self.opponent[0]], [self.opponent[1]])
            opponent_marker.pose.position.x = pos[0]
            opponent_marker.pose.position.y = pos[1]
            opponent_marker.pose.position.z = 0

        opponent_marker.pose.orientation.x = 0
        opponent_marker.pose.orientation.y = 0
        opponent_marker.pose.orientation.z = 0
        opponent_marker.pose.orientation.w = 1
        if on == False:
            opponent_marker.action = Marker.DELETE
        self.trailing_pub.publish(opponent_marker)

    def publish_mpc_states(self, states: list) -> None:
        """Publishes states into /mpc_controller/states"""
        msg = Float64MultiArray()
        msg.data = states
        self.mpc_states_pub.publish(msg)

if __name__ == "__main__":
    # client = dynamic_reconfigure.client.Client("MAP params", timeout=30, config_callback=callback)
    controller_manager = Controller_manager()
    controller_manager.control_loop()
 

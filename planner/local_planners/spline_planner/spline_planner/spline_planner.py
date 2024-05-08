import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rcl_interfaces.msg import FloatingPointRange, ParameterDescriptor, SetParametersResult, ParameterType
from rclpy.parameter import Parameter

from f110_msgs.msg import Obstacle, ObstacleArray, OTWpntArray, Wpnt, WpntArray
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray

import numpy as np
import time
from typing import List, Any, Tuple
from scipy.interpolate import InterpolatedUnivariateSpline as Spline
from frenet_conversion.frenet_converter import FrenetConverter

class ObstacleSpliner(Node):
    """
    This class implements a ROS node that performs splining around obstacles.

    It subscribes to the following topics:
        - `/perception/obstacles`: Subscribes to the obstacle array.
        - `/car_state/odom_frenet`: Subscribes to the car state in Frenet coordinates.
        - `/global_waypoints`: Subscribes to global waypoints.
        - `/global_waypoints_scaled`: Subscribes to the scaled global waypoints.

    The node publishes the following topics:
        - `/planner/avoidance/markers`: Publishes spline markers.
        - `/planner/avoidance/otwpnts`: Publishes splined waypoints.
        - `/planner/avoidance/considered_OBS`: Publishes markers for the closest obstacle.
        - `/planner/avoidance/propagated_obs`: Publishes markers for the propagated obstacle.
        - `/planner/avoidance/latency`: Publishes the latency of the spliner node. (only if measuring is enabled)
    """

    def __init__(self):
        super().__init__('spliner_node')

        # initialize the instance variable
        self.obs = ObstacleArray()
        self.gb_wpnts = None
        self.gb_scaled_wpnts = None
        self.gb_vmax = None
        self.gb_max_idx = None
        self.gb_max_s = None
        self.cur_s = None
        self.cur_d = None
        self.cur_vs = None
        self.lookahead = 10  # in meters [m]
        self.last_switch_time = self.get_clock().now().to_msg()
        self.last_ot_side = ""

        # Initialize instance variables
        self.declare_parameters(
            namespace='',
            parameters=[
                ('from_bag', False),
                ('measure', False),
            ])
        self.from_bag = self.get_parameter(
            'from_bag').get_parameter_value().bool_value
        self.measuring = self.get_parameter(
            'measure').get_parameter_value().bool_value

        # Setup subscriptions
        self.create_subscription(
            ObstacleArray, '/perception/obstacles', self.obs_cb, QoSProfile(depth=10))
        self.create_subscription(
            Odometry, '/car_state/frenet/odom', self.state_cb, QoSProfile(depth=10))
        self.create_subscription(
            WpntArray, '/global_waypoints', self.gb_cb, QoSProfile(depth=10))
        self.create_subscription(
            WpntArray, '/global_waypoints_scaled', self.gb_scaled_cb, QoSProfile(depth=10))

        # Setup publishers
        self.mrks_pub = self.create_publisher(
            MarkerArray, '/planner/avoidance/markers', QoSProfile(depth=10))
        self.evasion_pub = self.create_publisher(
            OTWpntArray, '/planner/avoidance/otwpnts', QoSProfile(depth=10))
        self.closest_obs_pub = self.create_publisher(
            Marker, '/planner/avoidance/considered_OBS', QoSProfile(depth=10))
        self.pub_propagated = self.create_publisher(
            Marker, '/planner/avoidance/propagated_obs', QoSProfile(depth=10))

        # dyn params sub
        self.pre_apex_0 = -4.0
        self.pre_apex_1 = -3.0
        self.pre_apex_2 = -1.5
        self.post_apex_0 = 2.0
        self.post_apex_1 = 3.0
        self.post_apex_2 = 4.0
        self.evasion_dist = 0.65
        self.obs_traj_tresh = 0.3
        self.spline_bound_mindist = 0.2
        self.fixed_pred_time = 0.15
        self.kd_obs_pred = 1.0
        
        pd = ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            floating_point_range=[FloatingPointRange(from_value=0.1, to_value=8.0, step=0.001)]
        )
        param_dicts = [
            {'name': 'pre_apex_0', 'default': abs(self.pre_apex_0), 'descriptor': pd},
            {'name': 'pre_apex_1', 'default': abs(self.pre_apex_1), 'descriptor': pd},
            {'name': 'pre_apex_2', 'default': abs(self.pre_apex_2), 'descriptor': pd},
            {'name': 'post_apex_0', 'default': self.post_apex_0, 'descriptor': pd},
            {'name': 'post_apex_1', 'default': self.post_apex_1, 'descriptor': pd},
            {'name': 'post_apex_2', 'default': self.post_apex_2, 'descriptor': pd},
            {'name': 'evasion_dist', 'default': self.evasion_dist, 'descriptor': pd},
            {'name': 'obs_traj_tresh', 'default': self.obs_traj_tresh, 'descriptor': pd},
            {'name': 'spline_bound_mindist', 'default': self.spline_bound_mindist, 'descriptor': pd},
            {'name': 'fixed_pred_time', 'default': self.fixed_pred_time, 'descriptor': pd},
            {'name': 'kd_obs_pred', 'default': self.kd_obs_pred, 'descriptor': pd}
        ]

        self.declare_all_parameters(param_dicts=param_dicts)
        self.add_on_set_parameters_callback(self.dyn_param_cb)

        if self.measuring:
            self.latency_pub = self.create_publisher(Float32, '/planner/avoidance/latency', QoSProfile(depth=10))

        # Wait for messages
        self.wait_for_messages()

        # Initialize converter, if required
        self.converter = self.initialize_converter()

        # Create timer
        self.create_timer(1/20, self.spliner_loop)

    #################### DYNAMIC PARAMS####################
    def declare_all_parameters(self, param_dicts: List[dict]):
        params = []
        for param_dict in param_dicts:
            param = self.declare_parameter(
                param_dict['name'], param_dict['default'], param_dict['descriptor'])
            params.append(param)
        return params

    def dyn_param_cb(self, params: List[Parameter]):
        """
        Notices the change in the parameters and alters the spline params accordingly
        """
        for param in params:
            param_name = param.name
            if param_name == 'pre_apex_0':
                self.pre_apex_0 = -1 * param.value
            elif param_name == 'pre_apex_1':
                self.pre_apex_1 = -1 * param.value
            elif param_name == 'pre_apex_2':
                self.pre_apex_2 = -1 * param.value
            elif param_name == 'post_apex_0':
                self.post_apex_0 = param.value
            elif param_name == 'post_apex_1':
                self.post_apex_1 = param.value
            elif param_name == 'post_apex_2':
                self.post_apex_2 = param.value
            elif param_name == 'evasion_dist':
                self.evasion_dist = param.value
            elif param_name == 'obs_traj_tresh':
                self.obs_traj_tresh = param.value
            elif param_name == 'spline_bound_mindist':
                self.spline_bound_mindist = param.value
            elif param_name == 'fixed_pred_time':
                self.fixed_pred_time = param.value
            elif param_name == 'kd_obs_pred':
                self.kd_obs_pred = param.value
        
        # Ensure ascending order for spline parameters
        if self.pre_apex_1 < self.pre_apex_0:
            self.get_logger().info("Adjusting pre_apex_1 to ensure ascending order.")
            self.pre_apex_1 = self.pre_apex_0

        if self.pre_apex_2 < self.pre_apex_1:
            self.get_logger().info("Adjusting pre_apex_2 to ensure ascending order.")
            self.pre_apex_2 = self.pre_apex_1

        if self.post_apex_0 < self.pre_apex_2:
            self.get_logger().info("Adjusting post_apex_0 to ensure ascending order.")
            self.post_apex_0 = self.pre_apex_2

        if self.post_apex_1 < self.post_apex_0:
            self.get_logger().info("Adjusting post_apex_1 to ensure ascending order.")
            self.post_apex_1 = self.post_apex_0

        if self.post_apex_2 < self.post_apex_1:
            self.get_logger().info("Adjusting post_apex_2 to ensure ascending order.")
            self.post_apex_2 = self.post_apex_1
        
        spline_params = [
            self.pre_apex_0,
            self.pre_apex_1,
            self.pre_apex_2,
            0,
            self.post_apex_0,
            self.post_apex_1,
            self.post_apex_2,
        ]
        
        self.get_logger().info(
            f" Dynamic reconf triggered new spline params: {spline_params} [m],\n"
            f" evasion apex distance: {self.evasion_dist} [m],\n"
            f" obstacle trajectory treshold: {self.obs_traj_tresh} [m]\n"
            f" obstacle prediciton k_d: {self.kd_obs_pred},    obstacle prediciton constant time: {self.fixed_pred_time} [s] "
        )
        return SetParametersResult(successful=True)

    #################### CALLBACKS####################
    def obs_cb(self, msg: ObstacleArray):
        # Callback function for obstacle data
        self.obs = msg

    def state_cb(self, msg: Odometry):
        # Callback function for car state data
        self.cur_s = msg.pose.pose.position.x
        self.cur_d = msg.pose.pose.position.y
        self.cur_vs = msg.twist.twist.linear.x

    def gb_cb(self, msg: WpntArray):
        # Callback function for global waypoints
        self.gb_wpnts = msg
        if self.gb_vmax is None:
            self.gb_vmax = np.max(
                np.array([wpnt.vx_mps for wpnt in msg.wpnts]))
            self.gb_max_idx = msg.wpnts[-1].id
            self.gb_max_s = msg.wpnts[-1].s_m

    def gb_scaled_cb(self, msg: WpntArray):
        # Callback function for scaled global waypoints
        self.gb_scaled_wpnts = msg

    #################### HELPER FUNCTIONS####################
    def wait_for_messages(self):
        self.get_logger().info('Carstate Node waiting for Odometry messages...')
        state_print = False
        gb_print = False
        scaled_gb_print = False
        waitlist = [self.cur_s, self.gb_wpnts, self.gb_scaled_wpnts]
        while None in waitlist:
            rclpy.spin_once(self)
            if self.cur_s is not None and not state_print:
                self.get_logger().info('Received State message.')
                state_print = True
            if self.gb_wpnts is not None and not gb_print:
                self.get_logger().info('Received Global Waypoints message.')
                gb_print = True
            if self.gb_scaled_wpnts is not None and not scaled_gb_print:
                self.get_logger().info('Received Scaled Global Waypoints message.')
                scaled_gb_print = True
            waitlist = [self.cur_s, self.gb_wpnts, self.gb_scaled_wpnts]
        self.get_logger().info('All required messages received. Continuing...')

    def initialize_converter(self):
        # Initialize any converters or additional components
        waypoint_array = self.gb_wpnts.wpnts
        waypoints_x = [waypoint.x_m for waypoint in waypoint_array]
        waypoints_y = [waypoint.y_m for waypoint in waypoint_array]
        waypoints_psi = [waypoint.psi_rad for waypoint in waypoint_array]
        return FrenetConverter(np.array(waypoints_x), np.array(waypoints_y), np.array(waypoints_psi))

    #################### UTILS FUNCTIONS####################
    def _predict_obs_movement(self, obs: Obstacle, mode: str = "constant") -> Obstacle:
        """
        Predicts the movement of an obstacle based on the current state and mode.

        TODO: opponent prediction should be completely isolated for added modularity       

        Args:
            obs (Obstacle): The obstacle to predict the movement for.
            mode (str, optional): The mode for predicting the movement. Defaults to "constant".

        Returns:
            Obstacle: The updated obstacle with the predicted movement.
        """
        # propagate opponent by time dependent on distance
        if (obs.s_center - self.cur_s) % self.gb_max_s < 10:  # TODO make param
            if mode == "adaptive":
                # distance in s coordinate
                cur_s = self.cur_s
                ot_distance = (obs.s_center - cur_s) % self.gb_max_s
                rel_speed = np.clip(self.gb_scaled_wpnts.wpnts[int(
                    cur_s * 10)].vx_mps - obs.vs, 0.1, 10)
                ot_time_distance = np.clip(ot_distance / rel_speed, 0, 5) * 0.5

                delta_s = ot_time_distance * obs.vs
                delta_d = ot_time_distance * obs.vd
                delta_d = -(obs.d_center + delta_d) * \
                    np.exp(-np.abs(self.kd_obs_pred * obs.d_center))

            elif mode == "adaptive_velheuristic":
                opponent_scaler = 0.7
                cur_s = self.cur_s
                ego_speed = self.gb_scaled_wpnts.wpnts[int(cur_s * 10)].vx_mps

                # distance in s coordinate
                ot_distance = (obs.s_center - cur_s) % self.gb_max_s
                rel_speed = (1 - opponent_scaler) * ego_speed
                ot_time_distance = np.clip(ot_distance / rel_speed, 0, 5)

                delta_s = ot_time_distance * opponent_scaler * ego_speed
                delta_d = -(obs.d_center) * \
                    np.exp(-np.abs(self.kd_obs_pred * obs.d_center))

            # propagate opponent by constant time
            elif mode == "constant":
                delta_s = self.fixed_pred_time * obs.vs
                delta_d = self.fixed_pred_time * obs.vd
                # delta_d = -(obs.d_center+delta_d)*np.exp(-np.abs(self.kd_obs_pred*obs.d_center))

            elif mode == "heuristic":
                # distance in s coordinate
                ot_distance = (obs.s_center - self.cur_s) % self.gb_max_s
                rel_speed = 3
                ot_time_distance = ot_distance / rel_speed

                delta_d = ot_time_distance * obs.vd
                delta_d = -(obs.d_center + delta_d) * \
                    np.exp(-np.abs(self.kd_obs_pred * obs.d_center))

            # update
            obs.s_start += delta_s
            obs.s_center += delta_s
            obs.s_end += delta_s
            obs.s_start %= self.gb_max_s
            obs.s_center %= self.gb_max_s
            obs.s_end %= self.gb_max_s

            obs.d_left += delta_d
            obs.d_center += delta_d
            obs.d_right += delta_d

            resp = self.converter.get_cartesian([obs.s_center], [obs.d_center])

            marker = self.xy_to_point(resp[0], resp[1], opponent=True)
            self.pub_propagated.publish(marker)

        return obs

    def _check_ot_side_possible(self, more_space) -> bool:
        # TODO make rosparam for cur_d threshold
        if abs(self.cur_d) > 0.25 and more_space != self.last_ot_side:
            self.get_logger().info("Can't switch sides, because we are not on the raceline")
            return False
        return True

    def _more_space(self, obstacle: Obstacle, gb_wpnts: List[Any], gb_idxs: List[int]) -> Tuple[str, float]:
        left_gap = abs(gb_wpnts[gb_idxs[0]].d_left - obstacle.d_left)
        right_gap = abs(gb_wpnts[gb_idxs[0]].d_right + obstacle.d_right)
        min_space = self.evasion_dist + self.spline_bound_mindist

        if right_gap > min_space and left_gap < min_space:
            # Compute apex distance to the right of the opponent
            d_apex_right = obstacle.d_right - self.evasion_dist
            # If we overtake to the right of the opponent BUT the apex is to the left of the raceline, then we set the apex to 0
            if d_apex_right > 0:
                d_apex_right = 0
            return "right", d_apex_right

        elif left_gap > min_space and right_gap < min_space:
            # Compute apex distance to the left of the opponent
            d_apex_left = obstacle.d_left + self.evasion_dist
            # If we overtake to the left of the opponent BUT the apex is to the right of the raceline, then we set the apex to 0
            if d_apex_left < 0:
                d_apex_left = 0
            return "left", d_apex_left
        else:
            candidate_d_apex_left = obstacle.d_left + self.evasion_dist
            candidate_d_apex_right = obstacle.d_right - self.evasion_dist

            if abs(candidate_d_apex_left) <= abs(candidate_d_apex_right):
                # If we overtake to the left of the opponent BUT the apex is to the right of the raceline, then we set the apex to 0
                if candidate_d_apex_left < 0:
                    candidate_d_apex_left = 0
                return "left", candidate_d_apex_left
            else:
                # If we overtake to the right of the opponent BUT the apex is to the left of the raceline, then we set the apex to 0
                if candidate_d_apex_right > 0:
                    candidate_d_apex_right = 0
                return "right", candidate_d_apex_right

    #################### MAIN FUNCTIONS####################
    def spliner_loop(self):
        # Main loop for the spliner node
        if self.measuring:
            start = time.perf_counter()

        # Sample data
        obs = self.obs
        gb_scaled_wpnts = self.gb_scaled_wpnts.wpnts
        wpnts = OTWpntArray()
        mrks = MarkerArray()

        # If obs then do splining around it
        if len(obs.obstacles) > 0:
            wpnts, mrks = self.do_spline(
                obstacles=obs, gb_wpnts=gb_scaled_wpnts)
        # Else delete spline markers
        else:
            del_mrk = Marker()
            del_mrk.header.stamp = self.get_clock().now().to_msg()
            del_mrk.action = Marker.DELETEALL
            mrks.markers.append(del_mrk)

        # Publish wpnts and markers
        if self.measuring:
            end = time.perf_counter()
            self.latency_pub.publish(end - start)
        self.evasion_pub.publish(wpnts)
        self.mrks_pub.publish(mrks)

    def do_spline(self, obstacles: ObstacleArray, gb_wpnts: WpntArray) -> Tuple[WpntArray, MarkerArray]:
        """
        Creates an evasion trajectory for the closest obstacle by splining between pre- and post-apex points.

        This function takes as input the obstacles to be evaded, and a list of global waypoints that describe a reference raceline.
        It only considers obstacles that are within a threshold of the raceline and generates an evasion trajectory for each of these obstacles.
        The evasion trajectory consists of a spline between pre- and post-apex points that are spaced apart from the obstacle.
        The spatial and velocity components of the spline are calculated using the `Spline` class, and the resulting waypoints and markers are returned.

        Args:
        - obstacles (ObstacleArray): An array of obstacle objects to be evaded.
        - gb_wpnts (WpntArray): A list of global waypoints that describe a reference raceline.
        - state (Odometry): The current state of the car.

        Returns:
        - wpnts (WpntArray): An array of waypoints that describe the evasion trajectory to the closest obstacle.
        - mrks (MarkerArray): An array of markers that represent the waypoints in a visualization format.

        """
        # Return wpnts and markers
        mrks = MarkerArray()
        wpnts = OTWpntArray()

        # Get spacing between wpnts for rough approximations
        wpnt_dist = gb_wpnts[1].s_m - gb_wpnts[0].s_m

        # Only use obstacles that are within a threshold of the raceline, else we don't care about them
        close_obs = self._obs_filtering(obstacles=obstacles)

        # If there are obstacles within the lookahead distance, then we need to generate an evasion trajectory considering the closest one
        if len(close_obs) > 0:
            # Get the closest obstacle handling wraparound
            closest_obs = min(
                close_obs, key=lambda obs: (
                    obs.s_center - self.cur_s) % self.gb_max_s
            )

            # Get Apex for evasion that is further away from the trackbounds
            if closest_obs.s_end < closest_obs.s_start:
                s_apex = (closest_obs.s_end + self.gb_max_s +
                          closest_obs.s_start) / 2
            else:
                s_apex = (closest_obs.s_end + closest_obs.s_start) / 2
            # Approximate next 20 indexes of global wpnts with wrapping => 2m and compute which side is the outside of the raceline
            gb_idxs = [int(s_apex / wpnt_dist + i) %
                       self.gb_max_idx for i in range(20)]
            kappas = np.array(
                [gb_wpnts[gb_idx].kappa_radpm for gb_idx in gb_idxs])
            outside = "left" if np.sum(kappas) < 0 else "right"
            # Choose the correct side and compute the distance to the apex based on left of right of the obstacle
            more_space, d_apex = self._more_space(
                closest_obs, gb_wpnts, gb_idxs)

            # Publish the point around which we are splining
            mrk = self.xy_to_point(
                x=gb_wpnts[gb_idxs[0]].x_m, y=gb_wpnts[gb_idxs[0]].y_m, opponent=False)
            self.closest_obs_pub.publish(mrk)

            # Choose wpnts from global trajectory for splining with velocity
            evasion_points = []
            spline_params = [
                self.pre_apex_0,
                self.pre_apex_1,
                self.pre_apex_2,
                0,
                self.post_apex_0,
                self.post_apex_1,
                self.post_apex_2,
            ]
            for i, dst in enumerate(spline_params):
                # scale dst linearly between 1 and 1.5 depending on the speed normalised to the max speed
                dst = dst * np.clip(1.0 + self.cur_vs / self.gb_vmax, 1, 1.5)
                # If we overtake on the outside, we smoothen the spline
                if outside == more_space:
                    si = s_apex + dst * 1.75  # TODO make parameter
                else:
                    si = s_apex + dst
                di = d_apex if dst == 0 else 0
                evasion_points.append([si, di])
            # Convert to nump
            evasion_points = np.array(evasion_points)

            # Spline spatialy for d with s as base
            # TODO read from ros params to make consistent in case it changes
            spline_resolution = 0.1
            spatial_spline = Spline(
                x=evasion_points[:, 0], y=evasion_points[:, 1])
            evasion_s = np.arange(
                evasion_points[0, 0], evasion_points[-1, 0], spline_resolution)
            # Clipe the d to the apex distance
            if d_apex < 0:
                evasion_d = np.clip(spatial_spline(evasion_s), d_apex, 0)
            else:
                evasion_d = np.clip(spatial_spline(evasion_s), 0, d_apex)

            # Handle Wrapping of s
            evasion_s = evasion_s % self.gb_max_s

            # Do frenet conversion via conversion service for spline and create markers and wpnts
            danger_flag = False
            resp = self.converter.get_cartesian(evasion_s, evasion_d)

            # Check if a side switch is possible
            if not self._check_ot_side_possible(more_space):
                danger_flag = True

            for i in range(evasion_s.shape[0]):
                gb_wpnt_i = int((evasion_s[i] / wpnt_dist) % self.gb_max_idx)
                # Check if wpnt is too close to the trackbounds but only if spline is actually off the raceline
                if abs(evasion_d[i]) > spline_resolution:
                    tb_dist = gb_wpnts[gb_wpnt_i].d_left if more_space == "left" else gb_wpnts[gb_wpnt_i].d_right
                    if abs(evasion_d[i]) > abs(tb_dist) - self.spline_bound_mindist:
                        self.get_logger().info(
                            "Evasion trajectory too close to TRACKBOUNDS, aborting evasion"
                        )
                        danger_flag = True
                        break
                # Get V from gb wpnts and go slower if we are going through the inside
                # TODO make speed scaling ros param
                vi = gb_wpnts[gb_wpnt_i].vx_mps if outside == more_space else gb_wpnts[gb_wpnt_i].vx_mps * 0.9
                wpnts.wpnts.append(
                    self.xyv_to_wpnts(
                        x=resp[0, i], y=resp[1, i], s=evasion_s[i], d=evasion_d[i], v=vi, wpnts=wpnts)
                )
                mrks.markers.append(self.xyv_to_markers(
                    x=resp[0, i], y=resp[1, i], v=vi, mrks=mrks))

            # Fill the rest of OTWpnts
            wpnts.header.stamp = self.get_clock().now().to_msg()
            wpnts.header.frame_id = "map"
            if not danger_flag:
                wpnts.ot_side = more_space
                wpnts.ot_line = outside
                wpnts.side_switch = True if self.last_ot_side != more_space else False
                wpnts.last_switch_time = self.last_switch_time

                # Update the last switch time and the last side
                if self.last_ot_side != more_space:
                    self.last_switch_time = self.get_clock().now().to_msg()
                self.last_ot_side = more_space
            else:
                wpnts.wpnts = []
                mrks.markers = []
                # This fools the statemachine to cool down
                wpnts.side_switch = True
                self.last_switch_time = self.get_clock().now().to_msg()
        return wpnts, mrks

    def _obs_filtering(self, obstacles: ObstacleArray) -> List[Obstacle]:
        # Only use obstacles that are within a threshold of the raceline, else we don't care about them
        obs_on_traj = [obs for obs in obstacles.obstacles if abs(
            obs.d_center) < self.obs_traj_tresh]

        # Only use obstacles that within self.lookahead in front of the car
        close_obs = []
        for obs in obs_on_traj:
            obs = self._predict_obs_movement(obs)
            # Handle wraparound
            dist_in_front = (obs.s_center - self.cur_s) % self.gb_max_s
            # dist_in_back = abs(dist_in_front % (-self.gb_max_s)) # distance from ego to obstacle in the back
            if dist_in_front < self.lookahead:
                close_obs.append(obs)
                # Not within lookahead
            else:
                pass
        return close_obs

    #################### VISUALIZATION FUNCTIONS####################
    def xyv_to_markers(self, x: float, y: float, v: float, mrks: MarkerArray) -> Marker:
        mrk = Marker()
        mrk.header.frame_id = "map"
        mrk.header.stamp = self.get_clock().now().to_msg()
        mrk.type = mrk.CYLINDER
        mrk.scale.x = 0.1
        mrk.scale.y = 0.1
        mrk.scale.z = float(v / self.gb_vmax)
        mrk.color.a = 1.0
        mrk.color.b = 0.75
        mrk.color.r = 0.75
        if self.from_bag:
            mrk.color.g = 0.75

        mrk.id = len(mrks.markers)
        mrk.pose.position.x = float(x)
        mrk.pose.position.y = float(y)
        mrk.pose.position.z = v / float(self.gb_vmax / 2)
        mrk.pose.orientation.w = 1.0

        return mrk

    def xy_to_point(self, x: float, y: float, opponent=True) -> Marker:
        mrk = Marker()
        mrk.header.frame_id = "map"
        mrk.header.stamp = self.get_clock().now().to_msg()
        mrk.type = mrk.SPHERE
        mrk.scale.x = 0.5
        mrk.scale.y = 0.5
        mrk.scale.z = 0.5
        mrk.color.a = 0.8
        mrk.color.b = 0.65
        mrk.color.r = 1.0 if opponent else 0.0
        mrk.color.g = 0.65

        mrk.pose.position.x = float(x)
        mrk.pose.position.y = float(y)
        mrk.pose.position.z = 0.01
        mrk.pose.orientation.w = 1.0

        return mrk

    def xyv_to_wpnts(self, s: float, d: float, x: float, y: float, v: float, wpnts: OTWpntArray) -> Wpnt:
        wpnt = Wpnt()
        wpnt.id = len(wpnts.wpnts)
        wpnt.x_m = float(x)
        wpnt.y_m = float(y)
        wpnt.s_m = float(s)
        wpnt.d_m = float(d)
        wpnt.vx_mps = float(v)
        return wpnt


def main(args=None):
    rclpy.init(args=args)
    spline_planner = ObstacleSpliner()
    rclpy.spin(spline_planner)
    spline_planner.destroy_node()
    rclpy.shutdown()

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.parameter import Parameter

import tf2_ros
from tf_transformations import concatenate_matrices, translation_matrix, quaternion_matrix, euler_from_quaternion

from f110_msgs.msg import OTWpntArray, Wpnt, WpntArray
from std_msgs.msg import Float32MultiArray, String
from visualization_msgs.msg import Marker, MarkerArray

from stack_master.parameter_event_handler import ParameterEventHandler
from local_planner.local_planner_params import LocalPlannerParams


def time_to_float(time_instant) -> float:
    return time_instant.sec + time_instant.nanosec * 1e-9


class LocalPlanner(Node):
    """
    Local Planner which calculates and pubishes local waypoints in the odom frame
    """

    def __init__(self):
        super().__init__("local_planner",
                          allow_undeclared_parameters=True,
                          automatically_declare_parameters_from_overrides=True)

        # PARAMETER DECLARATION
        self.params = LocalPlannerParams(self)

        self.cur_state = None
        self.state_functions = {  # which wpt-generation function to use for which state
            "GB_TRACK": self.GlobalTracking,
            "TRAILING": self.Trailing,
            "OVERTAKE": self.Overtaking,
            "FTGONLY": self.FTGOnly,
            "LOW_BAT": self.GlobalTracking,
        }

        self.overtake_wpnts = None

        # Position variables
        self.cur_s = 0.0
        self.cur_d = 0.0

        # waypoint variables
        self.cur_id_ot = 1
        self.max_s = 0.0
        self.current_position = None
        self.glb_wpnts = None
        self.gb_wpnts_arr = None
        self.gb_max_idx = None
        self.max_speed = -1  # max speed in global waypoints for visualising
        self.local_wpnts = WpntArray()
        self.waypoints_dist = 0.1  # [m]
        self.num_glb_wpnts = 0  # number of waypoints on global trajectory
        self.num_ot_points = 0
        self.frenet_wpnts = WpntArray()

        # spliner variables
        splini_ttl = self.params.splini_ttl if self.params.ot_planner == "spliner" else self.params.pred_splini_ttl
        self.splini_ttl_counter = int(splini_ttl * self.params.rate_hz)  # convert seconds to counters
        self.last_valid_avoidance_wpnts = None
        self.avoidance_wpnts = None

        # Graph Based Variables
        self.graph_based_wpts = None

        # create TF listener, to transform local waypoints from the map into the odom frame
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscriptions
        self.create_subscription(WpntArray, "/global_waypoints", self.glb_wpnts_og_cb, 10)  # from og wpnts
        self.create_subscription(WpntArray, "/global_waypoints/overtaking", self.overtake_cb, 10)
        self.create_subscription(String, "/state", self.state_cb, 1)
        self.get_logger().info("[Local Planner] Waiting for state machine state message...")
        while self.cur_state is None:
            rclpy.spin_once(self)
        self.get_logger().info("[Local Planner] Received state message.")

        self.create_subscription(WpntArray, "/global_waypoints_scaled", self.glb_wpnts_cb, 10)  # from velocity scaler
        self.get_logger().info("[Local Planner] Waiting for scaled global waypoints message...")
        while self.glb_wpnts is None:
            rclpy.spin_once(self)
        self.get_logger().info("[Local Planner] Received scaled global waypoints message.")

        self.create_subscription(WpntArray, "/planner/waypoints", self.frenet_planner_cb, 10)
        self.create_subscription(Float32MultiArray, "/planner/graph_based_wpnts", self.graphbased_wpts_cb, 10)
        if self.params.ot_planner == "spliner" or self.params.ot_planner == "predictive_spliner":
            self.create_subscription(OTWpntArray, "/planner/avoidance/otwpnts", self.avoidance_cb, 10)

        # cross-node listener for the state machine's dynamically reconfigurable parameters
        self.param_handler = ParameterEventHandler(self)
        for param_name in ("gb_ego_width_m", "splini_ttl", "splini_hyst_timer_sec"):
            self.param_handler.add_parameter_callback(
                parameter_name=param_name,
                node_name="state_machine",
                callback=self.dyn_param_cb,
            )

        # PUBLICATIONS
        self.loc_wpnt_pub = self.create_publisher(WpntArray, "/local_waypoints", 1)
        self.vis_loc_wpnt_pub = self.create_publisher(MarkerArray, "/local_waypoints/markers", 10)
        if self.params.ot_planner == "predictive_spliner":
            self.del_marker_pub = self.create_publisher(MarkerArray, "/planner/avoidance/markers_sqp", 10)

        # main loop
        self.create_timer(1 / self.params.rate_hz, self.loop)

    #################
    #   Callbacks   #
    #################
    def state_cb(self, msg: String):
        self.cur_state = msg.data

    def glb_wpnts_cb(self, data: WpntArray):
        """
        Callback function of velocity interpolator subscriber.

        Parameters
        ----------
        data
            Data received from velocity interpolator topic
        """
        self.glb_wpnts = data.wpnts[:-1]  # exclude last point (because last point == first point)
        self.num_glb_wpnts = len(self.glb_wpnts)
        self.max_s = data.wpnts[-1].s_m
        self.gb_max_idx = data.wpnts[-1].id
        if self.params.ot_planner == "graph_based":
            self.gb_wpnts_arr = np.array([
                [w.s_m, w.d_m, w.x_m, w.y_m, w.d_right, w.d_left, w.psi_rad,
                 w.kappa_radpm, w.vx_mps, w.ax_mps2] for w in data.wpnts
            ])

    def glb_wpnts_og_cb(self, data):
        """
        Callback function of OG global waypoints 100% speed.

        Parameters
        ----------
        data
            Data received from velocity interpolator topic
        """
        if self.max_speed == -1:
            self.max_speed = max([wpnt.vx_mps for wpnt in data.wpnts])
        else:
            pass

    def avoidance_cb(self, data: OTWpntArray):
        """spliniboi waypoints"""
        if len(data.wpnts) > 0:
            splini_ttl = self.params.splini_ttl if self.params.ot_planner == "spliner" else self.params.pred_splini_ttl
            self.splini_ttl_counter = int(splini_ttl * self.params.rate_hz)
            self.avoidance_wpnts = data

        # Otherwise we don't overwrite the avoidance waypoints
        else:
            pass

    def graphbased_wpts_cb(self, data):
        arr = np.asarray(data.data)
        self.graph_based_wpts = arr.reshape(data.layout.dim[0].size, data.layout.dim[1].size)
        self.graph_based_action = data.layout.dim[0].label

    def frenet_planner_cb(self, data: WpntArray):
        """frenet planner waypoints"""
        self.frenet_wpnts = data

    def overtake_cb(self, data):
        """
        Callback function of overtake subscriber.

        Parameters
        ----------
        data
            Data received from overtake topic
        """
        self.overtake_wpnts = data.wpnts
        self.num_ot_points = len(self.overtake_wpnts)

    def dyn_param_cb(self, param: Parameter):
        """
        Notices the change in the State Machine parameters and sets
        """
        if param.name == "gb_ego_width_m":
            self.params.gb_ego_width_m = param.value
        elif param.name == "splini_ttl" and self.params.ot_planner == "spliner":
            self.params.splini_ttl = param.value
            self.splini_ttl_counter = int(self.params.splini_ttl * self.params.rate_hz)  # convert seconds to counter
        elif param.name == "splini_hyst_timer_sec":
            self.params.splini_hyst_timer_sec = param.value

        self.get_logger().info(f"[Local Planner] Parameter '{param.name}' was set to {param.value}")

    ######################################
    # ATTRIBUTES/CONDITIONS CALCULATIONS #
    ######################################
    def _check_close_to_raceline(self) -> bool:
        return np.abs(self.cur_d) < self.params.gb_ego_width_m  # [m]

    def _check_availability_splini_wpts(self) -> bool:
        if self.avoidance_wpnts is None:
            return False
        elif len(self.avoidance_wpnts.wpnts) == 0:
            return False
        # Say no to the ot line if the last switch was less than 0.75 seconds ago
        elif (
            abs(time_to_float(self.avoidance_wpnts.header.stamp) - time_to_float(self.avoidance_wpnts.last_switch_time))
            < self.params.splini_hyst_timer_sec
        ):
            self.get_logger().debug("[Local Planner]: Still too fresh into the switch...")
            return False
        else:
            # If the splinis are valid update the last valid ones
            self.last_valid_avoidance_wpnts = self.avoidance_wpnts.wpnts.copy()
            return True

    def get_splini_wpts(self) -> WpntArray:
        """Obtain the waypoints by fusing those obtained by spliner with the
        global ones. Return the fused waypoints starting at s=0.0.
        """
        splini_glob = self.glb_wpnts.copy()

        # Handle wrapping
        if self.last_valid_avoidance_wpnts is not None:
            if len(self.last_valid_avoidance_wpnts) > len(splini_glob):
                # avoidance over more than 1 lap
                splini_idxs = [
                    int(s % len(splini_glob))
                    for s in range(
                        int(self.last_valid_avoidance_wpnts[0].s_m / self.waypoints_dist + 0.5),
                        int((self.max_s + self.last_valid_avoidance_wpnts[0].s_m) / self.waypoints_dist + 0.5),
                    )
                ]

            elif self.last_valid_avoidance_wpnts[-1].s_m > self.last_valid_avoidance_wpnts[0].s_m:
                splini_idxs = [
                    s
                    for s in range(
                        int(self.last_valid_avoidance_wpnts[0].s_m / self.waypoints_dist + 0.5),
                        int(self.last_valid_avoidance_wpnts[-1].s_m / self.waypoints_dist + 0.5),
                    )
                ]
            else:
                splini_idxs = [
                    int(s % len(splini_glob))
                    for s in range(
                        int(self.last_valid_avoidance_wpnts[0].s_m / self.waypoints_dist + 0.5),
                        int((self.max_s + self.last_valid_avoidance_wpnts[-1].s_m) / self.waypoints_dist + 0.5),
                    )
                ]

            # note: no lock needed here, the default rclpy executor runs callbacks and the timer sequentially
            for i, s in enumerate(splini_idxs):
                splini_glob[s] = self.last_valid_avoidance_wpnts[min(i, len(self.last_valid_avoidance_wpnts) - 1)]

        # If the last valid points have been reset, then we just pass the global waypoints
        else:
            self.get_logger().warn("[Local Planner] No valid avoidance waypoints, passing global waypoints")
            pass

        return splini_glob

    def get_graph_based_wpts(self) -> WpntArray:
        waypoint_arr = WpntArray()
        # Fill waypoint and marker array
        for i, coord in enumerate(self.graph_based_wpts[:self.params.n_loc_wpnts, :]):
            wpnt = Wpnt()
            wpnt.s_m = (coord[0] + self.cur_s) % self.max_s
            wpnt.x_m = coord[1]
            wpnt.y_m = coord[2]
            wpnt.d_m = 0.0
            wpnt.psi_rad = coord[3]
            wpnt.kappa_radpm = coord[4]
            wpnt.vx_mps = coord[5]
            wpnt.ax_mps2 = coord[6]
            wpnt.id = i
            # Get index of closest global waypoint in terms of s-coordinate
            idx = np.abs(self.gb_wpnts_arr[:, 0] - wpnt.s_m).argmin()
            # Get left and right distances to track bounds from the global waypoint
            d_left = self.gb_wpnts_arr[idx, 5]
            d_right = self.gb_wpnts_arr[idx, 4]
            # Use this information together with the d coordinate of the local waypoint in order to calculate
            # left and right track bounds distances of the local waypoint
            wpnt.d_left = d_left - wpnt.d_m
            wpnt.d_right = d_right + wpnt.d_m
            waypoint_arr.wpnts.append(wpnt)

        return waypoint_arr

    def _check_on_spline(self) -> bool:
        if self.last_valid_avoidance_wpnts is not None:
            # Check if section goes over end of track
            if self.last_valid_avoidance_wpnts[0].s_m > self.last_valid_avoidance_wpnts[-1].s_m:
                if self.cur_s > self.last_valid_avoidance_wpnts[0].s_m and self.cur_s < self.max_s:
                    return True
                elif self.cur_s < self.last_valid_avoidance_wpnts[-1].s_m and self.cur_s > 0:
                    return True
            else:
                if self.cur_s > self.last_valid_avoidance_wpnts[0].s_m and self.cur_s < self.last_valid_avoidance_wpnts[-1].s_m:
                    return True
        return False

    # Helpers
    def _find_nearest_ot_s(self) -> float:
        half_search_dim = 5

        # create indices
        idxs = [
            i % self.num_ot_points for i in range(self.cur_id_ot - half_search_dim, self.cur_id_ot + half_search_dim)
        ]
        ses = np.array([self.overtake_wpnts[i].s_m for i in idxs])

        dists = np.abs(self.cur_s - ses)
        chose_id = np.argmin(dists)
        s_ot = idxs[chose_id]
        s_ot %= self.num_ot_points

        return s_ot

    def get_map_to_odom(self):
        tf = self.tf_buffer.lookup_transform("odom", "map", Time(), timeout=Duration(seconds=5.0))
        t = tf.transform.translation
        r = tf.transform.rotation
        return [t.x, t.y, t.z], [r.x, r.y, r.z, r.w]

    def _pub_local_wpnts(self, wpts: WpntArray):
        loc_markers = MarkerArray()
        loc_wpnts = wpts
        loc_wpnts.header.stamp = self.get_clock().now().to_msg()
        loc_wpnts.header.frame_id = "odom"

        trans, rot = self.get_map_to_odom()
        T = concatenate_matrices(translation_matrix(trans), quaternion_matrix(rot))

        for i, wpnt in enumerate(loc_wpnts.wpnts):
            # transform from map to odom
            p_map = [wpnt.x_m, wpnt.y_m, 0.0, 1.0]
            p_odom = T.dot(p_map)

            wpnt.x_m = float(p_odom[0])
            wpnt.y_m = float(p_odom[1])

            mrk = Marker()
            mrk.header.frame_id = "odom"
            mrk.type = mrk.SPHERE
            mrk.scale.x = 0.15
            mrk.scale.y = 0.15
            mrk.scale.z = 0.15
            mrk.color.a = 1.0
            mrk.color.g = 1.0

            mrk.id = i
            mrk.pose.position.x = wpnt.x_m
            mrk.pose.position.y = wpnt.y_m
            mrk.pose.position.z = wpnt.vx_mps / self.max_speed  # Visualise speed in z dimension
            mrk.pose.orientation.w = 1.0
            loc_markers.markers.append(mrk)

        if len(loc_wpnts.wpnts) == 0:
            self.get_logger().warn("[Local Planner] No local waypoints published...", throttle_duration_sec=1.0)
        else:
            self.loc_wpnt_pub.publish(loc_wpnts)

        self.vis_loc_wpnt_pub.publish(loc_markers)

    def publish_del_marker(self):
        """Publishes a marker that deletes the previous markers"""
        mrk = MarkerArray()
        mrk.markers.append(Marker())
        mrk.markers[0].action = Marker.DELETEALL
        self.del_marker_pub.publish(mrk)

    def update_pose_from_tf(self):
        tf = self.tf_buffer.lookup_transform("map", "car_state/base_link", Time(), timeout=Duration(seconds=5.0))
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        q = tf.transform.rotation
        theta = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

        self.current_position = [x, y, theta]

    def update_frenet_from_pose(self):
        x, y, _ = self.current_position

        xs = np.array([w.x_m for w in self.glb_wpnts])
        ys = np.array([w.y_m for w in self.glb_wpnts])

        d2 = (xs - x)**2 + (ys - y)**2
        idx = np.argmin(d2)
        w = self.glb_wpnts[idx]  # nearest point on track
        self.cur_s = w.s_m

        ex = x - w.x_m
        ey = y - w.y_m
        self.cur_d = -np.sin(w.psi_rad) * ex + np.cos(w.psi_rad) * ey

        if self.num_ot_points != 0:
            self.cur_id_ot = int(self._find_nearest_ot_s())

    ########################################################
    # Functions to calculate local waypoints in Odom frame #
    ########################################################
    def GlobalTracking(self):
        s = int(self.cur_s / self.waypoints_dist + 0.5)
        return [self.glb_wpnts[(s + i) % self.num_glb_wpnts] for i in range(self.params.n_loc_wpnts)]

    def Trailing(self):
        if self._check_close_to_raceline():
            s = int(self.cur_s / self.waypoints_dist + 0.5)
            return [self.glb_wpnts[(s + i) % self.num_glb_wpnts] for i in range(self.params.n_loc_wpnts)]
        elif (self.params.ot_planner == "spliner" or self.params.ot_planner == "predictive_spliner") and self.last_valid_avoidance_wpnts is not None:
            # This allows us to trail on the last valid spline if necessary
            splini_wpts = self.get_splini_wpts()
            s = int(self.cur_s / self.waypoints_dist + 0.5)
            return [splini_wpts[(s + i) % self.num_glb_wpnts] for i in range(self.params.n_loc_wpnts)]
        else:
            s = int(self.cur_s / self.waypoints_dist + 0.5)
            return [self.glb_wpnts[(s + i) % self.num_glb_wpnts] for i in range(self.params.n_loc_wpnts)]

    def Overtaking(self):
        if (self.params.ot_planner == "spliner" or self.params.ot_planner == "predictive_spliner"):
            splini_wpts = self.get_splini_wpts()
            s = int(self.cur_s / self.waypoints_dist + 0.5)
            return [splini_wpts[(s + i) % self.num_glb_wpnts] for i in range(self.params.n_loc_wpnts)]
        elif self.params.ot_planner == "graph_based":
            try:
                return [wpnt for wpnt in self.get_graph_based_wpts().wpnts]
            except BaseException:
                return []
        elif self.params.ot_planner == "frenet":
            frenet_wpnts = self.frenet_wpnts
            return [wpnt for wpnt in frenet_wpnts.wpnts]
        else:
            s = self.cur_id_ot
            return [self.overtake_wpnts[(s + i) % self.num_ot_points] for i in range(self.params.n_loc_wpnts)]

    def FTGOnly(self):
        return []

    def loop(self):
        """
        Main loop of the local planner.
        """
        # update position
        self.update_pose_from_tf()
        self.update_frenet_from_pose()

        # decrease splini ttl counter used to cache the splini waypoints, once 0
        # it gets overwritten in case of empty avoidance
        if self.params.ot_planner == "predictive_spliner":

            self.splini_ttl_counter -= 1
            # Once ttl has reached 0 we overwrite the avoidance waypoints with the empty waypoints
            if self.splini_ttl_counter <= 0:
                if not self._check_on_spline():
                    self.avoidance_wpnts = None
                    self.last_valid_avoidance_wpnts = None
                    if self.params.ot_planner == "predictive_spliner":
                        self.publish_del_marker()
                elif self.splini_ttl_counter <= -int(self.params.pred_splini_ttl * self.params.rate_hz) * 3:
                    self.avoidance_wpnts = None
                    self.splini_ttl_counter = -10

        # decrease splini ttl counter used to cache the splini waypoints, once 0
        # it gets overwritten in case of empty avoidance
        if self.params.ot_planner == "spliner":
            self.splini_ttl_counter -= 1
            # Once ttl has reached 0 we overwrite the avoidance waypoints with the empty waypoints
            if self.splini_ttl_counter <= 0:
                self.last_valid_avoidance_wpnts = None
                self.avoidance_wpnts = WpntArray()
                self.splini_ttl_counter = -1

        # get the proper local waypoints based on the new state
        self.local_wpnts.wpnts = self.state_functions[self.cur_state]()
        self._pub_local_wpnts(self.local_wpnts)


def main(args=None):
    rclpy.init(args=args)

    local_planner = LocalPlanner()
    rclpy.spin(local_planner)

    local_planner.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

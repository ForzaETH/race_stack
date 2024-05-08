import yaml
import rclpy
from rclpy.node import Node
from f110_msgs.msg import Wpnt, WpntArray
import numpy as np
from ament_index_python.packages import get_package_share_directory
import os
from visualization_msgs.msg import MarkerArray, Marker
from scipy.interpolate import InterpolatedUnivariateSpline as Spline
import copy
from frenet_conversion.frenet_converter import FrenetConverter
import tqdm
from tf_transformations import quaternion_from_euler
from stack_master.parameter_event_handler import ParameterEventHandler

#from dynamic_reconfigure.msg import Config

class OvertakingInterpolator(Node):
    """
    Trajectory interpolator to merge the global waypoints with the shortest path
    """
    def __init__(self):
        super().__init__('ot_interpolator',
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)

        # sectors params
        self.glb_wpnts_og = None
        self.glb_wpnts_scaled = None
        self.interp_wpnt = None
        self.need_to_reinterpolate = True
        self.max_gb_speed = 10

        # get initial scaling
        self.sectors_params=self.parameters_to_dict()
        self.n_sectors = self.sectors_params['n_sectors']
        self.get_logger().info(str(self.sectors_params))
        self.yeet_factor = self.sectors_params['yeet_factor']
        self.spline_len = int(self.sectors_params['spline_len'])

        # SUBSCRIBE
        self.glb_wpnts_name = "/global_waypoints_scaled"
        self.glb_wpnts_sp_name = "/global_waypoints/shortest_path"
        # Add dynamic reconfigure
        self.handler = ParameterEventHandler(self)
        self.callback_handle = self.handler.add_parameter_event_callback(
            callback=self.dyn_param_cb,
        )
        self.glb_wpnt_scaled_sub = self.create_subscription(WpntArray, self.glb_wpnts_name, self.glb_wpnts_cb, 10)
        self.glb_wpnt_og_sub = self.create_subscription(WpntArray, "/global_waypoints", self.glb_wpnts_og_cb, 10)
        self.glb_wpnt_sp_sub = self.create_subscription(WpntArray, self.glb_wpnts_sp_name, self.glb_wpnts_sp_cb, 10)

        # new glb_waypoints
        self.ot_points_pub = self.create_publisher(WpntArray, "/global_waypoints/overtaking", 10)
        self.ot_markers_pub = self.create_publisher(MarkerArray, "/global_waypoints/overtaking/markers", 10)
        self.sector_pub = self.create_publisher(MarkerArray, '/ot_sector_markers', 10)

        timer_period = 0.5  # seconds
        self.wait_for_message_timer = self.create_timer(timer_period, self.wait_for_message_callback)
        self.get_logger().info("Wating on first messages from global waypoints topics (og, scaled, and shortest path).")
        
    def parameters_to_dict(self):
        params = {}
        for key in self._parameters:
            keylist = key.split('.')
            paramit = params
            for subkey in keylist[:-1]:
                paramit = paramit.setdefault(subkey, {})
            paramit[keylist[-1]] = self.get_parameter(key).value
        return params

    def glb_wpnts_og_cb(self, data:WpntArray):
        """
        Saves the global waypoints (minimum curvature)
        """
        self.waypoints = np.array([[wpnt.x_m, wpnt.y_m, wpnt.psi_rad] for wpnt in data.wpnts])
        self.glb_wpnts_og = data
        self.max_gb_speed = np.max(np.array([wpnt.vx_mps for wpnt in data.wpnts]))

    def glb_wpnts_cb(self, data:WpntArray):
        """
        Saves the global waypoints (minimum curvature)
        """
        self.glb_wpnts_scaled = data
        coords = np.array(
            [[wpnt.s_m, wpnt.x_m, wpnt.y_m] for wpnt in data.wpnts]
        )
        self.glb_spline_np = coords[:, 1:3]
        self.glb_spline_x = Spline(coords[:, 0], coords[:, 1])
        self.glb_spline_y = Spline(coords[:, 0], coords[:, 2])

    def glb_wpnts_sp_cb(self, data:WpntArray):
        """
        Saves the global waypoints (shortest path)
        """
        self.glb_wpnts_sp_og = data
        coords = np.array(
            [[wpnt.s_m, wpnt.x_m, wpnt.y_m, wpnt.vx_mps] for wpnt in data.wpnts]
        )
        self.ot_spline_np = coords[:, 1:3]
        self.ot_spline_x = Spline(coords[:, 0], coords[:, 1])
        self.ot_spline_y = Spline(coords[:, 0], coords[:, 2])
        self.ot_spline_speed = Spline(coords[:, 0], coords[:, 3])

    def wait_for_message_callback(self):
        if(self.glb_wpnts_scaled is None):
            return
        if(self.glb_wpnts_og is None):
            return
        if(self.glb_wpnts_sp_og is None):
            return
        self.get_logger().info("All messages received at least once!")

        # Finish initialization that depends on message content
        self.wait_for_message_timer.destroy()
        self.converter = self.initialize_converter()
        self.get_logger().info("Entering main loop!")
        timer_period = 0.5  # seconds
        self.wait_for_message_timer = self.create_timer(timer_period, self.loop_cb)
        
    def dyn_param_cb(self, parameter_event):
        if(parameter_event.node == '/sector_tuner'):
            self.need_to_reinterpolate = True
            self.glb_wpnts_scaled = None
            return
        if(parameter_event.node == '/ot_interpolator'):
            """
            Notices the change in the parameters and scales the global waypoints
            """
            # update params
            self.sectors_params=self.parameters_to_dict()
            self.need_to_reinterpolate = True
            self.get_logger().info(str(self.sectors_params))            
        # Else
        return

    def initialize_converter(self) -> FrenetConverter:
        """
        Initialize the FrenetConverter object
        """
        converter = FrenetConverter(self.waypoints[:, 0], self.waypoints[:, 1], self.waypoints[:, 2])
        self.get_logger().info("[OT Interpolator] initialized FrenetConverter object")
        return converter

    def get_interpolating_schedule(self):
        length_wpnts = len(self.glb_wpnts_og.wpnts)
        schedule = np.zeros(length_wpnts)
        # this is the switching sector length, longer is smoother, unit is 0.1 m
        switching_sec_len = self.spline_len

        for i in range(self.n_sectors):
            sector = f"Overtaking_sector{i}"
            if self.sectors_params[sector]['ot_flag']:
                if self.prev_sector_overtaking(i):
                    schedule[self.sectors_params[sector]['start']:self.sectors_params[sector]['start']+switching_sec_len-1] = 1
                else:
                    for j in range(switching_sec_len):
                        schedule[self.sectors_params[sector]['start']+j] = self.interpolating_function(j, switching_sec_len, mode='sigmoid')

                if self.next_sector_overtaking(i):
                    schedule[self.sectors_params[sector]['end']-switching_sec_len+1:self.sectors_params[sector]['end']+1] = 1
                else:
                    for j in range(1, switching_sec_len+1):
                        schedule[self.sectors_params[sector]['end']-j] = self.interpolating_function(j, switching_sec_len, mode='sigmoid')

                schedule[self.sectors_params[sector]['start']+switching_sec_len-1:self.sectors_params[sector]['end']-switching_sec_len+1] = 1

        return schedule

    def interpolating_function(self, idx: int, schedule_len: int, mode: str='linear'):
        """
        Defines how we go from zero to one in the interpolation of trajectories
        So it should be a function that goes from zero when called with idx = 0
        to one when called with idx=schedule_len
        """
        sig_scale = 4

        if mode == 'linear':
            return idx/schedule_len
        elif mode == 'sigmoid':
            sigmoid_idx = (idx/schedule_len)*sig_scale*2 - sig_scale
            return 1/(1 + np.exp(-sigmoid_idx))
        else:
            raise NotImplementedError(f"The mode '{mode}' is not implemented")

    def prev_sector_overtaking(self, idx):
        if idx == 0:
            return self.sectors_params[f"Overtaking_sector{self.n_sectors-1}"]['ot_flag']
        else:
            return self.sectors_params[f"Overtaking_sector{idx-1}"]['ot_flag']

    def next_sector_overtaking(self, idx):
        if idx == (self.n_sectors-1):
            return self.sectors_params[f"Overtaking_sector0"]['ot_flag']
        else:
            return self.sectors_params[f"Overtaking_sector{idx+1}"]['ot_flag']

    def interpolate_line(self, schedule):
        """
        Interpolates the point between the two trajectories according to the given schedule

        Parameters
        ----------
        start_s
            s parameter indicating the start of where we want to interpolate
        end_s
            s parameter indicating the end of where we want to interpolate
        """
        ses = []
        last_point = None
        nearest_p_idx = None
        self.interp_wpnt = copy.deepcopy(self.glb_wpnts_scaled)

        for i, waypoint in tqdm.tqdm(enumerate(self.interp_wpnt.wpnts)):
            if schedule[i] > 0:
                # find direction of global
                coordinate = np.array([self.glb_wpnts_scaled.wpnts[i].x_m, self.glb_wpnts_scaled.wpnts[i].y_m])
                direction = np.array([self.glb_spline_x(i/10, 1), self.glb_spline_y(i/10, 1)])
                direction /= np.linalg.norm(direction)

                # find nearest point
                nearest_p_idx = np.argmin((np.linalg.norm((self.ot_spline_np-coordinate), axis=1)))

                # find point perpendicular
                start_idx = nearest_p_idx
                idx_fwd = start_idx
                idx_bwd = start_idx
                min_perp_fwd = None
                min_perp_bwd = None
                #  look forward
                for _ in range(50):
                    point_on_mindist = np.array([self.ot_spline_x(idx_fwd/10), self.ot_spline_y(idx_fwd/10)])
                    vector = point_on_mindist-coordinate
                    vector /= np.linalg.norm(vector)
                    perp_fwd = np.abs(np.dot(direction, vector))
                    if min_perp_fwd is None:
                        min_perp_fwd = perp_fwd
                        point_fwd = copy.deepcopy(point_on_mindist)
                    else:
                        if perp_fwd < min_perp_fwd:
                            min_perp_fwd = perp_fwd
                            point_fwd = copy.deepcopy(point_on_mindist)
                        else:
                            break
                    idx_fwd += 0.01
                # look backward
                for _ in range(50):
                    point_on_mindist = np.array([self.ot_spline_x(idx_bwd/10), self.ot_spline_y(idx_bwd/10)])
                    vector = point_on_mindist-coordinate
                    vector /= np.linalg.norm(vector)
                    perp_bwd = np.abs(np.dot(direction, vector))
                    if min_perp_bwd is None:
                        min_perp_bwd = perp_bwd
                        point_bwd = copy.deepcopy(point_on_mindist)
                    else:
                        if perp_bwd < min_perp_bwd:
                            min_perp_bwd = perp_bwd
                            point_bwd = copy.deepcopy(point_on_mindist)
                        else:
                            break

                    idx_bwd -= 0.01

                if min_perp_bwd < min_perp_fwd:
                    point_chosen = point_bwd
                    idx_chosen = idx_bwd
                else:
                    point_chosen = point_fwd
                    idx_chosen = idx_fwd

                # interpolate
                new_point = schedule[i]*point_chosen + (1-schedule[i])*coordinate
                new_point_speed = self.glb_wpnts_scaled.wpnts[i].vx_mps*self.yeet_factor

                last_point = copy.deepcopy(new_point)
                waypoint.x_m = last_point[0]
                waypoint.y_m = last_point[1]
                waypoint.vx_mps = new_point_speed

    def recalculate_ot_s(self):
        # fit spline to whole new track, speed included
        # for s and d we take them relative to the global mincurv
        coords = np.inf*np.ones((len(self.interp_wpnt.wpnts), 4))
        s = 0
        i = 0
        for wpnt in self.interp_wpnt.wpnts:
            if i > 0:
                # for points different from the first we should not add if they're the same
                dist_from_previous = np.linalg.norm([wpnt.x_m - coords[i-1, 1], wpnt.y_m - coords[i-1, 2]])
                if dist_from_previous > 0:
                    s += dist_from_previous
                    coords[i, 0] = s
                    coords[i, 1] = wpnt.x_m
                    coords[i, 2] = wpnt.y_m
                    coords[i, 3] = wpnt.vx_mps
                    i+=1
            else:
                # default action for first point
                coords[i, 0] = s
                coords[i, 1] = wpnt.x_m
                coords[i, 2] = wpnt.y_m
                coords[i, 3] = wpnt.vx_mps
                i+=1
        tot_len = s + np.linalg.norm([coords[0, 1] - coords[i-1, 1], coords[0, 2] - coords[i-1, 2]])

        spline_x = Spline(coords[:i, 0], coords[:i, 1])
        spline_y = Spline(coords[:i, 0], coords[:i, 2])
        spline_speed = Spline(coords[:i, 0], coords[:i, 3])

        # rewrite whole array
        self.interp_wpnt = WpntArray()
        for i in range(0, int(tot_len*10)):
            new_wpnt = Wpnt()
            new_wpnt.id =  i
            new_wpnt.x_m = float(spline_x(i/10))
            new_wpnt.y_m = float(spline_y(i/10))
            new_wpnt.vx_mps = float(spline_speed(i/10))
            # curvature
            x_d = float(spline_x(i/10, 1))
            x_dd = float(spline_x(i/10, 2))
            y_d = float(spline_y(i/10, 1))
            y_dd = float(spline_y(i/10, 2))
            curvature = (
                np.abs(x_d * y_dd - y_d * x_dd)/pow((x_d**2 + y_d **2), 1.5)
            )
            new_wpnt.kappa_radpm = curvature
            # frenet components
            frenet_newpoint = self.converter.get_frenet(np.array([new_wpnt.x_m]), np.array([new_wpnt.y_m]))
            new_wpnt.s_m = float(frenet_newpoint[0])
            new_wpnt.d_m = float(frenet_newpoint[1])

            self.interp_wpnt.wpnts.append(copy.deepcopy(new_wpnt))

    def interpolate_points(self):
        """
        Scales the global waypoints'
        """
        # we need to wait the global waitpoint scaled, otherwise we might use old ones
        if self.glb_wpnts_scaled is None:
            return
        if self.need_to_reinterpolate:
            schedule =  self.get_interpolating_schedule()
            self.interpolate_line(schedule)
            self.recalculate_ot_s()

            self.need_to_reinterpolate = False
        pass

    def publish_spline_markers(self):
        max_vx_mps = self.max_gb_speed
        global_markers = MarkerArray()

        for i, wpnt in enumerate(self.interp_wpnt.wpnts):
            global_marker = Marker()
            global_marker.header.frame_id = 'map'
            global_marker.type = global_marker.CYLINDER
            global_marker.scale.x = 0.1
            global_marker.scale.y = 0.1
            global_marker.scale.z = wpnt.vx_mps / max_vx_mps
            global_marker.color.a = 1.0
            global_marker.color.r = 0.0
            global_marker.color.g = 0.0
            global_marker.color.b = 1.0

            global_marker.id = i
            global_marker.pose.position.x = wpnt.x_m
            global_marker.pose.position.y = wpnt.y_m
            global_marker.pose.position.z = wpnt.vx_mps / max_vx_mps / 2
            global_marker.pose.orientation.w = 1.0
            global_markers.markers.append(global_marker)

        self.ot_markers_pub.publish(global_markers)
        
    def publish_sector_markers(self):
        global_waypoints_vis = []
        for waypoint in self.glb_wpnts_og.wpnts:
            global_waypoints_vis.append([waypoint.x_m, waypoint.y_m, waypoint.s_m])
        # one needs to set the overtaking_map_params rosparams
        sec_markers = MarkerArray()

        for i in range(self.n_sectors):
            s = self.sectors_params[f"Overtaking_sector{i}"]['start']
            if s == (len(global_waypoints_vis) - 1):
                theta = np.arctan2((global_waypoints_vis[0][1] - global_waypoints_vis[s][1]),(global_waypoints_vis[0][0] - global_waypoints_vis[s][0]))
            else:
                theta = np.arctan2((global_waypoints_vis[s+1][1] - global_waypoints_vis[s][1]),(global_waypoints_vis[s+1][0] - global_waypoints_vis[s][0]))
            quaternions = quaternion_from_euler(0, 0, theta)
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.type = marker.ARROW
            marker.scale.x = 0.5
            marker.scale.y = 0.05
            marker.scale.z = 0.15
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.pose.position.x = global_waypoints_vis[s][0]
            marker.pose.position.y = global_waypoints_vis[s][1]
            marker.pose.position.z = 0.0
            marker.pose.orientation.x = quaternions[0]
            marker.pose.orientation.y = quaternions[1]
            marker.pose.orientation.z = quaternions[2]
            marker.pose.orientation.w = quaternions[3]
            marker.id = i
            sec_markers.markers.append(marker)

            marker_text = Marker()
            marker_text.header.frame_id = "map"
            marker_text.header.stamp = self.get_clock().now().to_msg()
            marker_text.type = marker_text.TEXT_VIEW_FACING
            marker_text.text = f"Start Overtaking Sector {i}"
            marker_text.scale.z = 0.4
            marker_text.color.r = 0.1
            marker_text.color.g = 0.1
            marker_text.color.b = 1.2
            marker_text.color.a = 1.0
            marker_text.pose.position.x = global_waypoints_vis[s][0]
            marker_text.pose.position.y = global_waypoints_vis[s][1]
            marker_text.pose.position.z = 1.5
            marker_text.pose.orientation.x = 0.0
            marker_text.pose.orientation.y = 0.0
            marker_text.pose.orientation.z = 0.0436194
            marker_text.pose.orientation.w = 0.9990482
            marker_text.id = i + self.n_sectors
            sec_markers.markers.append(marker_text)
        self.sector_pub.publish(sec_markers)    

    def loop_cb(self):
        self.interpolate_points()
        self.ot_points_pub.publish(self.interp_wpnt)
        self.publish_spline_markers()
        self.publish_sector_markers()


def main():
    rclpy.init()
    node = OvertakingInterpolator()
    rclpy.spin(node)
    rclpy.shutdown()

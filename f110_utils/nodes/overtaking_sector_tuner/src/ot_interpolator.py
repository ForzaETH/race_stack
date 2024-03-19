#! /usr/bin/env python3

import copy
import tqdm
from tracemalloc import start
import rospy
import numpy as np
import matplotlib.pyplot as plt
from f110_msgs.msg import Wpnt, WpntArray
from dynamic_reconfigure.msg import Config
from scipy.interpolate import InterpolatedUnivariateSpline as Spline
from visualization_msgs.msg import Marker, MarkerArray
from frenet_converter.frenet_converter import FrenetConverter


class OvertakingInterpolator:
    """
    Trajectory interpolator to merge the global waypoints with the shortest path
    """

    def __init__(self, debug_plot: bool = False) -> None:
        self.node_name = "ot_interpolator"
        rospy.init_node(self.node_name)

        # sectors params
        self.glb_wpnts_og = None
        self.glb_wpnts_scaled = None
        self.interp_wpnt = None
        self.need_to_reinterpolate = True
        self.max_gb_speed = 10
        
        # get initial scaling
        self.sectors_params = rospy.get_param("/ot_map_params")
        rospy.loginfo(self.sectors_params)
        self.n_sectors = self.sectors_params['n_sectors']
        self.yeet_factor = rospy.get_param("/ot_map_params/yeet_factor")
        self.spline_len = int(rospy.get_param("/ot_interpolator/spline_len", 30))

        # SUBSCRIBE
        self.glb_wpnts_name = "/global_waypoints_scaled"
        self.glb_wpnts_sp_name = "/global_waypoints/shortest_path"
        rospy.Subscriber("/ot_dyn_sector_server/parameter_updates", Config, self.dyn_param_cb)
        rospy.Subscriber("/dyn_sector_server/parameter_updates", Config, self.dyn_speed_param_cb)
        rospy.Subscriber(self.glb_wpnts_name, WpntArray, self.glb_wpnts_cb)
        rospy.Subscriber("/global_waypoints", WpntArray, self.glb_wpnts_og_cb)
        rospy.Subscriber(self.glb_wpnts_sp_name, WpntArray, self.glb_wpnts_sp_cb)
        
        # rospy.loginfo(f"[{self.node_name}] Waiting for the frenet conversion service...")
        # rospy.wait_for_service("/convert_glob2frenet_service")
        # rospy.loginfo(f"[{self.node_name}] Frenet conversion service found!")
        # self.g2f_converter = rospy.ServiceProxy("/convert_glob2frenet_service", Glob2Frenet)

        # new glb_waypoints
        self.ot_points_pub = rospy.Publisher("/global_waypoints/overtaking", WpntArray, queue_size=10)
        self.ot_markers_pub = rospy.Publisher("/global_waypoints/overtaking/markers", MarkerArray, queue_size=10)

        self.converter = self.initialize_converter()


    def glb_wpnts_og_cb(self, data:WpntArray):
        """
        Saves the global waypoints (minimum curvature)
        """
        self.waypoints = np.array([[wpnt.x_m, wpnt.y_m] for wpnt in data.wpnts])
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

    def dyn_param_cb(self, params:Config):
        """
        Notices the change in the parameters and scales the global waypoints
        """
        # update params 
        for param in params.bools:
            self.sectors_params[param.name]['ot_flag'] = param.value
        
        #assert params.doubles[0].name == 'yeet_factor'
        self.yeet_factor = params.doubles[0].value
        self.spline_len = int(params.doubles[1].value) 
        self.need_to_reinterpolate = True
        rospy.loginfo(self.sectors_params)

    def dyn_speed_param_cb(self, params:Config):
        self.need_to_reinterpolate = True

    def initialize_converter(self) -> FrenetConverter:
        """
        Initialize the FrenetConverter object"""
        rospy.wait_for_message("/global_waypoints", WpntArray)

        # Initialize the FrenetConverter object

        converter = FrenetConverter(self.waypoints[:, 0], self.waypoints[:, 1])
        rospy.loginfo("[OT Interpolator] initialized FrenetConverter object")

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

                # interpolate for speed
                # speed_scaling = self.glb_wpnts_scaled.wpnts[i].vx_mps/self.glb_wpnts_og.wpnts[i].vx_mps
                # new_point_speed = schedule[i]*self.ot_spline_speed(idx_chosen/10) + (1-schedule[i])*self.glb_wpnts_og.wpnts[i].vx_mps
                # new_point_speed *= speed_scaling*self.yeet_factor
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
        wpnts = []
        for i in range(0, int(tot_len*10)):
            new_wpnt = Wpnt()
            new_wpnt.id =  i
            new_wpnt.x_m = spline_x(i/10)
            new_wpnt.y_m = spline_y(i/10)
            new_wpnt.vx_mps = spline_speed(i/10)
            # curvature 
            x_d = spline_x(i/10, 1)
            x_dd = spline_x(i/10, 2)
            y_d = spline_y(i/10, 1)
            y_dd = spline_y(i/10, 2)
            curvature = (
                np.abs(x_d * y_dd - y_d * x_dd)/pow((x_d**2 + y_d **2), 1.5)
            )
            new_wpnt.kappa_radpm = curvature
            # frenet components
            frenet_newpoint = self.converter.get_frenet(np.array([new_wpnt.x_m]), np.array([new_wpnt.y_m]))
            new_wpnt.s_m = frenet_newpoint[0]
            new_wpnt.d_m = frenet_newpoint[1]

            self.interp_wpnt.wpnts.append(copy.deepcopy(new_wpnt))

    def interpolate_points(self):
        """
        Scales the global waypoints' 
        """
        if self.need_to_reinterpolate:
            # we need to wait the global waitpoint scaled, otherwise we might use old ones
            rospy.wait_for_message(self.glb_wpnts_name, WpntArray)
            schedule =  self.get_interpolating_schedule()
            self.interpolate_line(schedule)
            self.recalculate_ot_s()
            
            self.need_to_reinterpolate = False
        pass

    def publish_markers(self):
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
            global_marker.pose.orientation.w = 1
            global_markers.markers.append(global_marker)

        self.ot_markers_pub.publish(global_markers)

    def loop(self):
        rospy.loginfo(f"[{self.node_name}] Waiting for global waypoints...")
        rospy.loginfo(f"[{self.node_name}] Waiting for shortest path waypoints...")
        rospy.wait_for_message(self.glb_wpnts_name, WpntArray)        
        rospy.loginfo(f"[{self.node_name}] Global waypoints received!")
        rospy.wait_for_message(self.glb_wpnts_sp_name, WpntArray)        
        rospy.loginfo(f"[{self.node_name}] Shortest path waypoints received!")

        # initialise scaled points
        self.interpolate_points()

        run_rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            self.interpolate_points()
            self.ot_points_pub.publish(self.interp_wpnt)
            self.publish_markers()
            run_rate.sleep()

if __name__ == '__main__':
    ot_interpolator = OvertakingInterpolator()
    ot_interpolator.loop()

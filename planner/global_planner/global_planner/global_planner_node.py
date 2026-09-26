#!/usr/bin/env python3

import os
from pathlib import Path

import yaml
import cv2
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from skimage.morphology import skeletonize

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from f110_msgs.msg import WpntArray

import trajectory_planning_helpers as tph
from .readwrite_global_waypoints import write_global_waypoints
from global_racetrajectory_optimization.trajectory_optimizer import trajectory_optimizer
from global_racetrajectory_optimization import helper_funcs_glob
from .global_planner_utils import extract_centerline, \
    smooth_centerline, \
    extract_track_bounds, \
    dist_to_bounds, \
    add_dist_to_cent, \
    write_centerline, \
    publish_track_bounds, \
    create_wpnts_markers, \
    compare_direction


class GlobalPlannerNode(Node):
    """Global planner node"""
    
    def __init__(self):
        super().__init__('global_planner_node', allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)

        self.map_dir = Path(self.get_parameter('map_path').value)
        self.map_name = self.map_dir.name
        self.reverse_mapping = self.get_parameter('reverse_mapping').value
        self.show_plots = self.get_parameter('show_plots').value
        self.safety_width = self.get_parameter('safety_width').value
        self.safety_width_sp = self.get_parameter('safety_width_sp').value

        
        self.map_resolution = None
        self.watershed = True
        self.map_info_str = ""
        self.input_path = os.path.join(get_package_share_directory('stack_master'), 'config', 'global_planner')
        
        self.global_waypoints_pub = self.create_publisher(WpntArray, '/global_waypoints', 10)
        self.global_waypoints_sp_pub = self.create_publisher(WpntArray, '/global_waypoints/shortest_path', 10)
        self.global_waypoints_markers_pub = self.create_publisher(MarkerArray, '/global_waypoints/markers', 10)
        self.centerline_waypoints_markers_pub = self.create_publisher(MarkerArray, '/centerline_waypoints/markers', 10)
        self.track_bounds_pub = self.create_publisher(MarkerArray, '/trackbounds/markers', 10)
        self.shortest_path_waypoints_markers_pub = self.create_publisher(
            MarkerArray, '/global_waypoints/shortest_path/markers', 10)
        
        self.load_and_plot_map()
    
    
    def publish_plan(self,
                     global_waypoints: WpntArray,
                     global_waypoints_marker: MarkerArray,
                     centerline_waypoints_marker: MarkerArray,
                     global_waypoints_sp: WpntArray,
                     shortest_path_waypoints_marker: MarkerArray,
                     track_bounds_marker: MarkerArray
                     ) -> None:
        """Publishes the global plan as a MarkerArray message."""
        
        self.global_waypoints_markers_pub.publish(global_waypoints_marker)
        self.centerline_waypoints_markers_pub.publish(centerline_waypoints_marker)
        self.shortest_path_waypoints_markers_pub.publish(shortest_path_waypoints_marker)
        self.track_bounds_pub.publish(track_bounds_marker)
        self.global_waypoints_pub.publish(global_waypoints)
        self.global_waypoints_sp_pub.publish(global_waypoints_sp)

    def load_and_plot_map(self):
        """Loads the map from disk, applies filtering and plots it."""
        self.get_logger().info(f"Loading map: {self.map_name}")
        img_path = os.path.join(self.map_dir, self.map_name + '.png')
        self.filtered_map = cv2.flip(cv2.imread(img_path, 0), 0)
        skeleton = skeletonize(self.filtered_map, method='lee')
        map_info = yaml.safe_load(open(os.path.join(self.map_dir, self.map_name + '.yaml'), 'r'))
        self.map_resolution = map_info['resolution']
        self.map_origin = Point()
        self.map_origin.x = map_info['origin'][0]
        self.map_origin.y = map_info['origin'][1]
        self.initial_position = map_info['initial_pose']
        self.skeleton = skeleton
        try:
            f, (ax0, ax1) = plt.subplots(2, 1)
            f.suptitle(f"Map [{self.map_name}]: Filtered map versus morphological skeleton")
            ax0.imshow(self.filtered_map, cmap='gray')
            ax1.imshow(skeleton, cmap='gray')
            plt.show()
        except Exception as e:
            self.get_logger().warn(f"Could not display map plots, error: {e}")
            
    def plan(self):
        """ Main function: Load, Process, Publish """

        self.get_logger().info("Extracting centerline...")
        centerline = extract_centerline(
                skeleton=self.skeleton,
                cent_length=0.0,
                map_resolution=self.map_resolution)
        centerline_smooth = smooth_centerline(centerline)
        centerline_meter = np.zeros(np.shape(centerline_smooth))
        centerline_meter[:, 0] = centerline_smooth[:, 0] * self.map_resolution + self.map_origin.x
        centerline_meter[:, 1] = centerline_smooth[:, 1] * self.map_resolution + self.map_origin.y

        # interpolate centerline to 0.1m stepsize: less computation needed later for distance to track bounds
        centerline_meter = np.column_stack((centerline_meter, np.zeros((centerline_meter.shape[0], 2))))

        centerline_meter_int = helper_funcs_glob.src.interp_track.interp_track(reftrack=centerline_meter,
                                                                               stepsize_approx=0.1)[:, :2]

        # get distance to initial position for every point on centerline
        cent_distance = np.sqrt(np.power(centerline_meter_int[:, 0] - self.initial_position[0], 2)
                                + np.power(centerline_meter_int[:, 1] - self.initial_position[1], 2))

        min_dist_ind = np.argmin(cent_distance)

        cent_direction = np.angle([complex(centerline_meter_int[min_dist_ind, 0] -
                                           centerline_meter_int[min_dist_ind - 1, 0],
                                           centerline_meter_int[min_dist_ind, 1] -
                                           centerline_meter_int[min_dist_ind - 1, 1])])

        self.get_logger().info(f"Direction of the centerline: {cent_direction[0]}")
        if self.show_plots:
            plt.plot(centerline_meter_int[:, 0], centerline_meter_int[:, 1], 'ko', label='Centerline interpolated')
            plt.plot(centerline_meter_int[min_dist_ind - 1, 0], centerline_meter_int[min_dist_ind - 1, 1], 'ro',
                        label='First point')
            plt.plot(centerline_meter_int[min_dist_ind, 0], centerline_meter_int[min_dist_ind, 1], 'bo',
                        label='Second point')
            plt.legend()
            plt.gca().set_aspect('equal', adjustable='box')
            plt.grid()
            plt.show()
        
        # flip centerline if directions don't match
        if not compare_direction(cent_direction, self.initial_position[2]):
            centerline_smooth = np.flip(centerline_smooth, axis=0)
            centerline_meter_int = np.flip(centerline_meter_int, axis=0)

        # Flip again if necessary
        if self.reverse_mapping:
            centerline_smooth = np.flip(centerline_smooth, axis=0)
            centerline_meter_int = np.flip(centerline_meter_int, axis=0)
            self.get_logger().info('Centerline flipped')

        # extract track bounds
        try:
            self.get_logger().info('Using watershed for track bound extraction...')
            bound_r_water, bound_l_water = extract_track_bounds(centerline_smooth,
                                                                self.filtered_map,
                                                                map_resolution=self.map_resolution,
                                                                map_origin=self.map_origin,
                                                                initial_position=self.initial_position,
                                                                show_plots=self.show_plots)
            dist_transform = None
        except IOError:
            self.get_logger().warn('More than two track bounds detected with watershed algorithm')
            self.get_logger().info('Trying with simple distance transform...')
            self.watershed = False
            bound_r_water = None
            bound_l_water = None
            dist_transform = cv2.distanceTransform(self.filtered_map, cv2.DIST_L2, 5)
            

        ################################################################################################################
        # Compute global trajectory with mincurv_iqp optimization
        ################################################################################################################
        track_path_root = os.path.join(Path.home(), ".ros")
        iqp_centerline_path = os.path.join(track_path_root, 'map_centerline')
        sp_centerline_path = os.path.join(track_path_root, 'map_centerline_2')

        cent_with_dist = add_dist_to_cent(centerline_smooth=centerline_smooth,
                                          centerline_meter=centerline_meter_int,
                                          map_resolution=self.map_resolution,
                                          safety_width=self.safety_width,
                                          show_plots=self.show_plots,
                                          dist_transform=dist_transform,
                                          bound_r=bound_r_water,
                                          bound_l=bound_l_water,
                                          reverse=self.reverse_mapping)

        # Write centerline in a csv file and get a marker array of it
        centerline_waypoints, centerline_markers = write_centerline(cent_with_dist)

        # Add curvature and angle to centerline waypoints
        centerline_coords = np.array([
            [coord.x_m, coord.y_m] for coord in centerline_waypoints.wpnts
        ])

        psi_centerline, kappa_centerline = tph.calc_head_curv_num.\
            calc_head_curv_num(
                path=centerline_coords,
                el_lengths=0.1 * np.ones(len(centerline_coords) - 1),
                is_closed=False
            )
        for i, (psi, kappa) in enumerate(zip(psi_centerline, kappa_centerline)):
            centerline_waypoints.wpnts[i].s_m = i * 0.1
            # pi/2 added because trajectory_planning_helpers package assumes north to be zero psi
            centerline_waypoints.wpnts[i].psi_rad = psi + np.pi / 2
            centerline_waypoints.wpnts[i].kappa_radpm = kappa

        self.get_logger().info('Start Global Trajectory optimization with iterative minimum curvature...')
        try:
            global_trajectory_iqp, bound_r_iqp, bound_l_iqp, est_t_iqp = trajectory_optimizer(
                input_path=self.input_path, track_name=iqp_centerline_path, curv_opt_type='mincurv_iqp', safety_width=self.safety_width, plot=(
                    self.show_plots and not self.map_editor_mode))
        except RuntimeError as e:
            self.get_logger().warn(f"Error during iterative minimum curvature optimization, error: {e}")
            self.get_logger().info('Try again later!')
            return False

        self.map_info_str += f'IQP estimated lap time: {round(est_t_iqp, 4)}s; '
        self.map_info_str += f'IQP maximum speed: {round(np.amax(global_trajectory_iqp[:, 5]), 4)}m/s; '

        # do not use bounds of optimizer if the one's from the watershed algorithm are available
        if self.watershed:
            bound_r_iqp = bound_r_water
            bound_l_iqp = bound_l_water

        bounds_markers = publish_track_bounds(bound_r_iqp, bound_l_iqp, reverse=False)

        d_right_iqp, d_left_iqp = dist_to_bounds(trajectory=global_trajectory_iqp,
                                                 bound_r=bound_r_iqp,
                                                 bound_l=bound_l_iqp,
                                                 centerline=centerline_meter_int,
                                                 safety_width=self.safety_width,
                                                 show_plots=self.show_plots,
                                                 reverse=self.reverse_mapping)

        global_traj_wpnts_iqp, global_traj_markers_iqp = _create_wpnts_markers(trajectory=global_trajectory_iqp,
                                                                                   d_right=d_right_iqp,
                                                                                   d_left=d_left_iqp,
                                                                                   loginfo=self.get_logger().info)

        # publish global trajectory markers and waypoints
        self.get_logger().info('Done with iterative minimum curvature optimization')
        self.get_logger().info('Lap Completed now publishing global waypoints')

        ################################################################################################################
        # Compute global trajectory with shortest path optimization
        ################################################################################################################

        self.get_logger().info('Start reverse Global Trajectory optimization with shortest path...')

        self.get_logger().info('Start Global Trajectory optimization with iterative minimum curvature for overtaking...')
        global_trajectory_iqp_ot, *_ = trajectory_optimizer(input_path=self.input_path,
                                                            track_name=iqp_centerline_path,
                                                            curv_opt_type='mincurv_iqp',
                                                            safety_width=self.safety_width_sp,
                                                            plot=(self.show_plots and not self.map_editor_mode))

        # use new iqp path as centerline
        new_cent_with_dist = add_dist_to_cent(centerline_smooth=global_trajectory_iqp_ot[:, 1:3],
                                              centerline_meter=global_trajectory_iqp_ot[:, 1:3],
                                              map_resolution=self.map_resolution,
                                              safety_width=self.safety_width_sp,
                                              show_plots=self.show_plots,
                                              dist_transform=None,
                                              bound_r=bound_r_water,
                                              bound_l=bound_l_water,
                                              reverse=self.reverse_mapping)

        _, new_centerline_markers = write_centerline(new_cent_with_dist, sp_bool=True)

        # to use iqp as new centerline, set trackname='map_centerline_2', otherwise use track_name='map_centerline'
        # is a bit faster but cuts corner a bit more
        global_trajectory_sp, bound_r_sp, bound_l_sp, est_t_sp = trajectory_optimizer(
            input_path=self.input_path, track_name=sp_centerline_path, curv_opt_type='shortest_path', safety_width=self.safety_width_sp, plot=(
                self.show_plots and not self.map_editor_mode))

        self.est_lap_time = Float32()  # variable which will be published and used in l1_param_optimizer
        self.est_lap_time.data = est_t_sp

        self.map_info_str += f'SP estimated lap time: {round(est_t_sp, 4)}s; '
        self.map_info_str += f'SP maximum speed: {round(np.amax(global_trajectory_sp[:, 5]), 4)}m/s; '

        # do not use bounds of optimizer if the one's from the watershed algorithm are available
        if self.watershed:
            bound_r_sp = bound_r_water
            bound_l_sp = bound_l_water

        d_right_sp, d_left_sp = dist_to_bounds(trajectory=global_trajectory_sp,
                                               bound_r=bound_r_sp,
                                               bound_l=bound_l_sp,
                                               centerline=centerline_meter_int,
                                               safety_width=self.safety_width_sp,
                                               show_plots=self.show_plots,
                                               reverse=self.reverse_mapping)

        global_traj_wpnts_sp, global_traj_markers_sp = _create_wpnts_markers(trajectory=global_trajectory_sp,
                                                                                 d_right=d_right_sp,
                                                                                 d_left=d_left_sp,
                                                                                 loginfo=self.get_logger().info,
                                                                                 second_traj=True)

        # publish global trajectory markers and waypoints
        self.get_logger().info('Done with shortest path optimization')
        self.get_logger().info('Lap Completed now publishing shortest path global waypoints')
        self.publish_plan(global_waypoints=global_traj_wpnts_iqp,
                          global_waypoints_marker=global_traj_markers_iqp,
                          centerline_waypoints_marker=centerline_markers,
                          shortest_path_waypoints_marker=global_traj_markers_sp,
                            global_waypoints_sp=global_traj_wpnts_sp,
                          track_bounds_marker=bounds_markers)
        # Save info into a JSON file
        write_global_waypoints(
            self.map_dir,
            self.map_info_str,
            self.est_lap_time,
            centerline_markers,
            centerline_waypoints,
            global_traj_markers_iqp,
            global_traj_wpnts_iqp,
            global_traj_markers_sp,
            global_traj_wpnts_sp,
            bounds_markers
        )

def _create_wpnts_markers(trajectory: np.ndarray, 
                        d_right: np.ndarray,
                        d_left: np.ndarray,
                        loginfo = lambda x: None,
                        second_traj: bool = False) -> tuple[WpntArray, MarkerArray]:
    """
    Create and return a waypoint array and a marker array.

    Args:
        trajectory (np.ndarray): A trajectory with waypoints in the form [s_m, x_m, y_m, psi_rad, vx_mps, ax_mps2]
        d_right (np.ndarray): Distances to the right track bounds for every waypoint in `trajectory`
        d_left (np.ndarray): Distances to the left track bounds for every waypoint in `trajectory`
        second_traj (bool, optional): Display second trajectory with a different color than the first.
                                        Better for visualization. Defaults to False.

    Returns:
        tuple[WpntArray, MarkerArray]: A waypoint array and a marker array with all points of `trajectory`
    """
    max_vx_mps = max(trajectory[:, 5])
    speed_string = "Max speed: " + str(max_vx_mps)
    loginfo(speed_string)

    return create_wpnts_markers(trajectory, d_right, d_left, second_traj)
        

def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlannerNode()
    node.plan()
    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
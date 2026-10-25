#!/usr/bin/env python3

from pathlib import Path

import yaml
import cv2
import numpy as np
from skimage.morphology import skeletonize

import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from ament_index_python.packages import get_package_share_directory
from rs_msgs.msg import Wpnt, WpntArray

from trajectory_planning_helpers.calc_head_curv_num import calc_head_curv_num
from .plan import GlobalPlan
from global_racetrajectory_optimization.trajectory_optimizer import trajectory_optimizer
from global_racetrajectory_optimization import helper_funcs_glob
from .track_geometry import extract_centerline, \
    smooth_centerline, \
    extract_track_bounds, \
    dist_to_bounds, \
    add_dist_to_cent, \
    follows_driving_direction, \
    PlannerError
    
from rs_global_planner.parameters_codegen import global_planner as global_planner_parameters


class GlobalPlannerNode(Node):
    """
    Offline tool: computes the global plan of a map and stores it there (global_waypoints.json), then exits.

    Publishes nothing; the global plan server serves the stored plan.
    """
    
    def __init__(self):
        super().__init__('global_planner')

        self.params = global_planner_parameters.ParamListener(self).get_params()

        self.map_dir = Path(self.params.map_path)
        self.map_name = self.map_dir.name

        # Vehicle model and optimizer options of the trajectory optimizer
        self.input_path = Path(get_package_share_directory('stack_master')) / 'config' / 'global_planner'

        self.img, self.map_resolution, self.map_origin, self.initial_pose = self.load_map()
        plan = self.plan()
        plan.save(self.map_dir)
        self.get_logger().info(f'[GLOBAL_PLANNER] Global plan saved to {self.map_dir}')

    def load_map(self):
        """Load the map image (free = 255, flipped so row 0 is at the map origin) and its metadata."""
        
        img = cv2.imread(str(self.map_dir / f"{self.map_name}.png"), cv2.IMREAD_GRAYSCALE)
        if img is None:
            raise PlannerError(f"Cannot read map image in {self.map_dir}")
        
        info = yaml.safe_load((self.map_dir / f'{self.map_name}.yaml').read_text())

        return cv2.flip(img, 0), info['resolution'], np.array(info['origin'][:2]), info['initial_pose']
        
            
    def plan(self):
        """Compute the global plan (centerline, raceline, shortest path) from the map."""

        self.get_logger().info("Extracting centerline...")
        
        # Extract and smooth the centerline
        centerline_pixels = extract_centerline(skeletonize(self.img, method='lee'))
        centerline_smooth = smooth_centerline(centerline_pixels, self.map_resolution, self.params.centerline_smoothing_window_m)
        
        # Convert the centerline pixel coordinates to meters
        centerline_m = centerline_smooth * self.map_resolution + self.map_origin
        
        # Resample to an even 0.1 m spacing
        centerline = helper_funcs_glob.src.interp_track.interp_track(
            reftrack=np.column_stack((centerline_m, np.zeros((len(centerline_m), 2)))), stepsize_approx=0.1)[:, :2]

        # Order the centerline in the driving direction, judged by the heading at the start pose
        if not follows_driving_direction(centerline, self.initial_pose):
            centerline_smooth = np.flip(centerline_smooth, axis=0)
            centerline = np.flip(centerline, axis=0)

        # Track bounds from the driving-ordered centerline
        bound_right, bound_left = extract_track_bounds(centerline_smooth, self.img, self.map_resolution, self.map_origin)

        if self.params.reverse_mapping:
            centerline = np.flip(centerline, axis=0)
            self.get_logger().info('[GLOBAL_PLANNER] Centerline reversed')
            

        cent_with_dist = add_dist_to_cent(centerline=centerline,
                                          bound_r=bound_right,
                                          bound_l=bound_left,
                                          reverse=self.params.reverse_mapping)

        self.get_logger().info('[GLOBAL_PLANNER] Optimizing the raceline (iterative minimum curvature)...')
        raceline, _, _, est_t_iqp = trajectory_optimizer(
            input_path=self.input_path, reftrack=cent_with_dist, curv_opt_type='mincurv_iqp',
            safety_width=self.params.safety_width, plot=False)

        # The shortest path is optimized along the normals of a reference line; the raceline gives better spread normals
        # in corners than the centerline (which makes the shortest path cut corners)
        reference_with_dist = add_dist_to_cent(centerline=raceline[:, 1:3],
                                               bound_r=bound_right,
                                               bound_l=bound_left,
                                               reverse=self.params.reverse_mapping)

        self.get_logger().info('[GLOBAL_PLANNER] Optimizing the shortest path...')
        shortest_path, _, _, est_t_sp = trajectory_optimizer(
            input_path=self.input_path, reftrack=reference_with_dist, curv_opt_type='shortest_path',
            safety_width=self.params.safety_width_sp, plot=False)

        map_info = (f'IQP estimated lap time: {round(est_t_iqp, 4)}s; '
                    f'IQP maximum speed: {round(np.amax(raceline[:, 5]), 4)}m/s; '
                    f'SP estimated lap time: {round(est_t_sp, 4)}s; '
                    f'SP maximum speed: {round(np.amax(shortest_path[:, 5]), 4)}m/s; ')

        def trajectory_wpnts(trajectory: np.ndarray) -> WpntArray:
            d_right, d_left = dist_to_bounds(trajectory[:, 1:3], bound_right, bound_left, reverse=self.params.reverse_mapping)
            return _trajectory_wpnts(trajectory, d_right, d_left)

        # Bounds were extracted for the original driving direction; reverse_mapping swaps right and left
        if self.params.reverse_mapping:
            bound_right, bound_left = bound_left, bound_right

        return GlobalPlan(map_info=map_info,
                          est_lap_time=est_t_sp,
                          centerline=_centerline_wpnts(cent_with_dist),
                          raceline=trajectory_wpnts(raceline),
                          shortest_path=trajectory_wpnts(shortest_path),
                          bound_right=bound_right,
                          bound_left=bound_left)


def _trajectory_wpnts(trajectory: np.ndarray, d_right: np.ndarray, d_left: np.ndarray) -> WpntArray:
    """Waypoints of an optimizer trajectory [s_m, x_m, y_m, psi_rad, kappa_radpm, vx_mps, ax_mps2]."""
    # trajectory_planning_helpers has psi = 0 pointing north, ROS has it along x
    psi = trajectory[:, 3] + np.pi / 2
    psi = np.where(psi > np.pi, psi - 2 * np.pi, psi)
    return WpntArray(wpnts=[
        Wpnt(id=i, s_m=pnt[0], x_m=pnt[1], y_m=pnt[2], psi_rad=psi[i], kappa_radpm=pnt[4], vx_mps=pnt[5],
             ax_mps2=pnt[6], d_right=d_right[i], d_left=d_left[i])
        for i, pnt in enumerate(trajectory)])


def _centerline_wpnts(cent_with_dist: np.ndarray) -> WpntArray:
    """Waypoints of the centerline [x_m, y_m, w_right, w_left] with 0.1 m spacing, with heading and curvature."""
    psi, kappa = calc_head_curv_num(path=cent_with_dist[:, :2], el_lengths=0.1 * np.ones(len(cent_with_dist) - 1),
                                    is_closed=False)
    # trajectory_planning_helpers has psi = 0 pointing north, ROS has it along x
    return WpntArray(wpnts=[
        Wpnt(id=i, s_m=i * 0.1, x_m=pnt[0], y_m=pnt[1], psi_rad=psi[i] + np.pi / 2, kappa_radpm=kappa[i],
             d_right=pnt[2], d_left=pnt[3])
        for i, pnt in enumerate(cent_with_dist)])


def main(args=None):
    rclpy.init(args=args)
    try:
        GlobalPlannerNode()  # plans and stores the plan on construction
    except (PlannerError, RuntimeError, ValueError, IOError) as e:
        get_logger('global_planner').fatal(f'[GLOBAL_PLANNER] Planning failed: {e}')
        raise SystemExit(1)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()

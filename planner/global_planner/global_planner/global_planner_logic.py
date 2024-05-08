import os
import sys
import cv2
import copy
import yaml
import subprocess
import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple, Callable
from matplotlib.widgets import Button
from pathlib import Path

from std_msgs.msg import Float32
from f110_msgs.msg import WpntArray
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point, PoseStamped

from skimage.morphology import skeletonize
from tf_transformations import euler_from_quaternion

import trajectory_planning_helpers as tph
from .readwrite_global_waypoints import write_global_waypoints
from global_racetrajectory_optimization import helper_funcs_glob
from global_racetrajectory_optimization.trajectory_optimizer import trajectory_optimizer

from .global_planner_utils import extract_centerline, \
    smooth_centerline, \
    extract_track_bounds, \
    dist_to_bounds, \
    add_dist_to_cent, \
    write_centerline, \
    publish_track_bounds, \
    create_wpnts_markers, \
    compare_direction


class GlobalPlannerLogic:
    """
    Implements the logic of Global Planner
    """

    def __init__(self,
                 safety_width_: float,
                 safety_width_sp_: float,
                 occupancy_grid_thresh_: float,
                 map_editor_mode_: bool,
                 create_map_: bool,
                 map_name_: str,
                 map_dir_: str,
                 finish_script_path_: str,
                 input_path_: str,
                 show_plots_: bool = False,
                 filter_kernel_size_: int = 3,
                 required_laps_: int = 1,
                 reverse_mapping_: bool = False,
                 loginfo_: Callable[[str], None] = print,
                 logwarn_: Callable[[str], None] = sys.stderr.write,
                 logerror_: Callable[[str], None] = sys.stderr.write) -> None:

        # Parameters for processing the map from Cartographer
        self.safety_width = safety_width_
        self.safety_width_sp = safety_width_sp_
        self.occupancy_grid_threshold = occupancy_grid_thresh_
        self.filter_kernel_size = filter_kernel_size_

        # Debugging
        self.show_plots = show_plots_

        # Affects operation of the module
        self.map_editor_mode = map_editor_mode_
        self.compute_global_traj = not self.map_editor_mode
        self.create_map = create_map_

        # Output parameters
        self.map_name = map_name_
        '''desired map name.'''
        self.map_dir = map_dir_
        '''full path to desired map directory'''

        # Logging functions -> For compatibilitiy with ROS1 and ROS2
        self.loginfo = loginfo_
        self.logwarn = logwarn_
        self.logerror = logerror_

        # Operation mode defaults
        self.required_laps = required_laps_
        '''In normal mode, car needs to complete at least these number of laps to save map.'''
        self.reverse_mapping = reverse_mapping_
        '''If raceline should proceed in the opposite direction'''

        self.watershed = True
        '''use watershed algorithm for track bounds by default unless there is an error'''

        # Map parameters
        self.map_valid: bool = False
        '''updated in map callback'''
        self.map_origin = Point()
        self.map_info_str = ""

        if not self.create_map:
            with open(os.path.join(self.map_dir, self.map_name + '.yaml')) as f:
                data = yaml.safe_load(f)
                # only need resolution and origin for waypoints
                self.map_resolution = data['resolution']
                self.map_origin.x = data['origin'][0]
                self.map_origin.y = data['origin'][1]
                self.map_valid: bool = True

                self.initial_position = [self.map_origin.x, self.map_origin.y, 0]

        self.pose_valid: bool = False
        '''updated in pose callback'''

        # Poses (x, y, theta)
        self.current_position = (0.0, 0.0, 0.0)
        self.initial_position = (0.0, 0.0, 0.0)

        # Variables to check laps
        self.was_at_init_pos: bool = True
        self.x_max_diff = 0.5  # meter
        self.y_max_diff = 0.5  # meter
        self.theta_max_diff = np.pi / 2  # rad
        self.lap_count = 0

        # for comparing driven lap length with calculated centerline length
        self.cent_driven = None

        # Filepaths
        self.script_path = finish_script_path_
        '''path to script to call to finish map'''
        self.input_path = input_path_
        '''path to config files for the trajectory optimizer'''

        # For logging
        self.logged_once: bool = False

        # Plot "live" map and wait for button click
        if self.create_map:
            self.fig, (self.ax1, self.axfinish) = plt.subplots(2, 1, gridspec_kw={'height_ratios': [5, 1]})
            self.fig.suptitle('Filtered map')
            self.btn_compute = Button(self.axfinish, 'Map ready; compute global trajectory')
            self.ready_to_plan: bool = False
        else:
            self.ready_to_plan = True

    def global_plan_logic(self) -> Tuple[bool, str]:
        """
        Performs the logic for global planning.

        Returns:
            A tuple containing a boolean indicating if the global planning was successful,
            and a string representing the name of the map.
        """
        # If we are not creating a map, then we can simply compute the trajectory. No further action needed.
        if not self.create_map:
            self.loginfo("Not creating map, only computing trajectory.")
            self.compute_global_trajectory(cent_length=0.0)
            self.loginfo('Successfully computed trajectory.')

            return (True, self.map_name)

        # Wait for map and pose to be valid
        if not self.pose_valid or not self.map_valid:
            return (False, "")

        if not self.logged_once:
            self.loginfo("Map and pose received.")

        # Button callback
        def finish_cb(event):
            self.ready_to_plan = True

        def wait_for_input_and_save() -> tuple[bool, str]:
            """
            Waits for user input and saves the map.

            Returns:
                A tuple of bool and str indicating the success status and the name of the saved map.
            """
            self.btn_compute.on_clicked(finish_cb)
            # Filter and plot map
            filtered_map = self.filter_map_occupancy_grid()
            self.ax1.imshow(filtered_map, cmap='gray')
            plt.show(block=False)
            plt.pause(2)  # only update the plot every 2 seconds

            if self.ready_to_plan:
                self.initial_position = self.current_position  # use position after mapping is done
                self.save_map_png(filtered_map)
                self.loginfo('Saved map.')
                plt.close('all')
                if self.compute_global_traj:
                    self.loginfo(f"Approximate centerline length: {round(cent_length, 4)}m; ")
                    if self.compute_global_trajectory(cent_length=cent_length):
                        self.loginfo("Successfully computed global waypoints.")
                        return (True, self.map_name)
                    else:
                        self.logwarn("Was unable to compute global waypoints!")
                        self.ready_to_plan = False
                        return (False, "")
                return (True, self.map_name)
            return (False, "")

        if self.map_editor_mode:
            # If in map editor mode while mapping: Can make map at any time.
            if not self.logged_once:
                self.loginfo("Map editor ready")
                self.logged_once = True
            return wait_for_input_and_save()
        else:
            # Regular operation.
            if not self.logged_once:
                self.loginfo("Waiting for the car to complete a lap")
                self.logged_once = True
            # check if car is at initial position -> update number of completed laps
            self.update_lap_count()

            # calculate global trajectory after a certain number of completed laps
            if self.lap_count < self.required_laps:
                return (False, "")

            if self.required_laps > 0:
                # calculate length of driven path in first lap for an approx length of the centerline
                cent_length = np.sum(np.sqrt(np.sum(np.power(np.diff(self.cent_driven[:, :2], axis=0), 2), axis=1)))
                cent_len_str = f"Approximate centerline length: {round(cent_length, 4)}m; "
                self.map_info_str += cent_len_str
            else:
                cent_length = 0.0

            return wait_for_input_and_save()

    def compute_global_trajectory(self, cent_length: float) -> bool:
        """
        Compute the global optimized trajectory of a map.

        Calculate the centerline of the track and compute global optimized trajectory with minimum curvature
        optimization.
        Publish the markers and waypoints of the global optimized trajectory.
        A waypoint has the following form: [s_m, x_m, y_m, d_right, d_left, psi_rad, vx_mps, ax_mps2]

        Args:
            cent_length (float): The length of the centerline.

        Returns:
            bool: Success or failure
        """
        # open up png map from file
        img_path = os.path.join(self.map_dir, self.map_name + '.png')
        filtered_map = cv2.flip(cv2.imread(img_path, 0), 0)

        # skeletonize
        skeleton = skeletonize(filtered_map, method='lee')

        f, (ax0, ax1) = plt.subplots(2, 1)
        f.suptitle(f"Map [{self.map_name}]: Filtered map versus morphological skeleton")
        ax0.imshow(filtered_map, cmap='gray')
        ax1.imshow(skeleton, cmap='gray')
        plt.show()

        ################################################################################################################
        # Extract centerline from filtered occupancy grid map
        ################################################################################################################
        try:
            centerline = extract_centerline(
                skeleton=skeleton,
                cent_length=cent_length,
                map_resolution=self.map_resolution,
                map_editor_mode=self.map_editor_mode)
        except IOError:
            if self.map_editor_mode:
                self.logwarn('No closed contours found! Check the edited image...')
            else:
                self.logwarn('No closed contours found! Keep driving...')
            return False
        except ValueError:
            self.logwarn("Couldn't find a closed contour with similar length as driven path!")
            self.loginfo('Maybe missed a lap completion...')
            self.loginfo('Will try again in one lap, so drive at least one more lap!')
            self.was_at_init_pos = True
            self.initial_position = self.current_position
            self.lap_count = 0
            self.required_laps = 1
            self.cent_driven = [self.initial_position]
            return False

        centerline_smooth = smooth_centerline(centerline)

        # convert centerline from cells to meters
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

        if self.show_plots and not self.map_editor_mode:
            self.loginfo(f"Direction of the centerline: {cent_direction[0]}")
            self.loginfo(f"Direction of the initial car position: {self.initial_position[2]}")
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
            self.loginfo('Centerline flipped')

        # extract track bounds
        if self.watershed:
            try:
                bound_r_water, bound_l_water = extract_track_bounds(centerline_smooth,
                                                                    filtered_map,
                                                                    map_editor_mode=self.map_editor_mode,
                                                                    map_resolution=self.map_resolution,
                                                                    map_origin=self.map_origin,
                                                                    initial_position=self.initial_position,
                                                                    show_plots=self.show_plots)
                dist_transform = None
                self.loginfo('Using watershed for track bound extraction...')
            except IOError:
                self.logwarn('More than two track bounds detected with watershed algorithm')
                self.loginfo('Trying with simple distance transform...')
                self.watershed = False
                bound_r_water = None
                bound_l_water = None
                dist_transform = cv2.distanceTransform(filtered_map, cv2.DIST_L2, 5)
        else:
            self.loginfo('Using distance transform for track bound extraction...')
            bound_r_water = None
            bound_l_water = None
            dist_transform = cv2.distanceTransform(filtered_map, cv2.DIST_L2, 5)

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

        self.loginfo('Start Global Trajectory optimization with iterative minimum curvature...')
        try:
            global_trajectory_iqp, bound_r_iqp, bound_l_iqp, est_t_iqp = trajectory_optimizer(
                input_path=self.input_path, track_name=iqp_centerline_path, curv_opt_type='mincurv_iqp', safety_width=self.safety_width, plot=(
                    self.show_plots and not self.map_editor_mode))
        except RuntimeError as e:
            self.logwarn(f"Error during iterative minimum curvature optimization, error: {e}")
            self.loginfo('Try again later!')
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

        global_traj_wpnts_iqp, global_traj_markers_iqp = self.create_wpnts_markers(trajectory=global_trajectory_iqp,
                                                                                   d_right=d_right_iqp,
                                                                                   d_left=d_left_iqp)

        # publish global trajectory markers and waypoints
        self.loginfo('Done with iterative minimum curvature optimization')
        self.loginfo('Lap Completed now publishing global waypoints')

        ################################################################################################################
        # Compute global trajectory with shortest path optimization
        ################################################################################################################

        self.loginfo('Start reverse Global Trajectory optimization with shortest path...')

        self.loginfo('Start Global Trajectory optimization with iterative minimum curvature for overtaking...')
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

        global_traj_wpnts_sp, global_traj_markers_sp = self.create_wpnts_markers(trajectory=global_trajectory_sp,
                                                                                 d_right=d_right_sp,
                                                                                 d_left=d_left_sp,
                                                                                 second_traj=True)

        # publish global trajectory markers and waypoints
        self.loginfo('Done with shortest path optimization')
        self.loginfo('Lap Completed now publishing shortest path global waypoints')

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
        return True

    def filter_map_occupancy_grid(self) -> np.ndarray:
        """
        Filters the occupancy grid map by performing morphological opening and binarization.

        Returns:
            filtered_map (np.ndarray): The filtered occupancy grid map.
        """
        # Assume that map is from the occupancy grid and therefore needs to be processed
        original_map = np.int8(self.map_occupancy_grid).reshape(self.map_height, self.map_width)

        # get right shape for occupancy grid map
        # mark unknown (-1) as occupied (100)
        original_map = np.where(original_map == -1, 100, original_map)

        # binarised map
        bw = np.where(original_map < self.occupancy_grid_threshold, 255, 0)
        bw = np.uint8(bw)

        # Filtering with morphological opening
        kernel = np.ones((self.filter_kernel_size, self.filter_kernel_size), np.uint8)
        filtered_map = cv2.morphologyEx(bw, cv2.MORPH_OPEN, kernel, iterations=2)

        return filtered_map

    def save_map_png(self, filtered_map) -> None:
        """
        Save the filtered map as a PNG image and a YAML file in a specified directory.

        Args:
            filtered_map (numpy.ndarray): The filtered map to be saved as a PNG image.

        Raises:
            OSError: If the specified directory already exists.

        Returns:
            None
        """
        # create a folder 'map_name' in the data folder and raise an error if it already exists
        try:
            os.makedirs(self.map_dir)
        except OSError as e:
            self.logerror(f'Could not create the folder {self.map_dir} because it already exists.')
            raise OSError(e)

        self.loginfo(f'Successfully created the folder {self.map_dir}')

        def save_map_to_directory(map_dir: str, map_name: str):
            img_path = os.path.join(map_dir, map_name + '.png')
            flipped_map = cv2.flip(filtered_map, 0)
            cv2.imwrite(img_path, flipped_map)

            dict_map = {'image': self.map_name + '.png',
                        'resolution': self.map_resolution,
                        'origin': [self.map_origin.x, self.map_origin.y, 0],
                        'negate': 0,
                        'occupied_thresh': 0.65,
                        'free_thresh': 0.196}

            with open(os.path.join(self.map_dir, self.map_name + ".yaml"), 'w') as file:
                _ = yaml.dump(dict_map, file, default_flow_style=False)

        # write image as png and a yaml file in the folder
        save_map_to_directory(self.map_dir, self.map_name)
        save_map_to_directory(self.map_dir, 'pf_map')

        # Call ROS service to save PB stream
        pbstream_path = os.path.join(self.map_dir, self.map_name + ".pbstream")
        subprocess.Popen(args=[self.script_path, pbstream_path], shell=False).wait()

        self.loginfo(f'PNG and YAML file created and saved in the {self.map_dir} folder')

    def update_lap_count(self) -> None:
        """
        Updates the lap count based on the car's position.

        This method checks if the car is at the start position and wasn't there before,
        indicating that a lap has been completed. If a lap is completed, the lap count is
        incremented and a log message is printed.

        Returns:
            None
        """
        is_at_init_pos = self.at_init_pos_check()
        # check if car is now at start position and wasn't before --> means we completed a lap
        if is_at_init_pos and not self.was_at_init_pos:
            self.was_at_init_pos = True
            self.lap_count += 1
            self.loginfo(f"Laps completed {self.lap_count}")

        elif not is_at_init_pos:
            self.was_at_init_pos = False

    def at_init_pos_check(self) -> bool:
        """
        Check if the current position is similar to the initial position.

        Returns:
            bool: True if the current position is similar to the initial position, False otherwise.
        """
        # absolute values of difference between current and initial position
        x_diff = np.abs(self.current_position[0] - self.initial_position[0])
        y_diff = np.abs(self.current_position[1] - self.initial_position[1])

        # The smallest distance between the two angles is using this
        theta_diff0 = np.abs(self.current_position[2] - self.initial_position[2])
        theta_diff1 = 2 * np.pi - theta_diff0
        theta_diff = min(theta_diff0, theta_diff1)

        at_init_pos = (x_diff < self.x_max_diff) and (y_diff < self.y_max_diff) and (theta_diff < self.theta_max_diff)
        return at_init_pos

    def update_map(self, msg: OccupancyGrid) -> None:
        """
        Updates the map with the given OccupancyGrid message.

        Args:
            msg (OccupancyGrid): The OccupancyGrid message containing map data.

        Returns:
            None
        """
        # Data should not be overwritten if we use map maps from yaml file
        if self.create_map:
            self.map_width = msg.info.width  # uint32, [cells]
            self.map_height = msg.info.height  # uint32, [cells]
            self.map_resolution = msg.info.resolution  # float32, [m/cell]
            self.map_origin = msg.info.origin.position
            self.map_occupancy_grid = msg.data  # int8[]

        self.map_valid = True

    def update_pose(self, msg: PoseStamped) -> None:
        """
        Update the current pose of the robot.

        Args:
            msg (PoseStamped): The pose message containing the position and orientation.

        Returns:
            None
        """
        x = msg.pose.position.x
        y = msg.pose.position.y
        theta = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y,
                                       msg.pose.orientation.z, msg.pose.orientation.w])[2]

        if not self.pose_valid:
            self.pose_valid = True
            self.initial_position = (x, y, theta)

        self.current_position = (x, y, theta)

        if self.lap_count == 0:
            if self.cent_driven is None:
                self.cent_driven = np.array([self.current_position])
            else:
                self.cent_driven = np.append(self.cent_driven, [self.current_position], axis=0)

    def create_wpnts_markers(self, trajectory: np.ndarray,
                             d_right: np.ndarray,
                             d_left: np.ndarray,
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
        self.loginfo(speed_string)

        return create_wpnts_markers(trajectory, d_right, d_left, second_traj)

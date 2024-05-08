import cv2
import csv
import numpy as np
from pathlib import Path
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter
from skimage.segmentation import watershed
from ament_index_python.packages import get_package_share_directory

import trajectory_planning_helpers as tph
from global_racetrajectory_optimization import helper_funcs_glob

from geometry_msgs.msg import Point
from f110_msgs.msg import Wpnt, WpntArray
from visualization_msgs.msg import Marker, MarkerArray


def get_data_path(subpath=''):
    """
    Helper function to get an absolute path to the specified (relative) path within the data folder.
    """
    return Path(get_package_share_directory('stack_master')).parents[3] / 'src/race_stack/stack_master' / subpath


def extract_centerline(skeleton, cent_length: float, map_resolution: float, map_editor_mode: bool) -> np.ndarray:
    """
    Extract the centerline out of the skeletonized binary image.

    This function takes a skeletonized binary image as input and extracts the centerline from it. The centerline is
    determined by finding closed contours in the image and comparing their lengths to the expected approximate centerline
    length. The function returns the centerline in the form of a numpy array, where each element represents a point on
    the centerline in cells (not meters).

    Parameters
    ----------
    skeleton : np.ndarray
        The skeleton of the binarized and filtered map.
    cent_length : float
        The expected approximate centerline length.
    map_resolution : float
        The resolution of the map in meters per cell.
    map_editor_mode : bool
        A flag indicating whether the function is being called in map editor mode.

    Returns
    -------
    centerline : np.ndarray
        The centerline in the form [[x1, y1], [x2, y2], ...], where each element represents a point on the centerline
        in cells (not meters).

    Raises
    ------
    IOError
        If no closed contour is found in the skeletonized image.
    ValueError
        If all found contours do not have a similar line length as the centerline (can only happen if `map_editor_mode`
        is True).

    """
    # get contours from skeleton
    contours, hierarchy = cv2.findContours(skeleton, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)

    # save all closed contours
    closed_contours = []
    for i, elem in enumerate(contours):
        opened = hierarchy[0][i][2] < 0 and hierarchy[0][i][3] < 0
        if not opened:
            closed_contours.append(elem)

    # if we have no closed contour, we can't calculate a global trajectory
    if len(closed_contours) == 0:
        raise IOError("No closed contours")

    # calculate the line length of every contour to get the real centerline
    line_lengths = [np.inf] * len(closed_contours)
    for i, cont in enumerate(closed_contours):
        line_length = 0
        for k, pnt in enumerate(cont):
            line_length += np.sqrt((pnt[0][0] - cont[k - 1][0][0]) ** 2 +
                                   (pnt[0][1] - cont[k - 1][0][1]) ** 2)

        line_length *= map_resolution  # length in meters not cells
        # line length should be around the length of the centerline otherwise keep length infinity
        if np.abs(cent_length / line_length - 1.0) < 0.15:
            line_lengths[i] = line_length
        elif map_editor_mode or cent_length == 0.0:
            line_lengths[i] = line_length

    # take the shortest line
    min_line_length = min(line_lengths)

    if min_line_length == np.inf:
        raise ValueError("Only invalid closed contour line lengths")

    min_length_index = line_lengths.index(min_line_length)
    # print(line_lengths)
    smallest_contour = np.array(closed_contours[min_length_index]).flatten()

    # reshape smallest_contours from the shape [x1,y1,x2,y2,...] to [[x1,y1],[x2,y2],...]
    # this will be the centerline
    len_reshape = int(len(smallest_contour) / 2)
    centerline = smallest_contour.reshape(len_reshape, 2)

    return centerline


def smooth_centerline(centerline: np.ndarray) -> np.ndarray:
    """
    Smooth the centerline with a Savitzky-Golay filter.

    Notes
    -----
    The savgol filter doesn't ensure a smooth transition at the end and beginning of the centerline. That's why
    we apply a savgol filter to the centerline with start and end points on the other half of the track.
    Afterwards, we take the results of the second smoothed centerline for the beginning and end of the
    first centerline to get an overall smooth centerline

    Parameters
    ----------
    centerline : np.ndarray
        Unsmoothed centerline

    Returns
    -------
    centerline_smooth : np.ndarray
        Smooth centerline
    """
    # centerline_smooth = centerline
    # smooth centerline with a Savitzky Golay filter
    # filter_length = 20
    centerline_length = len(centerline)
    # print("Number of centerline points: ", centerline_length)

    if centerline_length > 2000:
        filter_length = int(centerline_length / 200) * 10 + 1
    elif centerline_length > 1000:
        filter_length = 81
    elif centerline_length > 500:
        filter_length = 41
    else:
        filter_length = 21
    centerline_smooth = savgol_filter(centerline, filter_length, 3, axis=0)

    # cen_len is half the length of the centerline
    cen_len = int(len(centerline) / 2)
    centerline2 = np.append(centerline[cen_len:], centerline[0:cen_len], axis=0)
    centerline_smooth2 = savgol_filter(centerline2, filter_length, 3, axis=0)

    # take points from second (smoothed) centerline for first centerline
    centerline_smooth[0:filter_length] = centerline_smooth2[cen_len:(cen_len + filter_length)]
    centerline_smooth[-filter_length:] = centerline_smooth2[(cen_len - filter_length):cen_len]

    return centerline_smooth


def extract_track_bounds(
        centerline: np.ndarray,
        filtered_bw: np.ndarray,
        map_editor_mode: bool,
        map_resolution: float,
        map_origin: Point,
        initial_position: np.ndarray,
        show_plots: bool) -> tuple[np.ndarray, np.ndarray]:
    """
    Extract the boundaries of the track.

    Use the watershed algorithm with the centerline as marker to extract the boundaries of the filtered black
    and white image of the map.

    Parameters
    ----------
    centerline : np.ndarray
        The centerline of the track (in cells not meters)
    filtered_bw : np.ndarray
        Filtered black and white image of the track
    map_editor_mode : bool
        Flag indicating whether the map editor mode is enabled
    map_resolution : float
        Resolution of the map (in meters per cell)
    map_origin : Point
        Origin point of the map
    initial_position : np.ndarray
        Initial position of the vehicle
    show_plots : bool
        Flag indicating whether to show plots

    Returns
    -------
    bound_right, bound_left : tuple[np.ndarray, np.ndarray]
        Points of the track bounds right and left in meters

    Raises
    ------
    IOError
        If there were more (or less) than two track bounds found
    """
    # create a black and white image of the centerline
    cent_img = np.zeros((filtered_bw.shape[0], filtered_bw.shape[1]), dtype=np.uint8)
    cv2.drawContours(cent_img, [centerline.astype(int)], 0, 255, 2, cv2.LINE_8)

    # create markers for watershed algorithm
    _, cent_markers = cv2.connectedComponents(cent_img)

    # apply watershed algorithm to get only the track (without any lidar beams outside the track)
    dist_transform = cv2.distanceTransform(filtered_bw, cv2.DIST_L2, 5)
    labels = watershed(-dist_transform, cent_markers, mask=filtered_bw)

    closed_contours = []

    for label in np.unique(labels):
        if label == 0:
            continue

        # Create a mask, the mask should be the track
        mask = np.zeros(filtered_bw.shape, dtype="uint8")
        mask[labels == label] = 255

        if show_plots and not map_editor_mode:
            plt.imshow(mask, cmap='gray')
            plt.show()

        # Find contours
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)

        # save all closed contours
        for i, cont in enumerate(contours):
            opened = hierarchy[0][i][2] < 0 and hierarchy[0][i][3] < 0
            if not opened:
                closed_contours.append(cont)

        # there must not be more (or less) than two closed contour
        if len(closed_contours) != 2:
            raise IOError("More than two track bounds detected! Check input")
        # draw the boundary into the centerline image
        cv2.drawContours(cent_img, closed_contours, 0, 255, 4)
        cv2.drawContours(cent_img, closed_contours, 1, 255, 4)

    # the longest closed contour is the outer boundary
    bound_long = max(closed_contours, key=len)
    bound_long = np.array(bound_long).flatten()

    # reshape from the shape [x1,y1,x2,y2,...] to [[x1,y1],[x2,y2],...]
    len_reshape = int(len(bound_long) / 2)
    bound_long = bound_long.reshape(len_reshape, 2)
    # convert to meter
    bound_long_meter = np.zeros(np.shape(bound_long))
    bound_long_meter[:, 0] = bound_long[:, 0] * map_resolution + map_origin.x
    bound_long_meter[:, 1] = bound_long[:, 1] * map_resolution + map_origin.y

    # inner boundary is the shorter one
    bound_short = min(closed_contours, key=len)
    bound_short = np.array(bound_short).flatten()

    # reshape from the shape [x1,y1,x2,y2,...] to [[x1,y1],[x2,y2],...]
    len_reshape = int(len(bound_short) / 2)
    bound_short = bound_short.reshape(len_reshape, 2)
    # convert to meter
    bound_short_meter = np.zeros(np.shape(bound_short))
    bound_short_meter[:, 0] = bound_short[:, 0] * map_resolution + map_origin.x
    bound_short_meter[:, 1] = bound_short[:, 1] * map_resolution + map_origin.y

    # get distance to initial position for every point on the outer bound to figure out if it is the right
    # or left boundary
    bound_distance = np.sqrt(np.power(bound_long_meter[:, 0] - initial_position[0], 2)
                             + np.power(bound_long_meter[:, 1] - initial_position[1], 2))

    min_dist_ind = np.argmin(bound_distance)

    bound_direction = np.angle([complex(bound_long_meter[min_dist_ind, 0] - bound_long_meter[min_dist_ind - 1, 0],
                                        bound_long_meter[min_dist_ind, 1] - bound_long_meter[min_dist_ind - 1, 1])])

    norm_angle_right = initial_position[2] - np.pi
    if norm_angle_right < -np.pi:
        norm_angle_right = norm_angle_right + 2 * np.pi

    if compare_direction(norm_angle_right, bound_direction):
        bound_right = bound_long_meter
        bound_left = bound_short_meter
    else:
        bound_right = bound_short_meter
        bound_left = bound_long_meter

    if show_plots and not map_editor_mode:
        plt.imshow(cent_img, cmap='gray')
        fig1, ax1 = plt.subplots()
        ax1.plot(bound_right[:, 0], bound_right[:, 1], 'b', label='Right bound')
        ax1.plot(bound_left[:, 0], bound_left[:, 1], 'g', label='Left bound')
        ax1.plot(centerline[:, 0] * map_resolution + map_origin.x,
                 centerline[:, 1] * map_resolution + map_origin.y, 'r', label='Centerline')
        ax1.legend()
        plt.show()

    return bound_right, bound_left


def dist_to_bounds(
        trajectory: np.ndarray,
        bound_r,
        bound_l,
        centerline: np.ndarray,
        safety_width: float,
        show_plots: bool,
        reverse: bool = False) -> tuple[np.ndarray, np.ndarray]:
    """
    Calculate the distance to track bounds for every point on a trajectory.

    Parameters
    ----------
    trajectory : np.ndarray
        A trajectory in form [s_m, x_m, y_m, psi_rad, vx_mps, ax_mps2] or [x_m, y_m]
    bound_r
        Points in meters of boundary right
    bound_l
        Points in meters of boundary left
    centerline : np.ndarray
        Centerline only needed if global trajectory is given and plot of it is wanted
    safety_width : float
        Safety width in meters
    show_plots : bool
        Flag indicating whether to show plots or not
    reverse : bool, optional
        Flag indicating whether the trajectory is in reverse direction or not, by default False

    Returns
    -------
    dists_right, dists_left : tuple[np.ndarray, np.ndarray]
        Distances to the right and left track boundaries for every waypoint
    """
    # check format of trajectory
    if len(trajectory[0]) > 2:
        help_trajectory = trajectory[:, 1:3]
        trajectory_str = "Global Trajectory"
    else:
        help_trajectory = trajectory
        trajectory_str = "Centerline"

    # interpolate track bounds
    bound_r_tmp = np.column_stack((bound_r, np.zeros((bound_r.shape[0], 2))))
    bound_l_tmp = np.column_stack((bound_l, np.zeros((bound_l.shape[0], 2))))

    bound_r_int = helper_funcs_glob.src.interp_track.interp_track(reftrack=bound_r_tmp,
                                                                  stepsize_approx=0.1)
    bound_l_int = helper_funcs_glob.src.interp_track.interp_track(reftrack=bound_l_tmp,
                                                                  stepsize_approx=0.1)

    # find the closest points of the track bounds to global trajectory waypoints
    n_wpnt = len(help_trajectory)
    dists_right = np.zeros(n_wpnt)  # contains (min) distances between waypoints and right bound
    dists_left = np.zeros(n_wpnt)  # contains (min) distances between waypoints and left bound

    for i, wpnt in enumerate(help_trajectory):
        dists_bound_right = np.sqrt(np.power(bound_r_int[:, 0] - wpnt[0], 2)
                                    + np.power(bound_r_int[:, 1] - wpnt[1], 2))
        dists_right[i] = np.amin(dists_bound_right)

        dists_bound_left = np.sqrt(np.power(bound_l_int[:, 0] - wpnt[0], 2)
                                   + np.power(bound_l_int[:, 1] - wpnt[1], 2))
        dists_left[i] = np.amin(dists_bound_left)

    if show_plots and trajectory_str == "Global Trajectory":
        width_veh_real = 0.3
        normvec_normalized_opt = tph.calc_normal_vectors.calc_normal_vectors(trajectory[:, 3])

        veh_bound1_virt = trajectory[:, 1:3] + normvec_normalized_opt * safety_width / 2
        veh_bound2_virt = trajectory[:, 1:3] - normvec_normalized_opt * safety_width / 2

        veh_bound1_real = trajectory[:, 1:3] + normvec_normalized_opt * width_veh_real / 2
        veh_bound2_real = trajectory[:, 1:3] - normvec_normalized_opt * width_veh_real / 2

        # plot track including optimized path
        fig, ax = plt.subplots()
        fig.suptitle("Track with optimized path")

        ax.plot(bound_r[:, 0], bound_r[:, 1], "k-", linewidth=1.0, label="Track bounds")
        ax.plot(bound_l[:, 0], bound_l[:, 1], "k-", linewidth=1.0)
        ax.plot(centerline[:, 0], centerline[:, 1], "k--", linewidth=1.0, label='Centerline')

        ax.plot(veh_bound1_virt[:, 0], veh_bound1_virt[:, 1], "b", linewidth=0.5, label="Vehicle width with safety")
        ax.plot(veh_bound2_virt[:, 0], veh_bound2_virt[:, 1], "b", linewidth=0.5)
        ax.plot(veh_bound1_real[:, 0], veh_bound1_real[:, 1], "c", linewidth=0.5, label="Real vehicle width")
        ax.plot(veh_bound2_real[:, 0], veh_bound2_real[:, 1], "c", linewidth=0.5)

        ax.plot(trajectory[:, 1], trajectory[:, 2], 'tab:orange', linewidth=2.0, label="Global trajectory")

        plt.grid()
        ax1 = plt.gca()

        point1_arrow = np.array([trajectory[0, 1], trajectory[0, 2]])
        point2_arrow = np.array([trajectory[5, 1], trajectory[5, 2]])
        vec_arrow = (point2_arrow - point1_arrow)
        ax1.arrow(point1_arrow[0], point1_arrow[1], vec_arrow[0], vec_arrow[1], width=0.05,
                  head_width=0.3, head_length=0.3, fc='g', ec='g')

        ax.set_aspect("equal", "datalim")
        plt.xlabel("x-distance from origin [m]")
        plt.ylabel("y-distance from origin [m]")
        plt.title(f"Global trajectory with vehicle width")
        plt.legend()

        plt.show()
    # Return flipped distances if map_editor reversing
    if reverse:
        return dists_left, dists_right
    else:
        return dists_right, dists_left


def add_dist_to_cent(centerline_smooth: np.ndarray,
                     centerline_meter: np.ndarray,
                     map_resolution: float,
                     safety_width: float,
                     show_plots: bool,
                     dist_transform=None,
                     bound_r: np.ndarray = None,
                     bound_l: np.ndarray = None,
                     reverse: bool = False) -> np.ndarray:
    """
    Add distance to track bounds to the centerline points.

    Parameters
    ----------
    centerline_smooth : np.ndarray
        Smooth centerline in cells (not meters)
    centerline_meter : np.ndarray
        Smooth centerline in meters (not cells)
    map_resolution : float
        Resolution of the map in meters per cell
    safety_width : float
        Safety width for the track bounds
    show_plots : bool
        Flag indicating whether to show plots or not
    dist_transform : Any, default=None
        Euclidean distance transform of the filtered black and white image
    bound_r : np.ndarray, default=None
        Points of the right track bound in meters
    bound_l : np.ndarray, default=None
        Points of the left track bound in meters
    reverse : bool, default=False
        Flag indicating whether the trajectory is being calculated in reverse direction

    Returns
    -------
    centerline_comp : np.ndarray
        Complete centerline with distance to right and left track bounds for every point
    """
    centerline_comp = np.zeros((len(centerline_meter), 4))

    if dist_transform is not None:
        width_track_right = dist_transform[centerline_smooth[:, 1].astype(int),
                                           centerline_smooth[:, 0].astype(int)] * map_resolution
        if len(width_track_right) != len(centerline_meter):
            width_track_right = np.interp(np.arange(0, len(centerline_meter)), np.arange(0, len(width_track_right)),
                                          width_track_right)
        width_track_left = width_track_right
    elif bound_r is not None and bound_l is not None:
        width_track_right, width_track_left = dist_to_bounds(centerline_meter, bound_r, bound_l,
                                                             centerline=centerline_meter,
                                                             safety_width=safety_width,
                                                             show_plots=show_plots,
                                                             reverse=reverse)
    else:
        raise IOError("No closed contours found...")

    centerline_comp[:, 0] = centerline_meter[:, 0]
    centerline_comp[:, 1] = centerline_meter[:, 1]
    centerline_comp[:, 2] = width_track_right
    centerline_comp[:, 3] = width_track_left
    return centerline_comp


def write_centerline(centerline: np.ndarray, sp_bool: bool = False) -> MarkerArray:
    """
    Create a csv file with centerline maps.

    The centerline maps includes position and width to the track bounds in meter, so that it can be used in the
    global trajectory optimizer. The file has the following format: x_m, y_m, w_tr_right_m, w_tr_left_m .

    Parameters
    ----------
    centerline : np.ndarray
        The centerline in form [x_m, y_m, w_tr_right_m, w_tr_left_m]
    sp_bool : bool, default=False
        Used for shortest path optimization when another centerline csv should be created

    Returns
    -------
    centerline_markers : MarkerArray
        Centerline as a MarkerArray which can be published
    """
    centerline_markers = MarkerArray()
    centerline_wpnts = WpntArray()

    cent_str = 'map_centerline.csv' if not sp_bool else 'map_centerline_2.csv'
    cent_path = Path.home() / ".ros" / cent_str
    with open(cent_path, 'w', newline='') as file:
        writer = csv.writer(file)
        id_cnt = 0  # for marker id

        for row in centerline:
            x_m = row[0]
            y_m = row[1]
            width_tr_right_m = row[2]
            width_tr_left_m = row[3]
            writer.writerow([x_m, y_m, width_tr_right_m, width_tr_left_m])

            cent_marker = Marker()
            cent_marker.header.frame_id = 'map'
            cent_marker.type = cent_marker.SPHERE
            cent_marker.scale.x = 0.05
            cent_marker.scale.y = 0.05
            cent_marker.scale.z = 0.05
            cent_marker.color.a = 1.0
            cent_marker.color.b = 1.0

            cent_marker.id = id_cnt
            cent_marker.pose.position.x = x_m
            cent_marker.pose.position.y = y_m
            cent_marker.pose.orientation.w = 1.0
            centerline_markers.markers.append(cent_marker)

            wpnt = Wpnt()
            wpnt.id = id_cnt
            wpnt.x_m = x_m
            wpnt.d_right = width_tr_right_m
            wpnt.d_left = width_tr_left_m
            wpnt.y_m = y_m
            centerline_wpnts.wpnts.append(wpnt)

            id_cnt += 1

    return centerline_wpnts, centerline_markers


def publish_track_bounds(bound_r, bound_l, reverse: bool = False) -> MarkerArray:
    """
    Publish the track bounds as a MarkerArray.

    Parameters
    ----------
    bound_r : np.ndarray
        Points of the right track bound in meters
    bound_l : np.ndarray
        Points of the left track bound in meters
    reverse : bool, default=False
        Flag indicating whether the trajectory is being calculated in reverse direction

    Returns
    -------
    bounds_markers : MarkerArray
        Track bounds as a MarkerArray which can be published
    """
    bounds_markers = MarkerArray()
    id_cnt = 0
    if reverse:
        bound_l_real = bound_r.copy()
        bound_r_real = bound_l.copy()
    else:
        bound_l_real = bound_l
        bound_r_real = bound_r

    for i, pnt_r in enumerate(bound_r_real):
        bnd_r_mrk = Marker()
        bnd_r_mrk.header.frame_id = 'map'
        bnd_r_mrk.type = bnd_r_mrk.SPHERE
        bnd_r_mrk.scale.x = 0.05
        bnd_r_mrk.scale.y = 0.05
        bnd_r_mrk.scale.z = 0.05
        bnd_r_mrk.color.a = 1.0
        bnd_r_mrk.color.b = 0.5
        bnd_r_mrk.color.r = 0.5

        bnd_r_mrk.id = id_cnt
        id_cnt += 1
        bnd_r_mrk.pose.position.x = pnt_r[0]
        bnd_r_mrk.pose.position.y = pnt_r[1]
        bnd_r_mrk.pose.orientation.w = 1.0
        bounds_markers.markers.append(bnd_r_mrk)

    for i, pnt_l in enumerate(bound_l_real):
        bnd_l_mrk = Marker()
        bnd_l_mrk.header.frame_id = 'map'
        bnd_l_mrk.type = bnd_l_mrk.SPHERE
        bnd_l_mrk.scale.x = 0.05
        bnd_l_mrk.scale.y = 0.05
        bnd_l_mrk.scale.z = 0.05
        bnd_l_mrk.color.a = 1.0
        bnd_l_mrk.color.r = 0.5
        bnd_l_mrk.color.g = 1.0

        bnd_l_mrk.id = id_cnt
        id_cnt += 1
        bnd_l_mrk.pose.position.x = pnt_l[0]
        bnd_l_mrk.pose.position.y = pnt_l[1]
        bnd_l_mrk.pose.orientation.w = 1.0
        bounds_markers.markers.append(bnd_l_mrk)

    return bounds_markers


def create_wpnts_markers(trajectory: np.ndarray, d_right: np.ndarray, d_left: np.ndarray,
                         second_traj: bool = False) -> tuple[WpntArray, MarkerArray]:
    """
    Create and return a waypoint array and a marker array.

    Parameters
    ----------
    trajectory : np.ndarray
        A trajectory with waypoints in form [s_m, x_m, y_m, psi_rad, vx_mps, ax_mps2]
    d_right : np.ndarray
        Distances to the right track bounds for every waypoint in {trajectory}
    d_left : np.ndarray
        Distances to the left track bounds for every waypoint in {trajectory}
    second_traj : bool, default=False
        Display second trajectory with a different color than first, better for visualization

    Returns
    -------
    global_wpnts, global_markers : tuple[WpntArray, MarkerArray]
        A waypoint array and a marker array with all points of {trajectory}
    """
    max_vx_mps = max(trajectory[:, 5])

    global_wpnts = WpntArray()
    global_markers = MarkerArray()

    for i, pnt in enumerate(trajectory):
        global_wpnt = Wpnt()
        global_wpnt.id = i
        global_wpnt.s_m = pnt[0]
        global_wpnt.x_m = pnt[1]
        global_wpnt.d_right = d_right[i]
        global_wpnt.d_left = d_left[i]
        global_wpnt.y_m = pnt[2]
        global_wpnt.psi_rad = conv_psi(pnt[3])
        global_wpnt.kappa_radpm = pnt[4]
        global_wpnt.vx_mps = pnt[5]
        global_wpnt.ax_mps2 = pnt[6]

        global_wpnts.wpnts.append(global_wpnt)

        global_marker = Marker()
        global_marker.header.frame_id = 'map'
        global_marker.type = global_marker.CYLINDER
        global_marker.scale.x = 0.1
        global_marker.scale.y = 0.1
        global_marker.scale.z = global_wpnt.vx_mps / max_vx_mps
        global_marker.color.a = 1.0
        global_marker.color.r = 1.0
        global_marker.color.g = 1.0 if second_traj else 0.0

        global_marker.id = i
        global_marker.pose.position.x = pnt[1]
        global_marker.pose.position.y = pnt[2]
        global_marker.pose.position.z = global_wpnt.vx_mps / max_vx_mps / 2
        global_marker.pose.orientation.w = 1.0
        global_markers.markers.append(global_marker)

    return global_wpnts, global_markers


def conv_psi(psi: float) -> float:
    """
    Convert psi angles where 0 is at the y-axis into psi angles where 0 is at the x-axis.

    Parameters
    ----------
    psi : float
        angle (in rad) to convert

    Returns
    -------
    new_psi : float
        converted angle
    """
    new_psi = psi + np.pi / 2

    if new_psi > np.pi:
        new_psi = new_psi - 2 * np.pi

    return new_psi


def compare_direction(alpha: float, beta: float) -> bool:
    """
    Compare the direction of two points and check if they point in the same direction.

    Parameters
    ----------
    alpha : float
        direction angle in rad
    beta : float
        direction angle in rad

    Returns
    -------
    bool
        True if alpha and beta point in the same direction
    """
    delta_theta = np.abs(alpha - beta)

    if delta_theta > np.pi:
        delta_theta = 2 * np.pi - delta_theta

    return delta_theta < np.pi / 2

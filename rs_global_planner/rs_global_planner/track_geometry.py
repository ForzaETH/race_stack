import cv2
import numpy as np
from scipy.signal import savgol_filter
from scipy.spatial import KDTree

class PlannerError(Exception):
    """Base class for all planner errors."""

def extract_centerline(skeleton) -> np.ndarray:
    """
    Extract the centerline out of the skeletonized binary image of the centerline.
    The skeleton is already the centerline.
    """
    # Obtain contours; hierarchy describes the nesting of contours
    contours, hierarchy = cv2.findContours(skeleton, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
    
    if hierarchy is None:
        raise PlannerError("[GLOBAL_PLANNER]: Skeleton is empty, no contours detected!")
    hierarchy = hierarchy[0]
    
    # Obtain closed contours (contour i has a child: hierarchy[i][2] >= 0, contour i has a parent: hierarchy[i][3] >= 0)
    closed = [ctr for ctr, h in zip(contours, hierarchy) if h[2] >= 0 or h[3] >= 0]
    
    if len(closed) == 0:
        raise PlannerError("[GLOBAL_PLANNER]: No closed contours detected!")
    
    # Determine the shortest contour
    shortest = min(closed, key=lambda ctr: cv2.arcLength(ctr, closed=True))
    
    # Reshape to [[x_i, y_i]]_i
    centerline = shortest.reshape(-1, 2)
    
    return centerline


def smooth_centerline(centerline: np.ndarray, map_resolution: float, window_m: float) -> np.ndarray:
    """Smooth the centerline (in cells) with a Savitzky-Golay filter over a window of window_m meters."""

    # Window in cells, forced odd as required by the Savitzky-Golay filter
    filter_length = 2 * round(window_m / (2 * map_resolution)) + 1

    # Important: mode='wrap' such that start / finish is smooth
    return savgol_filter(centerline, filter_length, 3, axis=0, mode='wrap')


def follows_driving_direction(centerline: np.ndarray, initial_pose: np.ndarray) -> bool:
    """True if the centerline [[x, y], ...] in meters is ordered in the car's driving direction at the start pose."""

    # Centerline direction next to the car, compared with its heading (less than 90 deg apart)
    k = np.argmin(np.linalg.norm(centerline - np.asarray(initial_pose[:2]), axis=1))
    tangent = centerline[(k + 1) % len(centerline)] - centerline[k - 1]
    heading = np.array([np.cos(initial_pose[2]), np.sin(initial_pose[2])])
    return bool(heading @ tangent > 0)


def extract_track_bounds(centerline: np.ndarray, map_img: np.ndarray, map_resolution: float, map_origin: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Extract the right and left track bounds in meters, for a centerline (in pixels) ordered in driving direction."""
    
    # Segment each white blob into components
    _, components = cv2.connectedComponents((map_img > 0).astype(np.uint8), connectivity=4)
    
    # Find the component in which the centerline lies
    track_label = components[int(centerline[0, 1]), int(centerline[0, 0])]
    
    # Keep only that component and set everything else to black
    track_mask = np.where(components == track_label, 255, 0).astype(np.uint8)
    
    # Obtain track contours
    contours, hierarchy = cv2.findContours(track_mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
    if hierarchy is None:
        raise PlannerError("[GLOBAL_PLANNER]: Map is empty, no contours detected!")
    hierarchy = hierarchy[0]
    
    # Obtain closed contours 
    closed = [ctr for ctr, h in zip(contours, hierarchy) if h[2] >= 0 or h[3] >= 0]
    
    if len(closed) != 2:
        raise PlannerError(f"[GLOBAL_PLANNER]: Got {len(closed)} contours, expected 2!")
    
    origin = np.asarray(map_origin[:2])
    
    # Longest contour is the outer track boundary
    outer_boundary = max(closed, key=lambda ctr: cv2.arcLength(ctr, closed=True)).reshape(-1, 2) * map_resolution + origin
    
    # Shortest contour is the inner track boundary
    inner_boundary = min(closed, key=lambda ctr: cv2.arcLength(ctr, closed=True)).reshape(-1, 2) * map_resolution + origin
        
    # Driving counter-clockwise around the loop puts the infield on the left, so the outer wall on the right
    x, y = centerline[:, 0], centerline[:, 1]
    outer_is_right = np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y) > 0  # shoelace: counter-clockwise order

    bound_right, bound_left = (outer_boundary, inner_boundary) if outer_is_right else (inner_boundary, outer_boundary)

    return bound_right, bound_left


def dist_to_bounds(points: np.ndarray, bound_r: np.ndarray, bound_l: np.ndarray,
                   reverse: bool = False) -> tuple[np.ndarray, np.ndarray]:
    """Distance from every point [x, y] to the nearest point of the right and left track bound, in meters."""

    # KDTree for faster querying
    dists_right = np.asarray(KDTree(bound_r).query(points)[0])
    dists_left = np.asarray(KDTree(bound_l).query(points)[0])
    return (dists_left, dists_right) if reverse else (dists_right, dists_left)


def add_dist_to_cent(centerline: np.ndarray, bound_r: np.ndarray, bound_l: np.ndarray,
                     reverse: bool = False) -> np.ndarray:
    """Centerline [x, y] in meters extended to [x, y, w_right, w_left], the input format of the optimizer."""
    width_right, width_left = dist_to_bounds(centerline, bound_r, bound_l, reverse=reverse)
    return np.column_stack((centerline, width_right, width_left))

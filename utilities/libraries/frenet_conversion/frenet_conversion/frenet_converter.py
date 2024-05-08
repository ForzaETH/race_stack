from typing import Union
import numpy as np
from scipy.interpolate import CubicSpline

class FrenetConverter:
    def __init__(self, waypoints_x: np.array, waypoints_y: np.array, waypoints_psi: np.array):
        self.waypoints_x = waypoints_x
        self.waypoints_y = waypoints_y
        self.waypoints_psi = waypoints_psi
        self.waypoints_s = None
        self.spline_x = None
        self.spline_y = None
        self.raceline_length = None
        self.waypoints_distance_m = 0.1 # [m]
        self.iter_max = 3
        self.closest_index = None

        self.build_raceline()

    def build_raceline(self):
        self.waypoints_s = [0.0]
        prev_wpnt_x =  self.waypoints_x[0]
        prev_wpnt_y =  self.waypoints_y[0]
        for wpnt_x, wpnt_y in zip(self.waypoints_x[1:], self.waypoints_y[1:]):
            dist = np.linalg.norm([wpnt_x - prev_wpnt_x, wpnt_y - prev_wpnt_y])
            prev_wpnt_x = wpnt_x
            prev_wpnt_y = wpnt_y
            self.waypoints_s.append(self.waypoints_s[-1] + dist)        
        self.spline_x = CubicSpline(self.waypoints_s, self.waypoints_x)
        self.spline_y = CubicSpline(self.waypoints_s, self.waypoints_y)
        self.raceline_length = self.waypoints_s[-1]

    def get_frenet(self, x, y, s=None) -> np.array:
        # Compute Frenet coordinates for a given (x, y) point
        self.closest_index = self.get_closest_index(x, y) #update idx for other functionalities
        if s is None:
            s = self.get_approx_s(x, y)
            s, d = self.get_frenet_coord(x, y, s)
        else:
            s, d = self.get_frenet_coord(x, y, s)

        return np.array([s, d])

    def get_approx_s(self, x, y) -> float:
        """
        Finds the s-coordinate of the given point by finding the nearest waypoint.
        """
        # get distance with broadcasting multiple arrays
        lenx = len(x)
        dist_x = x - np.tile(self.waypoints_x, (lenx, 1)).T
        dist_y = y - np.tile(self.waypoints_y, (lenx, 1)).T
        return np.argmin(np.linalg.norm([dist_x.T, dist_y.T], axis=0), axis=1)*self.waypoints_distance_m
    
    def get_frenet_velocities(self, vx, vy, theta) -> np.array:
        """
        Returns the Frenet velocities for the given Cartesian velocities.
        
        Args:
            vx (float): x-velocity
            vy (float): y-velocity
            theta (float): orientation of the vehicle
            
        Returns:
            np.array: [s_dot, d_dot] Frenet velocities
        """
        if self.closest_index is None:
            raise ValueError("FRENET CONVERTER: closest index is None, call get_closest_index first.")
        delta_psi = theta - self.waypoints_psi[self.closest_index]
        s_dot = vx * np.cos(delta_psi) - vy * np.sin(delta_psi)
        d_dot = vx * np.sin(delta_psi) + vy * np.cos(delta_psi)
        
        return np.array([s_dot, d_dot])

    def get_closest_index(self, x, y) -> int:
        """
        Finds the index of the closest waypoint to the given point.
        
        Args:
            x (float): x-coordinate of the point
            y (float): y-coordinate of the point
            
        Returns:
            int: index of the closest waypoint
        """
        # get distance with broadcasting multiple arrays
        lenx = len(x)
        dist_x = x - np.tile(self.waypoints_x, (lenx, 1)).T
        dist_y = y - np.tile(self.waypoints_y, (lenx, 1)).T
        self.closest_index = np.argmin(np.linalg.norm([dist_x.T, dist_y.T], axis=0), axis=1)
        return self.closest_index


    def get_frenet_coord(self, x, y, s, eps_m=0.01) -> float:
        """
        Finds the s-coordinate of the given point, considering the perpendicular
        projection of the point on the track.
        
        Args:
            x (float): x-coordinate of the point
            y (float): y-coordinate of the point
            s (float): estimated s-coordinate of the point
            eps_m (float): maximum error tolerance for the projection. Default is 0.01.
        
        Returns:
            The s-coordinate of the point on the track.
        """
        # Check if point is on the estimated s perpendicular to the track

        _, projection, d = self.check_perpendicular(x, y, s, eps_m)
        for i in range(self.iter_max):
            cand_s = (s + projection)%self.raceline_length
            _, cand_projection, cand_d = self.check_perpendicular(x, y, cand_s, eps_m)
            #print(f"candidate projection: {cand_projection}; projection: {projection}; d: {d} cand_d: {cand_d}")
            cand_projection = np.clip(cand_projection, -self.waypoints_distance_m/(2*self.iter_max), self.waypoints_distance_m/(2*self.iter_max))
            updated_idxs = np.abs(cand_projection) <= np.abs(projection)
            d[updated_idxs] = cand_d[updated_idxs]
            s[updated_idxs] = cand_s[updated_idxs]
            projection[updated_idxs] = cand_projection[updated_idxs]

        return s, d

    def check_perpendicular(self, x, y, s, eps_m=0.01) -> Union[bool, float]:
        # obtain unit vector parallel to the track
        dx_ds, dy_ds = self.get_derivative(s)
        tangent = np.array([dx_ds, dy_ds])
        if np.any(np.isnan(s)):
            raise ValueError("BUB FRENET CONVERTER: S is nan")
        tangent /= np.linalg.norm(tangent, axis=0)

        # obtain vector from the track to the point
        x_vec = x - self.spline_x(s)
        y_vec = y - self.spline_y(s)
        point_to_track = np.array([x_vec, y_vec])
        
        # check if the vectors are perpendicular
        # computes the projection of point_to_track on tangent
        proj = np.einsum('ij,ij->j', tangent, point_to_track)
        perps = np.array([-tangent[1, :], tangent[0, :]])
        d = np.einsum('ij,ij->j', perps, point_to_track)

        # TODO commented out because of computational efficiency
        # eps_m * point_to_track_norm is needed to make it scale invariant 
        # check_perpendicular becomes effectively cos(angle) <= eps_m

        # point_to_track_norm = np.linalg.norm(point_to_track, axis=0)
        # check_perpendicular = np.abs(proj) <= eps_m * point_to_track_norm
        check_perpendicular = None

        return check_perpendicular, proj, d
    
    def get_derivative(self, s) -> np.array:
        """
        Returns the derivative of the point corresponding to s on the chosen line. 
        
        Args: 
            s: parameter which is used to evaluate the spline
            line: argument used to choose the line. Can be 'int', 'mid', 'out'. Default is 'mid'.

        Returns:
            der: dx/ds, dy/ds
        """
        s = s%self.raceline_length

        der = [self.spline_x(s, 1), self.spline_y(s, 1)]
        
        return der
    

    def get_cartesian(self, s: float, d: float) -> np.array:
        """
        Convert Frenet coordinates to Cartesian coordinates
        
        Args:
            s (float): longitudinal coordinate
            d (float): lateral coordinate
            
        Returns:
            np.array: [x, y] Cartesian coordinates
        """
        x = self.spline_x(s)
        y = self.spline_y(s)
        psi = self.get_derivative(s)
        psi = np.arctan2(psi[1], psi[0])
        x += d * np.cos(psi + np.pi / 2)
        y += d * np.sin(psi + np.pi / 2)
        
        return np.array([x, y])
    



from typing import List, Union
import numpy as np
from numpy import arctan2
from scipy.interpolate import InterpolatedUnivariateSpline as Spline
import matplotlib.pyplot as plt

class EnhancedSpline():
    """
    A class that defines a spline, with functions that help.
    """

    def __init__(self, coords, params, track_length, lame = True) -> None:
        """
        Args:
            coords: coordinate that define the spline
            params: parameters that correspon to the coordinates
            track_length: total length
            safety_margin: the track will be shrinked by this amount on both
            sides so that the car will be at a more safe distance from the
            borders

        """
        self.track_length = track_length

        k = 2
        if lame:
            self.line_x = Spline(
                params,
                np.concatenate((coords[:, 0], np.array([coords[0, 0]]))),
                k = k,
                )
            self.line_y = Spline(params,
                np.concatenate((coords[:, 1], np.array([coords[0, 1]]))),
                k = k,
            )
        else:
            self.line_x = Spline(
                params,
                coords[:,0],
                k = k,
                )
            self.line_y = Spline(
                params,
                coords[:,1],
                k = k,
            )

    def get_coordinate(self, theta) -> np.array:
        """
        Returns the coordinate of the point corresponding to theta on the
        chosen line.

        Args:
            theta: parameter which is used to evaluate the spline
        Returns:
            coord: 2-d coordinate
        """
        theta = theta%self.track_length

        coord = np.array([self.line_x(theta), self.line_y(theta)])

        return coord

    def find_theta(self, coord: np.ndarray, theta_est, eps: float = 0.01):
        """
        Find the parameter of the track nearest to the point given an
        approximation of it.

        Args:
            coord: coordinate of point projected around, as np.array
            track: splinified track
            theta_est: best guess at theta
            eps: precision at which the algorithm looks around for theta
        """

        found = False
        max_dist = self.track_length/20

        dist = np.linalg.norm(
            coord - np.array(self.get_coordinate(theta_est)).reshape(2)
            )
        min_dist = dist
        min_theta = theta_est
        theta_i = theta_est
        while not found:
            theta_i += eps
            dist = np.linalg.norm(
                coord - np.array(self.get_coordinate(theta_i)).reshape(2)
                )

            if dist <= min_dist:
                min_dist = dist
                min_theta = theta_i
            else:
                found = True

            if theta_i - theta_est >= max_dist:
                found = True

        # now try to look backwards
        found = False
        theta_i = theta_est
        while not found:
            theta_i -= eps

            dist = np.linalg.norm(
                coord - np.array(self.get_coordinate(theta_i)).reshape(2)
                )

            if dist <= min_dist:
                min_dist = dist
                min_theta = theta_i
            else:
                found = True

            if theta_est - theta_i >= max_dist:
                found = True

        return min_theta

    def get_angle(self, theta) -> float:
        """
        Returns the angle wrt x axis tangent to the line given the
        parameter theta
        """
        theta = theta%self.track_length
        delt_y = self.line_y(theta, 1)
        delt_x = self.line_x(theta, 1)

        angle = arctan2(delt_y, delt_x)
        if angle<0:
            angle += 2*np.pi

        return angle

    def get_derivative(self, theta):
        """
        Returns the derivative of the point corresponding to theta.

        Args:
            theta: parameter which is used to evaluate the spline

        Returns:
            der: dx/dtheta, dy/dtheta
        """
        theta = theta%self.track_length

        der = [self.line_x(theta, 1), self.line_y(theta, 1)]

        return der

    def get_curvature(self, theta):
        """
        Returns the derivative of the point corresponding to theta.

        Args:
            theta: parameter which is used to evaluate the spline

        Returns:
            der: dx/dtheta, dy/dtheta
        """
        theta = theta%self.track_length

        x_d = self.line_x(theta, 1)
        x_dd = self.line_x(theta, 2)
        y_d = self.line_y(theta, 1)
        y_dd = self.line_y(theta, 2)

        curvature = (
            np.abs(x_d * y_dd - y_d * x_dd)/pow((x_d**2 + y_d **2), 1.5)
        )

        return curvature

    def get_dphi_dtheta(self, theta, line='mid'):
        """
        Returns the derivative of the angle corresponding to theta on the
        chosen line.

        Args:
            theta: parameter which is used to evaluate the spline
            line: argument used to choose the line. Can be 'int', 'mid',
            'out'. Default is 'mid'.

        Returns:
            der: dx/dtheta, dy/dtheta
        """
        theta = theta%self.track_length


        x_d = self.line_x(theta, 1)
        x_dd = self.line_x(theta, 2)
        y_d = self.line_y(theta, 1)
        y_dd = self.line_y(theta, 2)

        derivative = (x_d * y_dd - y_d * x_dd)/(x_d**2 + y_d **2)

        return derivative

    def find_theta_slow(self, coord: np.ndarray, eps: float = 0.1):
        """
        Find the parameter of the line nearest to the point without an
        approximation of it.
        Could be slower than find_theta as it looks along the whole line.

        Args:
            coord: coordinate of point projected around, as np.array
            track: splinified track
            eps: precision at which the algorithm looks around for theta
        """

        found = False
        dist = np.linalg.norm(coord - np.array(self.get_coordinate(0)))

        min_dist = dist
        min_theta = 0
        theta_i = 0

        while theta_i <= self.track_length:
            theta_i += eps
            dist = np.linalg.norm(
                coord - np.array(self.get_coordinate(theta_i))
                )

            if dist <= min_dist:
                min_dist = dist
                min_theta = theta_i


        return min_theta

    def is_coord_behind(self, coord: np.ndarray, theta: float):
        """
        Checks if the given coordinate is behind the given theta.
        Forward is defined as the track direction and a line is
        centered in the point corresponding to theta to divide
        "in front of" and "behind".
        The line is perpendicular to the direction of the line.

        Args:
            coord: coordinate to be checked
            theta: parameter of the spline for reference

        Returns:
            <bool>: true if the cordinate is behind, false if the
                coordinate is in front of the line
        """

        angle = self.get_angle(theta)
        norm_vec = np.array([np.cos(angle), np.sin(angle)])
        coord_on_boundary = self.get_coordinate(theta)
        coord_forward = self.get_coordinate(theta + 0.01)
        q_forward = np.dot(coord_forward, norm_vec)
        q_bound = np.dot(coord_on_boundary, norm_vec)
        q_coord = np.dot(coord, norm_vec)

        if (q_coord-q_bound)*(q_forward-q_bound) < 0:
            return True
        else:
            return False

class SplineTrack():
    """
    a class to represent the splinification of the track, with the
    center line and the two sides of the track
    """

    def __init__(
        self,
        path_to_file = None,
        coords_direct = None,
        path_to_traj = None,
        safety_margin: float= 0.0,
        ) -> None:
        """
        Args:
            path_to_file: path to the file where the coordinates of the
                track are saved
            coords_direct: array of shape ()
            path_to_traj: path to the file where the coordinates of the
                trajectory are saved
            safety_margin: the track will be shrinked by this amount on
                both sides so that the car will be at a more safe distance
                from the borders

        """

        # 1 Obtain points on the track #
        #################################
        if path_to_file is not None:
            coords = read_coords_from_file(path_to_file)
            coords = np.array(coords)
            params, self.track_length = find_params(coords[1])
        elif coords_direct is not None:
            coords = coords_direct
            params, self.track_length = find_params(coords[1])
        else:
            raise NotImplementedError("neither coordinates or the path to the coordinates was given")

        # 2 shrink the track for robustness #
        #####################################
        for i in range(len(params)-1):
            # shrink int line
            direction = coords[1,i,:] - coords[0,i,:]
            coords[0,i,:] = coords[0,i,:] + \
                safety_margin*direction/np.linalg.norm(direction+0.001)

            # shrink out line
            direction = coords[1,i,:] - coords[2,i,:]
            coords[2,i,:] = coords[2,i,:] + \
                safety_margin*direction/np.linalg.norm(direction+0.001)

        # 3 set track #
        ###############
        self.refline = EnhancedSpline(
            coords[1,:,:], params, self.track_length
            )
        self.intline = EnhancedSpline(
            coords[0,:,:], params, self.track_length
            )
        self.outline = EnhancedSpline(
            coords[2,:,:], params, self.track_length
            )

        # 4 set trajectory #
        ####################
        if path_to_traj is not None:
            traj_coords = read_sing_coords_from_file(path_to_traj)
            traj_coords = np.array(traj_coords)
            params, self.traj_len = find_params(traj_coords)
            self.trajectory = EnhancedSpline(
                traj_coords,
                params,
                self.track_length,
                )
            self.delta_traj = self.trajectory.find_theta_slow(
                self.refline.get_coordinate(0)
                )
        else:
            self.trajectory = self.refline
            self.traj_len = self.track_length
            self.delta_traj = 0


    def get_angle(self, theta, line = 'mid') -> float:
        """
        Returns the angle wrt x axis tangent to the midline interpolation
        given the parameter theta
        """

        if line == 'mid':
            return self.refline.get_angle(theta)
        elif line == 'int':
            return self.intline.get_angle(theta)
        elif line == 'out':
            return self.outline.get_angle(theta)
        else:
            raise ValueError("line can only be 'int', 'mid', or 'out'.")

    def get_coordinate(self, theta, line = 'mid') -> np.array:
        """
        Returns the coordinate of the point corresponding to theta on the
        chosen line.

        Args:
            theta: parameter which is used to evaluate the spline
            line: argument used to choose the line. Can be 'int', 'mid',
            'out'. Default is 'mid'.

        Returns:
            coord: 2-d coordinate
        """

        if line == 'mid':
            coord = self.refline.get_coordinate(theta)
        elif line == 'int':
            coord = self.intline.get_coordinate(theta)
        elif line == 'out':
            coord = self.outline.get_coordinate(theta)
        else:
            raise ValueError("line can only be 'int', 'mid', or 'out'.")

        return coord

    def get_derivative(self, theta, line = 'mid'):
        """
        Returns the derivative of the point corresponding to theta on the
        chosen line.

        Args:
            theta: parameter which is used to evaluate the spline
            line: argument used to choose the line. Can be 'int', 'mid',
            'out'. Default is 'mid'.

        Returns:
            der: dx/dtheta, dy/dtheta
        """
        theta = theta%self.track_length

        if line == 'mid':
            der = self.refline.get_derivative(theta)
        elif line == 'int':
            der = self.intline.get_derivative(theta)
        elif line == 'out':
            der = self.outline.get_derivative(theta)
        else:
            raise ValueError("line can only be 'int', 'mid', or 'out'.")

        return der

    def get_curvature(self, theta, line='mid'):
        """
        Returns the derivative of the point corresponding to theta on the
        chosen line.

        Args:
            theta: parameter which is used to evaluate the spline
            line: argument used to choose the line. Can be 'int', 'mid',
            'out'. Default is 'mid'.

        Returns:
            der: dx/dtheta, dy/dtheta
        """
        theta = theta%self.track_length

        if line == 'mid':
            curvature = self.refline.get_curvature(theta)
        elif line == 'int':
            curvature = self.intline.get_curvature(theta)
        elif line == 'out':
            curvature = self.outline.get_curvature(theta)
        else:
            raise ValueError("line can only be 'int', 'mid', or 'out'.")

        return curvature

    def get_dphi_dtheta(self, theta, line='mid'):
        """
        Returns the derivative of the angle corresponding to theta on the
        chosen line.

        Args:
            theta: parameter which is used to evaluate the spline
            line: argument used to choose the line. Can be 'int', 'mid',
            'out'. Default is 'mid'.

        Returns:
            der: dx/dtheta, dy/dtheta
        """
        theta = theta%self.track_length

        if line == 'mid':
            derivative = self.refline.get_dphi_dtheta(theta)
        elif line == 'int':
            derivative = self.intline.get_dphi_dtheta(theta)
        elif line == 'out':
            derivative = self.outline.get_dphi_dtheta(theta)
        else:
            raise ValueError("line can only be 'int', 'mid', or 'out'.")

        return derivative

    def find_theta(self, coord: np.ndarray, theta_est, eps: float = 0.01):
        """
        Find the parameter of the track nearest to the point given an
        approximation of it.

        Args:
            coord: coordinate of point projected around, as np.array
            track: splinified track
            theta_est: best guess at theta
            eps: precision at which the algorithm looks around for theta
        """

        min_theta = self.refline.find_theta(coord, theta_est, eps)

        return min_theta

    def find_theta_slow(self, coord: np.ndarray, eps: float = 0.1):
        """
        Find the parameter of the track nearest to the point without an
        approximation of it.
        Could be slower

        Args:
            coord: coordinate of point projected around, as np.array
            track: splinified track
            eps: precision at which the algorithm looks around for theta
        """

        min_theta = self.refline.find_theta_slow(coord, eps)

        return min_theta

    def is_coord_behind(self, coord: np.ndarray, theta: float):
        return self.refline.is_coord_behind(coord, theta)

class SplineTrackNew():
    """
    a class to represent the splinification of the track, with the center
    line, and the width

    we assume int is left and out is right
    """

    def __init__(
        self,
        path_to_file = None,
        path_to_traj = None,
        coords_param_direct = None,
        params = None,
        track_length = None,
        safety_margin: float= 0.0,
        **kwargs
        ) -> None:
        """
        a class to represent the splinification of the track, with the center
        line, and the width

        we assume int is left and out is right

        Either the dictionary coords_param_direct must be given or the path to file,
        otherwise it does not work.


        Args:
            path_to_file: path to the file where the coordinates of the
                track are saved
            path_to_traj: path to the file where the coordinates of the
                trajectory are saved
            coords_param_direct: an optional dictionary to provide the coordinates directly
                coords: a (N, 2) np.array that contains the centerline coordinates
                left_widths: a (N, 1) np.array that contains the left widths of the track
                right_widths: a (N, 1) np.array that contains the right widths of the track
            params: the parameters that correspond to the point, if they are not given they
                will be automatically computed. They are assumed to be ordered!!!
                They also should be longer than coordintaes by one, as the last coordinate
                is also the first (cit. Jesus)
            track_length: the track length
            safety_margin: the track will be shrinked by this amount on
                both sides so that the car will be at a more safe distance
                from the borders

        """


        if coords_param_direct is None:
            coords, left_widths, right_widths = read_coords_from_file_new(
                path_to_file
                )
            if params is None:
                params, self.track_length = find_params(coords)
            else:
                params = params
                self.track_length = track_length
        elif coords_param_direct is not None:
            coords = coords_param_direct['coords']
            left_widths = coords_param_direct['left_widths']
            right_widths = coords_param_direct['right_widths']
            if params is None:
                params, self.track_length = find_params(coords)
            else:
                params = params
                self.track_length = track_length

        # shrink the track for robustness
        left_widths -= safety_margin
        right_widths -= safety_margin
        left_widths = np.clip(left_widths, 0, None)
        right_widths = np.clip(right_widths, 0, None)

        self.refline = EnhancedSpline(coords, params[:-1], self.track_length, lame=False)
        self.left_widths = Spline(params[:-1],left_widths)
        self.right_widths = Spline(params[:-1],right_widths)

        if path_to_traj is not None:
            traj_coords = read_sing_coords_from_file(path_to_traj)
            traj_coords = np.array(traj_coords)
            params, self.traj_len = find_params(traj_coords)
            self.trajectory = EnhancedSpline(
                traj_coords, params, self.track_length
                )
            self.delta_traj = self.trajectory.find_theta_slow(
                self.refline.get_coordinate(0)
                )
        else:
            self.trajectory = self.refline
            self.traj_len = self.track_length
            self.delta_traj = 0

        # take some optional keyword arguments
        norm_vecs_x = coords_param_direct.get("norm_vecs_x", None)
        norm_vecs_y = coords_param_direct.get("norm_vecs_y", None)
        if norm_vecs_x is not None and norm_vecs_y is not None:
            self.norms = True
            self.norm_vecs_x = Spline(params[:-1], norm_vecs_x)
            self.norm_vecs_y = Spline(params[:-1], norm_vecs_y)
        else:
            rot_90 = np.array([[np.cos(-np.pi/2), -np.sin(-np.pi/2)],
                               [np.sin(-np.pi/2), np.cos(-np.pi/2)]])
            norm_vecs_x = np.zeros_like(left_widths)
            norm_vecs_y = np.zeros_like(left_widths)
            for i, param in enumerate(params[:-1]):
                tanget_vec = self.refline.get_derivative(param)
                norm_vec = rot_90@np.array(tanget_vec)
                norm_vec /= np.linalg.norm(norm_vec)
                norm_vecs_x[i] = norm_vec[0]
                norm_vecs_y[i] = norm_vec[1]

            self.norms = True
            self.norm_vecs_x = Spline(params[:-1], norm_vecs_x)
            self.norm_vecs_y = Spline(params[:-1], norm_vecs_y)

    def get_angle(self, theta, line = 'mid') -> float:
        """
        Returns the angle wrt x axis tangent to the midline interpolation
        given the parameter theta
        """

        if line == 'mid':
            return self.refline.get_angle(theta)
        else:
            raise ValueError("line can only be 'int'")

    def get_coordinate(self, theta, line = 'mid'):
        """
        Returns the coordinate of the point corresponding to theta on the
        chosen line.

        Args:
            theta: parameter which is used to evaluate the spline
            line: argument used to choose the line. Can be 'int', 'mid',
            'out'. Default is 'mid'.

        Returns:
            coord: 2-d coordinate
        """

        theta = theta%self.track_length

        coord = self.refline.get_coordinate(theta)
        tang = self.get_derivative(theta)
        tang = tang/np.linalg.norm(tang)

        if line == 'mid':
            pass
        elif line == 'int':
            left_rot = [[0, -1],[1, 0]]
            coord += self.left_widths(theta) * left_rot@(tang)
        elif line == 'out':
            right_rot = [[0, 1],[-1, 0]]
            coord += self.right_widths(theta) * right_rot@(tang)
        else:
            raise ValueError("line can only be 'int', 'mid', or 'out'.")

        return coord

    def get_frenet_coordinate(self, theta, displacement):
        """
        Returns the coordinate of the point corresponding to theta on the
        chosen line.

        Args:
            theta: s parameter to indicate where to take the point
            displacement: lateral distance to center line. Positive is left
                when considering forward direction.

        Returns:
            coord: 2-d coordinate
        """

        coord = self.refline.get_coordinate(theta)

        if self.norms:
            norm_v = np.array([
                self.norm_vecs_x(theta),
                self.norm_vecs_y(theta)
            ])
            # renormalize for safety
            norm_v /= np.linalg.norm(norm_v)

            return coord - norm_v*displacement
        else:
            raise NotImplementedError(
                "Normals are not available, it is currently impossible to get frenet coordinate"
            )




        return coord

    def get_derivative(self, theta, line = 'mid'):
        """
        Returns the derivative of the point corresponding to theta on the
        chosen line.

        Args:
            theta: parameter which is used to evaluate the spline
            line: argument used to choose the line. Can be 'int', 'mid',
            'out'. Default is 'mid'.

        Returns:
            der: dx/dtheta, dy/dtheta
        """
        theta = theta%self.track_length

        if line == 'mid':
            der = self.refline.get_derivative(theta)
        else:
            raise ValueError("line can only be 'int'.")

        return der

    def get_curvature(self, theta, line='mid'):
        """
        Returns the derivative of the point corresponding to theta on the
        chosen line.

        Args:
            theta: parameter which is used to evaluate the spline
            line: argument used to choose the line. Can be 'int', 'mid',
            'out'. Default is 'mid'.

        Returns:
            der: dx/dtheta, dy/dtheta
        """
        theta = theta%self.track_length

        if line == 'mid':
            curvature = self.refline.get_curvature(theta)
        else:
            raise ValueError("line can only be 'int'.")

        return curvature

    def get_dphi_dtheta(self, theta, line='mid'):
        """
        Returns the derivative of the angle corresponding to theta on the
        chosen line.

        Args:
            theta: parameter which is used to evaluate the spline
            line: argument used to choose the line. Can be 'int', 'mid',
            'out'. Default is 'mid'.

        Returns:
            der: dx/dtheta, dy/dtheta
        """
        theta = theta%self.track_length

        if line == 'mid':
            derivative = self.refline.get_dphi_dtheta(theta)
        else:
            raise ValueError("line can only be 'int', 'mid', or 'out'.")

        return derivative

    def find_theta(self, coord: np.ndarray, theta_est, eps: float = 0.01):
        """
        Find the parameter of the track nearest to the point given an
        approximation of it.

        Args:
            coord: coordinate of point projected around, as np.array
            track: splinified track
            theta_est: best guess at theta
            eps: precision at which the algorithm looks around for theta
        """

        min_theta = self.refline.find_theta(coord, theta_est, eps)

        return min_theta

    def find_theta_slow(self, coord: np.ndarray, eps: float = 0.1):
        """
        Find the parameter of the track nearest to the point without an
         approximation of it.
        Could be slower

        Args:
            coord: coordinate of point projected around, as np.array
            track: splinified track
            eps: precision at which the algorithm looks around for theta
        """

        min_theta = self.refline.find_theta_slow(coord, eps)

        return min_theta

    def is_coord_behind(self, coord: np.ndarray, theta: float):
        return self.refline.is_coord_behind(coord, theta)

    def plot(sp, min_x, max_x, num_points):
        # Example: Creating a spline (replace this with your spline object)
        x_values = np.linspace(min_x, max_x, num_points)
        y_values = sp(x_values)

        # Plotting
        plt.plot(x_values, y_values)
        plt.xlabel('X-axis')
        plt.ylabel('Y-axis')
        plt.title('Interpolated Univariate Spline')
        plt.show()

def read_coords_from_file(path_to_file: str):
    out_arr = []
    mid_arr = []
    int_arr = []

    with open(path_to_file, 'r') as file:
        coords = file.readlines()

    coords = coords[1:] # discard 1st line

    for el in coords:
        out, mid, inn = el.strip().split('], [')
        out_arr.append([float(el) for el in out.strip('[],').split(',')])
        mid_arr.append([float(el) for el in mid.strip('[],').split(',')])
        int_arr.append([float(el) for el in inn.strip('[],').split(',')])

    return out_arr, mid_arr, int_arr

def read_coords_from_file_new(path_to_file: str):
    """
    Reads coordinates from a file that save a track this way:

        first line:
            length: <number of coordinates>

        second line:
            center_line_x, centerline_y, left_width, right_width

        other lines:
            [numbers int the order specified above]

    Args:
        path_to_file: path to the coordinates' file

    Returns:
        center_line: np.array of shape (<length>, 2)
        left_widths: np.array of shape (<length>,)
        right_widths: np.array of shape (<length>,)
    """

    with open(path_to_file, 'r') as file:
        coords = file.readlines()

    # read file length
    length = int(coords[0].strip(', \n').split(':')[1])
    center_line = np.empty((length, 2))
    left_widths = np.empty((length))
    right_widths = np.empty((length))

    coords = coords[2:] # discard first two lines lines

    for i, el in enumerate(coords):
        center_line_x, center_line_y, left_w, right_w, _ = el.strip().split(',')
        center_line[i, :] = float(center_line_x), float(center_line_y)
        left_widths[i] = float(left_w)
        right_widths[i] = float(right_w)

    return center_line, left_widths, right_widths

def read_sing_coords_from_file(path_to_file: str):

    arr = []

    with open(path_to_file, 'r') as file:
        coords = file.readlines()

    coords = coords[1:] # discard 1st line

    for el in coords:
        out = el.strip().split(',')
        arr.append([float(el) for el in out])

    return arr

def find_params(coords: np.ndarray):
    """
    Finds the parameters to fit the spline(basically x axis for
    interpolation). Should be only used with central line.

    Returns:
        cum_param: an array of the parameters of shape len(coords) + 1
        length: length of the track
    """

    cum_param = [0]

    prev_el = coords[0]
    length = 0

    for el in coords[1:]:
        length += np.linalg.norm((el-prev_el))
        prev_el = el
        cum_param.append(length)

    length += np.linalg.norm(el - coords[0])
    cum_param.append(length)

    return cum_param, length

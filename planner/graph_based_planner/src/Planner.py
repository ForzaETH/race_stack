#!/usr/bin/env python3
import sys
import os

os.environ['OPENBLAS_NUM_THREADS'] = "1"
import numpy as np
import rospy
import uuid
import hashlib
import time
import pstats
import cProfile
from math import sin, cos

import GraphBasedPlanner.graph_ltpl.Graph_LTPL
import GraphBasedPlanner.graph_ltpl.imp_global_traj.src.import_globtraj_csv

import rospkg
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from dynamic_reconfigure.msg import Config
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker, MarkerArray
from f110_msgs.msg import WpntArray, Wpnt, ObstacleArray
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, Float32


class GraphPlanner:
    def __init__(self):
        self.waypoints = None # Global waypoints used by this planner
        self.current_position = None
        self.current_position_frenet = None
        self.current_heading = None
        self.current_speed = None
        self.waypoints_original = None #Standard global waypoints
        self.map_name = None
        self.obstacle_arr = None # Obstacle array
        self.speed_scaling = None # Speed scaling for car
        self.max_s = None # Max s coordinate for specific map
        self.timer = None
        self.wp_flag = False # Flag used to save compute
        self.close_to_s0 = False # Flag that tells if car is close to s=0
        self.examined_points = 80 # Number of examined points in deciding between left or right overtake
        self.display_graph = rospy.get_param('/GraphPlanner/display_graph', True) # Send graph markers or not
        self.testing = rospy.get_param('/GraphPlanner/testing', True) # True if on real car, False if simulation
        self.follow_mode = rospy.get_param('/GraphPlanner/follow_mode', 0) # Set to 2 for aggressive follow, 0 brakes often
        self.waypoint_distance = 0.1 # [m] distance between waypoints in global trajectory
        self.num_waypoints = 0 # Number of waypoints in global trajectory
        pkg_path = rospkg.RosPack().get_path("graph_based_planner")
        self.measuring = rospy.get_param("/measure", False)
        
        # Profiling stuff
        self.profile = rospy.get_param("/profile", False)
        self.profiler = cProfile.Profile()
        self.profile_filename = pkg_path + "/logs/" + "graph_based_profile_log.prof" # TODO change name 
        self.profile_report_filename = pkg_path + "/logs/" + "graph_based_profile_report.txt" # TODO change name

        # Subscribed topics
        rospy.Subscriber('/global_waypoints', WpntArray, self.waypoints_cb)
        rospy.Subscriber('/car_state/odom', Odometry, self.odom_cb)
        rospy.Subscriber('/perception/obstacles', ObstacleArray, self.obstacles_cb)
        rospy.Subscriber('/dyn_sector_server/parameter_updates', Config, self.dyn_params_cb)
        rospy.Subscriber('/car_state/odom_frenet', Odometry, self.odom_fre_cb)

        # Published topics
        self.lattice_visualizer = rospy.Publisher('/lattice_viz', MarkerArray, queue_size=10)
        self.action_mrk = rospy.Publisher('/action_marker', Marker, queue_size=10)
        self.path_pub = rospy.Publisher('/planner/graph_based_wpnts', Float32MultiArray, queue_size=10)
        if self.measuring:
            self.latency_pub = rospy.Publisher("/planner/graph_based/latency", Float32, queue_size=10)

        self.toppath = os.path.dirname(os.path.realpath(__file__))
        sys.path.append(self.toppath)
        self.toppath = self.toppath + "/GraphBasedPlanner"
        sys.path.append(self.toppath)

        # Define all relevant paths
        self.set_map_name()
        self.path_dict = {
            'globtraj_input_path': self.toppath + "/inputs/traj_ltpl_cl/traj_ltpl_cl_" + self.map_name + ".csv",
            'graph_store_path': self.toppath + "/inputs/stored_graph.pckl",
            'ltpl_offline_param_path': self.toppath + "/params/ltpl_config_offline.ini",
            'ltpl_online_param_path': self.toppath + "/params/ltpl_config_online.ini",
        }
        
        # load ax max
        racecar_version = rospy.get_param("/racecar_version")
        sys_id_path = rospkg.RosPack().get_path("steering_lookup")
        ax_max_path = str(sys_id_path) + f"/../id_analyser/models/{racecar_version}/ax_max_machines.csv"
        # load ax max skipping first line
        self.ax_max = np.loadtxt(ax_max_path, delimiter=',', skiprows=1)
        
        # TODO hardcoding, these could all be read from a config
        self.trailing_distance = 1
        self.v_max = 15
        

    #############
    # CALLBACKS #
    #############
    def dyn_params_cb(self, data: Config):
        """
                Callback function of /obstacles subscriber.

                Parameters
                ----------
                data
                    Data received from /obstacles topic
        """
        self.speed_scaling = data.doubles[0].value

    def obstacles_cb(self, data: ObstacleArray):
        """
                Callback function of /obstacles subscriber.

                Parameters
                ----------
                data
                    Data received from /obstacles topic
        """
        self.obstacle_arr = np.array([
            [obstacle.id, obstacle.s_start, obstacle.s_end, obstacle.d_left, obstacle.d_right,
             obstacle.is_actually_a_gap, obstacle.s_center, obstacle.d_center, obstacle.size,
             obstacle.vs, obstacle.vd, obstacle.is_static] for obstacle in data.obstacles
        ])

    def odom_cb(self, data: Odometry):
        """
                Callback function of /car_state/odom subscriber.

                Parameters
                ----------
                data
                    Data received from /car_state/odom topic
        """
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        self.current_position = np.array([x, y])
        self.current_speed = data.twist.twist.linear.x
        self.current_heading = euler_from_quaternion([data.pose.pose.orientation.x,
                                                      data.pose.pose.orientation.y,
                                                      data.pose.pose.orientation.z,
                                                      data.pose.pose.orientation.w])[2]

    def odom_fre_cb(self, data: Odometry):
        """
                Callback function of /car_state/odom_frenet subscriber.

                Parameters
                ----------
                data
                          Data received from /car_state/odom_frenet topic
        """
        s = data.pose.pose.position.x
        d = data.pose.pose.position.y
        self.current_position_frenet = np.array([s, d])

    def waypoints_cb(self, data: WpntArray):
        """
                Callback function of /global_waypoints subscriber.

                Parameters
                ----------
                data
                    Data received from /global_waypoints topic
        """
        if not self.wp_flag:
            # Store original waypoint array
            self.waypoints_original = np.array([
                [w.s_m, w.d_m, w.x_m, w.y_m,
                 w.d_right, w.d_left, w.psi_rad, w.kappa_radpm, w.vx_mps, w.ax_mps2] for w in data.wpnts
            ])
            mylist = []
            for wpnt in data.wpnts:
                # Get normalized normal vector from frenet space
                vector_base_glob = self.frenettoglob(wpnt.s_m, 0.0)
                vector_tip_glob = self.frenettoglob(wpnt.s_m, -1.0)
                dir = np.asarray([vector_tip_glob[0] - vector_base_glob[0], vector_tip_glob[1] - vector_base_glob[1]])
                norm_vec = self.unit_vector(dir)

                # Construct new waypoints with normal vector and alpha parameter set to 0 (which means refline == raceline)
                new_wpnt = [wpnt.x_m, wpnt.y_m, wpnt.d_right, wpnt.d_left,
                            norm_vec[0], norm_vec[1], 0.0, wpnt.s_m, wpnt.psi_rad,
                            wpnt.kappa_radpm, wpnt.vx_mps, wpnt.ax_mps2]
                mylist.append(new_wpnt)

            self.waypoints = np.asarray(mylist)
            self.wp_flag = True
            self.num_waypoints = len(self.waypoints)
        else:
            pass
    
    #############
    # FUNCTIONS #
    #############
    def set_map_name(self):
        """
                Sets the proper name for the map

        """
        if not self.testing:
            complete_name = rospy.get_param(
                '/f1tenth_simulator/image')  # If simulating get map name automatically, else get it from launch
            self.map_name = complete_name[:-4]

        else:
            self.map_name = rospy.get_param('/GraphPlanner/map_name')

    def create_map_csv(self) -> bool:
        """
                Function that checks it there is a pre-existing trajectory file for that specific map, and creates one
                if there isn't such file

                Returns
                ----------
                True
                    If the pre-existing trajectory file does not exist and the new file is generated correctly
                False
                    Found a pre-existing file, no further operation needed
        """
        pt = self.toppath + "/inputs/traj_ltpl_cl/traj_ltpl_cl_" + self.map_name + ".csv"
        if os.path.exists(pt):
            return False

        # Create random UUID
        rand_uuid = str(uuid.uuid4())

        # Hash ggv file with SHA1
        ggv_content = np.array([])
        ggv_hash = hashlib.sha1(ggv_content).hexdigest()

        # write UUID and GGV hash into file

        with open(pt, 'w') as fh:
            fh.write("# " + rand_uuid + "\n")
            fh.write("# " + ggv_hash + "\n")

        # Export trajectory data for local planner
        header = "x_ref_m; y_ref_m; width_right_m; width_left_m; x_normvec_m; y_normvec_m; " \
                 "alpha_m; s_racetraj_m; psi_racetraj_rad; kappa_racetraj_radpm; vx_racetraj_mps; ax_racetraj_mps2"
        fmt = "%.7f; %.7f; %.7f; %.7f; %.7f; %.7f; %.7f; %.7f; %.7f; %.7f; %.7f; %.7f"
        with open(pt, 'ab') as fh:
            np.savetxt(fh, self.waypoints, fmt=fmt, header=header)

        return True

    def visualize_lattice(self, graphbase):
        """
                Creates and sends markers to visualize the graph in RVIZ

                Parameters
                ----------
                graphbase
                    GraphBase object which holds all the spatial information about the graph
        """
        ############### NODES VISUALIZATION ##################
        i = 0
        nodes = graphbase.get_nodes()  # gets list of tuples (numberoflayer, nodenumber)
        node_markers = MarkerArray()
        # Fill node_markers array
        for node in nodes:
            node_pos = graphbase.get_node_info(node[0], node[1])[
                0]  # Takes layer and node number, gives an array. in position 0 you have the node's x and y
            node_marker = Marker()
            node_marker.header.frame_id = "map"
            node_marker.header.stamp = rospy.Time.now()
            node_marker.type = 2  # Sphere type
            node_marker.id = i
            node_marker.scale.x = 0.1
            node_marker.scale.y = 0.1
            node_marker.scale.z = 0.1
            node_marker.color.r = 0 / 255
            node_marker.color.g = 151 / 255
            node_marker.color.b = 252 / 255
            node_marker.color.a = 1.0
            node_marker.pose.position.x = node_pos[0]
            node_marker.pose.position.y = node_pos[1]
            node_marker.pose.position.z = 0
            node_marker.pose.orientation.x = 0
            node_marker.pose.orientation.y = 0
            node_marker.pose.orientation.z = 0
            node_marker.pose.orientation.w = 1
            node_markers.markers.append(node_marker)
            i = i + 1
        # Publish nodes
        self.lattice_visualizer.publish(node_markers)

        ############## EDGES VISUALIZATION ################

        edges = graphbase.get_edges()
        edge_markers = MarkerArray()

        # Get min and max cost for colored visualization
        mylist = []
        for edge in edges:
            spline = graphbase.get_edge(edge[0], edge[1], edge[2], edge[3])
            spline_cost = spline[2]
            mylist.append(spline_cost)
        costs = np.asarray(mylist)
        max_cost = np.amax(costs)
        min_cost = np.amin(costs)

        # Fill edge_markers array
        for edge in edges:
            spline = graphbase.get_edge(edge[0], edge[1], edge[2], edge[3])
            spline_coords = spline[1][:, 0:2]
            #if not self.debug:
                #spline_coords = [spline_coords[0], spline_coords[int(len(spline_coords) / 2)], spline_coords[-1]]
            spline_cost = spline[2]
            edge_marker = Marker()
            edge_marker.points = []
            for sampled_point in spline_coords:
                p = Point()
                p.x = sampled_point[0]
                p.y = sampled_point[1]
                p.z = 0.0
                edge_marker.points.append(p)
            edge_marker.header.frame_id = "map"
            edge_marker.header.stamp = rospy.Time.now()
            edge_marker.type = 4  # Type LINE STRIP
            edge_marker.id = i
            edge_marker.scale.x = 0.01
            edge_marker.pose.orientation.x = 0
            edge_marker.pose.orientation.y = 0
            edge_marker.pose.orientation.z = 0
            edge_marker.pose.orientation.w = 1
            edge_marker.color.a = 1.0
            # Normalize cost of the particular edge in a [0,1] interval
            norm_cost = (spline_cost - min_cost) / (max_cost - min_cost)
            # Set colors based on cost
            if norm_cost <= 0.1:
                edge_marker.color.r = 41 / 255
                edge_marker.color.g = 171 / 255  # dark green
                edge_marker.color.b = 67 / 255
            elif 0.1 < norm_cost <= 0.25:
                edge_marker.color.r = 43 / 255
                edge_marker.color.g = 209 / 255  # light green
                edge_marker.color.b = 65 / 255
            elif 0.25 < norm_cost <= 0.5:
                edge_marker.color.r = 255 / 255
                edge_marker.color.g = 255 / 255  # yellow
                edge_marker.color.b = 0 / 255
            elif 0.5 < norm_cost <= 0.75:
                edge_marker.color.r = 255 / 255
                edge_marker.color.g = 126 / 255  # orange
                edge_marker.color.b = 0 / 255
            elif 0.75 < norm_cost <= 1:
                edge_marker.color.r = 255 / 255
                edge_marker.color.g = 0 / 255  # red
                edge_marker.color.b = 0 / 255

            edge_markers.markers.append(edge_marker)
            i = i + 1
        rospy.loginfo("[Graph Based Planner] Number of nodes and edges is " + str(i))
        # Publish edge markers
        self.lattice_visualizer.publish(edge_markers)

    def get_closest_obs(self):
        """Function that returns the closest object to the car ( only objects in front of the car are taken into
        account

        Returns
        ----------
        None
            No object in front of the vehicle is detected
        Object Array
            1d array containing the obstacle information
            [obj_id, s_start, s_end, d_right, d_left, is_actually_a_gap]
        """
        if len(self.obstacle_arr) == 0:
            return None
        obs_arr_copy = self.obstacle_arr
        # Form a matrix with column 0 being the s coordinate of obstacles, column 1 the d coordinate, column 2 the
        # obstacle id
        matr = np.concatenate((obs_arr_copy[:, 1].reshape(len(obs_arr_copy[:, 0]), 1),
                               ((obs_arr_copy[:, 3] + obs_arr_copy[:, 4])
                                / 2).reshape(len(obs_arr_copy[:, 0]), 1),
                               obs_arr_copy[:, 0].reshape(len(obs_arr_copy[:, 0]), 1)),
                              axis=1)
        ls = []
        for obs in matr:
            glob = self.frenettoglob(obs[0], obs[1])
            ls.append([glob[0], glob[1], obs[2], obs[0] - self.current_position_frenet[0]])
        glob_mat = np.asarray(ls)
        # Take only obstacles with s greater than s_ego if the car is not close to s0
        if not self.close_to_s0:
            glob_mat = glob_mat[glob_mat[:, -1] >= 0]
        # Get closest obstacle
        if glob_mat.shape[0] != 0:
            idx = np.argmin(np.linalg.norm(np.subtract(glob_mat[:, :2], self.current_position), axis=1))
            return obs_arr_copy[obs_arr_copy[:, 0] == glob_mat[idx, 2]]
        else:
            return None

    def find_global_index(self, s: float) -> int:
        """Calculate the index of the element in the array which is closest in absolute value to the s.

        Parameters
        ----------
        s
            Reference value

        Returns
        ----------
        int
            Index of the element in the array which is closest in absolute value to the station
        """
        return (np.round(s/self.waypoint_distance).astype(int))%self.num_waypoints

    def unit_vector(self, vector):
        """Calculates the unit vector of a vector

        Parameters
        ----------
        vector
            Multi dimensional vector

        Returns
        ----------
        vector
            Normalized vectort
        """
        return vector / np.linalg.norm(vector) if np.linalg.norm(vector) != 0 else vector

    def angle_between(self, v1, v2) -> float:
        """Calculate the angle (radians) between two vectors

        Parameters
        ----------
        v1
            Vector 1
        v2
            Vector 2

        Returns
        ----------
        float
            Angle between the two vector (in radians)
        """
        v1_u = self.unit_vector(v1)
        v2_u = self.unit_vector(v2)
        return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

    def get_obj_list(self):
        """
                Calculates a list of dicts, each describing an obstacle (static or dynamic).
                An object's fields aree the following: 'id' , 'type', 'X', 'Y', 'theta', 'v', 'lenght', 'width'
                Type is always 'physical', theta is the heading, v is velocity along theta.

                Returns
                ----------
                list
                    The object list, list of dicts
        """
        obj_list = []
        if self.obstacle_arr is not None:
            for obs in self.obstacle_arr:
                # Get cartesian coordinates of middle of object
                s_middle = obs[6]
                d_middle = obs[7]
                obs_in_map = self.frenettoglob(s_middle, d_middle)

                # Calculate direction vector of the rectangular object (the two points are the two left vertices of the rectangle)
                p1 = self.frenettoglob(obs[1], obs[3])
                p2 = self.frenettoglob(obs[2], obs[3])
                dir_vector = np.array([p2[0] - p1[0], p2[1] - p1[1]])
                # Calculate theta
                theta = self.angle_between(dir_vector, np.array([0, 1]))  # Zero is north
                ide = obs[0]
                type = 'physical'
                X = obs_in_map[0]
                Y = obs_in_map[1]
                length = obs[8]
                width = length
                # Convert velocity vector from cartesian frame to Frenet frame
                velocity_vector_in_cartesian = self.fromfrenet_to_cartesian(obs[9], obs[10], obs[6])
                velocity_magnitude = np.linalg.norm(velocity_vector_in_cartesian)
                
                #velocity_projected_along_north = np.dot(velocity_vector_in_cartesian, np.array([0, -1])) \
                                       #/ np.linalg.norm(np.array([0, -1]))
                v = velocity_magnitude if not obs[11] else 0.0
                
                # Fill object dict
                obj = {'id': ide, 'type': type, 'X': X, 'Y': Y, 'theta': theta, 'v': v, 'length': length,
                       'width': width}
                obj_list.append(obj)
        return obj_list

    def fromfrenet_to_cartesian(self, velocity_s, velocity_d, object_s):
        """
                Function that transforms a velocity vector in cartesian frame to a velocity vector in Frenet frame.

                Parameters
                ----------
                velocity_s
                    S component of velocity vector to be transformed
                velocity_d
                    D component of velocity vector to be transformed
                object_s
                    S coordinate of obstacle

                Returns
                ----------
                numpy array
                    Velocity vector in cartesian frame

        """
        # Get closest waypoint's psi
        closest_wpnt_idx = self.find_global_index(object_s)
        delta_psi = self.waypoints_original[closest_wpnt_idx, 6]
        # Rotate vector using psi
        velocity_x = velocity_s * np.cos(delta_psi) + velocity_d * np.sin(delta_psi)
        velocity_y = velocity_s * - np.sin(delta_psi) + velocity_d * np.sin(delta_psi)
        return np.asarray([velocity_x, velocity_y])

    def visualize_action(self, action: str):
        """
                Function that visualizes the state of the car by displaying a colored cube in RVIZ.

                Parameters
                ----------
                action
                    Current state of the car to be displayed
        """
        mrk = Marker()
        mrk.type = mrk.CUBE
        mrk.id = 1
        mrk.header.frame_id = 'map'
        mrk.header.stamp = rospy.Time.now()
        mrk.color.a = 1.0
        mrk.color.g = 1.0
        mrk.pose.position.x = 0
        mrk.pose.position.y = -3  #TODO: set position in a smart way
        mrk.pose.position.z = 0
        mrk.scale.x = 2
        mrk.scale.y = 2
        mrk.scale.z = 2

        # Set color and log info based on the state of the car
        var = str()
        if action == 'straight':
            var = 'following the global tracjectory'
            mrk.color.r = 20 / 255
            mrk.color.g = 240 / 255    # green
            mrk.color.b = 40 / 255
        elif action == 'right':
            var = 'overtaking right'
            mrk.color.r = 20 / 255
            mrk.color.g = 23 / 255     # blue
            mrk.color.b = 230 / 255
        elif action == 'left':
            var = 'overtaking left'
            mrk.color.r = 230 / 255
            mrk.color.g = 230 / 255    # yellow
            mrk.color.b = 10 / 255
        elif action == 'follow':
            var = 'following'
            mrk.color.r = 230 / 255
            mrk.color.g = 220 / 255     # gray
            mrk.color.b = 230 / 255

        rospy.loginfo("[Graph Based Planner] The car is currently " + var)
        self.action_mrk.publish(mrk)

    def send_path_topic(self, traj_set, sel_action):
        """Function that sends the selected path to the path publisher node. If the car is in following mode
        calculate a custom path first

        Parameters
        ----------
        traj_set
            Dict holding paths for each available action
        sel_action
            Specific state the car is in for this iteration
        """
        if sel_action != None:
            mypath = np.asarray(traj_set[sel_action][0])
            # Send path to the other node
            msg = Float32MultiArray()
            msg.data = mypath.flatten()
            msg.layout.dim = [MultiArrayDimension(), MultiArrayDimension()]
            msg.layout.data_offset = 0
            msg.layout.dim[0].label = sel_action
            msg.layout.dim[0].size = mypath.shape[0]
            msg.layout.dim[0].stride = len(mypath.flatten())
            msg.layout.dim[1].label = "column"
            msg.layout.dim[1].size = mypath.shape[1]
            msg.layout.dim[1].stride = mypath.shape[1]
            self.path_pub.publish(msg)
        else:
            rospy.logwarn('[Graph Based Planner]: No action selected')

    def traj_cost(self, path) -> float:
        """Function that calculates a cost to a given path

        Parameters
        ----------
        path
            Path to be evaluated

        Returns
        ----------
        float
            Cost of path
        """
        cost = 0.0
        i = 0.0
        for point in path:
            # Calculate point distance to left and right track bound
            frenet_point = self.globtofrenet(point[1], point[2])
            d_point = frenet_point[1]
            close_s_idx = self.find_global_index(point[0])
            wpnt_d_left = self.waypoints_original[close_s_idx, 5]
            wpnt_d_right = self.waypoints_original[close_s_idx, 4]
            point_d_left = wpnt_d_left - d_point
            point_d_right = wpnt_d_right + d_point

            # Select min track bound distance and assign cost
            min_track_distance = np.abs(min(point_d_right, point_d_left))
            cost += (1/(1 + i)) * (1 / min_track_distance)
            i += 0.5
        return cost

    def globtofrenet(self, x, y):
        """Function that transforms cartesian coordiantes into frenet coordinates (helps with compute as compared
        to ros service)

        Parameters
        ----------
        X
            Cartesian coordinate
        Y
            Cartesian coordinate

        Returns
        ----------
        numpy array
            [s, d] array in Frenet frame
        """
        point = np.tile([x, y], (len(self.waypoints_original[:, 2:4]), 1))
        closest_pt_idx = np.argmin(np.linalg.norm(self.waypoints_original[:, 2:4] - point, axis=1))
        s = self.waypoints_original[closest_pt_idx, 0]
        d_x = x - self.waypoints_original[closest_pt_idx, 2]
        d_y = y - self.waypoints_original[closest_pt_idx, 3]
        d =  -d_x * sin(self.waypoints_original[closest_pt_idx, 6]) +\
              d_y * cos(self.waypoints_original[closest_pt_idx, 6])
        return np.asarray([s, d])

    def frenettoglob(self, s, d):
        """Function that transforms frenet coordinates into cartesian coordinates (helps with compute as compared
        to ros service)

        Parameters
        ----------
        S
            Frenet coordinate
        D
            Frenet coordinate

        Returns
        ----------
        numpy array
            [x, y] array in cartesian frame
        """
        closest_pt_idx = self.find_global_index(s)
        d_s = s - self.waypoints_original[closest_pt_idx, 0]
        x = self.waypoints_original[closest_pt_idx, 2] + (d_s * cos(self.waypoints_original[closest_pt_idx, 6])) \
            -d * sin(self.waypoints_original[closest_pt_idx, 6])
        y = self.waypoints_original[closest_pt_idx, 3] + (d * cos(self.waypoints_original[closest_pt_idx, 6])) +\
            (d_s * sin(self.waypoints_original[closest_pt_idx, 6]))
        return np.asarray([x, y])

    def choose_action(self, traj_set) -> str:
        """Function which is tasked with picking an action from the available action set

        Parameters
        ----------
        traj_set
            Trajectory set holding all the available action alongside the corresponding paths

        Returns
        ----------
        string
            Name of the chosen action
        """
        #If both left and right overtake is possible, compare their costs and choose the best
        if "right" in traj_set.keys() and "left" in traj_set.keys():
            path_r = traj_set['right'][0]
            path_l = traj_set['left'][0]
            # Cut points from the path which are behind the position of the car and points in which the velocity is 0
            if not self.close_to_s0:
                sliced_path_r = path_r[np.where(((self.current_position_frenet[0] <
                                                  (path_r[:, 0] + self.current_position_frenet[0]) % self.max_s) &
                                                 (path_r[:, 5] > 0)))]

                sliced_path_l = path_l[np.where(((self.current_position_frenet[0] <
                                                  (path_l[:, 0] + self.current_position_frenet[0]) % self.max_s) &
                                                 (path_l[:, 5] > 0)))]
            else:
                sliced_path_r = path_r[np.where(((self.current_position_frenet[0] <
                                                  (path_r[:, 0] + self.current_position_frenet[0]) % self.max_s) & (
                                                         path_r[:, 5] > 0)
                                                 | ((path_r[:, 0] + self.current_position_frenet[0]) > self.max_s)))]
                sliced_path_l = path_l[np.where(((self.current_position_frenet[0] <
                                                  (path_l[:, 0] + self.current_position_frenet[0]) % self.max_s) & (
                                                         path_l[:, 5] > 0)
                                                 | ((path_l[:, 0] + self.current_position_frenet[0]) > self.max_s)))]
            # Calculate costs for paths
            cost_r = self.traj_cost(sliced_path_r[:self.examined_points, :])
            cost_l = self.traj_cost(sliced_path_l[:self.examined_points, :])

            # Choose
            act = 'right' if cost_r < cost_l else 'left'

            return act
        # If left and right arent available together, prioritize going straight, then overtaking, then following
        else:
            for act in ["straight", "right", "left", "follow"]:
                if act in traj_set.keys():
                    return act

    #############
    # MAIN LOOP #
    #############
    def planner_loop(self):
        """
                Main online loop
        """
        ### INITIALIZATION ###

        # Wait for global waypoints
        rospy.loginfo('[Graph Based Planner] Waiting for Global Waypoints...')
        rospy.wait_for_message('/global_waypoints', WpntArray)
        rospy.loginfo('[Graph Based Planner] Global Waypoints received!')
        self.max_s = np.amax(self.waypoints_original[:, 0])
        # Wait for Odometry
        rospy.loginfo('[Graph Based Planner] Waiting for Odometry...')
        rospy.wait_for_message('/car_state/odom', Odometry)
        rospy.loginfo('[Graph Based Planner] Odometry received!')
        # Wait for speed scaling
        rospy.loginfo('[Graph Based Planner] Waiting for Dynamic parameters...')
        rospy.wait_for_message('dyn_sector_server/parameter_updates', Config)
        rospy.loginfo('[Graph Based Planner] Dynamic parameters received!')

        # Check existence/create trajectory file
        if not self.create_map_csv():
            rospy.loginfo('[Graph Based Planner] Found an already existing trajectory file for this map!')
        else:
            rospy.loginfo('[Graph Based Planner] Created a new trajectory file for this map!')

        # Intialize graph_ltpl-class
        ltpl_obj = GraphBasedPlanner.graph_ltpl.Graph_LTPL.Graph_LTPL(path_dict=self.path_dict,
                                                                      visual_mode=False,
                                                                      log_to_file=False)
        # Calculate offline graph
        ltpl_obj.graph_init(veh_param_mass=3.5)
        # Visualize graph
        if self.display_graph:
            graph_container = ltpl_obj._Graph_LTPL__graph_base
            self.visualize_lattice(graph_container)

        # Set start pos
        ltpl_obj.set_startpos(pos_est=self.current_position,
                              heading_est=self.current_heading + np.pi / 2)  # Zero heading is north, self.current_heading's zero is east
        #Initializations
        traj_set = {'straight': None}
        prev_action = 'straight'
        sel_action = 'straight'
        self.visualize_action('straight')
        main_rate=rospy.Rate(30)

        if self.profile:
            self.profiler.enable() # it is disabled on shutdown
         
        while not rospy.is_shutdown():
            start_time = time.perf_counter()
            
            # Handle close to s0 flag
            self.close_to_s0 = self.current_position_frenet[0] < 0.5 or self.current_position_frenet[0] > self.max_s - 0.5
            
            # -- SELECT ONE OF THE PROVIDED TRAJECTORIES --------------------------------------------------------------
            prev_action = sel_action
            
            # Get object list
            obj_list = self.get_obj_list()
            
            # Calculate paths
            ltpl_obj.calc_paths(prev_action_id=sel_action,
                                object_list=obj_list
                                )

            # Calculate velocity profile and retrieve trajectories
            traj_set = ltpl_obj.calc_vel_profile(pos_est=self.current_position,
                                                 vel_est=self.current_speed,
                                                 vel_max=self.v_max,
                                                 ax_max_machines=self.ax_max,
                                                 safety_d=self.trailing_distance
                                                 )[0]

            # Select a trajectory and send it to the controller
            sel_action = self.choose_action(traj_set)

            # Send latency
            if self.measuring:
                latency = time.perf_counter() - start_time
                self.latency_pub.publish(latency)
            
            # Visualize action
            if sel_action is not prev_action:
                self.visualize_action(sel_action)

            self.send_path_topic(traj_set, sel_action)
            main_rate.sleep()

    def on_shutdown(self):
        rospy.loginfo("[Graph Based Planner] Shutdown")
        if self.profile:
            self.profiler.disable()
            self.profiler.dump_stats(self.profile_filename)
            with open(self.profile_report_filename, "w") as file:
                self.profiler_formatter = pstats.Stats(self.profiler, stream=file)
                self.profiler_formatter.sort_stats('cumtime')
                self.profiler_formatter.print_stats()

if __name__ == '__main__':
    # Initialize node
    rospy.init_node('GraphPlanner', anonymous=False, log_level=rospy.INFO)
    graph = GraphPlanner()
    rospy.on_shutdown(graph.on_shutdown)

    graph.planner_loop()

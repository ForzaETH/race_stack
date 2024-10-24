#!/usr/bin/env python3
import time

import numpy as np
import rospy

from f110_msgs.msg import WpntArray, Wpnt
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from dynamic_reconfigure.msg import Config
from nav_msgs.msg import Odometry
from frenet_conversion.srv import Frenet2Glob, Glob2Frenet


class Path_Publisher:
    def __init__(self):
        self.selected_path = None
        self.sel_action = None
        self.frenet_pose = None
        self.waypoints_original = None # Original set of global waypoints
        self.max_s = None
        self.path_lenght = 80 #Lenght of path to give to controller
        self.loop_rate = 80
        self.close_to_s0 = False

        self.frenet2glob = rospy.ServiceProxy('convert_frenet2glob_service', Frenet2Glob)
        self.glob2frenet = rospy.ServiceProxy('convert_glob2frenet_service', Glob2Frenet)

        rospy.Subscriber('/global_waypoints', WpntArray, self.waypoint_cb)
        rospy.Subscriber('/planner_path', Float32MultiArray, self.planner_path_cb)
        rospy.Subscriber('/car_state/odom_frenet', Odometry, self.odom_fre_cb)
        rospy.Subscriber('/dyn_sector_server/parameter_updates', Config, self.dyn_params_cb)

        self.cmd_pub = rospy.Publisher('/local_waypoints', WpntArray, queue_size=10)
        self.wpnt_viz = rospy.Publisher('/local_waypoints/markers', MarkerArray, queue_size=10)

    def dyn_params_cb(self, data: Config):
        self.speed_scaling = data.doubles[0].value

    def waypoint_cb(self, data: WpntArray):
        self.waypoints_original = np.array([
            [w.s_m, w.d_m, w.x_m, w.y_m, w.d_right, w.d_left, w.psi_rad,
             w.kappa_radpm, w.vx_mps, w.ax_mps2] for w in data.wpnts
        ])

    def odom_fre_cb(self, data: Odometry):
        s = data.pose.pose.position.x
        d = data.pose.pose.position.y
        self.frenet_pose = np.array([s, d])

    def planner_path_cb(self, data):
        arr = np.asarray(data.data)
        self.selected_path = arr.reshape(data.layout.dim[0].size, data.layout.dim[1].size)
        self.sel_action = data.layout.dim[0].label

    def find_nearest_s(self, station: float, array) -> int:
        return np.abs(array - station).argmin()

    def send_path(self, path, sel_action):
        if self.selected_path is not None:
            waypoint_arr = WpntArray()
            marker_arr = MarkerArray()
            waypoint_arr.header.stamp = rospy.Time.now()
            waypoint_arr.header.frame_id = "map"

            self.close_to_s0 =  self.frenet_pose[0] < 0.5 or self.frenet_pose[0] > self.max_s - 0.5

            # get slice of waypoints
            print(f"What kind of path do we get? :{path.shape}")
            sliced_path = path
            # if not self.close_to_s0:
            #     if sel_action != 'follow':
            #         sliced_path = path[np.where(((self.frenet_pose[0] < (path[:, 0] + self.frenet_pose[0]) % self.max_s) & (path[:, 5] > 0)))]
            #     else:
            #         sliced_path = path[np.where((self.frenet_pose[0] < (path[:, 0] + self.frenet_pose[0]) % self.max_s))]
            # else:
            #     if sel_action != 'follow':
            #         sliced_path = path[np.where(((self.frenet_pose[0] <
            #                                 (path[:, 0] + self.frenet_pose[0]) % self.max_s) & (path[:, 5]>0)
            #                                 | ((path[:, 0] + self.frenet_pose[0]) > self.max_s)))]
            #     else:
            #         sliced_path = path[np.where((self.frenet_pose[0] <
            #                             (path[:, 0] + self.frenet_pose[0])% self.max_s) | ((path[:, 0]
            #                             +self.frenet_pose[0]) > self.max_s))]

            # Fill waypoint and marker array
            for i, coord in enumerate(sliced_path[:self.path_lenght, :]):
                wpnt = Wpnt()
                wpnt.s_m = (coord[0] + self.frenet_pose[0]) % self.max_s
                wpnt.x_m = coord[1]
                wpnt.y_m = coord[2]
                #point = self.glob2frenet(wpnt.x_m, wpnt.y_m)
                wpnt.d_m = 0.0
                wpnt.psi_rad = coord[3]
                wpnt.kappa_radpm = coord[4]
                wpnt.vx_mps = coord[5] * self.speed_scaling
                wpnt.ax_mps2 = coord[6]
                wpnt.id = i
                # Get index of closest global waypoint in terms of s-coordinate
                idx = self.find_nearest_s(wpnt.s_m, self.waypoints_original[:, 0])
                # Get left and right distances to track bounds from the global waypoint
                d_left = self.waypoints_original[idx, 5]
                d_right = self.waypoints_original[idx, 4]
                # Use this information together with the d coordinate of the local waypoint in order to calculate
                # left and right track bounds distances of the local waypoint
                wpnt.d_left = d_left - wpnt.d_m
                wpnt.d_right = d_right + wpnt.d_m
                waypoint_arr.wpnts.append(wpnt)

                mrk = Marker()
                mrk.header.frame_id = 'map'
                mrk.header.stamp = rospy.Time.now()
                mrk.type = mrk.SPHERE
                mrk.scale.x = 0.15
                mrk.scale.y = 0.15
                mrk.scale.z = 0.15
                mrk.color.a = 1.0
                mrk.color.g = 1.0
                mrk.id = i
                mrk.pose.position.x = wpnt.x_m
                mrk.pose.position.y = wpnt.y_m
                mrk.pose.position.z = 0.0
                mrk.pose.orientation.w = 1.0
                marker_arr.markers.append(mrk)

            # Send local waypoints to the controller and markers to the visualization topic
            self.cmd_pub.publish(waypoint_arr)
            self.wpnt_viz.publish(marker_arr)

    def pub_loop(self):
        rospy.wait_for_message('/global_waypoints', WpntArray)
        self.max_s = np.amax(self.waypoints_original[:, 0])
        rate = rospy.Rate(self.loop_rate)
        while not rospy.is_shutdown():
            self.send_path(self.selected_path, self.sel_action)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('PathPublisher', anonymous=True, log_level=rospy.INFO)
    pub = Path_Publisher()
    pub.pub_loop()


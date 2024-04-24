#!/usr/bin/env python3

import rospy
from f110_msgs.msg import LapData, WpntArray
from std_msgs.msg import Float32, Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker

from collections import deque
import numpy as np

import subprocess # save map
from rospkg import RosPack
from datetime import datetime
import os

class LapAnalyser:
    def __init__(self):

        # Wait for state machine to start to figure out where to place the visualization message
        self.vis_pos = Pose
        msg: Marker = rospy.wait_for_message("/state_marker", Marker, timeout=None)
        if msg is not None:
            self.vis_pos = msg.pose
        self.vis_pos.position.z += 1.5  # appear on top of the state marker
        rospy.loginfo(f"LapAnalyser will be centered at {self.vis_pos.position.x}, {self.vis_pos.position.y}, {self.vis_pos.position.z}")

        # stuff for min distance to track boundary
        self.wp_flag = False
        self.car_distance_to_boundary = []
        self.global_lateral_waypoints = None
        rospy.loginfo("[LapAnalyser] Waiting for /global_waypoints topic")
        rospy.wait_for_message('/global_waypoints', WpntArray)
        rospy.loginfo("[LapAnalyser] Ready to go")
        rospy.Subscriber('/global_waypoints', WpntArray, self.waypoints_cb) # TODO maybe add wait for topic/timeout?

        while self.global_lateral_waypoints is None:
            print("[Lap Analyzer] Waiting for global lateral waypoints")
            rospy.sleep(0.1)

        rospy.Subscriber('/car_state/odom_frenet', Odometry, self.frenet_odom_cb)  # car odom in frenet frame

        rospy.Subscriber('/lap_analyser/start', Empty, self.start_log_cb)

        # publishes once when a lap is completed
        self.lap_data_pub = rospy.Publisher('lap_data', LapData, queue_size=10)
        self.min_car_distance_to_boundary_pub = rospy.Publisher('min_car_distance_to_boundary', Float32, queue_size=10) # publishes every time a new car position is received
        self.lap_start_time = rospy.Time.now()
        self.last_s = 0
        self.accumulated_error = 0
        self.max_error = 0
        self.n_datapoints = 0
        self.lap_count = -1

        self.NUM_LAPS_ANALYSED = 10
        '''The number of laps to analyse and compute statistics for'''
        self.lap_time_acc = deque(maxlen=self.NUM_LAPS_ANALYSED)
        self.lat_err_acc = deque(maxlen=self.NUM_LAPS_ANALYSED)
        self.max_lat_err_acc = deque(maxlen=self.NUM_LAPS_ANALYSED)

        self.LOC_METHOD = rospy.get_param("~loc_algo", default="slam")

        # Publish stuff to RViz
        self.lap_data_vis = rospy.Publisher('lap_data_vis', Marker, queue_size=5)

        # Open up logfile
        self.logfile_name = f"lap_analyzer_{datetime.now().strftime('%d%m_%H%M')}.txt"
        self.logfile_par = os.path.join(RosPack().get_path('lap_analyser'), 'data')
        self.logfile_dir = os.path.join(self.logfile_par, self.logfile_name)
        if not os.path.exists(self.logfile_par):
            os.makedirs(self.logfile_par)
        with open(self.logfile_dir, 'w') as f:
            f.write(f"Laps done on " + datetime.now().strftime('%d %b %H:%M:%S') + '\n')


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
            self.global_lateral_waypoints = np.array([
                [w.s_m, w.d_right, w.d_left] for w in data.wpnts
            ]) 
            self.wp_flag = True
        else:
            pass

    def frenet_odom_cb(self, msg):
        if not self.wp_flag:
            return

        current_s = msg.pose.pose.position.x
        current_d = msg.pose.pose.position.y
        if self.check_for_finish_line_pass(current_s):
            if (self.lap_count == -1):
                self.lap_start_time = rospy.Time.now()
                rospy.loginfo("LapAnalyser: started first lap")
                self.lap_count = 0
            else:
                self.lap_count += 1
                self.publish_lap_info()
                self.publish_min_distance()
                self.car_distance_to_boundary = []
                self.lap_start_time = rospy.Time.now()
                self.max_error = abs(current_d)
                self.accumulated_error = abs(current_d)
                self.n_datapoints = 1

                # Compute and publish statistics. Perhaps publish to a file?
                if self.lap_count > 0 and self.lap_count % self.NUM_LAPS_ANALYSED == 0:
                    lap_time_str = f"Lap time over the past {self.NUM_LAPS_ANALYSED} laps: Mean: {np.mean(self.lap_time_acc):.4f}, Std: {np.std(self.lap_time_acc):.4f}"
                    avg_err_str = f"Avg Lat Error over the past {self.NUM_LAPS_ANALYSED} laps: Mean: {np.mean(self.lat_err_acc):.4f}, Std: {np.std(self.lat_err_acc):.4f}"
                    max_err_str = f"Max Lat Error over the past {self.NUM_LAPS_ANALYSED} laps: Mean: {np.mean(self.max_lat_err_acc):.4f}, Std: {np.std(self.max_lat_err_acc):.4f}"
                    rospy.logwarn(lap_time_str)
                    rospy.logwarn(avg_err_str)
                    rospy.logwarn(max_err_str)

                    with open(self.logfile_dir, 'a') as f:
                        f.write(lap_time_str+'\n')
                        f.write(avg_err_str+'\n')
                        f.write(max_err_str+'\n')

                    # // This was to check for map shift during SE/Loc/Sensor experiments. Not needed during the race.
                    # if self.LOC_METHOD == "slam":
                    #     # Create map folder
                    #     self.map_name = f"map_{datetime.now().strftime('%d%m_%H%M')}"
                    #     self.map_dir = os.path.join(RosPack().get_path('lap_analyser'), 'maps', self.map_name)
                    #     os.makedirs(self.map_dir)
                    #     rospy.loginfo(f"LapAnalyser: Saving map to {self.map_dir}")

                    #     subprocess.run(f"rosrun map_server map_saver -f {self.map_name} map:=/map_new", cwd=f"{self.map_dir}", shell=True)
        else:
            self.accumulated_error += abs(current_d)
            self.n_datapoints += 1
            if self.max_error < abs(current_d):
                self.max_error = abs(current_d)
        self.last_s = current_s

        # search for closest s value: s values of global waypoints do not match the s values of car position exactly
        s_ref_line_values = np.array(self.global_lateral_waypoints)[:, 0]
        index_of_interest = np.argmin(np.abs(s_ref_line_values - current_s)) # index where s car state value is closest to s ref line

        d_right = self.global_lateral_waypoints[index_of_interest, 1] # [w.s_m, w.d_right, w.d_left] 
        d_left = self.global_lateral_waypoints[index_of_interest, 2]

        dist_to_bound = self.get_distance_to_boundary(current_d, d_left, d_right)
        self.car_distance_to_boundary.append(dist_to_bound)  

    def start_log_cb(self, _):
        '''Start logging. Reset all metrics.'''
        rospy.loginfo(
            f"LapAnalyser: Start logging statistics for {self.NUM_LAPS_ANALYSED} laps.")
        self.accumulated_error = 0
        self.max_error = 0
        self.lap_count = -1
        self.n_datapoints = 0

    def check_for_finish_line_pass(self, current_s):
        # detect wrapping of the track, should happen exactly once per round
        if (self.last_s - current_s) > 1.0:
            return True
        else:
            return False

        # ? Future extension: would be cool to check for sector times...

    def publish_lap_info(self):
        msg = LapData()
        msg.lap_time = (rospy.Time.now() - self.lap_start_time).to_sec()
        rospy.loginfo(
            f"LapAnalyser: completed lap #{self.lap_count} in {msg.lap_time}")

        with open(self.logfile_dir, 'a') as f:
            f.write(f"Lap #{self.lap_count}: {msg.lap_time:.4f}" + '\n')

        msg.header.stamp = rospy.Time.now()
        msg.lap_count = self.lap_count
        msg.average_lateral_error_to_global_waypoints = self.accumulated_error / self.n_datapoints
        msg.max_lateral_error_to_global_waypoints = self.max_error
        self.lap_data_pub.publish(msg)

        # append to deques for statistics
        self.lap_time_acc.append(msg.lap_time)
        self.lat_err_acc.append(msg.average_lateral_error_to_global_waypoints)
        self.max_lat_err_acc.append(msg.max_lateral_error_to_global_waypoints)

        mark = Marker()
        mark.header.stamp = rospy.Time.now()
        mark.header.frame_id = 'map'
        mark.id = 0
        mark.ns = 'lap_info'
        mark.type = Marker.TEXT_VIEW_FACING
        mark.action = Marker.ADD
        mark.pose = self.vis_pos
        mark.scale.x = 0.0
        mark.scale.y = 0.0
        mark.scale.z = 0.5  # Upper case A
        mark.color.a = 1.0
        mark.color.r = 0.2
        mark.color.g = 0.2
        mark.color.b = 0.2
        mark.text = f"Lap {self.lap_count:02d} {msg.lap_time:.3f}s"
        self.lap_data_vis.publish(mark)

    def publish_min_distance(self):
        self.min_car_distance_to_boundary = np.min(self.car_distance_to_boundary)
        self.min_car_distance_to_boundary_pub.publish(self.min_car_distance_to_boundary)

    def get_distance_to_boundary(self, current_d, d_left, d_right):
        """
        comment this function    
        ----------
        Input: 
            current_d: lateral distance to reference line
            d_left: distance from ref. line to left track boundary
            d_right: distance from ref. line to right track boundary
        Output:
            distance: critical distance to track boundary (whichever is smaller, to the right or left)  
        """
        # calculate distance from car to boundary
        car_dist_to_bound_left = d_left - current_d                 
        car_dist_to_bound_right = d_right + current_d  

        # select whichever distance is smaller (to the right or left)
        if car_dist_to_bound_left > car_dist_to_bound_right: # car is closer to right boundary
            return car_dist_to_bound_right
        else:
            return car_dist_to_bound_left


if __name__ == '__main__':
    rospy.init_node('lap_analyser')
    analyser = LapAnalyser()
    while not rospy.is_shutdown():
        rospy.spin()

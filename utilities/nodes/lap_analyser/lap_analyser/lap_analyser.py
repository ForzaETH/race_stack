#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from f110_msgs.msg import LapData, WpntArray
from std_msgs.msg import Float32, Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker

import time
from ament_index_python.packages import get_package_share_directory
from collections import deque
import numpy as np

import subprocess # save map
from datetime import datetime
import os

class LapAnalyser(Node):
    def __init__(self):
        super().__init__('lap_analyser',
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)

        self.get_logger().info("Lap_analyser node started")

        # Wait for state machine to start to figure out where to place the visualization message
        self.vis_pos = Pose()
        self.state_marker = None
        # msg: Marker = rospy.wait_for_message("/state_marker", Marker, timeout=None)
        self.marker_sub = self.create_subscription(Marker, '/state_marker', self.marker_cb, 10)

        # while self.state_marker is None:
        #     print("[Lap Analyzer] Waiting for state marker")
        #     time.sleep(0.1)

        if self.state_marker is not None:
            self.vis_pos = self.state_marker.pose

        self.vis_pos.position.z += 1.5  # appear on top of the state marker
        self.get_logger().info(f"LapAnalyser will be centered at {self.vis_pos.position.x}, {self.vis_pos.position.y}, {self.vis_pos.position.z}")

        # stuff for min distance to track boundary
        self.wp_flag = False
        self.car_distance_to_boundary = []
        self.global_lateral_waypoints = None
        # self.get_logger().info("[LapAnalyser] Waiting for /global_waypoints topic")
        # rospy.wait_for_message('/global_waypoints', WpntArray)
        # self.get_logger().info("[LapAnalyser] Ready to go")
        self.gb_wpnts_sub = self.create_subscription(WpntArray, "/global_waypoints", self.waypoints_cb, 10) # TODO maybe add wait for topic/timeout?

        # while self.global_lateral_waypoints is None:
        #     print("[Lap Analyzer] Waiting for global lateral waypoints")
        #     time.sleep(0.1)
        self.odom_frenet_sub = self.create_subscription(Odometry, '/car_state/frenet/odom', self.frenet_odom_cb, 10) # car odom in frenet frame

        self.lap_analy_sub = self.create_subscription(Empty, '/lap_analyser/start', self.start_log_cb, 10)

        # publishes once when a lap is completed
        self.lap_data_pub = self.create_publisher(LapData, 'lap_data', 10)
        self.min_car_distance_to_boundary_pub = self.create_publisher(Float32, 'min_car_distance_to_boundary', 10) # publishes every time a new car position is received
        self.lap_start_time = self.get_clock().now()
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

        # self.LOC_METHOD = rospy.get_param("~loc_algo", default="slam")
        # self.LOC_METHOD = self.get_parameter("~loc_algo").value

        # Publish stuff to RViz
        self.lap_data_vis = self.create_publisher(Marker, 'lap_data_vis', 10)

        # Open up logfile
        package_path = get_package_share_directory('lap_analyser')
        ws_path = os.path.abspath(os.path.join(package_path, '..', '..', '..', '..'))
        data_path = os.path.join(ws_path, 'data/lap_analyser')
        self.get_logger().warn(data_path)
        if not os.path.exists(data_path):
            os.makedirs(data_path)
        
        self.logfile_name = f"lap_analyzer_{datetime.now().strftime('%d%m_%H%M')}.txt"
        self.logfile_dir = os.path.join(data_path, self.logfile_name)
        with open(self.logfile_dir, 'w') as f:
            f.write(f"Laps done on " + datetime.now().strftime('%d %b %H:%M:%S') + '\n')

    def marker_cb(self, data: Marker):
            self.state_marker = data



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
            self.get_logger().warn("frenet cb waiting for gb wpnts...", throttle_duration_sec=0.5)
            return

        current_s = msg.pose.pose.position.x
        current_d = msg.pose.pose.position.y
        if self.check_for_finish_line_pass(current_s):
            if (self.lap_count == -1):
                self.lap_start_time = self.get_clock().now()
                self.get_logger().info("LapAnalyser: started first lap")
                self.lap_count = 0
            else:
                self.lap_count += 1
                self.publish_lap_info()
                # self.publish_min_distance()
                self.car_distance_to_boundary = []
                self.lap_start_time = self.get_clock().now()
                self.max_error = abs(current_d)
                self.accumulated_error = abs(current_d)
                self.n_datapoints = 1

                # Compute and publish statistics. Perhaps publish to a file?
                if self.lap_count > 0 and self.lap_count % self.NUM_LAPS_ANALYSED == 0:
                    lap_time_str = f"Lap time over the past {self.NUM_LAPS_ANALYSED} laps: Mean: {np.mean(self.lap_time_acc):.4f}, Std: {np.std(self.lap_time_acc):.4f}"
                    avg_err_str = f"Avg Lat Error over the past {self.NUM_LAPS_ANALYSED} laps: Mean: {np.mean(self.lat_err_acc):.4f}, Std: {np.std(self.lat_err_acc):.4f}"
                    max_err_str = f"Max Lat Error over the past {self.NUM_LAPS_ANALYSED} laps: Mean: {np.mean(self.max_lat_err_acc):.4f}, Std: {np.std(self.max_lat_err_acc):.4f}"
                    self.get_logger().warn(lap_time_str)
                    self.get_logger().warn(avg_err_str)
                    self.get_logger().warn(max_err_str)

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
        self.get_logger().info(
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
        # msg.lap_time = (rclpy.time.Time() - self.lap_start_time).to_sec()
        lap_time = self.get_clock().now() - self.lap_start_time
        msg.lap_time = lap_time.nanoseconds / 1e9
        self.get_logger().info(
            f"LapAnalyser: completed lap #{self.lap_count} in {msg.lap_time}")

        with open(self.logfile_dir, 'a') as f:
            f.write(f"Lap #{self.lap_count}: {msg.lap_time:.4f}" + '\n')

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.lap_count = self.lap_count
        msg.average_lateral_error_to_global_waypoints = self.accumulated_error / self.n_datapoints
        msg.max_lateral_error_to_global_waypoints = self.max_error
        self.lap_data_pub.publish(msg)

        # append to deques for statistics
        self.lap_time_acc.append(msg.lap_time)
        self.lat_err_acc.append(msg.average_lateral_error_to_global_waypoints)
        self.max_lat_err_acc.append(msg.max_lateral_error_to_global_waypoints)

        mark = Marker()
        mark.header.stamp = self.get_clock().now().to_msg()
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

def main():
    rclpy.init()
    lap_analyser = LapAnalyser()
    rclpy.spin(lap_analyser)
    lap_analyser.destroy_node()
    rclpy.shutdown()

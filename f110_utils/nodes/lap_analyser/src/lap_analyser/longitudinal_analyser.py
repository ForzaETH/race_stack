#!/usr/bin/env python3

import rospy
from f110_msgs.msg import LapData, PidData
from nav_msgs.msg import Odometry

class LongAnalyser:
    def __init__(self):
        rospy.Subscriber('/car_state/odom_frenet', Odometry, self.frenet_odom_cb) # car odom in frenet frame
        rospy.Subscriber('/velocity_pid', PidData, self.velocity_cb)
        rospy.Subscriber('/trailing/gap_data', PidData, self.trailing_cb)

        
        self.trailing_data_pub = rospy.Publisher('/tlap_data/trailing', LapData, queue_size=10) # publishes once when a lap is completed
        self.vel_data_pub = rospy.Publisher('/lap_data/vel_ctr', LapData, queue_size=10) # publishes once when a lap is completed

        self.lap_start_time = rospy.Time.now()
        self.last_s = 0

        self.accumulated_vel_error = 0
        self.max_vel_error = 0
        self.vel_datapoints = 0

        self.accumulated_trailing_error = 0
        self.trailing_datapoints = 0
        self.max_trailing_error = 0

        self.lap_count = -1
    

    def frenet_odom_cb(self, msg):
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
                self.lap_start_time = rospy.Time.now()
        self.last_s = current_s

    def velocity_cb(self, data: PidData):
        error = abs(data.error)
        if error > self.max_vel_error:
            self.max_vel_error = error
        self.vel_datapoints += 1
        self.accumulated_vel_error += error

    def trailing_cb(self, data: PidData):
        error = abs(data.error)
        if error > self.max_trailing_error:
            self.max_trailing_error = error
        self.trailing_datapoints += 1
        self.accumulated_trailing_error += error

    def check_for_finish_line_pass(self, current_s):
        # detect wrapping of the track, should happen exactly once per round
        if (self.last_s - current_s) > 1.0:
            return True
        else:
            return False

    def publish_lap_info(self):
        lap_time = (rospy.Time.now() - self.lap_start_time).to_sec()
        rospy.loginfo(f"LapAnalyser: completed lap #{self.lap_count} in {lap_time}")
        
        if self.vel_datapoints != 0:
            average_vel_error = self.accumulated_vel_error / self.vel_datapoints
            print("Average Velocity Error: ", average_vel_error)
            print("Max Velocity Error: ", self.max_vel_error)
            
            vel_msg = LapData()
            vel_msg.header.stamp = rospy.Time.now()
            vel_msg.lap_count = self.lap_count
            vel_msg.lap_time = lap_time
            vel_msg.average_lateral_error_to_global_waypoints = average_vel_error
            vel_msg.max_lateral_error_to_global_waypoints = self.max_vel_error
            self.vel_data_pub.publish(vel_msg)

            self.vel_datapoints = 0
            self.max_vel_error = 0
            self.accumulated_vel_error = 0

        if self.trailing_datapoints != 0:
            average_trailing_error = self.accumulated_trailing_error / self.trailing_datapoints
            print("Average Trailing Error: ", average_trailing_error)
            print("Max Trailing Error: ", self.max_trailing_error)
            trailing_msg = LapData()
            trailing_msg.header.stamp = rospy.Time.now()
            trailing_msg.lap_count = self.lap_count
            trailing_msg.lap_time = lap_time
            trailing_msg.average_lateral_error_to_global_waypoints = average_trailing_error
            trailing_msg.max_lateral_error_to_global_waypoints = self.max_trailing_error
            self.trailing_data_pub.publish(trailing_msg)

            self.trailing_datapoints = 0
            self.max_trailing_error = 0
            self.accumulated_trailing_error = 0

if __name__ == '__main__':
  rospy.init_node('longitudinal_analyser')
  analyser = LongAnalyser()
  while not rospy.is_shutdown():
    rospy.spin()
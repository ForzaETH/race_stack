#!/usr/bin/env python3
import importlib
import tf
import rospy
import genpy.message
from rospy import ROSException
import sensor_msgs.msg
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import actionlib
import rostopic
import rosservice
from threading import Thread
from rosservice import ROSServiceException

import numpy as np


class RepubEKFtoOdom():
	def __init__(self):
		rospy.init_node('ekf_repub_node', anonymous=True)
		self.pub_ekf_odom = rospy.Publisher('ekf_odom', Odometry, queue_size=10)
		rospy.Subscriber('/robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, self.ekf_pose_cb)
		
		self.prev_x = 0
		self.prev_y = 0
		self.prev_yaw = 0
		self.prev_ts = rospy.get_rostime()
		
		rospy.spin()
		
	def ekf_pose_cb(self, data):
		pose = data.pose.pose
		ts = data.header.stamp
		ori_arr = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
		euler = tf.transformations.euler_from_quaternion(ori_arr)
		yaw = euler[2]
		
		dx   = pose.position.x   - self.prev_x
		dy   = pose.position.y   - self.prev_y
		dyaw = yaw - self.prev_yaw
		dt   = ts                - self.prev_ts
		dt = dt.to_sec()
		
		odom = Odometry()
		odom.header = data.header
		odom.child_frame_id = 'base_link'
		odom.pose.pose = pose
		odom.pose.covariance[0]  = 0.2 # x, covariances here are likely widely off
		odom.pose.covariance[7]  = 0.2 # y
		odom.pose.covariance[35] = 0.4 # yaw
		odom.twist.twist.linear.x    = dx/dt    
		odom.twist.twist.linear.y    = dy/dt    
		odom.twist.twist.angular.z = dyaw/dt
		
		self.prev_x = pose.position.x
		self.prev_y = pose.position.y
		self.prev_yaw = yaw
		self.prev_ts = ts
		
		self.pub_ekf_odom.publish(odom)
		
			

if __name__ == "__main__":
    rb = RepubEKFtoOdom()

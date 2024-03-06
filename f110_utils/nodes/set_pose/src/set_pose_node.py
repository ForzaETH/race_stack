#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
import subprocess


class SetInitialPose():
    def __init__(self):
        rospy.init_node('setpose', anonymous=True)

        rospy.on_shutdown(self.shutdown)

        # '/home/icra_crew/catkin_ws/src/pbl_f110_system/racecar/racecar/config/common/slam'
        self.CONFIG_DIR = rospy.get_param('set_slam_pose_node/config_dir')
        self.CONFIG_BASE = rospy.get_param('set_slam_pose_node/config_base')  # 'f110_2d_loc.lua'

        # A variable to hold the initial pose of the robot to be set by the user in RViz
        self.initial_pose = PoseWithCovarianceStamped()
        self.ready = False

        # Get the initial pose from the user
        rospy.loginfo("Click the 2D Pose Estimate button in RViz to set the robot's pose...")

        #rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)
        rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.update_initial_pose)

        # 0 is the one we saved during mapping, so in loc only we start trajectory 1
        self.trajectory_num = 1

        while not rospy.is_shutdown():
            rospy.sleep(1)

    def update_initial_pose(self, msg):
        print('initial pos', msg.pose.pose.position)
        print('initial orientation', msg.pose.pose.orientation)
        self.initial_pose = msg
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        xp = msg.pose.pose.position.x
        yp = msg.pose.pose.position.y
        zp = msg.pose.pose.position.z

        s = 'rosservice call /finish_trajectory ' + str(self.trajectory_num)
        print(s)
        process = subprocess.Popen(
            s, stdout=subprocess.PIPE, stderr=None, shell=True)
        output = process.communicate()
        # print(output[0])

        s = 'rosservice call /start_trajectory "{configuration_directory: '
        s = s + '"' + self.CONFIG_DIR + '"'
        s = s + ', configuration_basename: "' + self.CONFIG_BASE + \
            '", use_initial_pose: "true", initial_pose: '
        s = s + ' {position: {x: ' + \
            str(xp) + ', y: ' + str(yp) + ', z: '+str(zp) + '}'
        s = s + ', orientation: {x: ' + str(x) + ', y: ' + str(y) + ', z: '+str(
            z) + ', w: '+str(w)+'}}, relative_to_trajectory_id: 0}"'

        print("Calling:", s)
        process = subprocess.Popen(s, stdout=subprocess.PIPE, stderr=None, shell=True)
        output = process.communicate()

        # print(output[0])
        self.trajectory_num += 1

    def shutdown(self):
        rospy.loginfo("Stopping setinitpose...")


if __name__ == '__main__':
    try:
        SetInitialPose()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Localize finished.")

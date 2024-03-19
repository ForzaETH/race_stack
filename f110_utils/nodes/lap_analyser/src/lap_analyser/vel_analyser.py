#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from f110_msgs.msg import WpntArray
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from ackermann_msgs.msg import AckermannDriveStamped
from dynamic_reconfigure.msg import Config


class VelAnalyser:
    def __init__(self) -> None:
        # attributes
        self.gb_wpnt_sc = None
        self.frenet_odom = None
        self.pub_cur_vel_flag = True
        self.wheelbase = (
            rospy.get_param("/behavior_controller/l_cg2front") 
            + rospy.get_param("/behavior_controller/l_cg2rear")
        )

        # subscribers
        rospy.Subscriber("/global_waypoints_scaled", WpntArray, self.gb_wpnt_sc_cb)
        rospy.Subscriber(
            "/dyn_sector_server/parameter_updates", Config, self.dyn_par_cb
        )
        rospy.Subscriber("/car_state/odom_frenet", Odometry, self.frenet_odom_cb)
        rospy.Subscriber(
            "/vesc/high_level/ackermann_cmd_mux/input/nav_1",
            AckermannDriveStamped,
            self.ctrl_cb,
        )

        # publishers
        self.vel_traj_pub = rospy.Publisher(
            "velocity/trajectory", Point, queue_size=1000
        )
        self.steer_traj_pub = rospy.Publisher(
            "steering/trajectory", Point, queue_size=1000
        )
        self.vel_cur_pub = rospy.Publisher("velocity/current", Point, queue_size=10)
        self.steer_cur_pub = rospy.Publisher("steering_input", Point, queue_size=10)

        self.dyn_par_cb(None)

    def frenet_odom_cb(self, data: Odometry):
        self.frenet_odom = data

        cur_s = self.frenet_odom.pose.pose.position.x
        cur_v = self.frenet_odom.twist.twist.linear.x
        if np.round(cur_s / 0.05, 1) % 1 == 0.0:  # take points at 0.05 m distance
            if self.pub_cur_vel_flag:
                self.vel_cur_pub.publish(Point(x=cur_s, y=cur_v))
                self.steer_cur_pub.publish(Point(x=cur_s, y=self.steer_input))
                self.pub_cur_vel_flag = False
        else:
            if not self.pub_cur_vel_flag:
                self.pub_cur_vel_flag = True

    def gb_wpnt_sc_cb(self, data: WpntArray):
        self.gb_wpnt_sc = data.wpnts

    def dyn_par_cb(self, data: Config):
        rospy.wait_for_message("/global_waypoints_scaled", WpntArray)

        rate = rospy.Rate(2000)
        for wpnt in self.gb_wpnt_sc:
            self.vel_traj_pub.publish(Point(x=wpnt.s_m, y=wpnt.vx_mps))
            self.steer_traj_pub.publish(
                Point(x=wpnt.s_m, y=np.arctan(self.wheelbase*wpnt.kappa_radpm))
            )
            rate.sleep()  # needed for publishing correctly all the points

    def ctrl_cb(self, data: AckermannDriveStamped):
        self.steer_input = data.drive.steering_angle

    def loop(self):
        # this is not the loop of the node
        # the sleep is only needed to not avoid misses in publishing the messages
        rate = rospy.Rate(2000)
        for wpnt in self.gb_wpnt_sc:
            self.vel_traj_pub.publish(Point(x=wpnt.s_m, y=wpnt.vx_mps))
            self.steer_traj_pub.publish(
                Point(x=wpnt.s_m, y=np.arctan(self.wheelbase*wpnt.kappa_radpm))
            )
            rate.sleep()

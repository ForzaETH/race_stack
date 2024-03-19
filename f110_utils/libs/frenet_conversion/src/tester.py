#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import rospy
from f110_msgs.msg import Wpnt, WpntArray
from frenet_converter.frenet_converter import FrenetConverter
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class FrenetTester:
    def __init__(self) -> None:
        """
        A class to test the `frenet_converter` python library

        """
        rospy.init_node("frenet_tester", anonymous=True)
        rospy.on_shutdown(self.on_shutdown)

        self.waypoints = None
        self.init = False
        self.s_new = None
        self.d_new = None
        self.s_old = None
        self.d_old = None

        # Subscriptions
        rospy.Subscriber("/global_waypoints", WpntArray, self.global_waypoints_cb)
        rospy.Subscriber("/car_state/odom_frenet", Odometry, self.car_frenet_cb)
        rospy.Subscriber("/car_state/pose", PoseStamped, self.car_pose_cb)

        # Publishers
        self.car_frenet = rospy.Publisher("/car_frenet", Wpnt, queue_size=1)

        # Initialize node
        self.init = self.initialize_converter()
        rospy.loginfo("[Frenet tester] initialized")

    def global_waypoints_cb(self, msg: WpntArray) -> None:
        """
        Callback for the global waypoints topic

        Args:
            msg (WpntArray): the global waypoints

        """

        self.waypoints = np.array([[wpnt.x_m, wpnt.y_m] for wpnt in msg.wpnts])

    def car_pose_cb(self, msg: PoseStamped) -> None:
        """
        Callback for the car pose topic

        Args:
            msg (PoseStamped): the car pose

        """

        if self.waypoints is not None and self.init:
            self.cart_x = msg.pose.position.x
            self.cart_y = msg.pose.position.y
            # Get the Frenet coordinates of the car
            car_frenet = self.converter.get_frenet(np.array([msg.pose.position.x]), np.array([msg.pose.position.y]))
            car_frenet_msg = Wpnt()
            car_frenet_msg.x_m = car_frenet[0]
            car_frenet_msg.y_m = car_frenet[1]
            self.s_new = car_frenet[0]
            self.d_new = car_frenet[1]
            self.car_frenet.publish(car_frenet_msg)

    def car_frenet_cb(self, msg: Odometry) -> None:
        """
        Get current Frenet estimate
        """
        self.s_old = msg.pose.pose.position.x
        self.d_old = msg.pose.pose.position.y

    def initialize_converter(self) -> bool:
        """
        Initialize the FrenetConverter object"""
        rospy.wait_for_message("/global_waypoints", WpntArray)

        # Initialize the FrenetConverter object
        self.converter = FrenetConverter(self.waypoints[:, 0], self.waypoints[:, 1])
        rospy.loginfo("[Frenet tester] initialized FrenetConverter object")

        return True

    def on_shutdown(self):
        # save the dataframe to a csv file
        self.df.to_csv("frenet_statistics.csv")
        rospy.loginfo("[Frenet tester] saved frenet statistics to frenet_statistics.csv")

        # plot the s_diff and the d_diff statistics in a histogram, and save the image
        self.df["s_diff"].hist()
        plt.savefig("frenet_statistics_s.png")

        # clear s_diff plot
        plt.clf()
        self.df["d_diff"].hist()
        plt.savefig("frenet_statistics_d.png")

        rospy.loginfo("[Frenet tester] plotted frenet statistics to frenet_statistics.png")

    def loop(self):
        # print difference between the old estimate and the new one
        rate = rospy.Rate(10)
        rospy.wait_for_message("/car_state/odom_frenet", Odometry)
        rospy.wait_for_message("/car_state/pose", PoseStamped)

        # create statitics dataframe
        self.df = pd.DataFrame(columns=["s_old", "d_old", "s_new", "d_new", "s_diff", "d_diff"])

        while not rospy.is_shutdown():
            # compute the difference between the old and the new estimate
            s_diff_front = (self.s_new - self.s_old) % self.converter.raceline_length
            s_diff_back = (self.s_old - self.s_new) % self.converter.raceline_length

            s_diff = min(s_diff_front, s_diff_back)
            d_diff = self.d_new - self.d_old

            # save statistics to a dataframe, with concat
            self.df = pd.concat(
                [
                    self.df,
                    pd.DataFrame(
                        [[self.s_old, self.d_old, self.s_new, self.d_new, s_diff, d_diff]],
                        columns=["s_old", "d_old", "s_new", "d_new", "s_diff", "d_diff"],
                    ),
                ]
            )

            # If the difference is too much, trow a loud error
            if abs(s_diff) > 1 or abs(d_diff) > 1:
                print(f"Old s:{self.s_old}, d:{self.d_old}")
                print(f"New s:{self.s_new}, d:{self.d_new}")
                print(f"Diff s:{s_diff}, d:{d_diff}")
                rospy.logerr("[Frenet tester] Frenet conversion is not working properly")
                raise ValueError("[Frenet tester] Frenet conversion is not working properly")

            cart_new = self.converter.get_cartesian(self.s_old, self.d_old)
            print(
                f"Difference between old and new cartesian coordinates: {np.linalg.norm([cart_new[0] - self.cart_x, cart_new[1] - self.cart_y])}"
            )

            rate.sleep()


if __name__ == "__main__":
    frenet_tester = FrenetTester()
    frenet_tester.loop()

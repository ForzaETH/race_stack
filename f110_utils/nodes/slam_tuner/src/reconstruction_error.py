import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
import tf2_ros
import numpy as np
from scipy.interpolate import CubicSpline

from datetime import datetime
import matplotlib.pyplot as plt
import csv

from typing import List, Tuple

from errormetrics.csv_io import *


class ReconstructionError:
    '''
    Provides error metrics and plots for localization methods.
    '''

    def __init__(self):
        rospy.init_node('rec_err_node', anonymous=True)

        self.LOC_ALGO: str = rospy.get_param("/rec_err/loc_algo", "Unknown")
        '''String used to identify the localization algorithm to be compared against'''
        self.TIMEOUT: float = rospy.get_param("/rec_err/timeout", 15.0)
        '''Timeout to auto-compute a plot (when robot is standing still)'''
        self.POSE_TOPIC: str = rospy.get_param(
            '/rec_err/pose_topic', "/car_state/pose")
        '''Topic to get the "ground-truth" pose information from'''
        self.NUM_POSE_MSGS: int = rospy.get_param('/rec_err/n_pose_msgs', 20)
        '''Number of pose messages to skip between datapoints (filtering method)'''
        self.FILENAME: str = rospy.get_param(
            '/rec_err/out_graph', f"{self.LOC_ALGO}_err_metrics")
        self.FILENAME += ("_"+datetime.now().strftime("%d%m%y_%H%M%S"))
        '''Output file path (defaults to relative)'''

        # Wait until the requried tf transforms exist
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        print("Reconstruction Error Node waiting for TF Transformations")
        while not self.tf_buffer.can_transform("map", "base_link", rospy.Time(), rospy.Duration(1.0)):
            rospy.logwarn("Waiting for base_link->imu transformation")
        print("base_link->imu transformation OK")

        rospy.Subscriber("/create_plot", Empty, self.plot_data)
        rospy.Subscriber(self.POSE_TOPIC, PoseStamped, self.pose_cb)
        rospy.Subscriber("/tracked_pose", PoseStamped, self.slam_pose_cb)

        self._car_state_pose_cnt: int = 0        # to filter out noisy pose data
        self._x_acc = []
        self._y_acc = []
        self._sint_acc = []
        self._cost_acc = []

        self.prev_position: List[float] = None  # SE2 position of car
        self.curr_position: List[float] = None  # SE2 position of car

        self.was_stationary = False
        self.time_since_stationary: float = 0

        # Numpy arrays for interpolation --> time-syncing
        self.odom_msgs = []
        self.map_bl_list = []
        self.map_odom_list = []
        self.odom_bl_list = []

        self.tf_x = []
        self.tf_y = []
        self.tf_sint = []
        self.tf_cost = []
        self.tf_t = []

        # for tracking pose
        self.tracked_x = []
        self.tracked_y = []
        self.tracked_theta = []
        self.tracked_time = []
        self.tracked_pose = []

    def slam_pose_cb(self, data: PoseStamped) -> None:
        """
        Callback function of /tracked_pose subscriber.

        Parameters
        ----------
        data
            Data received from /car_state/pose topic
        """
        x = data.pose.position.x
        y = data.pose.position.y
        theta = euler_from_quaternion([data.pose.orientation.x, data.pose.orientation.y,
                                       data.pose.orientation.z, data.pose.orientation.w])[2]
        t = data.header.stamp.to_sec()

        self.tracked_x.append(x)
        self.tracked_y.append(y)
        self.tracked_theta.append(theta)
        self.tracked_time.append(t)
        self.tracked_pose.append((x, y, theta, t))

    def pose_cb(self, data: PoseStamped):
        """
        Callback function of /car_state/pose subscriber.

        Parameters
        ----------
        data
            Data received from /car_state/pose topic
        """

        x = data.pose.position.x
        y = data.pose.position.y
        theta = euler_from_quaternion([data.pose.orientation.x, data.pose.orientation.y,
                                       data.pose.orientation.z, data.pose.orientation.w])[2]

        # // Get the latest transforms
        self.update_tf_lists()

        # // Append most recent coordinates to 1d arrays for later interpolation (ensure that it's increasing in time)
        if len(self.tf_t) == 0 or self.map_bl_list[-1][-1] > self.tf_t[-1]:
            self.tf_x.append(self.map_bl_list[-1][0])
            self.tf_y.append(self.map_bl_list[-1][1])
            self.tf_sint.append(np.sin(self.map_bl_list[-1][2]))
            self.tf_cost.append(np.cos(self.map_bl_list[-1][2]))
            self.tf_t.append(self.map_bl_list[-1][3])

        # // Reject noisy pose data taking the running average of N pose estimates
        self._x_acc.append(x)
        self._y_acc.append(y)
        self._sint_acc.append(np.sin(theta))
        self._cost_acc.append(np.cos(theta))
        if len(self._x_acc) > self.NUM_POSE_MSGS:   # pop off the first elem in sliding win
            self._x_acc = self._x_acc[1:]
            self._y_acc = self._y_acc[1:]
            self._sint_acc = self._sint_acc[1:]
            self._cost_acc = self._cost_acc[1:]

        x = np.mean(self._x_acc)
        y = np.mean(self._y_acc)
        theta = np.arctan2(
            np.mean(self._sint_acc),
            np.mean(self._cost_acc)
        )

        self.odom_msgs.append((x, y, theta, data.header.stamp.to_sec()))
        self._car_state_pose_cnt = 0

        if self.curr_position is None:
            self.prev_position = [x, y, theta]
        self.curr_position = [x, y, theta]

        # // Check if the car has been idle for more than TIMEOUT seconds
        dx = (self.prev_position[0]-x)
        dy = (self.prev_position[1]-y)
        dtheta = (self.prev_position[2]-theta)

        if np.sqrt(dx**2+dy**2) < 0.25 and dtheta < 0.2:
            # Not enough movement, we are not moving!
            if self.was_stationary is False:
                self.was_stationary = True
                self.time_since_stationary = data.header.stamp.to_sec()
            else:
                time_elapsed = data.header.stamp.to_sec() - self.time_since_stationary
                # rospy.loginfo(f"Reconstruction Error: {time_elapsed:.2f}s since stationary.")

                if time_elapsed > self.TIMEOUT:
                    rospy.loginfo(
                        f"[Reconstruction Error]: {self.TIMEOUT:.2f}s passed since car stopped. Plotting data...")
                    self.was_stationary = False
                    self.time_since_stationary = data.header.stamp.to_sec()
                    self.plot_data(None)    # Auto plot, reset
        else:
            # We have moved off
            self.was_stationary = False

    def update_tf_lists(self):
        '''Updates the running lists of TF data. We keep 3 lists ongoing: map->odom, map->base_link, odom->base_link.'''

        def grab_tf_data(frame1, frame2, lst):
            # Get the latest transform between frame1 and frame2, but not if it has the same header (timestep) as the previous datapoint
            tf_msg = self.tf_buffer.lookup_transform(
                frame1, frame2, rospy.Time(0.0), rospy.Duration(2.0))

            if len(lst) and tf_msg.header.stamp.to_sec() == lst[-1][-1]:
                return

            tf_x = tf_msg.transform.translation.x
            tf_y = tf_msg.transform.translation.y
            tf_theta = euler_from_quaternion([tf_msg.transform.rotation.x, tf_msg.transform.rotation.y,
                                              tf_msg.transform.rotation.z, tf_msg.transform.rotation.w])[2]

            lst.append((tf_x, tf_y, tf_theta, tf_msg.header.stamp.to_sec()))

        grab_tf_data("map", "base_link", self.map_bl_list)
        grab_tf_data("map", "odom", self.map_odom_list)
        grab_tf_data("odom", "base_link", self.odom_bl_list)

    def plot_data(self, _):
        '''
        On recieving a message on the /create_plot topic, we calculate and plot out error metrics.
        '''

        rospy.logwarn(f"[Reconstruction Error]: Saving to {self.FILENAME}")

        pos_err = []
        ang_err = []
        t_vec = []
        data_out = []

        # Get interpolators for TF data
        tf_x_cs = CubicSpline(self.tf_t, self.tf_x)
        tf_y_cs = CubicSpline(self.tf_t, self.tf_y)
        tf_sint_cs = CubicSpline(self.tf_t, self.tf_sint)
        tf_cost_cs = CubicSpline(self.tf_t, self.tf_cost)

        # interpolate to get an error metric
        for odom_pose in self.odom_msgs:
            t = odom_pose[3]
            interp_x = tf_x_cs(t)
            interp_y = tf_y_cs(t)

            interp_sint = tf_sint_cs(t)
            interp_cost = tf_cost_cs(t)
            interp_theta = np.arctan2(interp_sint, interp_cost)

            # Get error
            pos_error = np.sqrt(
                (interp_x-odom_pose[0])**2 + (interp_y-odom_pose[1])**2)
            ang_error = np.abs(np.arctan2(
                np.sin(odom_pose[2]-interp_theta), np.cos(odom_pose[2]-interp_theta)))

            # Accumulate error
            pos_err.append(pos_error)
            ang_err.append(ang_error)
            t_vec.append(t)

            data_out.append(
                (t, odom_pose[0], odom_pose[1], odom_pose[2], interp_x, interp_y, interp_theta))

        # // Save t: odom(x,y,t), tf(x,y,t)
        save_csv(data_out, self.FILENAME+".csv",
                 header=["Time", "odom_x", "odom_y", "odom_theta", "tf_x", "tf_y", "tf_theta"])

        # // plot and save lateral error metric
        t_vec = [t-t_vec[0] for t in t_vec]     # start time from t=0
        avg_pos_err = np.average(pos_err)

        fig, (ax1, ax2) = plt.subplots(nrows=2, ncols=1)
        plt.title(
            f"Error metrics for localization method \"{self.LOC_ALGO}\", mean pos err: {avg_pos_err:.2f}m")

        color = 'tab:red'
        ax1.grid()
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel("Absolute Position error (m)")
        ax1.plot(t_vec, pos_err, color=color)

        color = 'tab:blue'
        ax2.grid()
        ax2 .set_xlabel('Time (s)')
        ax2.set_ylabel("Absolute Angular error (rad)")
        ax2.plot(t_vec, ang_err, color=color)

        fig.tight_layout()
        plt.savefig(self.FILENAME+".png")
        plt.close()

        # // Plot relative trajectories
        fig, axs = plt.subplots(nrows=1, ncols=3)
        plt.title(
            f"Trajectory history")

        color = 'tab:red'
        # alpha = np.arange(0.5, 1.0, len(self.map_bl_list))
        axs[0].set_title("map to base_link")
        axs[0].grid()
        axs[0].set_xlabel('x')
        axs[0].set_ylabel("y")
        axs[0].plot([t[0] for t in self.map_bl_list],
                    [t[1] for t in self.map_bl_list],
                    'x-',
                    color=color,
                    # alpha=alpha,
                    label="Map->Base_link TF Data"
                    )
        axs[0].plot([t[0] for t in self.tracked_pose],
                    [t[1] for t in self.tracked_pose],
                    'x-',
                    # alpha=alpha,
                    label="Tracked_Pose"
                    )
        axs[0].legend()

        color = 'tab:blue'
        # alpha = np.arange(0.5, 1.0, len(self.odom_bl_list))
        axs[1].set_title("odom to base_link")
        axs[1].grid()
        axs[1].set_xlabel('x')
        axs[1].set_ylabel("y")
        axs[1].plot([t[0] for t in self.odom_bl_list],
                    [t[1] for t in self.odom_bl_list],
                    'x-',
                    color=color,
                    # alpha=alpha
                    )

        color = 'tab:green'
        # alpha = np.arange(0.5, 1.0, len(self.map_odom_list))
        axs[2].set_title("map to odom")
        axs[2].grid()
        axs[2].set_xlabel('x')
        axs[2].set_ylabel("y")
        axs[2].plot([t[0] for t in self.map_odom_list],
                    [t[1] for t in self.map_odom_list],
                    'x-',
                    color=color,
                    # alpha=alpha
                    )

        fig.tight_layout()
        plt.savefig(self.FILENAME+"_traj_hist.png")
        plt.close()

        # // Save TF error metrics
        save_csv(self.map_bl_list, self.FILENAME+"_tf_mbl.csv",
                 header=["x", "y", "theta", "t"])
        save_csv(self.map_odom_list, self.FILENAME+"_tf_mo.csv",
                 header=["x", "y", "theta", "t"])
        save_csv(self.odom_bl_list, self.FILENAME+"_tf_obl.csv",
                 header=["x", "y", "theta", "t"])
        save_csv(self.tracked_pose, self.FILENAME+"_tracked_pose.csv",
                 header=["x", "y", "theta", "t"])

        # Get data on smoothed error


if __name__ == "__main__":
    node = ReconstructionError()
    rospy.spin()

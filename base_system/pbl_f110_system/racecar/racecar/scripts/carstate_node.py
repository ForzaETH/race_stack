#!/usr/bin/env python3
import tf2_ros
import tf.transformations as tft
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Vector3, TransformStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry, Path
import numpy as np

from typing import List

class CarStateNode():
    def __init__(self, prop_state=False):
        rospy.init_node('carstate_node', anonymous=True)
        self.DEBUG = rospy.get_param('/carstate_node/debug')
        self.LOCALIZATION = rospy.get_param('/carstate_node/localization', default="slam") 
        self.ODOM_TOPIC = rospy.get_param('/carstate_node/odom_topic')
        self.IMU_TOPIC = rospy.get_param('/carstate_node/imu_topic', default='/vesc/sensors/imu/raw')

        self.ekf_odom = None
        self.prop_state = prop_state
        self.acc = np.zeros(5)

        # Wait until the requried tf transforms exist
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        print("Waiting for TF Transformations")
        while not self.tf_buffer.can_transform("base_link", "imu", rospy.Time(), rospy.Duration(1.0)):
            rospy.logwarn("Waiting for base_link->imu transformation")
        print("base_link->imu transformation OK")

        while not self.tf_buffer.can_transform("odom", "base_link", rospy.Time(), rospy.Duration(1.0)):
            rospy.logwarn("Waiting for odom->base_link transformation")
        print("odom->base_link transformation OK")

        while not self.tf_buffer.can_transform("map", "odom", rospy.Time(), rospy.Duration(1.0)):
            rospy.logwarn("Waiting for map->odom transformation")
        print("map->odom transformation OK")

        rospy.Subscriber(self.ODOM_TOPIC, Odometry, self.ekf_odom_cb, tcp_nodelay=True)
        rospy.Subscriber(self.IMU_TOPIC, Imu, self.imu_cb, tcp_nodelay=True)
        self.pub_acc = rospy.Publisher('acc_estimate', Float64, queue_size=1, tcp_nodelay=True)
        self.pub_state_pose = rospy.Publisher('/car_state/pose', PoseStamped, queue_size=1, tcp_nodelay=True)
        self.pub_state_odom = rospy.Publisher('/car_state/odom', Odometry, queue_size=1, tcp_nodelay=True)
        self.pub_state_path = rospy.Publisher('/car_state/path', Path, queue_size=1, tcp_nodelay=True)
        self.pub_state_pitch = rospy.Publisher('/car_state/pitch', Float32, queue_size=1, tcp_nodelay=True)

        self.ekf_odom = None
        self.imu_data = None
        self.path_msg = Path()
        self.MAX_PATH_LEN = 500
        self.prop_state = prop_state
        self.path_counter = 0

        self.state_loop()

    def ekf_odom_cb(self, data):
        self.ekf_odom = data

        if self.prop_state:
            self.pub_acc.publish(np.mean(self.acc))
            self.ekf_odom.twist.twist.linear.x += 0.2*np.mean(self.acc)

    def imu_cb(self, data):
        # Update acceleration buffer
        self.acc[1:] = self.acc[:-1]
        self.acc[0] = -data.linear_acceleration.y

        # Update IMU message
        self.imu_data = data

    def path_handle(self, pose, odom):
        #Append to path for rviz
        self.path_msg.header = pose.header
        vx = odom.twist.twist.linear.x
        vy = odom.twist.twist.linear.y
        #only append if we are moving somewhat
        if np.sqrt(vx**2 + vy**2) > 0.1 and self.path_counter%8 == 0:
            self.path_msg.poses.append(pose)
            self.path_counter = 0
        else:
            self.path_counter += 1
        #Round robin on the array
        if len(self.path_msg.poses) >= self.MAX_PATH_LEN:
            self.path_msg.poses.pop(0)
            #self.path_msg.poses = self.path_msg.poses[-self.MAX_PATH_LEN:]

    def imu_to_rpy(self, imu:Imu):
        imu_quat = imu.orientation
        #transform to base_link
        try:
            b_imu = self.tf_buffer.lookup_transform('base_link', 'imu', rospy.Time(), rospy.Duration(5))
        except tf2_ros.LookupException:
            rospy.logwarn("No transform exists yet for base_link->IMU!")
            return 0,0,0

        rot_quat = quaternion_to_list(b_imu.transform.rotation)
        base_quat = tft.quaternion_multiply([imu_quat.x, imu_quat.y, imu_quat.z, imu_quat.w], rot_quat)
        r,p,y = tft.euler_from_quaternion(base_quat)
        return r, p, y

    def state_loop(self):
        rate = rospy.Rate(80)  # rate in hertz
        print('Carstate Node waiting for Odometry and IMU messages...')
        rospy.wait_for_message(self.ODOM_TOPIC, Odometry)
        rospy.wait_for_message(self.IMU_TOPIC, Imu)
        print('Carstate Node received EKF and SLAM')

        self.tf_buffer.can_transform("map", "base_link", rospy.Time(0), rospy.Duration(3))

        # Publish SLAM positional & EKF velocity states
        while not rospy.is_shutdown():
            carstate_pose_msg = PoseStamped()
            carstate_odom_msg = Odometry()

            trans_mb = self.get_slam_messages()

            carstate_pose_msg.header = trans_mb.header
            carstate_pose_msg.pose.position.x = trans_mb.transform.translation.x
            carstate_pose_msg.pose.position.y = trans_mb.transform.translation.y
            carstate_pose_msg.pose.position.z = trans_mb.transform.translation.z
            carstate_pose_msg.pose.orientation.x = trans_mb.transform.rotation.x
            carstate_pose_msg.pose.orientation.y = trans_mb.transform.rotation.y
            carstate_pose_msg.pose.orientation.z = trans_mb.transform.rotation.z
            carstate_pose_msg.pose.orientation.w = trans_mb.transform.rotation.w

            carstate_odom_msg.header = carstate_pose_msg.header
            carstate_odom_msg.pose.pose.position = carstate_pose_msg.pose.position
            carstate_odom_msg.pose.pose.orientation = carstate_pose_msg.pose.orientation

            #Handle Speed from EKF
            carstate_odom_msg.twist = self.ekf_odom.twist

            #Get rpy from imu
            _, p, _ = self.imu_to_rpy(imu=self.imu_data)
            pitch_msg = Float32()
            pitch_msg.data = p

            #Publish everything
            self.pub_state_pose.publish(carstate_pose_msg)
            self.pub_state_odom.publish(carstate_odom_msg)

            self.pub_state_pitch.publish(pitch_msg)
            # DEBUG ROS param to activate path
            if self.DEBUG:
                pose = self.slam_pose if self.LOCALIZATION == "slam" else self.pf_pose
                self.path_handle(pose=pose, odom=self.ekf_odom)
                self.pub_state_path.publish(self.path_msg)

            rate.sleep()

    def get_slam_messages(self) -> TransformStamped:
        trans_mb = self.tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0.0), rospy.Duration(2.0))
        return trans_mb

def quaternion_to_list(q: Quaternion)->List:
    return [q.x, q.y, q.z, q.w]

def translation_to_list(t: Vector3)->List:
    return [t.x, t.y, t.z]

if __name__ == "__main__":
    carstate_node = CarStateNode(prop_state=False)


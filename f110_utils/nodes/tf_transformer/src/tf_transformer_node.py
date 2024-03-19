#!/usr/bin/env python3

import rospy
import tf
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance, Vector3Stamped
import numpy as np


class TfTransformer:
    def __init__(self):
        # subscribers
        # wait for tf to come alive
        rospy.sleep(1.0)
        self.tf_listener = tf.TransformListener()
        rospy.Subscriber("/tf", TFMessage, self.tf_cb, queue_size=10)

        # publishers
        # requires remap in launch file
        self.odom_pub = rospy.Publisher('tf_odom', Odometry, queue_size=10)

        # parameters
        #self.experiment = rospy.get_param("/id_controller/experiment")

        rospy.logwarn("TF Transformer Ready")
        # initialize with high time between messages to trigger reset

        self.last_odom = Odometry()
        self.last_odom.header.stamp = rospy.Time(0)

        self.dt = 100
        rospy.spin()

    def calculate_first_derivative(self, now, last):
        return (now - last)/self.dt

    def tf_cb(self, msg):
        stamp = 0
        # check if tf is the one we care about
        for transform in msg.transforms:
            if transform.child_frame_id == "base_link":
                stamp = transform.header.stamp
        if stamp == 0:
            return

        # get the tf from map to base_link for position and rotation
        # as we want to express these in world coordinates
        try:
            # takes the latest (i.e. the one we just received)
            (trans, rot) = self.tf_listener.lookupTransform(
                "map", "base_link", rospy.Time(0))
        except tf.Exception as error:
            rospy.logwarn(error)
            return

        # Create odom message
        odom_msg = Odometry()
        odom_msg.header.frame_id = "map"
        odom_msg.header.stamp = stamp

        # transform the current tf to pose, tf is from world to body
        pose_msg = PoseWithCovariance()
        pose_msg.pose.position.x = trans[0]
        pose_msg.pose.position.y = trans[1]
        pose_msg.pose.position.z = trans[2]
        # quaternion from world to body
        pose_msg.pose.orientation.x = rot[0]
        pose_msg.pose.orientation.y = rot[1]
        pose_msg.pose.orientation.z = rot[2]
        pose_msg.pose.orientation.w = rot[3]

        odom_msg.pose = pose_msg

        self.dt = stamp.to_sec() - self.last_odom.header.stamp.to_sec()
        if self.dt < 0.1:
            v = Vector3Stamped()

            v.vector.x = \
                self.calculate_first_derivative(
                    trans[0], self.last_odom.pose.pose.position.x)
            v.vector.y = \
                self.calculate_first_derivative(
                    trans[1], self.last_odom.pose.pose.position.y)
            v.vector.z = \
                self.calculate_first_derivative(
                    trans[2], self.last_odom.pose.pose.position.z)
            v.header.stamp = rospy.Time(0)
            v.header.frame_id = "map"
            # print(v)
            # # transform twist into body frame using the transform (V' = RVR')
            # # we need w component at first place for this
            # t = np.array(rot[0:3])
            # rv = (-np.matmul(t.T, v), rot[3]*v+np.cross(t,v))
            # v_t = (rv[0]*rot[3]-np.matmul(rv[1].T,-t),rv[0]*-t+rot[3]*rv[1]+np.cross(rv[1],-t))

            # v_t = self.tf_listener.transformVector3("base_link", v)
            rpy = tf.transformations.euler_from_quaternion(rot)

            twist_msg = TwistWithCovariance()

            twist_msg.twist.linear.x = v.vector.x * \
                np.cos(rpy[2]) + v.vector.y * np.sin(rpy[2])
            twist_msg.twist.linear.y = v.vector.y * \
                np.cos(rpy[2]) - v.vector.x * np.sin(rpy[2])
            twist_msg.twist.linear.z = v.vector.z

            # twist_msg.twist.linear.x = v_t.vector.x
            # twist_msg.twist.linear.y = v_t.vector.y
            # twist_msg.twist.linear.z = v_t.vector.z

            # quaternion derivatives:
            # we need quat from body to world as we want angular rate in body frame
            e0 = rot[3]
            e1 = rot[0]
            e2 = rot[1]
            e3 = rot[2]
            de0 = self.calculate_first_derivative(
                e0, self.last_odom.pose.pose.orientation.w)
            de1 = self.calculate_first_derivative(
                e1, self.last_odom.pose.pose.orientation.x)
            de2 = self.calculate_first_derivative(
                e2, self.last_odom.pose.pose.orientation.y)
            de3 = self.calculate_first_derivative(
                e3, self.last_odom.pose.pose.orientation.z)

            # for quaternions refer to the robot dynamics lecture notes for transformation
            # between time derivative of quaternion to angular rate

            H = np.matrix(
                [[-e1, e0, -e3, e2],
                 [-e2, e3, e0, -e1],
                    [-e3, -e2, e1, e0]])
            de = np.matrix([[de0], [de1], [de2], [de3]])
            angular_velocity = 2 * np.matmul(H, de)
            twist_msg.twist.angular.x = angular_velocity[0]
            twist_msg.twist.angular.y = angular_velocity[1]
            twist_msg.twist.angular.z = angular_velocity[2]
            odom_msg.twist = twist_msg
            self.odom_pub.publish(odom_msg)

        self.last_odom = odom_msg


if __name__ == '__main__':
    rospy.init_node('tf_transformer')

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        TfTransformer = TfTransformer()
    except rospy.ROSInterruptException:
        pass

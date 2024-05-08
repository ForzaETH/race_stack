
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion

import csv

class PathRecorder:
    '''
    This node subscribes to a Odometry, Pose, or PoseWithCovarainceStamped source and saves it to a CSV file.
    '''

    def __init__(self) -> None:
        rospy.init_node("path_recorder_node", anonymous=True)

        self.SUB_TOPIC = rospy.get_param("~sub_topic")
        self.TOPIC_TYPE = rospy.get_param("~topic_type").lower()
        if self.TOPIC_TYPE=="odometry":
            rospy.Subscriber(self.SUB_TOPIC, Odometry, self.callback)
        elif self.TOPIC_TYPE=="posestamped":
            rospy.Subscriber(self.SUB_TOPIC, PoseStamped, self.callback)
        elif self.TOPIC_TYPE=="posewithcovariancestamped":
            rospy.Subscriber(self.SUB_TOPIC, PoseWithCovarianceStamped, self.callback)
        else:
            raise ValueError(f"Requested {self.TOPIC_TYPE} not supported. Choose between \'Odometry\', \'PoseStamped\', \'PoseWithCovarianceStamped\'.")

        self.OUT_FILENAME = rospy.get_param("~out_filename", f"{self.TOPIC_TYPE}_traj.csv")
        with open(self.OUT_FILENAME, 'w') as f:
            w = csv.writer(f)
            w.writerow(["t", "x", "y", "theta"])

    def callback(self, msg):
        if self.TOPIC_TYPE=="odometry":
            pose:Pose = msg.pose.pose
        elif self.TOPIC_TYPE=="posestamped":
            pose:Pose = msg.pose
        elif self.TOPIC_TYPE=="posewithcovariancestamped":
            pose:Pose = msg.pose.pose

        t = msg.header.stamp.to_sec()
        x = pose.position.x
        y = pose.position.y
        theta = euler_from_quaternion([pose.orientation.x, pose.orientation.y,
                                       pose.orientation.z, pose.orientation.w])[2]

        with open(self.OUT_FILENAME, 'a') as f:
            w = csv.writer(f)
            w.writerow([t, x, y, theta])

if __name__ == "__main__":
    n = PathRecorder()
    rospy.spin()
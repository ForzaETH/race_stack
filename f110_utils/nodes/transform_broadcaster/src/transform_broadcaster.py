import rospy
import tf2_ros
from nav_msgs.msg import Odometry
import geometry_msgs.msg

def handle_odom_update(msg:Odometry):
    # Transforms from odom to base_link.
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = msg.header.stamp
    t.header.frame_id = msg.header.frame_id
    t.child_frame_id = tf_child_frame

    pos = msg.pose.pose.position
    t.transform.translation.x = pos.x
    t.transform.translation.y = pos.y
    t.transform.translation.z = pos.z

    ori = msg.pose.pose.orientation
    t.transform.rotation.x = ori.x
    t.transform.rotation.y = ori.y
    t.transform.rotation.z = ori.z
    t.transform.rotation.w = ori.w

    br.sendTransform(t)

if __name__=="__main__":
    rospy.init_node("odom_transform_broadcaster", anonymous=True)
    odom_topic = rospy.get_param("/transform_broadcaster/odom_topic")
    tf_child_frame = rospy.get_param("/transform_broadcaster/child_frame", "base_link")
    rospy.Subscriber(odom_topic, Odometry, handle_odom_update)

    rospy.spin()
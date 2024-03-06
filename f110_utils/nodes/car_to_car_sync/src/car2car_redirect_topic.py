#!/usr/bin/env python3
import threading
import roslibpy
import rospy
from twisted.internet import reactor
from nav_msgs.msg import Odometry
from rospy_message_converter import message_converter


def start_roslibpy():
    reactor.run(installSignalHandlers=0)

def callback(data, args):
    topic_name, topic_type = args
    publisher = roslibpy.Topic(client, topic_name, topic_type)

    # Convert the ROS message to a dictionary
    message_dict = message_converter.convert_ros_message_to_dictionary(data)
    
    # Publish the message
    publisher.publish(roslibpy.Message(message_dict))

def redirector():
    rospy.init_node('car2car_sync_topic_redirector', anonymous=True)
    professor_ip = rospy.get_param('~professor_car_ip', '192.168.192.16') #NUC1 IP default
    topics = rospy.get_param('/car2car_topics')
    
    global client
    client = roslibpy.Ros(host=professor_ip, port=9090)
    global roslibpy_thread
    roslibpy_thread = threading.Thread(target=start_roslibpy)
    roslibpy_thread.start()

    for topic in topics:
        if topic['type'] == "nav_msgs/msg/Odometry":
            rospy.Subscriber(topic['subname'], Odometry, callback, (topic['pubname'], Odometry)) # Update the topic type here
            publisher = roslibpy.Topic(client, topic['pubname'], "nav_msgs/Odometry")
            publisher.advertise()
        else:
            rospy.logwarn("[Car to Car Redirect] Unsupported message type: " + topic['type'] + "Please adjust in car2car_sync_topic_redirector.py")

    rospy.spin()

def shutdown_hook():
    print("[Car to Car Redirect] Shutting down...")
    client.terminate()
    reactor.callFromThread(reactor.stop)
    roslibpy_thread.join()
    print("[Car to Car Redirect] Shutdown complete.")

if __name__ == '__main__':
    try:
        rospy.on_shutdown(shutdown_hook)
        redirector()
    except rospy.ROSInterruptException:
        pass

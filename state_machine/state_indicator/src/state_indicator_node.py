#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int8
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped
from ackermann_msgs.msg import AckermannDriveStamped
import tf2_ros
import tf2_geometry_msgs
import math

class StateIndicatorNode:
    """
    This class implements a ROS node that handles the terminal utilities of the state indicator. Furthermore it acts as a republisher node.
    Intensive computations are offloaded from the MCU to this node by subscribing to data and processing it into lightweight messages 
    for the MCU, as well as publishing them at a lower rate to reduce computational load on the MCU.

    It subscribes to the following topics:
    - `/state_machine`: Subscribes to the current state from the state machine.
    - `/trailing_opponent_marker`: Subscribes to the marker of the obstacle the racecar is trailing.
    - `/vesc/high_level/ackermann_cmd_mux/input/nav_1`: Subscribes to the ackermann telemetry data of the racecar.
    
    The node publishes the following topics:
    - `/state_indicator/mode`: Publishes the mode based on the user input through the terminal.
    - `/state_indicator/state`: Republishes the state with 20 Hz.
    - `/state_indicator/angle_led_index`: Publishes the index of the LED that should be light up on the state indicator to indicate 
                                          the position of the tracked obstacle at 10 Hz.
    - `/state_indicator/speed`: Republishes the speed of the racecar from the ackermann message at 10 Hz.
    - `/state_indicator/steering_angle`: Republishes the steering angle of the racecar from the ackermann message at 10 Hz.
    """

    def __init__(self):
        """
        Initialize the node, subscribe to topics, create publishers and set up member variables.
        """

        # Initialize the node
        self.name = "state_indicator_node"
        rospy.init_node(self.name, anonymous=True)

        # Subscribe to the topics
        rospy.Subscriber('/state_machine', String, self.state_callback)
        rospy.Subscriber('/trailing_opponent_marker', Marker, self.trainling_angle_callback)
        rospy.Subscriber('/vesc/high_level/ackermann_cmd_mux/input/nav_1', AckermannDriveStamped, self.ackermann_callback)

        # Publish the topics
        self.mode_pub = rospy.Publisher('/state_indicator/mode', Int32, queue_size=10)
        self.state_pub = rospy.Publisher('/state_indicator/state', String, queue_size=10)
        self.trailing_angle_led_index_pub = rospy.Publisher('/state_indicator/trailing_angle_led_index', Int8, queue_size=10)
        self.speed_pub = rospy.Publisher('/state_indicator/speed', Float32, queue_size=10)
        self.steering_pub = rospy.Publisher('/state_indicator/steering_angle', Float32, queue_size=10)

        # Define variables used to control the publishing frequency of the topics
        self.last_publish_time_state = rospy.Time.now()
        self.last_publish_time_ackermann = rospy.Time.now()
        self.last_publish_time_trailing_angle = rospy.Time.now()

        # Desired frequency in Hz
        self.publish_frequency_state = 20
        self.publish_frequency_ackermann = 10
        self.publish_frequency_trailing_angle = 10

        # Calculate the minimum interval
        self.min_publish_interval_state = 1.0 / self.publish_frequency_state
        self.min_publish_interval_ackermann = 1.0 / self.publish_frequency_ackermann
        self.min_publish_interval_trailing_angle = 1.0 / self.publish_frequency_trailing_angle

        # Initialize the mode and state
        self.current_mode = "STATE_MODE"
        self.current_state = "GB_TRACK"
        self.previous_state = "GB_TRACK"

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.led_count = 12 # Number of LEDs on the LED ring

        # Depending on how the LED ring is mounted relative to the sombrero, the angle_offset may need to be adjusted
        self.angle_offset = 180.0 # [degrees]

    #############
    # CALLBACKS #
    #############

    def state_callback(self, state_msg):
        """
        Callback function for the state of the racecar. Republishes the state topic at 30 Hz as a String.

        Args:
        - state_msg (String): The received message containing the state.
        """
    
        current_time = rospy.Time.now()
        # Check if we should publish based on the last_publish_time and the minimum_interval
        if (current_time - self.last_publish_time_state).to_sec() > self.min_publish_interval_state:
            if self.current_state != state_msg.data:
                if self.current_mode == "STATE_MODE":
                    # If the state has changed, print the change to the terminal and update current and previous states
                    rospy.loginfo(f"Switched to state: {state_msg.data}")
                    self.previous_state = self.current_state
                    self.current_state = state_msg.data
        
            # Republish the latest state message received
            self.state_pub.publish(state_msg)
            self.last_publish_time_state = current_time

    def ackermann_callback(self, ackermann_msg):
        """
        Callback function for the telemetry data of the racecar.Republishes the speed and steering angle topic at 10 Hz as a Float32.

        Args:
        - ackermann_msg (AckermannDriveStamped): The received message containing the telemetry data.
        """

        # Extract steering_angle and speed from received message
        steer = ackermann_msg.drive.steering_angle
        speed = ackermann_msg.drive.speed

        current_time = rospy.Time.now()
        # Check if we should publish based on the last_publish_time and the minimum_interval
        if (current_time - self.last_publish_time_ackermann).to_sec() > self.min_publish_interval_ackermann:
            # Publish steering angle and speed to separate topics
            self.steering_pub.publish(steer)
            self.speed_pub.publish(speed)
            self.last_publish_time_ackermann = current_time

    def trainling_angle_callback(self, marker_msg):
        """
        Callback function for the trainling angle. Extracts the opponent point from the marker and transforms it into the 'base_link' frame.
        Subsequently it calculates the angle between the opponent point and the x-axis and maps it to the an LED index, representing which LED on the ring to light up.
        Publishes the LED index at 10 Hz as an Int8.

        Args:
        - marker_msg (Marker): The received message containing the telemetry data.
        """

        current_time = rospy.Time.now()
        if (current_time - self.last_publish_time_trailing_angle).to_sec() > self.min_publish_interval_trailing_angle:
        
            # Create the opponent point in the 'map' frame
            opponent_point = PointStamped()
            opponent_point.header.frame_id = marker_msg.header.frame_id
            opponent_point.point = marker_msg.pose.position

            try:
                # Transform the point to the 'base_link' frame
                transform = self.tf_buffer.lookup_transform('base_link', 'map', rospy.Time())
                opponent_point_base_link = tf2_geometry_msgs.do_transform_point(opponent_point, transform)

                # Calculate the angle between the opponent point and the x-axis in 'base_link'
                angle = math.atan2(opponent_point_base_link.point.y, opponent_point_base_link.point.x)

                # map the angle to the LED index and publish it as an Int8
                angle_degrees = math.degrees(angle) + self.angle_offset
                angle_degrees = angle_degrees % 360
                led_index = int(round((360.0 - angle_degrees) * (self.led_count - 1) / 360.0))
                self.trailing_angle_led_index_pub.publish(Int8(led_index))
                self.last_publish_time_trailing_angle = current_time

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.loginfo('[State Indicator]: TF Exception')

    #############
    # MAIN LOOP #
    #############

    def run(self):
        rospy.loginfo("Type '1' for STATE_MODE or '2' for SPEED_STEERING_MODE")

        while not rospy.is_shutdown():
            user_input = input().strip()
            if user_input in ['1', '2']:
                mode = int(user_input)
                if mode == 1:
                    self.current_mode = "STATE_MODE"
                elif mode == 2:
                    self.current_mode = "SPEED_STEERING_MODE"
                self.mode_pub.publish(mode)
                rospy.loginfo(f"Switched to mode {mode}.")
            else:
                rospy.logwarn("Invalid input. Type '1' or '2' to switch between modes.")

        rospy.loginfo("Shutting down.")

if __name__ == '__main__':
    state_indicator = StateIndicatorNode()
    state_indicator.run()


#!/usr/bin/env python3
import rospy
import time
from std_msgs.msg import Float64
from vesc_msgs.msg import VescStateStamped
from nav_msgs.msg import Odometry
from dynamic_reconfigure.msg import Config


# import some utils.
import numpy as np
import copy as copy

class InterpolateThrottle:
    def __init__(self):
        # Allow our topics to be dynamic.
        self.rpm_input_topic   = rospy.get_param('~rpm_input_topic', '/vesc/commands/motor/unsmoothed_speed')
        self.rpm_output_topic  = rospy.get_param('~rpm_output_topic', '/vesc/commands/motor/speed')

        self.servo_input_topic   = rospy.get_param('~servo_input_topic', '/vesc/commands/servo/unsmoothed_position')
        self.servo_output_topic  = rospy.get_param('~servo_output_topic', '/vesc/commands/servo/position')

        self.current_input_topic   = rospy.get_param('~current_input_topic', '/vesc/commands/motor/unsmoothed_current')
        self.current_output_topic  = rospy.get_param('~current_output_topic', '/vesc/commands/motor/current')

        self.brake_input_topic   = rospy.get_param('~brake_input_topic', '/vesc/commands/motor/unsmoothed_brake')
        self.brake_output_topic  = rospy.get_param('~brake_output_topic', '/vesc/commands/motor/brake')

        # for motor speed commands
        self.max_acceleration = rospy.get_param('/vesc/max_acceleration')
        self.max_rpm = rospy.get_param('/vesc/vesc_driver/speed_max')
        self.min_rpm = rospy.get_param('/vesc/vesc_driver/speed_min')
        self.throttle_smoother_rate = rospy.get_param('/vesc/throttle_smoother_rate')
        self.speed_to_erpm_gain = rospy.get_param('/vesc/speed_to_erpm_gain')
        self.motor_timeout = rospy.get_param('/vesc/motor_timeout')

        # for servo position commands
        self.max_servo_speed = rospy.get_param('/vesc/max_servo_speed')
        self.steering_angle_to_servo_gain = rospy.get_param('/vesc/steering_angle_to_servo_gain')
        self.servo_smoother_rate = rospy.get_param('/vesc/servo_smoother_rate')
        self.max_servo = rospy.get_param('/vesc/vesc_driver/servo_max')
        self.min_servo = rospy.get_param('/vesc/vesc_driver/servo_min')
        self.servo_timeout = rospy.get_param('/vesc/servo_timeout')

        # for motor current commands
        self.max_accel_current = rospy.get_param('/vesc/max_accel_current')
        self.max_brake_current = rospy.get_param('/vesc/max_brake_current') # >0
        current_time_constant = rospy.get_param('/vesc/current_time_constant')
        self.current_smoother_rate = rospy.get_param('/vesc/current_smoother_rate')
        self.current_timeout = rospy.get_param('/vesc/current_timeout')
        # premultiply time constant with rate to get filter length that we actually need:
        self.current_filter_length =  1 / (self.current_smoother_rate * current_time_constant)

        # Variables
        self.last_rpm = 0
        self.desired_rpm = self.last_rpm
        
        self.last_servo = rospy.get_param('/vesc/steering_angle_to_servo_offset')
        self.desired_servo_position = self.last_servo

        self.last_current = 0
        self.desired_current = self.last_current

        self.slip_msg = Float64()
        self.scaler_msg = Float64()

        # Create topic subscribers and publishers
        self.rpm_output = rospy.Publisher(self.rpm_output_topic, Float64,queue_size=1)
        self.servo_output = rospy.Publisher(self.servo_output_topic, Float64,queue_size=1)
        self.current_output = rospy.Publisher(self.current_output_topic, Float64,queue_size=1)
        self.brake_output = rospy.Publisher(self.brake_output_topic, Float64,queue_size=1)
        
        rospy.Subscriber(self.rpm_input_topic, Float64, self._process_throttle_command)
        rospy.Subscriber(self.servo_input_topic, Float64, self._process_servo_command)
        rospy.Subscriber(self.current_input_topic, Float64, self._process_current_command)
        rospy.Subscriber(self.brake_input_topic, Float64, self._process_brake_command)

        self.publish_throttle = False
        self.t_last_throttle = rospy.Time.now()
        self.publish_servo = False
        self.t_last_servo = rospy.Time.now()
        self.publish_current = False
        self.t_last_current = rospy.Time.now()
        self.publish_brake = False
        self.t_last_brake = rospy.Time.now()

        #Traction control
        # subscribe to vesc data, control command
        rospy.Subscriber('/vesc/sensors/core', VescStateStamped, self.vesc_cb)
        rospy.Subscriber('/vesc/odom', Odometry, self.vesc_odom_cb)
        self.slip_pub = rospy.Publisher('/vesc/traction/slip', Float64, queue_size=1)
        self.slip_scaler_pub = rospy.Publisher('/vesc/traction/throttle_scaler', Float64, queue_size=1)
        self.prev_speed = 0
        self.speed = 0 
        self.prev_current_motor = 0
        self.current_motor = 0
        self.scaler = 1
        self.downscaler = 1 # TODO rename for clarity
        
        # Wait for dynamic params of traction controller
        while not rospy.get_param('/vesc/dynamic_traction_tuner_node/current_threshold', False):
            rospy.loginfo_throttle(1, "Waiting for dynamic traction control params...")

        self.max_delta_servo = abs(self.steering_angle_to_servo_gain * self.max_servo_speed / self.servo_smoother_rate)
        rospy.Timer(rospy.Duration(1.0/self.servo_smoother_rate), self._publish_servo_command)

        self.max_delta_rpm = abs(self.speed_to_erpm_gain * self.max_acceleration / self.throttle_smoother_rate)
        rospy.Timer(rospy.Duration(1.0/self.max_delta_rpm), self._publish_throttle_command)

        # handles breaking and acceleration
        rospy.Timer(rospy.Duration(1.0/self.current_smoother_rate), self._publish_current_command)
        
        # run the node
        self._run()

        # Keep the node alive
    def _run(self):
        rospy.spin()

    def _calc_traction_speed_scaler(self):
        # if current is decreasing and velocity is not decreasing
        d_vel = self.speed - self.prev_speed
        d_cur = self.prev_current_motor - self.current_motor
        slip_bool = False
        if (d_vel >= self.vel_th) and (d_cur <= -self.cur_th):
            slip_bool = True
            #print('SLIP DETECTED!!!')
            # kill the speed
            self.scaler *= self.downscaler 
        else:
            # else reset the killer
            self.scaler = min(self.scaler/self.downscaler, 1)

        #Publish scalers and slip detection for rosbags
        self.slip_msg.data = 1.0 if slip_bool else 0.0
        self.scaler_msg.data = self.scaler
        self.slip_pub.publish(self.slip_msg)
        self.slip_scaler_pub.publish(self.scaler_msg)

        return self.scaler

    def _publish_throttle_command(self, evt):
        # fetch traction control params
        self.cur_th = rospy.get_param('/vesc/dynamic_traction_tuner_node/current_threshold')
        self.downscaler = rospy.get_param('/vesc/dynamic_traction_tuner_node/downscaler')
        self.vel_th = rospy.get_param('/vesc/dynamic_traction_tuner_node/velocity_threshold')

        if self.publish_throttle:
            # timeout avoids continuing to publish if no commands come in
            if (rospy.Time.now() - self.t_last_throttle).to_sec() > self.motor_timeout:
                self.publish_throttle = False 
                self.last_rpm = 0
                self.desired_rpm = self.last_rpm
                return
            desired_delta = self.desired_rpm-self.last_rpm
            clipped_delta = max(min(desired_delta, self.max_delta_rpm), -self.max_delta_rpm)
            smoothed_rpm = self.last_rpm + clipped_delta
            self.last_rpm = smoothed_rpm         
            
            #Calculate traction scaler for speed 
            scaler = self._calc_traction_speed_scaler()
            smoothed_rpm *= scaler

            self.rpm_output.publish(Float64(smoothed_rpm))
            
    def _process_throttle_command(self,msg):
        input_rpm = msg.data
        # Do some sanity clipping
        input_rpm = min(max(input_rpm, self.min_rpm), self.max_rpm)
        self.desired_rpm = input_rpm
        self.publish_throttle = True
        self.t_last_throttle = rospy.Time.now()
        # reset other motor commands
        self.publish_current = False
        self.publish_brake = False
        self.last_current = 0

    def _publish_servo_command(self, evt):
        if self.publish_servo:
            # timeout avoids continuing to publish if no commands come in
            if (rospy.Time.now() - self.t_last_servo).to_sec() > self.servo_timeout:
                self.publish_servo = False 
                self.last_servo = rospy.get_param('/vesc/steering_angle_to_servo_offset')
                self.desired_servo_position = self.last_servo
                return
            desired_delta = self.desired_servo_position-self.last_servo
            clipped_delta = max(min(desired_delta, self.max_delta_servo), -self.max_delta_servo)
            smoothed_servo = self.last_servo + clipped_delta
            self.last_servo = smoothed_servo         
            self.servo_output.publish(Float64(smoothed_servo))

    def _process_servo_command(self,msg):
        input_servo = msg.data
        # Do some sanity clipping
        input_servo = min(max(input_servo, self.min_servo), self.max_servo)
        # set the target servo position
        self.desired_servo_position = input_servo
        self.publish_servo = True
        self.t_last_servo = rospy.Time.now()

    def _publish_current_command(self, evt):
        if self.publish_brake:
            # timeout avoids continuing to publish if no commands come in
            if (rospy.Time.now() - self.t_last_brake).to_sec() > self.current_timeout:
                self.publish_brake = False 
                self.last_current = 0
                self.desired_current = self.last_current
                return
            desired_delta = self.desired_current-self.last_current
            smoothed_current = self.last_current + desired_delta*self.current_filter_length
            self.last_current = smoothed_current         
            self.brake_output.publish(Float64((-1)*smoothed_current))
        elif self.publish_current:
            # timeout avoids continuing to publish if no commands come in
            if (rospy.Time.now() - self.t_last_current).to_sec() > self.current_timeout:
                self.publish_current = False 
                self.last_current = 0
                self.desired_current = self.last_current
                return
            desired_delta = self.desired_current-self.last_current
            smoothed_current = self.last_current + desired_delta*self.current_filter_length
            self.last_current = smoothed_current         
            self.current_output.publish(Float64(smoothed_current))

    def _process_current_command(self,msg):
        self.desired_current = max(min(msg.data, self.max_accel_current), 0)
        self.publish_current = True
        self.t_last_current = rospy.Time.now()
        # reset other motor commands
        if self.publish_brake:
            self.last_current = 0
            self.publish_brake = False
        self.publish_throttle = False
        self.last_rpm = 0
    
    def _process_brake_command(self,msg):
        # brake current is positive
        self.desired_current = max(min(msg.data, self.max_brake_current), 0)
        self.publish_brake = True
        self.t_last_brake = rospy.Time.now()
        # reset other motor commands
        if self.publish_current:
            self.last_current = 0
            self.publish_current = False
        self.publish_throttle = False
        self.last_rpm = 0

    def vesc_odom_cb(self, odom:Odometry):
        self.prev_speed = self.speed
        self.speed = odom.twist.twist.linear.x

    def vesc_cb(self, data:VescStateStamped):
        # shift old data
        self.prev_current_motor = self.current_motor

        # get data
        self.current_motor = data.state.current_motor # motor current (Ampere)

# Boilerplate node spin up. 
if __name__ == '__main__':
    try:
        rospy.init_node('Throttle_Interpolator')
        p = InterpolateThrottle()
    except rospy.ROSInterruptException:
        pass

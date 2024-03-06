#!/usr/bin/env python3
#node for recording rosbags. Node runs on NUC on drone while gui runs on flight book

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64

class IDController:
  
  def __init__(self):
    #subscribers
    # rospy.Subscriber("/vicon/F110rosbag_record", Record , self.GUI_cb, queue_size=10)
    
    #publishers
    self.cmd_pub = rospy.Publisher('drive_topic', AckermannDriveStamped, queue_size=10)
    # create publishers to vesc electric-RPM (speed) and servo commands (bridge ackermann to vesc node and directly talk to vesc driver)
    self.erpm_pub = rospy.Publisher("/vesc/commands/motor/speed", Float64, queue_size=10)
    self.servo_pub = rospy.Publisher("/vesc/commands/servo/position", Float64, queue_size=10)
    self.current_pub = rospy.Publisher("/vesc/commands/motor/current", Float64, queue_size=10)
    self.brake_pub = rospy.Publisher("/vesc/commands/motor/brake", Float64, queue_size=10)

    # parameters
    self.experiment = rospy.get_param("/id_controller/experiment")
    
    # 1 - send const vesc command
    self.const_erpm = rospy.get_param("/id_controller/const_erpm")
    self.const_servo = rospy.get_param("/id_controller/const_servo")

    # 2 - accelerate then decelerate via current commands
    self.const_curr = rospy.get_param("/id_controller/const_curr")
    self.const_brake = rospy.get_param("/id_controller/const_brake")
    self.accel_time = rospy.get_param("/id_controller/accel_time")
    self.decel_time = rospy.get_param("/id_controller/decel_time")

    # 3 - accelerate then decelerate via acceleration commands
    self.const_accel = rospy.get_param("/id_controller/const_accel")
    self.const_decel = rospy.get_param("/id_controller/const_decel")

    # 4 - drive with const erpm and increase servo position
    self.angle_time = rospy.get_param("/id_controller/angle_time")
    self.start_pos =  rospy.get_param("/id_controller/start_pos")
    self.end_pos =  rospy.get_param("/id_controller/end_pos")

    #5 drive with const motor speed and increase steering angle uses angle time from above
    self.start_angle = rospy.get_param("/id_controller/start_angle")
    self.end_angle = rospy.get_param("/id_controller/end_angle")
    self.const_speed = rospy.get_param("/id_controller/const_speed")

    #6 follow given acceleration profile and record IMU data
    self.acc_profile = rospy.get_param("id_controller/acc_profile")

    # 7 bang bang control on the servo with constant speed
    #self.period = rospy.get_param("/id_controller/bangbang_period")
    #self.repetitions = rospy.get_param("/id_controller/repetitions")
    #self.bangbang_steer = rospy.get_param("/id_controller/bangbang_steer")

    rospy.logwarn("Starting experiment #" + str(self.experiment))

    rate = rospy.Rate(30)
    self.start_time = rospy.get_time()
    while not rospy.is_shutdown():
      if self.experiment == 1:
        self.send_const_vesc_cmd()
      elif self.experiment == 2:
        self.accel_decel()
      elif self.experiment == 3:
        self.drive_accel_decel()
      elif self.experiment == 4:
        self.increase_servo_position()
      elif self.experiment == 5:
        self.increase_steering_angle()
      elif self.experiment == 6:
        self.acc_profile_recorder()
      elif self.experiment == 7:
        self.bang_bang_servo()
      rate.sleep()
  
  def send_const_vesc_cmd(self):
    erpm_msg = Float64(self.const_erpm)
    servo_msg = Float64(self.const_servo)
    self.erpm_pub.publish(erpm_msg)
    self.servo_pub.publish(servo_msg)

  def accel_decel(self):
    servo_msg = Float64(self.const_servo)
    self.servo_pub.publish(servo_msg)
    if (rospy.get_time() - self.start_time < self.accel_time):
      current_msg = Float64(self.const_curr)
      self.current_pub.publish(current_msg)
      rospy.loginfo("accelerating")
    elif (rospy.get_time() - self.start_time < self.accel_time + self.decel_time):
      brake_msg = Float64(self.const_brake)
      self.brake_pub.publish(brake_msg)
      rospy.loginfo("decelerating")

  def drive_accel_decel(self):
      drive_msg = AckermannDriveStamped()
      drive_msg.header.stamp = rospy.Time.now()
      drive_msg.drive.steering_angle = 0
      drive_msg.drive.steering_angle_velocity = 0
      drive_msg.drive.speed = 0
      drive_msg.drive.jerk = 512 # used a flag to indicate the ackermann controller to use acceleration instead of speed
      
      if (rospy.get_time() - self.start_time < 3): #wait for 3s
        rospy.loginfo_once("experiment starting")
      elif (rospy.get_time() - self.start_time < self.accel_time + 3):
            drive_msg.drive.acceleration = self.const_accel
            rospy.loginfo("accelerating")
            self.cmd_pub.publish(drive_msg)
      elif (rospy.get_time() - self.start_time < self.accel_time + self.decel_time + 3):
        drive_msg.drive.acceleration = self.const_decel
        self.cmd_pub.publish(drive_msg)
        rospy.loginfo("decelerating")
      
      else:
        rospy.loginfo_once("experiment over")


  def increase_servo_position(self):
    time_frac = (rospy.get_time() - self.start_time)/self.angle_time
    if time_frac <= 1:
      angle = self.end_pos * time_frac + self.start_pos * (1 - time_frac)
      servo_msg = Float64(angle)
      erpm_msg = Float64(self.const_erpm)
      self.erpm_pub.publish(erpm_msg)
      self.servo_pub.publish(servo_msg)
    else:
      rospy.loginfo_once("experiment over")

  def increase_steering_angle(self):
    time_frac = (rospy.get_time() - self.start_time)/self.angle_time
    if time_frac <= 1:
      angle = self.end_angle * time_frac + self.start_angle * (1 - time_frac)
      drive_msg = AckermannDriveStamped()
      drive_msg.header.stamp = rospy.Time.now()
      drive_msg.drive.steering_angle = angle
      drive_msg.drive.steering_angle_velocity = 0
      drive_msg.drive.speed = self.const_speed
      drive_msg.drive.acceleration = 0
      drive_msg.drive.jerk = 0
      self.cmd_pub.publish(drive_msg)
      if time_frac >= 0.9:
        rospy.logwarn("Ending soon, stop the bag")
    else:
      rospy.loginfo_once("experiment over")

  def acc_profile_recorder(self):
    drive_msg = AckermannDriveStamped()
    drive_msg.header.stamp = rospy.Time.now()
    drive_msg.drive.steering_angle = 0
    drive_msg.drive.steering_angle_velocity = 0

    if self.acc_profile == 1: # Step response to speed
      speed = 3
      duration = 4
      if (rospy.get_time() - self.start_time < 2): #stand still for 2 seconds
        drive_msg.drive.speed = 0
        drive_msg.drive.jerk = 0
        drive_msg.drive.acceleration = 0
        self.cmd_pub.publish(drive_msg)
        rospy.loginfo_once("experiment starting")
      elif (rospy.get_time() - self.start_time > duration + 2): # stand still again after experiment duration
        drive_msg.drive.speed = 0
        drive_msg.drive.jerk = 0
        drive_msg.drive.acceleration = 0
        self.cmd_pub.publish(drive_msg)
        rospy.loginfo_once("experiment over")
      else: # step input to 3 m/s after 2 seconds have passed
        drive_msg.drive.speed = speed
        drive_msg.drive.jerk = 0
        drive_msg.drive.acceleration = 0
        self.cmd_pub.publish(drive_msg)
        rospy.loginfo("accelerating")
    elif self.acc_profile == 2: # Max acc and braking test
      drive_msg = AckermannDriveStamped()
      drive_msg.header.stamp = rospy.Time.now()
      drive_msg.drive.steering_angle = 0
      drive_msg.drive.steering_angle_velocity = 0
      drive_msg.drive.speed = 0
      drive_msg.drive.jerk = 512
      if (rospy.get_time() - self.start_time < self.accel_time):
        drive_msg.drive.acceleration = 100
        rospy.loginfo("accelerating")
        self.cmd_pub.publish(drive_msg)
      elif (rospy.get_time() - self.start_time < self.accel_time + self.decel_time):
        drive_msg.drive.acceleration = -100
        self.cmd_pub.publish(drive_msg)
        rospy.loginfo("decelerating")
      else:
        rospy.loginfo_once("experiment over")
    elif self.acc_profile == 3: # speed steps at speed
      speed1 = 3
      speed2 = 5
      duration = 6
      if (rospy.get_time() - self.start_time < 2 or rospy.get_time() - self.start_time > 2 + duration): #stand still for 2 seconds
        drive_msg.drive.speed = 0
        drive_msg.drive.jerk = 0
        drive_msg.drive.acceleration = 0
        self.cmd_pub.publish(drive_msg)
        rospy.loginfo_once("standing still")    
      elif (rospy.get_time() - self.start_time > duration/3 + 2 and rospy.get_time() - self.start_time < duration/3*2 + 2): #accelerate to speed 2
        drive_msg.drive.speed = speed2
        drive_msg.drive.jerk = 0
        drive_msg.drive.acceleration = 0
        self.cmd_pub.publish(drive_msg)
        rospy.loginfo("accelerating further")
      elif (rospy.get_time() - self.start_time > duration/3*2 + 2 and rospy.get_time() - self.start_time < duration + 2): #decelerate to speed 1 again
        drive_msg.drive.speed = speed2
        drive_msg.drive.jerk = 0
        drive_msg.drive.acceleration = 0
        self.cmd_pub.publish(drive_msg)
        rospy.loginfo("decelerating")
      elif (rospy.get_time() - self.start_time > duration + 2): # stand still again after experiment duration
        drive_msg.drive.speed = 0
        drive_msg.drive.jerk = 0
        drive_msg.drive.acceleration = 0
        self.cmd_pub.publish(drive_msg)
        rospy.loginfo_once("experiment over")  
      else: # step input to speed 1 after 2 seconds have passed
        drive_msg.drive.speed = speed1
        drive_msg.drive.jerk = 0
        drive_msg.drive.acceleration = 0
        self.cmd_pub.publish(drive_msg)
        rospy.loginfo("accelerating")
    else:
      rospy.loginfo("invalid speed profile")
      drive_msg.drive.speed = 1
      drive_msg.drive.jerk = 512
      if (rospy.get_time() - self.start_time < self.accel_time):
        drive_msg.drive.acceleration = self.const_accel
        rospy.loginfo("accelerating")
        self.cmd_pub.publish(drive_msg)
      elif (rospy.get_time() - self.start_time < self.accel_time + self.decel_time):
        drive_msg.drive.acceleration = self.const_decel
        self.cmd_pub.publish(drive_msg)
        rospy.loginfo("decelerating")
      else:
        rospy.loginfo_once("experiment over")
    

  def bang_bang_servo(self):
    time_frac = (rospy.get_time() - self.start_time)/self.period

    steer_sign = - 1
    if time_frac%1 < 0.5:
      steer_sign = 1

    if time_frac <= 1*self.repetitions:
      angle = steer_sign*self.bangbang_steer
      
      drive_msg = AckermannDriveStamped()
      drive_msg.header.stamp = rospy.Time.now()
      drive_msg.drive.steering_angle = angle
      drive_msg.drive.steering_angle_velocity = 0
      drive_msg.drive.speed = self.const_speed
      drive_msg.drive.acceleration = 0
      drive_msg.drive.jerk = 0
      self.cmd_pub.publish(drive_msg)
    else:
      rospy.loginfo_once("experiment over")


if __name__ == '__main__':
  rospy.init_node('id_controller')
  rospy.loginfo('id Controller is running')

  # Go to class functions that do all the heavy lifting. Do error checking.
  try:
    IDController = IDController()
  except rospy.ROSInterruptException:
    pass

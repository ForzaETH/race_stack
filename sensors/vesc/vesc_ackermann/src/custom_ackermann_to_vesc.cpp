// Copyright 2020 F1TENTH Foundation
//
// Redistribution and use in source and binary forms, with or without modification, are permitted
// provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this list of conditions
//    and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice, this list
//    of conditions and the following disclaimer in the documentation and/or other materials
//    provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors may be used
//    to endorse or promote products derived from this software without specific prior
//    written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
// IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
// FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
// WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// -*- mode:c++; fill-column: 100; -*-

#include "vesc_ackermann/custom_ackermann_to_vesc.h"

#include <cmath>
#include <sstream>
#include <string>

#include <std_msgs/Float64.h>

namespace vesc_ackermann
{

template <typename T>
inline bool getRequiredParam(const ros::NodeHandle& nh, const std::string& name, T* value);

AckermannToVesc::AckermannToVesc(ros::NodeHandle nh, ros::NodeHandle private_nh)
{
  // get conversion parameters
  if (!getRequiredParam(nh, "speed_to_erpm_gain", &speed_to_erpm_gain_))
    return;
  if (!getRequiredParam(nh, "speed_to_erpm_offset", &speed_to_erpm_offset_))
    return;
  if (!getRequiredParam(nh, "steering_angle_to_servo_gain", &steering_to_servo_gain_))
    return;
  if (!getRequiredParam(nh, "steering_angle_to_servo_offset", &steering_to_servo_offset_))
    return;
  if (!getRequiredParam(nh, "acceleration_to_current_gain", &acceleration_to_current_gain_))
    return;
  if (!getRequiredParam(nh, "deceleration_to_current_gain", &deceleration_to_current_gain_))
    return;
  if (!getRequiredParam(nh, "velocity_to_current_gain", &velocity_to_current_gain_))
    return;

  // create publishers to vesc electric-RPM (speed) and servo commands
  erpm_pub_ = nh.advertise<std_msgs::Float64>("commands/motor/speed", 10);
  current_pub_ = nh.advertise<std_msgs::Float64>("commands/motor/current", 10);
  brake_pub_ = nh.advertise<std_msgs::Float64>("commands/motor/brake", 10);
  servo_pub_ = nh.advertise<std_msgs::Float64>("commands/servo/position", 10);

  // subscribe to ackermann topic
  ackermann_sub_ = nh.subscribe("ackermann_cmd", 10, &AckermannToVesc::ackermannCmdCallback, this);

  // subscribe to odometry topic
  odom_sub_ = nh.subscribe("odom", 10, &AckermannToVesc::odomCallback, this);
  linear_velocity_x_ = 0;
}

typedef nav_msgs::Odometry::ConstPtr OdomMsgPtr;
void AckermannToVesc::odomCallback(const OdomMsgPtr& msg)
{
  linear_velocity_x_ = msg->twist.twist.linear.x;

  // add deadzone in order to not react to noise in odom:
  if ( abs(linear_velocity_x_) < 0.3 ) {
    linear_velocity_x_ = 0;
  }
}

typedef ackermann_msgs::AckermannDriveStamped::ConstPtr AckermannMsgPtr;
void AckermannToVesc::ackermannCmdCallback(const AckermannMsgPtr& cmd)
{
  // calc steering angle (servo)
  std_msgs::Float64::Ptr servo_msg(new std_msgs::Float64);
  servo_msg->data = steering_to_servo_gain_ * cmd->drive.steering_angle + steering_to_servo_offset_;

  // abuse jerk to indicate whether we should follow acceleration commands or not 
  // (hacky but avoids needing new message type - TODO replace this by bitmask in custom ackermann message)
  if (cmd->drive.jerk == 512) {
    // interpret acceleration as commands and map them to current commands to the motor (directly)
    if (cmd->drive.acceleration >= 0) {
      // calc current command (motor)
      std_msgs::Float64::Ptr current_msg(new std_msgs::Float64);
      current_msg->data = acceleration_to_current_gain_ * cmd->drive.acceleration
          + velocity_to_current_gain_ * linear_velocity_x_;

      // publish
      if (ros::ok()) {
        current_pub_.publish(current_msg);
        servo_pub_.publish(servo_msg);
      }
    } else {
      // calc brake command (motor)
      std_msgs::Float64::Ptr brake_msg(new std_msgs::Float64);
      brake_msg->data = -(deceleration_to_current_gain_ * cmd->drive.acceleration
        + velocity_to_current_gain_ * linear_velocity_x_);

      // publish
      if (ros::ok()) {
        brake_pub_.publish(brake_msg);
        servo_pub_.publish(servo_msg);
      }
    }
  } else {
     // calc vesc electric RPM (speed)
    std_msgs::Float64::Ptr erpm_msg(new std_msgs::Float64);
    erpm_msg->data = speed_to_erpm_gain_ * cmd->drive.speed + speed_to_erpm_offset_;

    // publish
    if (ros::ok())
    {
      erpm_pub_.publish(erpm_msg);
      servo_pub_.publish(servo_msg);
    }
  }
}

template <typename T>
inline bool getRequiredParam(const ros::NodeHandle& nh, const std::string& name, T* value)
{
  if (nh.getParam(name, *value))
    return true;

  ROS_FATAL("AckermannToVesc: Parameter %s is required.", name.c_str());
  return false;
}

}  // namespace vesc_ackermann

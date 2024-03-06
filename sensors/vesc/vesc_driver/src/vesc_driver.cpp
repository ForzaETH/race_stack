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

#include "vesc_driver/vesc_driver.h"

#include <cassert>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>

#include <vesc_msgs/VescStateStamped.h>
#include <vesc_msgs/VescImuStamped.h>


namespace vesc_driver
{

using std::placeholders::_1;
using sensor_msgs::Imu;

VescDriver::VescDriver(ros::NodeHandle nh,
                       ros::NodeHandle private_nh) :
  vesc_(std::string(),
        std::bind(&VescDriver::vescPacketCallback, this, _1),
        std::bind(&VescDriver::vescErrorCallback, this, _1)),
  duty_cycle_limit_(private_nh, "duty_cycle", -1.0, 1.0), current_limit_(private_nh, "current"),
  brake_limit_(private_nh, "brake"), speed_limit_(private_nh, "speed"),
  position_limit_(private_nh, "position"), servo_limit_(private_nh, "servo", 0.0, 1.0),
  driver_mode_(MODE_INITIALIZING), fw_version_major_(-1), fw_version_minor_(-1)
{
  // get vesc serial port address
  std::string port;
  if (!private_nh.getParam("port", port))
  {
    ROS_FATAL("VESC communication port parameter required.");
    ros::shutdown();
    return;
  }

  // attempt to connect to the serial port
  try
  {
    vesc_.connect(port);
  }
  catch (SerialException e)
  {
    ROS_FATAL("Failed to connect to the VESC, %s.", e.what());
    ros::shutdown();
    return;
  }

  // create vesc state (telemetry) publisher
  state_pub_ = nh.advertise<vesc_msgs::VescStateStamped>("sensors/core", 10);
  imu_pub_ = nh.advertise<vesc_msgs::VescImuStamped>("sensors/imu", 10);
  imu_std_pub_ = nh.advertise<Imu>("sensors/imu/raw", 10);

  // since vesc state does not include the servo position, publish the commanded
  // servo position as a "sensor"
  servo_sensor_pub_ = nh.advertise<std_msgs::Float64>("sensors/servo_position_command", 10);

  // subscribe to motor and servo command topics
  duty_cycle_sub_ = nh.subscribe("commands/motor/duty_cycle", 10,
                                 &VescDriver::dutyCycleCallback, this);
  current_sub_ = nh.subscribe("commands/motor/current", 10, &VescDriver::currentCallback, this);
  brake_sub_ = nh.subscribe("commands/motor/brake", 10, &VescDriver::brakeCallback, this);
  speed_sub_ = nh.subscribe("commands/motor/speed", 10, &VescDriver::speedCallback, this);
  position_sub_ = nh.subscribe("commands/motor/position", 10, &VescDriver::positionCallback, this);
  servo_sub_ = nh.subscribe("commands/servo/position", 10, &VescDriver::servoCallback, this);

  // create a 50Hz timer, used for state machine & polling VESC telemetry
  timer_ = nh.createTimer(ros::Duration(1.0 / 50.0), &VescDriver::timerCallback, this);
}

/* TODO or TO-THINKABOUT LIST
  - what should we do on startup? send brake or zero command?
  - what to do if the vesc interface gives an error?
  - check version number against know compatable?
  - should we wait until we receive telemetry before sending commands?
  - should we track the last motor command
  - what to do if no motor command received recently?
  - what to do if no servo command received recently?
  - what is the motor safe off state (0 current?)
  - what to do if a command parameter is out of range, ignore?
  - try to predict vesc bounds (from vesc config) and command detect bounds errors
*/

void VescDriver::timerCallback(const ros::TimerEvent& event)
{
  // VESC interface should not unexpectedly disconnect, but test for it anyway
  if (!vesc_.isConnected())
  {
    ROS_FATAL("Unexpectedly disconnected from serial port.");
    timer_.stop();
    ros::shutdown();
    return;
  }

  /*
   * Driver state machine, modes:
   *  INITIALIZING - request and wait for vesc version
   *  OPERATING - receiving commands from subscriber topics
   */
  if (driver_mode_ == MODE_INITIALIZING)
  {
    // request version number, return packet will update the internal version numbers
    vesc_.requestFWVersion();
    if (fw_version_major_ >= 0 && fw_version_minor_ >= 0)
    {
      ROS_INFO("Connected to VESC with firmware version %d.%d",
               fw_version_major_, fw_version_minor_);
      driver_mode_ = MODE_OPERATING;
    }
  }
  else if (driver_mode_ == MODE_OPERATING)
  {
    // poll for vesc state (telemetry)
    vesc_.requestState();
    // poll for vesc imu
    vesc_.requestImuData();
  }
  else
  {
    // unknown mode, how did that happen?
    assert(false && "unknown driver mode");
  }
}

void VescDriver::vescPacketCallback(const std::shared_ptr<VescPacket const>& packet)
{
  if (packet->name() == "Values")
  {
    std::shared_ptr<VescPacketValues const> values =
      std::dynamic_pointer_cast<VescPacketValues const>(packet);

    vesc_msgs::VescStateStamped::Ptr state_msg(new vesc_msgs::VescStateStamped);
    state_msg->header.stamp = ros::Time::now();
    state_msg->state.voltage_input = values->v_in();
    state_msg->state.temperature_pcb = values->temp_pcb();
    state_msg->state.current_motor = values->current_motor();
    state_msg->state.current_input = values->current_in();
    state_msg->state.speed = values->rpm();
    state_msg->state.duty_cycle = values->duty_now();
    state_msg->state.charge_drawn = values->amp_hours();
    state_msg->state.charge_regen = values->amp_hours_charged();
    state_msg->state.energy_drawn = values->watt_hours();
    state_msg->state.energy_regen = values->watt_hours_charged();
    state_msg->state.displacement = values->tachometer();
    state_msg->state.distance_traveled = values->tachometer_abs();
    state_msg->state.fault_code = values->fault_code();

    state_pub_.publish(state_msg);
  }
  else if (packet->name() == "FWVersion")
  {
    std::shared_ptr<VescPacketFWVersion const> fw_version =
      std::dynamic_pointer_cast<VescPacketFWVersion const>(packet);
    // todo: might need lock here
    fw_version_major_ = fw_version->fwMajor();
    fw_version_minor_ = fw_version->fwMinor();
  }
  else if (packet->name() == "ImuData") {
    std::shared_ptr<VescPacketImu const> imuData = std::dynamic_pointer_cast<VescPacketImu const>(packet);

    vesc_msgs::VescImuStamped::Ptr imu_msg(new vesc_msgs::VescImuStamped);
    imu_msg->header.stamp = ros::Time::now();
    imu_msg->header.frame_id = "imu";
    sensor_msgs::Imu::Ptr std_imu_msg(new Imu);
    std_imu_msg->header.stamp = ros::Time::now();
    std_imu_msg->header.frame_id = "imu";

    imu_msg->imu.ypr.x = imuData->roll();
    imu_msg->imu.ypr.y = imuData->pitch();
    imu_msg->imu.ypr.z = imuData->yaw();

    imu_msg->imu.linear_acceleration.x = imuData->acc_x();
    imu_msg->imu.linear_acceleration.y = imuData->acc_y();
    imu_msg->imu.linear_acceleration.z = imuData->acc_z();

    imu_msg->imu.angular_velocity.x = imuData->gyr_x();
    imu_msg->imu.angular_velocity.y = imuData->gyr_y();
    imu_msg->imu.angular_velocity.z = imuData->gyr_z();

    imu_msg->imu.compass.x = imuData->mag_x();
    imu_msg->imu.compass.y = imuData->mag_y();
    imu_msg->imu.compass.z = imuData->mag_z();

    imu_msg->imu.orientation.w = imuData->q_y();
    imu_msg->imu.orientation.x = imuData->q_x();
    imu_msg->imu.orientation.y = imuData->q_z();
    imu_msg->imu.orientation.z = imuData->q_w();

    std_imu_msg->linear_acceleration.x = 9.806 * imuData->acc_x();
    std_imu_msg->linear_acceleration.y = 9.806 * imuData->acc_y();
    std_imu_msg->linear_acceleration.z = 9.806 * imuData->acc_z();

    std_imu_msg->angular_velocity.x = M_PI / 180 * imuData->gyr_x();
    std_imu_msg->angular_velocity.y = M_PI / 180 * imuData->gyr_y();
    std_imu_msg->angular_velocity.z = M_PI / 180 * imuData->gyr_z();

    std_imu_msg->orientation.w = imuData->q_w();
    std_imu_msg->orientation.x = imuData->q_x();
    std_imu_msg->orientation.y = imuData->q_y();
    std_imu_msg->orientation.z = imuData->q_z();
    
    imu_pub_.publish(imu_msg);
    imu_std_pub_.publish(std_imu_msg);
  }
}

void VescDriver::vescErrorCallback(const std::string& error)
{
  ROS_ERROR("%s", error.c_str());
}

/**
 * @param duty_cycle Commanded VESC duty cycle. Valid range for this driver is -1 to +1. However,
 *                   note that the VESC may impose a more restrictive bounds on the range depending
 *                   on its configuration, e.g. absolute value is between 0.05 and 0.95.
 */
void VescDriver::dutyCycleCallback(const std_msgs::Float64::ConstPtr& duty_cycle)
{
  if (driver_mode_ = MODE_OPERATING)
  {
    vesc_.setDutyCycle(duty_cycle_limit_.clip(duty_cycle->data));
  }
}

/**
 * @param current Commanded VESC current in Amps. Any value is accepted by this driver. However,
 *                note that the VESC may impose a more restrictive bounds on the range depending on
 *                its configuration.
 */
void VescDriver::currentCallback(const std_msgs::Float64::ConstPtr& current)
{
  if (driver_mode_ = MODE_OPERATING)
  {
    vesc_.setCurrent(current_limit_.clip(current->data));
  }
}

/**
 * @param brake Commanded VESC braking current in Amps. Any value is accepted by this driver.
 *              However, note that the VESC may impose a more restrictive bounds on the range
 *              depending on its configuration.
 */
void VescDriver::brakeCallback(const std_msgs::Float64::ConstPtr& brake)
{
  if (driver_mode_ = MODE_OPERATING)
  {
    vesc_.setBrake(brake_limit_.clip(brake->data));
  }
}

/**
 * @param speed Commanded VESC speed in electrical RPM. Electrical RPM is the mechanical RPM
 *              multiplied by the number of motor poles. Any value is accepted by this
 *              driver. However, note that the VESC may impose a more restrictive bounds on the
 *              range depending on its configuration.
 */
void VescDriver::speedCallback(const std_msgs::Float64::ConstPtr& speed)
{
  if (driver_mode_ = MODE_OPERATING)
  {
    vesc_.setSpeed(speed_limit_.clip(speed->data));
  }
}

/**
 * @param position Commanded VESC motor position in radians. Any value is accepted by this driver.
 *                 Note that the VESC must be in encoder mode for this command to have an effect.
 */
void VescDriver::positionCallback(const std_msgs::Float64::ConstPtr& position)
{
  if (driver_mode_ = MODE_OPERATING)
  {
    // ROS uses radians but VESC seems to use degrees. Convert to degrees.
    double position_deg = position_limit_.clip(position->data) * 180.0 / M_PI;
    vesc_.setPosition(position_deg);
  }
}

/**
 * @param servo Commanded VESC servo output position. Valid range is 0 to 1.
 */
void VescDriver::servoCallback(const std_msgs::Float64::ConstPtr& servo)
{
  if (driver_mode_ = MODE_OPERATING)
  {
    double servo_clipped(servo_limit_.clip(servo->data));
    vesc_.setServo(servo_clipped);
    // publish clipped servo value as a "sensor"
    std_msgs::Float64::Ptr servo_sensor_msg(new std_msgs::Float64);
    servo_sensor_msg->data = servo_clipped;
    servo_sensor_pub_.publish(servo_sensor_msg);
  }
}

VescDriver::CommandLimit::CommandLimit(const ros::NodeHandle& nh, const std::string& str,
                                       const boost::optional<double>& min_lower,
                                       const boost::optional<double>& max_upper) :
  name(str)
{
  // check if user's minimum value is outside of the range min_lower to max_upper
  double param_min;
  if (nh.getParam(name + "_min", param_min))
  {
    if (min_lower && param_min < *min_lower)
    {
      lower = *min_lower;
      ROS_WARN_STREAM("Parameter " << name << "_min (" << param_min <<
                      ") is less than the feasible minimum (" << *min_lower << ").");
    }
    else if (max_upper && param_min > *max_upper)
    {
      lower = *max_upper;
      ROS_WARN_STREAM("Parameter " << name << "_min (" << param_min <<
                      ") is greater than the feasible maximum (" << *max_upper << ").");
    }
    else
    {
      lower = param_min;
    }
  }
  else if (min_lower)
  {
    lower = *min_lower;
  }

  // check if the uers' maximum value is outside of the range min_lower to max_upper
  double param_max;
  if (nh.getParam(name + "_max", param_max))
  {
    if (min_lower && param_max < *min_lower)
    {
      upper = *min_lower;
      ROS_WARN_STREAM("Parameter " << name << "_max (" << param_max <<
                      ") is less than the feasible minimum (" << *min_lower << ").");
    }
    else if (max_upper && param_max > *max_upper)
    {
      upper = *max_upper;
      ROS_WARN_STREAM("Parameter " << name << "_max (" << param_max <<
                      ") is greater than the feasible maximum (" << *max_upper << ").");
    }
    else
    {
      upper = param_max;
    }
  }
  else if (max_upper)
  {
    upper = *max_upper;
  }

  // check for min > max
  if (upper && lower && *lower > *upper)
  {
    ROS_WARN_STREAM("Parameter " << name << "_max (" << *upper
                    << ") is less than parameter " << name << "_min (" << *lower << ").");
    double temp(*lower);
    lower = *upper;
    upper = temp;
  }

  std::ostringstream oss;
  oss << "  " << name << " limit: ";
  if (lower) oss << *lower << " ";
  else oss << "(none) ";
  if (upper) oss << *upper;
  else oss << "(none)";
  ROS_DEBUG_STREAM(oss.str());
}

double VescDriver::CommandLimit::clip(double value)
{
  if (lower && value < lower)
  {
    ROS_INFO_THROTTLE(10, "%s command value (%f) below minimum limit (%f), clipping.",
                      name.c_str(), value, *lower);
    return *lower;
  }
  if (upper && value > upper)
  {
    ROS_INFO_THROTTLE(10, "%s command value (%f) above maximum limit (%f), clipping.",
                      name.c_str(), value, *upper);
    return *upper;
  }
  return value;
}


}  // namespace vesc_driver

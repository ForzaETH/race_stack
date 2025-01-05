/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Parker-Lord Inertial Device Driver Implementation File
//
// Copyright (c) 2017, Brian Bingham
// Copyright (c) 2020, Parker Hannifin Corp
// This code is licensed under MIT license (see LICENSE file for details)
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

#include <string>

#include "microstrain_inertial_driver_common/utils/geo_utils.h"

#include "microstrain_inertial_driver_common/subscribers.h"

namespace microstrain
{

constexpr auto UTC_GPS_EPOCH_DUR = (315964800);
constexpr auto SECS_PER_WEEK = (60L * 60 * 24 * 7);

Subscribers::Subscribers(RosNodeType* node, Config* config)
  : node_(node), config_(config)
{
  // Initialize the transform buffer and listener ahead of time
  transform_buffer_ = createTransformBuffer(node_);
  transform_listener_ = createTransformListener(transform_buffer_);
}

bool Subscribers::activate()
{
  // Create a topic listener for external RTCM updates
  if (config_->ntrip_interface_enable_)
  {
    rtcm_sub_ = createSubscriber<>(node_, RTCM_TOPIC, 1000, &Subscribers::rtcmCallback, this);
  }

  // Setup the external measurement subscribers
  if (config_->subscribe_ext_time_ && config_->mip_device_->supportsDescriptor(mip::commands_base::DESCRIPTOR_SET, mip::commands_base::CMD_GPS_TIME_UPDATE))
  {
    MICROSTRAIN_INFO(node_, "Subscribing to %s for external time", EXT_TIME_TOPIC);
    external_time_sub_ = createSubscriber<>(node_, EXT_TIME_TOPIC, 1000, &Subscribers::externalTimeCallback, this);
  }
  if (config_->subscribe_ext_fix_ && config_->mip_device_->supportsDescriptor(mip::commands_aiding::DESCRIPTOR_SET, mip::commands_aiding::CMD_POS_LLH))
  {
    MICROSTRAIN_INFO(node_, "Subscribing to %s for external GPS position", EXT_FIX_TOPIC);
    external_gnss_position_sub_ = createSubscriber<>(node_, EXT_FIX_TOPIC, 1000, &Subscribers::externalGnssPositionCallback, this);
  }
  if (config_->subscribe_ext_vel_ned_ && config_->mip_device_->supportsDescriptor(mip::commands_aiding::DESCRIPTOR_SET, mip::commands_aiding::CMD_VEL_NED))
  {
    MICROSTRAIN_INFO(node_, "Subscribing to %s for external GPS Velocity in the NED frame", EXT_VEL_NED_TOPIC);
    external_vel_ned_sub_ = createSubscriber<>(node_, EXT_VEL_NED_TOPIC, 1000, &Subscribers::externalVelNedCallback, this);
  }
  if (config_->subscribe_ext_vel_enu_ && config_->mip_device_->supportsDescriptor(mip::commands_aiding::DESCRIPTOR_SET, mip::commands_aiding::CMD_VEL_NED))
  {
    MICROSTRAIN_INFO(node_, "Subscribing to %s for external GPS Velocity in the ENU frame", EXT_VEL_ENU_TOPIC);
    external_vel_enu_sub_ = createSubscriber<>(node_, EXT_VEL_ENU_TOPIC, 1000, &Subscribers::externalVelEnuCallback, this);
  }
  if (config_->subscribe_ext_vel_ecef_ && config_->mip_device_->supportsDescriptor(mip::commands_aiding::DESCRIPTOR_SET, mip::commands_aiding::CMD_VEL_ECEF))
  {
    MICROSTRAIN_INFO(node_, "Subscribing to %s for external GPS Velocity in the ECEF frame", EXT_VEL_ECEF_TOPIC);
    external_vel_ecef_sub_ = createSubscriber<>(node_, EXT_VEL_ECEF_TOPIC, 1000, &Subscribers::externalVelEcefCallback, this);
  }
  if (config_->subscribe_ext_vel_body_ && config_->mip_device_->supportsDescriptor(mip::commands_aiding::DESCRIPTOR_SET, mip::commands_aiding::CMD_VEL_ODOM))
  {
    MICROSTRAIN_INFO(node_, "Subscribing to %s for external Velocity in the Body frame", EXT_VEL_BODY_TOPIC);
    external_vel_body_sub_ = createSubscriber<>(node_, EXT_VEL_BODY_TOPIC, 1000, &Subscribers::externalVelBodyCallback, this);
  }
  if (config_->subscribe_ext_heading_ned_ && config_->mip_device_->supportsDescriptor(mip::commands_aiding::DESCRIPTOR_SET, mip::commands_aiding::CMD_HEADING_TRUE))
  {
    MICROSTRAIN_INFO(node_, "Subscribing to %s for external heading in the NED frame", EXT_HEADING_NED_TOPIC);
    external_heading_ned_sub_ = createSubscriber<>(node_, EXT_HEADING_NED_TOPIC, 1000, &Subscribers::externalHeadingNedCallback, this);
  }
  if (config_->subscribe_ext_heading_enu_ && config_->mip_device_->supportsDescriptor(mip::commands_aiding::DESCRIPTOR_SET, mip::commands_aiding::CMD_HEADING_TRUE))
  {
    MICROSTRAIN_INFO(node_, "Subscribing to %s for external heading in the ENU frame", EXT_HEADING_ENU_TOPIC);
    external_heading_enu_sub_ = createSubscriber<>(node_, EXT_HEADING_ENU_TOPIC, 1000, &Subscribers::externalHeadingEnuCallback, this);
  }
  if (config_->subscribe_ext_mag_ && config_->mip_device_->supportsDescriptor(mip::commands_aiding::DESCRIPTOR_SET, mip::commands_aiding::CMD_MAGNETIC_FIELD))
  {
    MICROSTRAIN_INFO(node_, "Subscribing to %s for external mag", EXT_MAG_TOPIC);
    external_mag_sub_ = createSubscriber(node_, EXT_MAG_TOPIC, 1000, &Subscribers::externalMagCallback, this);
  }
  if (config_->subscribe_ext_pressure_ && config_->mip_device_->supportsDescriptor(mip::commands_aiding::DESCRIPTOR_SET, mip::commands_aiding::CMD_PRESSURE))
  {
    MICROSTRAIN_INFO(node_, "Subscribing to %s for external pressure", EXT_PRESSURE_TOPIC);
    external_pressure_sub_ = createSubscriber(node_, EXT_PRESSURE_TOPIC, 1000, &Subscribers::externalPressureCallback, this);
  }

  return true;
}

void Subscribers::externalTimeCallback(const TimeReferenceMsg& time)
{
  // If the time is not within the first .25 of the top of the second, don't send it as it might cause an issue in processing
  double seconds = getTimeRefSecs(time.time_ref);
  if (seconds - round(seconds) >= 0.25)
  {
    MICROSTRAIN_WARN_THROTTLE(node_, 10, "Ignoring time ref %f because it is not within .25 seconds of the top of the second", seconds);
    return;
  }

  // Set the time on the device
  const int64_t gps_time = getTimeRefSec(time.time_ref) + config_->gps_leap_seconds_ - UTC_GPS_EPOCH_DUR;
  const uint32_t gps_tow = gps_time % static_cast<int32_t>(SECS_PER_WEEK);
  const uint32_t gps_week_number = (gps_time - gps_tow) / SECS_PER_WEEK;

  mip::CmdResult mip_cmd_result;
  MICROSTRAIN_DEBUG(node_, "Updating GPS week number to %u", gps_week_number);
  if (!(mip_cmd_result = mip::commands_base::writeGpsTimeUpdate(*(config_->mip_device_), mip::commands_base::GpsTimeUpdate::FieldId::WEEK_NUMBER, gps_week_number)))
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to send GPS time update for week number");
  MICROSTRAIN_DEBUG(node_, "Updating GPS time of week to %u", gps_tow);
  if (!(mip_cmd_result = mip::commands_base::writeGpsTimeUpdate(*(config_->mip_device_), mip::commands_base::GpsTimeUpdate::FieldId::TIME_OF_WEEK, gps_tow)))
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to send GPS time update for time of week");
}

void Subscribers::externalGnssPositionCallback(const NavSatFixMsg& fix)
{
  // Throw away messages that do not have a valid fix
  if (fix.status.status == NavSatFixMsg::_status_type::STATUS_NO_FIX)
  {
    MICROSTRAIN_WARN_THROTTLE(node_, 10, "Received LLH position with invalid fix. Ignoring");
    return;
  }

  // If the message has no uncertainty, we can't process it
  if (fix.position_covariance_type == NavSatFixMsg::COVARIANCE_TYPE_UNKNOWN)
  {
    MICROSTRAIN_WARN_THROTTLE(node_, 10, "Invalid NavSatFix message, must have uncertainty to process measurements");
    return;
  }

  // Get the sensor ID from the frame ID
  mip::commands_aiding::LlhPos llh_pos;
  llh_pos.time.reserved = 1;
  llh_pos.time.timebase = mip::commands_aiding::Time::Timebase::TIME_OF_ARRIVAL;
  if ((llh_pos.frame_id = getSensorIdFromFrameId(fix.header.frame_id)) == 0)
    return;

  // Fill out the rest of the message and send it
  llh_pos.valid_flags.setAll();
  llh_pos.latitude = fix.latitude;
  llh_pos.longitude = fix.longitude;
  llh_pos.height = fix.altitude;
  if (isNed(fix.header))
  {
    llh_pos.uncertainty[0] = sqrt(fix.position_covariance[0]);
    llh_pos.uncertainty[1] = sqrt(fix.position_covariance[4]);
  }
  else
  {
    llh_pos.uncertainty[0] = sqrt(fix.position_covariance[4]);
    llh_pos.uncertainty[1] = sqrt(fix.position_covariance[0]);
  }
  llh_pos.uncertainty[2] = sqrt(fix.position_covariance[8]);

  mip::CmdResult mip_cmd_result;
  if (!(mip_cmd_result = config_->mip_device_->device().runCommand<mip::commands_aiding::LlhPos>(llh_pos)))
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to send aiding LLH position aiding command");
}

void Subscribers::externalVelNedCallback(const TwistWithCovarianceStampedMsg& vel)
{
  // Get the sensor ID from the frame ID
  mip::commands_aiding::NedVel ned_vel;
  ned_vel.time.reserved = 1;
  ned_vel.time.timebase = mip::commands_aiding::Time::Timebase::TIME_OF_ARRIVAL;
  if ((ned_vel.frame_id = getSensorIdFromFrameId(vel.header.frame_id)) == 0)
    return;

  // Fill out the rest of the message and send it
  if (vel.twist.covariance[0] != 0)
  {
    ned_vel.velocity[0] = vel.twist.twist.linear.x;
    ned_vel.uncertainty[0] = sqrt(vel.twist.covariance[0]);
    ned_vel.valid_flags.x(true);
  }
  if (vel.twist.covariance[7] != 0)
  {
    ned_vel.velocity[1] = vel.twist.twist.linear.y;
    ned_vel.uncertainty[1] = sqrt(vel.twist.covariance[7]);
    ned_vel.valid_flags.y(true);
  }
  if (vel.twist.covariance[14] != 0)
  {
    ned_vel.velocity[2] = vel.twist.twist.linear.z;
    ned_vel.uncertainty[2] = sqrt(vel.twist.covariance[14]);
    ned_vel.valid_flags.z(true);
  }

  mip::CmdResult mip_cmd_result;
  if (!(mip_cmd_result = config_->mip_device_->device().runCommand<mip::commands_aiding::NedVel>(ned_vel)))
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to send NED velocity aiding command");
}

void Subscribers::externalVelEnuCallback(const TwistWithCovarianceStampedMsg& vel)
{
  // Get the sensor ID from the frame ID
  mip::commands_aiding::NedVel ned_vel;
  ned_vel.time.reserved = 1;
  ned_vel.time.timebase = mip::commands_aiding::Time::Timebase::TIME_OF_ARRIVAL;
  if ((ned_vel.frame_id = getSensorIdFromFrameId(vel.header.frame_id)) == 0)
    return;

  // Fill out the rest of the message and send it
  if (vel.twist.covariance[7] != 0)
  {
    ned_vel.velocity[0] = vel.twist.twist.linear.y;
    ned_vel.uncertainty[0] = sqrt(vel.twist.covariance[7]);
    ned_vel.valid_flags.x(true);
  }
  if (vel.twist.covariance[0] != 0)
  {
    ned_vel.velocity[1] = vel.twist.twist.linear.x;
    ned_vel.uncertainty[1] = sqrt(vel.twist.covariance[0]);
    ned_vel.valid_flags.y(true);
  }
  if (vel.twist.covariance[14] != 0)
  {
    ned_vel.velocity[2] = -vel.twist.twist.linear.z;
    ned_vel.uncertainty[2] = sqrt(vel.twist.covariance[14]);
    ned_vel.valid_flags.z(true);
  }

  mip::CmdResult mip_cmd_result;
  if (!(mip_cmd_result = config_->mip_device_->device().runCommand<mip::commands_aiding::NedVel>(ned_vel)))
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to send ENU velocity aiding command");
}

void Subscribers::externalVelEcefCallback(const TwistWithCovarianceStampedMsg& vel)
{
  // Get the sensor ID from the frame ID
  mip::commands_aiding::EcefVel ecef_vel;
  ecef_vel.time.reserved = 1;
  ecef_vel.time.timebase = mip::commands_aiding::Time::Timebase::TIME_OF_ARRIVAL;
  if ((ecef_vel.frame_id = getSensorIdFromFrameId(vel.header.frame_id)) == 0)
    return;

  // Fill out the rest of the message and send it
  if (vel.twist.covariance[0] != 0)
  {
    ecef_vel.velocity[0] = vel.twist.twist.linear.x;
    ecef_vel.uncertainty[0] = sqrt(vel.twist.covariance[0]);
    ecef_vel.valid_flags.x(true);
  }
  if (vel.twist.covariance[7] != 0)
  {
    ecef_vel.velocity[1] = vel.twist.twist.linear.y;
    ecef_vel.uncertainty[1] = sqrt(vel.twist.covariance[7]);
    ecef_vel.valid_flags.y(true);
  }
  if (vel.twist.covariance[14] != 0)
  {
    ecef_vel.velocity[2] = vel.twist.twist.linear.z;
    ecef_vel.uncertainty[2] = sqrt(vel.twist.covariance[14]);
    ecef_vel.valid_flags.z(true);
  }

  mip::CmdResult mip_cmd_result;
  if (!(mip_cmd_result = config_->mip_device_->device().runCommand<mip::commands_aiding::EcefVel>(ecef_vel)))
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to send ECEF velocity aiding command");
}

void Subscribers::externalVelBodyCallback(const TwistWithCovarianceStampedMsg& vel)
{
  // Get the sensor ID from the frame ID
  mip::commands_aiding::VehicleFixedFrameVelocity vehicle_fixed_frame_velocity;
  vehicle_fixed_frame_velocity.time.reserved = 1;
  vehicle_fixed_frame_velocity.time.timebase = mip::commands_aiding::Time::Timebase::TIME_OF_ARRIVAL;
  if ((vehicle_fixed_frame_velocity.frame_id = getSensorIdFromFrameId(vel.header.frame_id)) == 0)
    return;

  // Fill out the rest of the message and send it
  if (vel.twist.covariance[0] != 0)
  {
    vehicle_fixed_frame_velocity.velocity[0] = vel.twist.twist.linear.x;
    vehicle_fixed_frame_velocity.uncertainty[0] = sqrt(vel.twist.covariance[0]);
    vehicle_fixed_frame_velocity.valid_flags.x(true);
  }
  if (vel.twist.covariance[7] != 0)
  {
    vehicle_fixed_frame_velocity.velocity[1] = vel.twist.twist.linear.y;
    vehicle_fixed_frame_velocity.uncertainty[1] = sqrt(vel.twist.covariance[7]);
    vehicle_fixed_frame_velocity.valid_flags.y(true);
  }
  if (vel.twist.covariance[14] != 0)
  {
    vehicle_fixed_frame_velocity.velocity[2] = vel.twist.twist.linear.z;
    vehicle_fixed_frame_velocity.uncertainty[2] = sqrt(vel.twist.covariance[14]);
    vehicle_fixed_frame_velocity.valid_flags.z(true);
  }

  mip::CmdResult mip_cmd_result;
  if (!(mip_cmd_result = config_->mip_device_->device().runCommand<mip::commands_aiding::VehicleFixedFrameVelocity>(vehicle_fixed_frame_velocity)))
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to send body frame velocity command");
}

void Subscribers::externalHeadingNedCallback(const PoseWithCovarianceStampedMsg& heading)
{
  // Fill out the time of the message
  mip::commands_aiding::TrueHeading true_heading;
  true_heading.time.reserved = 1;
  true_heading.time.timebase = mip::commands_aiding::Time::Timebase::TIME_OF_ARRIVAL;
  if ((true_heading.frame_id = getSensorIdFromFrameId(heading.header.frame_id)) == 0)
    return;

  // Make sure we have uncertainty
  if (heading.pose.covariance[35] == 0)
  {
    MICROSTRAIN_WARN_THROTTLE(node_, 10, "Invalid heading received on %s. Must have yaw uncertainty", EXT_HEADING_NED_TOPIC);
    return;
  }

  // Convert the quaternion to RPY
  double r, p, y;
  tf2::Quaternion q;
  tf2::fromMsg(heading.pose.pose.orientation, q);
  tf2::Matrix3x3 m(q);
  m.getRPY(r, p, y);

  // Fill out the rest of the message and send it
  true_heading.valid_flags = 0xFFFF;
  true_heading.heading = y;
  true_heading.uncertainty = sqrt(heading.pose.covariance[35]);

  mip::CmdResult mip_cmd_result;
  if (!(mip_cmd_result = config_->mip_device_->device().runCommand<mip::commands_aiding::TrueHeading>(true_heading)))
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to send external heading command");
}

void Subscribers::externalHeadingEnuCallback(const PoseWithCovarianceStampedMsg& heading)
{
  // Fill out the time of the message
  mip::commands_aiding::TrueHeading true_heading;
  true_heading.time.reserved = 1;
  true_heading.time.timebase = mip::commands_aiding::Time::Timebase::TIME_OF_ARRIVAL;
  if ((true_heading.frame_id = getSensorIdFromFrameId(heading.header.frame_id)) == 0)
    return;

  // Make sure we have uncertainty
  if (heading.pose.covariance[35] == 0)
  {
    MICROSTRAIN_WARN_THROTTLE(node_, 10, "Invalid heading received on %s. Must have yaw uncertainty", EXT_HEADING_NED_TOPIC);
    return;
  }

  // Convert the quaternion to RPY and rotate it to NED
  double r, p, y;
  tf2::Quaternion q;
  tf2::fromMsg(heading.pose.pose.orientation, q);
  q = config_->ned_to_enu_transform_tf_.inverse() * q;
  tf2::Matrix3x3 m(q);
  m.getRPY(r, p, y);

  // Fill out the rest of the message and send it
  true_heading.valid_flags = 0xFFFF;
  true_heading.heading = y;
  true_heading.uncertainty = sqrt(heading.pose.covariance[35]);

  mip::CmdResult mip_cmd_result;
  if (!(mip_cmd_result = config_->mip_device_->device().runCommand<mip::commands_aiding::TrueHeading>(true_heading)))
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to send external heading command");
}

void Subscribers::externalMagCallback(const MagneticFieldMsg& mag)
{
  // Fill out the time of the message
  mip::commands_aiding::MagneticField magnetic_field;
  magnetic_field.time.reserved = 1;
  magnetic_field.time.timebase = mip::commands_aiding::Time::Timebase::TIME_OF_ARRIVAL;
  if ((magnetic_field.frame_id = getSensorIdFromFrameId(mag.header.frame_id)) == 0)
    return;

  // Fill out the rest of the message and send it
  if (mag.magnetic_field_covariance[0] != 0)
  {
    magnetic_field.magnetic_field[0] = mag.magnetic_field.x;
    magnetic_field.uncertainty[0] = sqrt(mag.magnetic_field_covariance[0]);
    magnetic_field.valid_flags.x(true);
  }
  if (mag.magnetic_field_covariance[4] != 0)
  {
    magnetic_field.magnetic_field[1] = mag.magnetic_field.y;
    magnetic_field.uncertainty[1] = sqrt(mag.magnetic_field_covariance[4]);
    magnetic_field.valid_flags.y(true);
  }
  if (mag.magnetic_field_covariance[8] != 0)
  {
    magnetic_field.magnetic_field[2] = mag.magnetic_field.z;
    magnetic_field.uncertainty[2] = sqrt(mag.magnetic_field_covariance[8]);
    magnetic_field.valid_flags.z(true);
  }

  mip::CmdResult mip_cmd_result;
  if (!(mip_cmd_result = config_->mip_device_->device().runCommand<mip::commands_aiding::MagneticField>(magnetic_field)))
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to send magnetic field command");
}

void Subscribers::externalPressureCallback(const FluidPressureMsg& fluid_pressure)
{
  mip::commands_aiding::Pressure pressure;
  pressure.time.reserved = 1;
  pressure.time.timebase = mip::commands_aiding::Time::Timebase::TIME_OF_ARRIVAL;
  if ((pressure.frame_id = getSensorIdFromFrameId(fluid_pressure.header.frame_id)) == 0)
    return;

  // Make sure we have uncertainty
  if (fluid_pressure.variance == 0)
  {
    MICROSTRAIN_WARN_THROTTLE(node_, 10, "Invalid pressure received on %s. Must have variance", EXT_PRESSURE_TOPIC);
    return;
  }

  // Convert from Pascals to mBar and send it
  pressure.valid_flags = 0xFFFF;
  pressure.pressure = fluid_pressure.fluid_pressure * 100;
  pressure.uncertainty = sqrt(fluid_pressure.variance);

  mip::CmdResult mip_cmd_result;
  if (!(mip_cmd_result = config_->mip_device_->device().runCommand<mip::commands_aiding::Pressure>(pressure)))
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to send external pressure command");
}

void Subscribers::rtcmCallback(const RTCMMsg& rtcm)
{
  MICROSTRAIN_DEBUG(node_, "Received RTCM message of size %lu", rtcm.message.size());
  if (config_->aux_device_)
  {
    if (!config_->aux_device_->send(rtcm.message.data(), rtcm.message.size()))
      MICROSTRAIN_ERROR(node_, "Failed to write RTCM to device");
    else
      MICROSTRAIN_DEBUG(node_, "Successfully wrote RTCM message of size %lu to aux port", rtcm.message.size());
  }
  else
  {
    MICROSTRAIN_WARN(node_, "Note: Device did not configure the aux port. See startup logs for why it was not configured");
  }
}

uint8_t Subscribers::getSensorIdFromFrameId(const std::string& frame_id)
{
  const auto& frame_id_iter = std::find(external_frame_ids_.begin(), external_frame_ids_.end(), frame_id);
  if (frame_id_iter != external_frame_ids_.end())
  {
    // Looks like we already configured this frame_id, so just grab the index
    return frame_id_iter - external_frame_ids_.begin();
  }
  else
  {
    // If we have reached max IDs, we can't do anything
    if (external_frame_ids_size_ >= config_->mip_device_->max_external_frame_ids_)
    {
      MICROSTRAIN_WARN_THROTTLE(node_, 10, "%u Sensor IDs have already been configured, we will not be able to configure %s", config_->mip_device_->max_external_frame_ids_, frame_id.c_str());
      return 0;
    }
    else
    {
      // Attempt to find the transform from ROS
      std::string tf_error_string;
      RosTimeType frame_time; setRosTime(&frame_time, 0, 0);
      if (transform_buffer_->canTransform(config_->frame_id_, frame_id, frame_time, RosDurationType(0, 0), &tf_error_string))
      {
        // Populate the reference frame command from the transform. Convert from ROS vehicle frame to Microstrain vehicle frame if operating in enu mode
        auto vehicle_to_sensor_transform = transform_buffer_->lookupTransform(config_->frame_id_, frame_id, frame_time);
        if (config_->use_enu_frame_)
        {
          tf2::Transform ros_vehicle_to_sensor_transform_tf, microstrain_vehicle_to_sensor_transform_tf;
          tf2::fromMsg(vehicle_to_sensor_transform.transform, ros_vehicle_to_sensor_transform_tf);
          const tf2::Vector3& ros_vehicle_to_sensor_translation_tf = ros_vehicle_to_sensor_transform_tf.getOrigin();
          const tf2::Quaternion& ros_vehicle_to_sensor_rotation_tf = ros_vehicle_to_sensor_transform_tf.getRotation();
          const tf2::Vector3& microstrain_vehicle_to_sensor_translation_tf = config_->ros_vehicle_to_microstrain_vehicle_transform_tf_ * ros_vehicle_to_sensor_translation_tf;
          const tf2::Quaternion& microstrain_vehicle_to_sensor_rotation_tf = config_->ros_vehicle_to_microstrain_vehicle_transform_tf_ * ros_vehicle_to_sensor_rotation_tf;
          microstrain_vehicle_to_sensor_transform_tf.setOrigin(microstrain_vehicle_to_sensor_translation_tf);
          microstrain_vehicle_to_sensor_transform_tf.setRotation(microstrain_vehicle_to_sensor_rotation_tf);
          vehicle_to_sensor_transform.transform = tf2::toMsg(microstrain_vehicle_to_sensor_transform_tf);
        }
        mip::commands_aiding::FrameConfig frame_config;
        frame_config.format = mip::commands_aiding::FrameConfig::Format::QUATERNION;
        frame_config.frame_id = external_frame_ids_size_ + 1;
        frame_config.function = mip::FunctionSelector::WRITE;
        frame_config.translation = {
          static_cast<float>(vehicle_to_sensor_transform.transform.translation.x),
          static_cast<float>(vehicle_to_sensor_transform.transform.translation.y),
          static_cast<float>(vehicle_to_sensor_transform.transform.translation.z)
        };
        frame_config.rotation.quaternion = {
          static_cast<float>(vehicle_to_sensor_transform.transform.rotation.w),
          static_cast<float>(vehicle_to_sensor_transform.transform.rotation.x),
          static_cast<float>(vehicle_to_sensor_transform.transform.rotation.y),
          static_cast<float>(vehicle_to_sensor_transform.transform.rotation.z)
        };

        // This is a little expensive to do for debug, but should only happen rarely
        const tf2::Quaternion q(
          vehicle_to_sensor_transform.transform.rotation.x,
          vehicle_to_sensor_transform.transform.rotation.y,
          vehicle_to_sensor_transform.transform.rotation.z,
          vehicle_to_sensor_transform.transform.rotation.w);
        const tf2::Matrix3x3 m(q);
        double r, p, y;
        m.getRPY(r, p, y);
        MICROSTRAIN_INFO(node_, "Associating MIP frame %u with ROS frame %s", frame_config.frame_id, frame_id.c_str());
        MICROSTRAIN_INFO(node_, "  Front (meters): %f, Right (meters): %f, Down (meters): %f", vehicle_to_sensor_transform.transform.translation.x, vehicle_to_sensor_transform.transform.translation.y, vehicle_to_sensor_transform.transform.translation.z);
        MICROSTRAIN_INFO(node_, "  Roll (degrees): %f, Pitch (degrees): %f, Yaw (degrees): %f", r * 180 / M_PI, p * 180 / M_PI, y * 180 / M_PI);

        // Attempt to send the command
        mip::CmdResult mip_cmd_result;
        if (!(mip_cmd_result = config_->mip_device_->device().runCommand<mip::commands_aiding::FrameConfig>(frame_config)))
        {
          MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to send aiding LLH position aiding command");
          return 0;
        }

        // Looks like everything is set up, append the frame ID and return it's index
        external_frame_ids_[++external_frame_ids_size_] = frame_id;
        return external_frame_ids_size_;
      }
      else
      {
        MICROSTRAIN_WARN_THROTTLE(node_, 10, "Unable to determine transform from %s to %s: %s", config_->frame_id_.c_str(), frame_id.c_str(), tf_error_string.c_str());
        return 0;
      }
    }
  }
}

}  // namespace microstrain

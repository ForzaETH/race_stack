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
#include <algorithm>

#include "microstrain_inertial_driver_common/publishers.h"
#include "microstrain_inertial_driver_common/utils/geo_utils.h"
#include "microstrain_inertial_driver_common/utils/mip/built_in_test.h"

namespace microstrain
{

constexpr auto USTRAIN_G =
    9.80665;  // from section 5.1.1 in
              // https://www.microstrain.com/sites/default/files/3dm-gx5-25_dcp_manual_8500-0065_reference_document.pdf

NavSatFixMsg::_position_covariance_type getTranslationCovarianceFromCovariance(const PoseWithCovarianceStampedMsg::_pose_type::_covariance_type& covariance)
{
  return
  {
    covariance[0],  covariance[1],  covariance[2],
    covariance[6],  covariance[7],  covariance[8],
    covariance[12], covariance[13], covariance[14]
  };
}

void setTranslationCovarianceOnCovariance(PoseWithCovarianceStampedMsg::_pose_type::_covariance_type* covariance, const NavSatFixMsg::_position_covariance_type& translation_covariance)
{
  (*covariance)[0] = translation_covariance[0];
  (*covariance)[1] = translation_covariance[1];
  (*covariance)[2] = translation_covariance[2];
  (*covariance)[6] = translation_covariance[3];
  (*covariance)[7] = translation_covariance[4];
  (*covariance)[8] = translation_covariance[5];
  (*covariance)[12] = translation_covariance[6];
  (*covariance)[13] = translation_covariance[7];
  (*covariance)[14] = translation_covariance[8];
}

ImuMsg::_orientation_covariance_type getRotationCovarianceFromCovariance(const PoseWithCovarianceStampedMsg::_pose_type::_covariance_type& covariance)
{
  return
  {
    covariance[21], covariance[22], covariance[23],
    covariance[27], covariance[28], covariance[29],
    covariance[33], covariance[34], covariance[35]
  };
}

void setRotationCovarianceOnCovariance(PoseWithCovarianceStampedMsg::_pose_type::_covariance_type* covariance, const ImuMsg::_orientation_covariance_type& rotation_covariance)
{
  (*covariance)[21] = rotation_covariance[0];
  (*covariance)[22] = rotation_covariance[1];
  (*covariance)[23] = rotation_covariance[2];
  (*covariance)[27] = rotation_covariance[3];
  (*covariance)[28] = rotation_covariance[4];
  (*covariance)[29] = rotation_covariance[5];
  (*covariance)[33] = rotation_covariance[6];
  (*covariance)[34] = rotation_covariance[7];
  (*covariance)[35] = rotation_covariance[8];
}

constexpr double gpsTimestampSecs(const mip::data_shared::GpsTimestamp& gps_timestamp)
{
  return gps_timestamp.week_number * 604800 + gps_timestamp.tow;
}

Publishers::Publishers(RosNodeType* node, Config* config)
  : node_(node), config_(config)
{
  // Initialize the transform buffer and listener ahead of time
  transform_buffer_ = createTransformBuffer(node_);
  transform_listener_ = createTransformListener(transform_buffer_);
}

bool Publishers::configure()
{
  imu_raw_pub_->configure(node_, config_);
  imu_pub_->configure(node_, config_);
  mag_pub_->configure(node_, config_);
  pressure_pub_->configure(node_, config_);
  wheel_speed_pub_->configure(node_, config_);

  for (const auto& pub : gnss_llh_position_pub_) pub->configure(node_, config_);
  for (const auto& pub : gnss_velocity_pub_) pub->configure(node_, config_);
  for (const auto& pub : gnss_velocity_ecef_pub_) pub->configure(node_, config_);
  for (const auto& pub : gnss_odometry_pub_) pub->configure(node_, config_);
  for (const auto& pub : gnss_time_pub_) pub->configure(node_, config_);

  filter_human_readable_status_pub_->configure(node_, config_);
  filter_imu_pub_->configure(node_, config_);
  filter_llh_position_pub_->configure(node_, config_);
  filter_velocity_pub_->configure(node_, config_);
  filter_velocity_ecef_pub_->configure(node_, config_);
  filter_dual_antenna_heading_pub_->configure(node_, config_);

  // Only publish odometry if we support the related position field
  if (config_->mip_device_->supportsDescriptor(mip::data_filter::DESCRIPTOR_SET, mip::data_filter::EcefPos::FIELD_DESCRIPTOR) || config_->mip_device_->supportsDescriptor(mip::data_filter::DESCRIPTOR_SET, mip::data_filter::PositionLlh::FIELD_DESCRIPTOR))
  {
    filter_odometry_earth_pub_->configure(node_, config_);
    filter_odometry_map_pub_->configure(node_, config_);
  }

  mip_sensor_overrange_status_pub_->configure(node_, config_);
  mip_sensor_temperature_statistics_pub_->configure(node_, config_);

  for (const auto& pub : mip_gnss_fix_info_pub_) pub->configure(node_, config_);
  for (const auto& pub : mip_gnss_sbas_info_pub_) pub->configure(node_, config_);
  for (const auto& pub : mip_gnss_rf_error_detection_pub_) pub->configure(node_, config_);

  if (config_->rtk_dongle_enable_ && config_->mip_device_->supportsDescriptor(mip::data_gnss::MIP_GNSS3_DATA_DESC_SET, mip::data_gnss::DATA_RTK_CORRECTIONS_STATUS))
    mip_gnss_corrections_rtk_corrections_status_pub_->configure(node_);

  mip_filter_status_pub_->configure(node_, config_);
  mip_filter_gnss_position_aiding_status_pub_->configure(node_, config_);
  mip_filter_multi_antenna_offset_correction_pub_->configure(node_, config_);
  mip_filter_aiding_measurement_summary_pub_->configure(node_, config_);
  mip_filter_gnss_dual_antenna_status_pub_->configure(node_, config_);

  mip_system_built_in_test_pub_->configure(node_, config_);

  const bool will_publish_nmea = (config_->mip_device_->connection() != nullptr && config_->mip_device_->connection()->shouldParseNmea()) ||
                                 (config_->aux_device_ != nullptr && config_->aux_device_->connection() != nullptr && config_->aux_device_->connection()->shouldParseNmea());
  if (will_publish_nmea)
    nmea_sentence_pub_->configure(node_);

  // Frame ID configuration
  imu_raw_pub_->getMessage()->header.frame_id = config_->frame_id_;
  imu_pub_->getMessage()->header.frame_id = config_->frame_id_;
  mag_pub_->getMessage()->header.frame_id = config_->frame_id_;
  pressure_pub_->getMessage()->header.frame_id = config_->frame_id_;
  wheel_speed_pub_->getMessage()->header.frame_id = config_->odometer_frame_id_;

  for (int i = 0; i < gnss_llh_position_pub_.size(); i++) gnss_llh_position_pub_[i]->getMessage()->header.frame_id = config_->gnss_frame_id_[i];
  for (int i = 0; i < gnss_velocity_pub_.size(); i++) gnss_velocity_pub_[i]->getMessage()->header.frame_id = config_->gnss_frame_id_[i];
  for (int i = 0; i < gnss_velocity_ecef_pub_.size(); i++) gnss_velocity_ecef_pub_[i]->getMessage()->header.frame_id = config_->gnss_frame_id_[i];
  for (int i = 0; i < gnss_odometry_pub_.size(); i++) gnss_odometry_pub_[i]->getMessage()->header.frame_id = config_->earth_frame_id_;
  for (int i = 0; i < gnss_odometry_pub_.size(); i++) gnss_odometry_pub_[i]->getMessage()->child_frame_id = config_->gnss_frame_id_[i];
  for (int i = 0; i < gnss_time_pub_.size(); i++) gnss_time_pub_[i]->getMessage()->header.frame_id = config_->gnss_frame_id_[i];

  filter_human_readable_status_pub_->getMessage()->header.frame_id = config_->frame_id_;
  filter_imu_pub_->getMessage()->header.frame_id = config_->frame_id_;
  filter_llh_position_pub_->getMessage()->header.frame_id = config_->frame_id_;
  filter_velocity_pub_->getMessage()->header.frame_id = config_->frame_id_;
  filter_velocity_ecef_pub_->getMessage()->header.frame_id = config_->frame_id_;
  filter_odometry_earth_pub_->getMessage()->header.frame_id = config_->earth_frame_id_;
  filter_odometry_earth_pub_->getMessage()->child_frame_id = config_->frame_id_;
  filter_odometry_map_pub_->getMessage()->header.frame_id = config_->map_frame_id_;
  filter_odometry_map_pub_->getMessage()->child_frame_id = config_->frame_id_;
  filter_dual_antenna_heading_pub_->getMessage()->header.frame_id = config_->frame_id_;

  config_->map_to_earth_transform_.header.frame_id = config_->earth_frame_id_;
  config_->map_to_earth_transform_.child_frame_id = config_->map_frame_id_;

  // Static covariance configuration
  auto imu_raw_msg = imu_raw_pub_->getMessage();
  auto imu_msg = imu_pub_->getMessage();
  auto mag_msg = mag_pub_->getMessage();
  std::copy(config_->imu_linear_cov_.begin(), config_->imu_linear_cov_.end(), imu_raw_msg->linear_acceleration_covariance.begin());
  std::copy(config_->imu_angular_cov_.begin(), config_->imu_angular_cov_.end(), imu_raw_msg->angular_velocity_covariance.begin());
  std::copy(config_->imu_linear_cov_.begin(), config_->imu_linear_cov_.end(), imu_msg->linear_acceleration_covariance.begin());
  std::copy(config_->imu_angular_cov_.begin(), config_->imu_angular_cov_.end(), imu_msg->angular_velocity_covariance.begin());
  std::copy(config_->imu_orientation_cov_.begin(), config_->imu_orientation_cov_.end(), imu_msg->orientation_covariance.begin());
  std::copy(config_->imu_mag_cov_.begin(), config_->imu_mag_cov_.end(), mag_msg->magnetic_field_covariance.begin());
  pressure_pub_->getMessage()->variance = config_->imu_pressure_vairance_;

  supports_filter_ecef_ = config_->mip_device_->supportsDescriptor(mip::data_filter::DESCRIPTOR_SET, mip::data_filter::DATA_ECEF_POS);

  // Human readable status message configuration
  auto filter_human_readable_status_msg = filter_human_readable_status_pub_->getMessage();
  filter_human_readable_status_msg->device_info.firmware_version = RosMipDevice::firmwareVersionString(config_->mip_device_->device_info_.firmware_version);
  filter_human_readable_status_msg->device_info.model_name = config_->mip_device_->device_info_.model_name;
  filter_human_readable_status_msg->device_info.model_number = config_->mip_device_->device_info_.model_number;
  filter_human_readable_status_msg->device_info.serial_number = config_->mip_device_->device_info_.serial_number;
  filter_human_readable_status_msg->device_info.lot_number = config_->mip_device_->device_info_.lot_number;
  filter_human_readable_status_msg->device_info.device_options = config_->mip_device_->device_info_.device_options;
  if (RosMipDevice::isPhilo(config_->mip_device_->device_info_))
    filter_human_readable_status_msg->dual_antenna_fix_type = HumanReadableStatusMsg::UNSUPPORTED;
  if (!config_->mip_device_->supportsDescriptorSet(mip::data_gnss::DESCRIPTOR_SET) && !config_->mip_device_->supportsDescriptorSet(mip::data_gnss::MIP_GNSS1_DATA_DESC_SET))
    filter_human_readable_status_msg->gnss_state = HumanReadableStatusMsg::UNSUPPORTED;

  // Transform broadcaster setup
  static_transform_broadcaster_ = createStaticTransformBroadcaster(node_);
  transform_broadcaster_ = createTransformBroadcaster(node_);

  // If the source is manual, set up our transform here
  if (config_->filter_relative_pos_config_ && config_->filter_relative_pos_source_ == REL_POS_SOURCE_MANUAL)
  {
    // Set the translation from config
    config_->map_to_earth_transform_.header.stamp = rosTimeNow(node_);
    if (config_->filter_relative_pos_frame_ == REL_POS_FRAME_ECEF)
    {
      config_->map_to_earth_transform_.transform.translation.x = config_->filter_relative_pos_ref_[0];
      config_->map_to_earth_transform_.transform.translation.y = config_->filter_relative_pos_ref_[1];
      config_->map_to_earth_transform_.transform.translation.z = config_->filter_relative_pos_ref_[2];
    }
    else if (config_->filter_relative_pos_frame_ == REL_POS_FRAME_LLH)
    {
      config_->geocentric_converter_.Forward(config_->filter_relative_pos_ref_[0], config_->filter_relative_pos_ref_[1], config_->filter_relative_pos_ref_[2],
          config_->map_to_earth_transform_.transform.translation.x, config_->map_to_earth_transform_.transform.translation.y, config_->map_to_earth_transform_.transform.translation.z);
    }
    else
    {
      MICROSTRAIN_ERROR(node_, "Invalid filter_relative_pos_frame %d", config_->filter_relative_pos_frame_);
      return false;
    }

    // Determine the rotation from ECEF to NED/ENU for this position
    double lat, lon, alt;
    config_->geocentric_converter_.Reverse(config_->map_to_earth_transform_.transform.translation.x, config_->map_to_earth_transform_.transform.translation.y, config_->map_to_earth_transform_.transform.translation.z, lat, lon, alt);
    if (config_->use_enu_frame_)
      config_->map_to_earth_transform_.transform.rotation = tf2::toMsg(ecefToEnuTransformQuat(lat, lon));
    else
      config_->map_to_earth_transform_.transform.rotation = tf2::toMsg(ecefToNedTransformQuat(lat, lon));

    // Note that the data is valid so we can publish it on activate
    config_->map_to_earth_transform_valid_ = true;
  }

  // Static antenna offsets
  mip::CmdResult mip_cmd_result;
  float gnss_antenna_offsets[3];
  if (config_->mip_device_->supportsDescriptor(mip::commands_filter::DESCRIPTOR_SET, mip::commands_filter::CMD_ANTENNA_OFFSET))
  {
    if (!(mip_cmd_result = mip::commands_filter::readAntennaOffset(*(config_->mip_device_), gnss_antenna_offsets)))
    {
      MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to read GNSS antenna offsets required for GNSS antenna transforms");
      return false;
    }
    TransformStampedMsg& gnss_1_antenna_link_to_imu_link_transform = gnss_antenna_link_to_imu_link_transform_[GNSS1_ID];
    gnss_1_antenna_link_to_imu_link_transform.header.stamp = rosTimeNow(node_);
    gnss_1_antenna_link_to_imu_link_transform.header.frame_id = config_->frame_id_;
    gnss_1_antenna_link_to_imu_link_transform.child_frame_id = config_->gnss_frame_id_[GNSS1_ID];
    gnss_1_antenna_link_to_imu_link_transform.transform.rotation = tf2::toMsg(tf2::Quaternion::getIdentity());

    const tf2::Transform gnss_1_antenna_link_to_microstrain_vehicle_transform_tf(tf2::Quaternion::getIdentity(), tf2::Vector3(gnss_antenna_offsets[0], gnss_antenna_offsets[1], gnss_antenna_offsets[2]));
    if (config_->use_enu_frame_)
    {
      const tf2::Transform gnss_1_antenna_link_to_ros_vehicle_transform_tf = config_->ros_vehicle_to_microstrain_vehicle_transform_tf_.inverse() * gnss_1_antenna_link_to_microstrain_vehicle_transform_tf;
      gnss_1_antenna_link_to_imu_link_transform.transform.translation.x = gnss_1_antenna_link_to_ros_vehicle_transform_tf.getOrigin().getX();
      gnss_1_antenna_link_to_imu_link_transform.transform.translation.y = gnss_1_antenna_link_to_ros_vehicle_transform_tf.getOrigin().getY();
      gnss_1_antenna_link_to_imu_link_transform.transform.translation.z = gnss_1_antenna_link_to_ros_vehicle_transform_tf.getOrigin().getZ();
    }
    else
    {
      gnss_1_antenna_link_to_imu_link_transform.transform.translation.x = gnss_1_antenna_link_to_microstrain_vehicle_transform_tf.getOrigin().getX();
      gnss_1_antenna_link_to_imu_link_transform.transform.translation.y = gnss_1_antenna_link_to_microstrain_vehicle_transform_tf.getOrigin().getY();
      gnss_1_antenna_link_to_imu_link_transform.transform.translation.z = gnss_1_antenna_link_to_microstrain_vehicle_transform_tf.getOrigin().getZ();
    }
  }
  else if (config_->mip_device_->supportsDescriptor(mip::commands_filter::DESCRIPTOR_SET, mip::commands_filter::CMD_MULTI_ANTENNA_OFFSET))
  {
    for (const uint8_t gnss_id : std::initializer_list<uint8_t>{GNSS1_ID, GNSS2_ID})
    {
      if (!(mip_cmd_result = mip::commands_filter::readMultiAntennaOffset(*(config_->mip_device_), gnss_id + 1, gnss_antenna_offsets)))
      {
        MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to read GNSS antenna offsets required for GNSS antenna transforms");
        return false;
      }
      TransformStampedMsg& gnss_x_antenna_link_to_imu_link_transform = gnss_antenna_link_to_imu_link_transform_[gnss_id];
      gnss_x_antenna_link_to_imu_link_transform.header.stamp = rosTimeNow(node_);
      gnss_x_antenna_link_to_imu_link_transform.header.frame_id = config_->frame_id_;
      gnss_x_antenna_link_to_imu_link_transform.child_frame_id = config_->gnss_frame_id_[gnss_id];
      gnss_x_antenna_link_to_imu_link_transform.transform.rotation = tf2::toMsg(tf2::Quaternion::getIdentity());

      const tf2::Transform gnss_x_antenna_link_to_microstrain_vehicle_transform_tf(tf2::Quaternion::getIdentity(), tf2::Vector3(gnss_antenna_offsets[0], gnss_antenna_offsets[1], gnss_antenna_offsets[2]));
      if (config_->use_enu_frame_)
      {
        const tf2::Transform gnss_x_antenna_link_to_ros_vehicle_transform_tf = config_->ros_vehicle_to_microstrain_vehicle_transform_tf_.inverse() * gnss_x_antenna_link_to_microstrain_vehicle_transform_tf;
        gnss_x_antenna_link_to_imu_link_transform.transform.translation.x = gnss_x_antenna_link_to_ros_vehicle_transform_tf.getOrigin().getX();
        gnss_x_antenna_link_to_imu_link_transform.transform.translation.y = gnss_x_antenna_link_to_ros_vehicle_transform_tf.getOrigin().getY();
        gnss_x_antenna_link_to_imu_link_transform.transform.translation.z = gnss_x_antenna_link_to_ros_vehicle_transform_tf.getOrigin().getZ();
      }
      else
      {
        gnss_x_antenna_link_to_imu_link_transform.transform.translation.x = gnss_x_antenna_link_to_microstrain_vehicle_transform_tf.getOrigin().getX();
        gnss_x_antenna_link_to_imu_link_transform.transform.translation.y = gnss_x_antenna_link_to_microstrain_vehicle_transform_tf.getOrigin().getY();
        gnss_x_antenna_link_to_imu_link_transform.transform.translation.z = gnss_x_antenna_link_to_microstrain_vehicle_transform_tf.getOrigin().getZ();
      }
    }
  }

  // Static odometer offset
  if (config_->mip_device_->supportsDescriptor(mip::commands_filter::DESCRIPTOR_SET, mip::commands_filter::CMD_SPEED_LEVER_ARM))
  {
    float speed_lever_arm[3];
    if (!(mip_cmd_result = mip::commands_filter::readSpeedLeverArm(*(config_->mip_device_), 1, speed_lever_arm)))
    {
      MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to read Filter Speed Lever Arm required for odometer transform");
      return false;
    }

    odometer_link_to_imu_link_transform_.header.stamp = rosTimeNow(node_);
    odometer_link_to_imu_link_transform_.header.frame_id = config_->frame_id_;
    odometer_link_to_imu_link_transform_.child_frame_id = config_->odometer_frame_id_;
    odometer_link_to_imu_link_transform_.transform.rotation = tf2::toMsg(tf2::Quaternion::getIdentity());

    // If running in the ENU frame, transform from our vehicle frame to the ROS vehicle frame
    const tf2::Transform odometer_link_to_microstrain_vehicle_transform_tf(tf2::Quaternion::getIdentity(), tf2::Vector3(speed_lever_arm[0], speed_lever_arm[1], speed_lever_arm[2]));
    if (config_->use_enu_frame_)
    {
      const tf2::Transform odometer_link_to_ros_vehicle_transform_tf = config_->ros_vehicle_to_microstrain_vehicle_transform_tf_.inverse() * odometer_link_to_microstrain_vehicle_transform_tf;
      odometer_link_to_imu_link_transform_.transform.translation.x = odometer_link_to_ros_vehicle_transform_tf.getOrigin().getX();
      odometer_link_to_imu_link_transform_.transform.translation.y = odometer_link_to_ros_vehicle_transform_tf.getOrigin().getY();
      odometer_link_to_imu_link_transform_.transform.translation.z = odometer_link_to_ros_vehicle_transform_tf.getOrigin().getZ();
    }
    else
    {
      odometer_link_to_imu_link_transform_.transform.translation.x = odometer_link_to_microstrain_vehicle_transform_tf.getOrigin().getX();
      odometer_link_to_imu_link_transform_.transform.translation.y = odometer_link_to_microstrain_vehicle_transform_tf.getOrigin().getY();
      odometer_link_to_imu_link_transform_.transform.translation.z = odometer_link_to_microstrain_vehicle_transform_tf.getOrigin().getZ();
    }
  }

  // Register callbacks for each data field we care about. Note that order is preserved here, so if a data field needs to be parsed before another, change it here.
  // Prospect shared field callbacks
  for (const uint8_t descriptor_set : std::initializer_list<uint8_t>{mip::data_sensor::DESCRIPTOR_SET, mip::data_gnss::DESCRIPTOR_SET, mip::data_gnss::MIP_GNSS1_DATA_DESC_SET, mip::data_gnss::MIP_GNSS2_DATA_DESC_SET, mip::data_gnss::MIP_GNSS3_DATA_DESC_SET, mip::data_filter::DESCRIPTOR_SET})
  {
    registerDataCallback<mip::data_shared::EventSource, &Publishers::handleSharedEventSource>(descriptor_set);
    registerDataCallback<mip::data_shared::Ticks, &Publishers::handleSharedTicks>(descriptor_set);
    registerDataCallback<mip::data_shared::DeltaTicks, &Publishers::handleSharedDeltaTicks>(descriptor_set);
    registerDataCallback<mip::data_shared::GpsTimestamp, &Publishers::handleSharedGpsTimestamp>(descriptor_set);
    registerDataCallback<mip::data_shared::DeltaTime, &Publishers::handleSharedDeltaTime>(descriptor_set);
    registerDataCallback<mip::data_shared::ReferenceTimestamp, &Publishers::handleSharedReferenceTimestamp>(descriptor_set);
    registerDataCallback<mip::data_shared::ReferenceTimeDelta, &Publishers::handleSharedReferenceTimeDelta>(descriptor_set);
  }

  // Philo shared field callbacks
  registerDataCallback<mip::data_sensor::GpsTimestamp, &Publishers::handleSensorGpsTimestamp>();
  registerDataCallback<mip::data_gnss::GpsTime, &Publishers::handleGnssGpsTime>();
  registerDataCallback<mip::data_filter::Timestamp, &Publishers::handleFilterTimestamp>();

  // IMU callbacks
  registerDataCallback<mip::data_sensor::ScaledAccel, &Publishers::handleSensorScaledAccel>();
  registerDataCallback<mip::data_sensor::ScaledGyro, &Publishers::handleSensorScaledGyro>();
  registerDataCallback<mip::data_sensor::DeltaTheta, &Publishers::handleSensorDeltaTheta>();
  registerDataCallback<mip::data_sensor::DeltaVelocity, &Publishers::handleSensorDeltaVelocity>();
  registerDataCallback<mip::data_sensor::CompQuaternion, &Publishers::handleSensorCompQuaternion>();
  registerDataCallback<mip::data_sensor::ScaledMag, &Publishers::handleSensorScaledMag>();
  registerDataCallback<mip::data_sensor::ScaledPressure, &Publishers::handleSensorScaledPressure>();
  registerDataCallback<mip::data_sensor::OdometerData, &Publishers::handleSensorOdometerData>();
  registerDataCallback<mip::data_sensor::OverrangeStatus, &Publishers::handleSensorOverrangeStatus>();
  registerDataCallback<mip::data_sensor::TemperatureAbs, &Publishers::handleSensorTemperatureStatistics>();

  // GNSS1/2 callbacks
  for (const uint8_t gnss_descriptor_set : std::initializer_list<uint8_t>{mip::data_gnss::DESCRIPTOR_SET, mip::data_gnss::MIP_GNSS1_DATA_DESC_SET, mip::data_gnss::MIP_GNSS2_DATA_DESC_SET})
  {
    registerDataCallback<mip::data_gnss::PosLlh, &Publishers::handleGnssPosLlh>(gnss_descriptor_set);
    registerDataCallback<mip::data_gnss::VelNed, &Publishers::handleGnssVelNed>(gnss_descriptor_set);
    registerDataCallback<mip::data_gnss::PosEcef, &Publishers::handleGnssPosEcef>(gnss_descriptor_set);
    registerDataCallback<mip::data_gnss::VelEcef, &Publishers::handleGnssVelEcef>(gnss_descriptor_set);
    registerDataCallback<mip::data_gnss::FixInfo, &Publishers::handleGnssFixInfo>(gnss_descriptor_set);
    registerDataCallback<mip::data_gnss::SbasInfo, &Publishers::handleGnssSbasInfo>(gnss_descriptor_set);
    registerDataCallback<mip::data_gnss::RfErrorDetection, &Publishers::handleGnssRfErrorDetection>(gnss_descriptor_set);
  }

  // Note: It is important to make sure this is after the GNSS1/2 callbacks
  for (const uint8_t gnss_descriptor_set : std::initializer_list<uint8_t>{mip::data_gnss::MIP_GNSS1_DATA_DESC_SET, mip::data_gnss::MIP_GNSS2_DATA_DESC_SET})
  {
    registerDataCallback<mip::data_gnss::GpsTime, &Publishers::handleGnssGpsTime>(gnss_descriptor_set);
  }

  // RTK callbacks
  registerDataCallback<mip::data_gnss::RtkCorrectionsStatus, &Publishers::handleRtkCorrectionsStatus>(mip::data_gnss::MIP_GNSS3_DATA_DESC_SET);
  registerDataCallback<mip::data_gnss::BaseStationInfo, &Publishers::handleRtkBaseStationInfo>(mip::data_gnss::MIP_GNSS3_DATA_DESC_SET);

  // Filter callbacks
  registerDataCallback<mip::data_filter::Status, &Publishers::handleFilterStatus>();
  registerDataCallback<mip::data_filter::EcefPos, &Publishers::handleFilterEcefPos>();
  registerDataCallback<mip::data_filter::EcefPosUncertainty, &Publishers::handleFilterEcefPosUncertainty>();
  registerDataCallback<mip::data_filter::PositionLlh, &Publishers::handleFilterPositionLlh>();
  registerDataCallback<mip::data_filter::PositionLlhUncertainty, &Publishers::handleFilterPositionLlhUncertainty>();
  registerDataCallback<mip::data_filter::AttitudeQuaternion, &Publishers::handleFilterAttitudeQuaternion>();
  registerDataCallback<mip::data_filter::EulerAnglesUncertainty, &Publishers::handleFilterEulerAnglesUncertainty>();
  registerDataCallback<mip::data_filter::VelocityNed, &Publishers::handleFilterVelocityNed>();
  registerDataCallback<mip::data_filter::VelocityNedUncertainty, &Publishers::handleFilterVelocityNedUncertainty>();
  registerDataCallback<mip::data_filter::EcefVel, &Publishers::handleFilterEcefVelocity>();
  registerDataCallback<mip::data_filter::EcefVelUncertainty, &Publishers::handleFilterEcefVelocityUncertainty>();
  registerDataCallback<mip::data_filter::CompAngularRate, &Publishers::handleFilterCompAngularRate>();
  registerDataCallback<mip::data_filter::CompAccel, &Publishers::handleFilterCompAccel>();
  registerDataCallback<mip::data_filter::LinearAccel, &Publishers::handleFilterLinearAccel>();
  registerDataCallback<mip::data_filter::GnssPosAidStatus, &Publishers::handleFilterGnssPosAidStatus>();
  registerDataCallback<mip::data_filter::MultiAntennaOffsetCorrection, &Publishers::handleFilterMultiAntennaOffsetCorrection>();
  registerDataCallback<mip::data_filter::GnssDualAntennaStatus, &Publishers::handleFilterGnssDualAntennaStatus>();
  registerDataCallback<mip::data_filter::AidingMeasurementSummary, &Publishers::handleFilterAidingMeasurementSummary>();

  // System callbacks
  registerDataCallback<mip::data_system::BuiltInTest, &Publishers::handleSystemBuiltInTest>();

  // After packet callback
  registerPacketCallback<&Publishers::handleAfterPacket>();
  return true;
}

bool Publishers::activate()
{
  imu_raw_pub_->activate();
  imu_pub_->activate();
  mag_pub_->activate();
  pressure_pub_->activate();
  wheel_speed_pub_->activate();

  for (const auto& pub : gnss_llh_position_pub_) pub->activate();
  for (const auto& pub : gnss_velocity_pub_) pub->activate();
  for (const auto& pub : gnss_velocity_ecef_pub_) pub->activate();
  for (const auto& pub : gnss_odometry_pub_) pub->activate();
  for (const auto& pub : gnss_time_pub_) pub->activate();

  filter_human_readable_status_pub_->activate();
  filter_imu_pub_->activate();
  filter_llh_position_pub_->activate();
  filter_velocity_pub_->activate();
  filter_velocity_ecef_pub_->activate();
  filter_odometry_earth_pub_->activate();
  filter_odometry_map_pub_->activate();
  filter_dual_antenna_heading_pub_->activate();

  mip_sensor_overrange_status_pub_->activate();
  mip_sensor_temperature_statistics_pub_->activate();

  for (const auto& pub : mip_gnss_fix_info_pub_) pub->activate();
  for (const auto& pub : mip_gnss_sbas_info_pub_) pub->activate();
  for (const auto& pub : mip_gnss_rf_error_detection_pub_) pub->activate();

  mip_gnss_corrections_rtk_corrections_status_pub_->activate();

  mip_filter_status_pub_->activate();
  mip_filter_gnss_position_aiding_status_pub_->activate();
  mip_filter_multi_antenna_offset_correction_pub_->activate();
  mip_filter_aiding_measurement_summary_pub_->activate();
  mip_filter_gnss_dual_antenna_status_pub_->activate();

  mip_system_built_in_test_pub_->activate();

  nmea_sentence_pub_->activate();

  // Publish the static transforms
  if (config_->tf_mode_ != TF_MODE_OFF && config_->filter_relative_pos_config_ && config_->filter_relative_pos_source_ == REL_POS_SOURCE_MANUAL)
    static_transform_broadcaster_->sendTransform(config_->map_to_earth_transform_);
  if (config_->publish_mount_to_frame_id_transform_)
    static_transform_broadcaster_->sendTransform(config_->mount_to_frame_id_transform_);

  // Static antenna offsets
  // Note: If streaming the antenna offset correction topic, correct the offsets with them
  if (config_->gnss_antenna_offset_source_[GNSS1_ID] == OFFSET_SOURCE_MANUAL)
    if (config_->mip_device_->supportsDescriptorSet(mip::data_gnss::DESCRIPTOR_SET) || config_->mip_device_->supportsDescriptorSet(mip::data_gnss::MIP_GNSS1_DATA_DESC_SET))
      static_transform_broadcaster_->sendTransform(gnss_antenna_link_to_imu_link_transform_[GNSS1_ID]);
  if (config_->gnss_antenna_offset_source_[GNSS2_ID] == OFFSET_SOURCE_MANUAL)
    if (config_->mip_device_->supportsDescriptorSet(mip::data_gnss::MIP_GNSS2_DATA_DESC_SET))
      static_transform_broadcaster_->sendTransform(gnss_antenna_link_to_imu_link_transform_[GNSS2_ID]);
  if (config_->filter_speed_lever_arm_source_ == OFFSET_SOURCE_MANUAL)
    if (config_->mip_device_->supportsDescriptor(mip::commands_filter::DESCRIPTOR_SET, mip::commands_filter::CMD_SPEED_LEVER_ARM))
      static_transform_broadcaster_->sendTransform(odometer_link_to_imu_link_transform_);
  return true;
}

bool Publishers::deactivate()
{
  imu_raw_pub_->deactivate();
  imu_pub_->deactivate();
  mag_pub_->deactivate();
  pressure_pub_->deactivate();
  wheel_speed_pub_->deactivate();

  for (const auto& pub : gnss_llh_position_pub_) pub->deactivate();
  for (const auto& pub : gnss_velocity_pub_) pub->deactivate();
  for (const auto& pub : gnss_velocity_ecef_pub_) pub->deactivate();
  for (const auto& pub : gnss_odometry_pub_) pub->deactivate();
  for (const auto& pub : gnss_time_pub_) pub->deactivate();

  filter_human_readable_status_pub_->deactivate();
  filter_imu_pub_->deactivate();
  filter_llh_position_pub_->deactivate();
  filter_odometry_earth_pub_->deactivate();
  filter_odometry_map_pub_->deactivate();
  filter_dual_antenna_heading_pub_->deactivate();

  mip_sensor_overrange_status_pub_->deactivate();
  mip_sensor_temperature_statistics_pub_->deactivate();

  for (const auto& pub : mip_gnss_fix_info_pub_) pub->deactivate();
  for (const auto& pub : mip_gnss_sbas_info_pub_) pub->deactivate();
  for (const auto& pub : mip_gnss_rf_error_detection_pub_) pub->deactivate();

  mip_gnss_corrections_rtk_corrections_status_pub_->deactivate();

  mip_filter_status_pub_->deactivate();
  mip_filter_gnss_position_aiding_status_pub_->deactivate();
  mip_filter_multi_antenna_offset_correction_pub_->deactivate();
  mip_filter_aiding_measurement_summary_pub_->deactivate();
  mip_filter_gnss_dual_antenna_status_pub_->deactivate();

  mip_system_built_in_test_pub_->deactivate();

  nmea_sentence_pub_->deactivate();
  return true;
}

void Publishers::publish()
{
  // This publish function will get called after each packet is processed.
  // For standard ROS messages this allows us to combine multiple MIP fields and then publish them
  // For custom ROS messages, the messages are published directly in the callbacks
  imu_raw_pub_->publish();
  imu_pub_->publish();
  mag_pub_->publish();
  pressure_pub_->publish();
  wheel_speed_pub_->publish();

  for (const auto& pub : gnss_llh_position_pub_) pub->publish();
  for (const auto& pub : gnss_velocity_pub_) pub->publish();
  for (const auto& pub : gnss_velocity_ecef_pub_) pub->publish();
  for (const auto& pub : gnss_odometry_pub_) pub->publish();
  for (const auto& pub : gnss_time_pub_) pub->publish();

  filter_human_readable_status_pub_->publish();
  filter_imu_pub_->publish();
  filter_llh_position_pub_->publish();
  filter_velocity_pub_->publish();
  filter_velocity_ecef_pub_->publish();
  filter_odometry_earth_pub_->publish();
  filter_odometry_map_pub_->publish();
  filter_dual_antenna_heading_pub_->publish();

  // Publish the dynamic transforms after the messages have been filled out
  std::string tf_error_string;
  RosTimeType frame_time; setRosTime(&frame_time, 0, 0);
  if (config_->tf_mode_ == TF_MODE_GLOBAL && imu_link_to_earth_transform_translation_updated_ && imu_link_to_earth_transform_attitude_updated_)
  {
    if (transform_buffer_->canTransform(config_->target_frame_id_, config_->frame_id_, frame_time, RosDurationType(0, 0), &tf_error_string))
    {
      tf2::Transform imu_link_to_target_transform_tf;
      const auto& imu_link_to_target_transform = transform_buffer_->lookupTransform(config_->target_frame_id_, config_->frame_id_, frame_time, RosDurationType(0, 0));
      tf2::fromMsg(imu_link_to_target_transform.transform, imu_link_to_target_transform_tf);

      const tf2::Transform& target_to_earth_transform_tf = imu_link_to_earth_transform_tf_stamped_ * imu_link_to_target_transform_tf.inverse();

      TransformStampedMsg target_to_earth_transform;
      target_to_earth_transform.header.stamp = tf2_ros::toMsg(imu_link_to_earth_transform_tf_stamped_.stamp_);
      target_to_earth_transform.header.frame_id = config_->earth_frame_id_;
      target_to_earth_transform.child_frame_id = config_->target_frame_id_;
      target_to_earth_transform.transform = tf2::toMsg(target_to_earth_transform_tf);

      // Publish and reset the booleans
      transform_broadcaster_->sendTransform(target_to_earth_transform);
      imu_link_to_earth_transform_translation_updated_ = false;
      imu_link_to_earth_transform_attitude_updated_ = false;
    }
    else
    {
      MICROSTRAIN_WARN_THROTTLE(node_, 2, "Unable to lookup transform from %s to %s: %s", config_->target_frame_id_.c_str(), config_->frame_id_.c_str(), tf_error_string.c_str());
    }
  }

  else if (config_->tf_mode_ == TF_MODE_RELATIVE && imu_link_to_map_transform_translation_updated_ && imu_link_to_map_transform_attitude_updated_)
  {
    if (transform_buffer_->canTransform(config_->target_frame_id_, config_->frame_id_, frame_time, RosDurationType(0, 0), &tf_error_string))
    {
      tf2::Transform imu_link_to_target_transform_tf;
      const auto& imu_link_to_target_transform = transform_buffer_->lookupTransform(config_->target_frame_id_, config_->frame_id_, frame_time, RosDurationType(0, 0));
      tf2::fromMsg(imu_link_to_target_transform.transform, imu_link_to_target_transform_tf);

      const tf2::Transform& target_to_map_transform_tf = imu_link_to_map_transform_tf_stamped_ * imu_link_to_target_transform_tf.inverse();

      TransformStampedMsg target_to_map_transform;
      target_to_map_transform.header.stamp = tf2_ros::toMsg(imu_link_to_map_transform_tf_stamped_.stamp_);
      target_to_map_transform.header.frame_id = config_->map_frame_id_;
      target_to_map_transform.child_frame_id = config_->target_frame_id_;
      target_to_map_transform.transform = tf2::toMsg(target_to_map_transform_tf);

      // Publish and reset the booleans
      transform_broadcaster_->sendTransform(target_to_map_transform);
      imu_link_to_map_transform_translation_updated_ = false;
      imu_link_to_map_transform_attitude_updated_ = false;
    }
    else
    {
      MICROSTRAIN_WARN_THROTTLE(node_, 2, "Unable to lookup transform from %s to %s: %s", config_->target_frame_id_.c_str(), config_->frame_id_.c_str(), tf_error_string.c_str());
    }
  }

  if (config_->tf_mode_ != TF_MODE_OFF)
  {
    if (config_->map_to_earth_transform_updated_)
    {
      // Send as a static transform since the transform should get updated pretty rarely
      static_transform_broadcaster_->sendTransform(config_->map_to_earth_transform_);
      config_->map_to_earth_transform_updated_ = false;
    }
  }
}

void Publishers::handleSharedEventSource(const mip::data_shared::EventSource& event_source, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  event_source_mapping_[descriptor_set] = event_source;
}

void Publishers::handleSharedTicks(const mip::data_shared::Ticks& ticks, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  ticks_mapping_[descriptor_set] = ticks;
}

void Publishers::handleSharedDeltaTicks(const mip::data_shared::DeltaTicks& delta_ticks, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  delta_ticks_mapping_[descriptor_set] = delta_ticks;
}

void Publishers::handleSharedGpsTimestamp(const mip::data_shared::GpsTimestamp& gps_timestamp, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  // If the timestamp came from the sensor descriptor set, update the clock bias monitor
  if (descriptor_set == mip::data_sensor::DESCRIPTOR_SET)
  {
    const double collected_timestamp_secs = static_cast<double>(timestamp) / 1000.0;
    clock_bias_monitor_.addTime(gpsTimestampSecs(gps_timestamp), collected_timestamp_secs);
  }

  // Save the GPS timestamp
  gps_timestamp_mapping_[descriptor_set] = gps_timestamp;

  // Update the GPS time message for this descriptor
  uint8_t gnss_index;
  switch (descriptor_set)
  {
    case mip::data_gnss::MIP_GNSS1_DATA_DESC_SET:
      gnss_index = 0;
      break;
    case mip::data_gnss::MIP_GNSS2_DATA_DESC_SET:
      gnss_index = 1;
      break;
    default:
      return;
  }
  auto gps_time_msg = gnss_time_pub_[gnss_index]->getMessageToUpdate();
  gps_time_msg->header.stamp = rosTimeNow(node_);
  setGpsTime(&gps_time_msg->time_ref, gps_timestamp);
}

void Publishers::handleSharedDeltaTime(const mip::data_shared::DeltaTime& delta_time, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  delta_time_mapping_[descriptor_set] = delta_time;
}

void Publishers::handleSharedReferenceTimestamp(const mip::data_shared::ReferenceTimestamp& reference_timestamp, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  reference_timestamp_mapping_[descriptor_set] = reference_timestamp;
}

void Publishers::handleSharedReferenceTimeDelta(const mip::data_shared::ReferenceTimeDelta& reference_time_delta, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  reference_time_delta_mapping_[descriptor_set] = reference_time_delta;
}

void Publishers::handleSensorGpsTimestamp(const mip::data_sensor::GpsTimestamp& gps_timestamp, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  // Convert the old philo timestamp into the new format and store it in the map
  mip::data_shared::GpsTimestamp stored_timestamp;
  stored_timestamp.tow = gps_timestamp.tow;
  stored_timestamp.week_number = gps_timestamp.week_number;
  stored_timestamp.valid_flags = gps_timestamp.valid_flags;
  handleSharedGpsTimestamp(stored_timestamp, descriptor_set, timestamp);
}

void Publishers::handleSensorScaledAccel(const mip::data_sensor::ScaledAccel& scaled_accel, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  auto imu_raw_msg = imu_raw_pub_->getMessageToUpdate();
  updateHeaderTime(&(imu_raw_msg->header), descriptor_set, timestamp);
  imu_raw_msg->linear_acceleration.x = USTRAIN_G * scaled_accel.scaled_accel[0];
  imu_raw_msg->linear_acceleration.y = USTRAIN_G * scaled_accel.scaled_accel[1];
  imu_raw_msg->linear_acceleration.z = USTRAIN_G * scaled_accel.scaled_accel[2];
  if (config_->use_enu_frame_)
  {
    imu_raw_msg->linear_acceleration.y *= -1.0;
    imu_raw_msg->linear_acceleration.z *= -1.0;
  }
}

void Publishers::handleSensorScaledGyro(const mip::data_sensor::ScaledGyro& scaled_gyro, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  auto imu_raw_msg = imu_raw_pub_->getMessageToUpdate();
  updateHeaderTime(&(imu_raw_msg->header), descriptor_set, timestamp);
  imu_raw_msg->angular_velocity.x = scaled_gyro.scaled_gyro[0];
  imu_raw_msg->angular_velocity.y = scaled_gyro.scaled_gyro[1];
  imu_raw_msg->angular_velocity.z = scaled_gyro.scaled_gyro[2];
  if (config_->use_enu_frame_)
  {
    imu_raw_msg->angular_velocity.y *= -1.0;
    imu_raw_msg->angular_velocity.z *= -1.0;
  }
}

void Publishers::handleSensorDeltaTheta(const mip::data_sensor::DeltaTheta& delta_theta, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  // Attempt to get the delta time, if we can't find it we will estimate based on the data rate
  float delta_time;
  if (delta_time_mapping_.find(descriptor_set) != delta_time_mapping_.end())
    delta_time = delta_time_mapping_.at(descriptor_set).seconds;
  else
    delta_time = 1 / imu_pub_->dataRate();

  // We use this delta measurement to exclude outliers and provide a more useful IMU measurement
  auto imu_msg = imu_pub_->getMessageToUpdate();
  updateHeaderTime(&(imu_msg->header), descriptor_set, timestamp);
  imu_msg->angular_velocity.x = delta_theta.delta_theta[0] / delta_time;
  imu_msg->angular_velocity.y = delta_theta.delta_theta[1] / delta_time;
  imu_msg->angular_velocity.z = delta_theta.delta_theta[2] / delta_time;
  if (config_->use_enu_frame_)
  {
    imu_msg->angular_velocity.y *= -1.0;
    imu_msg->angular_velocity.z *= -1.0;
  }
}

void Publishers::handleSensorDeltaVelocity(const mip::data_sensor::DeltaVelocity& delta_velocity, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  // Attempt to get the delta time, if we can't find it we will estimate based on the data rate
  float delta_time;
  if (delta_time_mapping_.find(descriptor_set) != delta_time_mapping_.end())
    delta_time = delta_time_mapping_.at(descriptor_set).seconds;
  else
    delta_time = 1 / imu_pub_->dataRate();

  auto imu_msg = imu_pub_->getMessageToUpdate();
  updateHeaderTime(&(imu_msg->header), descriptor_set, timestamp);
  imu_msg->linear_acceleration.x = USTRAIN_G * delta_velocity.delta_velocity[0] / delta_time;
  imu_msg->linear_acceleration.y = USTRAIN_G * delta_velocity.delta_velocity[1] / delta_time;
  imu_msg->linear_acceleration.z = USTRAIN_G * delta_velocity.delta_velocity[2] / delta_time;
  if (config_->use_enu_frame_)
  {
    imu_msg->linear_acceleration.y *= -1.0;
    imu_msg->linear_acceleration.z *= -1.0;
  }
}

void Publishers::handleSensorCompQuaternion(const mip::data_sensor::CompQuaternion& comp_quaternion, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  // Rotate the quaternion into the correct frame
  auto imu_msg = imu_pub_->getMessageToUpdate();
  updateHeaderTime(&(imu_msg->header), descriptor_set, timestamp);
  const tf2::Transform microstrain_vehicle_to_ned_transform_tf(tf2::Quaternion(comp_quaternion.q[1], comp_quaternion.q[2], comp_quaternion.q[3], comp_quaternion.q[0]));
  if (config_->use_enu_frame_)
  {
    const tf2::Transform ros_vehicle_to_enu_transform_tf = config_->ned_to_enu_transform_tf_ * microstrain_vehicle_to_ned_transform_tf * config_->ros_vehicle_to_microstrain_vehicle_transform_tf_;
    imu_msg->orientation = tf2::toMsg(ros_vehicle_to_enu_transform_tf.getRotation());
  }
  else
  {
    imu_msg->orientation = tf2::toMsg(microstrain_vehicle_to_ned_transform_tf.getRotation());
  }
}

void Publishers::handleSensorScaledMag(const mip::data_sensor::ScaledMag& scaled_mag, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  auto mag_msg = mag_pub_->getMessageToUpdate();
  updateHeaderTime(&(mag_msg->header), descriptor_set, timestamp);
  mag_msg->magnetic_field.x = scaled_mag.scaled_mag[0] / 10000;
  mag_msg->magnetic_field.y = scaled_mag.scaled_mag[1] / 10000;
  mag_msg->magnetic_field.z = scaled_mag.scaled_mag[2] / 10000;
  if (config_->use_enu_frame_)
  {
    mag_msg->magnetic_field.y *= -1.0;
    mag_msg->magnetic_field.z *= -1.0;
  }
}

void Publishers::handleSensorScaledPressure(const mip::data_sensor::ScaledPressure& scaled_pressure, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  auto pressure_msg = pressure_pub_->getMessageToUpdate();
  updateHeaderTime(&(pressure_msg->header), descriptor_set, timestamp);
  pressure_msg->fluid_pressure = scaled_pressure.scaled_pressure * 100;
}

void Publishers::handleSensorOdometerData(const mip::data_sensor::OdometerData& odometer_data, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  auto wheel_speed_msg = wheel_speed_pub_->getMessageToUpdate();
  updateHeaderTime(&(wheel_speed_msg->header), descriptor_set, timestamp);

  if (odometer_data.valid_flags == 1)
  {
    wheel_speed_msg->twist.twist.linear.x = odometer_data.speed;
    wheel_speed_msg->twist.covariance[0] = pow(odometer_data.uncertainty, 2);
  }
  else
  {
    MICROSTRAIN_WARN_ONCE(node_, "Wheel speed data is invalid. The odometer was likely not configured.");
    MICROSTRAIN_WARN_ONCE(node_, "  If you want wheel speed, make sure to configure the 'Hardware Odometer Control' section of the params file");
  }
}

void Publishers::handleSensorOverrangeStatus(const mip::data_sensor::OverrangeStatus& overrange_status, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  auto mip_sensor_overrange_status_msg = mip_sensor_overrange_status_pub_->getMessage();
  updateMipHeader(&(mip_sensor_overrange_status_msg->header), descriptor_set, timestamp);
  mip_sensor_overrange_status_msg->status.accel_x = overrange_status.status.accelX();
  mip_sensor_overrange_status_msg->status.accel_y = overrange_status.status.accelY();
  mip_sensor_overrange_status_msg->status.accel_z = overrange_status.status.accelZ();
  mip_sensor_overrange_status_msg->status.gyro_x = overrange_status.status.gyroX();
  mip_sensor_overrange_status_msg->status.gyro_y = overrange_status.status.gyroY();
  mip_sensor_overrange_status_msg->status.gyro_z = overrange_status.status.gyroZ();
  mip_sensor_overrange_status_msg->status.mag_x = overrange_status.status.magX();
  mip_sensor_overrange_status_msg->status.mag_y = overrange_status.status.magY();
  mip_sensor_overrange_status_msg->status.mag_z = overrange_status.status.magZ();
  mip_sensor_overrange_status_msg->status.press = overrange_status.status.press();
  mip_sensor_overrange_status_pub_->publish(*mip_sensor_overrange_status_msg);
}

void Publishers::handleSensorTemperatureStatistics(const mip::data_sensor::TemperatureAbs& temperature_statistics, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  auto mip_sensor_temperature_statistics_msg = mip_sensor_temperature_statistics_pub_->getMessage();
  updateMipHeader(&(mip_sensor_temperature_statistics_msg->header), descriptor_set, timestamp);
  mip_sensor_temperature_statistics_msg->min_temp = temperature_statistics.min_temp;
  mip_sensor_temperature_statistics_msg->max_temp = temperature_statistics.max_temp;
  mip_sensor_temperature_statistics_msg->mean_temp = temperature_statistics.mean_temp;
  mip_sensor_temperature_statistics_pub_->publish(*mip_sensor_temperature_statistics_msg);
}

void Publishers::handleGnssGpsTime(const mip::data_gnss::GpsTime& gps_time, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  // Convert the old philo timestamp into the new format and store it in the map
  mip::data_shared::GpsTimestamp stored_timestamp;
  stored_timestamp.tow = gps_time.tow;
  stored_timestamp.week_number = gps_time.week_number;
  stored_timestamp.valid_flags = gps_time.valid_flags;
  gps_timestamp_mapping_[descriptor_set] = stored_timestamp;

  // Also update the time ref messages
  uint8_t gnss_index;
  switch (descriptor_set)
  {
    case mip::data_gnss::DESCRIPTOR_SET:
    case mip::data_gnss::MIP_GNSS1_DATA_DESC_SET:
      gnss_index = 0;
      break;
    case mip::data_gnss::MIP_GNSS2_DATA_DESC_SET:
      gnss_index = 1;
      break;
    default:
      return;
  }
  auto gps_time_msg = gnss_time_pub_[gnss_index]->getMessageToUpdate();
  gps_time_msg->header.stamp = rosTimeNow(node_);
  setGpsTime(&gps_time_msg->time_ref, stored_timestamp);
}

void Publishers::handleGnssPosLlh(const mip::data_gnss::PosLlh& pos_llh, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  // Different message depending on the descriptor set
  const uint8_t gnss_index = (descriptor_set == mip::data_gnss::DESCRIPTOR_SET || descriptor_set == mip::data_gnss::MIP_GNSS1_DATA_DESC_SET) ? GNSS1_ID : GNSS2_ID;

  // GNSS navsatfix message
  auto gnss_llh_position_msg = gnss_llh_position_pub_[gnss_index]->getMessageToUpdate();
  updateHeaderTime(&(gnss_llh_position_msg->header), descriptor_set, timestamp);
  gnss_llh_position_msg->status.service = 1;
  gnss_llh_position_msg->position_covariance_type = 2;
  gnss_llh_position_msg->latitude = pos_llh.latitude;
  gnss_llh_position_msg->longitude = pos_llh.longitude;
  gnss_llh_position_msg->altitude = pos_llh.ellipsoid_height;
  gnss_llh_position_msg->position_covariance[0] = pow(pos_llh.horizontal_accuracy, 2);
  gnss_llh_position_msg->position_covariance[4] = pow(pos_llh.horizontal_accuracy, 2);
  gnss_llh_position_msg->position_covariance[8] = pow(pos_llh.vertical_accuracy, 2);
}

void Publishers::handleGnssVelNed(const mip::data_gnss::VelNed& vel_ned, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  // Different message depending on the descriptor set
  const uint8_t gnss_index = (descriptor_set == mip::data_gnss::DESCRIPTOR_SET || descriptor_set == mip::data_gnss::MIP_GNSS1_DATA_DESC_SET) ? GNSS1_ID : GNSS2_ID;

  // GNSS velocity message
  auto gnss_velocity_msg = gnss_velocity_pub_[gnss_index]->getMessageToUpdate();
  updateHeaderTime(&(gnss_velocity_msg->header), descriptor_set, timestamp);
  if (config_->use_enu_frame_)
  {
    gnss_velocity_msg->twist.twist.linear.x = vel_ned.v[1];
    gnss_velocity_msg->twist.twist.linear.y = vel_ned.v[0];
    gnss_velocity_msg->twist.twist.linear.z = -vel_ned.v[2];
  }
  else
  {
    gnss_velocity_msg->twist.twist.linear.x = vel_ned.v[0];
    gnss_velocity_msg->twist.twist.linear.y = vel_ned.v[1];
    gnss_velocity_msg->twist.twist.linear.z = vel_ned.v[2];
  }
  double velocity_covariance = pow(vel_ned.speed_accuracy, 2);
  gnss_velocity_msg->twist.covariance[0] = velocity_covariance;
  gnss_velocity_msg->twist.covariance[7] = velocity_covariance;
  gnss_velocity_msg->twist.covariance[14] = velocity_covariance;

  // GNSS odometry message (not counted as updating)
  auto gnss_odometry_msg = gnss_odometry_pub_[gnss_index]->getMessage();

  // Convert the ECEF coordinates to LLH so we can lookup the rotation
  double lat, lon, alt;
  config_->geocentric_converter_.Reverse(gnss_odometry_msg->pose.pose.position.x, gnss_odometry_msg->pose.pose.position.y, gnss_odometry_msg->pose.pose.position.z, lat, lon, alt);

  // Convert the course over ground to the correct frame
  tf2::Quaternion ned_to_microstrain_vehicle_quaternion_tf;
  ned_to_microstrain_vehicle_quaternion_tf.setRPY(0, 0, vel_ned.heading * M_PI / 180);
  const tf2::Transform ned_to_microstrain_vehicle_transform_tf(ned_to_microstrain_vehicle_quaternion_tf);
  if (config_->use_enu_frame_)
  {
    const tf2::Transform earth_to_enu_transform_tf(ecefToEnuTransform(lat, lon));
    const tf2::Transform ros_vehicle_to_earth_transform_tf = earth_to_enu_transform_tf.inverse() * config_->ned_to_enu_transform_tf_ * ned_to_microstrain_vehicle_transform_tf.inverse() * config_->ros_vehicle_to_microstrain_vehicle_transform_tf_;
    gnss_odometry_msg->pose.pose.orientation = tf2::toMsg(ros_vehicle_to_earth_transform_tf.getRotation());
  }
  else
  {
    const tf2::Transform earth_to_ned_transform_tf(ecefToNedTransform(lat, lon));
    const tf2::Transform microstrain_vehicle_to_earth_transform_tf = earth_to_ned_transform_tf.inverse() * ned_to_microstrain_vehicle_transform_tf.inverse();
    gnss_odometry_msg->pose.pose.orientation = tf2::toMsg(microstrain_vehicle_to_earth_transform_tf.getRotation());
  }
  gnss_odometry_msg->pose.covariance[35] = vel_ned.heading_accuracy;

  // Rotate the velocity to the sensor frame for the odometry message
  const tf2::Vector3 imu_velocity_in_ned_frame(vel_ned.v[0], vel_ned.v[1], vel_ned.v[2]);
  if (config_->use_enu_frame_)
  {
    const tf2::Vector3 imu_velocity_in_ros_vehicle_frame = config_->ros_vehicle_to_microstrain_vehicle_transform_tf_.inverse() * ned_to_microstrain_vehicle_transform_tf * imu_velocity_in_ned_frame;
    gnss_odometry_msg->twist.twist.linear.x = imu_velocity_in_ros_vehicle_frame.getX();
    gnss_odometry_msg->twist.twist.linear.y = imu_velocity_in_ros_vehicle_frame.getY();
    gnss_odometry_msg->twist.twist.linear.z = imu_velocity_in_ros_vehicle_frame.getZ();
  }
  else
  {
    const tf2::Vector3 imu_velocity_in_microstrain_vehicle_frame = ned_to_microstrain_vehicle_transform_tf * imu_velocity_in_ned_frame;
    gnss_odometry_msg->twist.twist.linear.x = imu_velocity_in_microstrain_vehicle_frame.getX();
    gnss_odometry_msg->twist.twist.linear.y = imu_velocity_in_microstrain_vehicle_frame.getY();
    gnss_odometry_msg->twist.twist.linear.z = imu_velocity_in_microstrain_vehicle_frame.getZ();
  }
  gnss_odometry_msg->twist.covariance = gnss_velocity_msg->twist.covariance;
}

void Publishers::handleGnssPosEcef(const mip::data_gnss::PosEcef& pos_ecef, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  // Different message depending on the descriptor set
  const uint8_t gnss_index = (descriptor_set == mip::data_gnss::DESCRIPTOR_SET || descriptor_set == mip::data_gnss::MIP_GNSS1_DATA_DESC_SET) ? GNSS1_ID : GNSS2_ID;
  auto gnss_odometry_msg = gnss_odometry_pub_[gnss_index]->getMessageToUpdate();
  updateHeaderTime(&(gnss_odometry_msg->header), descriptor_set, timestamp);
  gnss_odometry_msg->pose.pose.position.x = pos_ecef.x[0];
  gnss_odometry_msg->pose.pose.position.y = pos_ecef.x[1];
  gnss_odometry_msg->pose.pose.position.z = pos_ecef.x[2];
  gnss_odometry_msg->pose.covariance[0] = pow(pos_ecef.x_accuracy, 2);
  gnss_odometry_msg->pose.covariance[7] = pow(pos_ecef.x_accuracy, 2);
  gnss_odometry_msg->pose.covariance[14] = pow(pos_ecef.x_accuracy, 2);
}

void Publishers::handleGnssVelEcef(const mip::data_gnss::VelEcef& vel_ecef, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  // Different message depending on the descriptor set
  const uint8_t gnss_index = (descriptor_set == mip::data_gnss::DESCRIPTOR_SET || descriptor_set == mip::data_gnss::MIP_GNSS1_DATA_DESC_SET) ? GNSS1_ID : GNSS2_ID;
  auto gnss_velocity_ecef_msg = gnss_velocity_ecef_pub_[gnss_index]->getMessageToUpdate();
  updateHeaderTime(&(gnss_velocity_ecef_msg->header), descriptor_set, timestamp);
  gnss_velocity_ecef_msg->twist.twist.linear.x = vel_ecef.v[0];
  gnss_velocity_ecef_msg->twist.twist.linear.y = vel_ecef.v[1];
  gnss_velocity_ecef_msg->twist.twist.linear.z = vel_ecef.v[2];
  gnss_velocity_ecef_msg->twist.covariance[0] = pow(vel_ecef.v_accuracy, 2);
  gnss_velocity_ecef_msg->twist.covariance[7] = pow(vel_ecef.v_accuracy, 2);
  gnss_velocity_ecef_msg->twist.covariance[14] = pow(vel_ecef.v_accuracy, 2);
}

void Publishers::handleGnssFixInfo(const mip::data_gnss::FixInfo& fix_info, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  // Different message depending on the descriptor set
  const uint8_t gnss_index = (descriptor_set == mip::data_gnss::DESCRIPTOR_SET || descriptor_set == mip::data_gnss::MIP_GNSS1_DATA_DESC_SET) ? GNSS1_ID : GNSS2_ID;

  // GNSS Fix info message
  auto mip_gnss_fix_info_msg = mip_gnss_fix_info_pub_[gnss_index]->getMessage();
  updateMipHeader(&(mip_gnss_fix_info_msg->header), descriptor_set, timestamp);
  mip_gnss_fix_info_msg->fix_type = static_cast<uint8_t>(fix_info.fix_type);
  mip_gnss_fix_info_msg->num_sv = fix_info.num_sv;
  mip_gnss_fix_info_msg->fix_flags.sbas_used = fix_info.fix_flags & mip::data_gnss::FixInfo::FixFlags::SBAS_USED;
  mip_gnss_fix_info_msg->fix_flags.dgnss_used = fix_info.fix_flags & mip::data_gnss::FixInfo::FixFlags::DGNSS_USED;
  mip_gnss_fix_info_pub_[gnss_index]->publish(*mip_gnss_fix_info_msg);

  // GNSS fix message (not counted as updating)
  auto gnss_llh_position_msg = gnss_llh_position_pub_[gnss_index]->getMessage();
  if (fix_info.fix_type == mip::data_gnss::FixInfo::FixType::FIX_RTK_FIXED || fix_info.fix_type == mip::data_gnss::FixInfo::FixType::FIX_RTK_FLOAT)
    gnss_llh_position_msg->status.status = NavSatFixMsg::_status_type::STATUS_GBAS_FIX;
  else if (fix_info.fix_flags.sbasUsed())
    gnss_llh_position_msg->status.status = NavSatFixMsg::_status_type::STATUS_SBAS_FIX;
  else if (fix_info.fix_type == mip::data_gnss::FixInfo::FixType::FIX_3D)
    gnss_llh_position_msg->status.status = NavSatFixMsg::_status_type::STATUS_FIX;
  else
    gnss_llh_position_msg->status.status = NavSatFixMsg::_status_type::STATUS_NO_FIX;
}

void Publishers::handleGnssRfErrorDetection(const mip::data_gnss::RfErrorDetection& rf_error_detection, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  // Find the right index for the message
  uint8_t gnss_index;
  switch (descriptor_set)
  {
    case mip::data_gnss::MIP_GNSS1_DATA_DESC_SET:
      gnss_index = 0;
      break;
    case mip::data_gnss::MIP_GNSS2_DATA_DESC_SET:
      gnss_index = 1;
      break;
    default:
      return;  // Nothing to do if the descriptor set is not something we recognize
  }

  // Different message depending on the descriptor set
  auto mip_gnss_rf_error_detection_msg = mip_gnss_rf_error_detection_pub_[gnss_index]->getMessage();
  updateMipHeader(&(mip_gnss_rf_error_detection_msg->header), descriptor_set, timestamp);
  mip_gnss_rf_error_detection_msg->rf_band = static_cast<uint8_t>(rf_error_detection.rf_band);
  mip_gnss_rf_error_detection_msg->jamming_state = static_cast<uint8_t>(rf_error_detection.jamming_state);
  mip_gnss_rf_error_detection_msg->spoofing_state = static_cast<uint8_t>(rf_error_detection.spoofing_state);
  mip_gnss_rf_error_detection_pub_[gnss_index]->publish(*mip_gnss_rf_error_detection_msg);
}

void Publishers::handleGnssSbasInfo(const mip::data_gnss::SbasInfo& sbas_info, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  // Find the right index for the message
  uint8_t gnss_index;
  switch (descriptor_set)
  {
    case mip::data_gnss::MIP_GNSS1_DATA_DESC_SET:
      gnss_index = 0;
      break;
    case mip::data_gnss::MIP_GNSS2_DATA_DESC_SET:
      gnss_index = 1;
      break;
    default:
      return;  // Nothing to do if the descriptor set is not something we recognize
  }

  // Different message depending on descriptor
  auto mip_gnss_sbas_info_msg = mip_gnss_sbas_info_pub_[gnss_index]->getMessage();
  updateMipHeader(&(mip_gnss_sbas_info_msg->header), descriptor_set, timestamp);
  mip_gnss_sbas_info_msg->time_of_week = sbas_info.time_of_week;
  mip_gnss_sbas_info_msg->week_number = sbas_info.week_number;
  mip_gnss_sbas_info_msg->sbas_system = static_cast<uint8_t>(sbas_info.sbas_system);
  mip_gnss_sbas_info_msg->sbas_id = sbas_info.sbas_id;
  mip_gnss_sbas_info_msg->count = sbas_info.count;
  mip_gnss_sbas_info_msg->sbas_status.range_available = sbas_info.sbas_status.rangeAvailable();
  mip_gnss_sbas_info_msg->sbas_status.corrections_available = sbas_info.sbas_status.correctionsAvailable();
  mip_gnss_sbas_info_msg->sbas_status.integrity_available = sbas_info.sbas_status.integrityAvailable();
  mip_gnss_sbas_info_msg->sbas_status.test_mode = sbas_info.sbas_status.testMode();
  mip_gnss_sbas_info_pub_[gnss_index]->publish(*mip_gnss_sbas_info_msg);
}

void Publishers::handleRtkCorrectionsStatus(const mip::data_gnss::RtkCorrectionsStatus& rtk_corrections_status, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  const mip::commands_rtk::GetStatusFlags::StatusFlags dongle_status(rtk_corrections_status.dongle_status);
  if (0 == dongle_status.version()) {
    MICROSTRAIN_WARN_ONCE(node_, "RTK Dongle version 0 is unsupported. RtkCorrectionsStatus fields may be invalid.");
  }
  auto mip_gnss_corrections_rtk_corrections_status_msg = mip_gnss_corrections_rtk_corrections_status_pub_->getMessage();
  updateMipHeader(&(mip_gnss_corrections_rtk_corrections_status_msg->header), descriptor_set, timestamp);
  mip_gnss_corrections_rtk_corrections_status_msg->time_of_week = rtk_corrections_status.time_of_week;
  mip_gnss_corrections_rtk_corrections_status_msg->week_number = rtk_corrections_status.week_number;

  mip_gnss_corrections_rtk_corrections_status_msg->epoch_status.antenna_location_received = rtk_corrections_status.epoch_status.antennaLocationReceived();
  mip_gnss_corrections_rtk_corrections_status_msg->epoch_status.antenna_description_received = rtk_corrections_status.epoch_status.antennaDescriptionReceived();
  mip_gnss_corrections_rtk_corrections_status_msg->epoch_status.gps_received = rtk_corrections_status.epoch_status.gpsReceived();
  mip_gnss_corrections_rtk_corrections_status_msg->epoch_status.galileo_received = rtk_corrections_status.epoch_status.galileoReceived();
  mip_gnss_corrections_rtk_corrections_status_msg->epoch_status.glonass_received = rtk_corrections_status.epoch_status.glonassReceived();
  mip_gnss_corrections_rtk_corrections_status_msg->epoch_status.dongle_status_read_failed = rtk_corrections_status.epoch_status.dongleStatusReadFailed();

  mip_gnss_corrections_rtk_corrections_status_msg->dongle_status.modem_state = dongle_status.modemState();
  mip_gnss_corrections_rtk_corrections_status_msg->dongle_status.connection_type = dongle_status.connectionType();
  mip_gnss_corrections_rtk_corrections_status_msg->dongle_status.rssi = -1 * dongle_status.rssi();
  mip_gnss_corrections_rtk_corrections_status_msg->dongle_status.signal_quality = dongle_status.signalQuality();
  mip_gnss_corrections_rtk_corrections_status_msg->dongle_status.tower_change_indicator = dongle_status.towerChangeIndicator();
  mip_gnss_corrections_rtk_corrections_status_msg->dongle_status.nmea_timeout_flag = dongle_status.nmeaTimeout();
  mip_gnss_corrections_rtk_corrections_status_msg->dongle_status.server_timeout_flag = dongle_status.serverTimeout();
  mip_gnss_corrections_rtk_corrections_status_msg->dongle_status.rtcm_timeout_flag = dongle_status.correctionsTimeout();
  mip_gnss_corrections_rtk_corrections_status_msg->dongle_status.device_out_of_range_flag = dongle_status.deviceOutOfRange();
  mip_gnss_corrections_rtk_corrections_status_msg->dongle_status.corrections_unavailable_flag = dongle_status.correctionsUnavailable();

  mip_gnss_corrections_rtk_corrections_status_msg->gps_correction_latency = rtk_corrections_status.gps_correction_latency;
  mip_gnss_corrections_rtk_corrections_status_msg->glonass_correction_latency = rtk_corrections_status.glonass_correction_latency;
  mip_gnss_corrections_rtk_corrections_status_msg->galileo_correction_latency = rtk_corrections_status.galileo_correction_latency;
  mip_gnss_corrections_rtk_corrections_status_msg->beidou_correction_latency = rtk_corrections_status.beidou_correction_latency;
  mip_gnss_corrections_rtk_corrections_status_pub_->publish(*mip_gnss_corrections_rtk_corrections_status_msg);
}

void Publishers::handleRtkBaseStationInfo(const mip::data_gnss::BaseStationInfo& base_station_info, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  // Only update the earth to map transform if it has changed
  const bool changed = (
    base_station_info.ecef_pos[0] != config_->map_to_earth_transform_.transform.translation.x ||
    base_station_info.ecef_pos[1] != config_->map_to_earth_transform_.transform.translation.y ||
    base_station_info.ecef_pos[2] != config_->map_to_earth_transform_.transform.translation.z
  );
  if (config_->filter_relative_pos_source_ == REL_POS_SOURCE_BASE_STATION && changed)
  {
    updateHeaderTime(&(config_->map_to_earth_transform_.header), descriptor_set, timestamp);
    config_->map_to_earth_transform_.transform.translation.x = base_station_info.ecef_pos[0];
    config_->map_to_earth_transform_.transform.translation.y = base_station_info.ecef_pos[1];
    config_->map_to_earth_transform_.transform.translation.z = base_station_info.ecef_pos[2];

    // Find the rotation between ECEF and the ENU/NED frame
    double lat, lon, alt;
    config_->geocentric_converter_.Reverse(config_->map_to_earth_transform_.transform.translation.x, config_->map_to_earth_transform_.transform.translation.y, config_->map_to_earth_transform_.transform.translation.z, lat, lon, alt);
    if (config_->use_enu_frame_)
      config_->map_to_earth_transform_.transform.rotation = tf2::toMsg(ecefToEnuTransformQuat(lat, lon));
    else
      config_->map_to_earth_transform_.transform.rotation = tf2::toMsg(ecefToNedTransformQuat(lat, lon));

    MICROSTRAIN_INFO_THROTTLE(node_, 10, "Base station info received, relative position will now be published relative to the following position");
    MICROSTRAIN_INFO_THROTTLE(node_, 10, "  LLH: [%f, %f, %f]", lat, lon, alt);
    config_->map_to_earth_transform_valid_ = true;
    config_->map_to_earth_transform_updated_ = true;
  }
}

void Publishers::handleFilterTimestamp(const mip::data_filter::Timestamp& filter_timestamp, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  // Convert the old philo timestamp into the new format and store it in the map
  mip::data_shared::GpsTimestamp stored_timestamp;
  stored_timestamp.tow = filter_timestamp.tow;
  stored_timestamp.week_number = filter_timestamp.week_number;
  stored_timestamp.valid_flags = filter_timestamp.valid_flags;
  gps_timestamp_mapping_[descriptor_set] = stored_timestamp;
}

void Publishers::handleFilterStatus(const mip::data_filter::Status& status, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  auto mip_filter_status_msg = mip_filter_status_pub_->getMessage();
  updateMipHeader(&(mip_filter_status_msg->header), descriptor_set, timestamp);
  config_->filter_state_ = status.filter_state;
  mip_filter_status_msg->filter_state = static_cast<uint16_t>(status.filter_state);
  mip_filter_status_msg->dynamics_mode = static_cast<uint16_t>(status.dynamics_mode);

  // Populate both the philo and prospect flags, it is up to the customer to determine which device they have
  mip_filter_status_msg->gx5_status_flags.init_no_attitude = status.status_flags.gx5InitNoAttitude();
  mip_filter_status_msg->gx5_status_flags.init_no_position_velocity = status.status_flags.gx5InitNoPositionVelocity();
  mip_filter_status_msg->gx5_status_flags.run_imu_unavailable = status.status_flags.gx5RunImuUnavailable();
  mip_filter_status_msg->gx5_status_flags.run_gps_unavailable = status.status_flags.gx5RunGpsUnavailable();
  mip_filter_status_msg->gx5_status_flags.run_matrix_singularity = status.status_flags.gx5RunMatrixSingularity();
  mip_filter_status_msg->gx5_status_flags.run_position_covariance_warning = status.status_flags.gx5RunPositionCovarianceWarning();
  mip_filter_status_msg->gx5_status_flags.run_velocity_covariance_warning = status.status_flags.gx5RunVelocityCovarianceWarning();
  mip_filter_status_msg->gx5_status_flags.run_attitude_covariance_warning = status.status_flags.gx5RunAttitudeCovarianceWarning();
  mip_filter_status_msg->gx5_status_flags.run_nan_in_solution_warning = status.status_flags.gx5RunNanInSolutionWarning();
  mip_filter_status_msg->gx5_status_flags.run_gyro_bias_est_high_warning = status.status_flags.gx5RunGyroBiasEstHighWarning();
  mip_filter_status_msg->gx5_status_flags.run_accel_bias_est_high_warning = status.status_flags.gx5RunAccelBiasEstHighWarning();
  mip_filter_status_msg->gx5_status_flags.run_gyro_scale_factor_est_high_warning = status.status_flags.gx5RunGyroScaleFactorEstHighWarning();
  mip_filter_status_msg->gx5_status_flags.run_accel_scale_factor_est_high_warning = status.status_flags.gx5RunAccelScaleFactorEstHighWarning();
  mip_filter_status_msg->gx5_status_flags.run_mag_bias_est_high_warning = status.status_flags.gx5RunMagBiasEstHighWarning();
  mip_filter_status_msg->gx5_status_flags.run_ant_offset_correction_est_high_warning = status.status_flags.gx5RunAntOffsetCorrectionEstHighWarning();
  mip_filter_status_msg->gx5_status_flags.run_mag_hard_iron_est_high_warning = status.status_flags.gx5RunMagHardIronEstHighWarning();
  mip_filter_status_msg->gx5_status_flags.run_mag_soft_iron_est_high_warning = status.status_flags.gx5RunMagSoftIronEstHighWarning();

  mip_filter_status_msg->gq7_status_flags.filter_condition = status.status_flags.gq7FilterCondition();
  mip_filter_status_msg->gq7_status_flags.roll_pitch_warning = status.status_flags.gq7RollPitchWarning();
  mip_filter_status_msg->gq7_status_flags.heading_warning = status.status_flags.gq7HeadingWarning();
  mip_filter_status_msg->gq7_status_flags.position_warning = status.status_flags.gq7PositionWarning();
  mip_filter_status_msg->gq7_status_flags.velocity_warning = status.status_flags.gq7VelocityWarning();
  mip_filter_status_msg->gq7_status_flags.imu_bias_warning = status.status_flags.gq7ImuBiasWarning();
  mip_filter_status_msg->gq7_status_flags.gnss_clk_warning = status.status_flags.gq7GnssClkWarning();
  mip_filter_status_msg->gq7_status_flags.antenna_lever_arm_warning = status.status_flags.gq7AntennaLeverArmWarning();
  mip_filter_status_msg->gq7_status_flags.mounting_transform_warning = status.status_flags.gq7MountingTransformWarning();
  mip_filter_status_msg->gq7_status_flags.time_sync_warning = status.status_flags.gq7TimeSyncWarning();
  mip_filter_status_msg->gq7_status_flags.solution_error = status.status_flags.gq7SolutionError();
  mip_filter_status_pub_->publish(*mip_filter_status_msg);

  // Populate the human readable status message
  auto filter_human_readable_status_msg = filter_human_readable_status_pub_->getMessageToUpdate();
  updateHeaderTime(&filter_human_readable_status_msg->header, descriptor_set, timestamp);
  filter_human_readable_status_msg->status_flags.clear();
  if (RosMipDevice::isPhilo(config_->mip_device_->device_info_))  // Philo products
  {
    switch (status.filter_state)
    {
      case mip::data_filter::FilterMode::GX5_STARTUP:
        filter_human_readable_status_msg->filter_state = HumanReadableStatusMsg::FILTER_STATE_GX5_STARTUP;
        break;
      case mip::data_filter::FilterMode::GX5_INIT:
        filter_human_readable_status_msg->filter_state = HumanReadableStatusMsg::FILTER_STATE_GX5_INIT;
        break;
      case mip::data_filter::FilterMode::GX5_RUN_SOLUTION_VALID:
        filter_human_readable_status_msg->filter_state = HumanReadableStatusMsg::FILTER_STATE_GX5_RUN_SOLUTION_VALID;
        break;
      case mip::data_filter::FilterMode::GX5_RUN_SOLUTION_ERROR:
        filter_human_readable_status_msg->filter_state = HumanReadableStatusMsg::FILTER_STATE_GX5_RUN_SOLUTION_ERROR;
        break;
    }
    if (status.status_flags.gx5InitNoAttitude())
      filter_human_readable_status_msg->status_flags.push_back(HumanReadableStatusMsg::STATUS_FLAGS_GX5_INIT_NO_ATTITUDE);
    if (status.status_flags.gx5InitNoPositionVelocity())
      filter_human_readable_status_msg->status_flags.push_back(HumanReadableStatusMsg::STATUS_FLAGS_GX5_INIT_NO_POSITION_VELOCITY);
    if (status.status_flags.gx5RunImuUnavailable())
      filter_human_readable_status_msg->status_flags.push_back(HumanReadableStatusMsg::STATUS_FLAGS_GX5_RUN_IMU_UNAVAILABLE);
    if (status.status_flags.gx5RunGpsUnavailable())
      filter_human_readable_status_msg->status_flags.push_back(HumanReadableStatusMsg::STATUS_FLAGS_GX5_RUN_GPS_UNAVAILABLE);
    if (status.status_flags.gx5RunMatrixSingularity())
      filter_human_readable_status_msg->status_flags.push_back(HumanReadableStatusMsg::STATUS_FLAGS_GX5_RUN_MATRIX_SINGULARITY);
    if (status.status_flags.gx5RunPositionCovarianceWarning())
      filter_human_readable_status_msg->status_flags.push_back(HumanReadableStatusMsg::STATUS_FLAGS_GX5_RUN_POSITION_COVARIANCE_WARNING);
    if (status.status_flags.gx5RunVelocityCovarianceWarning())
      filter_human_readable_status_msg->status_flags.push_back(HumanReadableStatusMsg::STATUS_FLAGS_GX5_RUN_VELOCITY_COVARIANCE_WARNING);
    if (status.status_flags.gx5RunAttitudeCovarianceWarning())
      filter_human_readable_status_msg->status_flags.push_back(HumanReadableStatusMsg::STATUS_FLAGS_GX5_RUN_ATTITUDE_COVARIANCE_WARNING);
    if (status.status_flags.gx5RunNanInSolutionWarning())
      filter_human_readable_status_msg->status_flags.push_back(HumanReadableStatusMsg::STATUS_FLAGS_GX5_RUN_NAN_IN_SOLUTION_WARNING);
    if (status.status_flags.gx5RunGyroBiasEstHighWarning())
      filter_human_readable_status_msg->status_flags.push_back(HumanReadableStatusMsg::STATUS_FLAGS_GX5_RUN_GYRO_BIAS_EST_HIGH_WARNING);
    if (status.status_flags.gx5RunAccelBiasEstHighWarning())
      filter_human_readable_status_msg->status_flags.push_back(HumanReadableStatusMsg::STATUS_FLAGS_GX5_RUN_ACCEL_BIAS_EST_HIGH_WARNING);
    if (status.status_flags.gx5RunGyroScaleFactorEstHighWarning())
      filter_human_readable_status_msg->status_flags.push_back(HumanReadableStatusMsg::STATUS_FLAGS_GX5_RUN_GYRO_SCALE_FACTOR_EST_HIGH_WARNING);
    if (status.status_flags.gx5RunAccelScaleFactorEstHighWarning())
      filter_human_readable_status_msg->status_flags.push_back(HumanReadableStatusMsg::STATUS_FLAGS_GX5_RUN_ACCEL_SCALE_FACTOR_EST_HIGH_WARNING);
    if (status.status_flags.gx5RunMagBiasEstHighWarning())
      filter_human_readable_status_msg->status_flags.push_back(HumanReadableStatusMsg::STATUS_FLAGS_GX5_RUN_MAG_BIAS_EST_HIGH_WARNING);
    if (status.status_flags.gx5RunAntOffsetCorrectionEstHighWarning())
      filter_human_readable_status_msg->status_flags.push_back(HumanReadableStatusMsg::STATUS_FLAGS_GX5_RUN_ANT_OFFSET_CORRECTION_EST_HIGH_WARNING);
    if (status.status_flags.gx5RunMagHardIronEstHighWarning())
      filter_human_readable_status_msg->status_flags.push_back(HumanReadableStatusMsg::STATUS_FLAGS_GX5_RUN_MAG_HARD_IRON_EST_HIGH_WARNING);
    if (status.status_flags.gx5RunMagSoftIronEstHighWarning())
      filter_human_readable_status_msg->status_flags.push_back(HumanReadableStatusMsg::STATUS_FLAGS_GX5_RUN_MAG_SOFT_IRON_EST_HIGH_WARNING);
  }
  else if (RosMipDevice::isProspect(config_->mip_device_->device_info_))  // Prospect products
  {
    switch (status.filter_state)
    {
      case mip::data_filter::FilterMode::INIT:
        filter_human_readable_status_msg->filter_state = HumanReadableStatusMsg::FILTER_STATE_GQ7_INIT;
        break;
      case mip::data_filter::FilterMode::VERT_GYRO:
        filter_human_readable_status_msg->filter_state = HumanReadableStatusMsg::FILTER_STATE_GQ7_VERT_GYRO;
        break;
      case mip::data_filter::FilterMode::AHRS:
        filter_human_readable_status_msg->filter_state = HumanReadableStatusMsg::FILTER_STATE_GQ7_AHRS;
        break;
      case mip::data_filter::FilterMode::FULL_NAV:
        filter_human_readable_status_msg->filter_state = HumanReadableStatusMsg::FILTER_STATE_GQ7_FULL_NAV;
        break;
    }
    switch (status.status_flags.gq7FilterCondition())
    {
      case 1:
        filter_human_readable_status_msg->status_flags.push_back(HumanReadableStatusMsg::STATUS_FLAGS_GQ7_FILTER_CONDITION_STABLE);
        break;
      case 2:
        filter_human_readable_status_msg->status_flags.push_back(HumanReadableStatusMsg::STATUS_FLAGS_GQ7_FILTER_CONDITION_CONVERGING);
        break;
      case 3:
        filter_human_readable_status_msg->status_flags.push_back(HumanReadableStatusMsg::STATUS_FLAGS_GQ7_FILTER_CONDITION_UNSTABLE);
        break;
    }
    if (status.status_flags.gq7RollPitchWarning())
      filter_human_readable_status_msg->status_flags.push_back(HumanReadableStatusMsg::STATUS_FLAGS_GQ7_ROLL_PITCH_WARNING);
    if (status.status_flags.gq7HeadingWarning())
      filter_human_readable_status_msg->status_flags.push_back(HumanReadableStatusMsg::STATUS_FLAGS_GQ7_HEADING_WARNING);
    if (status.status_flags.gq7PositionWarning())
      filter_human_readable_status_msg->status_flags.push_back(HumanReadableStatusMsg::STATUS_FLAGS_GQ7_POSITION_WARNING);
    if (status.status_flags.gq7VelocityWarning())
      filter_human_readable_status_msg->status_flags.push_back(HumanReadableStatusMsg::STATUS_FLAGS_GQ7_VELOCITY_WARNING);
    if (status.status_flags.gq7ImuBiasWarning())
      filter_human_readable_status_msg->status_flags.push_back(HumanReadableStatusMsg::STATUS_FLAGS_GQ7_IMU_BIAS_WARNING);
    if (status.status_flags.gq7GnssClkWarning())
      filter_human_readable_status_msg->status_flags.push_back(HumanReadableStatusMsg::STATUS_FLAGS_GQ7_GNSS_CLK_WARNING);
    if (status.status_flags.gq7AntennaLeverArmWarning())
      filter_human_readable_status_msg->status_flags.push_back(HumanReadableStatusMsg::STATUS_FLAGS_GQ7_ANTENNA_LEVER_ARM_WARNING);
    if (status.status_flags.gq7MountingTransformWarning())
      filter_human_readable_status_msg->status_flags.push_back(HumanReadableStatusMsg::STATUS_FLAGS_GQ7_MOUNTING_TRANSFORM_WARNING);
    if (status.status_flags.gq7TimeSyncWarning())
      filter_human_readable_status_msg->status_flags.push_back(HumanReadableStatusMsg::STATUS_FLAGS_GQ7_TIME_SYNC_WARNING);
    if (status.status_flags.gq7SolutionError())
      filter_human_readable_status_msg->status_flags.push_back(HumanReadableStatusMsg::STATUS_FLAGS_GQ7_SOLUTION_ERROR);
  }
}

void Publishers::handleFilterEcefPos(const mip::data_filter::EcefPos& ecef_pos, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  auto filter_odometry_earth_msg = filter_odometry_earth_pub_->getMessageToUpdate();
  updateHeaderTime(&(filter_odometry_earth_msg->header), descriptor_set, timestamp);
  filter_odometry_earth_msg->pose.pose.position.x = ecef_pos.position_ecef[0];
  filter_odometry_earth_msg->pose.pose.position.y = ecef_pos.position_ecef[1];
  filter_odometry_earth_msg->pose.pose.position.z = ecef_pos.position_ecef[2];

  // Update the global transform if the data is valid
  if (ecef_pos.valid_flags == 1)
  {
    imu_link_to_earth_transform_translation_updated_ = true;
    imu_link_to_earth_transform_tf_stamped_.stamp_ = tf2_ros::fromMsg(filter_odometry_earth_msg->header.stamp);
    imu_link_to_earth_transform_tf_stamped_.setOrigin(tf2::Vector3(ecef_pos.position_ecef[0], ecef_pos.position_ecef[1], ecef_pos.position_ecef[2]));
  }
  // If the earth to map transform is not valid and we entered full navigation mode, populate the transform with this position
  const bool full_nav =
  (
    (RosMipDevice::isPhilo(config_->mip_device_->device_info_) && config_->filter_state_ == mip::data_filter::FilterMode::GX5_RUN_SOLUTION_VALID) ||
    (RosMipDevice::isProspect(config_->mip_device_->device_info_) && config_->filter_state_ == mip::data_filter::FilterMode::FULL_NAV)
  );
  if (!config_->map_to_earth_transform_valid_ && config_->filter_relative_pos_source_ == REL_POS_SOURCE_AUTO && full_nav)
  {
    // Find the rotation between ECEF and NED/ENU for this position
    double lat, lon, alt;
    config_->geocentric_converter_.Reverse(ecef_pos.position_ecef[0], ecef_pos.position_ecef[1], ecef_pos.position_ecef[2], lat, lon, alt);
    const tf2::Transform map_to_earth_transform_tf(
      config_->use_enu_frame_ ? ecefToEnuTransform(lat, lon).inverse() : ecefToNedTransform(lat, lon).inverse(),
      tf2::Vector3(ecef_pos.position_ecef[0], ecef_pos.position_ecef[1], ecef_pos.position_ecef[2])
    );
    config_->map_to_earth_transform_.header.stamp = rosTimeNow(node_);
    config_->map_to_earth_transform_.transform = tf2::toMsg(map_to_earth_transform_tf);

    MICROSTRAIN_INFO(node_, "Full nav achieved. Relative position will be reported relative to the following position");
    MICROSTRAIN_INFO(node_, "  LLH: [%f, %f, %f]", lat, lon, alt);
    MICROSTRAIN_INFO(node_, "  XYZW: [%f, %f, %f, %f]", config_->map_to_earth_transform_.transform.rotation.x, config_->map_to_earth_transform_.transform.rotation.y, config_->map_to_earth_transform_.transform.rotation.z, config_->map_to_earth_transform_.transform.rotation.w);
    config_->map_to_earth_transform_valid_ = true;
    config_->map_to_earth_transform_updated_ = true;
  }

  // If the map odometry message is enabled and we have relative position configuration attempt to transform the global position to the map frame
  if (filter_odometry_map_pub_->dataRate() > 0 && config_->filter_relative_pos_config_)
  {
    // Check if we can get the map to earth transform from the TF tree, if not see if we have a valid transform in memory
    std::string tf_error_string;
    RosTimeType frame_time = filter_odometry_earth_msg->header.stamp;
    TransformStampedMsg map_to_earth_transform;
    map_to_earth_transform.header.frame_id = config_->earth_frame_id_ + "_UNPOPULATED";
    map_to_earth_transform.child_frame_id = config_->map_frame_id_ + "_UNPOPULATED";
    if (config_->filter_relative_pos_source_ == REL_POS_SOURCE_EXTERNAL && transform_buffer_->canTransform(config_->earth_frame_id_, config_->map_frame_id_, frame_time, RosDurationType(0, 0), &tf_error_string))
      map_to_earth_transform = transform_buffer_->lookupTransform(config_->earth_frame_id_, config_->map_frame_id_, frame_time);
    else if (config_->filter_relative_pos_source_ != REL_POS_SOURCE_EXTERNAL && config_->map_to_earth_transform_valid_)
      map_to_earth_transform = config_->map_to_earth_transform_;

    // Check if we have a valid transform
    if (map_to_earth_transform.header.frame_id == config_->earth_frame_id_ && map_to_earth_transform.child_frame_id == config_->map_frame_id_)
    {
      tf2::Transform imu_to_earth_transform_tf(tf2::Quaternion::getIdentity(), tf2::Vector3(ecef_pos.position_ecef[0], ecef_pos.position_ecef[1], ecef_pos.position_ecef[2]));
      tf2::Transform map_to_earth_transform_tf;
      tf2::fromMsg(map_to_earth_transform.transform, map_to_earth_transform_tf);
      const tf2::Transform imu_to_map_transform_tf = map_to_earth_transform_tf.inverse() * imu_to_earth_transform_tf;

      // Fill in the map odometry message
      // Note that since the earth to map transform already puts us in either NED or ENU automatically there is no need to swap the values here
      auto filter_odometry_map_msg = filter_odometry_map_pub_->getMessageToUpdate();
      updateHeaderTime(&(filter_odometry_map_msg->header), descriptor_set, timestamp);
      filter_odometry_map_msg->pose.pose.position.x = imu_to_map_transform_tf.getOrigin().getX();
      filter_odometry_map_msg->pose.pose.position.y = imu_to_map_transform_tf.getOrigin().getY();
      filter_odometry_map_msg->pose.pose.position.z = imu_to_map_transform_tf.getOrigin().getZ();

      // Fill in the map to imu link transform if the data is valid
      if (ecef_pos.valid_flags == 1)
      {
        imu_link_to_map_transform_tf_stamped_.stamp_ = tf2_ros::fromMsg(rosTimeNow(node_));
        imu_link_to_map_transform_tf_stamped_.setOrigin(imu_to_map_transform_tf.getOrigin());
        imu_link_to_map_transform_translation_updated_ = true;
      }
    }
    else if (config_->filter_relative_pos_source_ == REL_POS_SOURCE_BASE_STATION)
    {
      MICROSTRAIN_WARN_ONCE(node_, "Base station info not received yet, waiting to publish relative position...");
    }
    else if (config_->filter_relative_pos_source_ == REL_POS_SOURCE_AUTO)
    {
      MICROSTRAIN_WARN_ONCE(node_, "Sensor not in full nav mode yet, waiting to publish relative position...");
    }
    else if (config_->filter_relative_pos_source_ == REL_POS_SOURCE_EXTERNAL)
    {
      MICROSTRAIN_WARN_THROTTLE(node_, 10, "No valid transform from %s to %s: %s", config_->earth_frame_id_.c_str(), config_->map_frame_id_.c_str(), tf_error_string.c_str());
    }
  }
}

void Publishers::handleFilterEcefPosUncertainty(const mip::data_filter::EcefPosUncertainty& ecef_pos_uncertainty, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  auto filter_odometry_earth_msg = filter_odometry_earth_pub_->getMessageToUpdate();
  updateHeaderTime(&(filter_odometry_earth_msg->header), descriptor_set, timestamp);
  filter_odometry_earth_msg->pose.covariance[0] = pow(ecef_pos_uncertainty.pos_uncertainty[0], 2);
  filter_odometry_earth_msg->pose.covariance[7] = pow(ecef_pos_uncertainty.pos_uncertainty[1], 2);
  filter_odometry_earth_msg->pose.covariance[14] = pow(ecef_pos_uncertainty.pos_uncertainty[2], 2);
}

void Publishers::handleFilterPositionLlh(const mip::data_filter::PositionLlh& position_llh, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  auto filter_llh_position_msg = filter_llh_position_pub_->getMessageToUpdate();
  updateHeaderTime(&(filter_llh_position_msg->header), descriptor_set, timestamp);
  filter_llh_position_msg->latitude = position_llh.latitude;
  filter_llh_position_msg->longitude = position_llh.longitude;
  filter_llh_position_msg->altitude = position_llh.ellipsoid_height;

  // If the device does not support ECEF, fill it out here, and call the callback ourselves
  if (!supports_filter_ecef_)
  {
    mip::data_filter::EcefPos ecef_pos;
    ecef_pos.valid_flags = position_llh.valid_flags;
    config_->geocentric_converter_.Forward(position_llh.latitude, position_llh.longitude, position_llh.ellipsoid_height,
      ecef_pos.position_ecef[0], ecef_pos.position_ecef[1], ecef_pos.position_ecef[2]);
    handleFilterEcefPos(ecef_pos, descriptor_set, timestamp);
  }
}

void Publishers::handleFilterPositionLlhUncertainty(const mip::data_filter::PositionLlhUncertainty& position_llh_uncertainty, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  // If we have to rotate the covariance, it is easier to do when uncertaintty is in a covariance matrix.
  // NOTE: The rotation between the microstrain vehicle and ROS vehicle is essentially the same for covariance.
  //       Since all it does is negate the y and z axis, but that gets squared anyways.
  const double n = pow(position_llh_uncertainty.north, 2);
  const double e = pow(position_llh_uncertainty.east, 2);
  const double d = pow(position_llh_uncertainty.down, 2);
  const PoseWithCovarianceStampedMsg::_pose_type::_covariance_type ned_frame_covariance =
  {
    n, 0, 0, 0, 0, 0,
    0, e, 0, 0, 0, 0,
    0, 0, d, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0
  };
  const PoseWithCovarianceStampedMsg::_pose_type::_covariance_type enu_frame_covariance = tf2::transformCovariance(ned_frame_covariance, config_->ned_to_enu_transform_tf_);

  // Filter fix message
  auto filter_llh_position_msg = filter_llh_position_pub_->getMessageToUpdate();
  updateHeaderTime(&(filter_llh_position_msg->header), descriptor_set, timestamp);
  filter_llh_position_msg->position_covariance_type = NavSatFixMsg::COVARIANCE_TYPE_DIAGONAL_KNOWN;
  if (config_->use_enu_frame_)
  {
    filter_llh_position_msg->position_covariance[0] = enu_frame_covariance[0];
    filter_llh_position_msg->position_covariance[4] = enu_frame_covariance[7];
    filter_llh_position_msg->position_covariance[8] = enu_frame_covariance[14];
  }
  else
  {
    filter_llh_position_msg->position_covariance[0] = ned_frame_covariance[0];
    filter_llh_position_msg->position_covariance[4] = ned_frame_covariance[7];
    filter_llh_position_msg->position_covariance[8] = ned_frame_covariance[14];
  }

  // Filter relative odometry message (not counted as updating)
  auto filter_odometry_map_msg = filter_odometry_map_pub_->getMessage();
  if (config_->use_enu_frame_)
    setTranslationCovarianceOnCovariance(&filter_odometry_map_msg->pose.covariance, getTranslationCovarianceFromCovariance(enu_frame_covariance));
  else
    setTranslationCovarianceOnCovariance(&filter_odometry_map_msg->pose.covariance, getTranslationCovarianceFromCovariance(ned_frame_covariance));

  // If the device does not support ECEF uncertainty, rotate this uncertainty into the ECEF frame and process it
  if (!supports_filter_ecef_)
  {
    const tf2::Transform ecef_to_ned_transform(ecefToNedTransform(filter_llh_position_msg->latitude, filter_llh_position_msg->longitude));
    const PoseWithCovarianceStampedMsg::_pose_type::_covariance_type ecef_frame_covariance = tf2::transformCovariance(ned_frame_covariance, ecef_to_ned_transform.inverse());
    mip::data_filter::EcefPosUncertainty ecef_pos_uncertainty;
    ecef_pos_uncertainty.pos_uncertainty =
    {
      static_cast<float>(sqrt(ecef_frame_covariance[0])),
      static_cast<float>(sqrt(ecef_frame_covariance[7])),
      static_cast<float>(sqrt(ecef_frame_covariance[14])),
    };
    handleFilterEcefPosUncertainty(ecef_pos_uncertainty, descriptor_set, timestamp);
  }
}

void Publishers::handleFilterAttitudeQuaternion(const mip::data_filter::AttitudeQuaternion& attitude_quaternion, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  // Filter odometry message rotated to ECEF (not counted as updating)
  auto filter_odometry_earth_msg = filter_odometry_earth_pub_->getMessage();

  // Convert the ECEF coordinates to LLH so we can lookup the rotation
  double lat, lon, alt;
  config_->geocentric_converter_.Reverse(filter_odometry_earth_msg->pose.pose.position.x, filter_odometry_earth_msg->pose.pose.position.y, filter_odometry_earth_msg->pose.pose.position.z, lat, lon, alt);

  // Put the orientation into the ECEF frame for our earth messages
  const tf2::Transform microstrain_vehicle_to_ned_transform_tf(tf2::Quaternion(attitude_quaternion.q[1], attitude_quaternion.q[2], attitude_quaternion.q[3], attitude_quaternion.q[0]));
  if (config_->use_enu_frame_)
  {
    const tf2::Transform earth_to_enu_transform_tf(ecefToEnuTransform(lat, lon));
    const tf2::Transform ros_vehicle_to_earth_transform_tf = earth_to_enu_transform_tf.inverse() * config_->ned_to_enu_transform_tf_ * microstrain_vehicle_to_ned_transform_tf * config_->ros_vehicle_to_microstrain_vehicle_transform_tf_;
    imu_link_to_earth_transform_tf_stamped_.setBasis(ros_vehicle_to_earth_transform_tf.getBasis());
    filter_odometry_earth_msg->pose.pose.orientation = tf2::toMsg(ros_vehicle_to_earth_transform_tf.getRotation());
  }
  else
  {
    const tf2::Transform earth_to_ned_transform_tf(ecefToNedTransform(lat, lon));
    const tf2::Transform microstrain_vehicle_to_earth_transform_tf = earth_to_ned_transform_tf.inverse() * microstrain_vehicle_to_ned_transform_tf;
    imu_link_to_earth_transform_tf_stamped_.setBasis(microstrain_vehicle_to_earth_transform_tf.getBasis());
    filter_odometry_earth_msg->pose.pose.orientation = tf2::toMsg(microstrain_vehicle_to_earth_transform_tf.getRotation());
  }
  imu_link_to_earth_transform_tf_stamped_.stamp_ = tf2_ros::fromMsg(rosTimeNow(node_));
  imu_link_to_earth_transform_attitude_updated_ = true;

  // Filtered IMU message
  auto filter_imu_msg = filter_imu_pub_->getMessageToUpdate();
  updateHeaderTime(&(filter_imu_msg->header), descriptor_set, timestamp);

  // Filter odometry map message (not counted as updating)
  auto filter_odometry_map_msg = filter_odometry_map_pub_->getMessage();

  // Put the orientation into the NED/ENU frame for our map messages
  if (config_->use_enu_frame_)
  {
    const tf2::Transform ros_vehicle_to_enu_transform_tf = config_->ned_to_enu_transform_tf_ * microstrain_vehicle_to_ned_transform_tf * config_->ros_vehicle_to_microstrain_vehicle_transform_tf_;
    imu_link_to_map_transform_tf_stamped_.setBasis(ros_vehicle_to_enu_transform_tf.getBasis());
    filter_odometry_map_msg->pose.pose.orientation = tf2::toMsg(ros_vehicle_to_enu_transform_tf.getRotation());
    filter_imu_msg->orientation = tf2::toMsg(ros_vehicle_to_enu_transform_tf.getRotation());
  }
  else
  {
    imu_link_to_map_transform_tf_stamped_.setBasis(microstrain_vehicle_to_ned_transform_tf.getBasis());
    filter_odometry_map_msg->pose.pose.orientation = tf2::toMsg(microstrain_vehicle_to_ned_transform_tf.getRotation());
    filter_imu_msg->orientation = tf2::toMsg(microstrain_vehicle_to_ned_transform_tf.getRotation());
  }
  imu_link_to_map_transform_tf_stamped_.stamp_ = tf2_ros::fromMsg(rosTimeNow(node_));
  imu_link_to_map_transform_attitude_updated_ = true;
}

void Publishers::handleFilterEulerAnglesUncertainty(const mip::data_filter::EulerAnglesUncertainty& euler_angles_uncertainty, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  // If we have to rotate the covariance, it is easier to do when uncertaintty is in a covariance matrix.
  // NOTE: The rotation between the microstrain vehicle and ROS vehicle is essentially the same for covariance.
  //       Since all it does is negate the y and z axis, but that gets squared anyways.
  const double r = pow(euler_angles_uncertainty.roll, 2);
  const double p = pow(euler_angles_uncertainty.pitch, 2);
  const double y = pow(euler_angles_uncertainty.yaw, 2);
  const PoseWithCovarianceStampedMsg::_pose_type::_covariance_type ned_frame_covariance =
  {
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, r, 0, 0,
    0, 0, 0, 0, p, 0,
    0, 0, 0, 0, 0, y
  };
  const PoseWithCovarianceStampedMsg::_pose_type::_covariance_type enu_frame_covariance = tf2::transformCovariance(ned_frame_covariance, config_->ned_to_enu_transform_tf_);

  // Filtered IMU message
  auto filter_imu_msg = filter_imu_pub_->getMessageToUpdate();
  updateHeaderTime(&(filter_imu_msg->header), descriptor_set, timestamp);
  if (config_->use_enu_frame_)
    filter_imu_msg->orientation_covariance = getRotationCovarianceFromCovariance(enu_frame_covariance);
  else
    filter_imu_msg->orientation_covariance = getRotationCovarianceFromCovariance(ned_frame_covariance);

  // Filter odometry message (not counted as updating)
  auto filter_odometry_earth_msg = filter_odometry_earth_pub_->getMessage();

  // Rotate the microstrain covariance matrix into ECEF.
  // NOTE: We view the NED frame and microstrain vehicle frame as the same here since there is no transform between them.
  double lat, lon, height;
  config_->geocentric_converter_.Reverse(filter_odometry_earth_msg->pose.pose.position.x, filter_odometry_earth_msg->pose.pose.position.y, filter_odometry_earth_msg->pose.pose.position.z, lat, lon, height);
  const tf2::Transform ecef_to_ned_transform(ecefToNedTransform(lat, lon));
  const PoseWithCovarianceStampedMsg::_pose_type::_covariance_type ecef_frame_covariance = tf2::transformCovariance(ned_frame_covariance, ecef_to_ned_transform.inverse());
  setRotationCovarianceOnCovariance(&filter_odometry_earth_msg->pose.covariance, getRotationCovarianceFromCovariance(ecef_frame_covariance));

  // Filter relative odometry message (not counted as updating)
  auto filter_odometry_map_msg = filter_odometry_map_pub_->getMessage();
  if (config_->use_enu_frame_)
    setRotationCovarianceOnCovariance(&filter_odometry_map_msg->pose.covariance, getRotationCovarianceFromCovariance(enu_frame_covariance));
  else
    setRotationCovarianceOnCovariance(&filter_odometry_map_msg->pose.covariance, getRotationCovarianceFromCovariance(ned_frame_covariance));
}

void Publishers::handleFilterVelocityNed(const mip::data_filter::VelocityNed& velocity_ned, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  // Filter ENU velocity message
  auto filter_velocity_msg = filter_velocity_pub_->getMessageToUpdate();
  updateHeaderTime(&(filter_velocity_msg->header), descriptor_set, timestamp);
  if (config_->use_enu_frame_)
  {
    filter_velocity_msg->twist.twist.linear.x = velocity_ned.east;
    filter_velocity_msg->twist.twist.linear.y = velocity_ned.north;
    filter_velocity_msg->twist.twist.linear.z = -velocity_ned.down;
  }
  else
  {
    filter_velocity_msg->twist.twist.linear.x = velocity_ned.north;
    filter_velocity_msg->twist.twist.linear.y = velocity_ned.east;
    filter_velocity_msg->twist.twist.linear.z = velocity_ned.down;
  }

  // Filter relative odometry message (not counted as updating)
  auto filter_odometry_map_msg = filter_odometry_map_pub_->getMessage();

  // Rotate the velocity to the sensor frame
  const tf2::Transform imu_to_map_transform_tf(
    tf2::Quaternion(
      filter_odometry_map_msg->pose.pose.orientation.x,
      filter_odometry_map_msg->pose.pose.orientation.y,
      filter_odometry_map_msg->pose.pose.orientation.z,
      filter_odometry_map_msg->pose.pose.orientation.w)
  );
  const tf2::Vector3 imu_velocity_in_map_frame(
    filter_velocity_msg->twist.twist.linear.x,
    filter_velocity_msg->twist.twist.linear.y,
    filter_velocity_msg->twist.twist.linear.z
  );
  const tf2::Vector3 imu_velocity_in_imu_frame = imu_to_map_transform_tf.inverse() * imu_velocity_in_map_frame;
  filter_odometry_map_msg->twist.twist.linear.x = imu_velocity_in_imu_frame.getX();
  filter_odometry_map_msg->twist.twist.linear.y = imu_velocity_in_imu_frame.getY();
  filter_odometry_map_msg->twist.twist.linear.z = imu_velocity_in_imu_frame.getZ();

  // Filter odometry message (not counted as updating)
  auto filter_odometry_earth_msg = filter_odometry_earth_pub_->getMessage();
  filter_odometry_earth_msg->twist.twist.linear = filter_odometry_map_msg->twist.twist.linear;
}

void Publishers::handleFilterVelocityNedUncertainty(const mip::data_filter::VelocityNedUncertainty& velocity_ned_uncertainty, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  // Filter ENU velocity message
  auto filter_velocity_msg = filter_velocity_pub_->getMessageToUpdate();
  updateHeaderTime(&(filter_velocity_msg->header), descriptor_set, timestamp);
  if (config_->use_enu_frame_)
  {
    filter_velocity_msg->twist.covariance[0] = pow(velocity_ned_uncertainty.east, 2);
    filter_velocity_msg->twist.covariance[7] = pow(velocity_ned_uncertainty.north, 2);
  }
  else
  {
    filter_velocity_msg->twist.covariance[0] = pow(velocity_ned_uncertainty.north, 2);
    filter_velocity_msg->twist.covariance[7] = pow(velocity_ned_uncertainty.east, 2);
  }
  filter_velocity_msg->twist.covariance[14] = pow(velocity_ned_uncertainty.down, 2);

  // Filter relative odometry message (not counted as updating)
  // NOTE: The rotation between the microstrain vehicle and ROS vehicle is essentially the same for covariance.
  //       Since all it does is negate the y and z axis, but that gets squared anyways.
  // NOTE: We view the NED frame and microstrain vehicle frame as the same here since there is no transform between them.
  auto filter_odometry_map_msg = filter_odometry_map_pub_->getMessage();
  filter_odometry_map_msg->twist.covariance[21] = pow(velocity_ned_uncertainty.north, 2);
  filter_odometry_map_msg->twist.covariance[28] = pow(velocity_ned_uncertainty.east, 2);
  filter_odometry_map_msg->twist.covariance[35] = pow(velocity_ned_uncertainty.down, 2);

  // Filter odometry message (not counted as updating)
  auto filter_odometry_earth_msg = filter_odometry_earth_pub_->getMessageToUpdate();
  filter_odometry_earth_msg->twist.covariance = filter_odometry_map_msg->twist.covariance;
}

void Publishers::handleFilterEcefVelocity(const mip::data_filter::EcefVel& ecef_vel, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  // Filter ECEF velocity message
  auto filter_velocity_ecef_msg = filter_velocity_ecef_pub_->getMessageToUpdate();
  updateHeaderTime(&(filter_velocity_ecef_msg->header), descriptor_set, timestamp);
  filter_velocity_ecef_msg->twist.twist.linear.x = ecef_vel.velocity_ecef[0];
  filter_velocity_ecef_msg->twist.twist.linear.y = ecef_vel.velocity_ecef[1];
  filter_velocity_ecef_msg->twist.twist.linear.z = ecef_vel.velocity_ecef[2];
}

void Publishers::handleFilterEcefVelocityUncertainty(const mip::data_filter::EcefVelUncertainty& ecef_velocity_uncertainty, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  // Filter ECEF velocity message
  auto filter_velocity_ecef_msg = filter_velocity_ecef_pub_->getMessageToUpdate();
  updateHeaderTime(&(filter_velocity_ecef_msg->header), descriptor_set, timestamp);
  filter_velocity_ecef_msg->twist.covariance[0] = pow(ecef_velocity_uncertainty.vel_uncertainty[0], 2);
  filter_velocity_ecef_msg->twist.covariance[7] = pow(ecef_velocity_uncertainty.vel_uncertainty[1], 2);
  filter_velocity_ecef_msg->twist.covariance[14] = pow(ecef_velocity_uncertainty.vel_uncertainty[2], 2);
}

void Publishers::handleFilterCompAngularRate(const mip::data_filter::CompAngularRate& comp_angular_rate, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  // Filtered IMU message
  auto filter_imu_msg = filter_imu_pub_->getMessageToUpdate();
  updateHeaderTime(&(filter_imu_msg->header), descriptor_set, timestamp);
  filter_imu_msg->angular_velocity.x = comp_angular_rate.gyro[0];
  filter_imu_msg->angular_velocity.y = comp_angular_rate.gyro[1];
  filter_imu_msg->angular_velocity.z = comp_angular_rate.gyro[2];
  if (config_->use_enu_frame_)
  {
    filter_imu_msg->angular_velocity.y *= -1;
    filter_imu_msg->angular_velocity.z *= -1;
  }

  // Filter velocity message (not counted as updating)
  auto filter_velocity_msg = filter_velocity_pub_->getMessage();
  filter_velocity_msg->twist.twist.angular = filter_imu_msg->angular_velocity;

  // Filter relative odometry message (not counted as updating)
  auto filter_odometry_map_msg = filter_odometry_map_pub_->getMessage();
  filter_odometry_map_msg->twist.twist.angular = filter_imu_msg->angular_velocity;

  // Filter odometry message (not counted as updating)
  auto filter_odometry_earth_msg = filter_odometry_earth_pub_->getMessage();
  filter_odometry_earth_msg->twist.twist.angular = filter_imu_msg->angular_velocity;

  // The angular velocity comes out in the sensor frame. Rotate it into the earth frame in order to populate the earth messages
  const tf2::Transform imu_to_earth_transform_tf(
    tf2::Quaternion(
      filter_odometry_earth_msg->pose.pose.orientation.x,
      filter_odometry_earth_msg->pose.pose.orientation.y,
      filter_odometry_earth_msg->pose.pose.orientation.z,
      filter_odometry_earth_msg->pose.pose.orientation.w)
  );
  const tf2::Vector3 imu_angular_velocity_in_imu_frame(
    filter_imu_msg->angular_velocity.x,
    filter_imu_msg->angular_velocity.y,
    filter_imu_msg->angular_velocity.z
  );
  const tf2::Vector3 imu_angular_velocity_in_earth_frame = imu_to_earth_transform_tf * imu_angular_velocity_in_imu_frame;

  // Filter velocity ECEF message (not counted as updating)
  auto filter_velocity_ecef_msg = filter_velocity_ecef_pub_->getMessage();
  filter_velocity_ecef_msg->twist.twist.angular.x = imu_angular_velocity_in_earth_frame.x();
  filter_velocity_ecef_msg->twist.twist.angular.y = imu_angular_velocity_in_earth_frame.y();
  filter_velocity_ecef_msg->twist.twist.angular.z = imu_angular_velocity_in_earth_frame.z();
}

void Publishers::handleFilterCompAccel(const mip::data_filter::CompAccel& comp_accel, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  if (config_->filter_use_compensated_accel_)
  {
    auto filter_imu_msg = filter_imu_pub_->getMessageToUpdate();
    updateHeaderTime(&(filter_imu_msg->header), descriptor_set, timestamp);
    filter_imu_msg->linear_acceleration.x = comp_accel.accel[0];
    filter_imu_msg->linear_acceleration.y = comp_accel.accel[1];
    filter_imu_msg->linear_acceleration.z = comp_accel.accel[2];
    if (config_->use_enu_frame_)
    {
      filter_imu_msg->linear_acceleration.y *= -1.0;
      filter_imu_msg->linear_acceleration.z *= -1.0;
    }
  }
}

void Publishers::handleFilterLinearAccel(const mip::data_filter::LinearAccel& linear_accel, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  if (!config_->filter_use_compensated_accel_)
  {
    auto filter_imu_msg = filter_imu_pub_->getMessageToUpdate();
    updateHeaderTime(&(filter_imu_msg->header), descriptor_set, timestamp);
    filter_imu_msg->linear_acceleration.x = linear_accel.accel[0];
    filter_imu_msg->linear_acceleration.y = linear_accel.accel[1];
    filter_imu_msg->linear_acceleration.z = linear_accel.accel[2];
    if (config_->use_enu_frame_)
    {
      filter_imu_msg->linear_acceleration.y *= -1.0;
      filter_imu_msg->linear_acceleration.z *= -1.0;
    }
  }
}

void Publishers::handleFilterGnssPosAidStatus(const mip::data_filter::GnssPosAidStatus& gnss_pos_aid_status, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  // Filter GNSS position aiding status
  auto mip_filter_gnss_position_aiding_status_msg = mip_filter_gnss_position_aiding_status_pub_->getMessage();
  updateMipHeader(&(mip_filter_gnss_position_aiding_status_msg->header), descriptor_set, timestamp);
  mip_filter_gnss_position_aiding_status_msg->receiver_id = gnss_pos_aid_status.receiver_id;
  mip_filter_gnss_position_aiding_status_msg->time_of_week = gnss_pos_aid_status.time_of_week;
  mip_filter_gnss_position_aiding_status_msg->status.tight_coupling = gnss_pos_aid_status.status.tightCoupling();
  mip_filter_gnss_position_aiding_status_msg->status.differential = gnss_pos_aid_status.status.differential();
  mip_filter_gnss_position_aiding_status_msg->status.integer_fix = gnss_pos_aid_status.status.integerFix();
  mip_filter_gnss_position_aiding_status_msg->status.gps_l1 = gnss_pos_aid_status.status.gpsL1();
  mip_filter_gnss_position_aiding_status_msg->status.gps_l2 = gnss_pos_aid_status.status.gpsL2();
  mip_filter_gnss_position_aiding_status_msg->status.gps_l5 = gnss_pos_aid_status.status.gpsL5();
  mip_filter_gnss_position_aiding_status_msg->status.glo_l1 = gnss_pos_aid_status.status.gloL1();
  mip_filter_gnss_position_aiding_status_msg->status.glo_l2 = gnss_pos_aid_status.status.gloL2();
  mip_filter_gnss_position_aiding_status_msg->status.gal_e1 = gnss_pos_aid_status.status.galE1();
  mip_filter_gnss_position_aiding_status_msg->status.gal_e5 = gnss_pos_aid_status.status.galE5();
  mip_filter_gnss_position_aiding_status_msg->status.gal_e6 = gnss_pos_aid_status.status.galE6();
  mip_filter_gnss_position_aiding_status_msg->status.bei_b1 = gnss_pos_aid_status.status.beiB1();
  mip_filter_gnss_position_aiding_status_msg->status.bei_b2 = gnss_pos_aid_status.status.beiB2();
  mip_filter_gnss_position_aiding_status_msg->status.bei_b3 = gnss_pos_aid_status.status.beiB3();
  mip_filter_gnss_position_aiding_status_msg->status.no_fix = gnss_pos_aid_status.status.noFix();
  mip_filter_gnss_position_aiding_status_msg->status.config_error = gnss_pos_aid_status.status.configError();
  mip_filter_gnss_position_aiding_status_pub_->publish(*mip_filter_gnss_position_aiding_status_msg);

  // Filter fix message (not counted as updating)
  auto filter_llh_position_msg = filter_llh_position_pub_->getMessage();

  // Take the best out of the two receivers for GNSS status
  const uint8_t gnss_index = gnss_pos_aid_status.receiver_id - 1;
  auto filter_human_readable_status_msg = filter_human_readable_status_pub_->getMessage();
  if (mip_filter_gnss_position_aiding_status_msg->status.integer_fix)
  {
    filter_llh_position_msg->status.status = NavSatFixMsg::_status_type::STATUS_GBAS_FIX;
    filter_human_readable_status_msg->gnss_state = HumanReadableStatusMsg::GNSS_STATE_RTK_FIXED;
    rtk_fixed_ = true;
  }
  else if (!rtk_fixed_)
  {
    if (mip_filter_gnss_position_aiding_status_msg->status.differential)
    {
      filter_llh_position_msg->status.status = NavSatFixMsg::_status_type::STATUS_GBAS_FIX;
      filter_human_readable_status_msg->gnss_state = HumanReadableStatusMsg::GNSS_STATE_RTK_FLOAT;
      rtk_float_ = true;
    }
    else if (!rtk_float_)
    {
      if (mip_gnss_fix_info_pub_[gnss_index]->getMessage()->fix_flags.sbas_used)
      {
        filter_human_readable_status_msg->gnss_state = HumanReadableStatusMsg::GNSS_STATE_SBAS;
        filter_llh_position_msg->status.status = NavSatFixMsg::_status_type::STATUS_SBAS_FIX;
        has_sbas_ = true;
      }
      else if (!has_sbas_)
      {
        if (!mip_filter_gnss_position_aiding_status_msg->status.no_fix)
        {
          filter_human_readable_status_msg->gnss_state = HumanReadableStatusMsg::GNSS_STATE_3D_FIX;
          filter_llh_position_msg->status.status = NavSatFixMsg::_status_type::STATUS_FIX;
          has_fix_ = true;
        }
        else if (!has_fix_)
        {
          filter_human_readable_status_msg->gnss_state = HumanReadableStatusMsg::GNSS_STATE_NO_FIX;
          filter_llh_position_msg->status.status = NavSatFixMsg::_status_type::STATUS_NO_FIX;
        }
      }
    }
  }
}

void Publishers::handleFilterMultiAntennaOffsetCorrection(const mip::data_filter::MultiAntennaOffsetCorrection& multi_antenna_offset_correction, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  auto mip_filter_multi_antenna_offset_correction_msg = mip_filter_multi_antenna_offset_correction_pub_->getMessage();
  updateMipHeader(&(mip_filter_multi_antenna_offset_correction_msg->header), descriptor_set, timestamp);
  mip_filter_multi_antenna_offset_correction_msg->receiver_id = multi_antenna_offset_correction.receiver_id;
  mip_filter_multi_antenna_offset_correction_msg->offset[0] = multi_antenna_offset_correction.offset[0];
  mip_filter_multi_antenna_offset_correction_msg->offset[1] = multi_antenna_offset_correction.offset[1];
  mip_filter_multi_antenna_offset_correction_msg->offset[2] = multi_antenna_offset_correction.offset[2];
  mip_filter_multi_antenna_offset_correction_pub_->publish(*mip_filter_multi_antenna_offset_correction_msg);

  const tf2::Transform gnss_x_antenna_correction_to_microstrain_vehicle_tf(tf2::Quaternion::getIdentity(), tf2::Vector3(multi_antenna_offset_correction.offset[0], multi_antenna_offset_correction.offset[1], multi_antenna_offset_correction.offset[2]));
  TransformStampedMsg gnss_x_antenna_to_imu_link_transform = gnss_antenna_link_to_imu_link_transform_[multi_antenna_offset_correction.receiver_id - 1];
  gnss_x_antenna_to_imu_link_transform.header.stamp = rosTimeNow(node_);
  if (config_->use_enu_frame_)
  {
    const tf2::Transform gnss_x_antenna_correction_to_ros_vehicle_tf = config_->ros_vehicle_to_microstrain_vehicle_transform_tf_.inverse() * gnss_x_antenna_correction_to_microstrain_vehicle_tf;
    gnss_x_antenna_to_imu_link_transform.transform.translation.x += gnss_x_antenna_correction_to_ros_vehicle_tf.getOrigin().getX();
    gnss_x_antenna_to_imu_link_transform.transform.translation.y += gnss_x_antenna_correction_to_ros_vehicle_tf.getOrigin().getY();
    gnss_x_antenna_to_imu_link_transform.transform.translation.z += gnss_x_antenna_correction_to_ros_vehicle_tf.getOrigin().getZ();
  }
  else
  {
    gnss_x_antenna_to_imu_link_transform.transform.translation.x += gnss_x_antenna_correction_to_microstrain_vehicle_tf.getOrigin().getX();
    gnss_x_antenna_to_imu_link_transform.transform.translation.y += gnss_x_antenna_correction_to_microstrain_vehicle_tf.getOrigin().getY();
    gnss_x_antenna_to_imu_link_transform.transform.translation.z += gnss_x_antenna_correction_to_microstrain_vehicle_tf.getOrigin().getZ();
  }
  static_transform_broadcaster_->sendTransform(gnss_x_antenna_to_imu_link_transform);
}

void Publishers::handleFilterGnssDualAntennaStatus(const mip::data_filter::GnssDualAntennaStatus& gnss_dual_antenna_status, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  // The timestamp here may be old, only update if it has changed
  mip::data_shared::GpsTimestamp gps_timestamp;
  if (gps_timestamp_mapping_.find(descriptor_set) != gps_timestamp_mapping_.end())
    gps_timestamp = gps_timestamp_mapping_.at(descriptor_set);
  gps_timestamp.tow = gnss_dual_antenna_status.time_of_week;
  const double gps_timestamp_secs = gpsTimestampSecs(gps_timestamp);
  if (gps_timestamp_secs <= last_dual_antenna_heading_gps_timestamp_secs_)
    return;
  last_dual_antenna_heading_gps_timestamp_secs_ = gps_timestamp_secs;

  // Filter Dual Antenna Status (pose version)
  auto filter_dual_antenna_heading_msg = filter_dual_antenna_heading_pub_->getMessageToUpdate();
  updateHeaderTime(&(filter_dual_antenna_heading_msg->header), descriptor_set, timestamp, &gps_timestamp);
  tf2::Quaternion microstrain_vehicle_to_ned_quaternion_tf;
  microstrain_vehicle_to_ned_quaternion_tf.setRPY(0, 0, gnss_dual_antenna_status.heading);
  tf2::Transform microstrain_vehicle_to_ned_transform_tf(microstrain_vehicle_to_ned_quaternion_tf);
  if (config_->use_enu_frame_)
  {
    const tf2::Transform ros_vehicle_to_enu_transform_tf = config_->ned_to_enu_transform_tf_ * microstrain_vehicle_to_ned_transform_tf * config_->ros_vehicle_to_microstrain_vehicle_transform_tf_;
    filter_dual_antenna_heading_msg->pose.pose.orientation = tf2::toMsg(ros_vehicle_to_enu_transform_tf.getRotation());
  }
  else
  {
    filter_dual_antenna_heading_msg->pose.pose.orientation = tf2::toMsg(microstrain_vehicle_to_ned_transform_tf.getRotation());
  }
  filter_dual_antenna_heading_msg->pose.covariance[35] = pow(gnss_dual_antenna_status.heading_unc, 2);

  // Filter GNSS Dual Antenna status
  auto mip_filter_gnss_dual_antenna_status_msg = mip_filter_gnss_dual_antenna_status_pub_->getMessage();
  updateMipHeader(&(mip_filter_gnss_dual_antenna_status_msg->header), descriptor_set, timestamp);
  mip_filter_gnss_dual_antenna_status_msg->time_of_week = gnss_dual_antenna_status.time_of_week;
  mip_filter_gnss_dual_antenna_status_msg->heading = gnss_dual_antenna_status.heading;
  mip_filter_gnss_dual_antenna_status_msg->heading_unc = gnss_dual_antenna_status.heading_unc;
  mip_filter_gnss_dual_antenna_status_msg->fix_type = static_cast<uint8_t>(gnss_dual_antenna_status.fix_type);
  mip_filter_gnss_dual_antenna_status_msg->status_flags.rcv_1_data_valid = gnss_dual_antenna_status.status_flags.rcv1DataValid();
  mip_filter_gnss_dual_antenna_status_msg->status_flags.rcv_2_data_valid = gnss_dual_antenna_status.status_flags.rcv2DataValid();
  mip_filter_gnss_dual_antenna_status_msg->status_flags.antenna_offsets_valid = gnss_dual_antenna_status.status_flags.antennaOffsetsValid();
  mip_filter_gnss_dual_antenna_status_msg->valid_flags = gnss_dual_antenna_status.valid_flags;
  mip_filter_gnss_dual_antenna_status_pub_->publish(*mip_filter_gnss_dual_antenna_status_msg);

  // Filter Human Readable status
  auto filter_human_readable_status_msg = filter_human_readable_status_pub_->getMessage();
  if (gnss_dual_antenna_status.fix_type == mip::data_filter::GnssDualAntennaStatus::FixType::FIX_NONE)
    filter_human_readable_status_msg->dual_antenna_fix_type = HumanReadableStatusMsg::DUAL_ANTENNA_FIX_TYPE_NONE;
  if (gnss_dual_antenna_status.fix_type == mip::data_filter::GnssDualAntennaStatus::FixType::FIX_DA_FLOAT)
    filter_human_readable_status_msg->dual_antenna_fix_type = HumanReadableStatusMsg::DUAL_ANTENNA_FIX_TYPE_FLOAT;
  if (gnss_dual_antenna_status.fix_type == mip::data_filter::GnssDualAntennaStatus::FixType::FIX_DA_FIXED)
    filter_human_readable_status_msg->dual_antenna_fix_type = HumanReadableStatusMsg::DUAL_ANTENNA_FIX_TYPE_FIXED;
}

void Publishers::handleFilterAidingMeasurementSummary(const mip::data_filter::AidingMeasurementSummary& aiding_measurement_summary, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  auto mip_filter_aiding_measurement_summary_msg = mip_filter_aiding_measurement_summary_pub_->getMessage();
  updateMipHeader(&(mip_filter_aiding_measurement_summary_msg->header), descriptor_set, timestamp);
  mip_filter_aiding_measurement_summary_msg->time_of_week = aiding_measurement_summary.time_of_week;
  mip_filter_aiding_measurement_summary_msg->source = aiding_measurement_summary.source;
  mip_filter_aiding_measurement_summary_msg->type = static_cast<uint8_t>(aiding_measurement_summary.type);

  mip_filter_aiding_measurement_summary_msg->indicator.enabled = aiding_measurement_summary.indicator.enabled();
  mip_filter_aiding_measurement_summary_msg->indicator.used = aiding_measurement_summary.indicator.used();
  mip_filter_aiding_measurement_summary_msg->indicator.residual_high_warning = aiding_measurement_summary.indicator.residualHighWarning();
  mip_filter_aiding_measurement_summary_msg->indicator.sample_time_warning = aiding_measurement_summary.indicator.sampleTimeWarning();
  mip_filter_aiding_measurement_summary_msg->indicator.configuration_error = aiding_measurement_summary.indicator.configurationError();
  mip_filter_aiding_measurement_summary_msg->indicator.max_num_meas_exceeded = aiding_measurement_summary.indicator.maxNumMeasExceeded();
  mip_filter_aiding_measurement_summary_pub_->publish(*mip_filter_aiding_measurement_summary_msg);
}

void Publishers::handleSystemBuiltInTest(const mip::data_system::BuiltInTest& built_in_test, const uint8_t descriptor_set, mip::Timestamp timestamp)
{
  auto mip_system_built_in_test_msg = mip_system_built_in_test_pub_->getMessage();
  updateMipHeader(&(mip_system_built_in_test_msg->header), descriptor_set, timestamp);
  std::copy(std::begin(built_in_test.result), std::end(built_in_test.result), std::begin(mip_system_built_in_test_msg->result));
  mip_system_built_in_test_pub_->publish(*mip_system_built_in_test_msg);

  // Parse out the BIT into the human readable status message
  auto filter_human_readable_status_msg = filter_human_readable_status_pub_->getMessage();
  filter_human_readable_status_msg->continuous_bit_flags.clear();
  if (RosMipDevice::isGq7(config_->mip_device_->device_info_))
  {
    mip::data_system::Gq7ContinuousBuiltInTest gq7_built_in_test = built_in_test;
    if (gq7_built_in_test.systemClockFailure())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_GQ7_SYSTEM_CLOCK_FAILURE);
    if (gq7_built_in_test.powerFault())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_GQ7_POWER_FAULT);
    if (gq7_built_in_test.firmwareFault())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_GQ7_FIRMWARE_FAULT);
    if (gq7_built_in_test.timingOverload())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_GQ7_TIMING_OVERLOAD);
    if (gq7_built_in_test.bufferOverrun())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_GQ7_BUFFER_OVERRUN);
    if (gq7_built_in_test.imuIpcFault())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_GQ7_IMU_IPC_FAULT);
    if (gq7_built_in_test.filterIpcFault())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_GQ7_FILTER_IPC_FAULT);
    if (gq7_built_in_test.gnssIpcFault())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_GQ7_GNSS_IPC_FAULT);
    if (gq7_built_in_test.imuClockFault())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_GQ7_IMU_CLOCK_FAULT);
    if (gq7_built_in_test.imuCommunicationFault())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_GQ7_IMU_COMMUNICATION_FAULT);
    if (gq7_built_in_test.imuTimingOverrun())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_GQ7_IMU_TIMING_OVERRUN);
    if (gq7_built_in_test.imuCalibrationErrorAccel())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_GQ7_IMU_CALIBRATION_ERROR_ACCEL);
    if (gq7_built_in_test.imuCalibrationErrorGyro())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_GQ7_IMU_CALIBRATION_ERROR_GYRO);
    if (gq7_built_in_test.imuCalibrationErrorMag())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_GQ7_IMU_CALIBRATION_ERROR_MAG);
    if (gq7_built_in_test.accelerometerGeneralFault())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_GQ7_ACCELEROMETER_GENERAL_FAULT);
    if (gq7_built_in_test.accelerometerOverRange())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_GQ7_ACCELEROMETER_OVER_RANGE);
    if (gq7_built_in_test.accelerometerSelfTestFail())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_GQ7_ACCELEROMETER_SELF_TEST_FAIL);
    if (gq7_built_in_test.gyroscopeGeneralFault())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_GQ7_GYROSCOPE_GENERAL_FAULT);
    if (gq7_built_in_test.gyroscopeOverRange())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_GQ7_GYROSCOPE_OVER_RANGE);
    if (gq7_built_in_test.gyroscopeSelfTestFail())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_GQ7_GYROSCOPE_SELF_TEST_FAIL);
    if (gq7_built_in_test.magnetometerGeneralFault())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_GQ7_MAGNETOMETER_GENERAL_FAULT);
    if (gq7_built_in_test.magnetometerOverRange())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_GQ7_MAGNETOMETER_OVER_RANGE);
    if (gq7_built_in_test.magnetometerSelfTestFail())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_GQ7_MAGNETOMETER_SELF_TEST_FAIL);
    if (gq7_built_in_test.pressureSensorGeneralFault())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_GQ7_PRESSURE_SENSOR_GENERAL_FAULT);
    if (gq7_built_in_test.pressureSensorOverRange())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_GQ7_PRESSURE_SENSOR_OVER_RANGE);
    if (gq7_built_in_test.pressureSensorSelfTestFail())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_GQ7_PRESSURE_SENSOR_SELF_TEST_FAIL);
    if (gq7_built_in_test.filterClockFault())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_GQ7_FILTER_CLOCK_FAULT);
    if (gq7_built_in_test.filterHardwareFault())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_GQ7_FILTER_HARDWARE_FAULT);
    if (gq7_built_in_test.filterTimingOverrun())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_GQ7_FILTER_TIMING_OVERRUN);
    if (gq7_built_in_test.filterTimingUnderrun())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_GQ7_FILTER_TIMING_UNDERRUN);
    if (gq7_built_in_test.filterCommunicationError())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_GQ7_FILTER_COMMUNICATION_ERROR);
    if (gq7_built_in_test.gnssClockFault())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_GQ7_GNSS_CLOCK_FAULT);
    if (gq7_built_in_test.gnssHardwareFault())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_GQ7_GNSS_HARDWARE_FAULT);
    if (gq7_built_in_test.gnssCommunicationError())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_GQ7_GNSS_COMMUNICATION_ERROR);
    if (gq7_built_in_test.gpsTimeFault())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_GQ7_GPS_TIME_FAULT);
    if (gq7_built_in_test.gnssTimingOverrun())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_GQ7_GNSS_TIMING_OVERRUN);
    if (gq7_built_in_test.gnssReceiver1PowerFault())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_GQ7_GNSS_RECEIVER_1_POWER_FAULT);
    if (gq7_built_in_test.gnssReceiver1Fault())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_GQ7_GNSS_RECEIVER_1_FAULT);
    if (gq7_built_in_test.gnssAntenna1Shorted())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_GQ7_GNSS_ANTENNA_1_SHORTED);
    if (gq7_built_in_test.gnssAntenna1Open())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_GQ7_GNSS_ANTENNA_1_OPEN);
    if (gq7_built_in_test.gnssReceiver1SolutionFault())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_GQ7_GNSS_RECEIVER_1_SOLUTION_FAULT);
    if (gq7_built_in_test.gnssReceiver2PowerFault())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_GQ7_GNSS_RECEIVER_2_POWER_FAULT);
    if (gq7_built_in_test.gnssReceiver2Fault())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_GQ7_GNSS_RECEIVER_2_FAULT);
    if (gq7_built_in_test.gnssAntenna2Shorted())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_GQ7_GNSS_ANTENNA_2_SHORTED);
    if (gq7_built_in_test.gnssAntenna2Open())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_GQ7_GNSS_ANTENNA_2_OPEN);
    if (gq7_built_in_test.gnssReceiver2SolutionFault())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_GQ7_GNSS_RECEIVER_2_SOLUTION_FAULT);
    if (gq7_built_in_test.rtcmCommunicationFault())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_GQ7_RTCM_COMMUNICATION_FAULT);
    if (gq7_built_in_test.rtkDongleFault())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_GQ7_RTK_DONGLE_FAULT);
  }
  else if (RosMipDevice::isCv7(config_->mip_device_->device_info_))
  {
    mip::data_system::Cv7ContinuousBuiltInTest cv7_built_in_test = built_in_test;
    if (cv7_built_in_test.systemClockFailure())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_CV7_SYSTEM_CLOCK_FAILURE);
    if (cv7_built_in_test.powerFault())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_CV7_POWER_FAULT);
    if (cv7_built_in_test.firmwareFault())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_CV7_FIRMWARE_FAULT);
    if (cv7_built_in_test.timingOverload())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_CV7_TIMING_OVERLOAD);
    if (cv7_built_in_test.bufferOverrun())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_CV7_BUFFER_OVERRUN);
    if (cv7_built_in_test.imuProcessFault())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_CV7_IMU_PROCESS_FAULT);
    if (cv7_built_in_test.imuDataRateMismatch())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_CV7_IMU_DATA_RATE_MISMATCH);
    if (cv7_built_in_test.imuOverrunDroppedData())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_CV7_IMU_OVERRUN_DROPPED_DATA);
    if (cv7_built_in_test.imuStuck())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_CV7_IMU_STUCK);
    if (cv7_built_in_test.filterProcessFault())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_CV7_FILTER_PROCESS_FAULT);
    if (cv7_built_in_test.filterDroppedData())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_CV7_FILTER_DROPPED_DATA);
    if (cv7_built_in_test.filterRateMismatch())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_CV7_FILTER_RATE_MISMATCH);
    if (cv7_built_in_test.filterStuck())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_CV7_FILTER_STUCK);
    if (cv7_built_in_test.imuClockFault())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_CV7_IMU_CLOCK_FAULT);
    if (cv7_built_in_test.imuCommunicationFault())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_CV7_IMU_COMMUNICATION_FAULT);
    if (cv7_built_in_test.imuTimingOverrun())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_CV7_IMU_TIMING_OVERRUN);
    if (cv7_built_in_test.imuCalibrationErrorAccelerometer())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_CV7_IMU_CALIBRATION_ERROR_ACCELEROMETER);
    if (cv7_built_in_test.imuCalibrationErrorGyroscope())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_CV7_IMU_CALIBRATION_ERROR_GYROSCOPE);
    if (cv7_built_in_test.imuCalibrationErrorMagnetometer())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_CV7_IMU_CALIBRATION_ERROR_MAGNETOMETER);
    if (cv7_built_in_test.accelerometerGeneralFault())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_CV7_ACCELEROMETER_GENERAL_FAULT);
    if (cv7_built_in_test.accelerometerOverRange())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_CV7_ACCELEROMETER_OVER_RANGE);
    if (cv7_built_in_test.accelerometerSelfTestFail())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_CV7_ACCELEROMETER_SELF_TEST_FAIL);
    if (cv7_built_in_test.gyroscopeGeneralFault())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_CV7_GYROSCOPE_GENERAL_FAULT);
    if (cv7_built_in_test.gyroscopeOverRange())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_CV7_GYROSCOPE_OVER_RANGE);
    if (cv7_built_in_test.gyroscopeSelfTestFail())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_CV7_GYROSCOPE_SELF_TEST_FAIL);
    if (cv7_built_in_test.magnetometerGeneralFault())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_CV7_MAGNETOMETER_GENERAL_FAULT);
    if (cv7_built_in_test.magnetometerOverRange())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_CV7_MAGNETOMETER_OVER_RANGE);
    if (cv7_built_in_test.magnetometerSelfTestFail())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_CV7_MAGNETOMETER_SELF_TEST_FAIL);
    if (cv7_built_in_test.pressureSensorGeneralFault())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_CV7_PRESSURE_SENSOR_GENERAL_FAULT);
    if (cv7_built_in_test.pressureSensorOverRange())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_CV7_PRESSURE_SENSOR_OVER_RANGE);
    if (cv7_built_in_test.pressureSensorSelfTestFail())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_CV7_PRESSURE_SENSOR_SELF_TEST_FAIL);
    if (cv7_built_in_test.factoryBitsInvalid())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_CV7_FACTORY_BITS_INVALID);
    if (cv7_built_in_test.filterFault())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_CV7_FILTER_FAULT);
    if (cv7_built_in_test.filterTimingOverrun())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_CV7_FILTER_TIMING_OVERRUN);
    if (cv7_built_in_test.filterTimingUnderrun())
      filter_human_readable_status_msg->continuous_bit_flags.push_back(filter_human_readable_status_msg->CONTINUOUS_BIT_FLAGS_CV7_FILTER_TIMING_UNDERRUN);
  }
}

void Publishers::handleAfterPacket(const mip::PacketRef& packet, mip::Timestamp timestamp)
{
  // Publish all the messages that have been updated
  publish();

  // Reset some shared descriptors. These are unique to the packets, and we do not want to cache them past this packet
  if (event_source_mapping_.find(packet.descriptorSet()) != event_source_mapping_.end())
    event_source_mapping_[packet.descriptorSet()].trigger_id = 0;

  // Reset whether or not we have RTK
  rtk_fixed_ = false;
  rtk_float_ = false;
  has_sbas_ = false;
  has_fix_ = false;
}

void Publishers::updateMipHeader(MipHeaderMsg* mip_header, uint8_t descriptor_set, mip::Timestamp timestamp, const mip::data_shared::GpsTimestamp* gps_timestamp)
{
  // Update the ROS header with the ROS timestamp
  updateHeaderTime(&mip_header->header, descriptor_set, timestamp, gps_timestamp);

  // Default frame ID is determined by the descriptor set
  if (descriptor_set == mip::data_gnss::DESCRIPTOR_SET || descriptor_set == mip::data_gnss::MIP_GNSS1_DATA_DESC_SET)
    mip_header->header.frame_id = config_->gnss_frame_id_[GNSS1_ID];
  else if (descriptor_set == mip::data_gnss::MIP_GNSS2_DATA_DESC_SET)
    mip_header->header.frame_id = config_->gnss_frame_id_[GNSS2_ID];
  else
    mip_header->header.frame_id = config_->frame_id_;

  // Set the event source if it was set for this packet
  if (event_source_mapping_.find(descriptor_set) != event_source_mapping_.end())
    mip_header->event_source = event_source_mapping_.at(descriptor_set).trigger_id;
  else
    mip_header->event_source = 0;

  // Set the most recent reference timestamp if we have one
  if (reference_timestamp_mapping_.find(descriptor_set) != reference_timestamp_mapping_.end())
    mip_header->reference_timestamp = reference_timestamp_mapping_.at(descriptor_set).nanoseconds;

  // Set the GPS timestamp if we have one (should always have one)
  mip::data_shared::GpsTimestamp gps_timestamp_copy;
  if (gps_timestamp != nullptr)
    gps_timestamp_copy = *gps_timestamp;
  else if (gps_timestamp_mapping_.find(descriptor_set) != gps_timestamp_mapping_.end())
    gps_timestamp_copy = gps_timestamp_mapping_.at(descriptor_set);
  mip_header->gps_timestamp.week_number = gps_timestamp_copy.week_number;
  mip_header->gps_timestamp.tow = gps_timestamp_copy.tow;
}

void Publishers::updateHeaderTime(RosHeaderType* header, uint8_t descriptor_set, mip::Timestamp timestamp, const mip::data_shared::GpsTimestamp* gps_timestamp)
{
  // Find the right GPS timestamp to use (may not be used)
  mip::data_shared::GpsTimestamp gps_timestamp_copy;
  if (gps_timestamp != nullptr)
    gps_timestamp_copy = *gps_timestamp;
  else if (gps_timestamp_mapping_.find(descriptor_set) != gps_timestamp_mapping_.end())
    gps_timestamp_copy = gps_timestamp_mapping_.at(descriptor_set);

  // Set the timestamp depending on how the node was configured
  if (config_->timestamp_source_ == TIMESTAMP_SOURCE_ROS)
  {
    setRosTime(&header->stamp, static_cast<double>(timestamp) / 1000.0);
  }
  else if (config_->timestamp_source_ == TIMESTAMP_SOURCE_MIP)
  {
    setGpsTime(&header->stamp, gps_timestamp_copy);
  }
  else if (config_->timestamp_source_ == TIMESTAMP_SOURCE_HYBRID)
  {
    double utc_timestamp = 0;
    if (clock_bias_monitor_.hasBiasEstimate())
    {
      const double current_utc_timestamp = gpsTimestampSecs(gps_timestamp_copy) - clock_bias_monitor_.getBiasEstimate();
      const double previous_utc_timestamp = previous_utc_timestamps_.find(descriptor_set) != previous_utc_timestamps_.end() ? previous_utc_timestamps_.at(descriptor_set) : 0;
      const double utc_timestamp_dt = current_utc_timestamp - previous_utc_timestamp;
      if (utc_timestamp_dt >= 0)
        utc_timestamp = current_utc_timestamp;
      else
        clock_bias_monitor_.reset();
      if (current_utc_timestamp != previous_utc_timestamp)
        previous_utc_timestamps_[descriptor_set] = current_utc_timestamp;
    }
    if (utc_timestamp == 0)
      utc_timestamp = static_cast<double>(timestamp) / 1000.0;
    double utc_timestamp_seconds;
    const double utc_timestamp_subseconds = modf(utc_timestamp, &utc_timestamp_seconds);
    setRosTime(&header->stamp, static_cast<int32_t>(utc_timestamp_seconds), static_cast<int32_t>(utc_timestamp_subseconds * 1000000000));
  }
}

void Publishers::setGpsTime(RosTimeType* time, const mip::data_shared::GpsTimestamp& timestamp)
{
  // Split the seconds and subseconds out to get around the double resolution issue
  double seconds;
  double subseconds = modf(timestamp.tow, &seconds);

  // Seconds since start of Unix time = seconds between 1970 and 1980 + number of weeks since 1980 * number of seconds in a week + number of complete seconds past in current week - leap seconds since start of GPS time
  const uint64_t utc_milliseconds = static_cast<uint64_t>((315964800 + timestamp.week_number * 604800 + static_cast<uint64_t>(seconds) - GPS_LEAP_SECONDS) * 1000L) + static_cast<uint64_t>(std::round(subseconds * 1000.0));
  setRosTime(time, static_cast<double>(utc_milliseconds) / 1000.0);
}

}  // namespace microstrain

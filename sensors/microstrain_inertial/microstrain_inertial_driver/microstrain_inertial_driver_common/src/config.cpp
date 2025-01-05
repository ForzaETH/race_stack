/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Parker-Lord Inertial Device Driver Implementation File
//
// Copyright (c) 2017, Brian Bingham
// Copyright (c) 2020, Parker Hannifin Corp
// This code is licensed under MIT license (see LICENSE file for details)
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

#include <errno.h>
#include <tuple>
#include <vector>
#include <string>
#include <memory>
#include <algorithm>

#include <GeographicLib/Geocentric.hpp>

#include "mip/mip_version.h"
#include "mip/definitions/commands_base.hpp"
#include "mip/definitions/commands_3dm.hpp"
#include "mip/definitions/commands_gnss.hpp"
#include "mip/definitions/commands_aiding.hpp"
#include "mip/definitions/data_sensor.hpp"
#include "mip/definitions/data_gnss.hpp"
#include "mip/definitions/data_filter.hpp"
#include "mip/platform/serial_connection.hpp"
#include "mip/extras/recording_connection.hpp"

#include "microstrain_inertial_driver_common/utils/mappings/mip_mapping.h"
#include "microstrain_inertial_driver_common/config.h"

namespace microstrain
{

Config::Config(RosNodeType* node) : node_(node)
{
  nmea_max_rate_hz_ = 0;

  // Initialize the transform buffer and listener ahead of time
  transform_buffer_ = createTransformBuffer(node_);
  transform_listener_ = createTransformListener(transform_buffer_);
}

bool Config::configure(RosNodeType* node)
{
  // Initialize some default and static config
  ned_to_enu_transform_tf_ = tf2::Transform(tf2::Matrix3x3(
    0, 1, 0,
    1, 0, 0,
    0, 0, -1
  ));
  ros_vehicle_to_microstrain_vehicle_transform_tf_ = tf2::Transform(tf2::Matrix3x3(
    1,  0,  0,
    0, -1,  0,
    0,  0, -1
  ));

  ///
  /// Generic configuration used by the rest of the driver
  ///

  // General
  getParam<bool>(node, "debug", debug_, false);
  getParam<bool>(node, "device_setup", device_setup_, true);

  // Reconnect
  getParam<int>(node, "reconnect_attempts", reconnect_attempts_, 0);
  getParam<bool>(node, "configure_after_reconnect", configure_after_reconnect_, true);

  // Timestamp source
  getParam<int>(node, "timestamp_source", timestamp_source_, 2);

  // Frame ID config
  getParam<std::string>(node, "frame_id", frame_id_, "imu_link");
  getParam<std::string>(node, "target_frame_id", target_frame_id_, "base_link");
  getParam<std::string>(node, "mount_frame_id", mount_frame_id_, "base_link");
  getParam<std::string>(node, "map_frame_id", map_frame_id_, "map");
  getParam<std::string>(node, "earth_frame_id", earth_frame_id_, "earth");
  getParam<std::string>(node, "gnss1_frame_id", gnss_frame_id_[GNSS1_ID], "gnss_1_antenna_link");
  getParam<std::string>(node, "gnss2_frame_id", gnss_frame_id_[GNSS2_ID], "gnss_2_antenna_link");
  getParam<std::string>(node, "odometer_frame_id", odometer_frame_id_, "odometer_link");
  getParam<bool>(node, "use_enu_frame", use_enu_frame_, false);

  // tf config
  getParam<int32_t>(node, "tf_mode", tf_mode_, TF_MODE_GLOBAL);
  getParam<bool>(node, "publish_mount_to_frame_id_transform", publish_mount_to_frame_id_transform_, true);

  // If using the NED frame, append that to the map frame ID
  if (!use_enu_frame_)
  {
    constexpr char ned_suffix[] = "_ned";
    map_frame_id_ += ned_suffix;
  }

  // Configure the static transforms
  std::vector<double> mount_to_frame_id_transform_vec;
  getParam<std::vector<double>>(node, "mount_to_frame_id_transform", mount_to_frame_id_transform_vec, {0, 0, 0, 0, 0, 0, 1});

  if (mount_to_frame_id_transform_vec.size() != 7)
  {
    MICROSTRAIN_ERROR(node, "mount_to_frame_id_transform  is invalid. Should have 7 elements (x, y, z, i, j, k, w), but has %lu", mount_to_frame_id_transform_vec.size());
    return false;
  }

  mount_to_frame_id_transform_.header.stamp = rosTimeNow(node);
  mount_to_frame_id_transform_.header.frame_id = mount_frame_id_;
  mount_to_frame_id_transform_.child_frame_id = frame_id_;
  mount_to_frame_id_transform_.transform.translation.x = mount_to_frame_id_transform_vec[0];
  mount_to_frame_id_transform_.transform.translation.y = mount_to_frame_id_transform_vec[1];
  mount_to_frame_id_transform_.transform.translation.z = mount_to_frame_id_transform_vec[2];
  mount_to_frame_id_transform_.transform.rotation.x = mount_to_frame_id_transform_vec[3];
  mount_to_frame_id_transform_.transform.rotation.y = mount_to_frame_id_transform_vec[4];
  mount_to_frame_id_transform_.transform.rotation.z = mount_to_frame_id_transform_vec[5];
  mount_to_frame_id_transform_.transform.rotation.w = mount_to_frame_id_transform_vec[6];

  // IMU
  getParam<std::vector<double>>(node, "imu_orientation_cov", imu_orientation_cov_, DEFAULT_MATRIX);
  getParam<std::vector<double>>(node, "imu_linear_cov", imu_linear_cov_, DEFAULT_MATRIX);
  getParam<std::vector<double>>(node, "imu_angular_cov", imu_angular_cov_, DEFAULT_MATRIX);
  getParam<std::vector<double>>(node, "imu_mag_cov", imu_mag_cov_, DEFAULT_MATRIX);
  getParam<double>(node, "imu_pressure_variance", imu_pressure_vairance_, 0.01);

  // GNSS 1/2
  std::vector<double> gnss_antenna_offset_double[NUM_GNSS];
  getParam<int>(node, "gnss1_antenna_offset_source", gnss_antenna_offset_source_[GNSS1_ID], OFFSET_SOURCE_MANUAL);
  getParam<int>(node, "gnss2_antenna_offset_source", gnss_antenna_offset_source_[GNSS2_ID], OFFSET_SOURCE_MANUAL);
  getParam<std::vector<double>>(node, "gnss1_antenna_offset", gnss_antenna_offset_double[GNSS1_ID], DEFAULT_VECTOR);
  getParam<std::vector<double>>(node, "gnss2_antenna_offset", gnss_antenna_offset_double[GNSS2_ID], DEFAULT_VECTOR);

  // HARDWARE ODOM
  getParam<bool>(node, "enable_hardware_odometer", enable_hardware_odometer_, false);

  // RTK/GQ7 specific
  getParam<bool>(node, "rtk_dongle_enable", rtk_dongle_enable_, true);
  getParam<bool>(node, "ntrip_interface_enable", ntrip_interface_enable_, false);
  rtk_dongle_enable_ = rtk_dongle_enable_ || ntrip_interface_enable_;  // If the NTRIP interface is enabled, we will enable the RTK interface

  // FILTER
  std::vector<double> filter_speed_lever_arm_double(3, 0.0);
  getParam<bool>(node, "filter_relative_position_config", filter_relative_pos_config_, false);
  getParam<int>(node, "filter_relative_position_source", filter_relative_pos_source_, 2);
  getParam<int32_t>(node, "filter_relative_position_frame", filter_relative_pos_frame_, 2);
  getParam<std::vector<double>>(node, "filter_relative_position_ref", filter_relative_pos_ref_, DEFAULT_VECTOR);
  getParam<double>(node, "gps_leap_seconds", gps_leap_seconds_, 18.0);
  getParam<bool>(node, "filter_enable_gnss_heading_aiding", filter_enable_gnss_heading_aiding_, true);
  getParam<bool>(node, "filter_enable_gnss_pos_vel_aiding", filter_enable_gnss_pos_vel_aiding_, true);
  getParam<bool>(node, "filter_enable_altimeter_aiding", filter_enable_altimeter_aiding_, false);
  getParam<bool>(node, "filter_enable_odometer_aiding", filter_enable_odometer_aiding_, false);
  getParam<bool>(node, "filter_enable_magnetometer_aiding", filter_enable_magnetometer_aiding_, false);
  getParam<bool>(node, "filter_enable_external_heading_aiding", filter_enable_external_heading_aiding_, false);
  getParam<bool>(node, "filter_enable_external_gps_time_update", filter_enable_external_gps_time_update_, false);
  getParam<bool>(node, "filter_enable_wheeled_vehicle_constraint", filter_enable_wheeled_vehicle_constraint_, false);
  getParam<bool>(node, "filter_enable_vertical_gyro_constraint", filter_enable_vertical_gyro_constraint_, false);
  getParam<bool>(node, "filter_enable_gnss_antenna_cal", filter_enable_gnss_antenna_cal_, false);
  getParam<bool>(node, "filter_use_compensated_accel", filter_use_compensated_accel_, true);
  getParam<int>(node, "filter_speed_lever_arm_source", filter_speed_lever_arm_source_, OFFSET_SOURCE_MANUAL);
  getParam<std::vector<double>>(node, "filter_speed_lever_arm", filter_speed_lever_arm_double, DEFAULT_VECTOR);
  filter_speed_lever_arm_ = std::vector<float>(filter_speed_lever_arm_double.begin(), filter_speed_lever_arm_double.end());

  // Subscribers
  getParam<bool>(node, "subscribe_ext_time", subscribe_ext_time_, false);
  getParam<bool>(node, "subscribe_ext_fix", subscribe_ext_fix_, false);
  getParam<bool>(node, "subscribe_ext_vel_ned", subscribe_ext_vel_ned_, false);
  getParam<bool>(node, "subscribe_ext_vel_enu", subscribe_ext_vel_enu_, false);
  getParam<bool>(node, "subscribe_ext_vel_ecef", subscribe_ext_vel_ecef_, false);
  getParam<bool>(node, "subscribe_ext_vel_body", subscribe_ext_vel_body_, false);
  getParam<bool>(node, "subscribe_ext_heading_ned", subscribe_ext_heading_ned_, false);
  getParam<bool>(node, "subscribe_ext_heading_enu", subscribe_ext_heading_enu_, false);
  getParam<bool>(node, "subscribe_ext_mag", subscribe_ext_mag_, false);
  getParam<bool>(node, "subscribe_ext_pressure", subscribe_ext_pressure_, false);

  // NMEA streaming
  getParam<bool>(node, "nmea_message_allow_duplicate_talker_ids", nmea_message_allow_duplicate_talker_ids_, false);

  // Raw data file save
  getParam<bool>(node, "raw_file_enable", raw_file_enable_, false);
  getParam<bool>(node, "raw_file_include_support_data", raw_file_include_support_data_, false);

  // ROS2 can only fetch double vectors from config, so convert the doubles to floats for the MIP SDK
  for (int i = 0; i < NUM_GNSS; i++)
    gnss_antenna_offset_[i] = std::vector<float>(gnss_antenna_offset_double[i].begin(), gnss_antenna_offset_double[i].end());

  // Log the driver version if it was built properly
  MICROSTRAIN_INFO(node_, "Running microstrain_inertial_driver version: %s", MICROSTRAIN_DRIVER_VERSION);

  // Log the MIP SDK version
  MICROSTRAIN_INFO(node_, "Using MIP SDK version: %s", MIP_SDK_VERSION_FULL);

  // Do some configuration validation
  if (!filter_relative_pos_config_ && device_setup_)
  {
    MICROSTRAIN_WARN(node_, "No relative position configured. We will not publish relative odometry or transforms.");
    MICROSTRAIN_WARN(node_, "  Please configure relative position to publish relative position data");
    setParam<float>(node, mip_publisher_mapping_->static_topic_to_data_rate_config_key_mapping_.at(FILTER_ODOMETRY_MAP_TOPIC).c_str(), DATA_CLASS_DATA_RATE_DO_NOT_STREAM);
  }

  // Connect to the device and set it up if we were asked to
  if (!connectDevice(node))
    return false;

  if (!setupDevice(node))
    return false;

  return true;
}

bool Config::connectDevice(RosNodeType* node)
{
  // Open the device interface
  mip_device_ = std::make_shared<RosMipDeviceMain>(node_);
  if (!mip_device_->configure(node))
    return false;

  // Connect the aux port
  if (ntrip_interface_enable_)
  {
    aux_device_ = std::make_shared<RosMipDeviceAux>(node_);
    if (!aux_device_->configure(node))
    {
      MICROSTRAIN_ERROR(node_, "Failed to open aux port");
      return false;
    }
    aux_device_->connection()->shouldParseNmea(ntrip_interface_enable_);
  }

  return true;
}

bool Config::setupDevice(RosNodeType* node)
{
  // Read the config used by this section
  bool save_settings;
  bool filter_reset_after_config;
  getParam<bool>(node, "save_settings", save_settings, true);
  getParam<bool>(node, "filter_reset_after_config", filter_reset_after_config, true);

  mip::CmdResult mip_cmd_result;

  // Configure the device to stream data using the topic mapping
  MICROSTRAIN_DEBUG(node_, "Setting up data streams");
  mip_publisher_mapping_ = std::make_shared<MipPublisherMapping>(node_, mip_device_);
  if (!mip_publisher_mapping_->configure(node))
    return false;

  // If the device has no way of obtaining a global position, disable global transform mode
  if (tf_mode_ == TF_MODE_GLOBAL && !mip_device_->supportsDescriptor(mip::data_filter::DESCRIPTOR_SET, mip::data_filter::EcefPos::FIELD_DESCRIPTOR) && !mip_device_->supportsDescriptor(mip::data_filter::DESCRIPTOR_SET, mip::data_filter::PositionLlh::FIELD_DESCRIPTOR))
  {
    MICROSTRAIN_ERROR(node_, "Device does not support Global tf_mode as it has no way of obtaining global position");
    return false;
  }

  // Send commands to the device to configure it
  if (device_setup_)
  {
    MICROSTRAIN_DEBUG(node_, "Configuring device");
    if (!configureBase(node) ||
        !configure3DM(node) ||
        !configureGNSS(node) ||
        !configureFilter(node))
      return false;

    // Save the settings to the device, if enabled
    if (save_settings)
    {
      if (mip_device_->supportsDescriptor(mip::commands_3dm::DESCRIPTOR_SET, mip::commands_3dm::CMD_DEVICE_SETTINGS))
      {
        MICROSTRAIN_INFO(node_, "Saving the launch file configuration settings to the device");
        if (!(mip_cmd_result = mip::commands_3dm::saveDeviceSettings(*mip_device_)))
        {
          MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to save device settings");
          return false;
        }
      }
      else
      {
        MICROSTRAIN_WARN(node_, "Device does not support the device settings command");
      }
    }
    else
    {
      MICROSTRAIN_INFO(node_, "Note: The settings were not saved as startup settings. Power cycling will remove changes from device");
    }

    // Reset the filter, if enabled
    if (filter_reset_after_config)
    {
      if (mip_device_->supportsDescriptor(mip::commands_filter::DESCRIPTOR_SET, mip::commands_filter::CMD_RESET_FILTER))
      {
        MICROSTRAIN_INFO(node_, "Resetting the filter after the configuration is complete.");
        if (!(mip_cmd_result = mip::commands_filter::reset(*mip_device_)))
        {
          MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to reset filter");
          return false;
        }
      }
      else
      {
        MICROSTRAIN_WARN(node_, "Device does not support the filter reset command");
      }
    }
    else
    {
      MICROSTRAIN_INFO(node_, "Note: The filter was not reset after configuration.");
    }
  }
  return true;
}

bool Config::configureBase(RosNodeType* node)
{
  // Read local config
  bool set_baud;
  int32_t aux_baudrate;
  getParam<bool>(node, "set_baud", set_baud, false);
  getParam<int32_t>(node, "aux_baudrate", aux_baudrate, 115200);

  mip::CmdResult mip_cmd_result;
  const uint8_t descriptor_set = mip::commands_base::DESCRIPTOR_SET;

  // We will handle setting the aux port baudrate here if we were requested to do so
  // We will not handle setting the main port baudrate because that is handled in the connection class in a way that will always work
  if (mip_device_->supportsDescriptor(descriptor_set, mip::commands_base::CMD_COMM_SPEED))
  {
    if (set_baud)
    {
      // Only set the baudrate if the device has an aux port (we can check by fetching the baudrate)
      uint32_t tmp_baud;
      if (!!(mip_cmd_result = mip::commands_base::readCommSpeed(*mip_device_, 2, &tmp_baud)))
      {
        MICROSTRAIN_INFO(node_, "Note: Setting aux port baudrate to %d", aux_baudrate);
        if (!(mip_cmd_result = mip::commands_base::writeCommSpeed(*mip_device_, 2, aux_baudrate)))
        {
          MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to write aux port baudrate");
          return false;
        }

        // Reopen the aux port if it is already open
        if (aux_device_ != nullptr)
        {
          if (!aux_device_->reconnect())
          {
            MICROSTRAIN_ERROR(node_, "Failed to open aux port after configuring baudrate");
            return false;
          }
        }
      }
    }
  }
  else
  {
    MICROSTRAIN_INFO(node_, "Device does not support comm speed command");
  }

  return true;
}

bool Config::configure3DM(RosNodeType* node)
{
  // Read local config
  bool gpio_config;
  bool nmea_message_config;
  int filter_pps_source;
  float hardware_odometer_scaling;
  float hardware_odometer_uncertainty;
  bool sbas_enable, sbas_enable_ranging, sbas_enable_corrections, sbas_apply_integrity;
  std::vector<uint16_t> sbas_prns;
  bool low_pass_filter_config;
  bool accel_low_pass_filter_enable, accel_low_pass_filter_auto;
  float accel_low_pass_filter_frequency;
  bool gyro_low_pass_filter_enable, gyro_low_pass_filter_auto;
  float gyro_low_pass_filter_frequency;
  bool mag_low_pass_filter_enable, mag_low_pass_filter_auto;
  float mag_low_pass_filter_frequency;
  bool pressure_low_pass_filter_enable, pressure_low_pass_filter_auto;
  float pressure_low_pass_filter_frequency;
  getParam<bool>(node, "gpio_config", gpio_config, false);
  getParam<bool>(node, "nmea_message_config", nmea_message_config, false);
  getParam<int32_t>(node, "filter_pps_source", filter_pps_source, 1);
  getParam<float>(node, "odometer_scaling", hardware_odometer_scaling, 0.0);
  getParam<float>(node, "odometer_uncertainty", hardware_odometer_uncertainty, 0.0);
  getParam<bool>(node, "sbas_enable", sbas_enable, false);
  getParam<bool>(node, "sbas_enable_ranging", sbas_enable_ranging, false);
  getParam<bool>(node, "sbas_enable_corrections", sbas_enable_corrections, false);
  getParam<bool>(node, "sbas_apply_integrity", sbas_apply_integrity, false);
  getUint16ArrayParam(node, "sbas_included_prns", sbas_prns, std::vector<uint16_t>());
  getParam<bool>(node, "low_pass_filter_config", low_pass_filter_config, false);
  getParam<bool>(node, "accel_low_pass_filter_enable", accel_low_pass_filter_enable, false);
  getParam<bool>(node, "accel_low_pass_filter_auto", accel_low_pass_filter_auto, false);
  getParamFloat(node, "accel_low_pass_filter_frequency", accel_low_pass_filter_frequency, 0);
  getParam<bool>(node, "gyro_low_pass_filter_enable", gyro_low_pass_filter_enable, false);
  getParam<bool>(node, "gyro_low_pass_filter_auto", gyro_low_pass_filter_auto, false);
  getParamFloat(node, "gyro_low_pass_filter_frequency", gyro_low_pass_filter_frequency, 0);
  getParam<bool>(node, "mag_low_pass_filter_enable", mag_low_pass_filter_enable, false);
  getParam<bool>(node, "mag_low_pass_filter_auto", mag_low_pass_filter_auto, false);
  getParamFloat(node, "mag_low_pass_filter_frequency", mag_low_pass_filter_frequency, 0);
  getParam<bool>(node, "pressure_low_pass_filter_enable", pressure_low_pass_filter_enable, false);
  getParam<bool>(node, "pressure_low_pass_filter_auto", pressure_low_pass_filter_auto, false);
  getParamFloat(node, "pressure_low_pass_filter_frequency", pressure_low_pass_filter_frequency, 0);

  mip::CmdResult mip_cmd_result;
  const uint8_t descriptor_set = mip::commands_3dm::DESCRIPTOR_SET;

  // Configure all available pins
  if (mip_device_->supportsDescriptor(descriptor_set, mip::commands_3dm::CMD_GPIO_CONFIG))
  {
    if (gpio_config)
    {
      const int max_num_gpio = 4;
      for (int gpio_pin = 1; gpio_pin < max_num_gpio + 1; gpio_pin++)
      {
        int32_t gpio_feature;
        int32_t gpio_behavior;
        int32_t gpio_pin_mode;
        getParam<int32_t>(node, "gpio" + std::to_string(gpio_pin) + "_feature", gpio_feature, 0);
        getParam<int32_t>(node, "gpio" + std::to_string(gpio_pin) + "_behavior", gpio_behavior, 0);
        getParam<int32_t>(node, "gpio" + std::to_string(gpio_pin) + "_pin_mode", gpio_pin_mode, 0);

        MICROSTRAIN_INFO(node_, "Configuring GPIO%i to: feature = %i, behavior = %i, pinMode = %i", gpio_pin, gpio_feature, gpio_behavior, gpio_pin_mode);

        mip::commands_3dm::GpioConfig::PinMode gpio_pin_mode_bitfield;
        gpio_pin_mode_bitfield.value = gpio_pin_mode;
        if (!(mip_cmd_result = mip::commands_3dm::writeGpioConfig(*mip_device_, gpio_pin,
            static_cast<mip::commands_3dm::GpioConfig::Feature>(gpio_feature),
            static_cast<mip::commands_3dm::GpioConfig::Behavior>(gpio_behavior),
            gpio_pin_mode_bitfield)))
        {
          MICROSTRAIN_ERROR(node_, "Failed to configure GPIO%i", gpio_pin);
          MICROSTRAIN_ERROR(node_, "  Error(%d): %s", mip_cmd_result.value, mip_cmd_result.name());
          return false;
        }
      }
    }
    else
    {
      MICROSTRAIN_INFO(node_, "Note: Not configuring GPIO");
    }
  }
  else
  {
    MICROSTRAIN_INFO(node_, "Note: The device does not support the GPIO config command");
  }

  // Set PPS source
  if (mip_device_->supportsDescriptor(descriptor_set, mip::commands_3dm::CMD_PPS_SOURCE))
  {
    MICROSTRAIN_INFO(node_, "Setting PPS source to 0x%04x", filter_pps_source);
    if (!(mip_cmd_result = mip::commands_3dm::writePpsSource(*mip_device_, static_cast<mip::commands_3dm::PpsSource::Source>(filter_pps_source))))
    {
      MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to configure PPS source");
      return false;
    }
  }
  else
  {
    MICROSTRAIN_INFO(node_, "Note: The device does not support the PPS source command");
  }

  // Hardware odometer configuration
  if (mip_device_->supportsDescriptor(descriptor_set, mip::commands_3dm::CMD_ODOMETER_CONFIG))
  {
    const auto hardware_odometer_mode = enable_hardware_odometer_ ? mip::commands_3dm::Odometer::Mode::QUADRATURE : mip::commands_3dm::Odometer::Mode::DISABLED;
    MICROSTRAIN_INFO(node_, "Setting hardware odometer to: mode = %d, scaling = %f, uncertainty = %f", static_cast<int32_t>(hardware_odometer_mode), hardware_odometer_scaling, hardware_odometer_uncertainty);
    if (!(mip::commands_3dm::writeOdometer(*mip_device_, hardware_odometer_mode, hardware_odometer_scaling, hardware_odometer_uncertainty)))
    {
      MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to configure hardware odometer");
      return false;
    }
  }
  else
  {
    MICROSTRAIN_INFO(node_, "Note: The device does not support the odometer settings command");
  }

  // Support channel setup
  if (raw_file_enable_)
  {
    if (mip_device_->supportsDescriptor(descriptor_set, mip::commands_3dm::CMD_CONFIGURE_FACTORY_STREAMING))
    {
      if (raw_file_include_support_data_)
      {
        if (!(mip_cmd_result = mip::commands_3dm::factoryStreaming(*mip_device_, mip::commands_3dm::FactoryStreaming::Action::MERGE, 0)))
        {
          MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to configure factory streaming channels");
          return false;
        }

        // Also enable aiding command echo when collecting a factory support binary
        if (mip_device_->supportsDescriptor(mip::commands_aiding::DESCRIPTOR_SET, mip::commands_aiding::AidingEchoControl::FIELD_DESCRIPTOR))
        {
          if (!(mip_cmd_result = mip::commands_aiding::writeAidingEchoControl(*mip_device_, mip::commands_aiding::AidingEchoControl::Mode::RESPONSE)))
          {
            MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to configure aiding echo control");
            return false;
          }
        }
        else
        {
          MICROSTRAIN_DEBUG(node_, "Device does not support aiding echo control");
        }
      }
      else
      {
        MICROSTRAIN_INFO(node_, "Not configuring factory streaming channels");
      }
    }
    else
    {
      MICROSTRAIN_INFO(node_, "Note: The device does not support the factory streaming channels setup command");
      if (raw_file_include_support_data_)
      {
        MICROSTRAIN_ERROR(node_, "Could not configure support data even though it was requested. Exiting...");
        return false;
      }
    }
  }

  // SBAS settings
  if (mip_device_->supportsDescriptor(descriptor_set, mip::commands_3dm::CMD_GNSS_SBAS_SETTINGS))
  {
    std::stringstream prn_ss;
    prn_ss << "[ ";
    for (const auto sbas_prn : sbas_prns)
      prn_ss << sbas_prn << ", ";
    prn_ss << "]";
    MICROSTRAIN_INFO(node_, "Configuring SBAS with:");
    MICROSTRAIN_INFO(node_, "  enable = %d", sbas_enable);
    MICROSTRAIN_INFO(node_, "  enable ranging = %d", sbas_enable_ranging);
    MICROSTRAIN_INFO(node_, "  enable corrections = %d", sbas_enable_corrections);
    MICROSTRAIN_INFO(node_, "  apply integrity = %d", sbas_apply_integrity);
    MICROSTRAIN_INFO(node_, "  prns: %s", prn_ss.str().c_str());
    mip::commands_3dm::GnssSbasSettings::SBASOptions sbas_options;
    sbas_options.enableRanging(sbas_enable_ranging);
    sbas_options.enableCorrections(sbas_enable_corrections);
    sbas_options.applyIntegrity(sbas_apply_integrity);
    if (!(mip_cmd_result = mip::commands_3dm::writeGnssSbasSettings(*mip_device_, sbas_enable, sbas_options, sbas_prns.size(), sbas_prns.data())))
    {
      MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to configure SBAS settings");
      return false;
    }
  }
  else
  {
    MICROSTRAIN_INFO(node_, "Note: The device does not support the SBAS settings command");
  }

  // NMEA Message format
  if (mip_device_->supportsDescriptor(descriptor_set, mip::commands_3dm::CMD_NMEA_MESSAGE_FORMAT))
  {
    if (nmea_message_config)
    {
      /// Get the talker IDs for the descriptor sets that need them
      int32_t gnss1_nmea_talker_id, gnss2_nmea_talker_id, filter_nmea_talker_id;
      getParam<int32_t>(node, "gnss1_nmea_talker_id", gnss1_nmea_talker_id, 0);
      getParam<int32_t>(node, "gnss2_nmea_talker_id", gnss2_nmea_talker_id, 0);
      getParam<int32_t>(node, "filter_nmea_talker_id", filter_nmea_talker_id, 0);

      // Save the talker IDs to a map so we can look up the right frame IDs
      nmea_talker_id_to_frame_id_mapping_[MipMapping::nmeaFormatTalkerIdString(static_cast<mip::commands_3dm::NmeaMessage::TalkerID>(gnss1_nmea_talker_id))] = gnss_frame_id_[GNSS1_ID];
      nmea_talker_id_to_frame_id_mapping_[MipMapping::nmeaFormatTalkerIdString(static_cast<mip::commands_3dm::NmeaMessage::TalkerID>(gnss2_nmea_talker_id))] = gnss_frame_id_[GNSS2_ID];
      nmea_talker_id_to_frame_id_mapping_[MipMapping::nmeaFormatTalkerIdString(static_cast<mip::commands_3dm::NmeaMessage::TalkerID>(filter_nmea_talker_id))] = frame_id_;

      // Populate the NMEA message config options
      std::vector<mip::commands_3dm::NmeaMessage> formats;
      if (!populateNmeaMessageFormat(node, "imu_nmea_prkr_data_rate", mip::commands_3dm::NmeaMessage::TalkerID::IGNORED, mip::data_sensor::DESCRIPTOR_SET, mip::commands_3dm::NmeaMessage::MessageID::PKRR, &formats) ||
          !populateNmeaMessageFormat(node, "gnss1_nmea_gga_data_rate", static_cast<mip::commands_3dm::NmeaMessage::TalkerID>(gnss1_nmea_talker_id), mip::data_gnss::MIP_GNSS1_DATA_DESC_SET, mip::commands_3dm::NmeaMessage::MessageID::GGA, &formats) ||
          !populateNmeaMessageFormat(node, "gnss1_nmea_gll_data_rate", static_cast<mip::commands_3dm::NmeaMessage::TalkerID>(gnss1_nmea_talker_id), mip::data_gnss::MIP_GNSS1_DATA_DESC_SET, mip::commands_3dm::NmeaMessage::MessageID::GLL, &formats) ||
          !populateNmeaMessageFormat(node, "gnss1_nmea_gsv_data_rate", static_cast<mip::commands_3dm::NmeaMessage::TalkerID>(gnss1_nmea_talker_id), mip::data_gnss::MIP_GNSS1_DATA_DESC_SET, mip::commands_3dm::NmeaMessage::MessageID::GSV, &formats) ||
          !populateNmeaMessageFormat(node, "gnss1_nmea_rmc_data_rate", static_cast<mip::commands_3dm::NmeaMessage::TalkerID>(gnss1_nmea_talker_id), mip::data_gnss::MIP_GNSS1_DATA_DESC_SET, mip::commands_3dm::NmeaMessage::MessageID::RMC, &formats) ||
          !populateNmeaMessageFormat(node, "gnss1_nmea_vtg_data_rate", static_cast<mip::commands_3dm::NmeaMessage::TalkerID>(gnss1_nmea_talker_id), mip::data_gnss::MIP_GNSS1_DATA_DESC_SET, mip::commands_3dm::NmeaMessage::MessageID::VTG, &formats) ||
          !populateNmeaMessageFormat(node, "gnss1_nmea_hdt_data_rate", static_cast<mip::commands_3dm::NmeaMessage::TalkerID>(gnss1_nmea_talker_id), mip::data_gnss::MIP_GNSS1_DATA_DESC_SET, mip::commands_3dm::NmeaMessage::MessageID::HDT, &formats) ||
          !populateNmeaMessageFormat(node, "gnss1_nmea_zda_data_rate", static_cast<mip::commands_3dm::NmeaMessage::TalkerID>(gnss1_nmea_talker_id), mip::data_gnss::MIP_GNSS1_DATA_DESC_SET, mip::commands_3dm::NmeaMessage::MessageID::ZDA, &formats) ||
          !populateNmeaMessageFormat(node, "gnss2_nmea_gga_data_rate", static_cast<mip::commands_3dm::NmeaMessage::TalkerID>(gnss2_nmea_talker_id), mip::data_gnss::MIP_GNSS2_DATA_DESC_SET, mip::commands_3dm::NmeaMessage::MessageID::GGA, &formats) ||
          !populateNmeaMessageFormat(node, "gnss2_nmea_gll_data_rate", static_cast<mip::commands_3dm::NmeaMessage::TalkerID>(gnss2_nmea_talker_id), mip::data_gnss::MIP_GNSS2_DATA_DESC_SET, mip::commands_3dm::NmeaMessage::MessageID::GLL, &formats) ||
          !populateNmeaMessageFormat(node, "gnss2_nmea_gsv_data_rate", static_cast<mip::commands_3dm::NmeaMessage::TalkerID>(gnss2_nmea_talker_id), mip::data_gnss::MIP_GNSS2_DATA_DESC_SET, mip::commands_3dm::NmeaMessage::MessageID::GSV, &formats) ||
          !populateNmeaMessageFormat(node, "gnss2_nmea_rmc_data_rate", static_cast<mip::commands_3dm::NmeaMessage::TalkerID>(gnss2_nmea_talker_id), mip::data_gnss::MIP_GNSS2_DATA_DESC_SET, mip::commands_3dm::NmeaMessage::MessageID::RMC, &formats) ||
          !populateNmeaMessageFormat(node, "gnss2_nmea_vtg_data_rate", static_cast<mip::commands_3dm::NmeaMessage::TalkerID>(gnss2_nmea_talker_id), mip::data_gnss::MIP_GNSS2_DATA_DESC_SET, mip::commands_3dm::NmeaMessage::MessageID::VTG, &formats) ||
          !populateNmeaMessageFormat(node, "gnss2_nmea_hdt_data_rate", static_cast<mip::commands_3dm::NmeaMessage::TalkerID>(gnss2_nmea_talker_id), mip::data_gnss::MIP_GNSS2_DATA_DESC_SET, mip::commands_3dm::NmeaMessage::MessageID::HDT, &formats) ||
          !populateNmeaMessageFormat(node, "gnss2_nmea_zda_data_rate", static_cast<mip::commands_3dm::NmeaMessage::TalkerID>(gnss2_nmea_talker_id), mip::data_gnss::MIP_GNSS2_DATA_DESC_SET, mip::commands_3dm::NmeaMessage::MessageID::ZDA, &formats) ||
          !populateNmeaMessageFormat(node, "filter_nmea_gga_data_rate", static_cast<mip::commands_3dm::NmeaMessage::TalkerID>(filter_nmea_talker_id), mip::data_filter::DESCRIPTOR_SET, mip::commands_3dm::NmeaMessage::MessageID::GGA, &formats) ||
          !populateNmeaMessageFormat(node, "filter_nmea_gll_data_rate", static_cast<mip::commands_3dm::NmeaMessage::TalkerID>(filter_nmea_talker_id), mip::data_filter::DESCRIPTOR_SET, mip::commands_3dm::NmeaMessage::MessageID::GLL, &formats) ||
          !populateNmeaMessageFormat(node, "filter_nmea_rmc_data_rate", static_cast<mip::commands_3dm::NmeaMessage::TalkerID>(filter_nmea_talker_id), mip::data_filter::DESCRIPTOR_SET, mip::commands_3dm::NmeaMessage::MessageID::RMC, &formats) ||
          !populateNmeaMessageFormat(node, "filter_nmea_hdt_data_rate", static_cast<mip::commands_3dm::NmeaMessage::TalkerID>(filter_nmea_talker_id), mip::data_filter::DESCRIPTOR_SET, mip::commands_3dm::NmeaMessage::MessageID::HDT, &formats) ||
          !populateNmeaMessageFormat(node, "filter_nmea_prka_data_rate", static_cast<mip::commands_3dm::NmeaMessage::TalkerID>(filter_nmea_talker_id), mip::data_filter::DESCRIPTOR_SET, mip::commands_3dm::NmeaMessage::MessageID::PKRA, &formats))
        return false;

      // Send them to the device
      if (formats.size() <= 0)
        MICROSTRAIN_INFO(node_, "Disabling NMEA message streaming from main port");
      else
        MICROSTRAIN_INFO(node_, "Sending %lu NMEA message formats to device", formats.size());
      if (!(mip_cmd_result = mip::commands_3dm::writeNmeaMessageFormat(*mip_device_, formats.size(), formats.data())))
      {
        MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to configure NMEA message format");
        return false;
      }
    }
    else
    {
      MICROSTRAIN_INFO(node_, "Not configuring NMEA message format because 'nmea_message_config' is false");
    }
  }
  else
  {
    MICROSTRAIN_INFO(node_, "Note: The device does not support the nmea message format command");
  }

  // Low pass filter settings.
  std::vector<std::tuple<uint8_t, bool, bool, float>> low_pass_filter_settings =
  {
    {mip::data_sensor::DATA_ACCEL_SCALED, accel_low_pass_filter_enable, accel_low_pass_filter_auto, accel_low_pass_filter_frequency},
    {mip::data_sensor::DATA_GYRO_SCALED, gyro_low_pass_filter_enable, gyro_low_pass_filter_auto, gyro_low_pass_filter_frequency},
    {mip::data_sensor::DATA_MAG_SCALED, mag_low_pass_filter_enable, mag_low_pass_filter_auto, mag_low_pass_filter_frequency},
    {mip::data_sensor::DATA_PRESSURE_SCALED, pressure_low_pass_filter_enable, pressure_low_pass_filter_auto, pressure_low_pass_filter_frequency},
  };
  const bool supports_deprecated_low_pass_filter_settings = mip_device_->supportsDescriptor(mip::commands_3dm::ImuLowpassFilter::DESCRIPTOR_SET, mip::commands_3dm::ImuLowpassFilter::FIELD_DESCRIPTOR);
  const bool supports_low_pass_filter_settings = mip_device_->supportsDescriptor(mip::commands_3dm::LowpassFilter::DESCRIPTOR_SET, mip::commands_3dm::LowpassFilter::FIELD_DESCRIPTOR);
  if (supports_deprecated_low_pass_filter_settings || supports_low_pass_filter_settings)
  {
    if (low_pass_filter_config)
    {
      for (const auto& low_pass_filter_entry : low_pass_filter_settings)
      {
        const uint8_t low_pass_filter_field_descriptor = std::get<0>(low_pass_filter_entry);
        const bool low_pass_filter_enable = std::get<1>(low_pass_filter_entry);
        const bool low_pass_filter_auto = std::get<2>(low_pass_filter_entry);
        const float low_pass_filter_frequency = std::get<3>(low_pass_filter_entry);
        if (supports_low_pass_filter_settings)
        {
          MICROSTRAIN_INFO(node_, "Configuring low pass filter with:");
          MICROSTRAIN_INFO(node_, "  descriptor_set = 0x%02x", mip::data_sensor::DESCRIPTOR_SET);
          MICROSTRAIN_INFO(node_, "  field_descriptor = 0x%02x", low_pass_filter_field_descriptor);
          MICROSTRAIN_INFO(node_, "  enable = %d", low_pass_filter_enable);
          MICROSTRAIN_INFO(node_, "  manual = %d", !low_pass_filter_auto);
          MICROSTRAIN_INFO(node_, "  frequency = %f", low_pass_filter_frequency);
          if (!(mip_cmd_result = mip::commands_3dm::writeLowpassFilter(*mip_device_, mip::data_sensor::DESCRIPTOR_SET, low_pass_filter_field_descriptor, low_pass_filter_enable, !low_pass_filter_auto, low_pass_filter_frequency)))
          {
            MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to configure low pass filter settings");
            return false;
          }
        }
        else
        {
          MICROSTRAIN_INFO(node_, "Configuring low pass filter with:");
          MICROSTRAIN_INFO(node_, "  field_descriptor = 0x%02x", low_pass_filter_field_descriptor);
          MICROSTRAIN_INFO(node_, "  enable = %d", low_pass_filter_enable);
          MICROSTRAIN_INFO(node_, "  manual = %d", !low_pass_filter_auto);
          MICROSTRAIN_INFO(node_, "  frequency = %u", static_cast<uint16_t>(std::round(low_pass_filter_frequency)));
          if (!(mip_cmd_result = mip::commands_3dm::writeImuLowpassFilter(*mip_device_, low_pass_filter_field_descriptor, low_pass_filter_enable, !low_pass_filter_auto, static_cast<uint16_t>(std::round(low_pass_filter_frequency)), 0)))
          {
            MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to configure low pass filter settings");
            return false;
          }
        }
      }
    }
    else
    {
      MICROSTRAIN_INFO(node_, "Not configuring low pass filter settings because 'low_pass_filter_config' is false");
    }
  }
  else
  {
    MICROSTRAIN_INFO(node_, "Note: The device does not support the low pass filter settings command");
  }

  return true;
}

bool Config::configureGNSS(RosNodeType* node)
{
  bool gnss_glonass_enable_bool;
  bool gnss_galileo_enable_bool;
  bool gnss_beidou_enable_bool;
  getParam<bool>(node, "gnss_glonass_enable", gnss_glonass_enable_bool, true);
  getParam<bool>(node, "gnss_galileo_enable", gnss_galileo_enable_bool, true);
  getParam<bool>(node, "gnss_beidou_enable", gnss_beidou_enable_bool, true);

  mip::CmdResult mip_cmd_result;
  const uint8_t descriptor_set = mip::commands_gnss::DESCRIPTOR_SET;

  // RTK configuration
  if (mip_device_->supportsDescriptor(descriptor_set, mip::commands_gnss::CMD_RTK_DONGLE_CONFIGURATION))
  {
    uint8_t reserved[3];
    MICROSTRAIN_INFO(node_, "Setting RTK dongle enable to %d", rtk_dongle_enable_);
    if (!(mip_cmd_result = mip::commands_gnss::writeRtkDongleConfiguration(*mip_device_, rtk_dongle_enable_, reserved)))
    {
      MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to write RTK dongle configuration");
      return false;
    }
  }
  else
  {
    MICROSTRAIN_INFO(node_, "Note: Device does not support the RTK dongle config command");
  }

  // GNSS Signal confiuration
  if (mip_device_->supportsDescriptor(descriptor_set, mip::commands_gnss::CMD_SIGNAL_CONFIGURATION))
  {
    uint8_t gnss_gps_enable = 3;
    uint8_t gnss_glonass_enable = gnss_glonass_enable_bool ? 3 : 0;
    uint8_t gnss_galileo_enable = gnss_galileo_enable_bool ? 3 : 0;
    uint8_t gnss_beidou_enable = gnss_beidou_enable_bool ? 3 : 0;
    uint8_t reserved[4];
    MICROSTRAIN_INFO(node_, "Setting GNSS Signal Configuration to:");
    MICROSTRAIN_INFO(node_, "  glonass_enable = %d", gnss_glonass_enable);
    MICROSTRAIN_INFO(node_, "  galileo_enable = %d", gnss_galileo_enable);
    MICROSTRAIN_INFO(node_, "  beidou_enable = %d", gnss_beidou_enable);
    if (!(mip_cmd_result = mip::commands_gnss::writeSignalConfiguration(*mip_device_, gnss_gps_enable, gnss_glonass_enable, gnss_galileo_enable, gnss_beidou_enable, reserved)))
    {
      MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to write GNSS Signal configuration");
      return false;
    }
  }
  else
  {
    MICROSTRAIN_INFO(node_, "Note: Device does not support Siangle Config command");
  }

  return true;
}

bool Config::configureFilter(RosNodeType* node)
{
  // Read some generic filter info
  int heading_source;
  float initial_heading;
  bool filter_auto_init = true;
  int dynamics_mode;
  int32_t declination_source;
  double declination;
  int32_t gnss_aiding_source_control;
  getParam<int32_t>(node, "filter_declination_source", declination_source, 2);
  getParam<double>(node, "filter_declination", declination, 0.23);
  getParam<int32_t>(node, "filter_heading_source", heading_source, 0x1);
  getParam<float>(node, "filter_initial_heading", initial_heading, 0.0);
  getParam<bool>(node, "filter_auto_init", filter_auto_init, true);
  getParam<int32_t>(node, "filter_dynamics_mode", dynamics_mode, 1);
  getParam<int32_t>(node, "filter_gnss_aiding_source_control", gnss_aiding_source_control, 1);

  // Read some QG7 specific filter options
  int filter_adaptive_level;
  int filter_adaptive_time_limit_ms;
  int filter_init_condition_src;
  int filter_auto_heading_alignment_selector;
  int filter_init_reference_frame;
  std::vector<double> filter_init_position_double(3, 0.0);
  std::vector<double> filter_init_velocity_double(3, 0.0);
  std::vector<double> filter_init_attitude_double(3, 0.0);
  double filter_gnss_antenna_cal_max_offset;
  std::vector<double> filter_lever_arm_offset_double(3, 0.0);
  getParam<int32_t>(node, "filter_adaptive_level", filter_adaptive_level, 2);
  getParam<int32_t>(node, "filter_adaptive_time_limit_ms", filter_adaptive_time_limit_ms, 15000);
  getParam<int32_t>(node, "filter_init_condition_src", filter_init_condition_src, 0);
  getParam<int32_t>(node, "filter_auto_heading_alignment_selector", filter_auto_heading_alignment_selector, 0);
  getParam<int32_t>(node, "filter_init_reference_frame", filter_init_reference_frame, 2);
  getParam<std::vector<double>>(node, "filter_init_position", filter_init_position_double, DEFAULT_VECTOR);
  getParam<std::vector<double>>(node, "filter_init_velocity", filter_init_velocity_double, DEFAULT_VECTOR);
  getParam<std::vector<double>>(node, "filter_init_attitude", filter_init_attitude_double, DEFAULT_VECTOR);
  getParam<double>(node, "filter_gnss_antenna_cal_max_offset", filter_gnss_antenna_cal_max_offset, 0.1);
  getParam<std::vector<double>>(node, "filter_lever_arm_offset", filter_lever_arm_offset_double, DEFAULT_VECTOR);

  // Sensor2vehicle config
  int filter_sensor2vehicle_frame_selector;
  std::vector<double> filter_sensor2vehicle_frame_transformation_euler_double(3, 0.0);
  std::vector<double> filter_sensor2vehicle_frame_transformation_matrix_double(9, 0.0);
  std::vector<double> filter_sensor2vehicle_frame_transformation_quaternion_double(4, 0.0);
  getParam<int32_t>(node, "filter_sensor2vehicle_frame_selector", filter_sensor2vehicle_frame_selector, 0);
  getParam<std::vector<double>>(node, "filter_sensor2vehicle_frame_transformation_euler", filter_sensor2vehicle_frame_transformation_euler_double, DEFAULT_VECTOR);
  getParam<std::vector<double>>(node, "filter_sensor2vehicle_frame_transformation_matrix", filter_sensor2vehicle_frame_transformation_matrix_double, DEFAULT_VECTOR);
  getParam<std::vector<double>>(node, "filter_sensor2vehicle_frame_transformation_quaternion", filter_sensor2vehicle_frame_transformation_quaternion_double, DEFAULT_VECTOR);

  // ROS2 can only fetch double vectors from config, so convert the doubles to floats for the MIP SDK
  std::vector<float> filter_init_position(filter_init_position_double.begin(), filter_init_position_double.end());
  std::vector<float> filter_init_velocity(filter_init_velocity_double.begin(), filter_init_velocity_double.end());
  std::vector<float> filter_init_attitude(filter_init_attitude_double.begin(), filter_init_attitude_double.end());

  std::vector<float> filter_sensor2vehicle_frame_transformation_euler(filter_sensor2vehicle_frame_transformation_euler_double.begin(), filter_sensor2vehicle_frame_transformation_euler_double.end());
  std::vector<float> filter_sensor2vehicle_frame_transformation_matrix(filter_sensor2vehicle_frame_transformation_matrix_double.begin(), filter_sensor2vehicle_frame_transformation_matrix_double.end());
  std::vector<float> filter_sensor2vehicle_frame_transformation_quaternion(filter_sensor2vehicle_frame_transformation_quaternion_double.begin(), filter_sensor2vehicle_frame_transformation_quaternion_double.end());

  std::vector<float> filter_lever_arm_offset(filter_lever_arm_offset_double.begin(), filter_lever_arm_offset_double.end());

  mip::CmdResult mip_cmd_result;
  const uint8_t descriptor_set = mip::commands_filter::DESCRIPTOR_SET;

  // Set Declination Source
  if (mip_device_->supportsDescriptor(descriptor_set, mip::commands_filter::CMD_DECLINATION_SOURCE))
  {
    // If the declination source is none, set the declination to 0
    const auto declination_source_enum = static_cast<mip::commands_filter::FilterMagParamSource>(declination_source);
    if (declination_source_enum == mip::commands_filter::FilterMagParamSource::NONE)
      declination = 0;

    MICROSTRAIN_INFO(node_, "Setting Declination Source to %d %f", declination_source, declination);
    if (!(mip_cmd_result = mip::commands_filter::writeMagneticDeclinationSource(*mip_device_, declination_source_enum, declination)))
    {
      MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to set declination source");
      return false;
    }
  }
  else
  {
    MICROSTRAIN_INFO(node_, "Note: Device does not support the declination source command.");
  }

  // If either antenna offset is configured with the transform selector, lookup the transform in the tf tree
  for (int i = 0; i < NUM_GNSS; i++)
  {
    if (gnss_antenna_offset_source_[i] == OFFSET_SOURCE_TRANSFORM)
    {
      // Override the antenna offset with the result from the transform tree
      const tf2::Transform& gnss_antenna_to_microstrain_vehicle_transform_tf = lookupLeverArmOffsetInMicrostrainVehicleFrame(gnss_frame_id_[i]);
      gnss_antenna_offset_[i][0] = gnss_antenna_to_microstrain_vehicle_transform_tf.getOrigin().x();
      gnss_antenna_offset_[i][1] = gnss_antenna_to_microstrain_vehicle_transform_tf.getOrigin().y();
      gnss_antenna_offset_[i][2] = gnss_antenna_to_microstrain_vehicle_transform_tf.getOrigin().z();
    }
  }

  // GNSS 1/2 antenna offsets
  if (mip_device_->supportsDescriptor(descriptor_set, mip::commands_filter::CMD_ANTENNA_OFFSET))
  {
    if (gnss_antenna_offset_source_[GNSS1_ID] != OFFSET_SOURCE_OFF)
    {
      MICROSTRAIN_INFO(node_, "Setting single antenna offset to [%f, %f, %f]",
          gnss_antenna_offset_[GNSS1_ID][0], gnss_antenna_offset_[GNSS1_ID][1], gnss_antenna_offset_[GNSS1_ID][2]);
      if (!(mip_cmd_result = mip::commands_filter::writeAntennaOffset(*mip_device_, gnss_antenna_offset_[GNSS1_ID].data())))
      {
        MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Cound not set single antenna offset");
        return false;
      }
    }
    else
    {
      MICROSTRAIN_INFO(node_, "Not configuring single antenna offset because gnss1_antenna_offset_source is %d", OFFSET_SOURCE_OFF);
    }
  }
  else if (mip_device_->supportsDescriptor(descriptor_set, mip::commands_filter::CMD_MULTI_ANTENNA_OFFSET))
  {
    if (gnss_antenna_offset_source_[GNSS1_ID] != OFFSET_SOURCE_OFF)
    {
      MICROSTRAIN_INFO(node_, "Setting GNSS1 antenna offset to [%f, %f, %f]",
          gnss_antenna_offset_[GNSS1_ID][0], gnss_antenna_offset_[GNSS1_ID][1], gnss_antenna_offset_[GNSS1_ID][2]);
      if (!(mip_cmd_result = mip::commands_filter::writeMultiAntennaOffset(*mip_device_, GNSS1_ID + 1, gnss_antenna_offset_[GNSS1_ID].data())))
      {
        MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Could not set multi antenna offset for GNSS1");
        return false;
      }
    }
    else
    {
      MICROSTRAIN_INFO(node_, "Not configuring GNSS1 antenna offset because gnss1_antenna_offset_source is %d", OFFSET_SOURCE_OFF);
    }

    if (gnss_antenna_offset_source_[GNSS2_ID] != OFFSET_SOURCE_OFF)
    {
      MICROSTRAIN_INFO(node_, "Setting GNSS2 antenna offset to [%f, %f, %f]",
          gnss_antenna_offset_[GNSS2_ID][0], gnss_antenna_offset_[GNSS2_ID][1], gnss_antenna_offset_[GNSS2_ID][2]);
      if (!(mip_cmd_result = mip::commands_filter::writeMultiAntennaOffset(*mip_device_, GNSS2_ID + 1, gnss_antenna_offset_[GNSS2_ID].data())))
      {
        MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Could not set multi antenna offset for GNSS2");
        return false;
      }
    }
    else
    {
      MICROSTRAIN_INFO(node_, "Not configuring GNSS2 antenna offset because gnss2_antenna_offset_source is %d", OFFSET_SOURCE_OFF);
    }
  }
  else
  {
    MICROSTRAIN_INFO(node_, "Note: Device does not support GNSS antenna offsets");
  }

  // Set dynamics mode
  if (mip_device_->supportsDescriptor(descriptor_set, mip::commands_filter::CMD_VEHICLE_DYNAMICS_MODE))
  {
    MICROSTRAIN_INFO(node_, "Setting vehicle dynamics mode to 0x%02x", dynamics_mode);
    if (!(mip_cmd_result = mip::commands_filter::writeVehicleDynamicsMode(*mip_device_, static_cast<mip::commands_filter::VehicleDynamicsMode::DynamicsMode>(dynamics_mode))))
    {
      MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Could not set vehicle dynamics mode");
      return false;
    }
  }
  else
  {
    MICROSTRAIN_INFO(node_, "Note: The device does not support the vehicle dynamics mode command.");
  }

  // Set GNSS aiding source control
  if (mip_device_->supportsDescriptor(descriptor_set, mip::commands_filter::CMD_GNSS_SOURCE_CONTROL))
  {
    MICROSTRAIN_INFO(node_, "Setting GNSS aiding source control to %d", gnss_aiding_source_control);
    if (!(mip_cmd_result = mip::commands_filter::writeGnssSource(*mip_device_, static_cast<mip::commands_filter::GnssSource::Source>(gnss_aiding_source_control))))
    {
      MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Could not set GNSS aiding source control");
      return false;
    }
  }

  // Set heading Source
  if (mip_device_->supportsDescriptor(descriptor_set, mip::commands_filter::CMD_HEADING_UPDATE_CONTROL))
  {
    MICROSTRAIN_INFO(node_, "Setting heading source to %d", heading_source);
    const auto heading_source_enum = static_cast<mip::commands_filter::HeadingSource::Source>(heading_source);
    if (!configureHeadingSource(heading_source_enum))
      return false;

    if (mip_device_->supportsDescriptor(descriptor_set, mip::commands_filter::CMD_SET_INITIAL_HEADING))
    {
      if (heading_source_enum == mip::commands_filter::HeadingSource::Source::NONE)
      {
        MICROSTRAIN_INFO(node_, "Setting initial heading to %f", initial_heading);
        if (!(mip_cmd_result = mip::commands_filter::setInitialHeading(*mip_device_, initial_heading)))
        {
          MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Could not set initial heading");
          return false;
        }
      }
      else
      {
        MICROSTRAIN_INFO(node_, "Note: Not setting initial heading because heading source is not 0");
      }
    }
    else
    {
      MICROSTRAIN_INFO(node_, "Note: Device does not support the set initial heading command");
    }
  }
  else
  {
    MICROSTRAIN_INFO(node_, "Note: The device does not support the heading source command.");
  }

  // Set the filter autoinitialization, if suppored
  if (mip_device_->supportsDescriptor(descriptor_set, mip::commands_filter::CMD_AUTOINIT_CONTROL))
  {
    MICROSTRAIN_INFO(node_, "Setting autoinitialization to %d", filter_auto_init);
    if (!(mip::commands_filter::writeAutoInitControl(*mip_device_, filter_auto_init)))
    {
      MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to configure filter auto initialization");
      return false;
    }
  }
  else
  {
    MICROSTRAIN_INFO(node_, "Note: The device does not support the filter autoinitialization command.");
  }

  // Set the filter adaptive settings
  if (mip_device_->supportsDescriptor(descriptor_set, mip::commands_filter::CMD_ADAPTIVE_FILTER_OPTIONS))
  {
    MICROSTRAIN_INFO(node_, "Setting autoadaptive options to: level = %d, time_limit = %d", filter_adaptive_level, filter_adaptive_time_limit_ms);
    if (!(mip_cmd_result = mip::commands_filter::writeAdaptiveFilterOptions(*mip_device_, filter_adaptive_level, filter_adaptive_time_limit_ms)))
    {
      MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to configure auto adaptive filter settings");
      return false;
    }
  }
  else
  {
    MICROSTRAIN_INFO(node_, "Note: The device does not support the filter adaptive settings command.");
  }

  // Set the filter aiding settings
  if (mip_device_->supportsDescriptor(descriptor_set, mip::commands_filter::CMD_AIDING_MEASUREMENT_ENABLE))
  {
    if (!configureFilterAidingMeasurement(mip::commands_filter::AidingMeasurementEnable::AidingSource::GNSS_POS_VEL, filter_enable_gnss_pos_vel_aiding_) ||
        !configureFilterAidingMeasurement(mip::commands_filter::AidingMeasurementEnable::AidingSource::GNSS_HEADING, filter_enable_gnss_heading_aiding_) ||
        !configureFilterAidingMeasurement(mip::commands_filter::AidingMeasurementEnable::AidingSource::ALTIMETER, filter_enable_altimeter_aiding_) ||
        !configureFilterAidingMeasurement(mip::commands_filter::AidingMeasurementEnable::AidingSource::SPEED, filter_enable_odometer_aiding_) ||
        !configureFilterAidingMeasurement(mip::commands_filter::AidingMeasurementEnable::AidingSource::MAGNETOMETER, filter_enable_magnetometer_aiding_) ||
        !configureFilterAidingMeasurement(mip::commands_filter::AidingMeasurementEnable::AidingSource::EXTERNAL_HEADING, filter_enable_external_heading_aiding_))
      return false;
  }
  else
  {
    MICROSTRAIN_INFO(node_, "Note: The device does not support the filter aiding command.");
  }

  // Set the filter speed lever arm
  if (mip_device_->supportsDescriptor(descriptor_set, mip::commands_filter::CMD_SPEED_LEVER_ARM))
  {
    if (filter_speed_lever_arm_source_ == OFFSET_SOURCE_TRANSFORM)
    {
      const auto& odometer_to_microstrain_vehicle_transform_tf = lookupLeverArmOffsetInMicrostrainVehicleFrame(odometer_frame_id_);
      filter_speed_lever_arm_[0] = odometer_to_microstrain_vehicle_transform_tf.getOrigin().x();
      filter_speed_lever_arm_[1] = odometer_to_microstrain_vehicle_transform_tf.getOrigin().y();
      filter_speed_lever_arm_[2] = odometer_to_microstrain_vehicle_transform_tf.getOrigin().z();
    }
    if (filter_speed_lever_arm_source_ != OFFSET_SOURCE_OFF)
    {
      MICROSTRAIN_INFO(node_, "Setting speed lever arm to: [%f, %f, %f]", filter_speed_lever_arm_[0], filter_speed_lever_arm_[1], filter_speed_lever_arm_[2]);
      if (!(mip_cmd_result = mip::commands_filter::writeSpeedLeverArm(*mip_device_, 1, filter_speed_lever_arm_.data())))
      {
        MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to configure speed lever arm");
        return false;
      }
    }
  }
  else
  {
    MICROSTRAIN_INFO(node_, "Note: The device does not support the filter speed lever arm command.");
  }

  // Set the wheeled vehicle constraint
  if (mip_device_->supportsDescriptor(descriptor_set, mip::commands_filter::CMD_VEHICLE_CONSTRAINT_CONTROL))
  {
    MICROSTRAIN_INFO(node_, "Setting wheeled vehicle contraint enable to %d", filter_enable_wheeled_vehicle_constraint_);
    if (!(mip_cmd_result = mip::commands_filter::writeWheeledVehicleConstraintControl(*mip_device_, filter_enable_wheeled_vehicle_constraint_)))
    {
      MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to configure wheeled vehicle constraint");
      return false;
    }
  }
  else
  {
    MICROSTRAIN_INFO(node_, "Note: The device does not support the wheeled vehicle constraint command.");
  }

  // Set the vertical gyro constraint
  if (mip_device_->supportsDescriptor(descriptor_set, mip::commands_filter::CMD_GYRO_CONSTRAINT_CONTROL))
  {
    MICROSTRAIN_INFO(node_, "Setting vertical gyro contraint enable to %d", filter_enable_vertical_gyro_constraint_);
    if (!(mip::commands_filter::writeVerticalGyroConstraintControl(*mip_device_, filter_enable_vertical_gyro_constraint_)))
    {
      MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to configure vertical gyro constraint");
      return false;
    }
  }
  else
  {
    MICROSTRAIN_INFO(node_, "Note: The device does not support the vertical gyro constraint command.");
  }

  // Set the GNSS antenna calibration settings
  if (mip_device_->supportsDescriptor(descriptor_set, mip::commands_filter::CMD_ANTENNA_CALIBRATION_CONTROL))
  {
    MICROSTRAIN_INFO(node_, "Setting GNSS antenna calibration control to: enable = %d, offset = %f", filter_enable_gnss_antenna_cal_, filter_gnss_antenna_cal_max_offset);
    if (!(mip_cmd_result = mip::commands_filter::writeGnssAntennaCalControl(*mip_device_, filter_enable_gnss_antenna_cal_, filter_gnss_antenna_cal_max_offset)))
    {
      MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to configure antenna calibration");
      return false;
    }
  }
  else
  {
    MICROSTRAIN_INFO(node_, "Note: The device does not support the GNSS antenna calibration command.");
  }

  // Set the filter initialization settings
  if (mip_device_->supportsDescriptor(descriptor_set, mip::commands_filter::CMD_INITIALIZATION_CONFIGURATION))
  {
    MICROSTRAIN_INFO(node_, "Setting filter initialization configuration to:");
    MICROSTRAIN_INFO(node_, "  auto init = %d", filter_auto_init);
    MICROSTRAIN_INFO(node_, "  initial condition source = %d", filter_init_condition_src);
    MICROSTRAIN_INFO(node_, "  auto heading alignment selector = %d", filter_auto_heading_alignment_selector);
    MICROSTRAIN_INFO(node_, "  initial attitude = [%f, %f, %f]", filter_init_attitude[0], filter_init_attitude[1], filter_init_attitude[2]);
    MICROSTRAIN_INFO(node_, "  initial position = [%f, %f, %f]", filter_init_position[0], filter_init_position[1], filter_init_position[2]);
    MICROSTRAIN_INFO(node_, "  initial velocity = [%f, %f, %f]", filter_init_velocity[0], filter_init_velocity[1], filter_init_velocity[2]);
    MICROSTRAIN_INFO(node_, "  reference frame selector = %d", filter_init_reference_frame);
    if (!(mip_cmd_result = mip::commands_filter::writeInitializationConfiguration(*mip_device_, !filter_auto_init,
        static_cast<mip::commands_filter::InitializationConfiguration::InitialConditionSource>(filter_init_condition_src),
        static_cast<mip::commands_filter::InitializationConfiguration::AlignmentSelector>(filter_auto_heading_alignment_selector),
        filter_init_attitude[2], filter_init_attitude[1], filter_init_attitude[0],
        filter_init_position.data(), filter_init_velocity.data(),
        static_cast<mip::commands_filter::FilterReferenceFrame>(filter_init_reference_frame))))
    {
      MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to configure filter initialization");
      return false;
    }
  }
  else
  {
    MICROSTRAIN_INFO(node_, "Note: The device does not support the next-gen filter initialization command.");
  }

  // Sensor to vehicle configuration
  // This includes some 3DM commands because some devices do this through the 3DM descriptor set, and some do it through the filter descriptor set
  if (filter_sensor2vehicle_frame_selector == 0)
  {
    MICROSTRAIN_INFO(node_, "Note: Not configuring sensor2vehicle transformation or rotation");
  }
  else if (filter_sensor2vehicle_frame_selector == 1)
  {
    if (mip_device_->supportsDescriptor(descriptor_set, mip::commands_filter::CMD_SENSOR2VEHICLE_ROTATION_EULER))
    {
      MICROSTRAIN_INFO(node_, "Setting sensor to vehicle rotation euler to [%f, %f, %f]", -filter_sensor2vehicle_frame_transformation_euler[0],
          -filter_sensor2vehicle_frame_transformation_euler[1], -filter_sensor2vehicle_frame_transformation_euler[2]);
      if (!(mip_cmd_result = mip::commands_filter::writeSensorToVehicleRotationEuler(*mip_device_, -filter_sensor2vehicle_frame_transformation_euler[0],
          -filter_sensor2vehicle_frame_transformation_euler[1], -filter_sensor2vehicle_frame_transformation_euler[2])))
      {
        MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to configure sensor to vehicle rotation euler");
        return false;
      }
    }
    else if (mip_device_->supportsDescriptor(mip::commands_3dm::DESCRIPTOR_SET, mip::commands_3dm::CMD_SENSOR2VEHICLE_TRANSFORM_EUL))
    {
      MICROSTRAIN_INFO(node_, "Setting sensor to vehicle transformation euler to [%f, %f, %f]", filter_sensor2vehicle_frame_transformation_euler[0],
          filter_sensor2vehicle_frame_transformation_euler[1], filter_sensor2vehicle_frame_transformation_euler[2]);
      if (!(mip_cmd_result = mip::commands_3dm::writeSensor2VehicleTransformEuler(*mip_device_, filter_sensor2vehicle_frame_transformation_euler[0],
          filter_sensor2vehicle_frame_transformation_euler[1], filter_sensor2vehicle_frame_transformation_euler[2])))
      {
        MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to configure sensor to vehicle transformation euler");
        return false;
      }
    }
    else
    {
      MICROSTRAIN_WARN(node_, "Note: The device does not support the sensor to vehicle transformation or rotation euler command");
    }
  }
  else if (filter_sensor2vehicle_frame_selector == 2)
  {
    if (mip_device_->supportsDescriptor(descriptor_set, mip::commands_filter::CMD_SENSOR2VEHICLE_ROTATION_DCM))
    {
      // Transpose the matrix for the rotation
      float dcm[9]
      {
          filter_sensor2vehicle_frame_transformation_matrix[0], filter_sensor2vehicle_frame_transformation_matrix[3], filter_sensor2vehicle_frame_transformation_matrix[6],
          filter_sensor2vehicle_frame_transformation_matrix[1], filter_sensor2vehicle_frame_transformation_matrix[4], filter_sensor2vehicle_frame_transformation_matrix[7],
          filter_sensor2vehicle_frame_transformation_matrix[2], filter_sensor2vehicle_frame_transformation_matrix[5], filter_sensor2vehicle_frame_transformation_matrix[8]
      };
      MICROSTRAIN_INFO(node_, "Setting sensor to vehicle rotation matrix to [ [%f, %f, %f], [%f, %f, %f], [%f, %f, %f] ]", dcm[0], dcm[1], dcm[2], dcm[3], dcm[4], dcm[5], dcm[6], dcm[7], dcm[8]);
      if (!(mip_cmd_result = mip::commands_filter::writeSensorToVehicleRotationDcm(*mip_device_, dcm)))
      {
        MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to configure sensor to vehicle rotation matrix");
        return false;
      }
    }
    else if (mip_device_->supportsDescriptor(mip::commands_3dm::DESCRIPTOR_SET, mip::commands_3dm::CMD_SENSOR2VEHICLE_TRANSFORM_DCM))
    {
      float dcm[9]
      {
        filter_sensor2vehicle_frame_transformation_matrix[0], filter_sensor2vehicle_frame_transformation_matrix[1], filter_sensor2vehicle_frame_transformation_matrix[2],
        filter_sensor2vehicle_frame_transformation_matrix[3], filter_sensor2vehicle_frame_transformation_matrix[4], filter_sensor2vehicle_frame_transformation_matrix[5],
        filter_sensor2vehicle_frame_transformation_matrix[6], filter_sensor2vehicle_frame_transformation_matrix[7], filter_sensor2vehicle_frame_transformation_matrix[8]
      };
      MICROSTRAIN_INFO(node_, "Setting sensor to vehicle rotation matrix to [ [%f, %f, %f], [%f, %f, %f], [%f, %f, %f] ]", dcm[0], dcm[1], dcm[2], dcm[3], dcm[4], dcm[5], dcm[6], dcm[7], dcm[8]);
      if (!(mip_cmd_result = mip::commands_3dm::writeSensor2VehicleTransformDcm(*mip_device_, dcm)))
      {
        MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to configure sensor to vehicle transformation matrix");
        return false;
      }
    }
    else
    {
      MICROSTRAIN_WARN(node_, "Note: The device does not support the sensor to vehicle transformation or rotation matrix command");
    }
  }
  else if (filter_sensor2vehicle_frame_selector == 3)
  {
    if (mip_device_->supportsDescriptor(descriptor_set, mip::commands_filter::CMD_SENSOR2VEHICLE_ROTATION_QUATERNION))
    {
      float quaternion[4]
      {
        filter_sensor2vehicle_frame_transformation_quaternion[3],
        -filter_sensor2vehicle_frame_transformation_quaternion[0],
        -filter_sensor2vehicle_frame_transformation_quaternion[1],
        -filter_sensor2vehicle_frame_transformation_quaternion[2]
      };
      MICROSTRAIN_INFO(node_, "Setting sensor to vehicle rotation quaternion to [%f, %f, %f, %f]", quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
      if (!(mip_cmd_result = mip::commands_filter::writeSensorToVehicleRotationQuaternion(*mip_device_, quaternion)))
      {
        MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to configure sensor to vehicle rotation quaternion");
        return false;
      }
    }
    else if (mip_device_->supportsDescriptor(mip::commands_3dm::DESCRIPTOR_SET, mip::commands_3dm::CMD_SENSOR2VEHICLE_TRANSFORM_EUL))
    {
      float quaternion[4]
      {
        filter_sensor2vehicle_frame_transformation_quaternion[3],
        filter_sensor2vehicle_frame_transformation_quaternion[0],
        filter_sensor2vehicle_frame_transformation_quaternion[1],
        filter_sensor2vehicle_frame_transformation_quaternion[2]
      };
      MICROSTRAIN_INFO(node_, "Setting sensor to vehicle transformation quaternion to [%f, %f, %f, %f]", quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
      if (!(mip_cmd_result = mip::commands_3dm::writeSensor2VehicleTransformQuaternion(*mip_device_, quaternion)))
      {
        MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to configure sensor to vehicle transformation quaternion");
        return false;
      }
    }
    else
    {
      MICROSTRAIN_WARN(node_, "Note: The device does not support the sensor to vehicle transformation or rotation quaternion command");
    }
  }
  else
  {
    MICROSTRAIN_ERROR(node_, "Unsupported sensor 2 vechicle frame selector: %d", filter_sensor2vehicle_frame_selector);
    return false;
  }

  // Filter lever arm offset configuration
  if (mip_device_->supportsDescriptor(descriptor_set, mip::commands_filter::CMD_REF_POINT_LEVER_ARM))
  {
    MICROSTRAIN_INFO(node_, "Setting filter reference point lever arm to [%f, %f, %f]", filter_lever_arm_offset[0], filter_lever_arm_offset[1], filter_lever_arm_offset[2]);
    if (!(mip_cmd_result = mip::commands_filter::writeRefPointLeverArm(*mip_device_, mip::commands_filter::RefPointLeverArm::ReferencePointSelector::VEH, filter_lever_arm_offset.data())))
    {
      MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to configure refernce point lever arm");
      return false;
    }
  }
  else
  {
    MICROSTRAIN_INFO(node_, "Note: The device does not support the reference point lever arm command");
  }

  return true;
}

bool Config::configureFilterAidingMeasurement(const mip::commands_filter::AidingMeasurementEnable::AidingSource aiding_source, const bool enable)
{
  // Find the name of the aiding measurement so we can log some info about it
  std::string aiding_measurement_name;
  switch (aiding_source)
  {
    case mip::commands_filter::AidingMeasurementEnable::AidingSource::GNSS_POS_VEL:
      aiding_measurement_name = "gnss pos/vel";
      break;
    case mip::commands_filter::AidingMeasurementEnable::AidingSource::GNSS_HEADING:
      aiding_measurement_name = "gnss heading";
      break;
    case mip::commands_filter::AidingMeasurementEnable::AidingSource::ALTIMETER:
      aiding_measurement_name = "altimeter";
      break;
    case mip::commands_filter::AidingMeasurementEnable::AidingSource::SPEED:
      aiding_measurement_name = "odometer";
      break;
    case mip::commands_filter::AidingMeasurementEnable::AidingSource::MAGNETOMETER:
      aiding_measurement_name = "magnetometer";
      break;
    case mip::commands_filter::AidingMeasurementEnable::AidingSource::EXTERNAL_HEADING:
      aiding_measurement_name = "external heading";
      break;
    default:
      aiding_measurement_name = std::to_string(static_cast<uint8_t>(aiding_source));
      break;
  }

  const mip::CmdResult mip_cmd_result = mip::commands_filter::writeAidingMeasurementEnable(*mip_device_, aiding_source, enable);
  if (mip_cmd_result == mip::CmdResult::NACK_INVALID_PARAM)
  {
    if (enable)
      MICROSTRAIN_WARN(node_, "Note: Filter aiding %s not supported, but it was requested. Disable in params file to remove this warning", aiding_measurement_name.c_str());
    else
      MICROSTRAIN_INFO(node_, "Note: Filter aiding %s not supported", aiding_measurement_name.c_str());
  }
  else if (!mip_cmd_result)
  {
    MICROSTRAIN_ERROR(node_, "Failed to set %s aiding measurement", aiding_measurement_name.c_str());
    MICROSTRAIN_ERROR(node_, "  Error(%d): %s", mip_cmd_result.value, mip_cmd_result.name());
    return false;
  }
  else
  {
    MICROSTRAIN_INFO(node_, "Filter aiding %s = %d", aiding_measurement_name.c_str(), enable);
  }
  return true;
}

bool Config::configureGnssSourceControl(const mip::commands_filter::GnssSource::Source gnss_source)
{
  const mip::CmdResult mip_cmd_result = mip::commands_filter::writeGnssSource(*mip_device_, gnss_source);
  if (mip_cmd_result == mip::CmdResult::NACK_INVALID_PARAM)
  {
    MICROSTRAIN_WARN(node_, "Gnss source 0x%02x is not valid for this device. Please refer to the device manual for more information", static_cast<uint32_t>(gnss_source));
  }
  else if (!mip_cmd_result)
  {
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to set gnss source");
    return false;
  }
  return true;
}


bool Config::configureHeadingSource(const mip::commands_filter::HeadingSource::Source heading_source)
{
  const mip::CmdResult mip_cmd_result = mip::commands_filter::writeHeadingSource(*mip_device_, heading_source);
  if (mip_cmd_result == mip::CmdResult::NACK_INVALID_PARAM)
  {
    MICROSTRAIN_WARN(node_, "Heading source 0x%02x is not valid for this device. Please refer to the device manual for more information", static_cast<uint32_t>(heading_source));
  }
  else if (!mip_cmd_result)
  {
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to set heading source");
    return false;
  }
  return true;
}

bool Config::populateNmeaMessageFormat(RosNodeType* config_node, const std::string& data_rate_key, mip::commands_3dm::NmeaMessage::TalkerID talker_id, uint8_t descriptor_set, mip::commands_3dm::NmeaMessage::MessageID message_id, std::vector<mip::commands_3dm::NmeaMessage>* formats)
{
  // Get the data rate for this message
  float data_rate;
  double real_data_rate;
  getParamFloat(config_node, data_rate_key, data_rate, 0);

  // Determine if we need a talker ID for this sentence
  bool talker_id_required = false;
  if (MipMapping::nmea_message_id_requires_talker_id_mapping_.find(message_id) != MipMapping::nmea_message_id_requires_talker_id_mapping_.end())
    talker_id_required = MipMapping::nmea_message_id_requires_talker_id_mapping_.at(message_id);

  // Populate the format object
  mip::commands_3dm::NmeaMessage format;
  format.decimation = mip_device_->getDecimationFromHertz(descriptor_set, data_rate, &real_data_rate);
  format.message_id = message_id;
  format.source_desc_set = descriptor_set;
  if (talker_id_required)
    format.talker_id = talker_id;

  // If the data rate is 0, we can just not add the structure to the vector
  const std::string& descriptor_set_string = MipMapping::descriptorSetString(descriptor_set);
  const std::string& message_id_string = MipMapping::nmeaFormatMessageIdString(message_id);
  const std::string& talker_id_string = MipMapping::nmeaFormatTalkerIdString(talker_id);
  if (data_rate != 0)
  {
    // Save the data rate if it is the highest one
    if (real_data_rate > nmea_max_rate_hz_)
      nmea_max_rate_hz_ = real_data_rate;

    // If we already have a message format with this talker ID, error or warn depending on config
    // Note that it is TECHNICALLY valid to have multiple configurations for the same NMEA sentence, but I can think of no reason why it would be useful, so we will also error on that
    if (format.talker_id != mip::commands_3dm::NmeaMessage::TalkerID::IGNORED)
    {
      for (const auto& existing_format : *formats)
      {
        if (existing_format.message_id == format.message_id && existing_format.talker_id == format.talker_id)
        {
          if (!nmea_message_allow_duplicate_talker_ids_)
          {
            MICROSTRAIN_ERROR(node_, "There is already an existing NMEA message with message ID: %s and talker ID: %s from the '%s' descriptor set.", message_id_string.c_str(), talker_id_string.c_str(), MipMapping::descriptorSetString(existing_format.source_desc_set).c_str());
            return false;
          }
          else
          {
            MICROSTRAIN_WARN(node_, "There is already an existing NMEA message with message ID: %s and talker ID: %s from the '%s' descriptor set.", message_id_string.c_str(), talker_id_string.c_str(), MipMapping::descriptorSetString(existing_format.source_desc_set).c_str());
            MICROSTRAIN_WARN(node_, "  Configuration will continue, but you will not be able to differentiate between %s%s NMEA sentences from the '%s' descriptor set and the '%s' descriptor set when they are published", talker_id_string.c_str(), message_id_string.c_str(), descriptor_set_string.c_str(), MipMapping::descriptorSetString(existing_format.source_desc_set).c_str());
          }
        }
      }
    }

    // Should finally have the fully formed struct, so add it to the vector
    if (talker_id_required)
      MICROSTRAIN_INFO(node_, "Configuring %s%s NMEA sentence from the '%s' descriptor set to stream at %.04f hz", talker_id_string.c_str(), message_id_string.c_str(), descriptor_set_string.c_str(), data_rate);
    else
      MICROSTRAIN_INFO(node_, "Configuring %s NMEA sentence from the '%s' descriptor set to stream at %.04f hz", message_id_string.c_str(), descriptor_set_string.c_str(), data_rate);
    formats->push_back(format);

    // Enable NMEA parsing on the main port
    mip_device_->connection()->shouldParseNmea(true);
  }
  else
  {
    MICROSTRAIN_DEBUG(node_, "Disabling %s%s NMEA sentence from the '%s' descriptor set becauese the data rate was 0", talker_id_string.c_str(), message_id_string.c_str(), descriptor_set_string.c_str());
  }
  return true;
}

tf2::Transform Config::lookupLeverArmOffsetInMicrostrainVehicleFrame(const std::string& target_frame_id)
{
  // Wait until we can find the transform for the requested frame id
  std::string tf_error_string;
  RosTimeType frame_time; setRosTime(&frame_time, 0, 0);
  constexpr int32_t seconds_to_wait = 2;
  while (!transform_buffer_->canTransform(frame_id_, target_frame_id, frame_time, RosDurationType(seconds_to_wait, 0), &tf_error_string) && rosOk())
  {
    MICROSTRAIN_WARN(node_, "Timed out waiting for transform from %s to %s, tf error: %s", frame_id_.c_str(), target_frame_id.c_str(), tf_error_string.c_str());
  }

  // If not using the enu frame, this can be plugged directly into the device, otherwise rotate it from the ROS body frame to our body frame
  tf2::Transform target_frame_to_microstrain_vehicle_frame_transform_tf;
  if (use_enu_frame_)
  {
    const auto& target_frame_to_ros_vehicle_frame_transform = transform_buffer_->lookupTransform(frame_id_, target_frame_id, frame_time);

    tf2::Transform target_frame_to_ros_vehicle_frame_transform_tf;
    tf2::fromMsg(target_frame_to_ros_vehicle_frame_transform.transform, target_frame_to_ros_vehicle_frame_transform_tf);

    target_frame_to_microstrain_vehicle_frame_transform_tf = ros_vehicle_to_microstrain_vehicle_transform_tf_ * target_frame_to_ros_vehicle_frame_transform_tf;
  }
  else
  {
    const auto& target_frame_to_microstrain_vehicle_frame_transform = transform_buffer_->lookupTransform(frame_id_, target_frame_id, frame_time);
    tf2::fromMsg(target_frame_to_microstrain_vehicle_frame_transform.transform, target_frame_to_microstrain_vehicle_frame_transform_tf);
  }

  return target_frame_to_microstrain_vehicle_frame_transform_tf;
}

}  // namespace microstrain

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
#include <memory>
#include <iomanip>

#include "microstrain_inertial_driver_common/services.h"

namespace microstrain
{
Services::Services(RosNodeType* node, Config* config) : node_(node), config_(config)
{
}

bool Services::configure()
{
  // Setup custom services
  raw_file_config_main_read_service_ = createService<RawFileConfigReadSrv>(node_, RAW_FILE_CONFIG_MAIN_READ_SERVICE, &Services::rawFileConfigMainRead, this);
  raw_file_config_main_write_service_ = createService<RawFileConfigWriteSrv>(node_, RAW_FILE_CONFIG_MAIN_WRITE_SERVICE, &Services::rawFileConfigMainWrite, this);
  if (config_->aux_device_ != nullptr)
  {
    raw_file_config_aux_read_service_ = createService<RawFileConfigReadSrv>(node_, RAW_FILE_CONFIG_AUX_READ_SERVICE, &Services::rawFileConfigAuxRead, this);
    raw_file_config_aux_write_service_ = createService<RawFileConfigWriteSrv>(node_, RAW_FILE_CONFIG_AUX_WRITE_SERVICE, &Services::rawFileConfigAuxWrite, this);
  }

  // Setup the MIP services
  {
    using namespace mip::commands_base;  // NOLINT(build/namespaces)
    mip_base_get_device_information_service_ = configureService<MipBaseGetDeviceInformationSrv, GetDeviceInfo>(MIP_BASE_GET_DEVICE_INFORMATION_SERVICE, &Services::mipBaseGetDeviceInformation);
  }
  {
    using namespace mip::commands_3dm;  // NOLINT(build/namespaces)
    mip_3dm_capture_gyro_bias_service_ = configureService<Mip3dmCaptureGyroBiasSrv, CaptureGyroBias>(MIP_3DM_CAPTURE_GYRO_BIAS_SERVICE, &Services::mip3dmCaptureGyroBias);
    mip_3dm_device_settings_save_service_ = configureService<EmptySrv, DeviceSettings>(MIP_3DM_DEVICE_SETTINGS_SAVE_SERVICE, &Services::mip3dmDeviceSettingsSave);
    mip_3dm_device_settings_load_service_ = configureService<EmptySrv, DeviceSettings>(MIP_3DM_DEVICE_SETTINGS_LOAD_SERVICE, &Services::mip3dmDeviceSettingsLoad);
    mip_3dm_gpio_state_read_service_ = configureService<Mip3dmGpioStateReadSrv, GpioConfig>(MIP_3DM_GPIO_STATE_READ_SERVICE, &Services::mip3dmGpioStateRead);
    mip_3dm_gpio_state_write_service_ = configureService<Mip3dmGpioStateWriteSrv, GpioConfig>(MIP_3DM_GPIO_STATE_WRITE_SERVICE, &Services::mip3dmGpioStateWrite);
  }
  {
    using namespace mip::commands_filter;  // NOLINT(build/namespaces)
    mip_filter_reset_service_ = configureService<EmptySrv, Reset>(MIP_FILTER_RESET_SERVICE, &Services::mipFilterReset);
  }

  return true;
}

bool Services::rawFileConfigMainRead(RawFileConfigReadSrv::Request& req, RawFileConfigReadSrv::Response& res)
{
  // Make sure that the connection is initialized
  const auto connection = config_->mip_device_->connection();
  if (connection == nullptr)
    return false;

  // Get the state of the connection
  res.enable = connection->rawFileEnable();
  res.file_path = connection->rawFilePath();
  return true;
}

bool Services::rawFileConfigMainWrite(RawFileConfigWriteSrv::Request& req, RawFileConfigWriteSrv::Response& res)
{
  // Make sure that the connection is initialized
  const auto connection = config_->mip_device_->connection();
  if (connection == nullptr)
    return false;

  // Turn on factory support and aiding control if we are recording the binary. If not, we can't really turn them off, so just leave them on
  if (req.enable)
  {
    // Enable factory support
    if (config_->mip_device_->supportsDescriptor(mip::commands_3dm::DESCRIPTOR_SET, mip::commands_3dm::CMD_CONFIGURE_FACTORY_STREAMING))
    {
      const mip::CmdResult mip_cmd_result = mip::commands_3dm::factoryStreaming(*(config_->mip_device_), mip::commands_3dm::FactoryStreaming::Action::MERGE, 0);
      if (!mip_cmd_result)
      {
        MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to configure factory streaming channels");
        return false;
      }
      else
      {
        MICROSTRAIN_DEBUG(node_, "Added factory streaming data");
      }
    }

    // Also enable aiding command echo when collecting a factory support binary
    if (config_->mip_device_->supportsDescriptor(mip::commands_aiding::DESCRIPTOR_SET, mip::commands_aiding::AidingEchoControl::FIELD_DESCRIPTOR))
    {
      const mip::CmdResult mip_cmd_result = mip::commands_aiding::writeAidingEchoControl(*(config_->mip_device_), mip::commands_aiding::AidingEchoControl::Mode::RESPONSE);
      if (!mip_cmd_result)
      {
        MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to configure aiding echo control");
        return false;
      }
      else
      {
        MICROSTRAIN_DEBUG(node_, "Added aiding messages to binary");
      }
    }
  }

  // Update the recording status of the connection
  return connection->updateRecordingState(req.enable, req.file_path);
}

bool Services::rawFileConfigAuxRead(RawFileConfigReadSrv::Request& req, RawFileConfigReadSrv::Response& res)
{
  // Make sure that the connection is initialized
  if (config_->aux_device_ == nullptr)
    return false;
  const auto connection = config_->aux_device_->connection();
  if (connection == nullptr)
    return false;

  // Get the state of the connection
  res.enable = connection->rawFileEnable();
  res.file_path = connection->rawFilePath();
  return true;
}

bool Services::rawFileConfigAuxWrite(RawFileConfigWriteSrv::Request& req, RawFileConfigWriteSrv::Response& res)
{
  // Make sure that the connection is initialized
  if (config_->aux_device_ == nullptr)
    return false;
  const auto connection = config_->aux_device_->connection();
  if (connection == nullptr)
    return false;

  // Update the recording status of the connection
  return connection->updateRecordingState(req.enable, req.file_path);
}

bool Services::mipBaseGetDeviceInformation(MipBaseGetDeviceInformationSrv::Request& req, MipBaseGetDeviceInformationSrv::Response& res)
{
  MICROSTRAIN_DEBUG(node_, "Getting device information");

  const mip::CmdResult mip_cmd_result = config_->mip_device_->getDeviceInfo(&config_->mip_device_->device_info_);
  if (!!mip_cmd_result)
  {
    MICROSTRAIN_DEBUG(node_, "Got device information");
    MICROSTRAIN_DEBUG(node_, "  model name = %s", config_->mip_device_->device_info_.model_name);
    MICROSTRAIN_DEBUG(node_, "  lot number = %s", config_->mip_device_->device_info_.lot_number);
    MICROSTRAIN_DEBUG(node_, "  model number = %s", config_->mip_device_->device_info_.model_number);
    MICROSTRAIN_DEBUG(node_, "  serial number = %s", config_->mip_device_->device_info_.serial_number);
    MICROSTRAIN_DEBUG(node_, "  device options = %s", config_->mip_device_->device_info_.device_options);
    MICROSTRAIN_DEBUG(node_, "  firmware version (raw) = %u", config_->mip_device_->device_info_.firmware_version);

    res.device_info.firmware_version = RosMipDevice::firmwareVersionString(config_->mip_device_->device_info_.firmware_version);
    res.device_info.model_name = config_->mip_device_->device_info_.model_name;
    res.device_info.model_number = config_->mip_device_->device_info_.model_number;
    res.device_info.serial_number = config_->mip_device_->device_info_.serial_number;
    res.device_info.lot_number = config_->mip_device_->device_info_.lot_number;
    res.device_info.device_options = config_->mip_device_->device_info_.device_options;
  }
  else
  {
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to get device info");
  }
  return !!mip_cmd_result;
}

bool Services::mip3dmCaptureGyroBias(Mip3dmCaptureGyroBiasSrv::Request& req, Mip3dmCaptureGyroBiasSrv::Response& res)
{
  const int32_t capture_timeout = 10000;
  MICROSTRAIN_DEBUG(node_, "Capturing gyro bias");
  MICROSTRAIN_WARN(node_, "Performing Gyro Bias capture. Device will pause publshing during this time period");
  MICROSTRAIN_WARN(node_, "Please keep device stationary during the %f second gyro bias capture interval", static_cast<float>(capture_timeout) / 1000);

  // We need to change the timeout to allow for this longer command
  const int32_t old_mip_sdk_timeout = config_->mip_device_->device().baseReplyTimeout();
  config_->mip_device_->device().setBaseReplyTimeout(capture_timeout * 2);

  float gyro_bias[3];
  const mip::CmdResult mip_cmd_result = mip::commands_3dm::captureGyroBias(*(config_->mip_device_), capture_timeout, gyro_bias);
  if (!!mip_cmd_result)
  {
    MICROSTRAIN_DEBUG(node_, "Captured gyro bias: [%f, %f, %f]", gyro_bias[0], gyro_bias[1], gyro_bias[2]);
    res.bias[0] = gyro_bias[0];
    res.bias[1] = gyro_bias[1];
    res.bias[2] = gyro_bias[2];

    MICROSTRAIN_INFO(node_, "Gyro bias captured, device will now resume normal operations");
  }
  else
  {
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to capture gyro bias");
  }

  // Reset the timeout
  config_->mip_device_->device().setBaseReplyTimeout(old_mip_sdk_timeout);

  return !!mip_cmd_result;
}

bool Services::mip3dmDeviceSettingsSave(EmptySrv::Request& req, EmptySrv::Response& res)
{
  MICROSTRAIN_DEBUG(node_, "Saving device settings");

  // We need to change the timeout to allow for this longer command
  const int32_t old_mip_sdk_timeout = config_->mip_device_->device().baseReplyTimeout();
  config_->mip_device_->device().setBaseReplyTimeout(10000);  // 10 seconds should be enough

  const mip::CmdResult mip_cmd_result = mip::commands_3dm::saveDeviceSettings(*config_->mip_device_);
  if (!!mip_cmd_result)
    MICROSTRAIN_DEBUG(node_, "Saved device settings");
  else
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to save device settings");

  // Reset the timeout
  config_->mip_device_->device().setBaseReplyTimeout(old_mip_sdk_timeout);

  return !!mip_cmd_result;
}

bool Services::mip3dmDeviceSettingsLoad(EmptySrv::Request& req, EmptySrv::Response& res)
{
  MICROSTRAIN_DEBUG(node_, "Loading device settings");

  // We need to change the timeout to allow for this longer command
  const int32_t old_mip_sdk_timeout = config_->mip_device_->device().baseReplyTimeout();
  config_->mip_device_->device().setBaseReplyTimeout(10000);  // 10 seconds should be enough

  const mip::CmdResult mip_cmd_result = mip::commands_3dm::loadDeviceSettings(*config_->mip_device_);
  if (!!mip_cmd_result)
    MICROSTRAIN_DEBUG(node_, "Loaded device settings");
  else
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to load device settings");

  // Reset the timeout
  config_->mip_device_->device().setBaseReplyTimeout(old_mip_sdk_timeout);

  return !!mip_cmd_result;
}

bool Services::mipFilterReset(EmptySrv::Request& req, EmptySrv::Response& res)
{
  MICROSTRAIN_DEBUG(node_, "Resetting filter");

  const mip::CmdResult mip_cmd_result = mip::commands_filter::reset(*(config_->mip_device_));
  if (!!mip_cmd_result)
    MICROSTRAIN_DEBUG(node_, "Reset filter");
  else
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to reset filter");

  // If we are using auto relative position config, reset that flag
  if (config_->filter_relative_pos_source_ == REL_POS_SOURCE_AUTO)
  {
    config_->map_to_earth_transform_valid_ = false;
    config_->filter_state_ = static_cast<mip::data_filter::FilterMode>(0);
  }

  return !!mip_cmd_result;
}

bool Services::mip3dmGpioStateRead(Mip3dmGpioStateReadSrv::Request& req, Mip3dmGpioStateReadSrv::Response& res)
{
  MICROSTRAIN_DEBUG(node_, "Reading GPIO state for pin %u", req.pin);

  bool state;
  const mip::CmdResult mip_cmd_result = mip::commands_3dm::readGpioState(*(config_->mip_device_), req.pin, &state);
  if (!!mip_cmd_result)
    MICROSTRAIN_DEBUG(node_, "Read GPIO state for pin %u: %d", req.pin, state);
  else
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to read GPIO state");

  return !!mip_cmd_result;
}

bool Services::mip3dmGpioStateWrite(Mip3dmGpioStateWriteSrv::Request& req, Mip3dmGpioStateWriteSrv::Response& res)
{
  MICROSTRAIN_DEBUG(node_, "Writing GPIO state for pin %u", req.pin);
  MICROSTRAIN_DEBUG(node_, "  state = %d", req.state);

  const mip::CmdResult mip_cmd_result = mip::commands_3dm::writeGpioState(*(config_->mip_device_), req.pin, req.state);
  if (!!mip_cmd_result)
    MICROSTRAIN_DEBUG(node_, "Wrote GPIO state for pin %u", req.pin);
  else
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Failed to write GPIO state");

  return !!mip_cmd_result;
}

}  // namespace microstrain

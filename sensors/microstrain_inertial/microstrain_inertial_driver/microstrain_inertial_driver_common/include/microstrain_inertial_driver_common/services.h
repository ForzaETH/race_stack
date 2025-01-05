/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Parker-Lord Driver Definition File
//
// Copyright (c) 2017, Brian Bingham
// Copyright (c) 2020, Parker Hannifin Corp
//
// This code is licensed under MIT license (see LICENSE file for details)
//
/////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef MICROSTRAIN_INERTIAL_DRIVER_COMMON_SERVICES_H
#define MICROSTRAIN_INERTIAL_DRIVER_COMMON_SERVICES_H

#include <memory>
#include <string>

#include "microstrain_inertial_driver_common/utils/ros_compat.h"
#include "microstrain_inertial_driver_common/config.h"

namespace microstrain
{

static constexpr auto RAW_FILE_CONFIG_MAIN_READ_SERVICE = "raw_file_config/main/read";
static constexpr auto RAW_FILE_CONFIG_MAIN_WRITE_SERVICE = "raw_file_config/main/write";
static constexpr auto RAW_FILE_CONFIG_AUX_READ_SERVICE = "raw_file_config/aux/read";
static constexpr auto RAW_FILE_CONFIG_AUX_WRITE_SERVICE = "raw_file_config/aux/write";
static constexpr auto MIP_BASE_GET_DEVICE_INFORMATION_SERVICE = "mip/base/get_device_information";
static constexpr auto MIP_3DM_CAPTURE_GYRO_BIAS_SERVICE = "mip/three_dm/capture_gyro_bias";
static constexpr auto MIP_3DM_DEVICE_SETTINGS_SAVE_SERVICE = "mip/three_dm/device_settings/save";
static constexpr auto MIP_3DM_DEVICE_SETTINGS_LOAD_SERVICE = "mip/three_dm/device_settings/load";
static constexpr auto MIP_3DM_GPIO_STATE_READ_SERVICE = "mip/three_dm/gpio_state/read";
static constexpr auto MIP_3DM_GPIO_STATE_WRITE_SERVICE = "mip/three_dm/gpio_state/write";
static constexpr auto MIP_FILTER_RESET_SERVICE = "mip/ekf/reset";

/**
 * Contains service functions and service handles
 */
class Services
{
public:
  /**
   * Default Constructor
   */
  Services() = default;

  /**
   * \brief Constructs this class with a reference to the node, and a config object
   * \param node  Reference to a node that will be saved to this class and used to log and interact with ROS
   * \param config Reference to the config object that will be saved to this class and used to determine whether or not to enable the services
   */
  Services(RosNodeType* node, Config* config);

  /**
   * \brief Configures the services. After this function is called, the services will be created, but (ROS2 only) will not be activated
   * \return true if configuration was successful and false if configuration failed
   */
  bool configure();

  // Service functions. Too many to document
  bool rawFileConfigMainRead(RawFileConfigReadSrv::Request& req, RawFileConfigReadSrv::Response& res);
  bool rawFileConfigMainWrite(RawFileConfigWriteSrv::Request& req, RawFileConfigWriteSrv::Response& res);
  bool rawFileConfigAuxRead(RawFileConfigReadSrv::Request& req, RawFileConfigReadSrv::Response& res);
  bool rawFileConfigAuxWrite(RawFileConfigWriteSrv::Request& req, RawFileConfigWriteSrv::Response& res);

  bool mipBaseGetDeviceInformation(MipBaseGetDeviceInformationSrv::Request& req, MipBaseGetDeviceInformationSrv::Response& res);

  bool mip3dmCaptureGyroBias(Mip3dmCaptureGyroBiasSrv::Request& req, Mip3dmCaptureGyroBiasSrv::Response& res);
  bool mip3dmDeviceSettingsSave(EmptySrv::Request& req, EmptySrv::Response& res);
  bool mip3dmDeviceSettingsLoad(EmptySrv::Request& req, EmptySrv::Response& res);
  bool mip3dmGpioStateRead(Mip3dmGpioStateReadSrv::Request& req, Mip3dmGpioStateReadSrv::Response& res);
  bool mip3dmGpioStateWrite(Mip3dmGpioStateWriteSrv::Request& req, Mip3dmGpioStateWriteSrv::Response& res);

  bool mipFilterReset(EmptySrv::Request& req, EmptySrv::Response& res);

private:
  /**
   * \brief Configures a non MIP command dependent service. This service will always be configured if this functions is called.
   * \tparam ServiceType. The ROS service type that the service will use
   * \param name The name to give the service
   * \param callback The callback to trigger when the service is called
   * \return Pointer to an initialized service
   */
  template<typename ServiceType>
  typename RosServiceType<ServiceType>::SharedPtr configureService(const std::string& name, bool (Services::*callback)(typename ServiceType::Request&, typename ServiceType::Response&));

  /**
   * \brief Configures a MIP command dependent service. This service will only be configured if the device supports the command
   * \tparam ServiceType The ROS service type that the service will use
   * \tparam MipType The MIP type that the device must support
   * \tparam DescriptorSet The descriptor set to use with the MipType if the MipType's descriptor set should not be used
   * \param name The name to give the service
   * \param callback The callback to trigger when the service is called
   * \return Pointer to an initialized service, or nullptr if the device does not support the MipType
   */
  template<typename ServiceType, typename MipType, uint8_t DescriptorSet = MipType::DESCRIPTOR_SET>
  typename RosServiceType<ServiceType>::SharedPtr configureService(const std::string& name, bool (Services::*callback)(typename ServiceType::Request&, typename ServiceType::Response&));

  // Handles to the ROS node and the config
  RosNodeType* node_;
  Config* config_;

  RosServiceType<RawFileConfigReadSrv>::SharedPtr raw_file_config_main_read_service_;
  RosServiceType<RawFileConfigWriteSrv>::SharedPtr raw_file_config_main_write_service_;
  RosServiceType<RawFileConfigReadSrv>::SharedPtr raw_file_config_aux_read_service_;
  RosServiceType<RawFileConfigWriteSrv>::SharedPtr raw_file_config_aux_write_service_;

  RosServiceType<MipBaseGetDeviceInformationSrv>::SharedPtr mip_base_get_device_information_service_;

  RosServiceType<Mip3dmCaptureGyroBiasSrv>::SharedPtr mip_3dm_capture_gyro_bias_service_;
  RosServiceType<EmptySrv>::SharedPtr mip_3dm_device_settings_save_service_;
  RosServiceType<EmptySrv>::SharedPtr mip_3dm_device_settings_load_service_;
  RosServiceType<Mip3dmGpioStateReadSrv>::SharedPtr mip_3dm_gpio_state_read_service_;
  RosServiceType<Mip3dmGpioStateWriteSrv>::SharedPtr mip_3dm_gpio_state_write_service_;

  RosServiceType<EmptySrv>::SharedPtr mip_filter_reset_service_;
};

template<typename ServiceType>
typename RosServiceType<ServiceType>::SharedPtr Services::configureService(const std::string& name, bool (Services::*callback)(typename ServiceType::Request&, typename ServiceType::Response&))
{
  MICROSTRAIN_DEBUG(node_, "Configuring service %s", name.c_str());
  return createService<ServiceType>(node_, name, callback, this);
}

template<typename ServiceType, typename MipType, uint8_t DescriptorSet>
typename RosServiceType<ServiceType>::SharedPtr Services::configureService(const std::string& name, bool (Services::*callback)(typename ServiceType::Request&, typename ServiceType::Response&))
{
  if (config_->mip_device_->supportsDescriptor(DescriptorSet, MipType::FIELD_DESCRIPTOR))
  {
    MICROSTRAIN_DEBUG(node_, "Configuring service %s to execute MIP command 0x%02x%02x", name.c_str(), DescriptorSet, MipType::FIELD_DESCRIPTOR);
    return createService<ServiceType>(node_, name, callback, this);
  }
  else
  {
    MICROSTRAIN_DEBUG(node_, "Device does not support the %s service because the device does not support descriptor 0x%02x%02x", name.c_str(), DescriptorSet, MipType::FIELD_DESCRIPTOR);
    return nullptr;
  }
}

}  // namespace microstrain

#endif  // MICROSTRAIN_INERTIAL_DRIVER_COMMON_SERVICES_H

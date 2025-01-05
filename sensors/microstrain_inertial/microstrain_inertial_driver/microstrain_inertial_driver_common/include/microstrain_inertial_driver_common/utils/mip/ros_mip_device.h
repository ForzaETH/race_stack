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

#ifndef MICROSTRAIN_INERTIAL_DRIVER_COMMON_UTILS_MIP_ROS_MIP_DEVICE_H
#define MICROSTRAIN_INERTIAL_DRIVER_COMMON_UTILS_MIP_ROS_MIP_DEVICE_H

#include <map>
#include <string>
#include <memory>
#include <vector>
#include <algorithm>
#include <functional>

#include "mip/mip_device.hpp"

#include "mip/definitions/commands_base.hpp"
#include "mip/definitions/commands_3dm.hpp"

#include "microstrain_inertial_driver_common/utils/mip/ros_connection.h"

namespace microstrain
{

/**
 * Wrapper to hold onto the mip::DeviceInterface and mip::Connection object and add convenience functions
 */
class RosMipDevice
{
 public:
  /**
   * \brief Initializes the device with a reference to the ROS node
   * \param node The node that this device was initialized with
   */
  explicit RosMipDevice(RosNodeType* node);

  /**
   * \brief Pure virtual configure function
   * \param config_node ROS node with information on how to configure this device
   * \return true if configuration was successful, false otherwise
   */
  virtual bool configure(RosNodeType* config_node) = 0;

  /**
   * \brief Convenience operator to allow this class to be used in place of a mip::DeviceInterface.
   *        Will throw if the device has not been initialized
   */
  operator mip::DeviceInterface&()
  {
    if (device_ != nullptr)
      return *device_;
    else
      throw std::runtime_error("Attempt to cast to device on RosMipDevice before it was initialized");
  }

  /**
   * \brief Convenience function to get a reference to the device interface.
   *        Will throw if the device has not been initialized
   * \return Reference to the device interface in this object
   */
  mip::DeviceInterface& device();

  /**
   * \brief Determines if the given device info is from a prospect device
   * \param device_info Populated and null terminated string version of the device info struct fetched from a device
   * \return true if the device is a prospect device, false otherwise
  */
  static bool isProspect(const mip::commands_base::BaseDeviceInfo& device_info);

  /**
   * \brief Determines if the given device info is from a philo device
   * \param device_info Populated and null terminated string version of the device info struct fetched from a device
   * \return true if the device is a philo device, false otherwise
  */
  static bool isPhilo(const mip::commands_base::BaseDeviceInfo& device_info);

  /**
   * \brief Determines if the given device info is from a GQ7
   * \param device_info Populated and null terminated string version of the device info struct fetched from a device
   * \return true if the device is a GQ7, false otherwise
  */
  static bool isGq7(const mip::commands_base::BaseDeviceInfo& device_info);

  /**
   * \brief Determines if the given device info is from a CV7
   * \param device_info Populated and null terminated string version of the device info struct fetched from a device
   * \return true if the device is a CV7, false otherwise
  */
  static bool isCv7(const mip::commands_base::BaseDeviceInfo& device_info);

  /**
   * \brief Sends data to the device
   * \param data Byte array to send to the device
   * \param data_len Length in bytes of the data array
   * \return true if the data was successfully sent, false otherwise
   */
  bool send(const uint8_t* data, size_t data_len);

  /**
   * \brief Receives data from the device
   * \param data Byte buffer to hold the data received from the device
   * \param data_len Max size of the data buffer
   * \param out_len Will contain the number of bytes actually read into the buffer
   * \return true if the data was successfully received, false otherwise
   */
  bool recv(uint8_t* data, size_t data_len, size_t* out_len);

  /**
   * \brief Attempts to connect to the device
   * \return true if the device was able to connect, false otherwise
  */
  bool connect();

  /**
   * \brief Attempts to disconnect from the device
   * \return true if the device was able to disconnect, false otherwise
  */
  bool disconnect();

  /**
   * \brief Attempts to reconnect to the device
   * \return true if the device was able to reconnect, false otherwise
  */
  bool reconnect();

  /**
   * \brief Gets the connection object
   * \return The connection object
   */
  std::shared_ptr<RosConnection> connection();

  /**
   * \brief Gets the device info from the device, and modifies the strings to be usable from C/C++
   * \param device_info Object to populate wih the device info
   * \return MIP command result representing the status of the call
   */
  mip::CmdResult getDeviceInfo(mip::commands_base::BaseDeviceInfo* device_info);

  /**
   * \brief Converts the uint16 firmware version returned from the device into a firmware string
   * \param firmware_version The firmware version returned from the device
   * \return The string interpretation of the firmware version
   */
  static std::string firmwareVersionString(uint16_t firmware_version);

 protected:
  /**
   * \brief Fixes string returned by the MIP device to work like normal strings. Updated string will be stored in the same buffer
   * \param str The string returned by the MIP device
   * \param str_len The length of the string buffer. Not the length of the string itself
   */
  static void fixMipString(char* str, const size_t str_len);

  RosNodeType* node_;  /// Reference to the ROS node that created this object

  std::shared_ptr<RosConnection> connection_;  // Pointer to the MIP connection
  std::unique_ptr<::mip::DeviceInterface> device_;  // Pointer to the device. Public so that functions that do not need to be wrapped can be called directly

  uint8_t buffer_[1024];  // Buffer to use for the MIP device
};

/**
 * \brief Helper macro for logging an error that occurs with the MIP SDK
 * \param node The ROS node object
 * \param mip_cmd_result The result of the MIP command that should be logged as an error
 * \param log Log string to log as an error
 */
#define MICROSTRAIN_MIP_SDK_ERROR(node, mip_cmd_result, log) \
  do \
  { \
    MICROSTRAIN_ERROR(node, log); \
    MICROSTRAIN_ERROR(node, "  Error(%d): %s", mip_cmd_result.value, mip_cmd_result.name()); \
  } while (0)

}  // namespace microstrain

#endif  // MICROSTRAIN_INERTIAL_DRIVER_COMMON_UTILS_MIP_ROS_MIP_DEVICE_H

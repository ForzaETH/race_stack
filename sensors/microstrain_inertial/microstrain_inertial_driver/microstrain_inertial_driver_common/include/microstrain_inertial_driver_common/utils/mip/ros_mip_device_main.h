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


#ifndef MICROSTRAIN_INERTIAL_DRIVER_COMMON_UTILS_MIP_ROS_MIP_DEVICE_MAIN_H
#define MICROSTRAIN_INERTIAL_DRIVER_COMMON_UTILS_MIP_ROS_MIP_DEVICE_MAIN_H

#include <map>
#include <memory>
#include <vector>
#include <algorithm>
#include <functional>

#include "mip/definitions/commands_base.hpp"
#include "mip/definitions/commands_3dm.hpp"

#include "microstrain_inertial_driver_common/utils/mip/ros_mip_device.h"

namespace microstrain
{

/**
 * Implementation of the RosMipDevice for the main port on a device
 */
class RosMipDeviceMain : public RosMipDevice
{
 public:
  using RosMipDevice::RosMipDevice;

  /**
   * \brief Configures the main device connection
   * \param config_node ROS node with configuration options used to configure the main connection
   */
  bool configure(RosNodeType* config_node) final;

  /**
   * \brief Forces the device to idle. It is possible for us to drop the response to the first setToIdle command.
   *        To solve the problem, we send the command multiple times with an interval in between until we get a success response
   * \return MIP command result reflecting the status of the most recent setToIdle command
   */
  mip::CmdResult forceIdle();

  /**
   * \brief Updates the supported device descriptors stored in this class
   * \return MIP command result reflecting the status of the command
   */
  mip::CmdResult updateDeviceDescriptors();

  /**
   * \brief Updates the base rate stored for the provided descriptor set
   * \param descriptor_set The descriptor set to update the base rate of.
   * \return MIP command result reflecting the status of the command
   */
  mip::CmdResult updateBaseRate(const uint8_t descriptor_set);

  /**
   * \brief Helper function to write the baudrate in a way that will work for all devices
   * \param baudrate The baudrate to write to the device
   * \param port Option port option which will be used on prospect devices to determine the port to change the baudrate on
   * \return MIP command result reflecting the status of the command
   */
  mip::CmdResult writeBaudRate(uint32_t baudrate, uint8_t port = 1);

  /**
   * \brief Helper function to read the message format in a way that will work for all devices
   * \param descriptor_set The descriptor set that the message format will be read for
   * \param num_descriptors Will be filled out with the number of mip::DescriptorRate objects in the descriptors pointer
   * \param num_descriptors_max Max number of descriptor rates that can be returned by this call
   * \param descriptors Descriptor rates returned from the device
   * \return MIP command result reflecting the status of the command
   */
  mip::CmdResult readMessageFormat(uint8_t descriptor_set, uint8_t* num_descriptors, uint8_t num_descriptors_max, mip::DescriptorRate* descriptors);

  /**
   * \brief Helper function to write the message format in a way that will work for all devices
   * \param descriptor_set The descriptor set that the message format will be writen for
   * \param num_descriptors The number of mip::DescriptorRate objects in the descriptors pointer
   * \param descriptors Descriptor rates to write to the device
   * \return MIP command result reflecting the status of the command
   */
  mip::CmdResult writeMessageFormat(uint8_t descriptor_set, uint8_t num_descriptors, const mip::DescriptorRate* descriptors);

  /**
   * \brief Helper function to enable or disable a datastream that will work for all devices.
   * \param descriptor_set The descriptor set to enable or disable
   * \param enable Whether to enable or disable the descriptor set
   * \return MIP command result reflecting the status of the command
   */
  mip::CmdResult writeDatastreamControl(uint8_t descriptor_set, bool enable);

  /**
   * \brief Convenience function to check if the device supports a given descriptor set
   * \param descriptor_set The descriptor set to check if the device supports
   * \return true if the device supports the descriptor set, false if not
   */
  bool supportsDescriptorSet(const uint8_t descriptor_set);

  /**
   * \brief Convenience function to check if the device supports a given field descriptor in a descriptor set
   * \param descriptor_set The descriptor set that the field_descriptor is in
   * \param field_descriptor The field descriptor to check if the device supports
   * \return true if the device supports the field descriptor within the descriptor set, false if not
   */
  bool supportsDescriptor(const uint8_t descriptor_set, const uint8_t field_descriptor);

  /**
   * \brief Convenience function to stream a descriptor at a requested data rate.
   * \param descriptor_set The descriptor set that the field_descriptor is in
   * \param field_descriptor The field descriptor to stream
   * \param hertz The data rate in hertz to stream the descriptor at
   * \return MIP command result reflecting the status of the command
  */
  mip::CmdResult streamDescriptor(const uint8_t descriptor_set, uint8_t field_descriptor, float hertz);

  /**
   * \brief Converts a value in hertz to a decimation value for a descriptor set
   * \param descriptor_set The descriptor set to use to lookup the base rate value for
   * \param hertz The value in hertz to convert to decmiation
   * \param actual_hertz Optional pointer to store the actual hertz that we can stream the data at
   * \return The decimation value for the given descriptor set and hertz
   */
  uint16_t getDecimationFromHertz(const uint8_t descriptor_set, const float hertz, double* actual_hertz = nullptr);

  // Expose some useful members, including the device information
  mip::commands_base::BaseDeviceInfo device_info_;

  // Number of frame IDs supported by this device
  uint16_t max_external_frame_ids_ = 0;

 private:
  std::vector<uint8_t> supported_descriptor_sets_;  // Supported descriptor sets of the node
  std::vector<uint16_t> supported_descriptors_;  // Supported field descriptors of the node

  std::map<uint8_t, uint16_t> base_rates_;  // Mapping between descriptor sets and their base rates.
};

}  // namespace microstrain

#endif  // MICROSTRAIN_INERTIAL_DRIVER_COMMON_UTILS_MIP_ROS_MIP_DEVICE_MAIN_H

/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Parker-Lord Inertial Device Driver Implementation File
//
// Copyright (c) 2017, Brian Bingham
// Copyright (c) 2020, Parker Hannifin Corp
// This code is licensed under MIT license (see LICENSE file for details)
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

#include <stdio.h>

#include <string>
#include <memory>
#include <thread>
#include <chrono>
#include <stdexcept>

#include "microstrain_inertial_driver_common/utils/mip/ros_mip_device_aux.h"

namespace microstrain
{

bool RosMipDeviceAux::configure(RosNodeType* config_node)
{
  // Initialize and connect the connection
  std::string port;
  int32_t baudrate;
  getParam<std::string>(config_node, "aux_port", port, "/dev/ttyACM1");
  getParam<int32_t>(config_node, "aux_baudrate", baudrate, 115200);
  connection_ = std::make_shared<RosConnection>(node_);
  if (!connection_->connect(config_node, port, baudrate))
    return false;

  // Setup the device interface
  mip::CmdResult mip_cmd_result;
  device_ = std::unique_ptr<mip::DeviceInterface>(new mip::DeviceInterface(connection_.get(), buffer_, sizeof(buffer_), connection_->parseTimeout(), connection_->baseReplyTimeout()));

  // Print the device info
  mip::commands_base::BaseDeviceInfo device_info;
  if (!(mip_cmd_result = getDeviceInfo(&device_info)))
  {
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Unable to read device info");
    return false;
  }
  MICROSTRAIN_INFO(node_, R"(Aux Connection Info:
    #######################
    Model Name:       %s
    Serial Number:    %s
    Firmware Version: %s
    #######################)", device_info.model_name, device_info.serial_number, firmwareVersionString(device_info.firmware_version).c_str());

  // Configure the connection with a working device
  if (!connection_->configure(config_node, this))
    return false;

  return true;
}

}  // namespace microstrain

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

#ifndef MICROSTRAIN_INERTIAL_DRIVER_COMMON_UTILS_MIP_ROS_MIP_DEVICE_AUX_H
#define MICROSTRAIN_INERTIAL_DRIVER_COMMON_UTILS_MIP_ROS_MIP_DEVICE_AUX_H

#include <map>
#include <memory>
#include <vector>
#include <algorithm>
#include <functional>

#include "microstrain_inertial_driver_common/utils/mip/ros_mip_device.h"

namespace microstrain
{

/**
 * Implementation of the RosMipDevice for the aux port on a device
 */
class RosMipDeviceAux : public RosMipDevice
{
 public:
  using RosMipDevice::RosMipDevice;

  /**
   * \brief Configures the aux device connection
   * \param config_node ROS node with configuration options used to configure the aux connection
   */
  bool configure(RosNodeType* config_node) final;
};

}  // namespace microstrain

#endif  // MICROSTRAIN_INERTIAL_DRIVER_COMMON_UTILS_MIP_ROS_MIP_DEVICE_AUX_H

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

#ifndef MICROSTRAIN_INERTIAL_DRIVER_COMMON_NODE_COMMON_H
#define MICROSTRAIN_INERTIAL_DRIVER_COMMON_NODE_COMMON_H

#include <stddef.h>
#include <string>
#include <vector>
#include <fstream>

#include "mip/mip_logging.h"

#include "microstrain_inertial_driver_common/config.h"
#include "microstrain_inertial_driver_common/publishers.h"
#include "microstrain_inertial_driver_common/subscribers.h"
#include "microstrain_inertial_driver_common/services.h"

namespace microstrain
{

/**
 * Base class for ROS1 and ROS2 nodes. The ROS1 and ROS2 nodes should extend this class
 */
class NodeCommon
{
public:
  /**
   * \brief Reads messages from the main port of the device, parses them, and publishes them. Meant to be executed in a loop
   */
  void parseAndPublishMain();

  /**
   * \brief Reads messages from the aux port of the device, parses them, and publishes them. Meant to be executed in a loop
   */
  void parseAndPublishAux();

  // Logging callback used by the MIP SDK
  void logCallback(const mip_log_level level, const std::string& log_str);

protected:
  /**
   * \brief Default constructor
   */
  NodeCommon() = default;

  /**
   * \brief Initializes the node into an idle state. In this state, the device is not connected, or configured and no services, publishers, or subscribers are setup
   * \param init_node  The main node that will be saved so it can be used later in the node's life
   * \return true if initialization succeeds and false if the initialization fails
   */
  bool initialize(RosNodeType* init_node);

  /**
   * \brief Configures the node. In this state, the device is connected, and the services, publishers and subscribers have been created
   * \param config_node  The ROS node that contains configuration information. For ROS1 this is the private node handle ("~")
   * \return true if configuration succeeds and false if the configuration fails
   */
  bool configure(RosNodeType* config_node);

  /**
   * \brief Activates the node. In this state, the publishers should be publishing, and the services and subscribers are ready to interact
   * \return true if activation succeeds and false if the activation fails
   */
  bool activate();

  /**
   * \brief Deactivates the node. In this state, the publishers will stop publishing, and the device will be set to idle
   */
  bool deactivate();

  /**
   * \brief Shuts down the node. In this state, all publishers, subscribers, and services will be deallocated
   */
  bool shutdown();

  RosNodeType* node_;
  RosNodeType* config_node_;
  Config config_;
  Publishers publishers_;
  Subscribers subscribers_;
  Services services_;

  double timer_update_rate_hz_;

  RosTimerType main_parsing_timer_;
  RosTimerType aux_parsing_timer_;

  std::string aux_string_;
};  // NodeCommon class

}  // namespace microstrain

#endif  // MICROSTRAIN_INERTIAL_DRIVER_COMMON_NODE_COMMON_H

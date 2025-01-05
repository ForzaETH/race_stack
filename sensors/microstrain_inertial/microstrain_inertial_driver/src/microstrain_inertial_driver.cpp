/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Parker-Lord ROS2 Inertial Driver Implementation File
// 
// Copyright (c) 2017, Brian Bingham
// Copyright (c)  2021, Parker Hannifin Corp
// 
// This code is licensed under MIT license (see LICENSE file for details)
// 
/////////////////////////////////////////////////////////////////////////////////////////////////////

#include <time.h>
#include <math.h>
#include <stdlib.h>

#include <ctime>
#include <string>
#include <vector>
#include <algorithm>

#include <tf2/LinearMath/Transform.h>

#include "lifecycle_msgs/msg/transition.hpp"

#include "microstrain_inertial_driver/microstrain_inertial_driver.h"

namespace microstrain
{

Microstrain::Microstrain() : rclcpp::Node("microstrain_inertial_driver_node")
{
  // Configure the logger
#if MICROSTRAIN_ROLLING == 1 || MICROSTRAIN_HUMBLE == 1 || MICROSTRAIN_GALACTIC == 1
  auto debug_enable = std::getenv("MICROSTRAIN_INERTIAL_DEBUG");
  if (debug_enable != nullptr && std::string(debug_enable) == "true")
    get_logger().set_level(rclcpp::Logger::Level::Debug);
#else
  RCLCPP_INFO(this->get_logger(), "This version of ROS2 does not support changing the log level in C++");
#endif

  //Initialize the helper classes
  if (!NodeCommon::initialize(this))
    RCLCPP_FATAL(this->get_logger(), "Failed to initialize base node");
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Configure Node Function
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::configure_node()
{
  ///////////////////////////////////////////////////////////////////////////
  //
  //Main loop setup
  ///
  ///////////////////////////////////////////////////////////////////////////
  try
  {
    RCLCPP_DEBUG(this->get_logger(), "Initializing base node");
    if (!NodeCommon::configure(this))
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to configure node base");
      return false;
    }
  }
  catch(const std::exception &e)
  {
    RCLCPP_ERROR(this->get_logger(), "Error: %s", e.what());
    return false;
  }

  return true;
} 


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Activate Node Function
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::activate_node()
{
  // Activate the base node to start the background tasks
  if (!NodeCommon::activate())
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to activate base node");
    return false;
  }

  // Start a timer around a wrapper function to catch errors
  main_parsing_timer_ = createTimer<Microstrain>(node_, timer_update_rate_hz_, &Microstrain::parse_and_publish_main_wrapper, this);

  // Start the aux timer if we were requested to do so
  if (config_.aux_device_ != nullptr)
  {
    RCLCPP_INFO(this->get_logger(), "Starting aux port parsing");
    aux_parsing_timer_ = createTimer<Microstrain>(node_, 2.0, &Microstrain::parse_and_publish_aux_wrapper, this);
  }
  
  return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Deactivate Node Function
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::deactivate_node()
{
  //Deactivate the base node
  if (!NodeCommon::deactivate())
  {
    RCLCPP_ERROR(this->get_logger(), "Unable to deactivate node base");
  }

  return true;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Shutdown/Cleanup Node Function
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::shutdown_or_cleanup_node()
{
  //Shutdown the base node
  if(!NodeCommon::shutdown())
  {
    // Even though this is an error, don't return a failure on shutdown as it is not a fatal error
    RCLCPP_ERROR(this->get_logger(), "Failed to shutdown base node");
  }

  return true;
}

void Microstrain::parse_and_publish_main_wrapper()
{
  // call the parsing function in a try catch block so we can transition the state instead of crashing when an error happens
  try
  {
    parseAndPublishMain();
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Error during main processing: %s", e.what());
    handle_exception();
  }
}

void Microstrain::parse_and_publish_aux_wrapper()
{
  // call the parsing function in a try catch block so we can transition the state instead of crashing when an error happens
  try
  {
    parseAndPublishAux();
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Error during aux processing: %s", e.what());
    handle_exception();
  }
}

void Microstrain::handle_exception()
{
  // Deactivate and shutdown
  deactivate_node();
  shutdown_or_cleanup_node();
}

} // namespace microstrain

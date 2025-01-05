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

#include "microstrain_inertial_driver/microstrain_inertial_driver_lifecycle.h"

namespace microstrain
{

MicrostrainLifecycle::MicrostrainLifecycle() : rclcpp_lifecycle::LifecycleNode("microstrain_inertial_driver_lifecycle_node")
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
// Configure State Callback
/////////////////////////////////////////////////////////////////////////////////////////////////////

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MicrostrainLifecycle::on_configure(const rclcpp_lifecycle::State &prev_state)
{
  //RCUTILS_LOG_INFO_NAMED(get_name(), "on_configure() is called.");
 
  if(configure_node())
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  else
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Activate State Callback
/////////////////////////////////////////////////////////////////////////////////////////////////////

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MicrostrainLifecycle::on_activate(const rclcpp_lifecycle::State &prev_state)
{
  //RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");

  if(activate_node())
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  else
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Deactivate State Callback
/////////////////////////////////////////////////////////////////////////////////////////////////////

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MicrostrainLifecycle::on_deactivate(const rclcpp_lifecycle::State &prev_state)
{
  //RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");

  if(deactivate_node())
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  else
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Cleanup State Callback
/////////////////////////////////////////////////////////////////////////////////////////////////////

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MicrostrainLifecycle::on_cleanup(const rclcpp_lifecycle::State &prev_state)
{
  //RCUTILS_LOG_INFO_NAMED(get_name(), "on_cleanup() is called.");

  if(shutdown_or_cleanup_node())
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  else
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
 // Shutdown State Callback
/////////////////////////////////////////////////////////////////////////////////////////////////////

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MicrostrainLifecycle::on_shutdown(const rclcpp_lifecycle::State &prev_state)
{
  //RCUTILS_LOG_INFO_NAMED(get_name(), "on_shutdown() is called.");

  if(shutdown_or_cleanup_node())
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  else
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
}




/////////////////////////////////////////////////////////////////////////////////////////////////////
// Configure Node Function
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainLifecycle::configure_node()
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

bool MicrostrainLifecycle::activate_node()
{
  // Activate the base node to start the background tasks
  if (!NodeCommon::activate())
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to activate base node");
    return false;
  }

  // Start a timer around a wrapper function to catch errors
  main_parsing_timer_ = createTimer<MicrostrainLifecycle>(node_, timer_update_rate_hz_, &MicrostrainLifecycle::parse_and_publish_main_wrapper, this);

  // Start the aux timer if we were requested to do so
  if (config_.aux_device_ != nullptr)
  {
    RCLCPP_INFO(this->get_logger(), "Starting aux port parsing");
    aux_parsing_timer_ = createTimer<MicrostrainLifecycle>(node_, 2.0, &MicrostrainLifecycle::parse_and_publish_aux_wrapper, this);
  }
  
  return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Deactivate Node Function
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool MicrostrainLifecycle::deactivate_node()
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

bool MicrostrainLifecycle::shutdown_or_cleanup_node()
{
  //Shutdown the base node
  if(!NodeCommon::shutdown())
  {
    // Even though this is an error, don't return a failure on shutdown as it is not a fatal error
    RCLCPP_ERROR(this->get_logger(), "Failed to shutdown base node");
  }

  return true;
}

void MicrostrainLifecycle::parse_and_publish_main_wrapper()
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

void MicrostrainLifecycle::parse_and_publish_aux_wrapper()
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

void MicrostrainLifecycle::handle_exception()
{
  // Manuallly transition to deactivate state so that the node can be cleanly restarted
  RCLCPP_INFO(this->get_logger(), "Transitioning to deactivate state");
  const auto& inactive_state = LifecycleNode::deactivate();
  if (inactive_state.label() == "inactive")
  {
    RCLCPP_WARN(this->get_logger(), "Successfully transitioned to inactive, cleaning up node to fresh state");
    const auto& cleanup_state = LifecycleNode::cleanup();
    if (cleanup_state.label() == "unconfigured")
    {
      RCLCPP_WARN(this->get_logger(), "Node has been successfully cleaned up from error. transition to configure state to reconfigure");
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Transition to cleanup resulted in transition to %s instead of inactive", cleanup_state.label().c_str());
      RCLCPP_ERROR(this->get_logger(), "Unable to recover, so transitioning to shutdown. This node is no longer usable");
      LifecycleNode::shutdown();
    }
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Transition to deactivate resulted in transition to %s instead of inactive", inactive_state.label().c_str());
    RCLCPP_ERROR(this->get_logger(), "Unable to recover, so transitioning to shutdown. This node is no longer usable");
    LifecycleNode::shutdown();
  }
}

} // namespace MicrostrainLifecycle

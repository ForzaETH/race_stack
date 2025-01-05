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

#include "microstrain_inertial_driver/microstrain_inertial_driver.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
   
  // Configure and activate the node
  auto node = std::make_shared<microstrain::Microstrain>();
  if (!node->configure_node())
  {
    RCLCPP_FATAL(node->get_logger(), "Failed to configure node");
    return 1;
  }
  if (!node->activate_node())
  {
    RCLCPP_FATAL(node->get_logger(), "Failed to activate node");
    return 2;
  }

  // Spin until we are shut down
  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(node->get_node_base_interface());
  exe.spin();

  // Shut down the node cleanly
  int status = 0;  // Success status. If we fail at any point this will be set to a positive number
  if (!node->deactivate_node())
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to deactivate node");
    status = 3;
  }
  if (!node->shutdown_or_cleanup_node())
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to shutdown node");
    status = 4;
  }

  // Shut down anything remaining using the ROS shutdown call
  rclcpp::shutdown();

  return status;
}

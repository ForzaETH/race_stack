/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Parker-Lord Driver Definition File
// 
// Copyright (c) 2017, Brian Bingham
// Copyright (c) 2021, Parker Hannifin Corp
// 
// This code is licensed under MIT license (see LICENSE file for details)
// 
/////////////////////////////////////////////////////////////////////////////////////////////////////


#ifndef _MICROSTRAIN_INERTIAL_DRIVER_MICROSTRAIN_INERTIAL_DRIVER_H
#define _MICROSTRAIN_INERTIAL_DRIVER_MICROSTRAIN_INERTIAL_DRIVER_H

#include <cstdio>
#include <unistd.h>
#include <time.h>
#include <iostream>
#include <fstream>
#include <functional>

#include "microstrain_inertial_driver_common/node_common.h"

namespace microstrain 
{

///
/// \brief Microstrain class
///
class Microstrain : public rclcpp::Node, public NodeCommon
{
 public:
  Microstrain();
  ~Microstrain() = default;

  bool configure_node();
  bool activate_node();
  bool deactivate_node();
  bool shutdown_or_cleanup_node();

  void parse_and_publish_main_wrapper();
  void parse_and_publish_aux_wrapper();

 private:

  template<typename Object, void (Object::*Callback)()>
  RosTimerType create_timer_wrapper(double rate_hz);

  void handle_exception();
}; //MicrostrainLifecycle class

template<typename Object, void (Object::*Callback)()>
RosTimerType Microstrain::create_timer_wrapper(double rate_hz)
{
#ifdef MICROSTRAIN_ROLLING
  return createTimer(std::chrono::duration<double, std::milli>(1 / rate_hz), std::bind(Callback, this));
#else
  return createTimer<Microstrain>(node_, rate_hz, Callback, this);
#endif
}

} // namespace microstrain

#endif  // _MICROSTRAIN_INERTIAL_DRIVER_MICROSTRAIN_INERTIAL_DRIVER_H

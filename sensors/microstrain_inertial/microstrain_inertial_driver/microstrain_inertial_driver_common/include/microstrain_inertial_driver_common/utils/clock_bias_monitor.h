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

#ifndef MICROSTRAIN_INERTIAL_DRIVER_COMMON_UTILS_CLOCK_BIAS_MONITOR_H
#define MICROSTRAIN_INERTIAL_DRIVER_COMMON_UTILS_CLOCK_BIAS_MONITOR_H

#include <vector>

#include "microstrain_inertial_driver_common/utils/ros_compat.h"

namespace microstrain
{

class ClockBiasMonitor
{
 public:
  /**
   * \brief Constructor
   * \param weight How much to weight the old bias estimate vs the new delta time. Closer to 1 means more weight on the old bias estimate
   * \param max_bias_estimate Max bias estimate before resetting to the current delta time. Helps prevents jumps and outliers
   * \param average_size Number of measurements to collect before computing an average and starting
  */
  explicit ClockBiasMonitor(const double weight = 0.5, const double max_bias_estimate = 1, const size_t source_time_average_size = 10);

  /**
   * \brief Adds a new time. The system time should be from as close as possible to the device time
   * \param source_time The source time you want to convert from
   * \param target_time The target time we will calculate the bias for
  */
  void addTime(const double source_time, const double target_time);

  /**
   * \brief Checks if the monitor has a bias estimate, this will be set to true, once addTime has been called once
   * \return Whether or not the monitor has a bias estimate
  */
  bool hasBiasEstimate() const;

  /**
   * \brief Gets the current bias estimate.
   * \return The current bias estimate
  */
  double getBiasEstimate() const;

  /**
   * \brief Resets the bias estimate
   */
  void reset();

 private:
  double weight_;  /// How much to weight the old bias estimate vs the new delta time. Closer to 1 means more weight on the old bias estimate
  double max_bias_estimate_;  /// Max bias estimate before resetting to the current delta time. Helps prevents jumps and outliers

  bool have_bias_estimate_ = false;  /// Will be set to true after getting the bias estimate for the first time
  double bias_estimate_ = 0.0;  /// Saved bias estimate. Can be used to convert a device time to a system time

  size_t delta_time_average_size_;  /// Number of source times we need to compute an average
  std::vector<double> delta_time_average_vector_;  /// Will be used to accumulate source times before computing an average
};

}  // namespace microstrain

#endif  // MICROSTRAIN_INERTIAL_DRIVER_COMMON_UTILS_CLOCK_BIAS_MONITOR_H

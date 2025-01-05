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
#ifndef MICROSTRAIN_INERTIAL_DRIVER_COMMON_SUBSCRIBERS_H
#define MICROSTRAIN_INERTIAL_DRIVER_COMMON_SUBSCRIBERS_H

#include <array>
#include <string>

#include "microstrain_inertial_driver_common/utils/ros_compat.h"
#include "microstrain_inertial_driver_common/utils/clock_bias_monitor.h"
#include "microstrain_inertial_driver_common/config.h"

#include "mip/definitions/commands_aiding.hpp"

namespace microstrain
{

static constexpr auto RTCM_TOPIC = "rtcm";

static constexpr auto EXT_TIME_TOPIC         = "ext/time";
static constexpr auto EXT_FIX_TOPIC          = "ext/llh_position";
static constexpr auto EXT_VEL_NED_TOPIC      = "ext/velocity_ned";
static constexpr auto EXT_VEL_ENU_TOPIC      = "ext/velocity_enu";
static constexpr auto EXT_VEL_ECEF_TOPIC     = "ext/velocity_ecef";
static constexpr auto EXT_VEL_BODY_TOPIC     = "ext/velocity_body";
static constexpr auto EXT_HEADING_NED_TOPIC  = "ext/heading_ned";
static constexpr auto EXT_HEADING_ENU_TOPIC  = "ext/heading_enu";
static constexpr auto EXT_MAG_TOPIC          = "ext/mag";
static constexpr auto EXT_PRESSURE_TOPIC     = "ext/pressure";

/**
 * Contains subscribers and the functions they call
 */
class Subscribers
{
public:
  /**
   * \brief Default Constructor
   */
  Subscribers() = default;

  /**
   * \brief Constructs this class with a reference to the node, and a config object
   * \param node  Reference to a node that will be saved to this class and used to log and interact with ROS
   * \param config Reference to the config object that will be saved to this class and used to determine whether or not to enable the subscriptions
   */
  Subscribers(RosNodeType* node, Config* config);

  /**
   * \brief Activates the subscribers. After this function is called, the subscriptions will be ready to receive messages
   * \return true if activation was successful and false if activation failed
   */
  bool activate();

  // External aiding measurement callbacks
  void externalTimeCallback(const TimeReferenceMsg& time);
  void externalGnssPositionCallback(const NavSatFixMsg& fix);
  void externalVelNedCallback(const TwistWithCovarianceStampedMsg& vel);
  void externalVelEnuCallback(const TwistWithCovarianceStampedMsg& vel);
  void externalVelEcefCallback(const TwistWithCovarianceStampedMsg& vel);
  void externalVelBodyCallback(const TwistWithCovarianceStampedMsg& vel);
  void externalHeadingNedCallback(const PoseWithCovarianceStampedMsg& heading);
  void externalHeadingEnuCallback(const PoseWithCovarianceStampedMsg& heading);
  void externalMagCallback(const MagneticFieldMsg& mag);
  void externalPressureCallback(const FluidPressureMsg& fluid_pressure);

  /**
   * \brief Accepts RTCM corrections from a ROS topic
   * \param rtcm Message containing 
   */
  void rtcmCallback(const RTCMMsg& rtcm);

  // External aiding measurement subscribers
  RosSubType<TimeReferenceMsg>::SharedPtr              external_time_sub_;
  RosSubType<NavSatFixMsg>::SharedPtr                  external_gnss_position_sub_;
  RosSubType<TwistWithCovarianceStampedMsg>::SharedPtr external_vel_ned_sub_;
  RosSubType<TwistWithCovarianceStampedMsg>::SharedPtr external_vel_enu_sub_;
  RosSubType<TwistWithCovarianceStampedMsg>::SharedPtr external_vel_ecef_sub_;
  RosSubType<TwistWithCovarianceStampedMsg>::SharedPtr external_vel_body_sub_;
  RosSubType<PoseWithCovarianceStampedMsg>::SharedPtr  external_heading_ned_sub_;
  RosSubType<PoseWithCovarianceStampedMsg>::SharedPtr  external_heading_enu_sub_;
  RosSubType<MagneticFieldMsg>::SharedPtr              external_mag_sub_;
  RosSubType<FluidPressureMsg>::SharedPtr              external_pressure_sub_;

  // RTCM subscriber
  RosSubType<RTCMMsg>::SharedPtr rtcm_sub_;

private:
  uint8_t getSensorIdFromFrameId(const std::string& frame_id);

  // Node Information
  RosNodeType* node_;
  Config* config_;

  // External frame IDs, the index of the string is the sensor ID
  uint16_t external_frame_ids_size_;
  std::array<std::string, 256> external_frame_ids_;

  // TF2 buffer lookup class
  TransformBufferType transform_buffer_;
  TransformListenerType transform_listener_;
};

}  // namespace microstrain

#endif  // MICROSTRAIN_INERTIAL_DRIVER_COMMON_SUBSCRIBERS_H

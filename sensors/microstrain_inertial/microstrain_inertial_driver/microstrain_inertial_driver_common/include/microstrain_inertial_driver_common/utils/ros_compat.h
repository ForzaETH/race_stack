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

#ifndef MICROSTRAIN_INERTIAL_DRIVER_COMMON_UTILS_ROS_COMPAT_H
#define MICROSTRAIN_INERTIAL_DRIVER_COMMON_UTILS_ROS_COMPAT_H

/**
 * Common Includes
 */
#include <string>
#include <memory>
#include <vector>

/**
 * Common Defines
 */
#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

// Version of the driver
#ifndef MICROSTRAIN_DRIVER_VERSION
#define MICROSTRAIN_DRIVER_VERSION "unknown"
#endif

namespace microstrain
{

constexpr auto GPS_LEAP_SECONDS = 18;

constexpr auto GNSS1_ID = 0;
constexpr auto GNSS2_ID = 1;
constexpr auto NUM_GNSS = 2;
};  // namespace microstrain

/**
 * ROS1 Includes
 */
#if MICROSTRAIN_ROS_VERSION == 1
#include "ros/ros.h"

#include "tf2_ros/buffer.h"
#include "tf2_ros/buffer_interface.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/TimeReference.h"
#include "sensor_msgs/FluidPressure.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/TransformStamped.h"
#include "sensor_msgs/MagneticField.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "rtcm_msgs/Message.h"
#include "nmea_msgs/Sentence.h"

#include "microstrain_inertial_msgs/HumanReadableStatus.h"

#include "microstrain_inertial_msgs/MipHeader.h"
#include "microstrain_inertial_msgs/MipSensorOverrangeStatus.h"
#include "microstrain_inertial_msgs/MipSensorTemperatureStatistics.h"
#include "microstrain_inertial_msgs/MipGnssFixInfo.h"
#include "microstrain_inertial_msgs/MipGnssSbasInfo.h"
#include "microstrain_inertial_msgs/MipGnssRfErrorDetection.h"
#include "microstrain_inertial_msgs/MipGnssCorrectionsRtkCorrectionsStatus.h"
#include "microstrain_inertial_msgs/MipFilterStatus.h"
#include "microstrain_inertial_msgs/MipFilterGnssPositionAidingStatus.h"
#include "microstrain_inertial_msgs/MipFilterMultiAntennaOffsetCorrection.h"
#include "microstrain_inertial_msgs/MipFilterAidingMeasurementSummary.h"
#include "microstrain_inertial_msgs/MipFilterGnssDualAntennaStatus.h"
#include "microstrain_inertial_msgs/MipSystemBuiltInTest.h"

#include "std_srvs/Empty.h"
#include "std_srvs/Trigger.h"

#include "microstrain_inertial_msgs/RawFileConfigRead.h"
#include "microstrain_inertial_msgs/RawFileConfigWrite.h"

#include "microstrain_inertial_msgs/MipBaseGetDeviceInformation.h"
#include "microstrain_inertial_msgs/Mip3dmCaptureGyroBias.h"
#include "microstrain_inertial_msgs/Mip3dmGpioStateRead.h"
#include "microstrain_inertial_msgs/Mip3dmGpioStateWrite.h"

/**
 * ROS2 Includes
 */
#elif MICROSTRAIN_ROS_VERSION == 2
#include "rclcpp/rclcpp.hpp"

#ifdef MICROSTRAIN_LIFECYCLE
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#endif

#include "tf2_ros/buffer.h"
#include "tf2_ros/buffer_interface.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

#include "lifecycle_msgs/msg/transition.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/time_reference.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/transform_stamped.h"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/multi_array_layout.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "rtcm_msgs/msg/message.hpp"
#include "nmea_msgs/msg/sentence.hpp"

#include "microstrain_inertial_msgs/msg/human_readable_status.hpp"

#include "microstrain_inertial_msgs/msg/mip_header.hpp"
#include "microstrain_inertial_msgs/msg/mip_sensor_overrange_status.hpp"
#include "microstrain_inertial_msgs/msg/mip_sensor_temperature_statistics.hpp"
#include "microstrain_inertial_msgs/msg/mip_gnss_fix_info.hpp"
#include "microstrain_inertial_msgs/msg/mip_gnss_sbas_info.hpp"
#include "microstrain_inertial_msgs/msg/mip_gnss_rf_error_detection.hpp"
#include "microstrain_inertial_msgs/msg/mip_gnss_corrections_rtk_corrections_status.hpp"
#include "microstrain_inertial_msgs/msg/mip_filter_status.hpp"
#include "microstrain_inertial_msgs/msg/mip_filter_gnss_position_aiding_status.hpp"
#include "microstrain_inertial_msgs/msg/mip_filter_multi_antenna_offset_correction.hpp"
#include "microstrain_inertial_msgs/msg/mip_filter_aiding_measurement_summary.hpp"
#include "microstrain_inertial_msgs/msg/mip_filter_gnss_dual_antenna_status.hpp"
#include "microstrain_inertial_msgs/msg/mip_system_built_in_test.hpp"

// .h header was deprecated in rolling and will likely be removed in future releases.
#if MICROSTRAIN_ROLLING == 1 || MICROSTRAIN_HUMBLE == 1
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#else
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#endif

#include "microstrain_inertial_msgs/srv/raw_file_config_read.hpp"
#include "microstrain_inertial_msgs/srv/raw_file_config_write.hpp"

#include "microstrain_inertial_msgs/srv/mip_base_get_device_information.hpp"
#include "microstrain_inertial_msgs/srv/mip3dm_capture_gyro_bias.hpp"
#include "microstrain_inertial_msgs/srv/mip3dm_gpio_state_read.hpp"
#include "microstrain_inertial_msgs/srv/mip3dm_gpio_state_write.hpp"
#else
#error "Unsupported ROS version. -DMICROSTRAIN_ROS_VERSION must be set to 1 or 2"
#endif


#if MICROSTRAIN_ROS_VERSION == 1
namespace tf2_ros
{
inline ::ros::Time fromMsg(const ::ros::Time& time)
{
  return time;
}
inline ::ros::Time toMsg(const ::ros::Time& time)
{
  return time;
}
}  // namespace tf2_ros
#endif

namespace microstrain
{
/**
 * ROS1 Defines
 */
#if MICROSTRAIN_ROS_VERSION == 1

/**
 * \brief Wrapper for a ROS1 publisher to make it look similar to a ROS2 publisher
 */
template<typename MessageType>
class RosPubType : public ::ros::Publisher
{
 public:
  using MessageSharedPtr = std::shared_ptr<MessageType>;
  using SharedPtr = std::shared_ptr<RosPubType<MessageType>>;

  explicit RosPubType(const ::ros::Publisher& rhs) : ::ros::Publisher(rhs) {}

  void on_activate() { (void)0; }
  void on_deactivate() { (void)0; }
};

template<typename MessageType>
class RosSubType : public ::ros::Subscriber
{
 public:
  using MessageSharedPtr = std::shared_ptr<MessageType>;
  using SharedPtr = std::shared_ptr<RosSubType<MessageType>>;

  explicit RosSubType(const ::ros::Subscriber& rhs) : ::ros::Subscriber(rhs) {}
};

/**
 * \brief Wrapper for a ROS1 service to make it look similar to a ROS2 service
 */
template<typename ServiceType>
class RosServiceType : public ::ros::ServiceServer
{
 public:
  using SharedPtr = std::shared_ptr<RosServiceType<ServiceType>>;

  explicit RosServiceType(const ::ros::ServiceServer& rhs) : ::ros::ServiceServer(rhs) {}
};

// ROS1 General Types
using RosNodeType = ::ros::NodeHandle;
using RosTimeType = ::ros::Time;
using RosDurationType = ::ros::Duration;
using RosTimerType = std::shared_ptr<::ros::Timer>;
using RosRateType = ::ros::Rate;
using RosHeaderType = ::std_msgs::Header;

// ROS1 Publisher Message Types
using OdometryMsg = ::nav_msgs::Odometry;
using ImuMsg = ::sensor_msgs::Imu;
using NavSatFixMsg = ::sensor_msgs::NavSatFix;
using FluidPressureMsg = ::sensor_msgs::FluidPressure;
using QuaternionMsg = ::geometry_msgs::Quaternion;
using TwistStampedMsg = ::geometry_msgs::TwistStamped;
using PoseWithCovarianceStampedMsg = ::geometry_msgs::PoseWithCovarianceStamped;
using TwistWithCovarianceStampedMsg = ::geometry_msgs::TwistWithCovarianceStamped;
using MagneticFieldMsg = ::sensor_msgs::MagneticField;
using TimeReferenceMsg = ::sensor_msgs::TimeReference;
using NMEASentenceMsg = ::nmea_msgs::Sentence;

using HumanReadableStatusMsg = ::microstrain_inertial_msgs::HumanReadableStatus;

using MipHeaderMsg = ::microstrain_inertial_msgs::MipHeader;
using MipSensorOverrangeStatusMsg = ::microstrain_inertial_msgs::MipSensorOverrangeStatus;
using MipSensorTemperatureStatisticsMsg = ::microstrain_inertial_msgs::MipSensorTemperatureStatistics;
using MipGnssFixInfoMsg = ::microstrain_inertial_msgs::MipGnssFixInfo;
using MipGnssSbasInfoMsg = ::microstrain_inertial_msgs::MipGnssSbasInfo;
using MipGnssRfErrorDetectionMsg = ::microstrain_inertial_msgs::MipGnssRfErrorDetection;
using MipGnssCorrectionsRtkCorrectionsStatusMsg = ::microstrain_inertial_msgs::MipGnssCorrectionsRtkCorrectionsStatus;
using MipFilterGnssPositionAidingStatusMsg = ::microstrain_inertial_msgs::MipFilterGnssPositionAidingStatus;
using MipFilterStatusMsg = ::microstrain_inertial_msgs::MipFilterStatus;
using MipFilterMultiAntennaOffsetCorrectionMsg = ::microstrain_inertial_msgs::MipFilterMultiAntennaOffsetCorrection;
using MipFilterAidingMeasurementSummaryMsg = ::microstrain_inertial_msgs::MipFilterAidingMeasurementSummary;
using MipFilterGnssDualAntennaStatusMsg = ::microstrain_inertial_msgs::MipFilterGnssDualAntennaStatus;
using MipSystemBuiltInTestMsg = ::microstrain_inertial_msgs::MipSystemBuiltInTest;

using TransformStampedMsg = ::geometry_msgs::TransformStamped;

// ROS1 TF types
using TransformBufferType = std::shared_ptr<::tf2_ros::Buffer>;
using TransformListenerType = std::shared_ptr<::tf2_ros::TransformListener>;
using StaticTransformBroadcasterType = std::shared_ptr<::tf2_ros::StaticTransformBroadcaster>;
using TransformBroadcasterType = std::shared_ptr<::tf2_ros::TransformBroadcaster>;

// ROS1 Subscriber Message Types
using BoolMsg = ::std_msgs::Bool;
using TimeReferenceMsg = ::sensor_msgs::TimeReference;
using RTCMMsg = ::rtcm_msgs::Message;

// ROS1 Service Message Types
using TriggerSrv = std_srvs::Trigger;
using EmptySrv = std_srvs::Empty;

using RawFileConfigReadSrv = ::microstrain_inertial_msgs::RawFileConfigRead;
using RawFileConfigWriteSrv = ::microstrain_inertial_msgs::RawFileConfigWrite;

using MipBaseGetDeviceInformationSrv = ::microstrain_inertial_msgs::MipBaseGetDeviceInformation;
using Mip3dmCaptureGyroBiasSrv = ::microstrain_inertial_msgs::Mip3dmCaptureGyroBias;
using Mip3dmGpioStateReadSrv = microstrain_inertial_msgs::Mip3dmGpioStateRead;
using Mip3dmGpioStateWriteSrv = microstrain_inertial_msgs::Mip3dmGpioStateWrite;

// ROS1 aliases not intended to be used outside this file
using ParamIntVector = std::vector<int32_t>;

// ROS1 Logging
#define MICROSTRAIN_DEBUG(NODE, ...) ROS_DEBUG(__VA_ARGS__)
#define MICROSTRAIN_INFO(NODE, ...) ROS_INFO(__VA_ARGS__)
#define MICROSTRAIN_WARN(NODE, ...) ROS_WARN(__VA_ARGS__)
#define MICROSTRAIN_ERROR(NODE, ...) ROS_ERROR(__VA_ARGS__)
#define MICROSTRAIN_FATAL(NOE, ...) ROS_FATAL(__VA_ARGS__)

#define MICROSTRAIN_DEBUG_THROTTLE(NODE, PERIOD, ...) ROS_DEBUG_THROTTLE(PERIOD, __VA_ARGS__)
#define MICROSTRAIN_INFO_THROTTLE(NODE, PERIOD, ...) ROS_INFO_THROTTLE(PERIOD, __VA_ARGS__)
#define MICROSTRAIN_WARN_THROTTLE(NODE, PERIOD, ...) ROS_WARN_THROTTLE(PERIOD, __VA_ARGS__)
#define MICROSTRAIN_ERROR_THROTTLE(NODE, PERIOD, ...) ROS_ERROR_THROTTLE(PERIOD, __VA_ARGS__)
#define MICROSTRAIN_FATAL_THROTTLE(NODE, PERIOD, ...) ROS_FATAL_THROTTLE(PERIOD, __VA_ARGS__)

#define MICROSTRAIN_DEBUG_ONCE(NODE, ...) ROS_DEBUG_ONCE(__VA_ARGS__)
#define MICROSTRAIN_INFO_ONCE(NODE, ...) ROS_INFO_ONCE(__VA_ARGS__)
#define MICROSTRAIN_WARN_ONCE(NODE, ...) ROS_WARN_ONCE(__VA_ARGS__)
#define MICROSTRAIN_ERROR_ONCE(NODE, ...) ROS_ERROR_ONCE(__VA_ARGS__)
#define MICROSTRAIN_FATAL_ONCE(NOE, ...) ROS_FATAL_ONCE(__VA_ARGS__)

// ROS1 functions

/**
 * brief Checks whether the node is still running
 * \return Whether the node is still running
 */
inline bool rosOk()
{
  return ros::ok();
}

/**
 * \brief Gets the current ROS time
 * \param node  Unused in this function as the ros time function is static
 * \return Current ROS time
 */
inline RosTimeType rosTimeNow(RosNodeType* node)
{
  return ros::Time::now();
}

/**
 * \brief Sets the time in seconds and nanoseconds to a ROS time object
 * \param time The time object to set the time on
 * \param sec Number of seconds to set on the object
 * \param nsec Number of nanoseconds to set on the object
 */
inline void setRosTime(RosTimeType* time, int32_t sec, int32_t nsec)
{
  time->sec = sec;
  time->nsec = nsec;
}

/**
 * \brief Gets the seconds from a ROS time object because the interface changed between ROS1 and ROS2
 * \param time_ref  The ros Time object to extract the seconds from
 * \return seconds from the ros time object
 */
inline int64_t getTimeRefSec(const ros::Time& time_ref)
{
  return time_ref.sec;
}

/**
 * \brief Gets the seconds and nanoseconds converted to seconds as a double
 * \param time_ref  The ros time object to extract the time from
 * \return seconds combined with nanoseconds from the ros time object
*/
inline double getTimeRefSecs(const ros::Time& time_ref)
{
  return static_cast<double>(time_ref.sec) + static_cast<double>(time_ref.nsec) / 1000000000.0;
}


/**
 * \brief Sets the sequence number on a ROS header. This is only useful in ROS1 as ROS2 removed the seq member
 * \param header  The header to set the sequence number on
 * \param seq  The sequence number to set on the header
 */
inline void setSeq(RosHeaderType* header, const uint32_t seq)
{
  header->seq = seq;
}

/**
 * \brief Extracts config from the ROS node.
 * \tparam ConfigType  The type to extract the config to e.g. int, std::string, double, etc.
 * \param node  The ROS node to extract the config from
 * \param param_name  The name of the config value to extract
 * \param param_val  Variable to store the extracted config value in
 * \param default_val  The default value to set param_val to if the config can't be found
 */
template <class ConfigType>
void getParam(RosNodeType* node, const std::string& param_name, ConfigType& param_val, const ConfigType& default_val)
{
  node->param<ConfigType>(param_name, param_val, default_val);
}

template <class ConfigType>
void setParam(RosNodeType* node, const std::string& param_name, const ConfigType& param_val)
{
  node->setParam(param_name, param_val);
}

inline TransformBufferType createTransformBuffer(RosNodeType* node)
{
  return std::make_shared<tf2_ros::Buffer>();
}

inline TransformListenerType createTransformListener(TransformBufferType buffer)
{
  return std::make_shared<tf2_ros::TransformListener>(*buffer);
}

/**
 * \brief Creates a static transform broadcaster
 * \param node The ROS node that the broadcaster will be associated with
 * \return Initialized shared pointer containing a transdorm broadcaster
 */
inline StaticTransformBroadcasterType createStaticTransformBroadcaster(RosNodeType* node)
{
  return std::make_shared<tf2_ros::StaticTransformBroadcaster>();
}

/**
 * \brief Creates a transform broadcaster
 * \param node The ROS node that the broadcaster will be associated with
 * \return Initialized shared pointer containing a transdorm broadcaster
 */
inline TransformBroadcasterType createTransformBroadcaster(RosNodeType* node)
{
  return std::make_shared<tf2_ros::TransformBroadcaster>();
}

/**
 * \brief Creates a ROS publisher
 * \tparam MessageType  The type of message that this publisher will publish
 * \param node  The ROS node to create the publisher on
 * \param topic  The topic that this publisher will publish on
 * \param queue_size  The size of the queue to enable in ROS
 * \return Shared Pointer containing the initialized publisher
 */
template <class MessageType>
typename RosPubType<MessageType>::SharedPtr createPublisher(RosNodeType* node, const std::string& topic,
                                                   const uint32_t queue_size)
{
  return std::make_shared<RosPubType<MessageType>>(node->template advertise<MessageType>(topic, queue_size));
}

/**
 * \brief Creates a ROS subscriber
 * \tparam MessageType  The type of message that this subscriber will listen to
 * \tparam ClassType  The type of class that the member function passed to this function will be on
 * \param node  The ROS node to create the publisher on
 * \param topic  The topic that this publisher will subscribe on
 * \param queue_size  The size of the queue to enable on ROS
 * \param fp  Function pointer to call whenever a message is received on the topic
 * \param obj  Reference to an object of type ClassType that will be passed as the this pointer to fp
 * \return Shared Pointer containing the initialized subscriber
 */
template <class MessageType, class ClassType>
typename RosSubType<MessageType>::SharedPtr createSubscriber(RosNodeType* node, const std::string& topic,
                                                     const uint32_t queue_size,
                                                     void (ClassType::*fp)(const MessageType&), ClassType* obj)
{
  return std::make_shared<RosSubType<MessageType>>(node->template subscribe(topic.c_str(), queue_size, fp, obj));
}

/**
 * \brief Creates a ROS Service
 * \tparam MessageType  The type of message that this service will use
 * \tparam ClassType The type of class that the callback will be registered to
 * \tparam RequestType  The type of request that this function will receive
 * \tparam ResponseType  The type of response that this function will respond with
 * \param node  The ROS node to create the service on
 * \param service  The name to give the created service
 * \param srv_func  Function pointer to a function that will be called when the service receives a message
 * \param obj  Reference to an object of type ClassType that will be used to call the callback
 * \return Shared Pointer containing the initialized service
 */
template <class MessageType, class ClassType, class RequestType, class ResponseType>
typename RosServiceType<MessageType>::SharedPtr createService(RosNodeType* node, const std::string& service,
                                                     bool (ClassType::*srv_func)(RequestType&, ResponseType&),
                                                     ClassType* obj)
{
  return std::make_shared<RosServiceType<MessageType>>(
      node->template advertiseService<ClassType, RequestType, ResponseType>(service, srv_func, obj));
}

/**
 * \brief Creates and starts a ROS timer
 * \tparam ClassType  The type of class that the callback will be registered to
 * \param node  The ROS node to create the timer on
 * \param hz  Rate in hertz to execute the callback on
 * \param fp  Function pointer to execute at the specified rate
 * \param obj  Reference to an object of type Class Type that will be used to call the callback
 * \return Shard pointer containing the initialized and started timer
 */
template <class ClassType>
RosTimerType createTimer(RosNodeType* node, double hz, void (ClassType::*fp)(), ClassType* obj)
{
  return std::make_shared<::ros::Timer>(
      node->template createTimer(ros::Duration(1.0 / hz), [=](const ros::TimerEvent& event) { (obj->*fp)(); }));
}

/**
 * \brief Stops a ROS timer
 * \param timer  The timer to stop
 */
inline void stopTimer(RosTimerType timer)
{
  timer->stop();
}

/**
 * ROS2 Defines
 */
#elif MICROSTRAIN_ROS_VERSION == 2
// ROS2 Generic Types
#ifdef MICROSTRAIN_LIFECYCLE
using RosNodeType = ::rclcpp_lifecycle::LifecycleNode;
#else
using RosNodeType = ::rclcpp::Node;
#endif
using RosTimeType = ::builtin_interfaces::msg::Time;
using RosDurationType = ::rclcpp::Duration;
using RosTimerType = ::rclcpp::TimerBase::SharedPtr;
using RosRateType = ::rclcpp::Rate;
using RosHeaderType = ::std_msgs::msg::Header;

template<typename MessageType>
#ifdef MICROSTRAIN_LIFECYCLE
using RosBasePubType = ::rclcpp_lifecycle::LifecyclePublisher<MessageType>;
#else
using RosBasePubType = ::rclcpp::Publisher<MessageType>;
#endif

/**
 * \brief Wrapper to allow the publisher from ROS2 be compatible with ROS1
 *        This could almost be just "using", but the "MessageSharedPtr" of the base class is constant, and we need it to not be constant
 */
template<typename MessageType>
class RosPubType : public RosBasePubType<MessageType>
{
 public:
  using MessageSharedPtr = std::shared_ptr<MessageType>;

  explicit RosPubType(const RosBasePubType<MessageType>& rhs) : RosBasePubType<MessageType>(rhs) {}

  // Compatibility for non lifecycle node
#ifndef MICROSTRAIN_LIFECYCLE
  using SharedPtr = std::shared_ptr<RosPubType<MessageType>>;
  void on_activate() {}  // NOOP
  void on_deactivate() {}  // NOOP
#endif
};

template<typename MessageType>
using RosSubType = ::rclcpp::Subscription<MessageType>;

// Alias for the service type so it can be compatible with ROS1
template<typename ServiceType>
using RosServiceType = ::rclcpp::Service<ServiceType>;

// ROS2 Publisher Message Types
using OdometryMsg = ::nav_msgs::msg::Odometry;
using ImuMsg = ::sensor_msgs::msg::Imu;
using NavSatFixMsg = ::sensor_msgs::msg::NavSatFix;
using FluidPressureMsg = ::sensor_msgs::msg::FluidPressure;
using QuaternionMsg = ::geometry_msgs::msg::Quaternion;
using TwistStampedMsg = ::geometry_msgs::msg::TwistStamped;
using PoseWithCovarianceStampedMsg = ::geometry_msgs::msg::PoseWithCovarianceStamped;
using TwistWithCovarianceStampedMsg = ::geometry_msgs::msg::TwistWithCovarianceStamped;
using MagneticFieldMsg = ::sensor_msgs::msg::MagneticField;
using TimeReferenceMsg = ::sensor_msgs::msg::TimeReference;
using NMEASentenceMsg = ::nmea_msgs::msg::Sentence;

using HumanReadableStatusMsg = ::microstrain_inertial_msgs::msg::HumanReadableStatus;

using MipHeaderMsg = ::microstrain_inertial_msgs::msg::MipHeader;
using MipSensorOverrangeStatusMsg = ::microstrain_inertial_msgs::msg::MipSensorOverrangeStatus;
using MipSensorTemperatureStatisticsMsg = ::microstrain_inertial_msgs::msg::MipSensorTemperatureStatistics;
using MipGnssFixInfoMsg = ::microstrain_inertial_msgs::msg::MipGnssFixInfo;
using MipGnssSbasInfoMsg = ::microstrain_inertial_msgs::msg::MipGnssSbasInfo;
using MipGnssRfErrorDetectionMsg = ::microstrain_inertial_msgs::msg::MipGnssRfErrorDetection;
using MipGnssCorrectionsRtkCorrectionsStatusMsg = ::microstrain_inertial_msgs::msg::MipGnssCorrectionsRtkCorrectionsStatus;
using MipFilterStatusMsg = ::microstrain_inertial_msgs::msg::MipFilterStatus;
using MipFilterGnssPositionAidingStatusMsg = ::microstrain_inertial_msgs::msg::MipFilterGnssPositionAidingStatus;
using MipFilterMultiAntennaOffsetCorrectionMsg = ::microstrain_inertial_msgs::msg::MipFilterMultiAntennaOffsetCorrection;
using MipFilterAidingMeasurementSummaryMsg = ::microstrain_inertial_msgs::msg::MipFilterAidingMeasurementSummary;
using MipFilterGnssDualAntennaStatusMsg = ::microstrain_inertial_msgs::msg::MipFilterGnssDualAntennaStatus;
using MipSystemBuiltInTestMsg = ::microstrain_inertial_msgs::msg::MipSystemBuiltInTest;

using TransformStampedMsg = ::geometry_msgs::msg::TransformStamped;

// ROS2 Transform Broadcaster
using TransformBufferType = std::shared_ptr<::tf2_ros::Buffer>;
using TransformListenerType = std::shared_ptr<::tf2_ros::TransformListener>;
using StaticTransformBroadcasterType = std::shared_ptr<::tf2_ros::StaticTransformBroadcaster>;
using TransformBroadcasterType = std::shared_ptr<::tf2_ros::TransformBroadcaster>;

// ROS2 Subscriber Message Types
using BoolMsg = ::std_msgs::msg::Bool;
using TimeReferenceMsg = ::sensor_msgs::msg::TimeReference;
using RTCMMsg = ::rtcm_msgs::msg::Message;

// ROS2 Service Message Types
using TriggerSrv = std_srvs::srv::Trigger;
using EmptySrv = std_srvs::srv::Empty;

using RawFileConfigReadSrv = microstrain_inertial_msgs::srv::RawFileConfigRead;
using RawFileConfigWriteSrv = microstrain_inertial_msgs::srv::RawFileConfigWrite;

using MipBaseGetDeviceInformationSrv = microstrain_inertial_msgs::srv::MipBaseGetDeviceInformation;
using Mip3dmCaptureGyroBiasSrv = microstrain_inertial_msgs::srv::Mip3dmCaptureGyroBias;
using Mip3dmGpioStateReadSrv = microstrain_inertial_msgs::srv::Mip3dmGpioStateRead;
using Mip3dmGpioStateWriteSrv = microstrain_inertial_msgs::srv::Mip3dmGpioStateWrite;

// ROS2 aliases not intended to be used outside this file
using ParamIntVector = std::vector<int64_t>;

// ROS2 Logging
#define MICROSTRAIN_DEBUG(NODE, ...) RCLCPP_DEBUG(NODE->get_logger(), __VA_ARGS__)
#define MICROSTRAIN_INFO(NODE, ...) RCLCPP_INFO(NODE->get_logger(), __VA_ARGS__)
#define MICROSTRAIN_WARN(NODE, ...) RCLCPP_WARN(NODE->get_logger(), __VA_ARGS__)
#define MICROSTRAIN_ERROR(NODE, ...) RCLCPP_ERROR(NODE->get_logger(), __VA_ARGS__)
#define MICROSTRAIN_FATAL(NODE, ...) RCLCPP_FATAL(NODE->get_logger(), __VA_ARGS__)

#define MICROSTRAIN_DEBUG_THROTTLE(NODE, PERIOD, ...)                                                                  \
  RCLCPP_DEBUG_THROTTLE(NODE->get_logger(), *NODE->get_clock(), PERIOD, __VA_ARGS__)
#define MICROSTRAIN_INFO_THROTTLE(NODE, PERIOD, ...)                                                                  \
  RCLCPP_INFO_THROTTLE(NODE->get_logger(), *NODE->get_clock(), PERIOD, __VA_ARGS__)
#define MICROSTRAIN_WARN_THROTTLE(NODE, PERIOD, ...)                                                                  \
  RCLCPP_WARN_THROTTLE(NODE->get_logger(), *NODE->get_clock(), PERIOD, __VA_ARGS__)
#define MICROSTRAIN_ERROR_THROTTLE(NODE, PERIOD, ...)                                                                  \
  RCLCPP_ERROR_THROTTLE(NODE->get_logger(), *NODE->get_clock(), PERIOD, __VA_ARGS__)
#define MICROSTRAIN_FATAL_THROTTLE(NODE, PERIOD, ...)                                                                  \
  RCLCPP_FATAL_THROTTLE(NODE->get_logger(), *NODE->get_clock(), PERIOD, __VA_ARGS__)

#define MICROSTRAIN_DEBUG_ONCE(NODE, ...) RCLCPP_DEBUG_ONCE(NODE->get_logger(), __VA_ARGS__)
#define MICROSTRAIN_INFO_ONCE(NODE, ...) RCLCPP_INFO_ONCE(NODE->get_logger(), __VA_ARGS__)
#define MICROSTRAIN_WARN_ONCE(NODE, ...) RCLCPP_WARN_ONCE(NODE->get_logger(), __VA_ARGS__)
#define MICROSTRAIN_ERROR_ONCE(NODE, ...) RCLCPP_ERROR_ONCE(NODE->get_logger(), __VA_ARGS__)
#define MICROSTRAIN_FATAL_ONCE(NODE, ...) RCLCPP_FATAL_ONCE(NODE->get_logger(), __VA_ARGS__)

// ROS2 functions

/**
 * brief Checks whether the node is still running
 * \return Whether the node is still running
 */
inline bool rosOk()
{
  return rclcpp::ok();
}

/**
 * \brief Gets the current ROS time
 * \param node  Unused in this function as the ros time function is static
 * \return Current ROS time
 */
inline RosTimeType rosTimeNow(RosNodeType* node)
{
  return node->get_clock()->now();
}

/**
 * \brief Sets the time in seconds and nanoseconds to a ROS time object
 * \param time The time object to set the time on
 * \param sec Number of seconds to set on the object
 * \param nsec Number of nanoseconds to set on the object
 */
inline void setRosTime(builtin_interfaces::msg::Time* time, int32_t sec, int32_t nsec)
{
  time->sec = sec;
  time->nanosec = nsec;
}

/**
 * \brief Gets the seconds from a ROS time object because the interface changed between ROS1 and ROS2
 * \param time_ref  The ros Time object to extract the seconds from
 * \return seconds from the ros time object
 */
inline int64_t getTimeRefSec(const builtin_interfaces::msg::Time& time_ref)
{
  return time_ref.sec;
}

/**
 * \brief Gets the seconds and nanoseconds converted to seconds as a double
 * \param time_ref  The ros time object to extract the time from
 * \return seconds combined with nanoseconds from the ros time object
*/
inline double getTimeRefSecs(const builtin_interfaces::msg::Time& time_ref)
{
  return static_cast<double>(time_ref.sec) + static_cast<double>(time_ref.nanosec) / 1000000000.0;
}

/**
 * \brief Sets the sequence number on a ROS header. This is only useful in ROS1 as ROS2 removed the seq member
 * \param header  The header to set the sequence number on
 * \param seq  The sequence number to set on the header
 */
inline void setSeq(RosHeaderType* header, const uint32_t seq)
{
  // NOOP because seq was removed in ROS2
}

/**
 * \brief Extracts config from the ROS node.
 * \tparam ConfigType  The type to extract the config to e.g. int, std::string, double, etc.
 * \param node  The ROS node to extract the config from
 * \param param_name  The name of the config value to extract
 * \param param_val  Variable to store the extracted config value in
 * \param default_val  The default value to set param_val to if the config can't be found
 */
template <class ConfigType>
void getParam(RosNodeType* node, const std::string& param_name, ConfigType& param_val, const ConfigType& default_val)
{
  if (node->has_parameter(param_name))
  {
    node->get_parameter_or<ConfigType>(param_name, param_val, default_val);
  }
  else
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
#if not defined MICROSTRAIN_FOXY
    descriptor.dynamic_typing = true;
#endif
    param_val = node->declare_parameter(param_name, rclcpp::ParameterValue{default_val}, descriptor).get<ConfigType>();
  }
}

template <class ConfigType>
void setParam(RosNodeType* node, const std::string& param_name, const ConfigType& param_val)
{
  if (!node->has_parameter(param_name))
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
#if not defined MICROSTRAIN_FOXY
    descriptor.dynamic_typing = true;
#endif
    node->declare_parameter(param_name, rclcpp::ParameterValue{}, descriptor);
  }

  node->set_parameter(rclcpp::Parameter(param_name, param_val));
}

inline TransformBufferType createTransformBuffer(RosNodeType* node)
{
  return std::make_shared<tf2_ros::Buffer>(node->get_clock());
}

inline TransformListenerType createTransformListener(TransformBufferType buffer)
{
  return std::make_shared<tf2_ros::TransformListener>(*buffer);
}

/**
 * \brief Creates a static transform broadcaster
 * \param node The ROS node that the broadcaster will be associated with
 * \return Initialized shared pointer containing a transdorm broadcaster
 */
inline StaticTransformBroadcasterType createStaticTransformBroadcaster(RosNodeType* node)
{
  return std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);
}

/**
 * \brief Creates a transform broadcaster
 * \param node The ROS node that the broadcaster will be associated with
 * \return Initialized shared pointer containing a transdorm broadcaster
 */
inline TransformBroadcasterType createTransformBroadcaster(RosNodeType* node)
{
  return std::make_shared<tf2_ros::TransformBroadcaster>(node);
}

/**
 * \brief Creates a ROS publisher
 * \tparam MessageType  The type of message that this publisher will publish
 * \param node  The ROS node to create the publisher on
 * \param topic  The topic that this publisher will publish on
 * \param queue_size  The size of the queue to enable in ROS
 * \return Shared Pointer containing the initialized publisher
 */
#ifdef MICROSTRAIN_LIFECYCLE
template <class MessageType>
typename ::rclcpp_lifecycle::LifecyclePublisher<MessageType>::SharedPtr createPublisher(RosNodeType* node,
                                                                                       const std::string& topic,
                                                                                       const uint32_t qos)
{
  return node->template create_publisher<MessageType>(topic, qos);
}
#else
template <class MessageType>
typename RosPubType<MessageType>::SharedPtr createPublisher(RosNodeType* node,
                                                            const std::string& topic,
                                                            const uint32_t qos)
{
  auto base_pub = node->template create_publisher<MessageType>(topic, qos);
  return std::make_shared<RosPubType<MessageType>>(*base_pub);
}
#endif

/**
 * \brief Creates a ROS subscriber
 * \tparam MessageType  The type of message that this subscriber will listen to
 * \tparam ClassType  The type of class that the member function passed to this function will be on
 * \param node  The ROS node to create the publisher on
 * \param topic  The topic that this publisher will subscribe on
 * \param queue_size  The size of the queue to enable on ROS
 * \param fp  Function pointer to call whenever a message is received on the topic
 * \param obj  Reference to an object of type ClassType that will be passed as the this pointer to fp
 * \return Shared Pointer containing the initialized subscriber
 */
template <class MessageType, class ClassType>
typename ::rclcpp::Subscription<MessageType>::SharedPtr createSubscriber(RosNodeType* node, const std::string& topic,
                                                                          const uint32_t qos,
                                                                          void (ClassType::*fp)(const MessageType&),
                                                                          ClassType* obj)
{
  return node->template create_subscription<MessageType>(
      topic, qos, [obj, fp](const std::shared_ptr<MessageType> req) { (obj->*fp)(*req); });
}

/**
 * \brief Creates a ROS Service
 * \tparam MessageType  The type of message that this service will use
 * \tparam ClassType The type of class that the callback will be registered to
 * \tparam RequestType  The type of request that this function will receive
 * \tparam ResponseType  The type of response that this function will respond with
 * \param node  The ROS node to create the service on
 * \param service  The name to give the created service
 * \param srv_func  Function pointer to a function that will be called when the service receives a message
 * \param obj  Reference to an object of type ClassType that will be used to call the callback
 * \return Shared Pointer containing the initialized service
 */
template <class MessageType, class ClassType, class RequestType, class ResponseType>
typename ::rclcpp::Service<MessageType>::SharedPtr
createService(RosNodeType* node, const std::string& service, bool (ClassType::*srv_func)(RequestType&, ResponseType&),
               ClassType* obj)
{
  return node->template create_service<MessageType>(
      service, [obj, srv_func](const std::shared_ptr<RequestType> req, std::shared_ptr<ResponseType> res)
      {
        (obj->*srv_func)(*req, *res);
      }
    );  // NOLINT(whitespace/parens)  No way to avoid this
}

/**
 * \brief Creates and starts a ROS timer
 * \tparam ClassType  The type of class that the callback will be registered to
 * \param node  The ROS node to create the timer on
 * \param hz  Rate in hertz to execute the callback on
 * \param fp  Function pointer to execute at the specified rate
 * \param obj  Reference to an object of type Class Type that will be used to call the callback
 * \return Shard pointer containing the initialized and started timer
 */
template <class ClassType>
RosTimerType createTimer(RosNodeType* node, double hz, void (ClassType::*fp)(), ClassType* obj)
{
  std::chrono::microseconds timer_interval_us(static_cast<int>(1.0 / hz * 1000000.0));
  return node->template create_wall_timer(timer_interval_us, [=]() { (obj->*fp)(); });
}

/**
 * \brief Stops a ROS timer
 * \param timer  The timer to stop
 */
inline void stopTimer(RosTimerType timer)
{
  timer->cancel();
}

#else
#error "Unsupported ROS version. -DMICROSTRAIN_ROS_VERSION must be set to 1 or 2"
#endif

/**
 * \brief Extention of getParam. Explicitly gets float parameter, even if it was specified as an int
 * \param node  The ROS node to extract the config from
 * \param param_name  The name of the config value to extract
 * \param param_val  Variable to store the extracted config value in
 * \param default_val  The default value to set param_val to if the config can't be found
 */
inline void getParamFloat(RosNodeType* node, const std::string& param_name, float& param_val, const float default_val)
{
  // Seems like ROS should be able to figure this out, but for ROS2 at least, we need to cast ints to floats so people don't have to put decimal points
  try
  {
    getParam<float>(node, param_name, param_val, default_val);
  }
  catch (const std::exception& e)
  {
    int32_t param_val_int;
    getParam<int32_t>(node, param_name, param_val_int, static_cast<int32_t>(default_val));
    param_val = static_cast<float>(param_val_int);
  }
}

/**
 * \brief Extension of getParam. Explicitly gets the array as an array of uint16 values
 * \param node  The ROS node to extract the config from
 * \param param_name  The name of the config value to extract
 * \param param_val  Variable to store the extracted config value in
 * \param default_val  The default value to set param_val to if the config can't be found
*/
inline void getUint16ArrayParam(RosNodeType* node, const std::string& param_name, std::vector<uint16_t>& param_val, const std::vector<uint16_t>& default_val)
{
  // Get the parameter as ints since that is all ROS supports
  ParamIntVector param_val_int;
  ParamIntVector default_val_int(default_val.begin(), default_val.end());
  getParam<ParamIntVector>(node, param_name, param_val_int, default_val_int);

  // Convert the type
  param_val = std::vector<uint16_t>(param_val_int.begin(), param_val_int.end());
}

inline void setRosTime(RosTimeType* time_ref, double time)
{
  // Split the time into seconds and subseconds
  double seconds;
  double subseconds = std::modf(time, &seconds);

  // Set the ros time
  if (time_ref != nullptr)
    setRosTime(time_ref, static_cast<int64_t>(seconds), static_cast<int64_t>(std::floor(subseconds * 1000000000)));
}

inline bool isNed(const RosHeaderType& header)
{
  const std::string& ned_suffix = "_ned";
  if (header.frame_id.length() > ned_suffix.length())
    return (0 == header.frame_id.compare(header.frame_id.length() - ned_suffix.length(), ned_suffix.length(), ned_suffix));
  else
    return false;
}

}  // namespace microstrain

#endif  // MICROSTRAIN_INERTIAL_DRIVER_COMMON_UTILS_ROS_COMPAT_H

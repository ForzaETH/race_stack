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

#ifndef MICROSTRAIN_INERTIAL_DRIVER_COMMON_UTILS_MAPPINGS_MIP_PUBLISHER_MAPPING_H_
#define MICROSTRAIN_INERTIAL_DRIVER_COMMON_UTILS_MAPPINGS_MIP_PUBLISHER_MAPPING_H_

#include <map>
#include <vector>
#include <string>
#include <memory>

#include "mip/mip_all.hpp"

#include "microstrain_inertial_driver_common/utils/mappings/mip_mapping.h"
#include "microstrain_inertial_driver_common/utils/ros_compat.h"
#include "microstrain_inertial_driver_common/utils/mip/ros_mip_device_main.h"

namespace microstrain
{

// Topic names
static constexpr auto IMU_DATA_RAW_TOPIC = "imu/data_raw";
static constexpr auto IMU_DATA_TOPIC = "imu/data";
static constexpr auto IMU_MAG_TOPIC = "imu/mag";
static constexpr auto IMU_PRESSURE_TOPIC = "imu/pressure";
static constexpr auto IMU_WHEEL_SPEED_TOPIC = "imu/wheel_speed";

static constexpr auto GNSS1_LLH_POSITION_TOPIC = "gnss_1/llh_position";
static constexpr auto GNSS1_VELOCITY_TOPIC = "gnss_1/velocity";
static constexpr auto GNSS1_VELOCITY_ECEF_TOPIC = "gnss_1/velocity_ecef";
static constexpr auto GNSS1_ODOMETRY_TOPIC = "gnss_1/odometry_earth";
static constexpr auto GNSS1_TIME_REF_TOPIC = "gnss_1/time";

static constexpr auto GNSS2_FIX_TOPIC = "gnss_2/llh_position";
static constexpr auto GNSS2_VELOCITY_TOPIC = "gnss_2/velocity";
static constexpr auto GNSS2_VELOCITY_ECEF_TOPIC = "gnss_2/velocity_ecef";
static constexpr auto GNSS2_ODOMETRY_TOPIC = "gnss_2/odometry_earth";
static constexpr auto GNSS2_TIME_REF_TOPIC = "gnss_2/time";

static constexpr auto FILTER_HUMAN_READABLE_STATUS_TOPIC = "ekf/status";
static constexpr auto FILTER_IMU_DATA_TOPIC = "ekf/imu/data";
static constexpr auto FILTER_LLH_POSITION_TOPIC = "ekf/llh_position";
static constexpr auto FILTER_VELOCITY_TOPIC = "ekf/velocity";
static constexpr auto FILTER_VELOCITY_ECEF_TOPIC = "ekf/velocity_ecef";
static constexpr auto FILTER_ODOMETRY_EARTH_TOPIC  = "ekf/odometry_earth";
static constexpr auto FILTER_ODOMETRY_MAP_TOPIC = "ekf/odometry_map";
static constexpr auto FILTER_DUAL_ANTENNA_HEADING_TOPIC = "ekf/dual_antenna_heading";

static constexpr auto MIP_SENSOR_TEMPERATURE_STATISTICS_TOPIC = "mip/sensor/temperature_statistics";
static constexpr auto MIP_SENSOR_OVERRANGE_STATUS_TOPIC = "mip/sensor/overrange_status";

static constexpr auto MIP_GNSS1_FIX_INFO_TOPIC = "mip/gnss_1/fix_info";
static constexpr auto MIP_GNSS1_SBAS_INFO_TOPIC = "mip/gnss_1/sbas_info";
static constexpr auto MIP_GNSS1_RF_ERROR_DETECTION_TOPIC = "mip/gnss_1/rf_error_detection";

static constexpr auto MIP_GNSS2_FIX_INFO_TOPIC = "mip/gnss_2/fix_info";
static constexpr auto MIP_GNSS2_SBAS_INFO_TOPIC = "mip/gnss_2/sbas_info";
static constexpr auto MIP_GNSS2_RF_ERROR_DETECTION_TOPIC = "mip/gnss_2/rf_error_detection";

static constexpr auto MIP_GNSS_CORRECTIONS_RTK_CORRECTIONS_STATUS_TOPIC = "mip/gnss_corrections/rtk_corrections_status";

static constexpr auto MIP_FILTER_STATUS_TOPIC = "mip/ekf/status";
static constexpr auto MIP_FILTER_GNSS_POSITION_AIDING_STATUS_TOPIC = "mip/ekf/gnss_position_aiding_status";
static constexpr auto MIP_FILTER_MULTI_ANTENNA_OFFSET_CORRECTION_TOPIC = "mip/ekf/multi_antenna_offset_correction";
static constexpr auto MIP_FILTER_GNSS_DUAL_ANTENNA_STATUS_TOPIC = "mip/ekf/gnss_dual_antenna_status";
static constexpr auto MIP_FILTER_AIDING_MEASUREMENT_SUMMARY_TOPIC = "mip/ekf/aiding_measurement_summary";

static constexpr auto MIP_SYSTEM_BUILT_IN_TEST_TOPIC = "mip/system/built_in_test";

static constexpr auto NMEA_SENTENCE_TOPIC = "nmea";

// Some other constants
static constexpr float FIELD_DATA_RATE_USE_DATA_CLASS = -1;
static constexpr float DATA_CLASS_DATA_RATE_DO_NOT_STREAM = 0;

/**
 * Container for both a descriptor set and field descriptor
 */
struct MipDescriptor
{
  uint8_t descriptor_set;  /// Descriptor set
  uint8_t field_descriptor;  /// Field descriptor within the descriptor_set
};

/**
 * Container that will hold information associated with a topic
 */
struct MipPublisherMappingInfo
{
  std::vector<uint8_t> descriptor_sets = {};  /// Descriptor sets used by this topic
  std::vector<MipDescriptor> descriptors = {};  /// Descriptors streamed by this topic
  float data_rate = DATA_CLASS_DATA_RATE_DO_NOT_STREAM;  /// Data rate that this topic is streamed at
};

/**
 * Helper class used to lookup MIP or device information given a topic name
 */
class MipPublisherMapping
{
 public:
  /**
   * \brief Default constructor
   */
  MipPublisherMapping() = default;

  /**
   * \brief Constructs the mapping with a reference to the ROS node and the device. The reference to the ROS node will be saved as a member variable for later usage
   * \param node  The ROS node that is constructing this object
   * \param inertial_device  Pointer to the inertial device that we will use to read information from the device 
   */
  MipPublisherMapping(RosNodeType* node, const std::shared_ptr<RosMipDeviceMain> inertial_device);

  /**
   * \brief Configures the data rates associated with the topics. Updates the map with a data rate for each topic
   * \param config_node  ROS node to read the config from
   * \return True if the configuration was successful, false otherwise
   */
  bool configure(RosNodeType* config_node);

  /**
   * \brief Gets the data classes (descriptor sets) that are used by the topic. Will only return the data classes supported by the device passed into the constructor
   * \param topic  Name of the topic to search for
   * \return List of data classes for the requested topic, or an empty vector if the topic cannot be found or is not supported by the device
   */
  std::vector<uint8_t> getDescriptorSets(const std::string& topic) const;

  /**
   * \brief Gets the descriptors that are used by the topic. Will only return the channel fields supported by the device passed into the constructor
   * \param topic  Name of the topic to search for
   * \return List of descriptors for the requested topic, or an empty vector if the topic cannot be found or is not supported by the device
   */
  std::vector<MipDescriptor> getDescriptors(const std::string& topic) const;

  /**
   * \brief Gets the data rate of the associated topic. Will only return a valid number if called after "configure"
   * \param topic  Name of the topic to search for
   * \return Data rate in hertz that the data is being streamed at
   */
  float getDataRate(const std::string& topic) const;

  /**
   * \brief Gets the maximum data rate among all topics. Will only return a valid number if called after "configure"
   * \param descriptor_set  Descriptor set to search for the max rate of. If set to the shared descriptor set, will return the highest data rate out of all descriptors
   * \return Maximum data rate among all topics in hertz
   */
  float getMaxDataRate(uint8_t descriptor_set = mip::data_shared::DESCRIPTOR_SET) const;

  /**
   * \brief Returns whether a topic is able to be published by a device. This will return true if the device supports the channel fields required by the topic
   * \param topic  Name of the topic to check if the device can publish
   * \return true if the device can publish the topic, false otherwise
   */
  bool canPublish(const std::string& topic) const;

  /**
   * \brief Returns whether a topic is configured to be published. This will return true if the device supports the channel fields required by the topic, AND if the user requested the topic to be published
   * \param topic  Name of the topic to check if the device should publish
   * \return true if the device can publish the topic, false otherwise
   */
  bool shouldPublish(const std::string& topic) const;

  // Static mappings for topics. Note that this map contains all possible topics regardless of what the device supports
  static const std::map<std::string, FieldWrapper::SharedPtrVec> static_topic_to_mip_type_mapping_;  /// Mapping between topics and MIP types which can be used to lookup the descriptor set and field descriptors for a topic.
  static const std::map<std::string, std::string> static_topic_to_data_rate_config_key_mapping_;  /// Mapping between topics and the keys in the config used to configure their data rates
 private:
  /**
   * \brief Streams the desired descriptor for all descriptor sets that support it at the highest rate of the descriptor sets.
   * \tparam MipType The type of MIP field to stream for all descriptor sets
   */
  template<typename MipType>
  void streamSharedDescriptor();

  /**
   * \brief Streams the desired MipType at the same rate as it's descriptor set
   * \tparam MipType The type of MIP field to stream
   * \tparam DescriptorSet Optional parameter to specify a different descriptor set from the MipType
   */
  template<typename MipType, uint8_t DescriptorSet = MipType::DESCRIPTOR_SET>
  void streamAtDescriptorSetRate();

  RosNodeType* node_;  /// Reference to the node object that initialized this class. Used for logging and extracting ROS information
  std::shared_ptr<RosMipDeviceMain> mip_device_;  /// Reference to the MIP device pointer used to read and write information to the device

  std::map<std::string, MipPublisherMappingInfo> topic_info_mapping_;  /// Will be populated based on the device with a mapping between with the topic and ROS and MIP configuration.
  std::map<uint8_t, std::vector<mip::DescriptorRate>> streamed_descriptors_mapping_;  /// Will be populated based on the device with a mapping between descriptor sets and the rates for each field descriptor.
};

template<typename MipType>
void MipPublisherMapping::streamSharedDescriptor()
{
  for (auto& streamed_descriptor_mapping : streamed_descriptors_mapping_)
  {
    // Only stream the descriptor if it is supported within the descriptor set
    const uint8_t descriptor_set = streamed_descriptor_mapping.first;
    if (mip_device_->supportsDescriptor(descriptor_set, MipType::FIELD_DESCRIPTOR))
    {
      // Stream the field descriptor at the highest rate among the descriptor set
      const uint16_t hertz = getMaxDataRate(descriptor_set);
      if (hertz != 0)
      {
        const uint16_t decimation = mip_device_->getDecimationFromHertz(descriptor_set, hertz);
        streamed_descriptor_mapping.second.insert(streamed_descriptor_mapping.second.begin(), {MipType::FIELD_DESCRIPTOR, decimation});
      }
    }
  }
}

template<typename MipType, uint8_t DescriptorSet>
void MipPublisherMapping::streamAtDescriptorSetRate()
{
  // Only stream the descriptor if is is supported within the descriptor set
  if (mip_device_->supportsDescriptor(DescriptorSet, MipType::FIELD_DESCRIPTOR))
  {
    // No need to stream this descriptor if we are not streaming anything else in the descriptor set
    if (streamed_descriptors_mapping_.find(DescriptorSet) != streamed_descriptors_mapping_.end())
    {
      // Stream the field descriptor at the highest rate among the descriptor set
      const uint16_t hertz = getMaxDataRate(DescriptorSet);
      const uint16_t decimation = mip_device_->getDecimationFromHertz(DescriptorSet, hertz);
      auto& streamed_descriptors_mapping_vec = streamed_descriptors_mapping_[DescriptorSet];
      streamed_descriptors_mapping_vec.insert(streamed_descriptors_mapping_vec.begin(), {MipType::FIELD_DESCRIPTOR, decimation});
    }
  }
}

}  // namespace microstrain

#endif  // MICROSTRAIN_INERTIAL_DRIVER_COMMON_UTILS_MAPPINGS_MIP_PUBLISHER_MAPPING_H

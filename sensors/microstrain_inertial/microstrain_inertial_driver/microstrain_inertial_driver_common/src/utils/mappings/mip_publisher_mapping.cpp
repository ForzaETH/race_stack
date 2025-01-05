/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Parker-Lord Inertial Device Driver Implementation File
//
// Copyright (c) 2017, Brian Bingham
// Copyright (c) 2020, Parker Hannifin Corp
// This code is licensed under MIT license (see LICENSE file for details)
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

#include <map>
#include <string>
#include <memory>
#include <vector>
#include <algorithm>

#include "microstrain_inertial_driver_common/config.h"
#include "microstrain_inertial_driver_common/utils/mappings/mip_publisher_mapping.h"

namespace microstrain
{

MipPublisherMapping::MipPublisherMapping(RosNodeType* node, const std::shared_ptr<RosMipDeviceMain> inertial_device) : node_(node), mip_device_(inertial_device)
{
  // Add all supported descriptors to the supported mapping
  for (const auto& mip_type_mapping : static_topic_to_mip_type_mapping_)
  {
    const std::string& topic = mip_type_mapping.first;
    const FieldWrapper::SharedPtrVec& fields = mip_type_mapping.second;

    // Check if any of the fields are supported, and add any supported fields to the list of topic info
    for (const auto& field : fields)
    {
      const uint8_t descriptor_set = field->descriptorSet();
      const uint8_t field_descriptor = field->fieldDescriptor();
      if (mip_device_->supportsDescriptor(descriptor_set, field_descriptor))
      {
        // Add the descriptor to the mapping for the topic
        if (topic_info_mapping_.find(topic) == topic_info_mapping_.end())
          topic_info_mapping_[topic] = MipPublisherMappingInfo();

        auto& topic_info = topic_info_mapping_[topic];
        topic_info.descriptors.push_back({descriptor_set, field_descriptor});
        if (std::find(topic_info.descriptor_sets.begin(), topic_info.descriptor_sets.end(), descriptor_set) == topic_info.descriptor_sets.end())
          topic_info.descriptor_sets.push_back(descriptor_set);
      }
      else if (descriptor_set == mip::data_filter::DESCRIPTOR_SET && field_descriptor == mip::data_filter::EcefPos::FIELD_DESCRIPTOR)
      {
        // Special handling for filtered ECEF position. We can convert LLH to ECEF, so if the device does not support ECEF, try to add LLH
        if (mip_device_->supportsDescriptor(descriptor_set, mip::data_filter::PositionLlh::FIELD_DESCRIPTOR))
        {
          // Add the descriptor to the mapping for the topic
          if (topic_info_mapping_.find(topic) == topic_info_mapping_.end())
            topic_info_mapping_[topic] = MipPublisherMappingInfo();

          auto& topic_info = topic_info_mapping_[topic];
          topic_info.descriptors.push_back({descriptor_set, mip::data_filter::PositionLlh::FIELD_DESCRIPTOR});
          if (std::find(topic_info.descriptor_sets.begin(), topic_info.descriptor_sets.end(), descriptor_set) == topic_info.descriptor_sets.end())
            topic_info.descriptor_sets.push_back(descriptor_set);
        }
      }
      else if (descriptor_set == mip::data_filter::DESCRIPTOR_SET && field_descriptor == mip::data_filter::EcefPosUncertainty::FIELD_DESCRIPTOR)
      {
        // Special handling for filtered ECEF position. We can convert LLH to ECEF, so if the device does not support ECEF, try to add LLH
        if (mip_device_->supportsDescriptor(descriptor_set, mip::data_filter::PositionLlhUncertainty::FIELD_DESCRIPTOR))
        {
          // Add the descriptor to the mapping for the topic
          if (topic_info_mapping_.find(topic) == topic_info_mapping_.end())
            topic_info_mapping_[topic] = MipPublisherMappingInfo();

          auto& topic_info = topic_info_mapping_[topic];
          topic_info.descriptors.push_back({descriptor_set, mip::data_filter::PositionLlhUncertainty::FIELD_DESCRIPTOR});
          if (std::find(topic_info.descriptor_sets.begin(), topic_info.descriptor_sets.end(), descriptor_set) == topic_info.descriptor_sets.end())
            topic_info.descriptor_sets.push_back(descriptor_set);
        }
      }
      else if (mip_device_->supportsDescriptorSet(descriptor_set))
      {
        MICROSTRAIN_DEBUG(node_, "Note: The device does not support field 0x%02x%02x associated with topic %s", descriptor_set, field_descriptor, topic.c_str());
      }
    }
    if (topic_info_mapping_.find(topic) == topic_info_mapping_.end())
      MICROSTRAIN_INFO(node_, "Note: The device does not support publishing the topic %s", topic.c_str());
  }
}

bool MipPublisherMapping::configure(RosNodeType* config_node)
{
  // Add the data rates to the topic info map
  for (auto& mapping : topic_info_mapping_)
  {
    const auto& topic = mapping.first;
    auto& topic_info = mapping.second;

    // Get the data rate for the topic, and if it is not the default, use it, otherwise use the data class data rate
    if (static_topic_to_data_rate_config_key_mapping_.find(topic) != static_topic_to_data_rate_config_key_mapping_.end())
    {
      getParamFloat(config_node, static_topic_to_data_rate_config_key_mapping_.at(topic), topic_info.data_rate, DATA_CLASS_DATA_RATE_DO_NOT_STREAM);
      if (topic_info.data_rate == FIELD_DATA_RATE_USE_DATA_CLASS)
      {
        MICROSTRAIN_WARN(node_, "Data rates of %0.1f are no longer supported. Disabling topic %s", FIELD_DATA_RATE_USE_DATA_CLASS, topic.c_str());
      }
    }
    else
    {
      MICROSTRAIN_ERROR(node_, "Topic %s does not have an associated data rate, this should be added to the 'static_topic_to_data_rate_config_key_mapping_' map", topic.c_str());
      return false;
    }

    // Get the decimation for the topic and add it to the map
    MICROSTRAIN_DEBUG(node_, "Configuring topic %s to stream at %.04f hz", topic.c_str(), topic_info.data_rate);
    for (const auto& descriptor : topic_info.descriptors)
    {
      double real_data_rate;
      const uint8_t descriptor_set = descriptor.descriptor_set;
      const uint8_t field_descriptor = descriptor.field_descriptor;
      const uint16_t decimation = mip_device_->getDecimationFromHertz(descriptor_set, topic_info.data_rate, &real_data_rate);

      // Update the data rate with the real data rate
      topic_info.data_rate = real_data_rate;

      // Append the channel to the proper entry for it's descriptor set
      if (streamed_descriptors_mapping_.find(descriptor_set) == streamed_descriptors_mapping_.end())
        streamed_descriptors_mapping_[descriptor_set] = {};
      auto& descriptor_rates = streamed_descriptors_mapping_[descriptor_set];

      // If the data rate is 0, do not stream any data
      if (topic_info.data_rate == DATA_CLASS_DATA_RATE_DO_NOT_STREAM)
      {
        MICROSTRAIN_DEBUG(node_, "Not configuring descriptor 0x%02x%02x to stream because it's data rate is set to 0", descriptor_set, field_descriptor);
        continue;
      }

      // If the channel has already been added, just update the rate of the existing entry
      auto existing_descriptor_rate = std::find_if(descriptor_rates.begin(), descriptor_rates.end(), [field_descriptor](const mip::DescriptorRate& d)
      {
        return d.descriptor == field_descriptor;
      }
      );
      if (existing_descriptor_rate != descriptor_rates.end())
      {
        if (existing_descriptor_rate->decimation > decimation)  // Lower decimation means faster data
        {
          MICROSTRAIN_DEBUG(node_, "Updating descriptor 0x%02x%02x to stream at %.04f hz", descriptor_set, field_descriptor, topic_info.data_rate);
          existing_descriptor_rate->decimation = decimation;
        }
        else
        {
          MICROSTRAIN_DEBUG(node_, "Descriptor 0x%02x%02x is already streaming faster than %.04f hz, so we are not updating it", descriptor_set, field_descriptor, topic_info.data_rate);
        }
      }
      else
      {
        MICROSTRAIN_DEBUG(node_, "Streaming descriptor 0x%02x%02x at a rate of %.04f hz", descriptor_set, field_descriptor, topic_info.data_rate);
        descriptor_rates.push_back({ field_descriptor, decimation });
      }
    }
  }

  // If we are using linear accel instead of compensated accel, replace the descriptor
  bool filter_use_compensated_accel; getParam(config_node, "filter_use_compensated_accel", filter_use_compensated_accel, true);
  if (!filter_use_compensated_accel)
  {
    if (mip_device_->supportsDescriptor(mip::data_filter::DESCRIPTOR_SET, mip::data_filter::DATA_LINEAR_ACCELERATION))
    {
      // Find and delete the compensated accel descriptor from streaming if it exists
      auto& descriptor_rates = streamed_descriptors_mapping_[mip::data_filter::DESCRIPTOR_SET];
      auto compensated_accel_rate_iter = std::find_if(descriptor_rates.begin(), descriptor_rates.end(), [](const mip::DescriptorRate& d)
      {
        return d.descriptor == mip::data_filter::DATA_COMPENSATED_ACCELERATION;
      }
      );
      mip::DescriptorRate compensated_accel_rate = *compensated_accel_rate_iter;
      if (compensated_accel_rate_iter != descriptor_rates.end())
        compensated_accel_rate_iter = descriptor_rates.erase(compensated_accel_rate_iter);

      // Insert the new descriptor rate
      compensated_accel_rate.descriptor = mip::data_filter::DATA_LINEAR_ACCELERATION;
      descriptor_rates.insert(compensated_accel_rate_iter, compensated_accel_rate);
    }
  }

  // Add shared descriptors
  if (mip_device_->supportsDescriptor(mip::data_sensor::DESCRIPTOR_SET, mip::data_shared::DATA_GPS_TIME))
  {
    // Prospect devices
    streamSharedDescriptor<mip::data_shared::GpsTimestamp>();
  }
  else
  {
    // Philo devices
    streamAtDescriptorSetRate<mip::data_sensor::GpsTimestamp>();
    streamAtDescriptorSetRate<mip::data_gnss::GpsTime>();
    streamAtDescriptorSetRate<mip::data_filter::Timestamp>();
  }
  if (mip_device_->supportsDescriptor(mip::data_sensor::DESCRIPTOR_SET, mip::data_shared::DATA_REFERENCE_TIME))
  {
    streamSharedDescriptor<mip::data_shared::ReferenceTimestamp>();
  }

  // Stream RTK data if the RTK dongle is enabled
  bool rtk_dongle_enable; getParam(config_node, "rtk_dongle_enable", rtk_dongle_enable, true);
  if (rtk_dongle_enable)
  {
    if (mip_device_->supportsDescriptor(mip::data_gnss::MIP_GNSS3_DATA_DESC_SET, mip::data_gnss::DATA_RTK_CORRECTIONS_STATUS))
    {
      MICROSTRAIN_DEBUG(node_, "Streaming descriptor 0x%02x%02x as an on demand descriptor (we will receive it at whatever rate the device does)", mip::data_gnss::MIP_GNSS3_DATA_DESC_SET, mip::data_gnss::RtkCorrectionsStatus::FIELD_DESCRIPTOR);
      if (streamed_descriptors_mapping_.find(mip::data_gnss::MIP_GNSS3_DATA_DESC_SET) == streamed_descriptors_mapping_.end())
        streamed_descriptors_mapping_[mip::data_gnss::MIP_GNSS3_DATA_DESC_SET] = {};
      streamed_descriptors_mapping_[mip::data_gnss::MIP_GNSS3_DATA_DESC_SET].push_back({ mip::data_gnss::RtkCorrectionsStatus::FIELD_DESCRIPTOR, 1 });
    }
  }

  // If the relative position mode is based on the base station, stream that information
  int filter_relative_pos_source; getParam(config_node, "filter_relative_position_source", filter_relative_pos_source, REL_POS_SOURCE_AUTO);
  if (filter_relative_pos_source == REL_POS_SOURCE_BASE_STATION)
  {
    if (mip_device_->supportsDescriptor(mip::data_gnss::MIP_GNSS3_DATA_DESC_SET, mip::data_gnss::DATA_RTK_CORRECTIONS_STATUS))
    {
      MICROSTRAIN_DEBUG(node_, "Streaming descriptor 0x%02x%02x as an on demand descriptor (we will receive it at whatever rate the device does)", mip::data_gnss::MIP_GNSS3_DATA_DESC_SET, mip::data_gnss::BaseStationInfo::FIELD_DESCRIPTOR);
      if (streamed_descriptors_mapping_.find(mip::data_gnss::MIP_GNSS3_DATA_DESC_SET) == streamed_descriptors_mapping_.end())
        streamed_descriptors_mapping_[mip::data_gnss::MIP_GNSS3_DATA_DESC_SET] = {};
      streamed_descriptors_mapping_[mip::data_gnss::MIP_GNSS3_DATA_DESC_SET].push_back({ mip::data_gnss::BaseStationInfo::FIELD_DESCRIPTOR, 1 });
    }
  }

  // Enable each of the descriptor sets and save the message format
  for (const auto& streamed_descriptor_mapping : streamed_descriptors_mapping_)
  {
    mip::CmdResult mip_cmd_result;
    const uint8_t descriptor_set = streamed_descriptor_mapping.first;
    const std::vector<mip::DescriptorRate> descriptor_rates = streamed_descriptor_mapping.second;
    if (!(mip_cmd_result = mip_device_->writeMessageFormat(descriptor_set, descriptor_rates.size(), descriptor_rates.data())))
    {
      MICROSTRAIN_ERROR(node_, "Failed to write message format for descriptor set 0x%02x", descriptor_set);
      MICROSTRAIN_ERROR(node_, "  Error(%d): %s", mip_cmd_result.value, mip_cmd_result.name());
      return false;
    }
    if (!(mip_cmd_result = mip_device_->writeDatastreamControl(descriptor_set, true)))
    {
      MICROSTRAIN_ERROR(node_, "Failed to enable descriptor set 0x%02x", descriptor_set);
      MICROSTRAIN_ERROR(node_, "  Error(%d): %s", mip_cmd_result.value, mip_cmd_result.name());
      return false;
    }
  }
  return true;
}

std::vector<uint8_t> MipPublisherMapping::getDescriptorSets(const std::string& topic) const
{
  if (topic_info_mapping_.find(topic) != topic_info_mapping_.end())
    return topic_info_mapping_.at(topic).descriptor_sets;
  else
    return {};
}

std::vector<MipDescriptor> MipPublisherMapping::getDescriptors(const std::string& topic) const
{
  if (topic_info_mapping_.find(topic) != topic_info_mapping_.end())
    return topic_info_mapping_.at(topic).descriptors;
  else
    return {};
}

float MipPublisherMapping::getDataRate(const std::string& topic) const
{
  if (topic_info_mapping_.find(topic) != topic_info_mapping_.end())
    return topic_info_mapping_.at(topic).data_rate;
  else
    return DATA_CLASS_DATA_RATE_DO_NOT_STREAM;
}

float MipPublisherMapping::getMaxDataRate(uint8_t descriptor_set) const
{
  std::vector<float> data_rates;
  for (const auto& element : static_topic_to_mip_type_mapping_)
  {
    const std::vector<uint8_t> descriptor_sets = getDescriptorSets(element.first);
    if (descriptor_set == mip::data_shared::DESCRIPTOR_SET || std::find(descriptor_sets.begin(), descriptor_sets.end(), descriptor_set) != descriptor_sets.end())
      data_rates.push_back(getDataRate(element.first));
  }
  return *std::max_element(data_rates.begin(), data_rates.end());
}

bool MipPublisherMapping::canPublish(const std::string& topic) const
{
  return !getDescriptors(topic).empty();
}

bool MipPublisherMapping::shouldPublish(const std::string& topic) const
{
  return canPublish(topic) && getDataRate(topic) != DATA_CLASS_DATA_RATE_DO_NOT_STREAM;
}

const std::map<std::string, FieldWrapper::SharedPtrVec> MipPublisherMapping::static_topic_to_mip_type_mapping_ =
{
  // /imu* topic mappings
  {IMU_DATA_RAW_TOPIC, {
    FieldWrapperType<mip::data_sensor::ScaledAccel>::initialize(),
    FieldWrapperType<mip::data_sensor::ScaledGyro>::initialize(),
  }},
  {IMU_DATA_TOPIC, {
    FieldWrapperType<mip::data_sensor::DeltaTheta>::initialize(),
    FieldWrapperType<mip::data_sensor::DeltaVelocity>::initialize(),
    FieldWrapperType<mip::data_sensor::CompQuaternion>::initialize(),
  }},
  {IMU_MAG_TOPIC, {
    FieldWrapperType<mip::data_sensor::ScaledMag>::initialize(),
  }},
  {IMU_PRESSURE_TOPIC, {
    FieldWrapperType<mip::data_sensor::ScaledPressure>::initialize(),
  }},
  {IMU_WHEEL_SPEED_TOPIC, {
    FieldWrapperType<mip::data_sensor::OdometerData>::initialize(),
  }},

  // GNSS1 topic mappings. Note that each of these topics will contain a field for both the GNSS and GNSS1 descriptor set
  {GNSS1_LLH_POSITION_TOPIC, {
    FieldWrapperType<mip::data_gnss::PosLlh, mip::data_gnss::DESCRIPTOR_SET>::initialize(),
    FieldWrapperType<mip::data_gnss::PosLlh, mip::data_gnss::MIP_GNSS1_DATA_DESC_SET>::initialize(),
  }},
  {GNSS1_VELOCITY_TOPIC, {
    FieldWrapperType<mip::data_gnss::VelNed, mip::data_gnss::DESCRIPTOR_SET>::initialize(),
    FieldWrapperType<mip::data_gnss::VelNed, mip::data_gnss::MIP_GNSS1_DATA_DESC_SET>::initialize(),
  }},
  {GNSS1_VELOCITY_ECEF_TOPIC, {
    FieldWrapperType<mip::data_gnss::VelEcef, mip::data_gnss::DESCRIPTOR_SET>::initialize(),
    FieldWrapperType<mip::data_gnss::VelEcef, mip::data_gnss::MIP_GNSS1_DATA_DESC_SET>::initialize(),
  }},
  {GNSS1_ODOMETRY_TOPIC, {
    FieldWrapperType<mip::data_gnss::PosEcef, mip::data_gnss::DESCRIPTOR_SET>::initialize(),
    FieldWrapperType<mip::data_gnss::PosEcef, mip::data_gnss::MIP_GNSS1_DATA_DESC_SET>::initialize(),
    FieldWrapperType<mip::data_gnss::VelNed, mip::data_gnss::DESCRIPTOR_SET>::initialize(),
    FieldWrapperType<mip::data_gnss::VelNed, mip::data_gnss::MIP_GNSS1_DATA_DESC_SET>::initialize(),
  }},
  {GNSS1_TIME_REF_TOPIC, {
    FieldWrapperType<mip::data_gnss::GpsTime, mip::data_gnss::DESCRIPTOR_SET>::initialize(),
    FieldWrapperType<mip::data_gnss::GpsTime, mip::data_gnss::MIP_GNSS1_DATA_DESC_SET>::initialize(),
  }},

  // GNSS2 topic mappings.
  {GNSS2_FIX_TOPIC, {
    FieldWrapperType<mip::data_gnss::PosLlh, mip::data_gnss::MIP_GNSS2_DATA_DESC_SET>::initialize(),
  }},
  {GNSS2_VELOCITY_TOPIC, {
    FieldWrapperType<mip::data_gnss::VelNed, mip::data_gnss::MIP_GNSS2_DATA_DESC_SET>::initialize(),
  }},
  {GNSS2_VELOCITY_ECEF_TOPIC, {
    FieldWrapperType<mip::data_gnss::VelEcef, mip::data_gnss::MIP_GNSS2_DATA_DESC_SET>::initialize(),
  }},
  {GNSS2_ODOMETRY_TOPIC, {
    FieldWrapperType<mip::data_gnss::PosEcef, mip::data_gnss::MIP_GNSS2_DATA_DESC_SET>::initialize(),
    FieldWrapperType<mip::data_gnss::VelNed, mip::data_gnss::MIP_GNSS2_DATA_DESC_SET>::initialize(),
  }},
  {GNSS2_TIME_REF_TOPIC, {
    FieldWrapperType<mip::data_gnss::GpsTime, mip::data_gnss::MIP_GNSS2_DATA_DESC_SET>::initialize(),
  }},

  // Filter topic mappings
  {FILTER_HUMAN_READABLE_STATUS_TOPIC, {
    FieldWrapperType<mip::data_gnss::FixInfo, mip::data_gnss::DESCRIPTOR_SET>::initialize(),
    FieldWrapperType<mip::data_gnss::FixInfo, mip::data_gnss::MIP_GNSS1_DATA_DESC_SET>::initialize(),
    FieldWrapperType<mip::data_gnss::FixInfo, mip::data_gnss::MIP_GNSS2_DATA_DESC_SET>::initialize(),
    FieldWrapperType<mip::data_filter::GnssPosAidStatus>::initialize(),
    FieldWrapperType<mip::data_filter::GnssDualAntennaStatus>::initialize(),
    FieldWrapperType<mip::data_filter::Status>::initialize(),
    FieldWrapperType<mip::data_system::BuiltInTest>::initialize(),
  }},
  {FILTER_IMU_DATA_TOPIC, {
    FieldWrapperType<mip::data_filter::AttitudeQuaternion>::initialize(),
    FieldWrapperType<mip::data_filter::CompAngularRate>::initialize(),
    FieldWrapperType<mip::data_filter::CompAccel>::initialize(),
    FieldWrapperType<mip::data_filter::EulerAnglesUncertainty>::initialize(),
  }},
  {FILTER_LLH_POSITION_TOPIC, {
    FieldWrapperType<mip::data_filter::PositionLlh>::initialize(),
    FieldWrapperType<mip::data_filter::PositionLlhUncertainty>::initialize(),
    FieldWrapperType<mip::data_filter::GnssPosAidStatus>::initialize(),
  }},
  {FILTER_VELOCITY_TOPIC, {
    FieldWrapperType<mip::data_filter::VelocityNed>::initialize(),
    FieldWrapperType<mip::data_filter::VelocityNedUncertainty>::initialize(),
    FieldWrapperType<mip::data_filter::CompAngularRate>::initialize(),
  }},
  {FILTER_VELOCITY_ECEF_TOPIC, {
    FieldWrapperType<mip::data_filter::EcefVel>::initialize(),
    FieldWrapperType<mip::data_filter::EcefVelUncertainty>::initialize(),
    FieldWrapperType<mip::data_filter::CompAngularRate>::initialize(),
  }},
  {FILTER_ODOMETRY_EARTH_TOPIC , {
    FieldWrapperType<mip::data_filter::EcefPos>::initialize(),
    FieldWrapperType<mip::data_filter::EcefPosUncertainty>::initialize(),
    FieldWrapperType<mip::data_filter::AttitudeQuaternion>::initialize(),
    FieldWrapperType<mip::data_filter::EulerAnglesUncertainty>::initialize(),
    FieldWrapperType<mip::data_filter::VelocityNed>::initialize(),
    FieldWrapperType<mip::data_filter::VelocityNedUncertainty>::initialize(),
    FieldWrapperType<mip::data_filter::CompAngularRate>::initialize(),
  }},
  {FILTER_ODOMETRY_MAP_TOPIC, {
    FieldWrapperType<mip::data_filter::EcefPos>::initialize(),
    FieldWrapperType<mip::data_filter::PositionLlhUncertainty>::initialize(),
    FieldWrapperType<mip::data_filter::AttitudeQuaternion>::initialize(),
    FieldWrapperType<mip::data_filter::EulerAnglesUncertainty>::initialize(),
    FieldWrapperType<mip::data_filter::VelocityNed>::initialize(),
    FieldWrapperType<mip::data_filter::VelocityNedUncertainty>::initialize(),
    FieldWrapperType<mip::data_filter::CompAngularRate>::initialize(),
  }},
  {FILTER_DUAL_ANTENNA_HEADING_TOPIC, {
    FieldWrapperType<mip::data_filter::GnssDualAntennaStatus>::initialize(),
  }},

  // MIP sensor (0x80) topic mappings
  {MIP_SENSOR_OVERRANGE_STATUS_TOPIC, {
    FieldWrapperType<mip::data_sensor::OverrangeStatus>::initialize(),
  }},
  {MIP_SENSOR_TEMPERATURE_STATISTICS_TOPIC, {
    FieldWrapperType<mip::data_sensor::TemperatureAbs>::initialize(),
  }},

  // MIP GNSS1 (0x81, 0x91) topic mappings
  {MIP_GNSS1_FIX_INFO_TOPIC, {
    FieldWrapperType<mip::data_gnss::FixInfo, mip::data_gnss::DESCRIPTOR_SET>::initialize(),
    FieldWrapperType<mip::data_gnss::FixInfo, mip::data_gnss::MIP_GNSS1_DATA_DESC_SET>::initialize(),
  }},
  {MIP_GNSS1_SBAS_INFO_TOPIC, {
    FieldWrapperType<mip::data_gnss::SbasInfo, mip::data_gnss::MIP_GNSS1_DATA_DESC_SET>::initialize(),
  }},
  {MIP_GNSS1_RF_ERROR_DETECTION_TOPIC, {
    FieldWrapperType<mip::data_gnss::RfErrorDetection, mip::data_gnss::MIP_GNSS1_DATA_DESC_SET>::initialize()
  }},

  // MIP GNSS2 (0x92) topic mappings
  {MIP_GNSS2_FIX_INFO_TOPIC, {
    FieldWrapperType<mip::data_gnss::FixInfo, mip::data_gnss::MIP_GNSS2_DATA_DESC_SET>::initialize(),
  }},
  {MIP_GNSS2_SBAS_INFO_TOPIC, {
    FieldWrapperType<mip::data_gnss::SbasInfo, mip::data_gnss::MIP_GNSS2_DATA_DESC_SET>::initialize(),
  }},
  {MIP_GNSS2_RF_ERROR_DETECTION_TOPIC, {
    FieldWrapperType<mip::data_gnss::RfErrorDetection, mip::data_gnss::MIP_GNSS2_DATA_DESC_SET>::initialize()
  }},

  // MIP Filter (0x82) topic mappings
  {MIP_FILTER_STATUS_TOPIC, {
    FieldWrapperType<mip::data_filter::Status>::initialize(),
  }},
  {MIP_FILTER_GNSS_POSITION_AIDING_STATUS_TOPIC, {
    FieldWrapperType<mip::data_filter::GnssPosAidStatus>::initialize(),
  }},
  {MIP_FILTER_MULTI_ANTENNA_OFFSET_CORRECTION_TOPIC, {
    FieldWrapperType<mip::data_filter::MultiAntennaOffsetCorrection>::initialize(),
  }},
  {MIP_FILTER_AIDING_MEASUREMENT_SUMMARY_TOPIC, {
    FieldWrapperType<mip::data_filter::AidingMeasurementSummary>::initialize(),
  }},
  {MIP_FILTER_GNSS_DUAL_ANTENNA_STATUS_TOPIC, {
    FieldWrapperType<mip::data_filter::GnssDualAntennaStatus>::initialize(),
  }},

  // MIP System (0xA0) topic mappings
  {MIP_SYSTEM_BUILT_IN_TEST_TOPIC, {
    FieldWrapperType<mip::data_system::BuiltInTest>::initialize(),
  }},
};

const std::map<std::string, std::string> MipPublisherMapping::static_topic_to_data_rate_config_key_mapping_ =
{
  // /imu* data rates
  {IMU_DATA_RAW_TOPIC,    "imu_data_raw_rate"},
  {IMU_DATA_TOPIC,        "imu_data_rate"},
  {IMU_MAG_TOPIC,         "imu_mag_data_rate"},
  {IMU_PRESSURE_TOPIC,    "imu_pressure_data_rate"},
  {IMU_WHEEL_SPEED_TOPIC, "imu_wheel_speed_data_rate"},

  // GNSS/GNSS1 data rates
  {GNSS1_LLH_POSITION_TOPIC,  "gnss1_llh_position_data_rate"},
  {GNSS1_VELOCITY_TOPIC,      "gnss1_velocity_data_rate"},
  {GNSS1_VELOCITY_ECEF_TOPIC, "gnss1_velocity_ecef_data_rate"},
  {GNSS1_ODOMETRY_TOPIC,      "gnss1_odometry_earth_data_rate"},
  {GNSS1_TIME_REF_TOPIC,      "gnss1_time_data_rate"},

  // GNSS2 data rates
  {GNSS2_FIX_TOPIC,           "gnss2_llh_position_data_rate"},
  {GNSS2_VELOCITY_TOPIC,      "gnss2_velocity_data_rate"},
  {GNSS2_VELOCITY_ECEF_TOPIC, "gnss2_velocity_ecef_data_rate"},
  {GNSS2_ODOMETRY_TOPIC,      "gnss2_odometry_earth_data_rate"},
  {GNSS2_TIME_REF_TOPIC,      "gnss2_time_data_rate"},

  // Filter data rates
  {FILTER_HUMAN_READABLE_STATUS_TOPIC, "filter_human_readable_status_data_rate"},
  {FILTER_IMU_DATA_TOPIC,              "filter_imu_data_rate"},
  {FILTER_LLH_POSITION_TOPIC,          "filter_llh_position_data_rate"},
  {FILTER_VELOCITY_TOPIC,              "filter_velocity_data_rate"},
  {FILTER_VELOCITY_ECEF_TOPIC,         "filter_velocity_ecef_data_rate"},
  {FILTER_ODOMETRY_EARTH_TOPIC,        "filter_odometry_earth_data_rate"},
  {FILTER_ODOMETRY_MAP_TOPIC,          "filter_odometry_map_data_rate"},
  {FILTER_DUAL_ANTENNA_HEADING_TOPIC,  "filter_dual_antenna_heading_data_rate"},

  // MIP sensor (0x80) data rates
  {MIP_SENSOR_OVERRANGE_STATUS_TOPIC,       "mip_sensor_overrange_status_data_rate"},
  {MIP_SENSOR_TEMPERATURE_STATISTICS_TOPIC, "mip_sensor_temperature_statistics_data_rate"},

  // MIP GNSS1 (0x81, 0x91) data rates
  {MIP_GNSS1_FIX_INFO_TOPIC,           "mip_gnss1_fix_info_data_rate"},
  {MIP_GNSS1_SBAS_INFO_TOPIC,          "mip_gnss1_sbas_info_data_rate"},
  {MIP_GNSS1_RF_ERROR_DETECTION_TOPIC, "mip_gnss1_rf_error_detection_data_rate"},

  // MIP GNSS2 (0x92) data rates
  {MIP_GNSS2_FIX_INFO_TOPIC,           "mip_gnss2_fix_info_data_rate"},
  {MIP_GNSS2_SBAS_INFO_TOPIC,          "mip_gnss2_sbas_info_data_rate"},
  {MIP_GNSS2_RF_ERROR_DETECTION_TOPIC, "mip_gnss2_rf_error_detection_data_rate"},

  // MIP filter (0x82) data rates
  {MIP_FILTER_STATUS_TOPIC,                          "mip_filter_status_data_rate"},
  {MIP_FILTER_GNSS_POSITION_AIDING_STATUS_TOPIC,     "mip_filter_gnss_position_aiding_status_data_rate"},
  {MIP_FILTER_MULTI_ANTENNA_OFFSET_CORRECTION_TOPIC, "mip_filter_multi_antenna_offset_correction_data_rate"},
  {MIP_FILTER_AIDING_MEASUREMENT_SUMMARY_TOPIC,      "mip_filter_aiding_measurement_summary_data_rate"},
  {MIP_FILTER_GNSS_DUAL_ANTENNA_STATUS_TOPIC,        "mip_filter_gnss_dual_antenna_status_data_rate"},

  // MIP System (0xA0) data rates
  {MIP_SYSTEM_BUILT_IN_TEST_TOPIC, "mip_system_built_in_test_data_rate"},
};

}  // namespace microstrain

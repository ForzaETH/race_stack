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

#ifndef MICROSTRAIN_INERTIAL_DRIVER_COMMON_UTILS_MAPPINGS_MIP_MAPPING_H_
#define MICROSTRAIN_INERTIAL_DRIVER_COMMON_UTILS_MAPPINGS_MIP_MAPPING_H_

#include <map>
#include <string>
#include <memory>
#include <vector>

#include <stdint.h>

#include "mip/mip_all.hpp"

namespace microstrain
{

/**
 * Wrapper for MIP fields which allows them to be stored in the same container
 */
class FieldWrapper
{
 public:
  using SharedPtr = std::shared_ptr<FieldWrapper>;
  using SharedPtrVec = std::vector<SharedPtr>;

  /**
   * \brief Will be overridden in the implementing class to return descriptor set for the associated field
   * \return descriptor set of the MIP field
   */
  virtual uint8_t descriptorSet() const = 0;

  /**
   * \brief Will be overridden in the implementing class to return field descriptor for the associated field
   * \return field descriptor of the MIP field
   */
  virtual uint8_t fieldDescriptor() const = 0;
};

/**
 * Template extension of FieldWrapper.
 * \tparam DataType Data type class to use for this field wrapper. Should be a class in one of the mip::data_* namespaces
 * \tparam DescriptorSet Optional descriptor set to use instead of the descriptor set on the data field class
 */
template<typename DataType, uint8_t DescriptorSet = DataType::DESCRIPTOR_SET>
class FieldWrapperType : public FieldWrapper
{
 public:
  using SharedPtr = std::shared_ptr<FieldWrapperType<DataType, DescriptorSet>>;
  using SharedPtrVec = std::vector<SharedPtr>;

  /**
   * \brief Returns the descriptor set of the DataType, or optionaly the DescriptorSet passed
   * \return The descriptor set that this template type was initialized with
   */
  uint8_t descriptorSet() const final
  {
    return DescriptorSet;
  }

  /**
   * \brief Returns the field descriptor of the DataType
   * \return The field descriptor that this template type was initialized with
   */
  uint8_t fieldDescriptor() const final
  {
    return DataType::FIELD_DESCRIPTOR;
  }

  /**
   * \brief Initializes an instance of this class
   * \return Shared pointer of this templated class type
   */
  static SharedPtr initialize()
  {
    return std::make_shared<FieldWrapperType<DataType, DescriptorSet>>();
  }
};

/**
 * Generic container to hold any mappings related to MIP, the MIP SDK, or anything of that type
 */
class MipMapping
{
 public:
  static constexpr auto INVALID_STRING = "Invalid";

  /**
   * \brief Looks up the string representation of a descriptor set
   * \param descriptor_set  The descriptor set to lookup
   * \return The string representation of the descriptor set, or Invalid if it can't be found
   */
  static std::string descriptorSetString(uint8_t descriptor_set);

  /**
   * \brief Looks up the NMEA format string interpertation given a message ID
   * \param message_id  The message ID to lookup
   * \return The string representation of the message ID
   */
  static std::string nmeaFormatMessageIdString(mip::commands_3dm::NmeaMessage::MessageID message_id);

  /**
   * \brief Looks up the NMEA format string interpertation given a talker ID
   * \param talker_id  The talker ID to lookup
   * \return The string representation of the talker ID
   */
  static std::string nmeaFormatTalkerIdString(mip::commands_3dm::NmeaMessage::TalkerID talker_id);

  static const std::map<uint8_t, std::string> descriptor_set_string_mapping_;  /// Mapping between descriptor sets and their string names

  static const std::map<std::string, mip::commands_3dm::NmeaMessage::MessageID> nmea_message_string_message_id_mapping_;  /// Mapping between the string representation of the NMEA message IDs and their actual enum values
  static const std::map<std::string, mip::commands_3dm::NmeaMessage::TalkerID> nmea_message_string_talker_id_mapping_;  /// Mapping between the string representation of the NMEA talker IDs and their actual enum values

  static const std::map<mip::commands_3dm::NmeaMessage::MessageID, bool> nmea_message_id_requires_talker_id_mapping_;  /// Mapping between message IDs and whether they require a talker ID
};

}  // namespace microstrain

#endif  // MICROSTRAIN_INERTIAL_DRIVER_COMMON_UTILS_MAPPINGS_MIP_MAPPING_H

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

#include "microstrain_inertial_driver_common/utils/mappings/mip_mapping.h"

namespace microstrain
{

std::string MipMapping::descriptorSetString(uint8_t descriptor_set)
{
  if (descriptor_set_string_mapping_.find(descriptor_set) != descriptor_set_string_mapping_.end())
    return descriptor_set_string_mapping_.at(descriptor_set);
  return INVALID_STRING;
}

std::string MipMapping::nmeaFormatMessageIdString(mip::commands_3dm::NmeaMessage::MessageID message_id)
{
  // This map should be unique, so just return the first entry that matches
  for (const auto& entry : nmea_message_string_message_id_mapping_)
    if (entry.second == message_id)
      return entry.first;
  return INVALID_STRING;
}

std::string MipMapping::nmeaFormatTalkerIdString(mip::commands_3dm::NmeaMessage::TalkerID talker_id)
{
  // This map should be unique, so just return the first entry that matches
  for (const auto& entry : nmea_message_string_talker_id_mapping_)
    if (entry.second == talker_id)
      return entry.first;
  return INVALID_STRING;
}

const std::map<uint8_t, std::string> MipMapping::descriptor_set_string_mapping_ =
{
  // Command descriptor sets
  {mip::commands_base::DESCRIPTOR_SET,   "Base command"},
  {mip::commands_3dm::DESCRIPTOR_SET,    "3DM command"},
  {mip::commands_filter::DESCRIPTOR_SET, "Filter command"},
  {mip::commands_gnss::DESCRIPTOR_SET,   "GNSS command"},
  {mip::commands_rtk::DESCRIPTOR_SET,    "RTK command"},
  {mip::commands_system::DESCRIPTOR_SET, "System command"},

  // Data descriptor sets
  {mip::data_shared::DESCRIPTOR_SET,        "Shared data"},
  {mip::data_system::DESCRIPTOR_SET,        "System data"},
  {mip::data_sensor::DESCRIPTOR_SET,        "Sensor (IMU) data"},
  {mip::data_gnss::DESCRIPTOR_SET,          "GNSS data"},
  {mip::data_gnss::MIP_GNSS1_DATA_DESC_SET, "GNSS1 data"},
  {mip::data_gnss::MIP_GNSS2_DATA_DESC_SET, "GNSS2 data"},
  {mip::data_gnss::MIP_GNSS3_DATA_DESC_SET, "GNSS3 data"},
  {mip::data_gnss::MIP_GNSS4_DATA_DESC_SET, "GNSS4 data"},
  {mip::data_gnss::MIP_GNSS5_DATA_DESC_SET, "GNSS5 data"},
  {mip::data_filter::DESCRIPTOR_SET,        "Filter data"},
};

const std::map<std::string, mip::commands_3dm::NmeaMessage::MessageID> MipMapping::nmea_message_string_message_id_mapping_ =
{
  {"GGA",  mip::commands_3dm::NmeaMessage::MessageID::GGA},
  {"GLL",  mip::commands_3dm::NmeaMessage::MessageID::GLL},
  {"GSV",  mip::commands_3dm::NmeaMessage::MessageID::GSV},
  {"RMC",  mip::commands_3dm::NmeaMessage::MessageID::RMC},
  {"VTG",  mip::commands_3dm::NmeaMessage::MessageID::VTG},
  {"HDT",  mip::commands_3dm::NmeaMessage::MessageID::HDT},
  {"ZDA",  mip::commands_3dm::NmeaMessage::MessageID::ZDA},
  {"PKRA", mip::commands_3dm::NmeaMessage::MessageID::PKRA},
  {"PKRR", mip::commands_3dm::NmeaMessage::MessageID::PKRR},
};

const std::map<std::string, mip::commands_3dm::NmeaMessage::TalkerID> MipMapping::nmea_message_string_talker_id_mapping_ =
{
  {"GN", mip::commands_3dm::NmeaMessage::TalkerID::GNSS},
  {"GP", mip::commands_3dm::NmeaMessage::TalkerID::GPS},
  {"GA", mip::commands_3dm::NmeaMessage::TalkerID::GALILEO},
  {"GL", mip::commands_3dm::NmeaMessage::TalkerID::GLONASS},
};

const std::map<mip::commands_3dm::NmeaMessage::MessageID, bool> MipMapping::nmea_message_id_requires_talker_id_mapping_ =
{
  {mip::commands_3dm::NmeaMessage::MessageID::GGA,  true},
  {mip::commands_3dm::NmeaMessage::MessageID::GLL,  true},
  {mip::commands_3dm::NmeaMessage::MessageID::GSV,  false},
  {mip::commands_3dm::NmeaMessage::MessageID::RMC,  true},
  {mip::commands_3dm::NmeaMessage::MessageID::VTG,  true},
  {mip::commands_3dm::NmeaMessage::MessageID::HDT,  true},
  {mip::commands_3dm::NmeaMessage::MessageID::ZDA,  true},
  {mip::commands_3dm::NmeaMessage::MessageID::PKRA, false},
  {mip::commands_3dm::NmeaMessage::MessageID::PKRR, false},
};

}  // namespace microstrain

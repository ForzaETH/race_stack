/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Parker-Lord Inertial Device Driver Implementation File
//
// Copyright (c) 2017, Brian Bingham
// Copyright (c) 2020, Parker Hannifin Corp
// This code is licensed under MIT license (see LICENSE file for details)
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

#include <sys/stat.h>

#include <vector>
#include <chrono>
#include <string>
#include <memory>
#include <iomanip>

#include "mip/platform/serial_connection.hpp"
#include "mip/extras/recording_connection.hpp"

#include "microstrain_inertial_driver_common/utils/mip/ros_connection.h"
#include "microstrain_inertial_driver_common/utils/mip/ros_mip_device.h"

namespace microstrain
{

constexpr auto NMEA_MAX_LENGTH = 82;

RosConnection::RosConnection(RosNodeType* node) : node_(node)
{
}

bool RosConnection::isConnected() const
{
  if (connection_)
    return connection_->isConnected();
  else
    return false;
}

bool RosConnection::connect()
{
  if (connection_)
    return connection_->connect();
  else
    return false;
}

bool RosConnection::disconnect()
{
  if (connection_)
    return connection_->disconnect();
  else
    return false;
}

bool RosConnection::connect(RosNodeType* config_node, const std::string& port, const int32_t baudrate)
{
  // Some convenient typedefs
  using SerialConnection = mip::platform::SerialConnection;
  using RecordingSerialConnection = mip::extras::RecordingConnectionWrapper<SerialConnection>;

  // If we were asked to, poll the port until it exists
  bool poll_port;
  double poll_rate_hz;
  int32_t poll_max_tries;
  getParam<bool>(config_node, "poll_port", poll_port, false);
  getParam<double>(config_node, "poll_rate_hz", poll_rate_hz, 1.0);
  getParam<int32_t>(config_node, "poll_max_tries", poll_max_tries, 60);
  if (poll_port)
  {
    int32_t poll_tries = 0;
    RosRateType poll_rate(poll_rate_hz);
    struct stat port_stat;
    while (stat(port.c_str(), &port_stat) != 0 && (poll_tries++ < poll_max_tries || poll_max_tries == -1))
    {
      // If the error isn't that the file does not exist, polling won't help, so we can fail here
      if (errno != ENOENT)
      {
        MICROSTRAIN_ERROR(node_,
            "Error while polling for file %s. File appears to exist, but stat returned error: %s",
            port.c_str(), strerror(errno));
        return false;
      }

      // Wait for the specified amount of time
      MICROSTRAIN_WARN(node_, "%s doesn't exist yet. Waiting for file to appear...", port.c_str());
      poll_rate.sleep();
    }

    // If the file still doesn't exist we can safely fail here.
    if (stat(port.c_str(), &port_stat) != 0)
    {
      MICROSTRAIN_ERROR(node_, "Unable to open requested port, error: %s", strerror(errno));
      return false;
    }
  }

  // If the raw file is enabled, use a different connection type
  try
  {
    MICROSTRAIN_INFO(node_, "Attempting to open serial port <%s> at <%d>", port.c_str(), baudrate);
    connection_ = std::unique_ptr<RecordingSerialConnection>(new RecordingSerialConnection(&record_file_, nullptr, port, baudrate));
  }
  catch (const std::exception& e)
  {
    MICROSTRAIN_ERROR(node_, "Failed to initialize the MIP connection: %s", e.what());
    return false;
  }
  if (!connection_->connect())
    return false;

  // TODO(robbiefish): Currently, using the mip_timeout_from_baudrate method results in too short of a timeout. For now, we can just use the longer timeouts, but it would be good to use shorter timeouts when possible
  // Different timeouts based on the type of connection (TCP/Serial)
  // parse_timeout_ = mip::C::mip_timeout_from_baudrate(baudrate);
  // base_reply_timeout_ = 500;
  parse_timeout_ = 1000;
  base_reply_timeout_ = 1000;
  return true;
}

bool RosConnection::configure(RosNodeType* config_node, RosMipDevice* device)
{
  // Setup the path to the raw file even if we are not recording
  time_t raw_time;
  struct tm curr_time;
  char curr_time_buffer[100];

  std::string raw_file_directory;
  getParam<bool>(config_node, "raw_file_enable", should_record_, false);
  getParam<std::string>(config_node, "raw_file_directory", raw_file_directory, std::string("."));

  // Get the device info
  mip::CmdResult mip_cmd_result;
  mip::commands_base::BaseDeviceInfo device_info;
  if (!(mip_cmd_result = device->getDeviceInfo(&device_info)))
  {
    MICROSTRAIN_MIP_SDK_ERROR(node_, mip_cmd_result, "Unable to read device info for binary file");
    return false;
  }

  // Get the current time
  time(&raw_time);
  localtime_r(&raw_time, &curr_time);
  strftime(curr_time_buffer, sizeof(curr_time_buffer), "%y_%m_%d_%H_%M_%S", &curr_time);

  std::string time_string(curr_time_buffer);

  if (raw_file_directory.back() != '/')
    raw_file_directory += "/";
  record_file_path_ = raw_file_directory + device_info.model_name + std::string("_") +
                          device_info.serial_number + std::string("_") + time_string + std::string(".bin");

  // Open raw data file, if enabled
  if (!updateRecordingState(should_record_, record_file_path_))
  {
    return false;
  }

  return true;
}

bool RosConnection::shouldParseNmea() const
{
  return should_parse_nmea_;
}

void RosConnection::shouldParseNmea(bool enable)
{
  should_parse_nmea_ = enable;
}

mip::Timeout RosConnection::parseTimeout() const
{
  return parse_timeout_;
}

mip::Timeout RosConnection::baseReplyTimeout() const
{
  return base_reply_timeout_;
}

std::vector<NMEASentenceMsg> RosConnection::nmeaMsgs()
{
  auto copy = nmea_msgs_;
  nmea_msgs_.clear();
  return copy;
}

bool RosConnection::rawFileEnable()
{
  return should_record_;
}

std::string RosConnection::rawFilePath()
{
  return record_file_path_;
}

bool RosConnection::updateRecordingState(const bool should_record, const std::string& record_file_path)
{
  // If we are already recording, we need to close the file, but keep that in mind in case we fail to update
  const bool was_recording = record_file_.is_open();
  if (was_recording)
  {
    MICROSTRAIN_INFO(node_, "Closing binary datafile at %s", record_file_path_.c_str());
    record_file_.close();
  }

  // Try to open the new file if we were requested to record
  if (should_record)
  {
    record_file_.open(record_file_path, std::ios::out | std::ios::binary | std::ios::trunc);
    if (!record_file_.is_open())
    {
      MICROSTRAIN_ERROR(node_, "ERROR opening raw binary datafile at %s", record_file_path.c_str());

      // If we failed to open the new file, open the old one but do not truncate it
      if (was_recording)
        record_file_.open(record_file_path_, std::ios::out | std::ios::binary);
      return false;
    }
    else
    {
      MICROSTRAIN_INFO(node_, "Raw binary datafile opened at %s", record_file_path.c_str());
    }
  }

  // Update the state
  should_record_ = should_record;
  record_file_path_ = record_file_path;
  return true;
}

bool RosConnection::sendToDevice(const uint8_t* data, size_t length)
{
  if (connection_ != nullptr)
    return connection_->sendToDevice(data, length);
  else
    return false;
}

bool RosConnection::recvFromDevice(uint8_t* buffer, size_t max_length, mip::Timeout timeout, size_t* count_out, mip::Timestamp* timestamp_out)
{
  const bool success = (connection_ != nullptr) ? connection_->recvFromDevice(buffer, max_length, timeout, count_out, timestamp_out) : false;
  if (success)
  {
    *timestamp_out = static_cast<mip::Timestamp>(getTimeRefSecs(rosTimeNow(node_)) * 1000.0);

    // Parse NMEA sentences if we were asked to
    if (should_parse_nmea_)
      extractNmea(buffer, *count_out);
  }
  return success;
}

const char* RosConnection::interfaceName() const
{
  return "ROS";
}

uint32_t RosConnection::parameter() const
{
  return 0;
}

void RosConnection::extractNmea(const uint8_t* data, size_t data_len)
{
  // Convert into a string if there was actually data
  nmea_string_ += std::string(reinterpret_cast<const char*>(data), data_len);

  // If there is no data, return early
  if (data_len <= 0)
    return;
  MICROSTRAIN_DEBUG(node_, "Read %lu new bytes from port. Parsing a total of %lu bytes including cached data", data_len, nmea_string_.size());

  // Iterate until we find a valid packet
  size_t trim_length = 0;
  for (size_t i = 0; i < nmea_string_.size(); i++)
  {
    // NMEA parsing logic
    if (nmea_string_[i] == '$' || nmea_string_[i] == '!')
    {
      MICROSTRAIN_DEBUG(node_, "Found possible beginning of NMEA sentence at %lu", i);

      // Attempt to find the end of the sentence (this index will point to the \r in the \r\n, so is technically one less than the end index)
      const size_t nmea_end_index = nmea_string_.find("\r\n", i + 1);
      if (nmea_end_index == std::string::npos)
      {
        MICROSTRAIN_DEBUG(node_, "Could not find end of NMEA sentence. Continuing...");
        continue;
      }
      MICROSTRAIN_DEBUG(node_, "Found possible end of NMEA sentence at %lu", nmea_end_index + 1);

      // If the sentence is too long, move on
      const size_t nmea_length = nmea_end_index - i;
      if (nmea_length > NMEA_MAX_LENGTH)
      {
        MICROSTRAIN_DEBUG(node_, "Found what appeared to be a valid NMEA sentence, but it was %lu bytes long, and the max size for a NMEA sentence is %d bytes", nmea_length, NMEA_MAX_LENGTH);
        continue;
      }

      // Attempt to find the checksum
      const size_t checksum_delimiter_index = nmea_string_.rfind('*', nmea_end_index);
      if (checksum_delimiter_index == std::string::npos)
      {
        MICROSTRAIN_DEBUG(node_, "Found beginning and end of NMEA sentence, but could not find the checksum. Skipping");
        continue;
      }
      const size_t checksum_start_index = checksum_delimiter_index + 1;

      // Extract the expected checksum
      const std::string& expected_checksum_str = nmea_string_.substr(checksum_start_index, nmea_end_index - checksum_start_index);
      uint16_t expected_checksum;
      try
      {
        expected_checksum = static_cast<uint16_t>(std::stoi(expected_checksum_str, nullptr, 16));
      }
      catch (const std::exception& e)
      {
        MICROSTRAIN_DEBUG(node_, "Checksum at end of NMEA sentence cannot be parsed into a hex number: %s", expected_checksum_str.c_str());
        continue;
      }

      // Calculate the actual checksum
      uint16_t actual_checksum = 0;
      for (size_t k = i + 1; k < checksum_start_index - 1; k++)
        actual_checksum ^= nmea_string_[k];

      // Extract the sentence
      const std::string& sentence = nmea_string_.substr(i, (nmea_end_index - i) + 2);

      // If the checksum is invalid, move on
      if (actual_checksum != expected_checksum)
      {
        MICROSTRAIN_DEBUG(node_, "Found what appeared to be a valid NMEA sentence, but the checksums did not match. Skipping");
        MICROSTRAIN_DEBUG(node_, "  Sentence:          %s", sentence.c_str());
        MICROSTRAIN_DEBUG(node_, "  Expected Checksum: 0x%02x", expected_checksum);
        MICROSTRAIN_DEBUG(node_, "  Actual Checksum:   0x%02x", actual_checksum);
        continue;
      }

      // Looks like it is a valid NMEA sentence. Publish
      NMEASentenceMsg msg;
      msg.header.stamp = rosTimeNow(node_);
      msg.sentence = sentence;
      nmea_msgs_.push_back(msg);

      // Move the iterator past the end of the sentence, and mark it for deletion
      MICROSTRAIN_DEBUG(node_, "NMEA sentence found starting at index %lu and ending at index %lu: %s", i, nmea_end_index + 1, sentence.c_str());
      trim_length = i = nmea_end_index + 1;
    }
  }

  // Trim the string
  MICROSTRAIN_DEBUG(node_, "Cached NMEA string is %lu bytes before trimming", nmea_string_.size());
  MICROSTRAIN_DEBUG(node_, "Trimming %lu bytes from the beginning of the cached NMEA string", trim_length);
  nmea_string_.erase(0, trim_length);
  if (nmea_string_.size() > NMEA_MAX_LENGTH)
  {
    MICROSTRAIN_DEBUG(node_, "Cached NMEA buffer has grown to %lu bytes. Trimming down to %d bytes", nmea_string_.size(), NMEA_MAX_LENGTH);
    nmea_string_.erase(0, nmea_string_.size() - NMEA_MAX_LENGTH);
  }
  MICROSTRAIN_DEBUG(node_, "Cached NMEA string is %lu bytes after trimming", nmea_string_.size());
}

}  // namespace microstrain

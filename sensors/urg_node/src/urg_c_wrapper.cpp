/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Chad Rockey, Mike O'Driscoll
 */

#include <urg_node/urg_c_wrapper.hpp>

#include <chrono>
#include <cinttypes>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "boost/crc.hpp"

namespace urg_node
{

URGCWrapper::URGCWrapper(
  const EthernetConnection & connection, bool & using_intensity,
  bool & using_multiecho, const rclcpp::Logger & logger)
: ip_address_(connection.ip_address),
  ip_port_(connection.ip_port),
  serial_port_(""),
  serial_baud_(0),
  use_intensity_(using_intensity),
  use_multiecho_(using_multiecho),
  system_latency_(std::chrono::seconds(0)),
  user_latency_(std::chrono::seconds(0)),
  logger_(logger)
{
  (void) adj_alpha_;

  long baudrate_or_port = (long)ip_port_;  // NOLINT
  const char * device = ip_address_.c_str();

  int result = urg_open(&urg_, URG_ETHERNET, device, baudrate_or_port);
  if (result < 0) {
    std::stringstream ss;
    ss << "Could not open network Hokuyo:\n";
    ss << ip_address_ << ":" << ip_port_ << "\n";
    ss << urg_error(&urg_);
    throw std::runtime_error(ss.str());
  }

  initialize(using_intensity, using_multiecho);
}

URGCWrapper::URGCWrapper(
  const SerialConnection & connection,
  bool & using_intensity, bool & using_multiecho, const rclcpp::Logger & logger)
: ip_address_(""),
  ip_port_(0),
  serial_port_(connection.serial_port),
  serial_baud_(connection.serial_baud),
  use_intensity_(using_intensity),
  use_multiecho_(using_multiecho),
  system_latency_(std::chrono::seconds(0)),
  user_latency_(std::chrono::seconds(0)),
  logger_(logger)
{
  (void) adj_alpha_;

  long baudrate_or_port = (long)serial_baud_;  // NOLINT
  const char * device = serial_port_.c_str();

  int result = urg_open(&urg_, URG_SERIAL, device, baudrate_or_port);
  if (result < 0) {
    std::stringstream ss;
    ss << "Could not open serial Hokuyo:\n";
    ss << serial_port_ << " @ " << serial_baud_ << "\n";
    ss << urg_error(&urg_);
    stop();
    urg_close(&urg_);
    throw std::runtime_error(ss.str());
  }

  initialize(using_intensity, using_multiecho);
}

void URGCWrapper::initialize(bool & using_intensity, bool & using_multiecho)
{
  int urg_data_size = urg_max_data_size(&urg_);
  // urg_max_data_size can return a negative, error code value.
  // Resizing based on this value will fail.
  if (urg_data_size < 0) {
    // This error can be caused by a URG-04LX in SCIP 1.1 mode, so we try to set SCIP 2.0 mode.
    if (setToSCIP2() && urg_max_data_size(&urg_) >= 0) {
      // If setting SCIP 2.0 was successful, we set urg_data_size to the correct value.
      urg_data_size = urg_max_data_size(&urg_);
    } else {
      urg_.last_errno = urg_data_size;
      std::stringstream ss;
      ss << "Could not initialize Hokuyo:\n";
      ss << urg_error(&urg_);
      stop();
      urg_close(&urg_);
      throw std::runtime_error(ss.str());
    }
  }
  // Ocassionally urg_max_data_size returns a string pointer,
  // make sure we don't allocate too much space,
  // the current known max is 1440 steps
  if (urg_data_size > 5000) {
    urg_data_size = 5000;
  }
  data_.resize(urg_data_size * URG_MAX_ECHO);
  intensity_.resize(urg_data_size * URG_MAX_ECHO);

  started_ = false;
  frame_id_ = "";
  first_step_ = 0;
  last_step_ = 0;
  cluster_ = 1;
  skip_ = 0;

  hardware_clock_ = 0.0;
  last_hardware_time_stamp_ = 0;
  hardware_clock_adj_ = 0.0;
  adj_count_ = 0;

  if (using_intensity) {
    using_intensity = isIntensitySupported();
  }

  if (using_multiecho) {
    using_multiecho = isMultiEchoSupported();
  }

  use_intensity_ = using_intensity;
  use_multiecho_ = using_multiecho;

  measurement_type_ = URG_DISTANCE;
  if (use_intensity_ && use_multiecho_) {
    measurement_type_ = URG_MULTIECHO_INTENSITY;
  } else if (use_intensity_) {
    measurement_type_ = URG_DISTANCE_INTENSITY;
  } else if (use_multiecho_) {
    measurement_type_ = URG_MULTIECHO;
  }
}

void URGCWrapper::start()
{
  if (!started_) {
    int result = urg_start_measurement(&urg_, measurement_type_, 0, skip_);
    if (result < 0) {
      std::stringstream ss;
      ss << "Could not start Hokuyo measurement:\n";
      if (use_intensity_) {
        ss << "With Intensity" << "\n";
      }
      if (use_multiecho_) {
        ss << "With MultiEcho" << "\n";
      }
      ss << urg_error(&urg_);
      throw std::runtime_error(ss.str());
    }
  }
  started_ = true;
}

void URGCWrapper::stop()
{
  urg_stop_measurement(&urg_);
  started_ = false;
}

URGCWrapper::~URGCWrapper()
{
  stop();
  urg_close(&urg_);
}

bool URGCWrapper::grabScan(sensor_msgs::msg::LaserScan & msg)
{
  msg.header.frame_id = frame_id_;
  msg.angle_min = getAngleMin();
  msg.angle_max = getAngleMax();
  msg.angle_increment = getAngleIncrement();
  msg.scan_time = getScanPeriod();
  msg.time_increment = getTimeIncrement();
  msg.range_min = getRangeMin();
  msg.range_max = getRangeMax();

  // Grab scan
  int num_beams = 0;
  long time_stamp = 0;  // NOLINT
  unsigned long long system_time_stamp = 0;  // NOLINT

  if (use_intensity_) {
    num_beams = urg_get_distance_intensity(
      &urg_, &data_[0], &intensity_[0], &time_stamp,
      &system_time_stamp);
  } else {
    num_beams = urg_get_distance(&urg_, &data_[0], &time_stamp, &system_time_stamp);
  }
  if (num_beams <= 0) {
    return false;
  }

  // Fill scan
  builtin_interfaces::msg::Time stampTime = rclcpp::Time(static_cast<int64_t>(system_time_stamp)) +
    system_latency_ + user_latency_ + getAngularTimeOffset();
  msg.header.stamp = stampTime;
  msg.ranges.resize(num_beams);

  if (use_intensity_) {
    msg.intensities.resize(num_beams);
  }

  for (int i = 0; i < num_beams; i++) {
    if (data_[(i) + 0] != 0) {
      msg.ranges[i] = static_cast<float>(data_[i]) / 1000.0;
      if (use_intensity_) {
        msg.intensities[i] = intensity_[i];
      }
    } else {
      msg.ranges[i] = std::numeric_limits<float>::quiet_NaN();
      continue;
    }
  }
  return true;
}

bool URGCWrapper::grabScan(sensor_msgs::msg::MultiEchoLaserScan & msg)
{
  msg.header.frame_id = frame_id_;
  msg.angle_min = getAngleMin();
  msg.angle_max = getAngleMax();
  msg.angle_increment = getAngleIncrement();
  msg.scan_time = getScanPeriod();
  msg.time_increment = getTimeIncrement();
  msg.range_min = getRangeMin();
  msg.range_max = getRangeMax();

  // Grab scan
  int num_beams = 0;
  long time_stamp = 0;  // NOLINT
  unsigned long long system_time_stamp;  // NOLINT

  if (use_intensity_) {
    num_beams = urg_get_multiecho_intensity(
      &urg_, &data_[0], &intensity_[0], &time_stamp,
      &system_time_stamp);
  } else {
    num_beams = urg_get_multiecho(&urg_, &data_[0], &time_stamp, &system_time_stamp);
  }
  if (num_beams <= 0) {
    return false;
  }

  // Fill scan
  // (uses vector.reserve wherever possible to avoid initalization and unecessary memory expansion)
  builtin_interfaces::msg::Time stampTime = rclcpp::Time(system_time_stamp) + system_latency_ +
    user_latency_ + getAngularTimeOffset();
  msg.header.stamp = stampTime;

  msg.ranges.reserve(num_beams);
  if (use_intensity_) {
    msg.intensities.reserve(num_beams);
  }

  for (int i = 0u; i < num_beams; i++) {
    sensor_msgs::msg::LaserEcho range_echo;
    range_echo.echoes.reserve(URG_MAX_ECHO);
    sensor_msgs::msg::LaserEcho intensity_echo;
    if (use_intensity_) {
      intensity_echo.echoes.reserve(URG_MAX_ECHO);
    }
    for (size_t j = 0; j < URG_MAX_ECHO; j++) {
      if (data_[(URG_MAX_ECHO * i) + j] != 0) {
        range_echo.echoes.push_back(static_cast<float>(data_[(URG_MAX_ECHO * i) + j]) / 1000.0f);
        if (use_intensity_) {
          intensity_echo.echoes.push_back(intensity_[(URG_MAX_ECHO * i) + j]);
        }
      } else {
        break;
      }
    }
    msg.ranges.push_back(range_echo);
    if (use_intensity_) {
      msg.intensities.push_back(intensity_echo);
    }
  }

  return true;
}

bool URGCWrapper::getAR00Status(URGStatus & status)
{
  // Construct and write AR00 command.
  std::string str_cmd;
  str_cmd += 0x02;  // STX
  str_cmd.append("000EAR00A012");  // AR00 cmd with length and checksum.
  str_cmd += 0x03;  // ETX

  // Get the response
  std::string response = sendCommand(str_cmd);

  if (response.empty()) {
    RCLCPP_WARN(logger_, "Received empty response from AR00 command");
    return false;
  }

  RCLCPP_DEBUG(logger_, "Full response: %s", response.c_str());

  // Strip STX and ETX before calculating the CRC.
  response.erase(0, 1);
  response.erase(response.size() - 1, 1);

  // Get the CRC, it's the last 4 chars.
  std::stringstream ss;
  ss << response.substr(response.size() - 4, 4);
  uint16_t crc;
  ss >> std::hex >> crc;

  // Remove the CRC from the check.
  std::string msg = response.substr(0, response.size() - 4);
  // Check the checksum.
  uint16_t checksum_result = checkCRC(msg.data(), msg.size());

  if (checksum_result != crc) {
    RCLCPP_WARN(logger_, "Received bad frame, incorrect checksum");
    return false;
  }

  // Debug output reponse up to scan data.
  RCLCPP_DEBUG(logger_, "Response: %s", response.substr(0, 41).c_str());
  // Decode the result if crc checks out.
  // Grab the status
  ss.clear();
  RCLCPP_DEBUG(logger_, "Status: %s", response.substr(8, 2).c_str());
  ss << response.substr(8, 2);  // Status is 8th position 2 chars.
  ss >> std::hex >> status.status;

  if (status.status != 0) {
    RCLCPP_WARN(logger_, "Received bad status");
    return false;
  }

  // Grab the operating mode
  ss.clear();
  RCLCPP_DEBUG(logger_, "Operating mode: %s", response.substr(10, 1).c_str());
  ss << response.substr(10, 1);
  ss >> std::hex >> status.operating_mode;

  // Grab the area number
  ss.clear();
  ss << response.substr(11, 2);
  RCLCPP_DEBUG(logger_, "Area Number: %s", response.substr(11, 2).c_str());
  ss >> std::hex >> status.area_number;
  // Per documentation add 1 to offset area number
  status.area_number++;

  // Grab the Error Status
  ss.clear();
  ss << response.substr(13, 1);
  RCLCPP_DEBUG(logger_, "Error status: %s", response.substr(13, 1).c_str());
  ss >> std::hex >> status.error_status;


  // Grab the error code
  ss.clear();
  ss << response.substr(14, 2);
  RCLCPP_DEBUG(logger_, "Error code: %s", response.substr(14, 2).c_str());
  ss >> std::hex >> status.error_code;
  // Offset by 0x40 is non-zero as per documentation
  if (status.error_code != 0) {
    status.error_code += 0x40;
  }

  // Get the lockout status
  ss.clear();
  ss << response.substr(16, 1);
  RCLCPP_DEBUG(logger_, "Lockout: %s", response.substr(16, 1).c_str());
  ss >> std::hex >> status.lockout_status;

  return true;
}

bool URGCWrapper::getDL00Status(UrgDetectionReport & report)
{
  // Construct and write DL00 command.
  std::string str_cmd;
  str_cmd += 0x02;  // STX
  str_cmd.append("000EDL005BCB");  // DL00 cmd with length and checksum.
  str_cmd += 0x03;  // ETX

  // Get the response
  std::string response = sendCommand(str_cmd);

  if (response.empty()) {
    RCLCPP_WARN(logger_, "Received empty response from DL00 command");
    return false;
  }

  RCLCPP_DEBUG(logger_, "Full response: %s", response.c_str());

  // Strip STX and ETX before calculating the CRC.
  response.erase(0, 1);
  response.erase(response.size() - 1, 1);

  // Get the CRC, it's the last 4 chars.
  std::stringstream ss;
  ss << response.substr(response.size() - 4, 4);
  uint16_t crc;
  ss >> std::hex >> crc;

  // Remove the CRC from the check.
  std::string msg = response.substr(0, response.size() - 4);
  // Check the checksum.
  uint16_t checksum_result = checkCRC(msg.data(), msg.size());

  if (checksum_result != crc) {
    RCLCPP_WARN(logger_, "Received bad frame, incorrect checksum");
    return false;
  }

  // Decode the result if crc checks out.
  // Grab the status
  uint16_t status = 0;
  ss.clear();
  RCLCPP_DEBUG(logger_, "Status: %s", response.substr(8, 2).c_str());
  ss << response.substr(8, 2);  // Status is 8th position 2 chars.
  ss >> std::hex >> status;

  if (status != 0) {
    RCLCPP_WARN(logger_, "Received bad status");
    return false;
  }

  std::vector<UrgDetectionReport> reports;
  msg = msg.substr(10);  // remove the header.
  // Process the message, there are 29 reports.
  // The 30th report is a circular buff marker of area with 0xFF
  // denoting the "last" report being the previous one.
  for (int i = 0; i < 30; i++) {
    uint16_t area = 0;
    uint16_t distance = 0;
    uint16_t step = 0;
    ss.clear();

    // Each msg is 64 chars long, offset which
    // report is being read in.
    uint16_t offset_pos = i * 64;
    ss << msg.substr(offset_pos, 2);  // Area is 2 chars long
    ss >> std::hex >> area;


    ss.clear();
    ss << msg.substr(offset_pos + 4, 4);  // Distance is offset 4 from beginning, 4 chars long.
    ss >> std::hex >> distance;

    ss.clear();
    ss << msg.substr(offset_pos + 8, 4);  // "Step" is offset 8 from beginning 4 chars long.
    ss >> std::hex >> step;
    RCLCPP_DEBUG(logger_, "%d Area: %d Distance: %d Step: %d", i, area, distance, step);

    UrgDetectionReport r;
    r.area = area;
    r.distance = distance;
    // From read value to angle of report is a value/8.
    r.angle = static_cast<float>(step) / 8.0;

    reports.push_back(r);
  }

  for (auto iter = reports.begin(); iter != reports.end(); ++iter) {
    // Check if value retrieved for area is FF.
    // if it is this is the last element lasers circular buffer.
    if (iter->area == 0xFF) {
      // Try and read the previous item.
      // if it's the beginning, then all reports
      // are empty.
      if (iter - 1 == reports.begin()) {
        RCLCPP_DEBUG(logger_, "All reports are empty, no detections available.");
        report.status = status;
        return false;
      }
      if (iter - 1 != reports.begin()) {
        report = *(iter - 1);
        report.area += 1;  // Final area is offset by 1.
        report.status = status;
        break;
      }
    }
  }

  return true;
}

bool URGCWrapper::setToSCIP2()
{
  if (urg_.connection.type == URG_ETHERNET) {
    return false;
  }

  char buffer[sizeof("SCIP2.0\n")];
  int n;

  do {
    n = serial_readline(&(urg_.connection.serial), buffer, sizeof(buffer), 1000);
  } while (n >= 0);

  serial_write(&(urg_.connection.serial), "SCIP2.0\n", sizeof(buffer));
  n = serial_readline(&(urg_.connection.serial), buffer, sizeof(buffer), 1000);

  // Check if switching was successful.
  if (n > 0 && strcmp(buffer, "SCIP2.0") == 0 &&
    urg_open(&urg_, URG_SERIAL, serial_port_.c_str(), (long)serial_baud_) >= 0)  // NOLINT
  {
    RCLCPP_DEBUG(logger_, "Set sensor to SCIP 2.0.");
    return true;
  }
  return false;
}

uint16_t URGCWrapper::checkCRC(const char * bytes, const uint32_t size)
{
  boost::crc_optimal<16, 0x1021, 0, 0, true, true> crc_kermit_type;
  crc_kermit_type.process_bytes(bytes, size);
  return crc_kermit_type.checksum();
}

std::string URGCWrapper::sendCommand(std::string cmd)
{
  std::string result;
  bool restart = false;

  if (isStarted()) {
    restart = true;
    // Scan must stop before sending a command
    stop();
  }

  // Get the socket reference and send
  int sock = urg_.connection.tcpclient.sock_desc;
  write(sock, cmd.c_str(), cmd.size());

  // All serial command structures start with STX + LEN as
  // the first 5 bytes, read those in.
  ssize_t total_read_len = 0;
  ssize_t read_len = 0;
  // Read in the header, make sure we get all 5 bytes expcted
  char recvb[5] = {0};
  ssize_t expected_read = 5;
  while (total_read_len < expected_read) {
    read_len = read(sock, recvb + total_read_len, expected_read - total_read_len);  // READ STX
    total_read_len += read_len;
    if (read_len <= 0) {
      RCLCPP_ERROR(logger_, "Read socket failed: %s", strerror(errno));
      result.clear();
      return result;
    }
  }

  std::string recv_header(recvb, read_len);
  // Convert the read len from hex chars to int.
  std::stringstream ss;
  ss << recv_header.substr(1, 4);
  ss >> std::hex >> expected_read;
  RCLCPP_DEBUG(logger_, "Read len: %lu bytes", expected_read);

  // Already read len of 5, take that out.
  uint32_t arr_size = expected_read - 5;
  // Bounds check the size, we really shouldn't exceed 8703 bytes
  // based on the currently known messages on the hokuyo documentations
  if (arr_size > 10000) {
    RCLCPP_ERROR(
      logger_, "Buffer creation bounds exceeded, shouldn't allocate: %" PRIu32 " bytes",
      arr_size);
    result.clear();
    return result;
  }

  RCLCPP_DEBUG(logger_, "Creating buffer read of arr_Size: %" PRIu32 " bytes", arr_size);
  // Create buffer space for read.
  auto data = std::make_unique<char[]>(arr_size);

  // Read the remaining command
  total_read_len = 0;
  read_len = 0;
  expected_read = arr_size;

  RCLCPP_DEBUG(logger_, "Expected body size: %lu bytes", expected_read);
  while (total_read_len < expected_read) {
    read_len = read(sock, data.get() + total_read_len, expected_read - total_read_len);
    total_read_len += read_len;
    RCLCPP_DEBUG(logger_, "Read in after header: %lu bytes", read_len);
    if (read_len <= 0) {
      RCLCPP_DEBUG(logger_, "Read socket failed: %s", strerror(errno));
      result.clear();
      return result;
    }
  }

  // Combine the read portions to return for processing.
  result += recv_header;
  result += std::string(data.get(), expected_read);

  // Resume scan after sending.
  if (restart) {
    start();
  }

  return result;
}

bool URGCWrapper::isStarted() const
{
  return started_;
}

double URGCWrapper::getRangeMin() const
{
  long minr;  // NOLINT
  long maxr;  // NOLINT
  urg_distance_min_max(&urg_, &minr, &maxr);
  return static_cast<double>(minr) / 1000.0;
}

double URGCWrapper::getRangeMax() const
{
  long minr;  // NOLINT
  long maxr;  // NOLINT
  urg_distance_min_max(&urg_, &minr, &maxr);
  return static_cast<double>(maxr) / 1000.0;
}

double URGCWrapper::getAngleMin() const
{
  return urg_step2rad(&urg_, first_step_);
}

double URGCWrapper::getAngleMax() const
{
  return urg_step2rad(&urg_, last_step_);
}

double URGCWrapper::getAngleMinLimit() const
{
  int min_step;
  int max_step;
  urg_step_min_max(&urg_, &min_step, &max_step);
  return urg_step2rad(&urg_, min_step);
}

double URGCWrapper::getAngleMaxLimit() const
{
  int min_step;
  int max_step;
  urg_step_min_max(&urg_, &min_step, &max_step);
  return urg_step2rad(&urg_, max_step);
}

double URGCWrapper::getAngleIncrement() const
{
  double angle_min = getAngleMin();
  double angle_max = getAngleMax();
  return cluster_ * (angle_max - angle_min) / static_cast<double>(last_step_ - first_step_);
}

double URGCWrapper::getScanPeriod() const
{
  long scan_usec = urg_scan_usec(&urg_);  // NOLINT
  return 1.e-6 * static_cast<double>(scan_usec);
}

double URGCWrapper::getTimeIncrement() const
{
  int min_step;
  int max_step;
  urg_step_min_max(&urg_, &min_step, &max_step);
  double scan_period = getScanPeriod();
  double circle_fraction = (getAngleMaxLimit() - getAngleMinLimit()) / (2.0 * 3.141592);
  return cluster_ * circle_fraction * scan_period / static_cast<double>(max_step - min_step);
}


std::string URGCWrapper::getIPAddress() const
{
  return ip_address_;
}

int URGCWrapper::getIPPort() const
{
  return ip_port_;
}

std::string URGCWrapper::getSerialPort() const
{
  return serial_port_;
}

int URGCWrapper::getSerialBaud() const
{
  return serial_baud_;
}

std::string URGCWrapper::getVendorName()
{
  return std::string(urg_sensor_vendor(&urg_));
}

std::string URGCWrapper::getProductName()
{
  return std::string(urg_sensor_product_type(&urg_));
}

std::string URGCWrapper::getFirmwareVersion()
{
  return std::string(urg_sensor_firmware_version(&urg_));
}

std::string URGCWrapper::getFirmwareDate()
{
  return std::string(urg_sensor_firmware_date(&urg_));
}

std::string URGCWrapper::getProtocolVersion()
{
  return std::string(urg_sensor_protocol_version(&urg_));
}

std::string URGCWrapper::getDeviceID()
{
  return std::string(urg_sensor_serial_id(&urg_));
}

rclcpp::Duration URGCWrapper::getComputedLatency() const
{
  return system_latency_;
}

rclcpp::Duration URGCWrapper::getUserLatency() const
{
  return user_latency_;
}

std::string URGCWrapper::getSensorStatus()
{
  return std::string(urg_sensor_status(&urg_));
}

std::string URGCWrapper::getSensorState()
{
  return std::string(urg_sensor_state(&urg_));
}

void URGCWrapper::setFrameId(const std::string & frame_id)
{
  frame_id_ = frame_id;
}

void URGCWrapper::setUserLatency(const double latency)
{
  user_latency_ = rclcpp::Duration(std::chrono::duration<double>(latency));
}

// Must be called before urg_start
bool URGCWrapper::setAngleLimitsAndCluster(double & angle_min, double & angle_max, int cluster)
{
  if (started_) {
    return false;  // Must not be streaming
  }

  // Set step limits
  first_step_ = urg_rad2step(&urg_, angle_min);
  last_step_ = urg_rad2step(&urg_, angle_max);
  cluster_ = cluster;

  // Make sure step limits are not the same
  if (first_step_ == last_step_) {
    // Make sure we're not at a limit
    int min_step;
    int max_step;
    urg_step_min_max(&urg_, &min_step, &max_step);
    if (first_step_ == min_step) {  // At beginning of range
      last_step_ = first_step_ + 1;
    } else {  // At end of range (or all other cases)
      first_step_ = last_step_ - 1;
    }
  }

  // Make sure angle_max is greater than angle_min (should check this after end limits)
  if (last_step_ < first_step_) {
    double temp = first_step_;
    first_step_ = last_step_;
    last_step_ = temp;
  }

  angle_min = urg_step2rad(&urg_, first_step_);
  angle_max = urg_step2rad(&urg_, last_step_);
  int result = urg_set_scanning_parameter(&urg_, first_step_, last_step_, cluster);
  if (result < 0) {
    return false;
  }
  return true;
}

void URGCWrapper::setSkip(int skip)
{
  skip_ = skip;
}

bool URGCWrapper::isIntensitySupported()
{
  if (started_) {
    return false;  // Must not be streaming
  }

  urg_start_measurement(&urg_, URG_DISTANCE_INTENSITY, 0, 0);
  int ret = urg_get_distance_intensity(&urg_, &data_[0], &intensity_[0], NULL, NULL);
  if (ret <= 0) {
    return false;  // Failed to start measurement with intensity: must not support it
  }
  urg_stop_measurement(&urg_);
  return true;
}

bool URGCWrapper::isMultiEchoSupported()
{
  if (started_) {
    return false;  // Must not be streaming
  }

  urg_start_measurement(&urg_, URG_MULTIECHO, 0, 0);
  int ret = urg_get_multiecho(&urg_, &data_[0], NULL, NULL);
  if (ret <= 0) {
    return false;  // Failed to start measurement with multiecho: must not support it
  }
  urg_stop_measurement(&urg_);
  return true;
}

rclcpp::Duration URGCWrapper::getAngularTimeOffset() const
{
  // Adjust value for Hokuyo's timestamps
  // Hokuyo's timestamps start from the rear center of the device (at Pi according to ROS standards)
  double circle_fraction = 0.0;
  if (first_step_ == 0 && last_step_ == 0) {
    circle_fraction = (getAngleMinLimit() + 3.141592) / (2.0 * 3.141592);
  } else {
    circle_fraction = (getAngleMin() + 3.141592) / (2.0 * 3.141592);
  }
  return rclcpp::Duration(std::chrono::duration<double>(circle_fraction * getScanPeriod()));
}

rclcpp::Duration URGCWrapper::computeLatency(size_t num_measurements)
{
  system_latency_ = rclcpp::Duration(std::chrono::seconds(0));

  rclcpp::Duration start_offset = getNativeClockOffset(1);
  rclcpp::Duration previous_offset(std::chrono::seconds(0));

  std::vector<rclcpp::Duration> time_offsets;
  for (size_t i = 0; i < num_measurements; i++) {
    rclcpp::Duration scan_offset = getTimeStampOffset(1);
    rclcpp::Duration post_offset = getNativeClockOffset(1);
    rclcpp::Duration adjusted_scan_offset = scan_offset - start_offset;
    rclcpp::Duration adjusted_post_offset = post_offset - start_offset;
    rclcpp::Duration average_offset(
      std::chrono::duration<double>(
        adjusted_post_offset.nanoseconds() / 2.0 +
        previous_offset.nanoseconds() / 2.0));

    time_offsets.push_back(adjusted_scan_offset - average_offset);

    previous_offset = adjusted_post_offset;
  }

  // Get median value
  // Sort vector using nth_element (partially sorts up to the median index)
  std::nth_element(
    time_offsets.begin(),
    time_offsets.begin() + time_offsets.size() / 2, time_offsets.end());
  system_latency_ = time_offsets[time_offsets.size() / 2];
  // Angular time offset makes the output comparable to that of hokuyo_node
  return system_latency_ + getAngularTimeOffset();
}

rclcpp::Duration URGCWrapper::getNativeClockOffset(size_t num_measurements)
{
  if (started_) {
    std::stringstream ss;
    ss << "Cannot get native clock offset while started.";
    throw std::runtime_error(ss.str());
  }

  if (urg_start_time_stamp_mode(&urg_) < 0) {
    std::stringstream ss;
    ss << "Cannot start time stamp mode.";
    throw std::runtime_error(ss.str());
  }

  std::vector<rclcpp::Duration> time_offsets;
  for (size_t i = 0; i < num_measurements; i++) {
    rclcpp::Time request_time(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count());
    double urg_ts = urg_time_stamp(&urg_);
    rclcpp::Time laser_time(1e6 * urg_ts);
    rclcpp::Time response_time(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count());
    rclcpp::Time average_time(response_time.nanoseconds() / 2.0 + request_time.nanoseconds() / 2.0);
    time_offsets.push_back(laser_time - average_time);
  }

  if (urg_stop_time_stamp_mode(&urg_) < 0) {
    std::stringstream ss;
    ss << "Cannot stop time stamp mode.";
    throw std::runtime_error(ss.str());
  }

  // Return median value
  // Sort vector using nth_element (partially sorts up to the median index)
  std::nth_element(
    time_offsets.begin(),
    time_offsets.begin() + time_offsets.size() / 2, time_offsets.end());
  return time_offsets[time_offsets.size() / 2];
}

rclcpp::Duration URGCWrapper::getTimeStampOffset(size_t num_measurements)
{
  if (started_) {
    std::stringstream ss;
    ss << "Cannot get time stamp offset while started.";
    throw std::runtime_error(ss.str());
  }

  start();

  std::vector<rclcpp::Duration> time_offsets;
  for (size_t i = 0; i < num_measurements; i++) {
    long time_stamp;  // NOLINT
    unsigned long long system_time_stamp;  // NOLINT
    int ret = 0;

    if (measurement_type_ == URG_DISTANCE) {
      ret = urg_get_distance(&urg_, &data_[0], &time_stamp, &system_time_stamp);
    } else if (measurement_type_ == URG_DISTANCE_INTENSITY) {
      ret = urg_get_distance_intensity(
        &urg_, &data_[0], &intensity_[0], &time_stamp,
        &system_time_stamp);
    } else if (measurement_type_ == URG_MULTIECHO) {
      ret = urg_get_multiecho(&urg_, &data_[0], &time_stamp, &system_time_stamp);
    } else if (measurement_type_ == URG_MULTIECHO_INTENSITY) {
      ret = urg_get_multiecho_intensity(
        &urg_, &data_[0], &intensity_[0], &time_stamp,
        &system_time_stamp);
    }

    if (ret <= 0) {
      std::stringstream ss;
      ss << "Cannot get scan to measure time stamp offset.";
      throw std::runtime_error(ss.str());
    }

    rclcpp::Time laser_timestamp(1e6 * time_stamp);
    rclcpp::Time system_time(system_time_stamp);

    time_offsets.push_back(laser_timestamp - system_time);
  }

  stop();

  // Return median value
  // Sort vector using nth_element (partially sorts up to the median index)
  std::nth_element(
    time_offsets.begin(),
    time_offsets.begin() + time_offsets.size() / 2, time_offsets.end());
  return time_offsets[time_offsets.size() / 2];
}
}  // namespace urg_node

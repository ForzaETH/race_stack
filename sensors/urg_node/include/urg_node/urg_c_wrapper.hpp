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
 * Author: Chad Rockey
 */

#ifndef URG_NODE__URG_C_WRAPPER_HPP_
#define URG_NODE__URG_C_WRAPPER_HPP_

#include <chrono>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/multi_echo_laser_scan.hpp"

#include "urg_c/urg_sensor.h"
#include "urg_c/urg_utils.h"

namespace urg_node
{

class URGStatus
{
public:
  URGStatus()
  {
    status = 0;
    operating_mode = 0;
    area_number = 0;
    error_status = false;
    error_code = 0;
    lockout_status = false;
  }

  uint16_t status;
  uint16_t operating_mode;
  uint16_t area_number;
  bool error_status;
  uint16_t error_code;
  bool lockout_status;
};

class UrgDetectionReport
{
public:
  UrgDetectionReport()
  {
    status = 0;
    area = 0;
    distance = 0;
    angle = 0;
  }

  uint16_t status;
  uint16_t area;
  uint16_t distance;
  float angle;
};

struct EthernetConnection
{
  std::string ip_address;
  int ip_port;
};

struct SerialConnection
{
  std::string serial_port;
  int serial_baud;
};

class URGCWrapper
{
public:
  URGCWrapper(
    const EthernetConnection & connection,
    bool & using_intensity, bool & using_multiecho,
    const rclcpp::Logger & logger = rclcpp::get_logger("urg_c_wrapper"));

  URGCWrapper(
    const SerialConnection & connection,
    bool & using_intensity, bool & using_multiecho,
    const rclcpp::Logger & logger = rclcpp::get_logger("urg_c_wrapper"));

  ~URGCWrapper();

  void start();

  void stop();

  bool isStarted() const;

  double getRangeMin() const;

  double getRangeMax() const;

  double getAngleMin() const;

  double getAngleMax() const;

  double getAngleMinLimit() const;

  double getAngleMaxLimit() const;

  double getAngleIncrement() const;

  double getScanPeriod() const;

  double getTimeIncrement() const;

  std::string getIPAddress() const;

  int getIPPort() const;

  std::string getSerialPort() const;

  int getSerialBaud() const;

  std::string getVendorName();

  std::string getProductName();

  std::string getFirmwareVersion();

  std::string getFirmwareDate();

  std::string getProtocolVersion();

  std::string getDeviceID();

  rclcpp::Duration getComputedLatency() const;

  rclcpp::Duration getUserLatency() const;

  std::string getSensorStatus();

  std::string getSensorState();

  void setFrameId(const std::string & frame_id);

  void setUserLatency(const double latency);

  bool setAngleLimitsAndCluster(double & angle_min, double & angle_max, int cluster);

  void setSkip(int skip);

  rclcpp::Duration computeLatency(size_t num_measurements);

  bool grabScan(sensor_msgs::msg::LaserScan & msg);

  bool grabScan(sensor_msgs::msg::MultiEchoLaserScan & msg);

  bool getAR00Status(URGStatus & status);

  bool getDL00Status(UrgDetectionReport & report);

private:
  void initialize(bool & using_intensity, bool & using_multiecho);

  bool isIntensitySupported();

  bool isMultiEchoSupported();

  rclcpp::Duration getAngularTimeOffset() const;

  rclcpp::Duration getNativeClockOffset(size_t num_measurements);

  rclcpp::Duration getTimeStampOffset(size_t num_measurements);

  /**
   * @brief Set the Hokuyo URG-04LX from SCIP 1.1 mode to SCIP 2.0 mode.
   * @returns True if successful and false if not.
   */
  bool setToSCIP2();

  /**
   * @brief calculate the crc of a given set of bytes.
   * @param bytes The bytes array to be processed.
   * @param size The size of the bytes array.
   * @return the calculated CRC of the bytes.
   */
  uint16_t checkCRC(const char * bytes, const uint32_t size);

  /**
   * @brief Send an arbitrary serial command to the lidar. These commands
   * can also be sent via the ethernet socket.
   * @param cmd The arbitrary command fully formatted to be sent as provided
   * @returns The textual response of the Lidar, empty if, but may return lidar's own error string.
   */
  std::string sendCommand(std::string cmd);

  std::string ip_address_;
  int ip_port_;
  std::string serial_port_;
  int serial_baud_;

  std::string frame_id_;  ///< Output frame_id for each laserscan.

  urg_t urg_;
  bool started_;

  // TODO(karsten1987): Verify the real data type of this
  // cppcheck complains that `long` isn't type safe.
  // ignoring this check for now given that this requires changes in urg_c as well.
  std::vector<long> data_;  // NOLINT
  std::vector<unsigned short> intensity_;  // NOLINT

  bool use_intensity_;
  bool use_multiecho_;
  urg_measurement_type_t measurement_type_;
  int first_step_;
  int last_step_;
  int cluster_;
  int skip_;

  rclcpp::Duration system_latency_;
  rclcpp::Duration user_latency_;

  double hardware_clock_;
  long last_hardware_time_stamp_;  // NOLINT
  double hardware_clock_adj_;
  const double adj_alpha_ = .01;
  uint64_t adj_count_;

  /// Logger object used for debug info
  rclcpp::Logger logger_;
};
}  // namespace urg_node

#endif  // URG_NODE__URG_C_WRAPPER_HPP_

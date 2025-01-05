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

#ifndef MICROSTRAIN_INERTIAL_DRIVER_COMMON_UTILS_MIP_ROS_CONNECTION_H
#define MICROSTRAIN_INERTIAL_DRIVER_COMMON_UTILS_MIP_ROS_CONNECTION_H

#include <vector>
#include <string>
#include <memory>
#include <fstream>

#include "mip/mip_device.hpp"

#include "microstrain_inertial_driver_common/utils/ros_compat.h"

namespace microstrain
{

// Predeclare here so we can pass as a parameter to the later configure step
class RosMipDevice;

/**
 * ROS implementation of the MIP connection class
 */
class RosConnection : public mip::Connection
{
 public:
  /**
   * \brief Constructs the ROS connection given a reference to the node that initialized it.
   * \param node Reference to the node that is initializing the connection
   */
  explicit RosConnection(RosNodeType* node);

  /**
   * \brief Tests if the connection is connected
   * \return true if the connection is connected
  */
  bool isConnected() const final;

  /**
   * \brief Connects this connection to the device
   * \return true if the connection succeeds
  */
  bool connect() final;

  /**
   * \brief Disconnects the connection from the device
   * \return true if the connection is disconnected
  */
  bool disconnect() final;

  /**
   * \brief Connects to the MIP device. When this function is finished, if it returns true, the device will be open, but not necesarilly ready to communicate
   * \param config_node Reference to a ROS node object that contains configuration information
   * \param port The serial port to connect to
   * \param baudrate The baudrate to open the serial port at
   * \return true if the connection was successful and false otherwise
   */
  bool connect(RosNodeType* config_node, const std::string& port, const int32_t baudrate);

  /**
   * \brief Configures the RosConnection object. This should be called after connect
   * \param config_node Reference to a ROS node object that contains configuration information
   * \param device The device to use to read information from the device
   * \return true if the configuration was successful and false otherwise
   */
  bool configure(RosNodeType* config_node, RosMipDevice* device);

  /**
   * \brief Gets whether or not this connection is parsing NMEA
   * \return Whether or not this connection is parsing NMEA
  */
  bool shouldParseNmea() const;

  /**
   * \brief Configures the connection object to attempt to parse or not attempt to parse NMEA sentences
   * \param enable Whether or not to enable NMEA parsing
  */
  void shouldParseNmea(bool enable);

  /**
   * \brief Returns the parse timeout for this connection object
   * \return the parse timeout to use with this connection object
   */
  mip::Timeout parseTimeout() const;

  /**
   * \brief Returns the base reply timeout for this connection object
   * \return the base reply timeout to use with this connection object
   */
  mip::Timeout baseReplyTimeout() const;

  /**
   * \brief Returns the NMEA messages collected by the connection, and clears the list of messages on the connection object
   * \return List of NMEA messages collected by the connection
   */
  std::vector<NMEASentenceMsg> nmeaMsgs();

  /**
   * \brief Gets whether we are recording to a raw file
   * \return Whether the raw file is enabled
   */
  bool rawFileEnable();

  /**
   * \brief Gets the raw file path where data is being recorded
   * \return Empty string if data is not being recorded
   */
  std::string rawFilePath();

  /**
   * \brief Updates the recording state, so will either start, or stop recording or switch files if needed
   * \param should_record Whether or not we should be recording data
   * \param record_file_path The full path to where the data should be recorded
   * \return Whether or not the state was able to be updated
   */
  bool updateRecordingState(const bool should_record, const std::string& record_file_path);

  // Implemented in order to satisfy the requirements for the MIP connection
  bool sendToDevice(const uint8_t* data, size_t length) final;
  bool recvFromDevice(uint8_t* buffer, size_t max_length, mip::Timeout timeout, size_t* count_out, mip::Timestamp* timestamp_out) final;
  const char* interfaceName() const final;
  uint32_t parameter() const final;

 private:
  /**
   * \brief Extracts NMEA data from a byte array
   * \param data  Raw bytes that may contain a NMEA sentence
   * \param data_len  Length of the data array in bytes
   */
  void extractNmea(const uint8_t* data, size_t data_len);

  RosNodeType* node_;  /// Reference to the ROS node that created this connection

  std::unique_ptr<mip::Connection> connection_;  /// Connection object used to actually interact with the device
  mip::Timeout parse_timeout_;  /// Parse timeout given the type of connection configured
  mip::Timeout base_reply_timeout_;  /// Base reply timeout given the type of connection configured

  bool should_record_;  /// Whether or not we should record binary data on this connection
  std::string record_file_path_;  /// The path to where data will be recorded
  std::ofstream record_file_;  /// The file that the binary data should be recorded to

  bool should_parse_nmea_;  /// Whether or not we should attempt to parse and extract NMEA sentences on this connection
  std::string nmea_string_;  /// Cached data read from the port, used to extraxt NMEA messages
  std::vector<NMEASentenceMsg> nmea_msgs_;  /// List of NMEA messages received by this connection
};

}  // namespace microstrain

#endif  // MICROSTRAIN_INERTIAL_DRIVER_COMMON_UTILS_MIP_ROS_CONNECTION_H

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
 * Author: Chad Rockey, Michael Carroll, Mike O'Driscoll
 */

#include "urg_node/urg_node.hpp"

#include <memory>
#include <string>
#include <vector>

namespace urg_node
{

// Useful typedefs
typedef diagnostic_updater::FrequencyStatusParam FrequencyStatusParam;

UrgNode::UrgNode(const rclcpp::NodeOptions & node_options)
: Node("urg_node", node_options),
  diagnostic_updater_(this),
  error_code_(0),
  error_count_(0),
  error_limit_(4),
  lockout_status_(false),
  system_latency_(std::chrono::seconds(0)),
  user_latency_(std::chrono::seconds(0)),
  is_started_(false),
  close_diagnostics_(true),
  close_scan_(true),
  ip_address_(""),
  ip_port_(10940),
  serial_port_("/dev/cu.usbmodem141101"),
  serial_baud_(115200),
  calibrate_time_(false),
  publish_intensity_(false),
  publish_multiecho_(false),
  diagnostics_tolerance_(0.05),
  diagnostics_window_time_(5.0),
  detailed_status_(false),
  angle_min_(-3.14),
  angle_max_(3.14),
  cluster_(1),
  skip_(0),
  default_user_latency_(0.0),
  laser_frame_id_("laser"),
  service_yield_(true)
{
  (void) synchronize_time_;
  initSetup();
}

void UrgNode::initSetup()
{
  // Declare parameters so we can change these later.
  ip_address_ = this->declare_parameter<std::string>("ip_address", ip_address_);
  ip_port_ = this->declare_parameter<int>("ip_port", ip_port_);
  laser_frame_id_ = this->declare_parameter<std::string>("laser_frame_id", laser_frame_id_);
  serial_port_ = this->declare_parameter<std::string>("serial_port", serial_port_);
  serial_baud_ = this->declare_parameter<int>("serial_baud", serial_baud_);
  calibrate_time_ = this->declare_parameter<bool>("calibrate_time", calibrate_time_);
  publish_intensity_ = this->declare_parameter<bool>("publish_intensity", publish_intensity_);
  publish_multiecho_ = this->declare_parameter<bool>("publish_multiecho", publish_multiecho_);
  error_limit_ = this->declare_parameter<int>("error_limit", error_limit_);
  diagnostics_tolerance_ = this->declare_parameter<double>(
    "diagnostics_tolerance",
    diagnostics_tolerance_);
  diagnostics_window_time_ = this->declare_parameter<double>(
    "diagnostics_window_time",
    diagnostics_window_time_);
  detailed_status_ = this->declare_parameter<bool>("get_detailed_status", detailed_status_);
  default_user_latency_ = this->declare_parameter<double>(
    "default_user_latency",
    default_user_latency_);
  angle_min_ = this->declare_parameter<double>("angle_min", angle_min_);
  angle_max_ = this->declare_parameter<double>("angle_max", angle_max_);
  skip_ = this->declare_parameter<int>("skip", skip_);
  cluster_ = this->declare_parameter<int>("cluster", cluster_);

  // Set up publishers and diagnostics updaters, we only need one
  if (publish_multiecho_) {
    echoes_pub_ =
      std::make_unique<laser_proc::LaserPublisher>(this->get_node_topics_interface(), 20);
  } else {
    laser_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 20);
  }

  status_service_ = this->create_service<std_srvs::srv::Trigger>(
    "update_laser_status",
    std::bind(
      &UrgNode::statusCallback, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  // TODO(karsten1987): ros2 does not have latched topics yet, need to play with QoS
  status_pub_ = this->create_publisher<urg_node_msgs::msg::Status>("laser_status", 1);

  diagnostic_updater_.add("Hardware Status", this, &UrgNode::populateDiagnosticsStatus);

  parameters_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(
      &UrgNode::param_change_callback, this,
      std::placeholders::_1));

  // Put this in a separate thread since it might take a while to connect
  run_thread_ = std::thread(std::bind(&UrgNode::run, this));
}

UrgNode::~UrgNode()
{
  if (run_thread_.joinable()) {
    run_thread_.join();
  }
  if (diagnostics_thread_.joinable()) {
    // Clean up our diagnostics thread.
    close_diagnostics_ = true;
    diagnostics_thread_.join();
  }
  if (scan_thread_.joinable()) {
    close_scan_ = true;
    scan_thread_.join();
  }
}

bool UrgNode::updateStatus()
{
  bool result = false;
  service_yield_ = true;
  std::unique_lock<std::mutex> lock(lidar_mutex_);

  if (urg_) {
    device_status_ = urg_->getSensorStatus();

    if (detailed_status_) {
      URGStatus status;
      if (urg_->getAR00Status(status)) {
        urg_node_msgs::msg::Status msg;
        msg.operating_mode = status.operating_mode;
        msg.error_status = status.error_status;
        msg.error_code = status.error_code;
        msg.lockout_status = status.lockout_status;

        lockout_status_ = status.lockout_status;
        error_code_ = status.error_code;

        UrgDetectionReport report;
        if (urg_->getDL00Status(report)) {
          msg.area_number = report.area;
          msg.distance = report.distance;
          msg.angle = report.angle;
        } else {
          RCLCPP_WARN(this->get_logger(), "Failed to get detection report.");
        }
        // Publish the status on the latched topic for inspection.
        status_pub_->publish(msg);
        result = true;
      } else {
        RCLCPP_WARN(this->get_logger(), "Failed to retrieve status");
        urg_node_msgs::msg::Status msg;
        status_pub_->publish(msg);
      }
    }
  }
  return result;
}

void UrgNode::statusCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std_srvs::srv::Trigger::Request::SharedPtr req,
  const std_srvs::srv::Trigger::Response::SharedPtr res)
{
  (void) request_header;
  (void) req;

  RCLCPP_INFO(this->get_logger(), "Got update lidar status callback");
  res->success = false;
  res->message = "Laser not ready";

  if (updateStatus()) {
    res->message = "Status retrieved";
    res->success = true;
  } else {
    res->message = "Failed to update status";
    res->success = false;
  }
}

rcl_interfaces::msg::SetParametersResult UrgNode::param_change_callback(
  const std::vector<rclcpp::Parameter> parameters)
{
  auto result = rcl_interfaces::msg::SetParametersResult();
  result.successful = true;

  // Will concat a string explaining why it failed,
  // should only be used for logging and user interfaces.
  std::stringstream string_result;

  for (auto parameter : parameters) {
    rclcpp::ParameterType parameter_type = parameter.get_type();

    if (parameter.get_name().compare("laser_frame_id") == 0) {
      if (parameter_type == rclcpp::ParameterType::PARAMETER_STRING) {
        result.successful &= true;
      } else {
        string_result << "The parameter " << parameter.get_name() <<
          " is of the wrong type, should be a string.\n";
        result.successful = false;
      }

    } else if (parameter.get_name().compare("error_limit") == 0) {
      if (parameter_type == rclcpp::ParameterType::PARAMETER_INTEGER) {
        // TODO(tbd) check if value = 0 is okay
        if (parameter.as_int() >= 0) {
          result.successful &= true;
        } else {
          string_result << "The parameter " << parameter.get_name() << " should be > 0.\n";
          result.successful = false;
        }
      } else {
        string_result << "The parameter " << parameter.get_name() <<
          " is of the wrong type, should be an integer.\n";
        result.successful = false;
      }

    } else if (parameter.get_name().compare("default_user_latency") == 0) {
      if (parameter_type == rclcpp::ParameterType::PARAMETER_INTEGER ||
        parameter_type == rclcpp::ParameterType::PARAMETER_DOUBLE)
      {
        result.successful &= true;
      } else {
        string_result << "The parameter " << parameter.get_name() <<
          " is of the wrong type, should be an integer or a double.\n";
        result.successful = false;
      }

    } else if (parameter.get_name().compare("angle_min") == 0) {
      if (parameter_type == rclcpp::ParameterType::PARAMETER_INTEGER ||
        parameter_type == rclcpp::ParameterType::PARAMETER_DOUBLE)
      {
        result.successful &= true;
      } else {
        string_result << "The parameter " << parameter.get_name() <<
          " is of the wrong type, should be an integer or a double.\n";
        result.successful = false;
      }

    } else if (parameter.get_name().compare("angle_max") == 0) {
      if (parameter_type == rclcpp::ParameterType::PARAMETER_INTEGER ||
        parameter_type == rclcpp::ParameterType::PARAMETER_DOUBLE)
      {
        result.successful &= true;
      } else {
        string_result << "The parameter " << parameter.get_name() <<
          " is of the wrong type, should be an integer or a double.\n";
        result.successful = false;
      }

    } else if (parameter.get_name().compare("cluster") == 0) {
      if (parameter_type == rclcpp::ParameterType::PARAMETER_INTEGER) {
        if (parameter.as_int() >= 1 && parameter.as_int() <= 99) {
          result.successful &= true;
        } else {
          string_result << "The parameter " << parameter.get_name() <<
            " should be between 1 and 99.\n";
          result.successful = false;
        }
      } else {
        string_result << "The parameter " << parameter.get_name() <<
          " is of the wrong type, should be an integer.\n";
        result.successful = false;
      }

    } else if (parameter.get_name().compare("skip") == 0) {
      if (parameter_type == rclcpp::ParameterType::PARAMETER_INTEGER) {
        if (parameter.as_int() >= 0 && parameter.as_int() <= 9) {
          result.successful &= true;
        } else {
          string_result << "The parameter " << parameter.get_name() <<
            " should be between 0 and 9.\n";
          result.successful = false;
        }
      } else {
        string_result << "The parameter " << parameter.get_name() <<
          " is of the wrong type, should be an integer.\n";
        result.successful = false;
      }
    }
  }
  result.reason = string_result.str();

  return result;
}

void UrgNode::calibrate_time_offset()
{
  std::unique_lock<std::mutex> lock(lidar_mutex_);
  if (!urg_) {
    RCLCPP_DEBUG(this->get_logger(), "Unable to calibrate time offset. Not Ready.");
    return;
  }
  try {
    // Don't let outside interruption effect lidar offset.
    RCLCPP_INFO(this->get_logger(), "Starting calibration. This will take a few seconds.");
    RCLCPP_WARN(this->get_logger(), "Time calibration is still experimental.");
    rclcpp::Duration latency = urg_->computeLatency(10);
    system_latency_ = urg_->getComputedLatency();
    user_latency_ = urg_->getUserLatency();
    RCLCPP_INFO(
      this->get_logger(), "Calibration finished. Latency is: %.4f sec.",
      (double)(latency.nanoseconds() * 1e-9));
  } catch (const std::runtime_error & e) {
    RCLCPP_FATAL(this->get_logger(), "Could not calibrate time offset: %s", e.what());
    throw e;
  }
}

// Diagnostics update task to be run in a thread.
void UrgNode::updateDiagnostics()
{
  while (!close_diagnostics_) {
    diagnostic_updater_.force_update();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
}

// Populate a diagnostics status message.
void UrgNode::populateDiagnosticsStatus(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (!urg_) {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      "Not Connected");
    return;
  }

  if (!ip_address_.empty()) {
    stat.add("IP Address", ip_address_);
    stat.add("IP Port", ip_port_);
  } else {
    stat.add("Serial Port", serial_port_);
    stat.add("Serial Baud", serial_baud_);
  }

  if (!is_started_) {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      "Not Connected: " + device_status_);
  } else if (device_status_ != std::string("Sensor works well.") &&  //NOLINT
    device_status_ != std::string("Stable 000 no error.") &&
    device_status_ != std::string("sensor is working normally"))
  {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      "Abnormal status: " + device_status_);
  } else if (error_code_ != 0) {
    stat.summaryf(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      "Lidar reporting error code: %X",
      error_code_);
  } else if (lockout_status_) {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      "Lidar locked out.");
  } else {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::OK,
      "Streaming");
  }

  stat.add("Vendor Name", vendor_name_);
  stat.add("Product Name", product_name_);
  stat.add("Firmware Version", firmware_version_);
  stat.add("Firmware Date", firmware_date_);
  stat.add("Protocol Version", protocol_version_);
  stat.add("Device ID", device_id_);
  stat.add("Computed Latency", system_latency_.nanoseconds());
  stat.add("User Time Offset", user_latency_.nanoseconds());

  // Things not explicitly required by REP-0138, but still interesting.
  stat.add("Device Status", device_status_);
  stat.add("Scan Retrieve Error Count", error_count_);

  stat.add("Lidar Error Code", error_code_);
  stat.add("Locked out", lockout_status_);
}

bool UrgNode::connect()
{
  // Don't let external access to retrieve
  // status during the connection process.
  std::unique_lock<std::mutex> lock(lidar_mutex_);

  try {
    urg_.reset();  // Clear any previous connections();
    if (!ip_address_.empty()) {
      EthernetConnection connection{ip_address_, ip_port_};
      urg_.reset(
        new urg_node::URGCWrapper(
          connection,
          publish_intensity_, publish_multiecho_, this->get_logger()));
    } else {
      SerialConnection connection{serial_port_, serial_baud_};
      urg_.reset(
        new urg_node::URGCWrapper(
          connection,
          publish_intensity_, publish_multiecho_, this->get_logger()));
    }

    std::stringstream ss;
    ss << "Connected to";
    if (publish_multiecho_) {
      ss << " multiecho";
    }
    if (!ip_address_.empty()) {
      ss << " network";
    } else {
      ss << " serial";
    }
    ss << " device with";
    if (publish_intensity_) {
      ss << " intensity and";
    }
    ss << " ID: " << urg_->getDeviceID();
    RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());

    device_status_ = urg_->getSensorStatus();
    vendor_name_ = urg_->getVendorName();
    product_name_ = urg_->getProductName();
    firmware_version_ = urg_->getFirmwareVersion();
    firmware_date_ = urg_->getFirmwareDate();
    protocol_version_ = urg_->getProtocolVersion();
    device_id_ = urg_->getDeviceID();

    if (urg_) {
      diagnostic_updater_.setHardwareID(urg_->getDeviceID());
    }

    // Configure initial properties (in place of initial dynamic reconfigure)

    // The publish frequency changes based on the number of skipped scans.
    // Update accordingly here.
    freq_min_ = 1.0 / (urg_->getScanPeriod() * (skip_ + 1));

    urg_->setAngleLimitsAndCluster(angle_min_, angle_max_, cluster_);
    urg_->setSkip(skip_);

    urg_->setFrameId(laser_frame_id_);
    urg_->setUserLatency(default_user_latency_);

    return true;
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR(this->get_logger(), "Error connecting to Hokuyo: %s", e.what());
    urg_.reset();
    return false;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Unknown error connecting to Hokuyo: %s", e.what());
    urg_.reset();
    return false;
  }

  return false;
}

void UrgNode::scanThread()
{
  while (!close_scan_) {
    if (!urg_) {
      if (!connect()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        continue;  // Connect failed, sleep, try again.
      }
    }

    if (calibrate_time_) {
      calibrate_time_offset();
    }

    if (!urg_ || !rclcpp::ok()) {
      continue;
    }

    // Before starting, update the status
    updateStatus();

    // Start the urgwidget
    try {
      // If the connection failed, don't try and connect
      // pointer is invalid.
      if (!urg_) {
        continue;  // Return to top of main loop, not connected.
      }
      device_status_ = urg_->getSensorStatus();
      urg_->start();
      is_started_ = true;
      RCLCPP_INFO(this->get_logger(), "Streaming data.");
      // Clear the error count.
      error_count_ = 0;
    } catch (const std::runtime_error & e) {
      RCLCPP_ERROR(this->get_logger(), "Error starting Hokuyo: %s", e.what());
      urg_.reset();
      rclcpp::sleep_for(std::chrono::seconds(1));
      continue;  // Return to top of main loop
    } catch (...) {
      RCLCPP_ERROR(this->get_logger(), "Unknown error starting Hokuyo");
      urg_.reset();
      rclcpp::sleep_for(std::chrono::seconds(1));
      continue;  // Return to top of main loop
    }

    while (!close_scan_) {
      // Don't allow external access during grabbing the scan.
      try {
        std::unique_lock<std::mutex> lock(lidar_mutex_);
        is_started_ = urg_->isStarted();
        if (publish_multiecho_) {
          sensor_msgs::msg::MultiEchoLaserScan msg;
          if (urg_->grabScan(msg)) {
            echoes_pub_->publish(msg);
            echoes_freq_->tick();
          } else {
            RCLCPP_WARN(this->get_logger(), "Could not grab multi echo scan.");
            device_status_ = urg_->getSensorStatus();
            error_count_++;
          }
        } else {
          sensor_msgs::msg::LaserScan msg;
          if (urg_->grabScan(msg)) {
            laser_pub_->publish(msg);
            laser_freq_->tick();
          } else {
            RCLCPP_WARN(this->get_logger(), "Could not grab single echo scan.");
            device_status_ = urg_->getSensorStatus();
            error_count_++;
          }
        }
      } catch (...) {
        RCLCPP_WARN(this->get_logger(), "Unknown error grabbing Hokuyo scan.");
        error_count_++;
      }

      if (service_yield_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        service_yield_ = false;
      }

      // Reestablish connection if things seem to have gone wrong.
      if (error_count_ > error_limit_) {
        RCLCPP_ERROR(this->get_logger(), "Error count exceeded limit, reconnecting.");
        urg_.reset();
        rclcpp::sleep_for(std::chrono::seconds(2));

        break;  // Return to top of main loop
      }
    }
  }
}

void UrgNode::run()
{
  // Setup initial connection
  connect();

  // Stop diagnostics
  if (!close_diagnostics_) {
    close_diagnostics_ = true;
    diagnostics_thread_.join();
  }

  if (publish_multiecho_) {
    echoes_freq_.reset(
      new diagnostic_updater::HeaderlessTopicDiagnostic(
        "Laser Echoes",
        diagnostic_updater_,
        FrequencyStatusParam(
          &freq_min_, &freq_min_, diagnostics_tolerance_,
          diagnostics_window_time_)));
  } else {
    laser_freq_.reset(
      new diagnostic_updater::HeaderlessTopicDiagnostic(
        "Laser Scan",
        diagnostic_updater_,
        FrequencyStatusParam(
          &freq_min_, &freq_min_, diagnostics_tolerance_,
          diagnostics_window_time_)));
  }

  //// Now that we are setup, kick off diagnostics.
  close_diagnostics_ = false;
  diagnostics_thread_ = std::thread(std::bind(&UrgNode::updateDiagnostics, this));

  // Start scanning now that everything is configured.
  close_scan_ = false;
  scan_thread_ = std::thread(std::bind(&UrgNode::scanThread, this));
}
}  // namespace urg_node

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(urg_node::UrgNode)

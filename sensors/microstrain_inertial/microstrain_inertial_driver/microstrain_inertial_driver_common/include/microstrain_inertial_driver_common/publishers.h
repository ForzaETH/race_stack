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
#ifndef MICROSTRAIN_INERTIAL_DRIVER_COMMON_PUBLISHERS_H
#define MICROSTRAIN_INERTIAL_DRIVER_COMMON_PUBLISHERS_H

#include <map>
#include <memory>
#include <vector>
#include <string>

#include <Eigen/Geometry>

#include "mip/mip_all.hpp"

#include "microstrain_inertial_driver_common/utils/ros_compat.h"
#include "microstrain_inertial_driver_common/utils/clock_bias_monitor.h"
#include "microstrain_inertial_driver_common/config.h"

namespace microstrain
{

/**
 * Contains ROS messages and the publishers that will publish them
 */
class Publishers
{
public:
  /**
   * \brief Default Constructor
   */
  Publishers() = default;

  /**
   * \brief Constructs this class with a reference to the node, and a config object
   * \param node  Reference to a node that will be saved to this class and used to log and interact with ROS
   * \param config Reference to the config object that will be saved to this class and used to determine whether or not to publish
   */
  Publishers(RosNodeType* node, Config* config);

  /**
   * \brief Configures the publishers. After this function is called, the publishers will be created, but (ROS2 only) will not be activated
   * \return true if configuration was successful and false if configuration failed
   */
  bool configure();

  /**
   * \brief Only technically needed for ROS2 lifecycle publishers, this will activate the publishers, so they are ready to be published on
   * \return true if activation was successful and false if activation failed
   */
  bool activate();

  /**
   * \brief Only technically needed for ROS2 lifecycle publishers, this will deactivate the publishers, so they can be reconfigured and reactivated
   * \return true if deactivation was successful and false if deactivation failed
   */
  bool deactivate();

  /**
   * \brief Publishes all messages that have changed and are configured to be published
   */
  void publish();

  /**
   * Wrapper for a publisher
   * @tparam MessageType The type of ROS message that this publisher will publish
   */
  template<typename MessageType>
  class Publisher
  {
   public:
    using SharedPtr = std::shared_ptr<Publisher<MessageType>>;
    using SharedPtrVec = std::vector<SharedPtr>;

    /**
     * \brief Constructs the publisher wrapper given a topic
     * \param topic The topic that this publisher will publish to
     */
    explicit Publisher(const std::string& topic) : topic_(topic)
    {
      message_ = std::make_shared<MessageType>();
    }

    /**
     * \brief Helper function to initialize a shared pointer of this publisher type
     * \param topic The topic that this publisher will publish to
     * \return An instance of this object in a shared pointer
     */
    static SharedPtr initialize(const std::string& topic)
    {
      return std::make_shared<Publisher<MessageType>>(topic);
    }

    /**
     * \brief Helper function to initialize a vector of shared pointers of this publisher type
     * \param topics The topics that this publishers will publish to
     * \return Vector of the same length as topics of publishers of this type
     */
    static SharedPtrVec initializeVec(const std::vector<std::string>& topics)
    {
      SharedPtrVec ptrs;
      for (const auto& topic : topics)
        ptrs.push_back(initialize(topic));
      return ptrs;
    }

    /**
     * \brief Configures this publisher by creating a ROS publisher. This function is used when the topic does not have a mapping in MipPublisherMapping
     * \param node The node to initialize the publisher on
     */
    void configure(RosNodeType* node)
    {
      publisher_ = createPublisher<MessageType>(node, topic_, 100);
    }

    /**
     * \brief Configures the publisher by creating a ROS publisher. The publisher will only be created if the publisher mapping in config says that we should publish the topic
     * \param node The node to initialize the publisher on
     * \param config Configuration object to use to look up if we should publish this topic
     */
    void configure(RosNodeType* node, Config* config)
    {
      data_rate_ = config->mip_publisher_mapping_->getDataRate(topic_);
      if (config->mip_publisher_mapping_->shouldPublish(topic_))
        configure(node);
    }

    /**
     * \brief Checks if this publisher has been configured
     * \return True if the publisher is configured
    */
    bool configured() const
    {
      return publisher_ != nullptr;
    }

    /**
     * \brief Activates the publisher. After this function is called, the publisher is ready to call publish on
     */
    void activate()
    {
      if (publisher_ != nullptr)
        publisher_->on_activate();
    }

    /**
     * \brief Deactivates the publisher
     */
    void deactivate()
    {
      if (publisher_ != nullptr)
        publisher_->on_deactivate();
    }

    /**
     * \brief Publishes the message on the publisher. Will only be published if the message has been updated and the publisher was configured to actually publish
     */
    void publish()
    {
      if (publisher_ != nullptr && message_ != nullptr && updated_)
      {
        publisher_->publish(*message_);
        updated_ = false;
      }
    }

    /**
     * \brief Publishes a message on the publisher. Will always publish the message as long as the publisher is initialized
     * \param msg The message to publish
     */
    void publish(const MessageType& msg)
    {
      if (publisher_ != nullptr)
        publisher_->publish(msg);
    }

    /**
     * \brief Gets the topic this publisher will publish to
     * \return The topic this publisher will publish to
     */
    std::string topic() const
    {
      return topic_;
    }

    float dataRate() const
    {
      return data_rate_;
    }

    /**
     * \brief Gets whether or not this publisher's message has been updated
     * \return true if the message has been updated, false if not
     */
    bool updated() const
    {
      return updated_;
    }

    /**
     * \brief Gets the message without setting updated to true. Useful for updating things on the message at startup, or that do not mean the message must be republished
     * \return Pointer to the message that this publisher is holding
     */
    typename RosPubType<MessageType>::MessageSharedPtr getMessage()
    {
      return message_;
    }

    /**
     * \brief Gets the message and sets updated to true. Useful for updating the message if it should then be published after updating.
     * \return Pointer to the message that this publisher is holding
     */
    typename RosPubType<MessageType>::MessageSharedPtr getMessageToUpdate()
    {
      updated_ = true;
      return getMessage();
    }


   private:
    const std::string topic_;  /// The topic that this class will publish to
    float data_rate_;  /// The data rate in hertz that this topic is streamed at
    bool updated_;  /// Whether or not the message has been updated since the last iteration

    typename RosPubType<MessageType>::MessageSharedPtr message_;  /// Pointer to a message that can be updated and published by this class
    typename RosPubType<MessageType>::SharedPtr publisher_;  /// Pointer to the ROS publisher that will do the actual publishing for this class
  };


  // IMU Publishers
  Publisher<ImuMsg>::SharedPtr                        imu_raw_pub_     = Publisher<ImuMsg>::initialize(IMU_DATA_RAW_TOPIC);
  Publisher<ImuMsg>::SharedPtr                        imu_pub_         = Publisher<ImuMsg>::initialize(IMU_DATA_TOPIC);
  Publisher<MagneticFieldMsg>::SharedPtr              mag_pub_         = Publisher<MagneticFieldMsg>::initialize(IMU_MAG_TOPIC);
  Publisher<FluidPressureMsg>::SharedPtr              pressure_pub_    = Publisher<FluidPressureMsg>::initialize(IMU_PRESSURE_TOPIC);
  Publisher<TwistWithCovarianceStampedMsg>::SharedPtr wheel_speed_pub_ = Publisher<TwistWithCovarianceStampedMsg>::initialize(IMU_WHEEL_SPEED_TOPIC);

  // GNSS publishers
  Publisher<NavSatFixMsg>::SharedPtrVec                  gnss_llh_position_pub_  = Publisher<NavSatFixMsg>::initializeVec({GNSS1_LLH_POSITION_TOPIC, GNSS2_FIX_TOPIC});
  Publisher<TwistWithCovarianceStampedMsg>::SharedPtrVec gnss_velocity_pub_      = Publisher<TwistWithCovarianceStampedMsg>::initializeVec({GNSS1_VELOCITY_TOPIC, GNSS2_VELOCITY_TOPIC});
  Publisher<TwistWithCovarianceStampedMsg>::SharedPtrVec gnss_velocity_ecef_pub_ = Publisher<TwistWithCovarianceStampedMsg>::initializeVec({GNSS1_VELOCITY_ECEF_TOPIC, GNSS2_VELOCITY_ECEF_TOPIC});
  Publisher<OdometryMsg>::SharedPtrVec                   gnss_odometry_pub_      = Publisher<OdometryMsg>::initializeVec({GNSS1_ODOMETRY_TOPIC, GNSS2_ODOMETRY_TOPIC});
  Publisher<TimeReferenceMsg>::SharedPtrVec              gnss_time_pub_          = Publisher<TimeReferenceMsg>::initializeVec({GNSS1_TIME_REF_TOPIC, GNSS2_TIME_REF_TOPIC});

  // Filter publishers
  Publisher<HumanReadableStatusMsg>::SharedPtr            filter_human_readable_status_pub_ = Publisher<HumanReadableStatusMsg>::initialize(FILTER_HUMAN_READABLE_STATUS_TOPIC);
  Publisher<ImuMsg>::SharedPtr                            filter_imu_pub_                   = Publisher<ImuMsg>::initialize(FILTER_IMU_DATA_TOPIC);
  Publisher<NavSatFixMsg>::SharedPtr                      filter_llh_position_pub_          = Publisher<NavSatFixMsg>::initialize(FILTER_LLH_POSITION_TOPIC);
  Publisher<OdometryMsg>::SharedPtr                       filter_odometry_earth_pub_        = Publisher<OdometryMsg>::initialize(FILTER_ODOMETRY_EARTH_TOPIC );
  Publisher<OdometryMsg>::SharedPtr                       filter_odometry_map_pub_          = Publisher<OdometryMsg>::initialize(FILTER_ODOMETRY_MAP_TOPIC);
  Publisher<TwistWithCovarianceStampedMsg>::SharedPtr     filter_velocity_pub_              = Publisher<TwistWithCovarianceStampedMsg>::initialize(FILTER_VELOCITY_TOPIC);
  Publisher<TwistWithCovarianceStampedMsg>::SharedPtr     filter_velocity_ecef_pub_         = Publisher<TwistWithCovarianceStampedMsg>::initialize(FILTER_VELOCITY_ECEF_TOPIC);
  Publisher<PoseWithCovarianceStampedMsg>::SharedPtr      filter_dual_antenna_heading_pub_  = Publisher<PoseWithCovarianceStampedMsg>::initialize(FILTER_DUAL_ANTENNA_HEADING_TOPIC);


  // MIP Sensor (0x80) publishers
  Publisher<MipSensorOverrangeStatusMsg>::SharedPtr       mip_sensor_overrange_status_pub_       = Publisher<MipSensorOverrangeStatusMsg>::initialize(MIP_SENSOR_OVERRANGE_STATUS_TOPIC);
  Publisher<MipSensorTemperatureStatisticsMsg>::SharedPtr mip_sensor_temperature_statistics_pub_ = Publisher<MipSensorTemperatureStatisticsMsg>::initialize(MIP_SENSOR_TEMPERATURE_STATISTICS_TOPIC);

  // MIP GNSS (0x81, 0x91, 0x92) publishers
  Publisher<MipGnssFixInfoMsg>::SharedPtrVec          mip_gnss_fix_info_pub_           = Publisher<MipGnssFixInfoMsg>::initializeVec({MIP_GNSS1_FIX_INFO_TOPIC, MIP_GNSS2_FIX_INFO_TOPIC});
  Publisher<MipGnssSbasInfoMsg>::SharedPtrVec         mip_gnss_sbas_info_pub_          = Publisher<MipGnssSbasInfoMsg>::initializeVec({MIP_GNSS1_SBAS_INFO_TOPIC, MIP_GNSS2_SBAS_INFO_TOPIC});
  Publisher<MipGnssRfErrorDetectionMsg>::SharedPtrVec mip_gnss_rf_error_detection_pub_ = Publisher<MipGnssRfErrorDetectionMsg>::initializeVec({MIP_GNSS1_RF_ERROR_DETECTION_TOPIC, MIP_GNSS2_RF_ERROR_DETECTION_TOPIC});

  // MIP GNSS Corrections (0x93) publishers
  Publisher<MipGnssCorrectionsRtkCorrectionsStatusMsg>::SharedPtr mip_gnss_corrections_rtk_corrections_status_pub_ = Publisher<MipGnssCorrectionsRtkCorrectionsStatusMsg>::initialize(MIP_GNSS_CORRECTIONS_RTK_CORRECTIONS_STATUS_TOPIC);

  // MIP filter (0x82) publishers
  Publisher<MipFilterStatusMsg>::SharedPtr                         mip_filter_status_pub_                          = Publisher<MipFilterStatusMsg>::initialize(MIP_FILTER_STATUS_TOPIC);
  Publisher<MipFilterGnssPositionAidingStatusMsg>::SharedPtr       mip_filter_gnss_position_aiding_status_pub_     = Publisher<MipFilterGnssPositionAidingStatusMsg>::initialize(MIP_FILTER_GNSS_POSITION_AIDING_STATUS_TOPIC);
  Publisher<MipFilterMultiAntennaOffsetCorrectionMsg>::SharedPtr   mip_filter_multi_antenna_offset_correction_pub_ = Publisher<MipFilterMultiAntennaOffsetCorrectionMsg>::initialize(MIP_FILTER_MULTI_ANTENNA_OFFSET_CORRECTION_TOPIC);
  Publisher<MipFilterAidingMeasurementSummaryMsg>::SharedPtr       mip_filter_aiding_measurement_summary_pub_      = Publisher<MipFilterAidingMeasurementSummaryMsg>::initialize(MIP_FILTER_AIDING_MEASUREMENT_SUMMARY_TOPIC);
  Publisher<MipFilterGnssDualAntennaStatusMsg>::SharedPtr          mip_filter_gnss_dual_antenna_status_pub_        = Publisher<MipFilterGnssDualAntennaStatusMsg>::initialize(MIP_FILTER_GNSS_DUAL_ANTENNA_STATUS_TOPIC);

  // MIP System (0xA0) publishers
  Publisher<MipSystemBuiltInTestMsg>::SharedPtr mip_system_built_in_test_pub_ = Publisher<MipSystemBuiltInTestMsg>::initialize(MIP_SYSTEM_BUILT_IN_TEST_TOPIC);

  // NMEA sentence publisher
  Publisher<NMEASentenceMsg>::SharedPtr nmea_sentence_pub_ = Publisher<NMEASentenceMsg>::initialize(NMEA_SENTENCE_TOPIC);

  // Transform Broadcasters
  StaticTransformBroadcasterType static_transform_broadcaster_ = nullptr;
  TransformBroadcasterType transform_broadcaster_ = nullptr;

  // Will be set to true when pose information is updated, and reset to false when the transform is published
  bool imu_link_to_earth_transform_translation_updated_ = false;
  bool imu_link_to_earth_transform_attitude_updated_ = false;
  bool imu_link_to_map_transform_translation_updated_ = false;
  bool imu_link_to_map_transform_attitude_updated_ = false;

  // Transforms that will be updated on each iteation
  tf2::Stamped<tf2::Transform> imu_link_to_earth_transform_tf_stamped_;
  tf2::Stamped<tf2::Transform> imu_link_to_map_transform_tf_stamped_;

  // Published transforms
  TransformStampedMsg gnss_antenna_link_to_imu_link_transform_[NUM_GNSS];
  TransformStampedMsg odometer_link_to_imu_link_transform_;

private:
  /**
   * \brief Helper function to register a packet callback on this class
   * \tparam Callback The Callback function on this class to call when the data is received
   * \param descriptor_set The descriptor set to register the packet callback for
   * \param after_fields Whether this callback should be triggered before or after the field callbacks
   */
  template<void (Publishers::*Callback)(const mip::PacketRef&, mip::Timestamp)>
  void registerPacketCallback(const uint8_t descriptor_set = mip::C::MIP_DISPATCH_ANY_DESCRIPTOR, bool after_fields = true);

  /**
   * \brief Helper function to register a data callback on this class
   * \tparam DataField The type of data to listen for
   * \tparam Callback The Callback function on this class to call when the data is received
   * \param descriptor_set The descriptor set to use for the DataField. Defaults to the DataField's descriptor set
   */
  template<class DataField, void (Publishers::*Callback)(const DataField&, uint8_t, mip::Timestamp)>
  void registerDataCallback(const uint8_t descriptor_set = DataField::DESCRIPTOR_SET);

  // Calbacks to handle shared data from the MIP device
  void handleSharedEventSource(const mip::data_shared::EventSource& event_source, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleSharedTicks(const mip::data_shared::Ticks& ticks, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleSharedDeltaTicks(const mip::data_shared::DeltaTicks& delta_ticks, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleSharedGpsTimestamp(const mip::data_shared::GpsTimestamp& gps_timestamp, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleSharedDeltaTime(const mip::data_shared::DeltaTime& delta_time, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleSharedReferenceTimestamp(const mip::data_shared::ReferenceTimestamp& reference_timestamp, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleSharedReferenceTimeDelta(const mip::data_shared::ReferenceTimeDelta& reference_time_delta, const uint8_t descriptor_set, mip::Timestamp timestamp);

  // Callbacks to handle sensor data from the MIP device
  void handleSensorGpsTimestamp(const mip::data_sensor::GpsTimestamp& gps_timestamp, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleSensorScaledAccel(const mip::data_sensor::ScaledAccel& scaled_accel, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleSensorScaledGyro(const mip::data_sensor::ScaledGyro& scaled_gyro, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleSensorDeltaTheta(const mip::data_sensor::DeltaTheta& delta_theta, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleSensorDeltaVelocity(const mip::data_sensor::DeltaVelocity& delta_velocity, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleSensorCompQuaternion(const mip::data_sensor::CompQuaternion& comp_quaternion, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleSensorScaledMag(const mip::data_sensor::ScaledMag& scaled_mag, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleSensorScaledPressure(const mip::data_sensor::ScaledPressure& scaled_pressure, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleSensorOdometerData(const mip::data_sensor::OdometerData& odometer_data, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleSensorOverrangeStatus(const mip::data_sensor::OverrangeStatus& overrange_status, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleSensorTemperatureStatistics(const mip::data_sensor::TemperatureAbs& temperature_statistics, const uint8_t descriptor_set, mip::Timestamp timestamp);

  // Callbcaks to handle GNSS1/2 data from the device
  void handleGnssGpsTime(const mip::data_gnss::GpsTime& gps_time, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleGnssPosLlh(const mip::data_gnss::PosLlh& pos_llh, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleGnssVelNed(const mip::data_gnss::VelNed& vel_ned, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleGnssPosEcef(const mip::data_gnss::PosEcef& pos_ecef, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleGnssVelEcef(const mip::data_gnss::VelEcef& vel_ecef, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleGnssFixInfo(const mip::data_gnss::FixInfo& fix_info, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleGnssSbasInfo(const mip::data_gnss::SbasInfo& sbas_info, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleGnssRfErrorDetection(const mip::data_gnss::RfErrorDetection& rf_error_detection, const uint8_t descriptor_set, mip::Timestamp timestamp);

  // Callbacks to handle RTK data from the device
  void handleRtkCorrectionsStatus(const mip::data_gnss::RtkCorrectionsStatus& rtk_corrections_status, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleRtkBaseStationInfo(const mip::data_gnss::BaseStationInfo& base_station_info, const uint8_t descriptor_set, mip::Timestamp timestamp);

  // Callbacks to handle filter data from the device
  void handleFilterTimestamp(const mip::data_filter::Timestamp& filter_timestamp, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleFilterStatus(const mip::data_filter::Status& status, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleFilterEcefPos(const mip::data_filter::EcefPos& ecef_pos, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleFilterEcefPosUncertainty(const mip::data_filter::EcefPosUncertainty& ecef_pos_uncertainty, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleFilterPositionLlh(const mip::data_filter::PositionLlh& position_llh, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleFilterPositionLlhUncertainty(const mip::data_filter::PositionLlhUncertainty& position_llh_uncertainty, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleFilterAttitudeQuaternion(const mip::data_filter::AttitudeQuaternion& attitude_quaternion, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleFilterEulerAnglesUncertainty(const mip::data_filter::EulerAnglesUncertainty& euler_angles_uncertainty, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleFilterVelocityNed(const mip::data_filter::VelocityNed& velocity_ned, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleFilterVelocityNedUncertainty(const mip::data_filter::VelocityNedUncertainty& velocity_ned_uncertainty, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleFilterEcefVelocity(const mip::data_filter::EcefVel& ecef_vel, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleFilterEcefVelocityUncertainty(const mip::data_filter::EcefVelUncertainty& ecef_vel_uncertainty, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleFilterCompAngularRate(const mip::data_filter::CompAngularRate& comp_angular_rate, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleFilterCompAccel(const mip::data_filter::CompAccel& comp_accel, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleFilterLinearAccel(const mip::data_filter::LinearAccel& linear_accel, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleFilterGnssPosAidStatus(const mip::data_filter::GnssPosAidStatus& gnss_pos_aid_status, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleFilterMultiAntennaOffsetCorrection(const mip::data_filter::MultiAntennaOffsetCorrection& multi_antenna_offset_correction, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleFilterGnssDualAntennaStatus(const mip::data_filter::GnssDualAntennaStatus& gnss_dual_antenna_status, const uint8_t descriptor_set, mip::Timestamp timestamp);
  void handleFilterAidingMeasurementSummary(const mip::data_filter::AidingMeasurementSummary& aiding_measurement_summary, const uint8_t descriptor_set, mip::Timestamp timestamp);

  // Callbacks to handle system data from the device
  void handleSystemBuiltInTest(const mip::data_system::BuiltInTest& built_in_test, const uint8_t descriptor_set, mip::Timestamp timestamp);

  /**
   * \brief Called after a packet has been processed.
   * \param packet The packet that was processed
   * \param timestamp The timestamp of when the packet was received
  */
  void handleAfterPacket(const mip::PacketRef& packet, mip::Timestamp timestamp);

  /**
   * \brief Updates the microstrain header contained in all MIP specific custom messages
   * \param mip_header The header to update
   * \param descriptor_set The descriptor set that the message comes from
   * \param timestamp The timstamp that the data was collected at
   * \param gps_timestamp Optional timestamp to use instead of the most recent GPS timestamp. Only used if non-null
  */
  void updateMipHeader(MipHeaderMsg* mip_header, uint8_t descriptor_set, mip::Timestamp timestamp, const mip::data_shared::GpsTimestamp* gps_timestamp = nullptr);

  /**
   * \brief Updates the header's timestamp to the type of timestamp based on the node's configuration
   * \param header The header to update the timestamp of
   * \param descriptor_set The descriptor set that should be used to lookup the timestamp if we want to use the device timestamp
   * \param timestamp The timestamp provided by the MIP SDK for when the packet was received
   * \param gps_timestamp Optional timestamp to use instead of the most recent GPS timestamp. Only used if non-null
   */
  void updateHeaderTime(RosHeaderType* header, uint8_t descriptor_set, mip::Timestamp timestamp, const mip::data_shared::GpsTimestamp* gps_timestamp = nullptr);

  /**
   * \brief Updates the header's timestamp to the UTC representation of the GPS timestamp
   * \param header The time object to set the time on
   * \param timestamp The GPS timestamp to use to update the header
   */
  static void setGpsTime(RosTimeType* time, const mip::data_shared::GpsTimestamp& timestamp);

  // List of MIP dispatch handlers used to subscribe to data from the MIP SDK
  std::vector<std::shared_ptr<mip::C::mip_dispatch_handler>> mip_dispatch_handlers_;

  // Handles to the ROS node and the config
  RosNodeType* node_;
  Config* config_;

  // Mapping between every shared data field and descriptor sets
  std::map<uint8_t, mip::data_shared::EventSource> event_source_mapping_;
  std::map<uint8_t, mip::data_shared::Ticks> ticks_mapping_;
  std::map<uint8_t, mip::data_shared::DeltaTicks> delta_ticks_mapping_;
  std::map<uint8_t, mip::data_shared::GpsTimestamp> gps_timestamp_mapping_;
  std::map<uint8_t, mip::data_shared::DeltaTime> delta_time_mapping_;
  std::map<uint8_t, mip::data_shared::ReferenceTimestamp> reference_timestamp_mapping_;
  std::map<uint8_t, mip::data_shared::ReferenceTimeDelta> reference_time_delta_mapping_;

  // Previous timestamp for each descriptor set. Only used for hybrid timestamping
  std::map<uint8_t, double> previous_utc_timestamps_;

  // Older philo devices do not support ECEF position, so we will need to convert from LLH to ECEF ourselves
  bool supports_filter_ecef_ = false;

  // Keep track of the filter state as the messages may override each other if we don't
  bool rtk_fixed_ = false;
  bool rtk_float_ = false;
  bool has_sbas_  = false;
  bool has_fix_   = false;

  // TF2 buffer lookup class
  TransformBufferType transform_buffer_;
  TransformListenerType transform_listener_;

  // Keep track of the last GPS timestamp we received for dual antenna heading so we don't publish stale data
  double last_dual_antenna_heading_gps_timestamp_secs_;

  std::chrono::time_point<std::chrono::system_clock> start_time_;
  double last_timestamp_ = 0;

  // Clock model used to translate device time to ROS time for each descriptor set
  ClockBiasMonitor clock_bias_monitor_ = ClockBiasMonitor(0.99, 1.0);
};

template<void (Publishers::*Callback)(const mip::PacketRef&, mip::Timestamp)>
void Publishers::registerPacketCallback(const uint8_t descriptor_set, bool after_fields)
{
  // Regsiter a handler for the callback
  mip_dispatch_handlers_.push_back(std::make_shared<mip::C::mip_dispatch_handler>());

  // Pass to the MIP SDK
  config_->mip_device_->device().registerPacketCallback<Publishers, Callback>(*(mip_dispatch_handlers_.back()), descriptor_set, after_fields, this);
}

template<class DataField, void (Publishers::*Callback)(const DataField&, uint8_t, mip::Timestamp)>
void Publishers::registerDataCallback(const uint8_t descriptor_set)
{
  // Register a handler for the callback
  mip_dispatch_handlers_.push_back(std::make_shared<mip::C::mip_dispatch_handler>());

  // Pass to the MIP SDK
  config_->mip_device_->device().registerDataCallback<DataField, Publishers, Callback>(*(mip_dispatch_handlers_.back()), this, descriptor_set);
}

}  // namespace microstrain

#endif  // MICROSTRAIN_INERTIAL_DRIVER_COMMON_PUBLISHERS_H

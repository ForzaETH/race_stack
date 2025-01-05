#pragma once

#include "common.h"
#include "descriptors.h"
#include "../mip_result.h"

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
namespace mip {
namespace C {
extern "C" {

#endif // __cplusplus
struct mip_interface;
struct mip_serializer;
struct mip_field;

////////////////////////////////////////////////////////////////////////////////
///@addtogroup MipCommands_c  MIP Commands [C]
///@{
///@defgroup 3dm_commands_c  3dm Commands [C]
///
///@{

////////////////////////////////////////////////////////////////////////////////
// Descriptors
////////////////////////////////////////////////////////////////////////////////

enum 
{
    MIP_3DM_CMD_DESC_SET                             = 0x0C,
    
    MIP_CMD_DESC_3DM_POLL_IMU_MESSAGE                = 0x01,
    MIP_CMD_DESC_3DM_POLL_GNSS_MESSAGE               = 0x02,
    MIP_CMD_DESC_3DM_POLL_FILTER_MESSAGE             = 0x03,
    MIP_CMD_DESC_3DM_POLL_NMEA_MESSAGE               = 0x04,
    MIP_CMD_DESC_3DM_GET_IMU_BASE_RATE               = 0x06,
    MIP_CMD_DESC_3DM_GET_GNSS_BASE_RATE              = 0x07,
    MIP_CMD_DESC_3DM_IMU_MESSAGE_FORMAT              = 0x08,
    MIP_CMD_DESC_3DM_GNSS_MESSAGE_FORMAT             = 0x09,
    MIP_CMD_DESC_3DM_FILTER_MESSAGE_FORMAT           = 0x0A,
    MIP_CMD_DESC_3DM_GET_FILTER_BASE_RATE            = 0x0B,
    MIP_CMD_DESC_3DM_NMEA_MESSAGE_FORMAT             = 0x0C,
    MIP_CMD_DESC_3DM_POLL_DATA                       = 0x0D,
    MIP_CMD_DESC_3DM_GET_BASE_RATE                   = 0x0E,
    MIP_CMD_DESC_3DM_MESSAGE_FORMAT                  = 0x0F,
    MIP_CMD_DESC_3DM_CONFIGURE_FACTORY_STREAMING     = 0x10,
    MIP_CMD_DESC_3DM_CONTROL_DATA_STREAM             = 0x11,
    MIP_CMD_DESC_3DM_RAW_RTCM_2_3_MESSAGE            = 0x20,
    MIP_CMD_DESC_3DM_GNSS_CONSTELLATION_SETTINGS     = 0x21,
    MIP_CMD_DESC_3DM_GNSS_SBAS_SETTINGS              = 0x22,
    MIP_CMD_DESC_3DM_GNSS_ASSISTED_FIX_SETTINGS      = 0x23,
    MIP_CMD_DESC_3DM_GNSS_TIME_ASSISTANCE            = 0x24,
    MIP_CMD_DESC_3DM_PPS_SOURCE                      = 0x28,
    MIP_CMD_DESC_3DM_EVENT_SUPPORT                   = 0x2A,
    MIP_CMD_DESC_3DM_EVENT_CONTROL                   = 0x2B,
    MIP_CMD_DESC_3DM_EVENT_TRIGGER_STATUS            = 0x2C,
    MIP_CMD_DESC_3DM_EVENT_ACTION_STATUS             = 0x2D,
    MIP_CMD_DESC_3DM_EVENT_TRIGGER_CONFIG            = 0x2E,
    MIP_CMD_DESC_3DM_EVENT_ACTION_CONFIG             = 0x2F,
    MIP_CMD_DESC_3DM_DEVICE_STARTUP_SETTINGS         = 0x30,
    MIP_CMD_DESC_3DM_SENSOR2VEHICLE_TRANSFORM_EUL    = 0x31,
    MIP_CMD_DESC_3DM_SENSOR2VEHICLE_TRANSFORM_QUAT   = 0x32,
    MIP_CMD_DESC_3DM_SENSOR2VEHICLE_TRANSFORM_DCM    = 0x33,
    MIP_CMD_DESC_3DM_SET_GNSS_DYNAMICS_MODE          = 0x34,
    MIP_CMD_DESC_3DM_SET_IMU_SIGNAL_COND             = 0x35,
    MIP_CMD_DESC_3DM_SET_IMU_TIMESTAMP               = 0x36,
    MIP_CMD_DESC_3DM_ACCEL_BIAS                      = 0x37,
    MIP_CMD_DESC_3DM_GYRO_BIAS                       = 0x38,
    MIP_CMD_DESC_3DM_CAPTURE_GYRO_BIAS               = 0x39,
    MIP_CMD_DESC_3DM_HARD_IRON_OFFSET                = 0x3A,
    MIP_CMD_DESC_3DM_SOFT_IRON_MATRIX                = 0x3B,
    MIP_CMD_DESC_3DM_REALIGN_UP                      = 0x3C,
    MIP_CMD_DESC_3DM_REALIGN_NORTH                   = 0x3D,
    MIP_CMD_DESC_3DM_CONING_AND_SCULLING_ENABLE      = 0x3E,
    MIP_CMD_DESC_3DM_UART_BAUDRATE                   = 0x40,
    MIP_CMD_DESC_3DM_GPIO_CONFIG                     = 0x41,
    MIP_CMD_DESC_3DM_GPIO_STATE                      = 0x42,
    MIP_CMD_DESC_3DM_ODOMETER_CONFIG                 = 0x43,
    MIP_CMD_DESC_3DM_IMU_LOWPASS_FILTER              = 0x50,
    MIP_CMD_DESC_3DM_LEGACY_COMP_FILTER              = 0x51,
    MIP_CMD_DESC_3DM_SENSOR_RANGE                    = 0x52,
    MIP_CMD_DESC_3DM_CALIBRATED_RANGES               = 0x53,
    MIP_CMD_DESC_3DM_LOWPASS_FILTER                  = 0x54,
    MIP_CMD_DESC_3DM_DATASTREAM_FORMAT               = 0x60,
    MIP_CMD_DESC_3DM_DEVICE_POWER_STATE              = 0x61,
    MIP_CMD_DESC_3DM_SAVE_RESTORE_GPS_SETTINGS       = 0x62,
    MIP_CMD_DESC_3DM_DEVICE_SETTINGS                 = 0x63,
    MIP_CMD_DESC_3DM_RAW_CLIP_SETTINGS               = 0x70,
    
    MIP_REPLY_DESC_3DM_IMU_MESSAGE_FORMAT            = 0x80,
    MIP_REPLY_DESC_3DM_GNSS_MESSAGE_FORMAT           = 0x81,
    MIP_REPLY_DESC_3DM_FILTER_MESSAGE_FORMAT         = 0x82,
    MIP_REPLY_DESC_3DM_IMU_BASE_RATE                 = 0x83,
    MIP_REPLY_DESC_3DM_GNSS_BASE_RATE                = 0x84,
    MIP_REPLY_DESC_3DM_DATASTREAM_ENABLE             = 0x85,
    MIP_REPLY_DESC_3DM_IMU_SIGNAL_SETTINGS           = 0x86,
    MIP_REPLY_DESC_3DM_UART_BAUDRATE                 = 0x87,
    MIP_REPLY_DESC_3DM_DATASTREAM_FORMAT             = 0x88,
    MIP_REPLY_DESC_3DM_POWER_STATE                   = 0x89,
    MIP_REPLY_DESC_3DM_FILTER_BASE_RATE              = 0x8A,
    MIP_REPLY_DESC_3DM_ADVANCED_DATA_FILTER          = 0x8B,
    MIP_REPLY_DESC_3DM_POLL_DATA                     = 0x8D,
    MIP_REPLY_DESC_3DM_BASE_RATE                     = 0x8E,
    MIP_REPLY_DESC_3DM_MESSAGE_FORMAT                = 0x8F,
    MIP_REPLY_DESC_3DM_COMMUNICATIONS_MODE           = 0x91,
    MIP_REPLY_DESC_3DM_GNSS_DYNAMICS_MODE            = 0x92,
    MIP_REPLY_DESC_3DM_IMU_TIMESTAMP_VALUE           = 0x93,
    MIP_REPLY_DESC_3DM_IMU_BASIC_STATUS              = 0x94,
    MIP_REPLY_DESC_3DM_IMU_ADVANCED_STATUS           = 0x95,
    MIP_REPLY_DESC_3DM_RAW_CLIP_SETTINGS             = 0x96,
    MIP_REPLY_DESC_3DM_LEGACY_COMP_FILTER            = 0x97,
    MIP_REPLY_DESC_3DM_ACCEL_BIAS_VECTOR             = 0x9A,
    MIP_REPLY_DESC_3DM_GYRO_BIAS_VECTOR              = 0x9B,
    MIP_REPLY_DESC_3DM_HARD_IRON_OFFSET_VECTOR       = 0x9C,
    MIP_REPLY_DESC_3DM_SOFT_IRON_COMP_MATRIX         = 0x9D,
    MIP_REPLY_DESC_3DM_CONING_AND_SCULLING_ENABLE    = 0x9E,
    MIP_REPLY_DESC_3DM_GNSS_CONSTELLATION_SETTINGS   = 0xA0,
    MIP_REPLY_DESC_3DM_GNSS_SBAS_SETTINGS            = 0xA1,
    MIP_REPLY_DESC_3DM_GNSS_ASSISTED_FIX_SETTINGS    = 0xA2,
    MIP_REPLY_DESC_3DM_GNSS_TIME_ASSISTANCE          = 0xA3,
    MIP_REPLY_DESC_3DM_SENSOR2VEHICLE_TRANSFORM_EUL  = 0xB1,
    MIP_REPLY_DESC_3DM_SENSOR2VEHICLE_TRANSFORM_QUAT = 0xB2,
    MIP_REPLY_DESC_3DM_SENSOR2VEHICLE_TRANSFORM_DCM  = 0xB3,
    MIP_REPLY_DESC_3DM_EVENT_SUPPORT                 = 0xB4,
    MIP_REPLY_DESC_3DM_EVENT_CONTROL                 = 0xB5,
    MIP_REPLY_DESC_3DM_EVENT_TRIGGER_STATUS          = 0xB6,
    MIP_REPLY_DESC_3DM_EVENT_ACTION_STATUS           = 0xB7,
    MIP_REPLY_DESC_3DM_EVENT_TRIGGER_CONFIG          = 0xB8,
    MIP_REPLY_DESC_3DM_EVENT_ACTION_CONFIG           = 0xB9,
    MIP_REPLY_DESC_3DM_NMEA_MESSAGE_FORMAT           = 0x8C,
    MIP_REPLY_DESC_3DM_PPS_SOURCE                    = 0xA8,
    MIP_REPLY_DESC_3DM_GPIO_CONFIG                   = 0xC1,
    MIP_REPLY_DESC_3DM_GPIO_STATE                    = 0xC2,
    MIP_REPLY_DESC_3DM_ODOMETER_CONFIG               = 0xC3,
    MIP_REPLY_DESC_3DM_SENSOR_RANGE                  = 0xD2,
    MIP_REPLY_DESC_3DM_CALIBRATED_RANGES             = 0xD3,
    MIP_REPLY_DESC_3DM_LOWPASS_FILTER                = 0xD4,
};

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

typedef uint8_t mip_nmea_message_message_id;
static const mip_nmea_message_message_id MIP_NMEA_MESSAGE_MESSAGE_ID_GGA  = 1;   ///<  GPS System Fix Data. Source can be the Filter or GNSS1/2 datasets.
static const mip_nmea_message_message_id MIP_NMEA_MESSAGE_MESSAGE_ID_GLL  = 2;   ///<  Geographic Position Lat/Lon. Source can be the Filter or GNSS1/2 datasets.
static const mip_nmea_message_message_id MIP_NMEA_MESSAGE_MESSAGE_ID_GSV  = 3;   ///<  GNSS Satellites in View. Source must be either GNSS1 or GNSS2 datasets. The talker ID must be set to IGNORED.
static const mip_nmea_message_message_id MIP_NMEA_MESSAGE_MESSAGE_ID_RMC  = 4;   ///<  Recommended Minimum Specific GNSS Data. Source can be the Filter or GNSS1/2 datasets.
static const mip_nmea_message_message_id MIP_NMEA_MESSAGE_MESSAGE_ID_VTG  = 5;   ///<  Course over Ground. Source can be the Filter or GNSS1/2 datasets.
static const mip_nmea_message_message_id MIP_NMEA_MESSAGE_MESSAGE_ID_HDT  = 6;   ///<  Heading, True. Source can be the Filter or GNSS1/2 datasets.
static const mip_nmea_message_message_id MIP_NMEA_MESSAGE_MESSAGE_ID_ZDA  = 7;   ///<  Time & Date. Source must be the GNSS1 or GNSS2 datasets.
static const mip_nmea_message_message_id MIP_NMEA_MESSAGE_MESSAGE_ID_PKRA = 129; ///<  Parker proprietary Euler angles. Source must be the Filter dataset. The talker ID must be set to IGNORED.
static const mip_nmea_message_message_id MIP_NMEA_MESSAGE_MESSAGE_ID_PKRR = 130; ///<  Parker proprietary Angular Rate/Acceleration. Source must be the Sensor dataset. The talker ID must be set to IGNORED.

typedef uint8_t mip_nmea_message_talker_id;
static const mip_nmea_message_talker_id MIP_NMEA_MESSAGE_TALKER_ID_IGNORED = 0; ///<  Talker ID cannot be changed.
static const mip_nmea_message_talker_id MIP_NMEA_MESSAGE_TALKER_ID_GNSS    = 1; ///<  NMEA message will be produced with talker id "GN".
static const mip_nmea_message_talker_id MIP_NMEA_MESSAGE_TALKER_ID_GPS     = 2; ///<  NMEA message will be produced with talker id "GP".
static const mip_nmea_message_talker_id MIP_NMEA_MESSAGE_TALKER_ID_GALILEO = 3; ///<  NMEA message will be produced with talker id "GA".
static const mip_nmea_message_talker_id MIP_NMEA_MESSAGE_TALKER_ID_GLONASS = 4; ///<  NMEA message will be produced with talker id "GL".

struct mip_nmea_message
{
    mip_nmea_message_message_id message_id; ///< NMEA sentence type.
    mip_nmea_message_talker_id talker_id; ///< NMEA talker ID. Ignored for proprietary sentences.
    uint8_t source_desc_set; ///< Data descriptor set where the data will be sourced. Available options depend on the sentence.
    uint16_t decimation; ///< Decimation from the base rate for source_desc_set. Frequency is limited to 10 Hz or the base rate, whichever is lower. Must be 0 when polling.
    
};
typedef struct mip_nmea_message mip_nmea_message;
void insert_mip_nmea_message(struct mip_serializer* serializer, const mip_nmea_message* self);
void extract_mip_nmea_message(struct mip_serializer* serializer, mip_nmea_message* self);

void insert_mip_nmea_message_message_id(struct mip_serializer* serializer, const mip_nmea_message_message_id self);
void extract_mip_nmea_message_message_id(struct mip_serializer* serializer, mip_nmea_message_message_id* self);

void insert_mip_nmea_message_talker_id(struct mip_serializer* serializer, const mip_nmea_message_talker_id self);
void extract_mip_nmea_message_talker_id(struct mip_serializer* serializer, mip_nmea_message_talker_id* self);

typedef uint8_t mip_sensor_range_type;
static const mip_sensor_range_type MIP_SENSOR_RANGE_TYPE_ALL   = 0; ///<  Only allowed for SAVE, LOAD, and DEFAULT function selectors.
static const mip_sensor_range_type MIP_SENSOR_RANGE_TYPE_ACCEL = 1; ///<  Accelerometer. Range is specified in g.
static const mip_sensor_range_type MIP_SENSOR_RANGE_TYPE_GYRO  = 2; ///<  Gyroscope. Range is specified in degrees/s.
static const mip_sensor_range_type MIP_SENSOR_RANGE_TYPE_MAG   = 3; ///<  Magnetometer. Range is specified in Gauss.
static const mip_sensor_range_type MIP_SENSOR_RANGE_TYPE_PRESS = 4; ///<  Pressure sensor. Range is specified in hPa.

void insert_mip_sensor_range_type(struct mip_serializer* serializer, const mip_sensor_range_type self);
void extract_mip_sensor_range_type(struct mip_serializer* serializer, mip_sensor_range_type* self);


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
///@defgroup c_3dm_poll_imu_message  (0x0C,0x01) Poll Imu Message [C]
/// Poll the device for an IMU message with the specified format
/// 
/// This function polls for an IMU message using the provided format. The resulting message
/// will maintain the order of descriptors sent in the command and any unrecognized
/// descriptors are ignored. If the format is not provided, the device will attempt to use the
/// stored format (set with the Set IMU Message Format command.) If no format is provided
/// and there is no stored format, the device will respond with a NACK. The reply packet contains
/// an ACK/NACK field. The polled data packet is sent separately as an IMU Data packet.
///
///@{

struct mip_3dm_poll_imu_message_command
{
    bool suppress_ack; ///< Suppress the usual ACK/NACK reply.
    uint8_t num_descriptors; ///< Number of descriptors in the descriptor list.
    mip_descriptor_rate descriptors[83]; ///< Descriptor list.
    
};
typedef struct mip_3dm_poll_imu_message_command mip_3dm_poll_imu_message_command;
void insert_mip_3dm_poll_imu_message_command(struct mip_serializer* serializer, const mip_3dm_poll_imu_message_command* self);
void extract_mip_3dm_poll_imu_message_command(struct mip_serializer* serializer, mip_3dm_poll_imu_message_command* self);

mip_cmd_result mip_3dm_poll_imu_message(struct mip_interface* device, bool suppress_ack, uint8_t num_descriptors, const mip_descriptor_rate* descriptors);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_3dm_poll_gnss_message  (0x0C,0x02) Poll Gnss Message [C]
/// Poll the device for an GNSS message with the specified format
/// 
/// This function polls for a GNSS message using the provided format. The resulting message
/// will maintain the order of descriptors sent in the command and any unrecognized
/// descriptors are ignored. If the format is not provided, the device will attempt to use the
/// stored format (set with the Set GNSS Message Format command.) If no format is provided
/// and there is no stored format, the device will respond with a NACK. The reply packet contains
/// an ACK/NACK field. The polled data packet is sent separately as a GNSS Data packet.
///
///@{

struct mip_3dm_poll_gnss_message_command
{
    bool suppress_ack; ///< Suppress the usual ACK/NACK reply.
    uint8_t num_descriptors; ///< Number of descriptors in the descriptor list.
    mip_descriptor_rate descriptors[83]; ///< Descriptor list.
    
};
typedef struct mip_3dm_poll_gnss_message_command mip_3dm_poll_gnss_message_command;
void insert_mip_3dm_poll_gnss_message_command(struct mip_serializer* serializer, const mip_3dm_poll_gnss_message_command* self);
void extract_mip_3dm_poll_gnss_message_command(struct mip_serializer* serializer, mip_3dm_poll_gnss_message_command* self);

mip_cmd_result mip_3dm_poll_gnss_message(struct mip_interface* device, bool suppress_ack, uint8_t num_descriptors, const mip_descriptor_rate* descriptors);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_3dm_poll_filter_message  (0x0C,0x03) Poll Filter Message [C]
/// Poll the device for an Estimation Filter message with the specified format
/// 
/// This function polls for an Estimation Filter message using the provided format. The resulting message
/// will maintain the order of descriptors sent in the command and any unrecognized
/// descriptors are ignored. If the format is not provided, the device will attempt to use the
/// stored format (set with the Set Estimation Filter Message Format command.) If no format is provided
/// and there is no stored format, the device will respond with a NACK. The reply packet contains
/// an ACK/NACK field. The polled data packet is sent separately as an Estimation Filter Data packet.
///
///@{

struct mip_3dm_poll_filter_message_command
{
    bool suppress_ack; ///< Suppress the usual ACK/NACK reply.
    uint8_t num_descriptors; ///< Number of descriptors in the format list.
    mip_descriptor_rate descriptors[83]; ///< Descriptor format list.
    
};
typedef struct mip_3dm_poll_filter_message_command mip_3dm_poll_filter_message_command;
void insert_mip_3dm_poll_filter_message_command(struct mip_serializer* serializer, const mip_3dm_poll_filter_message_command* self);
void extract_mip_3dm_poll_filter_message_command(struct mip_serializer* serializer, mip_3dm_poll_filter_message_command* self);

mip_cmd_result mip_3dm_poll_filter_message(struct mip_interface* device, bool suppress_ack, uint8_t num_descriptors, const mip_descriptor_rate* descriptors);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_3dm_imu_message_format  (0x0C,0x08) Imu Message Format [C]
/// Set, read, or save the format of the IMU data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
///
///@{

struct mip_3dm_imu_message_format_command
{
    mip_function_selector function;
    uint8_t num_descriptors; ///< Number of descriptors
    mip_descriptor_rate descriptors[82]; ///< Descriptor format list.
    
};
typedef struct mip_3dm_imu_message_format_command mip_3dm_imu_message_format_command;
void insert_mip_3dm_imu_message_format_command(struct mip_serializer* serializer, const mip_3dm_imu_message_format_command* self);
void extract_mip_3dm_imu_message_format_command(struct mip_serializer* serializer, mip_3dm_imu_message_format_command* self);

struct mip_3dm_imu_message_format_response
{
    uint8_t num_descriptors; ///< Number of descriptors
    mip_descriptor_rate descriptors[82]; ///< Descriptor format list.
    
};
typedef struct mip_3dm_imu_message_format_response mip_3dm_imu_message_format_response;
void insert_mip_3dm_imu_message_format_response(struct mip_serializer* serializer, const mip_3dm_imu_message_format_response* self);
void extract_mip_3dm_imu_message_format_response(struct mip_serializer* serializer, mip_3dm_imu_message_format_response* self);

mip_cmd_result mip_3dm_write_imu_message_format(struct mip_interface* device, uint8_t num_descriptors, const mip_descriptor_rate* descriptors);
mip_cmd_result mip_3dm_read_imu_message_format(struct mip_interface* device, uint8_t* num_descriptors_out, uint8_t num_descriptors_out_max, mip_descriptor_rate* descriptors_out);
mip_cmd_result mip_3dm_save_imu_message_format(struct mip_interface* device);
mip_cmd_result mip_3dm_load_imu_message_format(struct mip_interface* device);
mip_cmd_result mip_3dm_default_imu_message_format(struct mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_3dm_gps_message_format  (0x0C,0x09) Gps Message Format [C]
/// Set, read, or save the format of the GNSS data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
///
///@{

struct mip_3dm_gps_message_format_command
{
    mip_function_selector function;
    uint8_t num_descriptors; ///< Number of descriptors
    mip_descriptor_rate descriptors[82]; ///< Descriptor format list.
    
};
typedef struct mip_3dm_gps_message_format_command mip_3dm_gps_message_format_command;
void insert_mip_3dm_gps_message_format_command(struct mip_serializer* serializer, const mip_3dm_gps_message_format_command* self);
void extract_mip_3dm_gps_message_format_command(struct mip_serializer* serializer, mip_3dm_gps_message_format_command* self);

struct mip_3dm_gps_message_format_response
{
    uint8_t num_descriptors; ///< Number of descriptors
    mip_descriptor_rate descriptors[82]; ///< Descriptor format list.
    
};
typedef struct mip_3dm_gps_message_format_response mip_3dm_gps_message_format_response;
void insert_mip_3dm_gps_message_format_response(struct mip_serializer* serializer, const mip_3dm_gps_message_format_response* self);
void extract_mip_3dm_gps_message_format_response(struct mip_serializer* serializer, mip_3dm_gps_message_format_response* self);

mip_cmd_result mip_3dm_write_gps_message_format(struct mip_interface* device, uint8_t num_descriptors, const mip_descriptor_rate* descriptors);
mip_cmd_result mip_3dm_read_gps_message_format(struct mip_interface* device, uint8_t* num_descriptors_out, uint8_t num_descriptors_out_max, mip_descriptor_rate* descriptors_out);
mip_cmd_result mip_3dm_save_gps_message_format(struct mip_interface* device);
mip_cmd_result mip_3dm_load_gps_message_format(struct mip_interface* device);
mip_cmd_result mip_3dm_default_gps_message_format(struct mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_3dm_filter_message_format  (0x0C,0x0A) Filter Message Format [C]
/// Set, read, or save the format of the Estimation Filter data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
///
///@{

struct mip_3dm_filter_message_format_command
{
    mip_function_selector function;
    uint8_t num_descriptors; ///< Number of descriptors (limited by payload size)
    mip_descriptor_rate descriptors[82];
    
};
typedef struct mip_3dm_filter_message_format_command mip_3dm_filter_message_format_command;
void insert_mip_3dm_filter_message_format_command(struct mip_serializer* serializer, const mip_3dm_filter_message_format_command* self);
void extract_mip_3dm_filter_message_format_command(struct mip_serializer* serializer, mip_3dm_filter_message_format_command* self);

struct mip_3dm_filter_message_format_response
{
    uint8_t num_descriptors; ///< Number of descriptors (limited by payload size)
    mip_descriptor_rate descriptors[82];
    
};
typedef struct mip_3dm_filter_message_format_response mip_3dm_filter_message_format_response;
void insert_mip_3dm_filter_message_format_response(struct mip_serializer* serializer, const mip_3dm_filter_message_format_response* self);
void extract_mip_3dm_filter_message_format_response(struct mip_serializer* serializer, mip_3dm_filter_message_format_response* self);

mip_cmd_result mip_3dm_write_filter_message_format(struct mip_interface* device, uint8_t num_descriptors, const mip_descriptor_rate* descriptors);
mip_cmd_result mip_3dm_read_filter_message_format(struct mip_interface* device, uint8_t* num_descriptors_out, uint8_t num_descriptors_out_max, mip_descriptor_rate* descriptors_out);
mip_cmd_result mip_3dm_save_filter_message_format(struct mip_interface* device);
mip_cmd_result mip_3dm_load_filter_message_format(struct mip_interface* device);
mip_cmd_result mip_3dm_default_filter_message_format(struct mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_3dm_imu_get_base_rate  (0x0C,0x06) Imu Get Base Rate [C]
/// Get the base rate for the IMU data in Hz
/// 
/// This is the fastest rate for this type of data available on the device.
/// This is used in conjunction with the IMU Message Format Command to set streaming data at a specified rate.
///
///@{

struct mip_3dm_imu_get_base_rate_response
{
    uint16_t rate; ///< [hz]
    
};
typedef struct mip_3dm_imu_get_base_rate_response mip_3dm_imu_get_base_rate_response;
void insert_mip_3dm_imu_get_base_rate_response(struct mip_serializer* serializer, const mip_3dm_imu_get_base_rate_response* self);
void extract_mip_3dm_imu_get_base_rate_response(struct mip_serializer* serializer, mip_3dm_imu_get_base_rate_response* self);

mip_cmd_result mip_3dm_imu_get_base_rate(struct mip_interface* device, uint16_t* rate_out);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_3dm_gps_get_base_rate  (0x0C,0x07) Gps Get Base Rate [C]
/// Get the base rate for the GNSS data in Hz
/// 
/// This is the fastest rate for this type of data available on the device.
/// This is used in conjunction with the GNSS Message Format Command to set streaming data at a specified rate.
///
///@{

struct mip_3dm_gps_get_base_rate_response
{
    uint16_t rate; ///< [hz]
    
};
typedef struct mip_3dm_gps_get_base_rate_response mip_3dm_gps_get_base_rate_response;
void insert_mip_3dm_gps_get_base_rate_response(struct mip_serializer* serializer, const mip_3dm_gps_get_base_rate_response* self);
void extract_mip_3dm_gps_get_base_rate_response(struct mip_serializer* serializer, mip_3dm_gps_get_base_rate_response* self);

mip_cmd_result mip_3dm_gps_get_base_rate(struct mip_interface* device, uint16_t* rate_out);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_3dm_filter_get_base_rate  (0x0C,0x0B) Filter Get Base Rate [C]
/// Get the base rate for the Estimation Filter data in Hz
/// 
/// This is the fastest rate for this type of data available on the device.
/// This is used in conjunction with the Estimation Filter Message Format Command to set streaming data at a specified rate.
///
///@{

struct mip_3dm_filter_get_base_rate_response
{
    uint16_t rate; ///< [hz]
    
};
typedef struct mip_3dm_filter_get_base_rate_response mip_3dm_filter_get_base_rate_response;
void insert_mip_3dm_filter_get_base_rate_response(struct mip_serializer* serializer, const mip_3dm_filter_get_base_rate_response* self);
void extract_mip_3dm_filter_get_base_rate_response(struct mip_serializer* serializer, mip_3dm_filter_get_base_rate_response* self);

mip_cmd_result mip_3dm_filter_get_base_rate(struct mip_interface* device, uint16_t* rate_out);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_3dm_poll_data  (0x0C,0x0D) Poll Data [C]
/// Poll the device for a message with the specified descriptor set and format.
/// 
/// This function polls for a message using the provided format. The resulting message
/// will maintain the order of descriptors sent in the command and any unrecognized
/// descriptors are ignored. If the format is not provided, the device will attempt to use the
/// stored format (set with the Set Message Format command.) If no format is provided
/// and there is no stored format, the device will respond with a NACK. The reply packet contains
/// an ACK/NACK field. The polled data packet is sent separately as a normal Data packet.
///
///@{

struct mip_3dm_poll_data_command
{
    uint8_t desc_set; ///< Data descriptor set. Must be supported.
    bool suppress_ack; ///< Suppress the usual ACK/NACK reply.
    uint8_t num_descriptors; ///< Number of descriptors in the format list.
    uint8_t descriptors[82]; ///< Descriptor format list.
    
};
typedef struct mip_3dm_poll_data_command mip_3dm_poll_data_command;
void insert_mip_3dm_poll_data_command(struct mip_serializer* serializer, const mip_3dm_poll_data_command* self);
void extract_mip_3dm_poll_data_command(struct mip_serializer* serializer, mip_3dm_poll_data_command* self);

mip_cmd_result mip_3dm_poll_data(struct mip_interface* device, uint8_t desc_set, bool suppress_ack, uint8_t num_descriptors, const uint8_t* descriptors);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_3dm_get_base_rate  (0x0C,0x0E) Get Base Rate [C]
/// Get the base rate for the specified descriptor set in Hz.
///
///@{

struct mip_3dm_get_base_rate_command
{
    uint8_t desc_set; ///< This is the data descriptor set. It must be a supported descriptor.
    
};
typedef struct mip_3dm_get_base_rate_command mip_3dm_get_base_rate_command;
void insert_mip_3dm_get_base_rate_command(struct mip_serializer* serializer, const mip_3dm_get_base_rate_command* self);
void extract_mip_3dm_get_base_rate_command(struct mip_serializer* serializer, mip_3dm_get_base_rate_command* self);

struct mip_3dm_get_base_rate_response
{
    uint8_t desc_set; ///< Echoes the parameter in the command.
    uint16_t rate; ///< Base rate in Hz (0 = variable, unknown, or user-defined rate.  Data will be sent when received).
    
};
typedef struct mip_3dm_get_base_rate_response mip_3dm_get_base_rate_response;
void insert_mip_3dm_get_base_rate_response(struct mip_serializer* serializer, const mip_3dm_get_base_rate_response* self);
void extract_mip_3dm_get_base_rate_response(struct mip_serializer* serializer, mip_3dm_get_base_rate_response* self);

mip_cmd_result mip_3dm_get_base_rate(struct mip_interface* device, uint8_t desc_set, uint16_t* rate_out);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_3dm_message_format  (0x0C,0x0F) Message Format [C]
/// Set, read, or save the format for a given data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
///
///@{

struct mip_3dm_message_format_command
{
    mip_function_selector function;
    uint8_t desc_set; ///< Data descriptor set. Must be supported. When function is SAVE, LOAD, or DEFAULT, can be 0 to apply to all descriptor sets.
    uint8_t num_descriptors; ///< Number of descriptors (limited by payload size)
    mip_descriptor_rate descriptors[82]; ///< List of descriptors and decimations.
    
};
typedef struct mip_3dm_message_format_command mip_3dm_message_format_command;
void insert_mip_3dm_message_format_command(struct mip_serializer* serializer, const mip_3dm_message_format_command* self);
void extract_mip_3dm_message_format_command(struct mip_serializer* serializer, mip_3dm_message_format_command* self);

struct mip_3dm_message_format_response
{
    uint8_t desc_set; ///< Echoes the descriptor set from the command.
    uint8_t num_descriptors; ///< Number of descriptors in the list.
    mip_descriptor_rate descriptors[82]; ///< List of descriptors and decimations.
    
};
typedef struct mip_3dm_message_format_response mip_3dm_message_format_response;
void insert_mip_3dm_message_format_response(struct mip_serializer* serializer, const mip_3dm_message_format_response* self);
void extract_mip_3dm_message_format_response(struct mip_serializer* serializer, mip_3dm_message_format_response* self);

mip_cmd_result mip_3dm_write_message_format(struct mip_interface* device, uint8_t desc_set, uint8_t num_descriptors, const mip_descriptor_rate* descriptors);
mip_cmd_result mip_3dm_read_message_format(struct mip_interface* device, uint8_t desc_set, uint8_t* num_descriptors_out, uint8_t num_descriptors_out_max, mip_descriptor_rate* descriptors_out);
mip_cmd_result mip_3dm_save_message_format(struct mip_interface* device, uint8_t desc_set);
mip_cmd_result mip_3dm_load_message_format(struct mip_interface* device, uint8_t desc_set);
mip_cmd_result mip_3dm_default_message_format(struct mip_interface* device, uint8_t desc_set);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_3dm_nmea_poll_data  (0x0C,0x04) Nmea Poll Data [C]
/// Poll the device for a NMEA message with the specified format.
/// 
/// This function polls for a NMEA message using the provided format.
/// If the format is not provided, the device will attempt to use the
/// stored format (set with the Set NMEA Message Format command.) If no format is provided
/// and there is no stored format, the device will respond with a NACK. The reply packet contains
/// an ACK/NACK field. The polled data packet is sent separately as normal NMEA messages.
///
///@{

struct mip_3dm_nmea_poll_data_command
{
    bool suppress_ack; ///< Suppress the usual ACK/NACK reply.
    uint8_t count; ///< Number of format entries (limited by payload size)
    mip_nmea_message format_entries[40]; ///< List of format entries.
    
};
typedef struct mip_3dm_nmea_poll_data_command mip_3dm_nmea_poll_data_command;
void insert_mip_3dm_nmea_poll_data_command(struct mip_serializer* serializer, const mip_3dm_nmea_poll_data_command* self);
void extract_mip_3dm_nmea_poll_data_command(struct mip_serializer* serializer, mip_3dm_nmea_poll_data_command* self);

mip_cmd_result mip_3dm_nmea_poll_data(struct mip_interface* device, bool suppress_ack, uint8_t count, const mip_nmea_message* format_entries);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_3dm_nmea_message_format  (0x0C,0x0C) Nmea Message Format [C]
/// Set, read, or save the NMEA message format.
///
///@{

struct mip_3dm_nmea_message_format_command
{
    mip_function_selector function;
    uint8_t count; ///< Number of format entries (limited by payload size)
    mip_nmea_message format_entries[40]; ///< List of format entries.
    
};
typedef struct mip_3dm_nmea_message_format_command mip_3dm_nmea_message_format_command;
void insert_mip_3dm_nmea_message_format_command(struct mip_serializer* serializer, const mip_3dm_nmea_message_format_command* self);
void extract_mip_3dm_nmea_message_format_command(struct mip_serializer* serializer, mip_3dm_nmea_message_format_command* self);

struct mip_3dm_nmea_message_format_response
{
    uint8_t count; ///< Number of format entries (limited by payload size)
    mip_nmea_message format_entries[40]; ///< List of format entries.
    
};
typedef struct mip_3dm_nmea_message_format_response mip_3dm_nmea_message_format_response;
void insert_mip_3dm_nmea_message_format_response(struct mip_serializer* serializer, const mip_3dm_nmea_message_format_response* self);
void extract_mip_3dm_nmea_message_format_response(struct mip_serializer* serializer, mip_3dm_nmea_message_format_response* self);

mip_cmd_result mip_3dm_write_nmea_message_format(struct mip_interface* device, uint8_t count, const mip_nmea_message* format_entries);
mip_cmd_result mip_3dm_read_nmea_message_format(struct mip_interface* device, uint8_t* count_out, uint8_t count_out_max, mip_nmea_message* format_entries_out);
mip_cmd_result mip_3dm_save_nmea_message_format(struct mip_interface* device);
mip_cmd_result mip_3dm_load_nmea_message_format(struct mip_interface* device);
mip_cmd_result mip_3dm_default_nmea_message_format(struct mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_3dm_device_settings  (0x0C,0x30) Device Settings [C]
/// Save, Load, or Reset to Default the values for all device settings.
/// 
/// When a save current settings command is issued, a brief data disturbance may occur while all settings are written to non-volatile memory.
/// 
/// This command should have a long timeout as it may take up to 1 second to complete.
///
///@{

struct mip_3dm_device_settings_command
{
    mip_function_selector function;
    
};
typedef struct mip_3dm_device_settings_command mip_3dm_device_settings_command;
void insert_mip_3dm_device_settings_command(struct mip_serializer* serializer, const mip_3dm_device_settings_command* self);
void extract_mip_3dm_device_settings_command(struct mip_serializer* serializer, mip_3dm_device_settings_command* self);

mip_cmd_result mip_3dm_save_device_settings(struct mip_interface* device);
mip_cmd_result mip_3dm_load_device_settings(struct mip_interface* device);
mip_cmd_result mip_3dm_default_device_settings(struct mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_3dm_uart_baudrate  (0x0C,0x40) Uart Baudrate [C]
/// Read, Save, Load, or Reset to Default the baud rate of the main communication channel.
/// 
/// For all functions except 0x01 (use new settings), the new baud rate value is ignored.
/// Please see the device user manual for supported baud rates.
/// 
/// The device will wait until all incoming and outgoing data has been sent, up
/// to a maximum of 250 ms, before applying any change.
/// 
/// No guarantee is provided as to what happens to commands issued during this
/// delay period; They may or may not be processed and any responses aren't
/// guaranteed to be at one rate or the other. The same applies to data packets.
/// 
/// It is highly recommended that the device be idle before issuing this command
/// and that it be issued in its own packet. Users should wait 250 ms after
/// sending this command before further interaction.
///
///@{

struct mip_3dm_uart_baudrate_command
{
    mip_function_selector function;
    uint32_t baud;
    
};
typedef struct mip_3dm_uart_baudrate_command mip_3dm_uart_baudrate_command;
void insert_mip_3dm_uart_baudrate_command(struct mip_serializer* serializer, const mip_3dm_uart_baudrate_command* self);
void extract_mip_3dm_uart_baudrate_command(struct mip_serializer* serializer, mip_3dm_uart_baudrate_command* self);

struct mip_3dm_uart_baudrate_response
{
    uint32_t baud;
    
};
typedef struct mip_3dm_uart_baudrate_response mip_3dm_uart_baudrate_response;
void insert_mip_3dm_uart_baudrate_response(struct mip_serializer* serializer, const mip_3dm_uart_baudrate_response* self);
void extract_mip_3dm_uart_baudrate_response(struct mip_serializer* serializer, mip_3dm_uart_baudrate_response* self);

mip_cmd_result mip_3dm_write_uart_baudrate(struct mip_interface* device, uint32_t baud);
mip_cmd_result mip_3dm_read_uart_baudrate(struct mip_interface* device, uint32_t* baud_out);
mip_cmd_result mip_3dm_save_uart_baudrate(struct mip_interface* device);
mip_cmd_result mip_3dm_load_uart_baudrate(struct mip_interface* device);
mip_cmd_result mip_3dm_default_uart_baudrate(struct mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_3dm_factory_streaming  (0x0C,0x10) Factory Streaming [C]
/// Configures the device for recording data for technical support.
/// 
/// This command will configure all available data streams to predefined
/// formats designed to be used with technical support.
///
///@{

typedef uint8_t mip_3dm_factory_streaming_command_action;
static const mip_3dm_factory_streaming_command_action MIP_3DM_FACTORY_STREAMING_COMMAND_ACTION_OVERWRITE = 0; ///<  Replaces the message format(s), removing any existing descriptors.
static const mip_3dm_factory_streaming_command_action MIP_3DM_FACTORY_STREAMING_COMMAND_ACTION_MERGE     = 1; ///<  Merges support descriptors into existing format(s). May reorder descriptors.
static const mip_3dm_factory_streaming_command_action MIP_3DM_FACTORY_STREAMING_COMMAND_ACTION_ADD       = 2; ///<  Adds descriptors to the current message format(s) without changing existing descriptors. May result in duplicates.

struct mip_3dm_factory_streaming_command
{
    mip_3dm_factory_streaming_command_action action;
    uint8_t reserved; ///< Reserved. Set to 0x00.
    
};
typedef struct mip_3dm_factory_streaming_command mip_3dm_factory_streaming_command;
void insert_mip_3dm_factory_streaming_command(struct mip_serializer* serializer, const mip_3dm_factory_streaming_command* self);
void extract_mip_3dm_factory_streaming_command(struct mip_serializer* serializer, mip_3dm_factory_streaming_command* self);

void insert_mip_3dm_factory_streaming_command_action(struct mip_serializer* serializer, const mip_3dm_factory_streaming_command_action self);
void extract_mip_3dm_factory_streaming_command_action(struct mip_serializer* serializer, mip_3dm_factory_streaming_command_action* self);

mip_cmd_result mip_3dm_factory_streaming(struct mip_interface* device, mip_3dm_factory_streaming_command_action action, uint8_t reserved);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_3dm_datastream_control  (0x0C,0x11) Datastream Control [C]
/// Enable/disable the selected data stream.
/// 
/// Each data stream (descriptor set) can be enabled or disabled.
/// The default for the device is all streams enabled.
/// For all functions except 0x01 (use new setting),
/// the new enable flag value is ignored and can be omitted.
///
///@{

enum { MIP_3DM_DATASTREAM_CONTROL_COMMAND_LEGACY_IMU_STREAM = 0x01 };
enum { MIP_3DM_DATASTREAM_CONTROL_COMMAND_LEGACY_GNSS_STREAM = 0x02 };
enum { MIP_3DM_DATASTREAM_CONTROL_COMMAND_LEGACY_FILTER_STREAM = 0x03 };
enum { MIP_3DM_DATASTREAM_CONTROL_COMMAND_ALL_STREAMS = 0x00 };
struct mip_3dm_datastream_control_command
{
    mip_function_selector function;
    uint8_t desc_set; ///< The descriptor set of the stream to control. When function is SAVE, LOAD, or DEFAULT, can be ALL_STREAMS(0) to apply to all descriptor sets. On Generation 5 products, this must be one of the above legacy constants.
    bool enable; ///< True or false to enable or disable the stream.
    
};
typedef struct mip_3dm_datastream_control_command mip_3dm_datastream_control_command;
void insert_mip_3dm_datastream_control_command(struct mip_serializer* serializer, const mip_3dm_datastream_control_command* self);
void extract_mip_3dm_datastream_control_command(struct mip_serializer* serializer, mip_3dm_datastream_control_command* self);

struct mip_3dm_datastream_control_response
{
    uint8_t desc_set;
    bool enabled;
    
};
typedef struct mip_3dm_datastream_control_response mip_3dm_datastream_control_response;
void insert_mip_3dm_datastream_control_response(struct mip_serializer* serializer, const mip_3dm_datastream_control_response* self);
void extract_mip_3dm_datastream_control_response(struct mip_serializer* serializer, mip_3dm_datastream_control_response* self);

mip_cmd_result mip_3dm_write_datastream_control(struct mip_interface* device, uint8_t desc_set, bool enable);
mip_cmd_result mip_3dm_read_datastream_control(struct mip_interface* device, uint8_t desc_set, bool* enabled_out);
mip_cmd_result mip_3dm_save_datastream_control(struct mip_interface* device, uint8_t desc_set);
mip_cmd_result mip_3dm_load_datastream_control(struct mip_interface* device, uint8_t desc_set);
mip_cmd_result mip_3dm_default_datastream_control(struct mip_interface* device, uint8_t desc_set);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_3dm_constellation_settings  (0x0C,0x21) Constellation Settings [C]
/// This command configures which satellite constellations are enabled and how many channels are dedicated to tracking each constellation.
/// 
/// Maximum number of tracking channels to use (total for all constellations):
/// 0 to max_channels_available (from reply message)
/// 
/// For each constellation you wish to use, include a ConstellationSettings struct.  Note the following:
/// 
/// Total number of tracking channels (sum of "reserved_channels" for all constellations) must be <= 32:
/// 0 -> 32 Number of reserved channels
/// 0 -> 32 Max number of channels (>= reserved channels)
/// 
/// The factory default setting is: GPS and GLONASS enabled.  Min/Max for GPS = 8/16, GLONASS = 8/14, SBAS = 1/3, QZSS = 0/3.
/// 
/// Warning: SBAS functionality shall not be used in "safety of life" applications!
/// Warning: Any setting that causes the total reserved channels to exceed 32 will result in a NACK.
/// Warning: You cannot enable GLONASS and BeiDou at the same time.
/// Note:    Enabling SBAS and QZSS augments GPS accuracy.
/// Note:    It is recommended to disable GLONASS and BeiDou if a GPS-only antenna or GPS-only SAW filter is used.
///
///@{

typedef uint8_t mip_3dm_constellation_settings_command_constellation_id;
static const mip_3dm_constellation_settings_command_constellation_id MIP_3DM_CONSTELLATION_SETTINGS_COMMAND_CONSTELLATION_ID_GPS     = 0; ///<  GPS (G1-G32)
static const mip_3dm_constellation_settings_command_constellation_id MIP_3DM_CONSTELLATION_SETTINGS_COMMAND_CONSTELLATION_ID_SBAS    = 1; ///<  SBAS (S120-S158)
static const mip_3dm_constellation_settings_command_constellation_id MIP_3DM_CONSTELLATION_SETTINGS_COMMAND_CONSTELLATION_ID_GALILEO = 2; ///<  GALILEO (E1-E36)
static const mip_3dm_constellation_settings_command_constellation_id MIP_3DM_CONSTELLATION_SETTINGS_COMMAND_CONSTELLATION_ID_BEIDOU  = 3; ///<  BeiDou (B1-B37)
static const mip_3dm_constellation_settings_command_constellation_id MIP_3DM_CONSTELLATION_SETTINGS_COMMAND_CONSTELLATION_ID_QZSS    = 5; ///<  QZSS (Q1-Q5)
static const mip_3dm_constellation_settings_command_constellation_id MIP_3DM_CONSTELLATION_SETTINGS_COMMAND_CONSTELLATION_ID_GLONASS = 6; ///<  GLONASS (R1-R32)

typedef uint16_t mip_3dm_constellation_settings_command_option_flags;
static const mip_3dm_constellation_settings_command_option_flags MIP_3DM_CONSTELLATION_SETTINGS_COMMAND_OPTION_FLAGS_NONE   = 0x0000;
static const mip_3dm_constellation_settings_command_option_flags MIP_3DM_CONSTELLATION_SETTINGS_COMMAND_OPTION_FLAGS_L1SAIF = 0x0001; ///<  Available only for QZSS
static const mip_3dm_constellation_settings_command_option_flags MIP_3DM_CONSTELLATION_SETTINGS_COMMAND_OPTION_FLAGS_ALL    = 0x0001;

struct mip_3dm_constellation_settings_command_settings
{
    mip_3dm_constellation_settings_command_constellation_id constellation_id; ///< Constellation ID
    uint8_t enable; ///< Enable/Disable constellation
    uint8_t reserved_channels; ///< Minimum number of channels reserved for this constellation
    uint8_t max_channels; ///< Maximum number of channels to use for this constallation
    mip_3dm_constellation_settings_command_option_flags option_flags; ///< Constellation option Flags
    
};
typedef struct mip_3dm_constellation_settings_command_settings mip_3dm_constellation_settings_command_settings;
struct mip_3dm_constellation_settings_command
{
    mip_function_selector function;
    uint16_t max_channels;
    uint8_t config_count;
    mip_3dm_constellation_settings_command_settings settings[42];
    
};
typedef struct mip_3dm_constellation_settings_command mip_3dm_constellation_settings_command;
void insert_mip_3dm_constellation_settings_command(struct mip_serializer* serializer, const mip_3dm_constellation_settings_command* self);
void extract_mip_3dm_constellation_settings_command(struct mip_serializer* serializer, mip_3dm_constellation_settings_command* self);

void insert_mip_3dm_constellation_settings_command_constellation_id(struct mip_serializer* serializer, const mip_3dm_constellation_settings_command_constellation_id self);
void extract_mip_3dm_constellation_settings_command_constellation_id(struct mip_serializer* serializer, mip_3dm_constellation_settings_command_constellation_id* self);

void insert_mip_3dm_constellation_settings_command_option_flags(struct mip_serializer* serializer, const mip_3dm_constellation_settings_command_option_flags self);
void extract_mip_3dm_constellation_settings_command_option_flags(struct mip_serializer* serializer, mip_3dm_constellation_settings_command_option_flags* self);

void insert_mip_3dm_constellation_settings_command_settings(struct mip_serializer* serializer, const mip_3dm_constellation_settings_command_settings* self);
void extract_mip_3dm_constellation_settings_command_settings(struct mip_serializer* serializer, mip_3dm_constellation_settings_command_settings* self);

struct mip_3dm_constellation_settings_response
{
    uint16_t max_channels_available; ///< Maximum channels available
    uint16_t max_channels_use; ///< Maximum channels to use
    uint8_t config_count; ///< Number of constellation configurations
    mip_3dm_constellation_settings_command_settings settings[42]; ///< Constellation Settings
    
};
typedef struct mip_3dm_constellation_settings_response mip_3dm_constellation_settings_response;
void insert_mip_3dm_constellation_settings_response(struct mip_serializer* serializer, const mip_3dm_constellation_settings_response* self);
void extract_mip_3dm_constellation_settings_response(struct mip_serializer* serializer, mip_3dm_constellation_settings_response* self);

mip_cmd_result mip_3dm_write_constellation_settings(struct mip_interface* device, uint16_t max_channels, uint8_t config_count, const mip_3dm_constellation_settings_command_settings* settings);
mip_cmd_result mip_3dm_read_constellation_settings(struct mip_interface* device, uint16_t* max_channels_available_out, uint16_t* max_channels_use_out, uint8_t* config_count_out, uint8_t config_count_out_max, mip_3dm_constellation_settings_command_settings* settings_out);
mip_cmd_result mip_3dm_save_constellation_settings(struct mip_interface* device);
mip_cmd_result mip_3dm_load_constellation_settings(struct mip_interface* device);
mip_cmd_result mip_3dm_default_constellation_settings(struct mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_3dm_gnss_sbas_settings  (0x0C,0x22) Gnss Sbas Settings [C]
/// Configure the SBAS subsystem
/// 
/// 
/// 
///
///@{

typedef uint16_t mip_3dm_gnss_sbas_settings_command_sbasoptions;
static const mip_3dm_gnss_sbas_settings_command_sbasoptions MIP_3DM_GNSS_SBAS_SETTINGS_COMMAND_SBASOPTIONS_NONE               = 0x0000;
static const mip_3dm_gnss_sbas_settings_command_sbasoptions MIP_3DM_GNSS_SBAS_SETTINGS_COMMAND_SBASOPTIONS_ENABLE_RANGING     = 0x0001; ///<  Use SBAS pseudo-ranges in position solution
static const mip_3dm_gnss_sbas_settings_command_sbasoptions MIP_3DM_GNSS_SBAS_SETTINGS_COMMAND_SBASOPTIONS_ENABLE_CORRECTIONS = 0x0002; ///<  Use SBAS differential corrections
static const mip_3dm_gnss_sbas_settings_command_sbasoptions MIP_3DM_GNSS_SBAS_SETTINGS_COMMAND_SBASOPTIONS_APPLY_INTEGRITY    = 0x0004; ///<  Use SBAS integrity information.  If enabled, only GPS satellites for which integrity information is available will be used.
static const mip_3dm_gnss_sbas_settings_command_sbasoptions MIP_3DM_GNSS_SBAS_SETTINGS_COMMAND_SBASOPTIONS_ALL                = 0x0007;

struct mip_3dm_gnss_sbas_settings_command
{
    mip_function_selector function;
    uint8_t enable_sbas; ///< 0 - SBAS Disabled, 1 - SBAS enabled
    mip_3dm_gnss_sbas_settings_command_sbasoptions sbas_options; ///< SBAS options, see definition
    uint8_t num_included_prns; ///< Number of SBAS PRNs to include in search (0 = include all)
    uint16_t included_prns[39]; ///< List of specific SBAS PRNs to search for
    
};
typedef struct mip_3dm_gnss_sbas_settings_command mip_3dm_gnss_sbas_settings_command;
void insert_mip_3dm_gnss_sbas_settings_command(struct mip_serializer* serializer, const mip_3dm_gnss_sbas_settings_command* self);
void extract_mip_3dm_gnss_sbas_settings_command(struct mip_serializer* serializer, mip_3dm_gnss_sbas_settings_command* self);

void insert_mip_3dm_gnss_sbas_settings_command_sbasoptions(struct mip_serializer* serializer, const mip_3dm_gnss_sbas_settings_command_sbasoptions self);
void extract_mip_3dm_gnss_sbas_settings_command_sbasoptions(struct mip_serializer* serializer, mip_3dm_gnss_sbas_settings_command_sbasoptions* self);

struct mip_3dm_gnss_sbas_settings_response
{
    uint8_t enable_sbas; ///< 0 - SBAS Disabled, 1 - SBAS enabled
    mip_3dm_gnss_sbas_settings_command_sbasoptions sbas_options; ///< SBAS options, see definition
    uint8_t num_included_prns; ///< Number of SBAS PRNs to include in search (0 = include all)
    uint16_t included_prns[39]; ///< List of specific SBAS PRNs to search for
    
};
typedef struct mip_3dm_gnss_sbas_settings_response mip_3dm_gnss_sbas_settings_response;
void insert_mip_3dm_gnss_sbas_settings_response(struct mip_serializer* serializer, const mip_3dm_gnss_sbas_settings_response* self);
void extract_mip_3dm_gnss_sbas_settings_response(struct mip_serializer* serializer, mip_3dm_gnss_sbas_settings_response* self);

mip_cmd_result mip_3dm_write_gnss_sbas_settings(struct mip_interface* device, uint8_t enable_sbas, mip_3dm_gnss_sbas_settings_command_sbasoptions sbas_options, uint8_t num_included_prns, const uint16_t* included_prns);
mip_cmd_result mip_3dm_read_gnss_sbas_settings(struct mip_interface* device, uint8_t* enable_sbas_out, mip_3dm_gnss_sbas_settings_command_sbasoptions* sbas_options_out, uint8_t* num_included_prns_out, uint8_t num_included_prns_out_max, uint16_t* included_prns_out);
mip_cmd_result mip_3dm_save_gnss_sbas_settings(struct mip_interface* device);
mip_cmd_result mip_3dm_load_gnss_sbas_settings(struct mip_interface* device);
mip_cmd_result mip_3dm_default_gnss_sbas_settings(struct mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_3dm_gnss_assisted_fix  (0x0C,0x23) Gnss Assisted Fix [C]
/// Set the options for assisted GNSS fix.
/// 
/// Devices that implement this command have a dedicated GNSS flash memory and a non-volatile FRAM.
/// These storage mechanisms are used to retain information about the last good GNSS fix. This can greatly reduces the TTFF (Time To First Fix) depending on the age of the stored information.
/// The TTFF can be as low as one second, or up to the equivalent of a cold start. There is a small increase in power used when enabling assisted fix.
/// 
/// The fastest fix will be obtained by supplying the device with a GNSS Assist Time Update message containing the current GPS time immediately after subsequent power up.
/// This allows the device to determine if the last GNSS information saved is still fresh enough to improve the TTFF.
/// 
/// NOTE: Non-volatile GNSS memory is cleared when going from an enabled state to a disabled state.
/// WARNING: The clearing operation results in an erase operation on the GNSS Flash. The flash has a limited durability of 100,000 write/erase cycles
///
///@{

typedef uint8_t mip_3dm_gnss_assisted_fix_command_assisted_fix_option;
static const mip_3dm_gnss_assisted_fix_command_assisted_fix_option MIP_3DM_GNSS_ASSISTED_FIX_COMMAND_ASSISTED_FIX_OPTION_NONE    = 0; ///<  No assisted fix (default)
static const mip_3dm_gnss_assisted_fix_command_assisted_fix_option MIP_3DM_GNSS_ASSISTED_FIX_COMMAND_ASSISTED_FIX_OPTION_ENABLED = 1; ///<  Enable assisted fix

struct mip_3dm_gnss_assisted_fix_command
{
    mip_function_selector function;
    mip_3dm_gnss_assisted_fix_command_assisted_fix_option option; ///< Assisted fix options
    uint8_t flags; ///< Assisted fix flags (set to 0xFF)
    
};
typedef struct mip_3dm_gnss_assisted_fix_command mip_3dm_gnss_assisted_fix_command;
void insert_mip_3dm_gnss_assisted_fix_command(struct mip_serializer* serializer, const mip_3dm_gnss_assisted_fix_command* self);
void extract_mip_3dm_gnss_assisted_fix_command(struct mip_serializer* serializer, mip_3dm_gnss_assisted_fix_command* self);

void insert_mip_3dm_gnss_assisted_fix_command_assisted_fix_option(struct mip_serializer* serializer, const mip_3dm_gnss_assisted_fix_command_assisted_fix_option self);
void extract_mip_3dm_gnss_assisted_fix_command_assisted_fix_option(struct mip_serializer* serializer, mip_3dm_gnss_assisted_fix_command_assisted_fix_option* self);

struct mip_3dm_gnss_assisted_fix_response
{
    mip_3dm_gnss_assisted_fix_command_assisted_fix_option option; ///< Assisted fix options
    uint8_t flags; ///< Assisted fix flags (set to 0xFF)
    
};
typedef struct mip_3dm_gnss_assisted_fix_response mip_3dm_gnss_assisted_fix_response;
void insert_mip_3dm_gnss_assisted_fix_response(struct mip_serializer* serializer, const mip_3dm_gnss_assisted_fix_response* self);
void extract_mip_3dm_gnss_assisted_fix_response(struct mip_serializer* serializer, mip_3dm_gnss_assisted_fix_response* self);

mip_cmd_result mip_3dm_write_gnss_assisted_fix(struct mip_interface* device, mip_3dm_gnss_assisted_fix_command_assisted_fix_option option, uint8_t flags);
mip_cmd_result mip_3dm_read_gnss_assisted_fix(struct mip_interface* device, mip_3dm_gnss_assisted_fix_command_assisted_fix_option* option_out, uint8_t* flags_out);
mip_cmd_result mip_3dm_save_gnss_assisted_fix(struct mip_interface* device);
mip_cmd_result mip_3dm_load_gnss_assisted_fix(struct mip_interface* device);
mip_cmd_result mip_3dm_default_gnss_assisted_fix(struct mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_3dm_gnss_time_assistance  (0x0C,0x24) Gnss Time Assistance [C]
/// Provide the GNSS subsystem with initial time information.
/// 
/// This message is required immediately after power up if GNSS Assist was enabled when the device was powered off.
/// This will initialize the subsystem clock to help reduce the time to first fix (TTFF).
///
///@{

struct mip_3dm_gnss_time_assistance_command
{
    mip_function_selector function;
    double tow; ///< GPS Time of week [seconds]
    uint16_t week_number; ///< GPS Weeks since 1980 [weeks]
    float accuracy; ///< Accuracy of time information [seconds]
    
};
typedef struct mip_3dm_gnss_time_assistance_command mip_3dm_gnss_time_assistance_command;
void insert_mip_3dm_gnss_time_assistance_command(struct mip_serializer* serializer, const mip_3dm_gnss_time_assistance_command* self);
void extract_mip_3dm_gnss_time_assistance_command(struct mip_serializer* serializer, mip_3dm_gnss_time_assistance_command* self);

struct mip_3dm_gnss_time_assistance_response
{
    double tow; ///< GPS Time of week [seconds]
    uint16_t week_number; ///< GPS Weeks since 1980 [weeks]
    float accuracy; ///< Accuracy of time information [seconds]
    
};
typedef struct mip_3dm_gnss_time_assistance_response mip_3dm_gnss_time_assistance_response;
void insert_mip_3dm_gnss_time_assistance_response(struct mip_serializer* serializer, const mip_3dm_gnss_time_assistance_response* self);
void extract_mip_3dm_gnss_time_assistance_response(struct mip_serializer* serializer, mip_3dm_gnss_time_assistance_response* self);

mip_cmd_result mip_3dm_write_gnss_time_assistance(struct mip_interface* device, double tow, uint16_t week_number, float accuracy);
mip_cmd_result mip_3dm_read_gnss_time_assistance(struct mip_interface* device, double* tow_out, uint16_t* week_number_out, float* accuracy_out);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_3dm_imu_lowpass_filter  (0x0C,0x50) Imu Lowpass Filter [C]
/// Advanced configuration for the IMU data quantity low-pass filters.
/// 
/// Deprecated, use the lowpass filter (0x0C,0x54) command instead.
/// 
/// The scaled data quantities are by default filtered through a single-pole IIR low-pass filter
/// which is configured with a -3dB cutoff frequency of half the reporting frequency (set by
/// decimation factor in the IMU Message Format command) to prevent aliasing on a per data
/// quantity basis. This advanced configuration command allows for the cutoff frequency to
/// be configured independently of the data reporting frequency as well as allowing for a
/// complete bypass of the digital low-pass filter.
/// 
/// Possible data descriptors:
/// 0x04 - Scaled accelerometer data
/// 0x05 - Scaled gyro data
/// 0x06 - Scaled magnetometer data (if applicable)
/// 0x17 - Scaled pressure data (if applicable)
///
///@{

struct mip_3dm_imu_lowpass_filter_command
{
    mip_function_selector function;
    uint8_t target_descriptor; ///< Field descriptor of filtered quantity within the Sensor data set. Supported values are accel (0x04), gyro (0x05), mag (0x06), and pressure (0x17), provided the data is supported by the device. Except with the READ function selector, this can be 0 to apply to all of the above quantities.
    bool enable; ///< The target data will be filtered if this is true.
    bool manual; ///< If false, the cutoff frequency is set to half of the streaming rate as configured by the message format command. Otherwise, the cutoff frequency is set according to the following 'frequency' parameter.
    uint16_t frequency; ///< -3dB cutoff frequency in Hz. Will not affect filtering if 'manual' is false.
    uint8_t reserved; ///< Reserved, set to 0x00.
    
};
typedef struct mip_3dm_imu_lowpass_filter_command mip_3dm_imu_lowpass_filter_command;
void insert_mip_3dm_imu_lowpass_filter_command(struct mip_serializer* serializer, const mip_3dm_imu_lowpass_filter_command* self);
void extract_mip_3dm_imu_lowpass_filter_command(struct mip_serializer* serializer, mip_3dm_imu_lowpass_filter_command* self);

struct mip_3dm_imu_lowpass_filter_response
{
    uint8_t target_descriptor;
    bool enable; ///< True if the filter is currently enabled.
    bool manual; ///< True if the filter cutoff was manually configured.
    uint16_t frequency; ///< The cutoff frequency of the filter. If the filter is in auto mode, this value is unspecified.
    uint8_t reserved; ///< Reserved and must be ignored.
    
};
typedef struct mip_3dm_imu_lowpass_filter_response mip_3dm_imu_lowpass_filter_response;
void insert_mip_3dm_imu_lowpass_filter_response(struct mip_serializer* serializer, const mip_3dm_imu_lowpass_filter_response* self);
void extract_mip_3dm_imu_lowpass_filter_response(struct mip_serializer* serializer, mip_3dm_imu_lowpass_filter_response* self);

mip_cmd_result mip_3dm_write_imu_lowpass_filter(struct mip_interface* device, uint8_t target_descriptor, bool enable, bool manual, uint16_t frequency, uint8_t reserved);
mip_cmd_result mip_3dm_read_imu_lowpass_filter(struct mip_interface* device, uint8_t target_descriptor, bool* enable_out, bool* manual_out, uint16_t* frequency_out, uint8_t* reserved_out);
mip_cmd_result mip_3dm_save_imu_lowpass_filter(struct mip_interface* device, uint8_t target_descriptor);
mip_cmd_result mip_3dm_load_imu_lowpass_filter(struct mip_interface* device, uint8_t target_descriptor);
mip_cmd_result mip_3dm_default_imu_lowpass_filter(struct mip_interface* device, uint8_t target_descriptor);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_3dm_pps_source  (0x0C,0x28) Pps Source [C]
/// Controls the Pulse Per Second (PPS) source.
///
///@{

typedef uint8_t mip_3dm_pps_source_command_source;
static const mip_3dm_pps_source_command_source MIP_3DM_PPS_SOURCE_COMMAND_SOURCE_DISABLED   = 0; ///<  PPS output is disabled. Not valid for PPS source command.
static const mip_3dm_pps_source_command_source MIP_3DM_PPS_SOURCE_COMMAND_SOURCE_RECEIVER_1 = 1; ///<  PPS is provided by GNSS receiver 1.
static const mip_3dm_pps_source_command_source MIP_3DM_PPS_SOURCE_COMMAND_SOURCE_RECEIVER_2 = 2; ///<  PPS is provided by GNSS receiver 2.
static const mip_3dm_pps_source_command_source MIP_3DM_PPS_SOURCE_COMMAND_SOURCE_GPIO       = 3; ///<  PPS is provided to an external GPIO pin. Use the GPIO Setup command to choose and configure the pin.
static const mip_3dm_pps_source_command_source MIP_3DM_PPS_SOURCE_COMMAND_SOURCE_GENERATED  = 4; ///<  PPS is generated from the system oscillator.

struct mip_3dm_pps_source_command
{
    mip_function_selector function;
    mip_3dm_pps_source_command_source source;
    
};
typedef struct mip_3dm_pps_source_command mip_3dm_pps_source_command;
void insert_mip_3dm_pps_source_command(struct mip_serializer* serializer, const mip_3dm_pps_source_command* self);
void extract_mip_3dm_pps_source_command(struct mip_serializer* serializer, mip_3dm_pps_source_command* self);

void insert_mip_3dm_pps_source_command_source(struct mip_serializer* serializer, const mip_3dm_pps_source_command_source self);
void extract_mip_3dm_pps_source_command_source(struct mip_serializer* serializer, mip_3dm_pps_source_command_source* self);

struct mip_3dm_pps_source_response
{
    mip_3dm_pps_source_command_source source;
    
};
typedef struct mip_3dm_pps_source_response mip_3dm_pps_source_response;
void insert_mip_3dm_pps_source_response(struct mip_serializer* serializer, const mip_3dm_pps_source_response* self);
void extract_mip_3dm_pps_source_response(struct mip_serializer* serializer, mip_3dm_pps_source_response* self);

mip_cmd_result mip_3dm_write_pps_source(struct mip_interface* device, mip_3dm_pps_source_command_source source);
mip_cmd_result mip_3dm_read_pps_source(struct mip_interface* device, mip_3dm_pps_source_command_source* source_out);
mip_cmd_result mip_3dm_save_pps_source(struct mip_interface* device);
mip_cmd_result mip_3dm_load_pps_source(struct mip_interface* device);
mip_cmd_result mip_3dm_default_pps_source(struct mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_3dm_gpio_config  (0x0C,0x41) Gpio Config [C]
/// Configures the user GPIO pins on the connector for use with several built-in functions or for general input or output.
/// 
/// GPIO pins are device-dependent. Some features are only available on
/// certain pins. Some behaviors require specific configurations.
/// Consult the device user manual for restrictions and default settings.
/// 
/// To avoid glitches on GPIOs configured as an output in a mode other than
/// GPIO, always configure the relevant function before setting up the pin
/// with this command. Otherwise, the pin state will be undefined between
/// this command and the one to set up the feature. For input pins, use
/// this command first so the state is well-defined when the feature is
/// initialized.
/// 
/// Some configurations can only be active on one pin at a time. If such
/// configuration is applied to a second pin, the second one will take
/// precedence and the original pin's configuration will be reset.
/// 
///
///@{

typedef uint8_t mip_3dm_gpio_config_command_feature;
static const mip_3dm_gpio_config_command_feature MIP_3DM_GPIO_CONFIG_COMMAND_FEATURE_UNUSED    = 0; ///<  The pin is not used. It may be technically possible to read the pin state in this mode, but this is not guaranteed to be true of all devices or pins.
static const mip_3dm_gpio_config_command_feature MIP_3DM_GPIO_CONFIG_COMMAND_FEATURE_GPIO      = 1; ///<  General purpose input or output. Use this for direct control of pin output state or to stream the state of the pin.
static const mip_3dm_gpio_config_command_feature MIP_3DM_GPIO_CONFIG_COMMAND_FEATURE_PPS       = 2; ///<  Pulse per second input or output.
static const mip_3dm_gpio_config_command_feature MIP_3DM_GPIO_CONFIG_COMMAND_FEATURE_ENCODER   = 3; ///<  Motor encoder/odometer input.
static const mip_3dm_gpio_config_command_feature MIP_3DM_GPIO_CONFIG_COMMAND_FEATURE_TIMESTAMP = 4; ///<  Precision Timestamping. Use with Event Trigger Configuration (0x0C,0x2E).
static const mip_3dm_gpio_config_command_feature MIP_3DM_GPIO_CONFIG_COMMAND_FEATURE_UART      = 5; ///<  UART data or control lines.

typedef uint8_t mip_3dm_gpio_config_command_behavior;
static const mip_3dm_gpio_config_command_behavior MIP_3DM_GPIO_CONFIG_COMMAND_BEHAVIOR_UNUSED            = 0;  ///<  Use 0 unless otherwise specified.
static const mip_3dm_gpio_config_command_behavior MIP_3DM_GPIO_CONFIG_COMMAND_BEHAVIOR_GPIO_INPUT        = 1;  ///<  Pin will be an input. This can be used to stream or poll the value and is the default setting.
static const mip_3dm_gpio_config_command_behavior MIP_3DM_GPIO_CONFIG_COMMAND_BEHAVIOR_GPIO_OUTPUT_LOW   = 2;  ///<  Pin is an output initially in the LOW state. This state will be restored during system startup if the configuration is saved.
static const mip_3dm_gpio_config_command_behavior MIP_3DM_GPIO_CONFIG_COMMAND_BEHAVIOR_GPIO_OUTPUT_HIGH  = 3;  ///<  Pin is an output initially in the HIGH state. This state will be restored during system startup if the configuration is saved.
static const mip_3dm_gpio_config_command_behavior MIP_3DM_GPIO_CONFIG_COMMAND_BEHAVIOR_PPS_INPUT         = 1;  ///<  Pin will receive the pulse-per-second signal. Only one pin can have this behavior. This will only work if the PPS Source command is configured to GPIO.
static const mip_3dm_gpio_config_command_behavior MIP_3DM_GPIO_CONFIG_COMMAND_BEHAVIOR_PPS_OUTPUT        = 2;  ///<  Pin will transmit the pulse-per-second signal from the device.
static const mip_3dm_gpio_config_command_behavior MIP_3DM_GPIO_CONFIG_COMMAND_BEHAVIOR_ENCODER_A         = 1;  ///<  Encoder "A" quadrature input. Only one pin can have this behavior. The last command to set this behavior will take precedence.
static const mip_3dm_gpio_config_command_behavior MIP_3DM_GPIO_CONFIG_COMMAND_BEHAVIOR_ENCODER_B         = 2;  ///<  Encoder "B" quadrature input. Only one pin can have this behavior. The last command to set this behavior will take precedence.
static const mip_3dm_gpio_config_command_behavior MIP_3DM_GPIO_CONFIG_COMMAND_BEHAVIOR_TIMESTAMP_RISING  = 1;  ///<  Rising edges will be timestamped.
static const mip_3dm_gpio_config_command_behavior MIP_3DM_GPIO_CONFIG_COMMAND_BEHAVIOR_TIMESTAMP_FALLING = 2;  ///<  Falling edges will be timestamped.
static const mip_3dm_gpio_config_command_behavior MIP_3DM_GPIO_CONFIG_COMMAND_BEHAVIOR_TIMESTAMP_EITHER  = 3;  ///<  Both rising and falling edges will be timestamped.
static const mip_3dm_gpio_config_command_behavior MIP_3DM_GPIO_CONFIG_COMMAND_BEHAVIOR_UART_PORT2_TX     = 33; ///<  (0x21) UART port 2 transmit.
static const mip_3dm_gpio_config_command_behavior MIP_3DM_GPIO_CONFIG_COMMAND_BEHAVIOR_UART_PORT2_RX     = 34; ///<  (0x22) UART port 2 receive.
static const mip_3dm_gpio_config_command_behavior MIP_3DM_GPIO_CONFIG_COMMAND_BEHAVIOR_UART_PORT3_TX     = 49; ///<  (0x31) UART port 3 transmit.
static const mip_3dm_gpio_config_command_behavior MIP_3DM_GPIO_CONFIG_COMMAND_BEHAVIOR_UART_PORT3_RX     = 50; ///<  (0x32) UART port 3 receive.

typedef uint8_t mip_3dm_gpio_config_command_pin_mode;
static const mip_3dm_gpio_config_command_pin_mode MIP_3DM_GPIO_CONFIG_COMMAND_PIN_MODE_NONE       = 0x00;
static const mip_3dm_gpio_config_command_pin_mode MIP_3DM_GPIO_CONFIG_COMMAND_PIN_MODE_OPEN_DRAIN = 0x01; ///<  The pin will be an open-drain output. The state will be either LOW or FLOATING instead of LOW or HIGH, respectively. This is used to connect multiple open-drain outputs from several devices. An internal or external pull-up resistor is typically used in combination. The maximum voltage of an open drain output is subject to the device maximum input voltage range found in the specifications.
static const mip_3dm_gpio_config_command_pin_mode MIP_3DM_GPIO_CONFIG_COMMAND_PIN_MODE_PULLDOWN   = 0x02; ///<  The pin will have an internal pull-down resistor enabled. This is useful for connecting inputs to signals which can only be pulled high such as mechanical switches. Cannot be used in combination with pull-up. See the device specifications for the resistance value.
static const mip_3dm_gpio_config_command_pin_mode MIP_3DM_GPIO_CONFIG_COMMAND_PIN_MODE_PULLUP     = 0x04; ///<  The pin will have an internal pull-up resistor enabled. Useful for connecting inputs to signals which can only be pulled low such as mechanical switches, or in combination with an open drain output. Cannot be used in combination with pull-down. See the device specifications for the resistance value. Use of this mode may restrict the maximum allowed input voltage. See the device datasheet for details.
static const mip_3dm_gpio_config_command_pin_mode MIP_3DM_GPIO_CONFIG_COMMAND_PIN_MODE_ALL        = 0x07;

struct mip_3dm_gpio_config_command
{
    mip_function_selector function;
    uint8_t pin; ///< GPIO pin number counting from 1. For save, load, and default function selectors, this can be 0 to select all pins.
    mip_3dm_gpio_config_command_feature feature; ///< Determines how the pin will be used.
    mip_3dm_gpio_config_command_behavior behavior; ///< Select an appropriate value from the enumeration based on the selected feature (e.g. for PPS, select one of the values prefixed with PPS_.)
    mip_3dm_gpio_config_command_pin_mode pin_mode; ///< GPIO configuration. May be restricted depending on device, pin, feature, and behavior. See device user manual.
    
};
typedef struct mip_3dm_gpio_config_command mip_3dm_gpio_config_command;
void insert_mip_3dm_gpio_config_command(struct mip_serializer* serializer, const mip_3dm_gpio_config_command* self);
void extract_mip_3dm_gpio_config_command(struct mip_serializer* serializer, mip_3dm_gpio_config_command* self);

void insert_mip_3dm_gpio_config_command_feature(struct mip_serializer* serializer, const mip_3dm_gpio_config_command_feature self);
void extract_mip_3dm_gpio_config_command_feature(struct mip_serializer* serializer, mip_3dm_gpio_config_command_feature* self);

void insert_mip_3dm_gpio_config_command_behavior(struct mip_serializer* serializer, const mip_3dm_gpio_config_command_behavior self);
void extract_mip_3dm_gpio_config_command_behavior(struct mip_serializer* serializer, mip_3dm_gpio_config_command_behavior* self);

void insert_mip_3dm_gpio_config_command_pin_mode(struct mip_serializer* serializer, const mip_3dm_gpio_config_command_pin_mode self);
void extract_mip_3dm_gpio_config_command_pin_mode(struct mip_serializer* serializer, mip_3dm_gpio_config_command_pin_mode* self);

struct mip_3dm_gpio_config_response
{
    uint8_t pin; ///< GPIO pin number counting from 1. For save, load, and default function selectors, this can be 0 to select all pins.
    mip_3dm_gpio_config_command_feature feature; ///< Determines how the pin will be used.
    mip_3dm_gpio_config_command_behavior behavior; ///< Select an appropriate value from the enumeration based on the selected feature (e.g. for PPS, select one of the values prefixed with PPS_.)
    mip_3dm_gpio_config_command_pin_mode pin_mode; ///< GPIO configuration. May be restricted depending on device, pin, feature, and behavior. See device user manual.
    
};
typedef struct mip_3dm_gpio_config_response mip_3dm_gpio_config_response;
void insert_mip_3dm_gpio_config_response(struct mip_serializer* serializer, const mip_3dm_gpio_config_response* self);
void extract_mip_3dm_gpio_config_response(struct mip_serializer* serializer, mip_3dm_gpio_config_response* self);

mip_cmd_result mip_3dm_write_gpio_config(struct mip_interface* device, uint8_t pin, mip_3dm_gpio_config_command_feature feature, mip_3dm_gpio_config_command_behavior behavior, mip_3dm_gpio_config_command_pin_mode pin_mode);
mip_cmd_result mip_3dm_read_gpio_config(struct mip_interface* device, uint8_t pin, mip_3dm_gpio_config_command_feature* feature_out, mip_3dm_gpio_config_command_behavior* behavior_out, mip_3dm_gpio_config_command_pin_mode* pin_mode_out);
mip_cmd_result mip_3dm_save_gpio_config(struct mip_interface* device, uint8_t pin);
mip_cmd_result mip_3dm_load_gpio_config(struct mip_interface* device, uint8_t pin);
mip_cmd_result mip_3dm_default_gpio_config(struct mip_interface* device, uint8_t pin);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_3dm_gpio_state  (0x0C,0x42) Gpio State [C]
/// Allows the state of the pin to be read or controlled.
/// 
/// This command serves two purposes: 1) To allow reading the state of a pin via command,
/// rather than polling a data quantity, and 2) to provide a way to set the output state
/// without also having to specify the operating mode.
/// 
/// The state read back from the pin is the physical state of the pin, rather than a
/// configuration value. The state can be read regardless of its configuration as long as
/// the device supports GPIO input on that pin. If the pin is set to an output, the read
/// value would match the output value.
/// 
/// While the state of a pin can always be set, it will only have an observable effect if
/// the pin is set to output mode.
/// 
/// This command does not support saving, loading, or resetting the state. Instead, use the
/// GPIO Configuration command, which allows the initial state to be configured.
///
///@{

struct mip_3dm_gpio_state_command
{
    mip_function_selector function;
    uint8_t pin; ///< GPIO pin number counting from 1. Cannot be 0.
    bool state; ///< The pin state.
    
};
typedef struct mip_3dm_gpio_state_command mip_3dm_gpio_state_command;
void insert_mip_3dm_gpio_state_command(struct mip_serializer* serializer, const mip_3dm_gpio_state_command* self);
void extract_mip_3dm_gpio_state_command(struct mip_serializer* serializer, mip_3dm_gpio_state_command* self);

struct mip_3dm_gpio_state_response
{
    uint8_t pin; ///< GPIO pin number counting from 1. Cannot be 0.
    bool state; ///< The pin state.
    
};
typedef struct mip_3dm_gpio_state_response mip_3dm_gpio_state_response;
void insert_mip_3dm_gpio_state_response(struct mip_serializer* serializer, const mip_3dm_gpio_state_response* self);
void extract_mip_3dm_gpio_state_response(struct mip_serializer* serializer, mip_3dm_gpio_state_response* self);

mip_cmd_result mip_3dm_write_gpio_state(struct mip_interface* device, uint8_t pin, bool state);
mip_cmd_result mip_3dm_read_gpio_state(struct mip_interface* device, uint8_t pin, bool* state_out);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_3dm_odometer  (0x0C,0x43) Odometer [C]
/// Configures the hardware odometer interface.
/// 
///
///@{

typedef uint8_t mip_3dm_odometer_command_mode;
static const mip_3dm_odometer_command_mode MIP_3DM_ODOMETER_COMMAND_MODE_DISABLED   = 0; ///<  Encoder is disabled.
static const mip_3dm_odometer_command_mode MIP_3DM_ODOMETER_COMMAND_MODE_QUADRATURE = 2; ///<  Quadrature encoder mode.

struct mip_3dm_odometer_command
{
    mip_function_selector function;
    mip_3dm_odometer_command_mode mode; ///< Mode setting.
    float scaling; ///< Encoder pulses per meter of distance traveled [pulses/m]. Distance traveled is computed using the formula d = p / N * 2R * pi, where d is distance, p is the number of pulses received, N is the encoder resolution, and R is the wheel radius. By simplifying all of the parameters into one, the formula d = p / S is obtained, where s is the odometer scaling factor passed to this command. S is equivalent to N / (2R * pi) and has units of pulses / meter. N is in units of "A" pulses per revolution and R is in meters. Make this value negative if the odometer is mounted so that it rotates backwards.
    float uncertainty; ///< Uncertainty in encoder counts to distance translation (1-sigma value) [m/m].
    
};
typedef struct mip_3dm_odometer_command mip_3dm_odometer_command;
void insert_mip_3dm_odometer_command(struct mip_serializer* serializer, const mip_3dm_odometer_command* self);
void extract_mip_3dm_odometer_command(struct mip_serializer* serializer, mip_3dm_odometer_command* self);

void insert_mip_3dm_odometer_command_mode(struct mip_serializer* serializer, const mip_3dm_odometer_command_mode self);
void extract_mip_3dm_odometer_command_mode(struct mip_serializer* serializer, mip_3dm_odometer_command_mode* self);

struct mip_3dm_odometer_response
{
    mip_3dm_odometer_command_mode mode; ///< Mode setting.
    float scaling; ///< Encoder pulses per meter of distance traveled [pulses/m]. Distance traveled is computed using the formula d = p / N * 2R * pi, where d is distance, p is the number of pulses received, N is the encoder resolution, and R is the wheel radius. By simplifying all of the parameters into one, the formula d = p / S is obtained, where s is the odometer scaling factor passed to this command. S is equivalent to N / (2R * pi) and has units of pulses / meter. N is in units of "A" pulses per revolution and R is in meters. Make this value negative if the odometer is mounted so that it rotates backwards.
    float uncertainty; ///< Uncertainty in encoder counts to distance translation (1-sigma value) [m/m].
    
};
typedef struct mip_3dm_odometer_response mip_3dm_odometer_response;
void insert_mip_3dm_odometer_response(struct mip_serializer* serializer, const mip_3dm_odometer_response* self);
void extract_mip_3dm_odometer_response(struct mip_serializer* serializer, mip_3dm_odometer_response* self);

mip_cmd_result mip_3dm_write_odometer(struct mip_interface* device, mip_3dm_odometer_command_mode mode, float scaling, float uncertainty);
mip_cmd_result mip_3dm_read_odometer(struct mip_interface* device, mip_3dm_odometer_command_mode* mode_out, float* scaling_out, float* uncertainty_out);
mip_cmd_result mip_3dm_save_odometer(struct mip_interface* device);
mip_cmd_result mip_3dm_load_odometer(struct mip_interface* device);
mip_cmd_result mip_3dm_default_odometer(struct mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_3dm_get_event_support  (0x0C,0x2A) Get Event Support [C]
/// Lists the available trigger or action types.
/// 
/// There are a limited number of trigger and action slots available
/// in the device. Up to M triggers and N actions can be configured at once
/// in slots 1..M and 1..N respectively. M and N are identified by the
/// max_instances field in the response with the appropriate query selector.
/// 
/// Each slot can be configured as one of a variety of different types of
/// triggers or actions. The supported types are enumerated in the response
/// to this command. Additionally, there is a limit on the number of a given
/// type. In other words, while the device may support M triggers in total,
/// only a few of them maybe usable as a given type. This limit helps optimize
/// device resources. The limit is identified in the count field.
/// 
/// All of the information in this command is available in the user manual.
/// This command provides a programmatic method for obtaining the information.
/// 
///
///@{

typedef uint8_t mip_3dm_get_event_support_command_query;
static const mip_3dm_get_event_support_command_query MIP_3DM_GET_EVENT_SUPPORT_COMMAND_QUERY_TRIGGER_TYPES = 1; ///<  Query the supported trigger types and max count for each.
static const mip_3dm_get_event_support_command_query MIP_3DM_GET_EVENT_SUPPORT_COMMAND_QUERY_ACTION_TYPES  = 2; ///<  Query the supported action types and max count for each.

struct mip_3dm_get_event_support_command_info
{
    uint8_t type; ///< Trigger or action type, as defined in the respective setup command.
    uint8_t count; ///< This is the maximum number of instances supported for this type.
    
};
typedef struct mip_3dm_get_event_support_command_info mip_3dm_get_event_support_command_info;
struct mip_3dm_get_event_support_command
{
    mip_3dm_get_event_support_command_query query; ///< What type of information to retrieve.
    
};
typedef struct mip_3dm_get_event_support_command mip_3dm_get_event_support_command;
void insert_mip_3dm_get_event_support_command(struct mip_serializer* serializer, const mip_3dm_get_event_support_command* self);
void extract_mip_3dm_get_event_support_command(struct mip_serializer* serializer, mip_3dm_get_event_support_command* self);

void insert_mip_3dm_get_event_support_command_query(struct mip_serializer* serializer, const mip_3dm_get_event_support_command_query self);
void extract_mip_3dm_get_event_support_command_query(struct mip_serializer* serializer, mip_3dm_get_event_support_command_query* self);

void insert_mip_3dm_get_event_support_command_info(struct mip_serializer* serializer, const mip_3dm_get_event_support_command_info* self);
void extract_mip_3dm_get_event_support_command_info(struct mip_serializer* serializer, mip_3dm_get_event_support_command_info* self);

struct mip_3dm_get_event_support_response
{
    mip_3dm_get_event_support_command_query query; ///< Query type specified in the command.
    uint8_t max_instances; ///< Number of slots available. The 'instance' number for the configuration or control commands must be between 1 and this value.
    uint8_t num_entries; ///< Number of supported types.
    mip_3dm_get_event_support_command_info entries[126]; ///< List of supported types.
    
};
typedef struct mip_3dm_get_event_support_response mip_3dm_get_event_support_response;
void insert_mip_3dm_get_event_support_response(struct mip_serializer* serializer, const mip_3dm_get_event_support_response* self);
void extract_mip_3dm_get_event_support_response(struct mip_serializer* serializer, mip_3dm_get_event_support_response* self);

mip_cmd_result mip_3dm_get_event_support(struct mip_interface* device, mip_3dm_get_event_support_command_query query, uint8_t* max_instances_out, uint8_t* num_entries_out, uint8_t num_entries_out_max, mip_3dm_get_event_support_command_info* entries_out);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_3dm_event_control  (0x0C,0x2B) Event Control [C]
/// Enables or disables event triggers.
/// 
/// Triggers can be disabled, enabled, and tested. While disabled, a trigger will
/// not evaluate its logic and effective behave like no trigger is configured.
/// A disabled trigger will not activate any actions. Triggers are disabled by default.
/// 
/// Use this command to enable (or disable) a trigger, or to place it into a test mode.
/// When in test mode, the trigger logic is disabled but the output is forced to
/// the active state, meaning that it will behave as if the trigger logic is satisfied
/// and any associated actions will execute.
///
///@{

typedef uint8_t mip_3dm_event_control_command_mode;
static const mip_3dm_event_control_command_mode MIP_3DM_EVENT_CONTROL_COMMAND_MODE_DISABLED   = 0; ///<  Trigger is disabled.
static const mip_3dm_event_control_command_mode MIP_3DM_EVENT_CONTROL_COMMAND_MODE_ENABLED    = 1; ///<  Trigger is enabled and will work normally.
static const mip_3dm_event_control_command_mode MIP_3DM_EVENT_CONTROL_COMMAND_MODE_TEST       = 2; ///<  Forces the trigger to the active state for testing purposes.
static const mip_3dm_event_control_command_mode MIP_3DM_EVENT_CONTROL_COMMAND_MODE_TEST_PULSE = 3; ///<  Trigger is forced to the active state for one event cycle only. After the test cycle, the mode reverts to the previous state (either enabled or disabled).

struct mip_3dm_event_control_command
{
    mip_function_selector function;
    uint8_t instance; ///< Trigger instance to affect. 0 can be used to apply the mode to all configured triggers, except when the function selector is READ.
    mip_3dm_event_control_command_mode mode; ///< How to change the trigger state. Except when instance is 0, the corresponding trigger must be configured, i.e. not have type 0.
    
};
typedef struct mip_3dm_event_control_command mip_3dm_event_control_command;
void insert_mip_3dm_event_control_command(struct mip_serializer* serializer, const mip_3dm_event_control_command* self);
void extract_mip_3dm_event_control_command(struct mip_serializer* serializer, mip_3dm_event_control_command* self);

void insert_mip_3dm_event_control_command_mode(struct mip_serializer* serializer, const mip_3dm_event_control_command_mode self);
void extract_mip_3dm_event_control_command_mode(struct mip_serializer* serializer, mip_3dm_event_control_command_mode* self);

struct mip_3dm_event_control_response
{
    uint8_t instance; ///< Trigger instance to affect. 0 can be used to apply the mode to all configured triggers, except when the function selector is READ.
    mip_3dm_event_control_command_mode mode; ///< How to change the trigger state. Except when instance is 0, the corresponding trigger must be configured, i.e. not have type 0.
    
};
typedef struct mip_3dm_event_control_response mip_3dm_event_control_response;
void insert_mip_3dm_event_control_response(struct mip_serializer* serializer, const mip_3dm_event_control_response* self);
void extract_mip_3dm_event_control_response(struct mip_serializer* serializer, mip_3dm_event_control_response* self);

mip_cmd_result mip_3dm_write_event_control(struct mip_interface* device, uint8_t instance, mip_3dm_event_control_command_mode mode);
mip_cmd_result mip_3dm_read_event_control(struct mip_interface* device, uint8_t instance, mip_3dm_event_control_command_mode* mode_out);
mip_cmd_result mip_3dm_save_event_control(struct mip_interface* device, uint8_t instance);
mip_cmd_result mip_3dm_load_event_control(struct mip_interface* device, uint8_t instance);
mip_cmd_result mip_3dm_default_event_control(struct mip_interface* device, uint8_t instance);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_3dm_get_event_trigger_status  (0x0C,0x2C) Get Event Trigger Status [C]
///
///@{

typedef uint8_t mip_3dm_get_event_trigger_status_command_status;
static const mip_3dm_get_event_trigger_status_command_status MIP_3DM_GET_EVENT_TRIGGER_STATUS_COMMAND_STATUS_NONE    = 0x00;
static const mip_3dm_get_event_trigger_status_command_status MIP_3DM_GET_EVENT_TRIGGER_STATUS_COMMAND_STATUS_ACTIVE  = 0x01; ///<  True if the trigger is currently active (either due to its logic or being in test mode).
static const mip_3dm_get_event_trigger_status_command_status MIP_3DM_GET_EVENT_TRIGGER_STATUS_COMMAND_STATUS_ENABLED = 0x02; ///<  True if the trigger is enabled.
static const mip_3dm_get_event_trigger_status_command_status MIP_3DM_GET_EVENT_TRIGGER_STATUS_COMMAND_STATUS_TEST    = 0x04; ///<  True if the trigger is in test mode.
static const mip_3dm_get_event_trigger_status_command_status MIP_3DM_GET_EVENT_TRIGGER_STATUS_COMMAND_STATUS_ALL     = 0x07;

struct mip_3dm_get_event_trigger_status_command_entry
{
    uint8_t type; ///< Configured trigger type.
    mip_3dm_get_event_trigger_status_command_status status; ///< Trigger status.
    
};
typedef struct mip_3dm_get_event_trigger_status_command_entry mip_3dm_get_event_trigger_status_command_entry;
struct mip_3dm_get_event_trigger_status_command
{
    uint8_t requested_count; ///< Number of entries requested. If 0, requests all trigger slots.
    uint8_t requested_instances[20]; ///< List of trigger instances to query.
    
};
typedef struct mip_3dm_get_event_trigger_status_command mip_3dm_get_event_trigger_status_command;
void insert_mip_3dm_get_event_trigger_status_command(struct mip_serializer* serializer, const mip_3dm_get_event_trigger_status_command* self);
void extract_mip_3dm_get_event_trigger_status_command(struct mip_serializer* serializer, mip_3dm_get_event_trigger_status_command* self);

void insert_mip_3dm_get_event_trigger_status_command_status(struct mip_serializer* serializer, const mip_3dm_get_event_trigger_status_command_status self);
void extract_mip_3dm_get_event_trigger_status_command_status(struct mip_serializer* serializer, mip_3dm_get_event_trigger_status_command_status* self);

void insert_mip_3dm_get_event_trigger_status_command_entry(struct mip_serializer* serializer, const mip_3dm_get_event_trigger_status_command_entry* self);
void extract_mip_3dm_get_event_trigger_status_command_entry(struct mip_serializer* serializer, mip_3dm_get_event_trigger_status_command_entry* self);

struct mip_3dm_get_event_trigger_status_response
{
    uint8_t count; ///< Number of entries requested. If requested_count was 0, this is the number of supported trigger slots.
    mip_3dm_get_event_trigger_status_command_entry triggers[20]; ///< A list of the configured triggers. Entries are in the order requested, or in increasing order if count was 0.
    
};
typedef struct mip_3dm_get_event_trigger_status_response mip_3dm_get_event_trigger_status_response;
void insert_mip_3dm_get_event_trigger_status_response(struct mip_serializer* serializer, const mip_3dm_get_event_trigger_status_response* self);
void extract_mip_3dm_get_event_trigger_status_response(struct mip_serializer* serializer, mip_3dm_get_event_trigger_status_response* self);

mip_cmd_result mip_3dm_get_event_trigger_status(struct mip_interface* device, uint8_t requested_count, const uint8_t* requested_instances, uint8_t* count_out, uint8_t count_out_max, mip_3dm_get_event_trigger_status_command_entry* triggers_out);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_3dm_get_event_action_status  (0x0C,0x2D) Get Event Action Status [C]
///
///@{

struct mip_3dm_get_event_action_status_command_entry
{
    uint8_t action_type; ///< Configured action type.
    uint8_t trigger_id; ///< Associated trigger instance.
    
};
typedef struct mip_3dm_get_event_action_status_command_entry mip_3dm_get_event_action_status_command_entry;
struct mip_3dm_get_event_action_status_command
{
    uint8_t requested_count; ///< Number of entries requested. If 0, requests all action slots.
    uint8_t requested_instances[20]; ///< List of action instances to query.
    
};
typedef struct mip_3dm_get_event_action_status_command mip_3dm_get_event_action_status_command;
void insert_mip_3dm_get_event_action_status_command(struct mip_serializer* serializer, const mip_3dm_get_event_action_status_command* self);
void extract_mip_3dm_get_event_action_status_command(struct mip_serializer* serializer, mip_3dm_get_event_action_status_command* self);

void insert_mip_3dm_get_event_action_status_command_entry(struct mip_serializer* serializer, const mip_3dm_get_event_action_status_command_entry* self);
void extract_mip_3dm_get_event_action_status_command_entry(struct mip_serializer* serializer, mip_3dm_get_event_action_status_command_entry* self);

struct mip_3dm_get_event_action_status_response
{
    uint8_t count; ///< Number of entries requested. If requested_count was 0, this is the number of supported action slots.
    mip_3dm_get_event_action_status_command_entry actions[20]; ///< A list of the configured actions. Entries are in the order requested, or in increasing order if count was 0.
    
};
typedef struct mip_3dm_get_event_action_status_response mip_3dm_get_event_action_status_response;
void insert_mip_3dm_get_event_action_status_response(struct mip_serializer* serializer, const mip_3dm_get_event_action_status_response* self);
void extract_mip_3dm_get_event_action_status_response(struct mip_serializer* serializer, mip_3dm_get_event_action_status_response* self);

mip_cmd_result mip_3dm_get_event_action_status(struct mip_interface* device, uint8_t requested_count, const uint8_t* requested_instances, uint8_t* count_out, uint8_t count_out_max, mip_3dm_get_event_action_status_command_entry* actions_out);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_3dm_event_trigger  (0x0C,0x2E) Event Trigger [C]
/// Configures various types of event triggers.
///
///@{

typedef uint8_t mip_3dm_event_trigger_command_gpio_params_mode;
static const mip_3dm_event_trigger_command_gpio_params_mode MIP_3DM_EVENT_TRIGGER_COMMAND_GPIO_PARAMS_MODE_DISABLED   = 0; ///<  The pin will have no effect and the trigger will never activate.
static const mip_3dm_event_trigger_command_gpio_params_mode MIP_3DM_EVENT_TRIGGER_COMMAND_GPIO_PARAMS_MODE_WHILE_HIGH = 1; ///<  The trigger will be active while the pin is high.
static const mip_3dm_event_trigger_command_gpio_params_mode MIP_3DM_EVENT_TRIGGER_COMMAND_GPIO_PARAMS_MODE_WHILE_LOW  = 2; ///<  The trigger will be active while the pin is low.
static const mip_3dm_event_trigger_command_gpio_params_mode MIP_3DM_EVENT_TRIGGER_COMMAND_GPIO_PARAMS_MODE_EDGE       = 4; ///<  Use if the pin is configured for timestamping via the 3DM Gpio Configuration command (0x0C41).

struct mip_3dm_event_trigger_command_gpio_params
{
    uint8_t pin; ///< GPIO pin number.
    mip_3dm_event_trigger_command_gpio_params_mode mode; ///< How the pin state affects the trigger.
    
};
typedef struct mip_3dm_event_trigger_command_gpio_params mip_3dm_event_trigger_command_gpio_params;
typedef uint8_t mip_3dm_event_trigger_command_threshold_params_type;
static const mip_3dm_event_trigger_command_threshold_params_type MIP_3DM_EVENT_TRIGGER_COMMAND_THRESHOLD_PARAMS_TYPE_WINDOW   = 1; ///<  Window comparison. Trigger is active if low_thres &lt;= value &lt;= high_thres. If the thresholds are reversed, the trigger is active when value &lt; high_thres or value &gt; low_thres.
static const mip_3dm_event_trigger_command_threshold_params_type MIP_3DM_EVENT_TRIGGER_COMMAND_THRESHOLD_PARAMS_TYPE_INTERVAL = 2; ///<  Trigger at evenly-spaced intervals. Normally used with time fields to trigger periodically. Trigger is active when (value % interval) &lt;= int_thres. If the thresholds are reversed (high_thres &lt; low_thres) then the trigger is active when (value % low_thres) &gt; high_thres.

struct mip_3dm_event_trigger_command_threshold_params
{
    uint8_t desc_set; ///< Descriptor set of target data quantity.
    uint8_t field_desc; ///< Field descriptor of target data quantity.
    uint8_t param_id; ///< 1-based index of the target parameter within the MIP field. E.g. for Scaled Accel (0x80,0x04) a value of 2 would represent the Y axis.
    mip_3dm_event_trigger_command_threshold_params_type type; ///< Determines the type of comparison.
    union
    {
        double low_thres;
        double int_thres;
    };
    union
    {
        double high_thres;
        double interval;
    };
    
};
typedef struct mip_3dm_event_trigger_command_threshold_params mip_3dm_event_trigger_command_threshold_params;
enum { MIP_3DM_EVENT_TRIGGER_COMMAND_COMBINATION_PARAMS_LOGIC_NEVER = 0x0000 };
enum { MIP_3DM_EVENT_TRIGGER_COMMAND_COMBINATION_PARAMS_LOGIC_ALWAYS = 0xFFFF };
enum { MIP_3DM_EVENT_TRIGGER_COMMAND_COMBINATION_PARAMS_LOGIC_NONE = 0x0001 };
enum { MIP_3DM_EVENT_TRIGGER_COMMAND_COMBINATION_PARAMS_LOGIC_OR = 0xFFFE };
enum { MIP_3DM_EVENT_TRIGGER_COMMAND_COMBINATION_PARAMS_LOGIC_NAND = 0x7FFF };
enum { MIP_3DM_EVENT_TRIGGER_COMMAND_COMBINATION_PARAMS_LOGIC_XOR_ONE = 0x0116 };
enum { MIP_3DM_EVENT_TRIGGER_COMMAND_COMBINATION_PARAMS_LOGIC_ONLY_A = 0x0002 };
enum { MIP_3DM_EVENT_TRIGGER_COMMAND_COMBINATION_PARAMS_LOGIC_ONLY_B = 0x0004 };
enum { MIP_3DM_EVENT_TRIGGER_COMMAND_COMBINATION_PARAMS_LOGIC_ONLY_C = 0x0010 };
enum { MIP_3DM_EVENT_TRIGGER_COMMAND_COMBINATION_PARAMS_LOGIC_ONLY_D = 0x0100 };
enum { MIP_3DM_EVENT_TRIGGER_COMMAND_COMBINATION_PARAMS_LOGIC_AND_AB = 0x8888 };
enum { MIP_3DM_EVENT_TRIGGER_COMMAND_COMBINATION_PARAMS_LOGIC_AB_OR_C = 0xF8F8 };
enum { MIP_3DM_EVENT_TRIGGER_COMMAND_COMBINATION_PARAMS_LOGIC_AND = 0x8000 };
struct mip_3dm_event_trigger_command_combination_params
{
    uint16_t logic_table; ///< The last column of a truth table describing the output given the state of each input.
    uint8_t input_triggers[4]; ///< List of trigger IDs for inputs. Use 0 for unused inputs.
    
};
typedef struct mip_3dm_event_trigger_command_combination_params mip_3dm_event_trigger_command_combination_params;
typedef uint8_t mip_3dm_event_trigger_command_type;
static const mip_3dm_event_trigger_command_type MIP_3DM_EVENT_TRIGGER_COMMAND_TYPE_NONE        = 0; ///<  No trigger selected. The state will always be inactive.
static const mip_3dm_event_trigger_command_type MIP_3DM_EVENT_TRIGGER_COMMAND_TYPE_GPIO        = 1; ///<  Trigger based on the state of a GPIO pin. See GpioParams.
static const mip_3dm_event_trigger_command_type MIP_3DM_EVENT_TRIGGER_COMMAND_TYPE_THRESHOLD   = 2; ///<  Compare a data quantity against a high and low threshold. See ThresholdParams.
static const mip_3dm_event_trigger_command_type MIP_3DM_EVENT_TRIGGER_COMMAND_TYPE_COMBINATION = 3; ///<  Logical combination of two or more triggers. See CombinationParams.

union mip_3dm_event_trigger_command_parameters
{
    mip_3dm_event_trigger_command_gpio_params gpio;
    mip_3dm_event_trigger_command_threshold_params threshold;
    mip_3dm_event_trigger_command_combination_params combination;
};
typedef union mip_3dm_event_trigger_command_parameters mip_3dm_event_trigger_command_parameters;

struct mip_3dm_event_trigger_command
{
    mip_function_selector function;
    uint8_t instance; ///< Trigger number. When function is SAVE, LOAD, or DEFAULT, this can be 0 to apply to all instances.
    mip_3dm_event_trigger_command_type type; ///< Type of trigger to configure.
    mip_3dm_event_trigger_command_parameters parameters;
    
};
typedef struct mip_3dm_event_trigger_command mip_3dm_event_trigger_command;
void insert_mip_3dm_event_trigger_command(struct mip_serializer* serializer, const mip_3dm_event_trigger_command* self);
void extract_mip_3dm_event_trigger_command(struct mip_serializer* serializer, mip_3dm_event_trigger_command* self);

void insert_mip_3dm_event_trigger_command_gpio_params(struct mip_serializer* serializer, const mip_3dm_event_trigger_command_gpio_params* self);
void extract_mip_3dm_event_trigger_command_gpio_params(struct mip_serializer* serializer, mip_3dm_event_trigger_command_gpio_params* self);

void insert_mip_3dm_event_trigger_command_gpio_params_mode(struct mip_serializer* serializer, const mip_3dm_event_trigger_command_gpio_params_mode self);
void extract_mip_3dm_event_trigger_command_gpio_params_mode(struct mip_serializer* serializer, mip_3dm_event_trigger_command_gpio_params_mode* self);

void insert_mip_3dm_event_trigger_command_threshold_params(struct mip_serializer* serializer, const mip_3dm_event_trigger_command_threshold_params* self);
void extract_mip_3dm_event_trigger_command_threshold_params(struct mip_serializer* serializer, mip_3dm_event_trigger_command_threshold_params* self);

void insert_mip_3dm_event_trigger_command_threshold_params_type(struct mip_serializer* serializer, const mip_3dm_event_trigger_command_threshold_params_type self);
void extract_mip_3dm_event_trigger_command_threshold_params_type(struct mip_serializer* serializer, mip_3dm_event_trigger_command_threshold_params_type* self);

void insert_mip_3dm_event_trigger_command_combination_params(struct mip_serializer* serializer, const mip_3dm_event_trigger_command_combination_params* self);
void extract_mip_3dm_event_trigger_command_combination_params(struct mip_serializer* serializer, mip_3dm_event_trigger_command_combination_params* self);

void insert_mip_3dm_event_trigger_command_type(struct mip_serializer* serializer, const mip_3dm_event_trigger_command_type self);
void extract_mip_3dm_event_trigger_command_type(struct mip_serializer* serializer, mip_3dm_event_trigger_command_type* self);

struct mip_3dm_event_trigger_response
{
    uint8_t instance; ///< Trigger number. When function is SAVE, LOAD, or DEFAULT, this can be 0 to apply to all instances.
    mip_3dm_event_trigger_command_type type; ///< Type of trigger to configure.
    mip_3dm_event_trigger_command_parameters parameters;
    
};
typedef struct mip_3dm_event_trigger_response mip_3dm_event_trigger_response;
void insert_mip_3dm_event_trigger_response(struct mip_serializer* serializer, const mip_3dm_event_trigger_response* self);
void extract_mip_3dm_event_trigger_response(struct mip_serializer* serializer, mip_3dm_event_trigger_response* self);

mip_cmd_result mip_3dm_write_event_trigger(struct mip_interface* device, uint8_t instance, mip_3dm_event_trigger_command_type type, const mip_3dm_event_trigger_command_parameters* parameters);
mip_cmd_result mip_3dm_read_event_trigger(struct mip_interface* device, uint8_t instance, mip_3dm_event_trigger_command_type* type_out, mip_3dm_event_trigger_command_parameters* parameters_out);
mip_cmd_result mip_3dm_save_event_trigger(struct mip_interface* device, uint8_t instance);
mip_cmd_result mip_3dm_load_event_trigger(struct mip_interface* device, uint8_t instance);
mip_cmd_result mip_3dm_default_event_trigger(struct mip_interface* device, uint8_t instance);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_3dm_event_action  (0x0C,0x2F) Event Action [C]
/// Configures various types of event actions.
///
///@{

typedef uint8_t mip_3dm_event_action_command_gpio_params_mode;
static const mip_3dm_event_action_command_gpio_params_mode MIP_3DM_EVENT_ACTION_COMMAND_GPIO_PARAMS_MODE_DISABLED     = 0; ///<  Pin state will not be changed.
static const mip_3dm_event_action_command_gpio_params_mode MIP_3DM_EVENT_ACTION_COMMAND_GPIO_PARAMS_MODE_ACTIVE_HIGH  = 1; ///<  Pin will be set high when the trigger is active and low otherwise.
static const mip_3dm_event_action_command_gpio_params_mode MIP_3DM_EVENT_ACTION_COMMAND_GPIO_PARAMS_MODE_ACTIVE_LOW   = 2; ///<  Pin will be set low when the trigger is active and high otherwise.
static const mip_3dm_event_action_command_gpio_params_mode MIP_3DM_EVENT_ACTION_COMMAND_GPIO_PARAMS_MODE_ONESHOT_HIGH = 5; ///<  Pin will be set high each time the trigger activates. It will not be set low.
static const mip_3dm_event_action_command_gpio_params_mode MIP_3DM_EVENT_ACTION_COMMAND_GPIO_PARAMS_MODE_ONESHOT_LOW  = 6; ///<  Pin will be set low each time the trigger activates. It will not be set high.
static const mip_3dm_event_action_command_gpio_params_mode MIP_3DM_EVENT_ACTION_COMMAND_GPIO_PARAMS_MODE_TOGGLE       = 7; ///<  Pin will change to the opposite state each time the trigger activates.

struct mip_3dm_event_action_command_gpio_params
{
    uint8_t pin; ///< GPIO pin number.
    mip_3dm_event_action_command_gpio_params_mode mode; ///< Behavior of the pin.
    
};
typedef struct mip_3dm_event_action_command_gpio_params mip_3dm_event_action_command_gpio_params;
struct mip_3dm_event_action_command_message_params
{
    uint8_t desc_set; ///< MIP data descriptor set.
    uint16_t decimation; ///< Decimation from the base rate. If 0, a packet is emitted each time the trigger activates. Otherwise, packets will be streamed while the trigger is active. The internal decimation counter is reset if the trigger deactivates.
    uint8_t num_fields; ///< Number of mip fields in the packet. Limited to 12.
    uint8_t descriptors[20]; ///< List of field descriptors.
    
};
typedef struct mip_3dm_event_action_command_message_params mip_3dm_event_action_command_message_params;
typedef uint8_t mip_3dm_event_action_command_type;
static const mip_3dm_event_action_command_type MIP_3DM_EVENT_ACTION_COMMAND_TYPE_NONE    = 0; ///<  No action. Parameters should be empty.
static const mip_3dm_event_action_command_type MIP_3DM_EVENT_ACTION_COMMAND_TYPE_GPIO    = 1; ///<  Control the state of a GPIO pin. See GpioParameters.
static const mip_3dm_event_action_command_type MIP_3DM_EVENT_ACTION_COMMAND_TYPE_MESSAGE = 2; ///<  Output a data packet. See MessageParameters.

union mip_3dm_event_action_command_parameters
{
    mip_3dm_event_action_command_gpio_params gpio;
    mip_3dm_event_action_command_message_params message;
};
typedef union mip_3dm_event_action_command_parameters mip_3dm_event_action_command_parameters;

struct mip_3dm_event_action_command
{
    mip_function_selector function;
    uint8_t instance; ///< Action number. When function is SAVE, LOAD, or DEFAULT, this can be 0 to apply to all instances.
    uint8_t trigger; ///< Trigger ID number.
    mip_3dm_event_action_command_type type; ///< Type of action to configure.
    mip_3dm_event_action_command_parameters parameters;
    
};
typedef struct mip_3dm_event_action_command mip_3dm_event_action_command;
void insert_mip_3dm_event_action_command(struct mip_serializer* serializer, const mip_3dm_event_action_command* self);
void extract_mip_3dm_event_action_command(struct mip_serializer* serializer, mip_3dm_event_action_command* self);

void insert_mip_3dm_event_action_command_gpio_params(struct mip_serializer* serializer, const mip_3dm_event_action_command_gpio_params* self);
void extract_mip_3dm_event_action_command_gpio_params(struct mip_serializer* serializer, mip_3dm_event_action_command_gpio_params* self);

void insert_mip_3dm_event_action_command_gpio_params_mode(struct mip_serializer* serializer, const mip_3dm_event_action_command_gpio_params_mode self);
void extract_mip_3dm_event_action_command_gpio_params_mode(struct mip_serializer* serializer, mip_3dm_event_action_command_gpio_params_mode* self);

void insert_mip_3dm_event_action_command_message_params(struct mip_serializer* serializer, const mip_3dm_event_action_command_message_params* self);
void extract_mip_3dm_event_action_command_message_params(struct mip_serializer* serializer, mip_3dm_event_action_command_message_params* self);

void insert_mip_3dm_event_action_command_type(struct mip_serializer* serializer, const mip_3dm_event_action_command_type self);
void extract_mip_3dm_event_action_command_type(struct mip_serializer* serializer, mip_3dm_event_action_command_type* self);

struct mip_3dm_event_action_response
{
    uint8_t instance; ///< Action number. When function is SAVE, LOAD, or DEFAULT, this can be 0 to apply to all instances.
    uint8_t trigger; ///< Trigger ID number.
    mip_3dm_event_action_command_type type; ///< Type of action to configure.
    mip_3dm_event_action_command_parameters parameters;
    
};
typedef struct mip_3dm_event_action_response mip_3dm_event_action_response;
void insert_mip_3dm_event_action_response(struct mip_serializer* serializer, const mip_3dm_event_action_response* self);
void extract_mip_3dm_event_action_response(struct mip_serializer* serializer, mip_3dm_event_action_response* self);

mip_cmd_result mip_3dm_write_event_action(struct mip_interface* device, uint8_t instance, uint8_t trigger, mip_3dm_event_action_command_type type, const mip_3dm_event_action_command_parameters* parameters);
mip_cmd_result mip_3dm_read_event_action(struct mip_interface* device, uint8_t instance, uint8_t* trigger_out, mip_3dm_event_action_command_type* type_out, mip_3dm_event_action_command_parameters* parameters_out);
mip_cmd_result mip_3dm_save_event_action(struct mip_interface* device, uint8_t instance);
mip_cmd_result mip_3dm_load_event_action(struct mip_interface* device, uint8_t instance);
mip_cmd_result mip_3dm_default_event_action(struct mip_interface* device, uint8_t instance);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_3dm_accel_bias  (0x0C,0x37) Accel Bias [C]
/// Configures the user specified accelerometer bias
/// 
/// The user specified bias is subtracted from the calibrated accelerometer output.  Value is input in the sensor frame.
///
///@{

struct mip_3dm_accel_bias_command
{
    mip_function_selector function;
    mip_vector3f bias; ///< accelerometer bias in the sensor frame (x,y,z) [g]
    
};
typedef struct mip_3dm_accel_bias_command mip_3dm_accel_bias_command;
void insert_mip_3dm_accel_bias_command(struct mip_serializer* serializer, const mip_3dm_accel_bias_command* self);
void extract_mip_3dm_accel_bias_command(struct mip_serializer* serializer, mip_3dm_accel_bias_command* self);

struct mip_3dm_accel_bias_response
{
    mip_vector3f bias; ///< accelerometer bias in the sensor frame (x,y,z) [g]
    
};
typedef struct mip_3dm_accel_bias_response mip_3dm_accel_bias_response;
void insert_mip_3dm_accel_bias_response(struct mip_serializer* serializer, const mip_3dm_accel_bias_response* self);
void extract_mip_3dm_accel_bias_response(struct mip_serializer* serializer, mip_3dm_accel_bias_response* self);

mip_cmd_result mip_3dm_write_accel_bias(struct mip_interface* device, const float* bias);
mip_cmd_result mip_3dm_read_accel_bias(struct mip_interface* device, float* bias_out);
mip_cmd_result mip_3dm_save_accel_bias(struct mip_interface* device);
mip_cmd_result mip_3dm_load_accel_bias(struct mip_interface* device);
mip_cmd_result mip_3dm_default_accel_bias(struct mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_3dm_gyro_bias  (0x0C,0x38) Gyro Bias [C]
/// Configures the user specified gyroscope bias
/// 
/// The user specified bias is subtracted from the calibrated angular rate output.  Value is input in the sensor frame.
///
///@{

struct mip_3dm_gyro_bias_command
{
    mip_function_selector function;
    mip_vector3f bias; ///< gyro bias in the sensor frame (x,y,z) [radians/second]
    
};
typedef struct mip_3dm_gyro_bias_command mip_3dm_gyro_bias_command;
void insert_mip_3dm_gyro_bias_command(struct mip_serializer* serializer, const mip_3dm_gyro_bias_command* self);
void extract_mip_3dm_gyro_bias_command(struct mip_serializer* serializer, mip_3dm_gyro_bias_command* self);

struct mip_3dm_gyro_bias_response
{
    mip_vector3f bias; ///< gyro bias in the sensor frame (x,y,z) [radians/second]
    
};
typedef struct mip_3dm_gyro_bias_response mip_3dm_gyro_bias_response;
void insert_mip_3dm_gyro_bias_response(struct mip_serializer* serializer, const mip_3dm_gyro_bias_response* self);
void extract_mip_3dm_gyro_bias_response(struct mip_serializer* serializer, mip_3dm_gyro_bias_response* self);

mip_cmd_result mip_3dm_write_gyro_bias(struct mip_interface* device, const float* bias);
mip_cmd_result mip_3dm_read_gyro_bias(struct mip_interface* device, float* bias_out);
mip_cmd_result mip_3dm_save_gyro_bias(struct mip_interface* device);
mip_cmd_result mip_3dm_load_gyro_bias(struct mip_interface* device);
mip_cmd_result mip_3dm_default_gyro_bias(struct mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_3dm_capture_gyro_bias  (0x0C,0x39) Capture Gyro Bias [C]
/// Samples gyro for a specified time range and writes the averaged result to the Gyro Bias vector in RAM
/// 
/// The device will average the gyro output for the duration of "averaging_time_ms." To store the resulting vector
/// in non-volatile memory, use the Set Gyro Bias command.
/// IMPORTANT: The device must be stationary and experiencing minimum vibration for the duration of "averaging_time_ms"
/// Averaging Time range: 1000 to 30,000
///
///@{

struct mip_3dm_capture_gyro_bias_command
{
    uint16_t averaging_time_ms; ///< Averaging time [milliseconds]
    
};
typedef struct mip_3dm_capture_gyro_bias_command mip_3dm_capture_gyro_bias_command;
void insert_mip_3dm_capture_gyro_bias_command(struct mip_serializer* serializer, const mip_3dm_capture_gyro_bias_command* self);
void extract_mip_3dm_capture_gyro_bias_command(struct mip_serializer* serializer, mip_3dm_capture_gyro_bias_command* self);

struct mip_3dm_capture_gyro_bias_response
{
    mip_vector3f bias; ///< gyro bias in the sensor frame (x,y,z) [radians/second]
    
};
typedef struct mip_3dm_capture_gyro_bias_response mip_3dm_capture_gyro_bias_response;
void insert_mip_3dm_capture_gyro_bias_response(struct mip_serializer* serializer, const mip_3dm_capture_gyro_bias_response* self);
void extract_mip_3dm_capture_gyro_bias_response(struct mip_serializer* serializer, mip_3dm_capture_gyro_bias_response* self);

mip_cmd_result mip_3dm_capture_gyro_bias(struct mip_interface* device, uint16_t averaging_time_ms, float* bias_out);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_3dm_mag_hard_iron_offset  (0x0C,0x3A) Mag Hard Iron Offset [C]
/// Configure the user specified magnetometer hard iron offset vector
/// 
/// The values for this offset are determined empirically by external software algorithms
/// based on calibration data taken after the device is installed in its application. These values
/// can be obtained and set by using the LORD "MIP Iron Calibration" application.
/// Alternatively, on some systems, the auto-mag calibration feature may be used to capture these values in-run.
/// The offset is applied to the scaled magnetometer vector prior to output.
///
///@{

struct mip_3dm_mag_hard_iron_offset_command
{
    mip_function_selector function;
    mip_vector3f offset; ///< hard iron offset in the sensor frame (x,y,z) [Gauss]
    
};
typedef struct mip_3dm_mag_hard_iron_offset_command mip_3dm_mag_hard_iron_offset_command;
void insert_mip_3dm_mag_hard_iron_offset_command(struct mip_serializer* serializer, const mip_3dm_mag_hard_iron_offset_command* self);
void extract_mip_3dm_mag_hard_iron_offset_command(struct mip_serializer* serializer, mip_3dm_mag_hard_iron_offset_command* self);

struct mip_3dm_mag_hard_iron_offset_response
{
    mip_vector3f offset; ///< hard iron offset in the sensor frame (x,y,z) [Gauss]
    
};
typedef struct mip_3dm_mag_hard_iron_offset_response mip_3dm_mag_hard_iron_offset_response;
void insert_mip_3dm_mag_hard_iron_offset_response(struct mip_serializer* serializer, const mip_3dm_mag_hard_iron_offset_response* self);
void extract_mip_3dm_mag_hard_iron_offset_response(struct mip_serializer* serializer, mip_3dm_mag_hard_iron_offset_response* self);

mip_cmd_result mip_3dm_write_mag_hard_iron_offset(struct mip_interface* device, const float* offset);
mip_cmd_result mip_3dm_read_mag_hard_iron_offset(struct mip_interface* device, float* offset_out);
mip_cmd_result mip_3dm_save_mag_hard_iron_offset(struct mip_interface* device);
mip_cmd_result mip_3dm_load_mag_hard_iron_offset(struct mip_interface* device);
mip_cmd_result mip_3dm_default_mag_hard_iron_offset(struct mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_3dm_mag_soft_iron_matrix  (0x0C,0x3B) Mag Soft Iron Matrix [C]
/// Configure the user specified magnetometer soft iron offset matrix
/// 
/// The values for this matrix are determined empirically by external software algorithms
/// based on calibration data taken after the device is installed in its application. These values
/// can be obtained and set by using the LORD "MIP Iron Calibration" application.
/// Alternatively, on some systems, the auto-mag calibration feature may be used to capture these values in-run.
/// The matrix is applied to the scaled magnetometer vector prior to output.
/// 
/// The matrix is in row major order:
/// EQSTART M = \begin{bmatrix} 0 &amp; 1 &amp; 2 \\ 3 &amp; 4 &amp; 5 \\ 6 &amp; 7 &amp; 8 \end{bmatrix} EQEND
/// 
///
///@{

struct mip_3dm_mag_soft_iron_matrix_command
{
    mip_function_selector function;
    mip_matrix3f offset; ///< soft iron matrix [dimensionless]
    
};
typedef struct mip_3dm_mag_soft_iron_matrix_command mip_3dm_mag_soft_iron_matrix_command;
void insert_mip_3dm_mag_soft_iron_matrix_command(struct mip_serializer* serializer, const mip_3dm_mag_soft_iron_matrix_command* self);
void extract_mip_3dm_mag_soft_iron_matrix_command(struct mip_serializer* serializer, mip_3dm_mag_soft_iron_matrix_command* self);

struct mip_3dm_mag_soft_iron_matrix_response
{
    mip_matrix3f offset; ///< soft iron matrix [dimensionless]
    
};
typedef struct mip_3dm_mag_soft_iron_matrix_response mip_3dm_mag_soft_iron_matrix_response;
void insert_mip_3dm_mag_soft_iron_matrix_response(struct mip_serializer* serializer, const mip_3dm_mag_soft_iron_matrix_response* self);
void extract_mip_3dm_mag_soft_iron_matrix_response(struct mip_serializer* serializer, mip_3dm_mag_soft_iron_matrix_response* self);

mip_cmd_result mip_3dm_write_mag_soft_iron_matrix(struct mip_interface* device, const float* offset);
mip_cmd_result mip_3dm_read_mag_soft_iron_matrix(struct mip_interface* device, float* offset_out);
mip_cmd_result mip_3dm_save_mag_soft_iron_matrix(struct mip_interface* device);
mip_cmd_result mip_3dm_load_mag_soft_iron_matrix(struct mip_interface* device);
mip_cmd_result mip_3dm_default_mag_soft_iron_matrix(struct mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_3dm_coning_sculling_enable  (0x0C,0x3E) Coning Sculling Enable [C]
/// Controls the Coning and Sculling Compenstation setting.
///
///@{

struct mip_3dm_coning_sculling_enable_command
{
    mip_function_selector function;
    bool enable; ///< If true, coning and sculling compensation is enabled.
    
};
typedef struct mip_3dm_coning_sculling_enable_command mip_3dm_coning_sculling_enable_command;
void insert_mip_3dm_coning_sculling_enable_command(struct mip_serializer* serializer, const mip_3dm_coning_sculling_enable_command* self);
void extract_mip_3dm_coning_sculling_enable_command(struct mip_serializer* serializer, mip_3dm_coning_sculling_enable_command* self);

struct mip_3dm_coning_sculling_enable_response
{
    bool enable; ///< If true, coning and sculling compensation is enabled.
    
};
typedef struct mip_3dm_coning_sculling_enable_response mip_3dm_coning_sculling_enable_response;
void insert_mip_3dm_coning_sculling_enable_response(struct mip_serializer* serializer, const mip_3dm_coning_sculling_enable_response* self);
void extract_mip_3dm_coning_sculling_enable_response(struct mip_serializer* serializer, mip_3dm_coning_sculling_enable_response* self);

mip_cmd_result mip_3dm_write_coning_sculling_enable(struct mip_interface* device, bool enable);
mip_cmd_result mip_3dm_read_coning_sculling_enable(struct mip_interface* device, bool* enable_out);
mip_cmd_result mip_3dm_save_coning_sculling_enable(struct mip_interface* device);
mip_cmd_result mip_3dm_load_coning_sculling_enable(struct mip_interface* device);
mip_cmd_result mip_3dm_default_coning_sculling_enable(struct mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_3dm_sensor_2_vehicle_transform_euler  (0x0C,0x31) Sensor 2 Vehicle Transform Euler [C]
/// Sets the sensor-to-vehicle frame transformation using Yaw, Pitch, Roll Euler angles.
/// These are the Yaw, Pitch, and Roll mounting angles of the sensor with respect to vehicle frame of reference,
/// and describe the transformation of vectors from the sensor body frame to the vehicle frame.<br/>
/// Note: This is the transformation, the inverse of the rotation defined in our legacy products.<br/>
/// The transformation may be stored in the device as a matrix or quaternion.  When Euler angles are read back from the device, they may not
/// be exactly equal to the Euler angles used to set the transformation, but they are functionally equivalent, such that they result in the same transformation.<br/>
/// <br/><br/>
/// This transformation to the vehicle frame will be applied to the following output quantities:<br/><br/>
/// IMU:<br/>
/// Scaled Acceleration<br/>
/// Scaled Gyro<br/>
/// Scaled Magnetometer<br/>
/// Delta Theta<br/>
/// Delta Velocity<br/>
/// Complementary Filter Orientation<br/>
/// <br/><br/>
/// Estimation Filter:<br/>
/// Estimated Orientation, Quaternion<br/>
/// Estimated Orientation, Matrix<br/>
/// Estimated Orientation, Euler Angles<br/>
/// Estimated Linear Acceleration<br/>
/// Estimated Angular Rate<br/>
/// Estimated Gravity Vector<br/>
/// <br/>
/// Changing this setting will force all low-pass filters, the complementary filter, and the estimation filter to reset.
///
///@{

struct mip_3dm_sensor_2_vehicle_transform_euler_command
{
    mip_function_selector function;
    float roll; ///< [radians]
    float pitch; ///< [radians]
    float yaw; ///< [radians]
    
};
typedef struct mip_3dm_sensor_2_vehicle_transform_euler_command mip_3dm_sensor_2_vehicle_transform_euler_command;
void insert_mip_3dm_sensor_2_vehicle_transform_euler_command(struct mip_serializer* serializer, const mip_3dm_sensor_2_vehicle_transform_euler_command* self);
void extract_mip_3dm_sensor_2_vehicle_transform_euler_command(struct mip_serializer* serializer, mip_3dm_sensor_2_vehicle_transform_euler_command* self);

struct mip_3dm_sensor_2_vehicle_transform_euler_response
{
    float roll; ///< [radians]
    float pitch; ///< [radians]
    float yaw; ///< [radians]
    
};
typedef struct mip_3dm_sensor_2_vehicle_transform_euler_response mip_3dm_sensor_2_vehicle_transform_euler_response;
void insert_mip_3dm_sensor_2_vehicle_transform_euler_response(struct mip_serializer* serializer, const mip_3dm_sensor_2_vehicle_transform_euler_response* self);
void extract_mip_3dm_sensor_2_vehicle_transform_euler_response(struct mip_serializer* serializer, mip_3dm_sensor_2_vehicle_transform_euler_response* self);

mip_cmd_result mip_3dm_write_sensor_2_vehicle_transform_euler(struct mip_interface* device, float roll, float pitch, float yaw);
mip_cmd_result mip_3dm_read_sensor_2_vehicle_transform_euler(struct mip_interface* device, float* roll_out, float* pitch_out, float* yaw_out);
mip_cmd_result mip_3dm_save_sensor_2_vehicle_transform_euler(struct mip_interface* device);
mip_cmd_result mip_3dm_load_sensor_2_vehicle_transform_euler(struct mip_interface* device);
mip_cmd_result mip_3dm_default_sensor_2_vehicle_transform_euler(struct mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_3dm_sensor_2_vehicle_transform_quaternion  (0x0C,0x32) Sensor 2 Vehicle Transform Quaternion [C]
/// Set the sensor to vehicle frame transformation using unit length quaternion.
/// 
/// Note: This is the transformation, the inverse of the rotation.
/// 
/// This quaternion describes the transformation of vectors from the sensor body frame to the vehicle frame of reference, and satisfies the following relationship:<br/>
/// 
/// EQSTART p^{veh} = q^{-1} p^{sen} q EQEND<br/>
/// 
/// Where:<br/>
/// EQSTART q = (q_w, q_x, q_y, q_z) EQEND is the quaternion describing the transformation. <br/>
/// EQSTART p^{sen} = (0, v^{sen}_x, v^{sen}_y, v^{sen}_z) EQEND and EQSTART v^{sen} EQEND is a 3-element vector expressed in the sensor body frame.<br/>
/// EQSTART p^{veh} = (0, v^{veh}_x, v^{veh}_y, v^{veh}_z) EQEND and EQSTART v^{veh} EQEND is a 3-element vector expressed in the vehicle frame.<br/>
/// 
/// The transformation may be stored in the device as a matrix or a quaternion.  When the quaternion is read back from the device, it may not
/// be exactly equal to the quaternion used to set the transformation, but it is functionally equivalent.<br/>
/// <br/><br/>
/// This transformation affects the following output quantities:<br/><br/>
/// IMU:<br/>
/// Scaled Acceleration<br/>
/// Scaled Gyro<br/>
/// Scaled Magnetometer<br/>
/// Delta Theta<br/>
/// Delta Velocity<br/>
/// <br/><br/>
/// Estimation Filter:<br/>
/// Estimated Orientation, Quaternion<br/>
/// Estimated Orientation, Matrix<br/>
/// Estimated Orientation, Euler Angles<br/>
/// Estimated Linear Acceleration<br/>
/// Estimated Angular Rate<br/>
/// Estimated Gravity Vector<br/>
/// <br/>
/// Changing this setting will force all low-pass filters, the complementary filter, and the estimation filter to reset.
///
///@{

struct mip_3dm_sensor_2_vehicle_transform_quaternion_command
{
    mip_function_selector function;
    mip_quatf q; ///< Unit length quaternion representing transform [w, i, j, k]
    
};
typedef struct mip_3dm_sensor_2_vehicle_transform_quaternion_command mip_3dm_sensor_2_vehicle_transform_quaternion_command;
void insert_mip_3dm_sensor_2_vehicle_transform_quaternion_command(struct mip_serializer* serializer, const mip_3dm_sensor_2_vehicle_transform_quaternion_command* self);
void extract_mip_3dm_sensor_2_vehicle_transform_quaternion_command(struct mip_serializer* serializer, mip_3dm_sensor_2_vehicle_transform_quaternion_command* self);

struct mip_3dm_sensor_2_vehicle_transform_quaternion_response
{
    mip_quatf q; ///< Unit length quaternion representing transform [w, i, j, k]
    
};
typedef struct mip_3dm_sensor_2_vehicle_transform_quaternion_response mip_3dm_sensor_2_vehicle_transform_quaternion_response;
void insert_mip_3dm_sensor_2_vehicle_transform_quaternion_response(struct mip_serializer* serializer, const mip_3dm_sensor_2_vehicle_transform_quaternion_response* self);
void extract_mip_3dm_sensor_2_vehicle_transform_quaternion_response(struct mip_serializer* serializer, mip_3dm_sensor_2_vehicle_transform_quaternion_response* self);

mip_cmd_result mip_3dm_write_sensor_2_vehicle_transform_quaternion(struct mip_interface* device, const float* q);
mip_cmd_result mip_3dm_read_sensor_2_vehicle_transform_quaternion(struct mip_interface* device, float* q_out);
mip_cmd_result mip_3dm_save_sensor_2_vehicle_transform_quaternion(struct mip_interface* device);
mip_cmd_result mip_3dm_load_sensor_2_vehicle_transform_quaternion(struct mip_interface* device);
mip_cmd_result mip_3dm_default_sensor_2_vehicle_transform_quaternion(struct mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_3dm_sensor_2_vehicle_transform_dcm  (0x0C,0x33) Sensor 2 Vehicle Transform Dcm [C]
/// Set the sensor to vehicle frame transformation using a using a 3 x 3 direction cosine matrix EQSTART M_{ned}^{veh} EQEND, stored in row-major order in a 9-element array.
/// 
/// These angles define the transformation of vectors from the sensor body frame to the fixed vehicle frame, according to:<br/>
/// EQSTART v^{veh} = M_{sen}^{veh} v^{sen} EQEND<br/>
/// 
/// Where:<br/>
/// 
/// EQSTART v^{sen} EQEND is a 3-element vector expressed in the sensor body frame. <br/>
/// EQSTART v^{veh} EQEND is the same 3-element vector expressed in the vehicle frame.  <br/>
/// <br/>
/// The matrix elements are stored is row-major order: EQSTART M_{sen}^{veh} = \begin{bmatrix} M_{11}, M_{12}, M_{13}, M_{21}, M_{22}, M_{23}, M_{31}, M_{32}, M_{33} \end{bmatrix} EQEND
/// The transformation may be stored in the device as a matrix or a quaternion. When EQSTART M_{sen}^{veh} EQEND is read back from the device, it may not
/// be exactly equal to array used to set the transformation, but it is functionally equivalent.<br/>
/// <br/><br/>
/// This transformation affects the following output quantities:<br/><br/>
/// IMU:<br/>
/// Scaled Acceleration<br/>
/// Scaled Gyro<br/>
/// Scaled Magnetometer<br/>
/// Delta Theta<br/>
/// Delta Velocity<br/>
/// <br/><br/>
/// Estimation Filter:<br/>
/// Estimated Orientation, Quaternion<br/>
/// Estimated Orientation, Matrix<br/>
/// Estimated Orientation, Euler Angles<br/>
/// Estimated Linear Acceleration<br/>
/// Estimated Angular Rate<br/>
/// Estimated Gravity Vector<br/>
/// <br/>
/// Changing this setting will force all low-pass filters, the complementary filter, and the estimation filter to reset.
///
///@{

struct mip_3dm_sensor_2_vehicle_transform_dcm_command
{
    mip_function_selector function;
    mip_matrix3f dcm; ///< 3 x 3 direction cosine matrix, stored in row-major order
    
};
typedef struct mip_3dm_sensor_2_vehicle_transform_dcm_command mip_3dm_sensor_2_vehicle_transform_dcm_command;
void insert_mip_3dm_sensor_2_vehicle_transform_dcm_command(struct mip_serializer* serializer, const mip_3dm_sensor_2_vehicle_transform_dcm_command* self);
void extract_mip_3dm_sensor_2_vehicle_transform_dcm_command(struct mip_serializer* serializer, mip_3dm_sensor_2_vehicle_transform_dcm_command* self);

struct mip_3dm_sensor_2_vehicle_transform_dcm_response
{
    mip_matrix3f dcm; ///< 3 x 3 direction cosine matrix, stored in row-major order
    
};
typedef struct mip_3dm_sensor_2_vehicle_transform_dcm_response mip_3dm_sensor_2_vehicle_transform_dcm_response;
void insert_mip_3dm_sensor_2_vehicle_transform_dcm_response(struct mip_serializer* serializer, const mip_3dm_sensor_2_vehicle_transform_dcm_response* self);
void extract_mip_3dm_sensor_2_vehicle_transform_dcm_response(struct mip_serializer* serializer, mip_3dm_sensor_2_vehicle_transform_dcm_response* self);

mip_cmd_result mip_3dm_write_sensor_2_vehicle_transform_dcm(struct mip_interface* device, const float* dcm);
mip_cmd_result mip_3dm_read_sensor_2_vehicle_transform_dcm(struct mip_interface* device, float* dcm_out);
mip_cmd_result mip_3dm_save_sensor_2_vehicle_transform_dcm(struct mip_interface* device);
mip_cmd_result mip_3dm_load_sensor_2_vehicle_transform_dcm(struct mip_interface* device);
mip_cmd_result mip_3dm_default_sensor_2_vehicle_transform_dcm(struct mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_3dm_complementary_filter  (0x0C,0x51) Complementary Filter [C]
/// Configure the settings for the complementary filter which produces the following (0x80) descriptor set values: attitude matrix (0x80,09), quaternion (0x80,0A), and  Euler angle (0x80,0C) outputs.
/// 
/// The filter can be configured to correct for pitch and roll using the accelerometer (with the assumption that linear acceleration is minimal),
/// and to correct for heading using the magnetometer (with the assumption that the local magnetic field is dominated by the Earth's own magnetic field).
/// Pitch/roll and heading corrections each have their own configurable time constants, with a valid range of 1-1000 seconds. The default time constant is 10 seconds.
///
///@{

struct mip_3dm_complementary_filter_command
{
    mip_function_selector function;
    bool pitch_roll_enable; ///< Enable Pitch/Roll corrections
    bool heading_enable; ///< Enable Heading corrections (only available on devices with magnetometer)
    float pitch_roll_time_constant; ///< Time constant associated with the pitch/roll corrections [s]
    float heading_time_constant; ///< Time constant associated with the heading corrections [s]
    
};
typedef struct mip_3dm_complementary_filter_command mip_3dm_complementary_filter_command;
void insert_mip_3dm_complementary_filter_command(struct mip_serializer* serializer, const mip_3dm_complementary_filter_command* self);
void extract_mip_3dm_complementary_filter_command(struct mip_serializer* serializer, mip_3dm_complementary_filter_command* self);

struct mip_3dm_complementary_filter_response
{
    bool pitch_roll_enable; ///< Enable Pitch/Roll corrections
    bool heading_enable; ///< Enable Heading corrections (only available on devices with magnetometer)
    float pitch_roll_time_constant; ///< Time constant associated with the pitch/roll corrections [s]
    float heading_time_constant; ///< Time constant associated with the heading corrections [s]
    
};
typedef struct mip_3dm_complementary_filter_response mip_3dm_complementary_filter_response;
void insert_mip_3dm_complementary_filter_response(struct mip_serializer* serializer, const mip_3dm_complementary_filter_response* self);
void extract_mip_3dm_complementary_filter_response(struct mip_serializer* serializer, mip_3dm_complementary_filter_response* self);

mip_cmd_result mip_3dm_write_complementary_filter(struct mip_interface* device, bool pitch_roll_enable, bool heading_enable, float pitch_roll_time_constant, float heading_time_constant);
mip_cmd_result mip_3dm_read_complementary_filter(struct mip_interface* device, bool* pitch_roll_enable_out, bool* heading_enable_out, float* pitch_roll_time_constant_out, float* heading_time_constant_out);
mip_cmd_result mip_3dm_save_complementary_filter(struct mip_interface* device);
mip_cmd_result mip_3dm_load_complementary_filter(struct mip_interface* device);
mip_cmd_result mip_3dm_default_complementary_filter(struct mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_3dm_sensor_range  (0x0C,0x52) Sensor Range [C]
/// Changes the IMU sensor gain.
/// 
/// This allows you to optimize the range to get the best accuracy and performance
/// while minimizing over-range events.
/// 
/// Use the 3DM Get Calibrated Sensor Ranges (0x0C,0x53) command to determine
/// the appropriate setting value for your application. Using values other than
/// those specified may result in a NACK or inaccurate measurement data.
///
///@{

struct mip_3dm_sensor_range_command
{
    mip_function_selector function;
    mip_sensor_range_type sensor; ///< Which type of sensor will get the new range value.
    uint8_t setting; ///< Use the 3DM Get Calibrated Sensor Ranges (0x0C,0x53) command to determine this value.
    
};
typedef struct mip_3dm_sensor_range_command mip_3dm_sensor_range_command;
void insert_mip_3dm_sensor_range_command(struct mip_serializer* serializer, const mip_3dm_sensor_range_command* self);
void extract_mip_3dm_sensor_range_command(struct mip_serializer* serializer, mip_3dm_sensor_range_command* self);

struct mip_3dm_sensor_range_response
{
    mip_sensor_range_type sensor; ///< Which type of sensor will get the new range value.
    uint8_t setting; ///< Use the 3DM Get Calibrated Sensor Ranges (0x0C,0x53) command to determine this value.
    
};
typedef struct mip_3dm_sensor_range_response mip_3dm_sensor_range_response;
void insert_mip_3dm_sensor_range_response(struct mip_serializer* serializer, const mip_3dm_sensor_range_response* self);
void extract_mip_3dm_sensor_range_response(struct mip_serializer* serializer, mip_3dm_sensor_range_response* self);

mip_cmd_result mip_3dm_write_sensor_range(struct mip_interface* device, mip_sensor_range_type sensor, uint8_t setting);
mip_cmd_result mip_3dm_read_sensor_range(struct mip_interface* device, mip_sensor_range_type sensor, uint8_t* setting_out);
mip_cmd_result mip_3dm_save_sensor_range(struct mip_interface* device, mip_sensor_range_type sensor);
mip_cmd_result mip_3dm_load_sensor_range(struct mip_interface* device, mip_sensor_range_type sensor);
mip_cmd_result mip_3dm_default_sensor_range(struct mip_interface* device, mip_sensor_range_type sensor);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_3dm_calibrated_sensor_ranges  (0x0C,0x53) Calibrated Sensor Ranges [C]
/// Returns the supported sensor ranges which may be used with the 3DM Sensor Range (0x0C,0x52) command.
/// 
/// The response includes an array of (u8, float) pairs which map each allowed setting
/// to the corresponding maximum range in physical units. See SensorRangeType for units.
///
///@{

struct mip_3dm_calibrated_sensor_ranges_command_entry
{
    uint8_t setting; ///< The value used in the 3DM Sensor Range command and response.
    float range; ///< The actual range value. Units depend on the sensor type.
    
};
typedef struct mip_3dm_calibrated_sensor_ranges_command_entry mip_3dm_calibrated_sensor_ranges_command_entry;
struct mip_3dm_calibrated_sensor_ranges_command
{
    mip_sensor_range_type sensor; ///< The sensor to query. Cannot be ALL.
    
};
typedef struct mip_3dm_calibrated_sensor_ranges_command mip_3dm_calibrated_sensor_ranges_command;
void insert_mip_3dm_calibrated_sensor_ranges_command(struct mip_serializer* serializer, const mip_3dm_calibrated_sensor_ranges_command* self);
void extract_mip_3dm_calibrated_sensor_ranges_command(struct mip_serializer* serializer, mip_3dm_calibrated_sensor_ranges_command* self);

void insert_mip_3dm_calibrated_sensor_ranges_command_entry(struct mip_serializer* serializer, const mip_3dm_calibrated_sensor_ranges_command_entry* self);
void extract_mip_3dm_calibrated_sensor_ranges_command_entry(struct mip_serializer* serializer, mip_3dm_calibrated_sensor_ranges_command_entry* self);

struct mip_3dm_calibrated_sensor_ranges_response
{
    mip_sensor_range_type sensor; ///< The sensor type from the command.
    uint8_t num_ranges; ///< Number of supported ranges.
    mip_3dm_calibrated_sensor_ranges_command_entry ranges[50]; ///< List of possible range settings.
    
};
typedef struct mip_3dm_calibrated_sensor_ranges_response mip_3dm_calibrated_sensor_ranges_response;
void insert_mip_3dm_calibrated_sensor_ranges_response(struct mip_serializer* serializer, const mip_3dm_calibrated_sensor_ranges_response* self);
void extract_mip_3dm_calibrated_sensor_ranges_response(struct mip_serializer* serializer, mip_3dm_calibrated_sensor_ranges_response* self);

mip_cmd_result mip_3dm_calibrated_sensor_ranges(struct mip_interface* device, mip_sensor_range_type sensor, uint8_t* num_ranges_out, uint8_t num_ranges_out_max, mip_3dm_calibrated_sensor_ranges_command_entry* ranges_out);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_3dm_lowpass_filter  (0x0C,0x54) Lowpass Filter [C]
/// This command controls the low-pass anti-aliasing filter supported data quantities.
/// 
/// See the device user manual for data quantities which support the anti-aliasing filter.
/// 
/// If set to automatic mode, the frequency will track half of the transmission rate
/// of the target descriptor according to the configured message format (0x0C,0x0F).
/// For example, if scaled accel (0x80,0x04) is set to stream at 100 Hz, the filter would
/// be set to 50 Hz. Changing the message format to 200 Hz would automatically adjust the
/// filter to 100 Hz.
/// 
/// For WRITE, SAVE, LOAD, and DEFAULT function selectors, the descriptor set and/or field descriptor
/// may be 0x00 to set, save, load, or reset the setting for all supported descriptors. The
/// field descriptor must be 0x00 if the descriptor set is 0x00.
/// 
///
///@{

struct mip_3dm_lowpass_filter_command
{
    mip_function_selector function;
    uint8_t desc_set; ///< Descriptor set of the quantity to be filtered.
    uint8_t field_desc; ///< Field descriptor of the quantity to be filtered.
    bool enable; ///< The filter will be enabled if this is true.
    bool manual; ///< If false, the frequency parameter is ignored and the filter will track to half of the configured message format frequency.
    float frequency; ///< Cutoff frequency in Hz. This will return the actual frequency when read out in automatic mode.
    
};
typedef struct mip_3dm_lowpass_filter_command mip_3dm_lowpass_filter_command;
void insert_mip_3dm_lowpass_filter_command(struct mip_serializer* serializer, const mip_3dm_lowpass_filter_command* self);
void extract_mip_3dm_lowpass_filter_command(struct mip_serializer* serializer, mip_3dm_lowpass_filter_command* self);

struct mip_3dm_lowpass_filter_response
{
    uint8_t desc_set; ///< Descriptor set of the quantity to be filtered.
    uint8_t field_desc; ///< Field descriptor of the quantity to be filtered.
    bool enable; ///< The filter will be enabled if this is true.
    bool manual; ///< If false, the frequency parameter is ignored and the filter will track to half of the configured message format frequency.
    float frequency; ///< Cutoff frequency in Hz. This will return the actual frequency when read out in automatic mode.
    
};
typedef struct mip_3dm_lowpass_filter_response mip_3dm_lowpass_filter_response;
void insert_mip_3dm_lowpass_filter_response(struct mip_serializer* serializer, const mip_3dm_lowpass_filter_response* self);
void extract_mip_3dm_lowpass_filter_response(struct mip_serializer* serializer, mip_3dm_lowpass_filter_response* self);

mip_cmd_result mip_3dm_write_lowpass_filter(struct mip_interface* device, uint8_t desc_set, uint8_t field_desc, bool enable, bool manual, float frequency);
mip_cmd_result mip_3dm_read_lowpass_filter(struct mip_interface* device, uint8_t desc_set, uint8_t field_desc, bool* enable_out, bool* manual_out, float* frequency_out);
mip_cmd_result mip_3dm_save_lowpass_filter(struct mip_interface* device, uint8_t desc_set, uint8_t field_desc);
mip_cmd_result mip_3dm_load_lowpass_filter(struct mip_interface* device, uint8_t desc_set, uint8_t field_desc);
mip_cmd_result mip_3dm_default_lowpass_filter(struct mip_interface* device, uint8_t desc_set, uint8_t field_desc);

///@}
///

///@}
///@}
///
////////////////////////////////////////////////////////////////////////////////
#ifdef __cplusplus
} // namespace C
} // namespace mip
} // extern "C"
#endif // __cplusplus


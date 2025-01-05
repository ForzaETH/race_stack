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
///@defgroup filter_commands_c  Filter Commands [C]
///
///@{

////////////////////////////////////////////////////////////////////////////////
// Descriptors
////////////////////////////////////////////////////////////////////////////////

enum 
{
    MIP_FILTER_CMD_DESC_SET                                                   = 0x0D,
    
    MIP_CMD_DESC_FILTER_RESET_FILTER                                          = 0x01,
    MIP_CMD_DESC_FILTER_SET_INITIAL_ATTITUDE                                  = 0x02,
    MIP_CMD_DESC_FILTER_SET_INITIAL_HEADING                                   = 0x03,
    MIP_CMD_DESC_FILTER_SET_INITIAL_HEADING_FROM_MAG                          = 0x04,
    MIP_CMD_DESC_FILTER_RUN                                                   = 0x05,
    MIP_CMD_DESC_FILTER_SELECT_FILTER                                         = 0x0F,
    MIP_CMD_DESC_FILTER_VEHICLE_DYNAMICS_MODE                                 = 0x10,
    MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_EULER                         = 0x11,
    MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_OFFSET                                 = 0x12,
    MIP_CMD_DESC_FILTER_ANTENNA_OFFSET                                        = 0x13,
    MIP_CMD_DESC_FILTER_ESTIMATION_CONTROL_FLAGS                              = 0x14,
    MIP_CMD_DESC_FILTER_GNSS_SOURCE_CONTROL                                   = 0x15,
    MIP_CMD_DESC_FILTER_EXTERNAL_GNSS_UPDATE                                  = 0x16,
    MIP_CMD_DESC_FILTER_EXTERNAL_HEADING_UPDATE                               = 0x17,
    MIP_CMD_DESC_FILTER_HEADING_UPDATE_CONTROL                                = 0x18,
    MIP_CMD_DESC_FILTER_AUTOINIT_CONTROL                                      = 0x19,
    MIP_CMD_DESC_FILTER_ACCEL_NOISE                                           = 0x1A,
    MIP_CMD_DESC_FILTER_GYRO_NOISE                                            = 0x1B,
    MIP_CMD_DESC_FILTER_ACCEL_BIAS_MODEL                                      = 0x1C,
    MIP_CMD_DESC_FILTER_GYRO_BIAS_MODEL                                       = 0x1D,
    MIP_CMD_DESC_FILTER_ZUPT_CONTROL                                          = 0x1E,
    MIP_CMD_DESC_FILTER_EXTERNAL_HEADING_UPDATE_WITH_TIME                     = 0x1F,
    MIP_CMD_DESC_FILTER_ANGULAR_ZUPT_CONTROL                                  = 0x20,
    MIP_CMD_DESC_FILTER_TARE_ORIENTATION                                      = 0x21,
    MIP_CMD_DESC_FILTER_COMMANDED_ZUPT                                        = 0x22,
    MIP_CMD_DESC_FILTER_COMMANDED_ANGULAR_ZUPT                                = 0x23,
    MIP_CMD_DESC_FILTER_AUTO_HEADING_UPDATE_CONTROL                           = 0x24,
    MIP_CMD_DESC_FILTER_MAG_AUTO_CALIBRATION_CONTROL                          = 0x25,
    MIP_CMD_DESC_FILTER_MAG_CAPTURE_AUTO_CALIBRATION                          = 0x27,
    MIP_CMD_DESC_FILTER_GRAVITY_NOISE                                         = 0x28,
    MIP_CMD_DESC_FILTER_PRESSURE_NOISE                                        = 0x29,
    MIP_CMD_DESC_FILTER_GRAVITY_NOISE_MINIMUM                                 = 0x2A,
    MIP_CMD_DESC_FILTER_HARD_IRON_OFFSET_NOISE                                = 0x2B,
    MIP_CMD_DESC_FILTER_SOFT_IRON_MATRIX_NOISE                                = 0x2C,
    MIP_CMD_DESC_FILTER_LOW_PASS_SENSOR_FILTER                                = 0x30,
    MIP_CMD_DESC_FILTER_MAG_NOISE                                             = 0x42,
    MIP_CMD_DESC_FILTER_DECLINATION_SOURCE                                    = 0x43,
    MIP_CMD_DESC_FILTER_HOT_START_CONTROL                                     = 0x48,
    MIP_CMD_DESC_FILTER_SECONDARY_VELOCITY_AIDING_CONTROL                     = 0x4A,
    MIP_CMD_DESC_FILTER_INCLINATION_SOURCE                                    = 0x4C,
    MIP_CMD_DESC_FILTER_MAGNETIC_MAGNITUDE_SOURCE                             = 0x4D,
    MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_DCM                           = 0x4E,
    MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_QUATERNION                    = 0x4F,
    MIP_CMD_DESC_FILTER_REFERENCE_POSITION                                    = 0x26,
    MIP_CMD_DESC_FILTER_ENABLE_MEASUREMENT                                    = 0x41,
    MIP_CMD_DESC_FILTER_ACCEL_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL    = 0x44,
    MIP_CMD_DESC_FILTER_MAG_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL      = 0x45,
    MIP_CMD_DESC_FILTER_MAG_DIP_ANGLE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL      = 0x46,
    MIP_CMD_DESC_FILTER_ALTITUDE_AIDING_CONTROL                               = 0x47,
    MIP_CMD_DESC_FILTER_SECONDARY_PITCH_ROLL_AIDING_CONTROL                   = 0x4B,
    MIP_CMD_DESC_FILTER_AIDING_MEASUREMENT_ENABLE                             = 0x50,
    MIP_CMD_DESC_FILTER_KINEMATIC_CONSTRAINT                                  = 0x51,
    MIP_CMD_DESC_FILTER_INITIALIZATION_CONFIGURATION                          = 0x52,
    MIP_CMD_DESC_FILTER_ADAPTIVE_FILTER_OPTIONS                               = 0x53,
    MIP_CMD_DESC_FILTER_MULTI_ANTENNA_OFFSET                                  = 0x54,
    MIP_CMD_DESC_FILTER_REL_POS_CONFIGURATION                                 = 0x55,
    MIP_CMD_DESC_FILTER_REF_POINT_LEVER_ARM                                   = 0x56,
    MIP_CMD_DESC_FILTER_SPEED_MEASUREMENT                                     = 0x60,
    MIP_CMD_DESC_FILTER_SPEED_LEVER_ARM                                       = 0x61,
    MIP_CMD_DESC_VERTICAL_GYRO_CONSTRAINT_CONTROL                             = 0x62,
    MIP_CMD_DESC_WHEELED_VEHICLE_CONSTRAINT_CONTROL                           = 0x63,
    MIP_CMD_DESC_GNSS_ANTENNA_CALIBRATION_CONTROL                             = 0x64,
    MIP_CMD_DESC_SENSOR_TO_VEHICLE_CALIBRATION_CONTROL                        = 0x65,
    
    MIP_REPLY_DESC_FILTER_VEHICLE_DYNAMICS_MODE                               = 0x80,
    MIP_REPLY_DESC_FILTER_SENSOR2VEHICLE_ROTATION_EULER                       = 0x81,
    MIP_REPLY_DESC_FILTER_SENSOR2VEHICLE_OFFSET                               = 0x82,
    MIP_REPLY_DESC_FILTER_ANTENNA_OFFSET                                      = 0x83,
    MIP_REPLY_DESC_FILTER_ESTIMATION_CONTROL_FLAGS                            = 0x84,
    MIP_REPLY_DESC_FILTER_GNSS_SOURCE_CONTROL                                 = 0x86,
    MIP_REPLY_DESC_FILTER_HEADING_UPDATE_CONTROL                              = 0x87,
    MIP_REPLY_DESC_FILTER_AUTOINIT_CONTROL                                    = 0x88,
    MIP_REPLY_DESC_FILTER_ACCEL_NOISE                                         = 0x89,
    MIP_REPLY_DESC_FILTER_GYRO_NOISE                                          = 0x8A,
    MIP_REPLY_DESC_FILTER_MAG_NOISE                                           = 0xB1,
    MIP_REPLY_DESC_FILTER_ACCEL_BIAS_MODEL                                    = 0x8B,
    MIP_REPLY_DESC_FILTER_GYRO_BIAS_MODEL                                     = 0x8C,
    MIP_REPLY_DESC_FILTER_ZUPT_CONTROL                                        = 0x8D,
    MIP_REPLY_DESC_FILTER_ANGULAR_ZUPT_CONTROL                                = 0x8E,
    MIP_REPLY_DESC_FILTER_SELECT_FILTER                                       = 0x8F,
    MIP_REPLY_DESC_FILTER_GRAVITY_NOISE                                       = 0x93,
    MIP_REPLY_DESC_FILTER_PRESSURE_NOISE                                      = 0x94,
    MIP_REPLY_DESC_FILTER_GRAVITY_NOISE_MINIMUM                               = 0x95,
    MIP_REPLY_DESC_FILTER_HARD_IRON_OFFSET_NOISE                              = 0x96,
    MIP_REPLY_DESC_FILTER_SOFT_IRON_MATRIX_NOISE                              = 0x97,
    MIP_REPLY_DESC_FILTER_LOW_PASS_SENSOR_FILTER                              = 0xA0,
    MIP_REPLY_DESC_FILTER_SET_INITIAL_HEADING                                 = 0x98,
    MIP_REPLY_DESC_FILTER_REFERENCE_POSITION                                  = 0x90,
    MIP_REPLY_DESC_FILTER_AUTO_HEADING_UPDATE_CONTROL                         = 0x91,
    MIP_REPLY_DESC_FILTER_MAG_AUTO_CALIBRATION_CONTROL                        = 0x92,
    MIP_REPLY_DESC_FILTER_ENABLE_MEASUREMENT                                  = 0xB0,
    MIP_REPLY_DESC_FILTER_DECLINATION_SOURCE                                  = 0xB2,
    MIP_REPLY_DESC_FILTER_ACCEL_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL  = 0xB3,
    MIP_REPLY_DESC_FILTER_MAG_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL    = 0xB4,
    MIP_REPLY_DESC_FILTER_MAG_DIP_ANGLE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL    = 0xB5,
    MIP_REPLY_DESC_FILTER_MAG_ANGULAR_RATE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL = 0xB6,
    MIP_REPLY_DESC_FILTER_ALTITUDE_AIDING_CONTROL                             = 0xB7,
    MIP_REPLY_DESC_FILTER_HOT_START_CONTROL                                   = 0xB8,
    MIP_REPLY_DESC_FILTER_SECONDARY_VELOCITY_AIDING_CONTROL                   = 0xBA,
    MIP_REPLY_DESC_FILTER_SECONDARY_PITCH_ROLL_AIDING_CONTROL                 = 0xBB,
    MIP_REPLY_DESC_FILTER_INCLINATION_SOURCE                                  = 0xBC,
    MIP_REPLY_DESC_FILTER_MAGNETIC_MAGNITUDE_SOURCE                           = 0xBD,
    MIP_REPLY_DESC_FILTER_SENSOR2VEHICLE_ROTATION_DCM                         = 0xBE,
    MIP_REPLY_DESC_FILTER_SENSOR2VEHICLE_ROTATION_QUATERNION                  = 0xBF,
    MIP_REPLY_DESC_FILTER_AIDING_MEASUREMENT_ENABLE                           = 0xD0,
    MIP_REPLY_DESC_FILTER_KINEMATIC_CONSTRAINT                                = 0xD1,
    MIP_REPLY_DESC_FILTER_INITIALIZATION_CONFIGURATION                        = 0xD2,
    MIP_REPLY_DESC_FILTER_ADAPTIVE_FILTER_OPTIONS                             = 0xD3,
    MIP_REPLY_DESC_FILTER_MULTI_ANTENNA_OFFSET                                = 0xD4,
    MIP_REPLY_DESC_FILTER_REL_POS_CONFIGURATION                               = 0xD5,
    MIP_REPLY_DESC_FILTER_SPEED_MEASUREMENT                                   = 0xE0,
    MIP_REPLY_DESC_VERTICAL_GYRO_CONSTRAINT_CONTROL                           = 0xE2,
    MIP_REPLY_DESC_WHEELED_VEHICLE_CONSTRAINT_CONTROL                         = 0xE3,
    MIP_REPLY_DESC_GNSS_ANTENNA_CALIBRATION_CONTROL                           = 0xE4,
    MIP_REPLY_DESC_FILTER_TARE_ORIENTATION                                    = 0xA1,
    MIP_REPLY_DESC_FILTER_REF_POINT_LEVER_ARM                                 = 0xD6,
    MIP_REPLY_DESC_FILTER_SPEED_LEVER_ARM                                     = 0xE1,
};

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

typedef uint8_t mip_filter_reference_frame;
static const mip_filter_reference_frame MIP_FILTER_REFERENCE_FRAME_ECEF = 1; ///<  WGS84 Earth-fixed, earth centered coordinates
static const mip_filter_reference_frame MIP_FILTER_REFERENCE_FRAME_LLH  = 2; ///<  WGS84 Latitude, longitude, and height above ellipsoid

void insert_mip_filter_reference_frame(struct mip_serializer* serializer, const mip_filter_reference_frame self);
void extract_mip_filter_reference_frame(struct mip_serializer* serializer, mip_filter_reference_frame* self);

typedef uint8_t mip_filter_mag_param_source;
static const mip_filter_mag_param_source MIP_FILTER_MAG_PARAM_SOURCE_NONE   = 1; ///<  No source. See command documentation for default behavior
static const mip_filter_mag_param_source MIP_FILTER_MAG_PARAM_SOURCE_WMM    = 2; ///<  Magnetic field is assumed to conform to the World Magnetic Model, calculated using current location estimate as an input to the model.
static const mip_filter_mag_param_source MIP_FILTER_MAG_PARAM_SOURCE_MANUAL = 3; ///<  Magnetic field is assumed to have the parameter specified by the user.

void insert_mip_filter_mag_param_source(struct mip_serializer* serializer, const mip_filter_mag_param_source self);
void extract_mip_filter_mag_param_source(struct mip_serializer* serializer, mip_filter_mag_param_source* self);

typedef uint8_t mip_filter_adaptive_measurement;
static const mip_filter_adaptive_measurement MIP_FILTER_ADAPTIVE_MEASUREMENT_DISABLED = 0; ///<  No adaptive measurement
static const mip_filter_adaptive_measurement MIP_FILTER_ADAPTIVE_MEASUREMENT_FIXED    = 1; ///<  Enable fixed adaptive measurement (use specified limits)
static const mip_filter_adaptive_measurement MIP_FILTER_ADAPTIVE_MEASUREMENT_AUTO     = 2; ///<  Enable auto adaptive measurement

void insert_mip_filter_adaptive_measurement(struct mip_serializer* serializer, const mip_filter_adaptive_measurement self);
void extract_mip_filter_adaptive_measurement(struct mip_serializer* serializer, mip_filter_adaptive_measurement* self);


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_reset  (0x0D,0x01) Reset [C]
/// Resets the filter to the initialization state.
/// 
/// If the auto-initialization feature is disabled, the initial attitude or heading must be set in
/// order to enter the run state after a reset.
///
///@{

mip_cmd_result mip_filter_reset(struct mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_set_initial_attitude  (0x0D,0x02) Set Initial Attitude [C]
/// Set the sensor initial attitude.
/// 
/// This command can only be issued in the "Init" state and should be used with a good
/// estimate of the vehicle attitude.  The Euler angles are the sensor body frame with respect
/// to the NED frame.
/// 
/// The valid input ranges are as follows:
/// 
/// Roll:    [-pi, pi]
/// Pitch:   [-pi/2, pi/2]
/// Heading: [-pi, pi]
/// 
///
///@{

struct mip_filter_set_initial_attitude_command
{
    float roll; ///< [radians]
    float pitch; ///< [radians]
    float heading; ///< [radians]
    
};
typedef struct mip_filter_set_initial_attitude_command mip_filter_set_initial_attitude_command;
void insert_mip_filter_set_initial_attitude_command(struct mip_serializer* serializer, const mip_filter_set_initial_attitude_command* self);
void extract_mip_filter_set_initial_attitude_command(struct mip_serializer* serializer, mip_filter_set_initial_attitude_command* self);

mip_cmd_result mip_filter_set_initial_attitude(struct mip_interface* device, float roll, float pitch, float heading);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_estimation_control  (0x0D,0x14) Estimation Control [C]
/// Estimation Control Flags
/// 
/// Controls which parameters are estimated by the Kalman Filter.
/// 
/// Desired settings should be logically ORed together.
/// 
/// Examples:
/// 
/// 0x0001 - Enable Gyro Bias Estimation Only
/// 0x0063 - Enable Gyro Bias, Accel Bias, and Mag Auto Hard and Soft Iron Cal States Only
/// 
///
///@{

typedef uint16_t mip_filter_estimation_control_command_enable_flags;
static const mip_filter_estimation_control_command_enable_flags MIP_FILTER_ESTIMATION_CONTROL_COMMAND_ENABLE_FLAGS_NONE               = 0x0000;
static const mip_filter_estimation_control_command_enable_flags MIP_FILTER_ESTIMATION_CONTROL_COMMAND_ENABLE_FLAGS_GYRO_BIAS          = 0x0001; ///<  
static const mip_filter_estimation_control_command_enable_flags MIP_FILTER_ESTIMATION_CONTROL_COMMAND_ENABLE_FLAGS_ACCEL_BIAS         = 0x0002; ///<  
static const mip_filter_estimation_control_command_enable_flags MIP_FILTER_ESTIMATION_CONTROL_COMMAND_ENABLE_FLAGS_GYRO_SCALE_FACTOR  = 0x0004; ///<  
static const mip_filter_estimation_control_command_enable_flags MIP_FILTER_ESTIMATION_CONTROL_COMMAND_ENABLE_FLAGS_ACCEL_SCALE_FACTOR = 0x0008; ///<  
static const mip_filter_estimation_control_command_enable_flags MIP_FILTER_ESTIMATION_CONTROL_COMMAND_ENABLE_FLAGS_ANTENNA_OFFSET     = 0x0010; ///<  
static const mip_filter_estimation_control_command_enable_flags MIP_FILTER_ESTIMATION_CONTROL_COMMAND_ENABLE_FLAGS_AUTO_MAG_HARD_IRON = 0x0020; ///<  
static const mip_filter_estimation_control_command_enable_flags MIP_FILTER_ESTIMATION_CONTROL_COMMAND_ENABLE_FLAGS_AUTO_MAG_SOFT_IRON = 0x0040; ///<  
static const mip_filter_estimation_control_command_enable_flags MIP_FILTER_ESTIMATION_CONTROL_COMMAND_ENABLE_FLAGS_ALL                = 0x007F;

struct mip_filter_estimation_control_command
{
    mip_function_selector function;
    mip_filter_estimation_control_command_enable_flags enable; ///< See above
    
};
typedef struct mip_filter_estimation_control_command mip_filter_estimation_control_command;
void insert_mip_filter_estimation_control_command(struct mip_serializer* serializer, const mip_filter_estimation_control_command* self);
void extract_mip_filter_estimation_control_command(struct mip_serializer* serializer, mip_filter_estimation_control_command* self);

void insert_mip_filter_estimation_control_command_enable_flags(struct mip_serializer* serializer, const mip_filter_estimation_control_command_enable_flags self);
void extract_mip_filter_estimation_control_command_enable_flags(struct mip_serializer* serializer, mip_filter_estimation_control_command_enable_flags* self);

struct mip_filter_estimation_control_response
{
    mip_filter_estimation_control_command_enable_flags enable; ///< See above
    
};
typedef struct mip_filter_estimation_control_response mip_filter_estimation_control_response;
void insert_mip_filter_estimation_control_response(struct mip_serializer* serializer, const mip_filter_estimation_control_response* self);
void extract_mip_filter_estimation_control_response(struct mip_serializer* serializer, mip_filter_estimation_control_response* self);

mip_cmd_result mip_filter_write_estimation_control(struct mip_interface* device, mip_filter_estimation_control_command_enable_flags enable);
mip_cmd_result mip_filter_read_estimation_control(struct mip_interface* device, mip_filter_estimation_control_command_enable_flags* enable_out);
mip_cmd_result mip_filter_save_estimation_control(struct mip_interface* device);
mip_cmd_result mip_filter_load_estimation_control(struct mip_interface* device);
mip_cmd_result mip_filter_default_estimation_control(struct mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_external_gnss_update  (0x0D,0x16) External Gnss Update [C]
/// Provide a filter measurement from an external GNSS
/// 
/// The GNSS source control must be set to "external" for this command to succeed, otherwise it will be NACK'd.
/// Please refer to your device user manual for information on the maximum rate of this message.
/// 
///
///@{

struct mip_filter_external_gnss_update_command
{
    double gps_time; ///< [seconds]
    uint16_t gps_week; ///< [GPS week number, not modulus 1024]
    double latitude; ///< [degrees]
    double longitude; ///< [degrees]
    double height; ///< Above WGS84 ellipsoid [meters]
    mip_vector3f velocity; ///< NED Frame [meters/second]
    mip_vector3f pos_uncertainty; ///< NED Frame, 1-sigma [meters]
    mip_vector3f vel_uncertainty; ///< NED Frame, 1-sigma [meters/second]
    
};
typedef struct mip_filter_external_gnss_update_command mip_filter_external_gnss_update_command;
void insert_mip_filter_external_gnss_update_command(struct mip_serializer* serializer, const mip_filter_external_gnss_update_command* self);
void extract_mip_filter_external_gnss_update_command(struct mip_serializer* serializer, mip_filter_external_gnss_update_command* self);

mip_cmd_result mip_filter_external_gnss_update(struct mip_interface* device, double gps_time, uint16_t gps_week, double latitude, double longitude, double height, const float* velocity, const float* pos_uncertainty, const float* vel_uncertainty);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_external_heading_update  (0x0D,0x17) External Heading Update [C]
/// Provide a filter measurement from an external heading source
/// 
/// The heading must be the sensor frame with respect to the NED frame.
/// 
/// The heading update control must be set to external for this command to update the filter; otherwise it is NACK'd.
/// Heading angle uncertainties of &lt;= 0.0 will be NACK'd
/// 
/// Please refer to your device user manual for information on the maximum rate of this message.
/// 
/// On -25 models, if the declination source (0x0D, 0x43) is not valid, true heading updates will be NACK'd.
/// On -45 models, if the declination source is invalid, magnetic heading updates will be NACK'd.
/// 
/// 
///
///@{

struct mip_filter_external_heading_update_command
{
    float heading; ///< Bounded by +-PI [radians]
    float heading_uncertainty; ///< 1-sigma [radians]
    uint8_t type; ///< 1 - True, 2 - Magnetic
    
};
typedef struct mip_filter_external_heading_update_command mip_filter_external_heading_update_command;
void insert_mip_filter_external_heading_update_command(struct mip_serializer* serializer, const mip_filter_external_heading_update_command* self);
void extract_mip_filter_external_heading_update_command(struct mip_serializer* serializer, mip_filter_external_heading_update_command* self);

mip_cmd_result mip_filter_external_heading_update(struct mip_interface* device, float heading, float heading_uncertainty, uint8_t type);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_external_heading_update_with_time  (0x0D,0x1F) External Heading Update With Time [C]
/// Provide a filter measurement from an external heading source at a specific GPS time
/// 
/// This is more accurate than the External Heading Update (0x0D, 0x17) and should be used in applications
/// where the rate of heading change will cause significant measurement error due to the sampling, transmission,
/// and processing time required.  Accurate time stamping of the heading information is important.
/// 
/// The heading must be the sensor frame with respect to the NED frame.
/// 
/// The heading update control must be set to external for this command to update the filter; otherwise it is NACK'd.
/// Heading angle uncertainties of &lt;= 0.0 will be NACK'd
/// 
/// Please refer to your device user manual for information on the maximum rate of this message.
/// 
/// On -25 models, if the declination source (0x0D, 0x43) is not valid, true heading updates will be NACK'd.
/// On -45 models, if the declination source is invalid, magnetic heading updates will be NACK'd.
/// 
/// 
///
///@{

struct mip_filter_external_heading_update_with_time_command
{
    double gps_time; ///< [seconds]
    uint16_t gps_week; ///< [GPS week number, not modulus 1024]
    float heading; ///< Relative to true north, bounded by +-PI [radians]
    float heading_uncertainty; ///< 1-sigma [radians]
    uint8_t type; ///< 1 - True, 2 - Magnetic
    
};
typedef struct mip_filter_external_heading_update_with_time_command mip_filter_external_heading_update_with_time_command;
void insert_mip_filter_external_heading_update_with_time_command(struct mip_serializer* serializer, const mip_filter_external_heading_update_with_time_command* self);
void extract_mip_filter_external_heading_update_with_time_command(struct mip_serializer* serializer, mip_filter_external_heading_update_with_time_command* self);

mip_cmd_result mip_filter_external_heading_update_with_time(struct mip_interface* device, double gps_time, uint16_t gps_week, float heading, float heading_uncertainty, uint8_t type);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_tare_orientation  (0x0D,0x21) Tare Orientation [C]
/// Tare the device orientation.
/// 
/// This function uses the current device orientation relative to the NED frame as the current sensor to vehicle transformation.
/// This command is provided as a convenient way to set the sensor to vehicle frame transformation.
/// The filter must be initialized and have a valid attitude output. If the attitude is not valid, an error will be returned.
///
///@{

typedef uint8_t mip_filter_tare_orientation_command_mip_tare_axes;
static const mip_filter_tare_orientation_command_mip_tare_axes MIP_FILTER_TARE_ORIENTATION_COMMAND_MIP_TARE_AXES_NONE  = 0x0;
static const mip_filter_tare_orientation_command_mip_tare_axes MIP_FILTER_TARE_ORIENTATION_COMMAND_MIP_TARE_AXES_ROLL  = 0x1; ///<  
static const mip_filter_tare_orientation_command_mip_tare_axes MIP_FILTER_TARE_ORIENTATION_COMMAND_MIP_TARE_AXES_PITCH = 0x2; ///<  
static const mip_filter_tare_orientation_command_mip_tare_axes MIP_FILTER_TARE_ORIENTATION_COMMAND_MIP_TARE_AXES_YAW   = 0x4; ///<  
static const mip_filter_tare_orientation_command_mip_tare_axes MIP_FILTER_TARE_ORIENTATION_COMMAND_MIP_TARE_AXES_ALL   = 0x7;

struct mip_filter_tare_orientation_command
{
    mip_function_selector function;
    mip_filter_tare_orientation_command_mip_tare_axes axes; ///< Axes to tare
    
};
typedef struct mip_filter_tare_orientation_command mip_filter_tare_orientation_command;
void insert_mip_filter_tare_orientation_command(struct mip_serializer* serializer, const mip_filter_tare_orientation_command* self);
void extract_mip_filter_tare_orientation_command(struct mip_serializer* serializer, mip_filter_tare_orientation_command* self);

void insert_mip_filter_tare_orientation_command_mip_tare_axes(struct mip_serializer* serializer, const mip_filter_tare_orientation_command_mip_tare_axes self);
void extract_mip_filter_tare_orientation_command_mip_tare_axes(struct mip_serializer* serializer, mip_filter_tare_orientation_command_mip_tare_axes* self);

struct mip_filter_tare_orientation_response
{
    mip_filter_tare_orientation_command_mip_tare_axes axes; ///< Axes to tare
    
};
typedef struct mip_filter_tare_orientation_response mip_filter_tare_orientation_response;
void insert_mip_filter_tare_orientation_response(struct mip_serializer* serializer, const mip_filter_tare_orientation_response* self);
void extract_mip_filter_tare_orientation_response(struct mip_serializer* serializer, mip_filter_tare_orientation_response* self);

mip_cmd_result mip_filter_write_tare_orientation(struct mip_interface* device, mip_filter_tare_orientation_command_mip_tare_axes axes);
mip_cmd_result mip_filter_read_tare_orientation(struct mip_interface* device, mip_filter_tare_orientation_command_mip_tare_axes* axes_out);
mip_cmd_result mip_filter_save_tare_orientation(struct mip_interface* device);
mip_cmd_result mip_filter_load_tare_orientation(struct mip_interface* device);
mip_cmd_result mip_filter_default_tare_orientation(struct mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_vehicle_dynamics_mode  (0x0D,0x10) Vehicle Dynamics Mode [C]
/// Controls the vehicle dynamics mode.
///
///@{

typedef uint8_t mip_filter_vehicle_dynamics_mode_command_dynamics_mode;
static const mip_filter_vehicle_dynamics_mode_command_dynamics_mode MIP_FILTER_VEHICLE_DYNAMICS_MODE_COMMAND_DYNAMICS_MODE_PORTABLE        = 1; ///<  
static const mip_filter_vehicle_dynamics_mode_command_dynamics_mode MIP_FILTER_VEHICLE_DYNAMICS_MODE_COMMAND_DYNAMICS_MODE_AUTOMOTIVE      = 2; ///<  
static const mip_filter_vehicle_dynamics_mode_command_dynamics_mode MIP_FILTER_VEHICLE_DYNAMICS_MODE_COMMAND_DYNAMICS_MODE_AIRBORNE        = 3; ///<  
static const mip_filter_vehicle_dynamics_mode_command_dynamics_mode MIP_FILTER_VEHICLE_DYNAMICS_MODE_COMMAND_DYNAMICS_MODE_AIRBORNE_HIGH_G = 4; ///<  

struct mip_filter_vehicle_dynamics_mode_command
{
    mip_function_selector function;
    mip_filter_vehicle_dynamics_mode_command_dynamics_mode mode;
    
};
typedef struct mip_filter_vehicle_dynamics_mode_command mip_filter_vehicle_dynamics_mode_command;
void insert_mip_filter_vehicle_dynamics_mode_command(struct mip_serializer* serializer, const mip_filter_vehicle_dynamics_mode_command* self);
void extract_mip_filter_vehicle_dynamics_mode_command(struct mip_serializer* serializer, mip_filter_vehicle_dynamics_mode_command* self);

void insert_mip_filter_vehicle_dynamics_mode_command_dynamics_mode(struct mip_serializer* serializer, const mip_filter_vehicle_dynamics_mode_command_dynamics_mode self);
void extract_mip_filter_vehicle_dynamics_mode_command_dynamics_mode(struct mip_serializer* serializer, mip_filter_vehicle_dynamics_mode_command_dynamics_mode* self);

struct mip_filter_vehicle_dynamics_mode_response
{
    mip_filter_vehicle_dynamics_mode_command_dynamics_mode mode;
    
};
typedef struct mip_filter_vehicle_dynamics_mode_response mip_filter_vehicle_dynamics_mode_response;
void insert_mip_filter_vehicle_dynamics_mode_response(struct mip_serializer* serializer, const mip_filter_vehicle_dynamics_mode_response* self);
void extract_mip_filter_vehicle_dynamics_mode_response(struct mip_serializer* serializer, mip_filter_vehicle_dynamics_mode_response* self);

mip_cmd_result mip_filter_write_vehicle_dynamics_mode(struct mip_interface* device, mip_filter_vehicle_dynamics_mode_command_dynamics_mode mode);
mip_cmd_result mip_filter_read_vehicle_dynamics_mode(struct mip_interface* device, mip_filter_vehicle_dynamics_mode_command_dynamics_mode* mode_out);
mip_cmd_result mip_filter_save_vehicle_dynamics_mode(struct mip_interface* device);
mip_cmd_result mip_filter_load_vehicle_dynamics_mode(struct mip_interface* device);
mip_cmd_result mip_filter_default_vehicle_dynamics_mode(struct mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_sensor_to_vehicle_rotation_euler  (0x0D,0x11) Sensor To Vehicle Rotation Euler [C]
/// Set the sensor to vehicle frame rotation using Yaw, Pitch, Roll Euler angles.
/// 
/// Note: This is the rotation, the inverse of the transformation.
/// These angles define the rotation from the sensor body frame to the fixed vehicle frame.<br/>
/// Please reference the device Theory of Operation for more information.<br/>
/// The rotation is stored in the device as a quaternion.  When Euler angles are read back from the device, they may not
/// be equivalent in value to the Euler angles used to set the rotation, but they are functionally equivalent.<br/>
/// <br/><br/>
/// This rotation affects the following output quantities:<br/><br/>
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
///
///@{

struct mip_filter_sensor_to_vehicle_rotation_euler_command
{
    mip_function_selector function;
    float roll; ///< [radians]
    float pitch; ///< [radians]
    float yaw; ///< [radians]
    
};
typedef struct mip_filter_sensor_to_vehicle_rotation_euler_command mip_filter_sensor_to_vehicle_rotation_euler_command;
void insert_mip_filter_sensor_to_vehicle_rotation_euler_command(struct mip_serializer* serializer, const mip_filter_sensor_to_vehicle_rotation_euler_command* self);
void extract_mip_filter_sensor_to_vehicle_rotation_euler_command(struct mip_serializer* serializer, mip_filter_sensor_to_vehicle_rotation_euler_command* self);

struct mip_filter_sensor_to_vehicle_rotation_euler_response
{
    float roll; ///< [radians]
    float pitch; ///< [radians]
    float yaw; ///< [radians]
    
};
typedef struct mip_filter_sensor_to_vehicle_rotation_euler_response mip_filter_sensor_to_vehicle_rotation_euler_response;
void insert_mip_filter_sensor_to_vehicle_rotation_euler_response(struct mip_serializer* serializer, const mip_filter_sensor_to_vehicle_rotation_euler_response* self);
void extract_mip_filter_sensor_to_vehicle_rotation_euler_response(struct mip_serializer* serializer, mip_filter_sensor_to_vehicle_rotation_euler_response* self);

mip_cmd_result mip_filter_write_sensor_to_vehicle_rotation_euler(struct mip_interface* device, float roll, float pitch, float yaw);
mip_cmd_result mip_filter_read_sensor_to_vehicle_rotation_euler(struct mip_interface* device, float* roll_out, float* pitch_out, float* yaw_out);
mip_cmd_result mip_filter_save_sensor_to_vehicle_rotation_euler(struct mip_interface* device);
mip_cmd_result mip_filter_load_sensor_to_vehicle_rotation_euler(struct mip_interface* device);
mip_cmd_result mip_filter_default_sensor_to_vehicle_rotation_euler(struct mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_sensor_to_vehicle_rotation_dcm  (0x0D,0x4E) Sensor To Vehicle Rotation Dcm [C]
/// Set the sensor to vehicle frame rotation using a row-major direction cosine matrix.
/// 
/// Note: This is the rotation, the inverse of the transformation.
/// This matrix defines the rotation from the sensor body frame to the fixed vehicle frame.<br/>
/// Please reference the device Theory of Operation for more information.<br/>
/// The matrix must be orthonormal (tolerance 1e-3) or the device will NACK the command.
/// The rotation is stored in the device as a quaternion.  When the DCM is read back from the device, the components may not
/// be exactly equivalent in value to the DCM used to set the rotation, but they are functionally equivalent.<br/>
/// <br/>
/// Matrix element order:<br/><br/>
/// 
/// EQSTART T_{SEN}^{VEH} = \begin{bmatrix} 0 &amp; 1 &amp; 2\\  3 &amp; 4 &amp; 5\\ 6 &amp; 7 &amp; 8 \end{bmatrix} EQEND
/// 
/// <br/><br/>
/// This rotation affects the following output quantities:<br/><br/>
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
///
///@{

struct mip_filter_sensor_to_vehicle_rotation_dcm_command
{
    mip_function_selector function;
    mip_matrix3f dcm;
    
};
typedef struct mip_filter_sensor_to_vehicle_rotation_dcm_command mip_filter_sensor_to_vehicle_rotation_dcm_command;
void insert_mip_filter_sensor_to_vehicle_rotation_dcm_command(struct mip_serializer* serializer, const mip_filter_sensor_to_vehicle_rotation_dcm_command* self);
void extract_mip_filter_sensor_to_vehicle_rotation_dcm_command(struct mip_serializer* serializer, mip_filter_sensor_to_vehicle_rotation_dcm_command* self);

struct mip_filter_sensor_to_vehicle_rotation_dcm_response
{
    mip_matrix3f dcm;
    
};
typedef struct mip_filter_sensor_to_vehicle_rotation_dcm_response mip_filter_sensor_to_vehicle_rotation_dcm_response;
void insert_mip_filter_sensor_to_vehicle_rotation_dcm_response(struct mip_serializer* serializer, const mip_filter_sensor_to_vehicle_rotation_dcm_response* self);
void extract_mip_filter_sensor_to_vehicle_rotation_dcm_response(struct mip_serializer* serializer, mip_filter_sensor_to_vehicle_rotation_dcm_response* self);

mip_cmd_result mip_filter_write_sensor_to_vehicle_rotation_dcm(struct mip_interface* device, const float* dcm);
mip_cmd_result mip_filter_read_sensor_to_vehicle_rotation_dcm(struct mip_interface* device, float* dcm_out);
mip_cmd_result mip_filter_save_sensor_to_vehicle_rotation_dcm(struct mip_interface* device);
mip_cmd_result mip_filter_load_sensor_to_vehicle_rotation_dcm(struct mip_interface* device);
mip_cmd_result mip_filter_default_sensor_to_vehicle_rotation_dcm(struct mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_sensor_to_vehicle_rotation_quaternion  (0x0D,0x4F) Sensor To Vehicle Rotation Quaternion [C]
/// Set the sensor to vehicle frame rotation using a quaternion.
/// 
/// Note: This is the rotation, the inverse of the transformation.
/// This quaternion defines the rotation from the sensor body frame to the fixed vehicle frame.<br/>
/// Please reference the device Theory of Operation for more information.<br/>
/// The quaternion must be unit length (tolerance 1e-3) or the device will NACK the command.
/// The rotation is stored in the device as a unit quaternion.  When the quaternion elements are read back from the device, they may not
/// be equivalent in value to the quaternion used to set the rotation, due to normalization.<br/>
/// <br/>
/// Quaternion element definition:<br/><br/>
/// <br/>
/// EQSTART Q_{SEN}^{VEH} = \begin{bmatrix} q_{0} &amp; q_{1}*i  &amp; q_{2}*j  &amp; q_{3}*k \end{bmatrix} EQEND
/// <br/><br/>
/// This rotation affects the following output quantities:<br/><br/>
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
///
///@{

struct mip_filter_sensor_to_vehicle_rotation_quaternion_command
{
    mip_function_selector function;
    mip_quatf quat;
    
};
typedef struct mip_filter_sensor_to_vehicle_rotation_quaternion_command mip_filter_sensor_to_vehicle_rotation_quaternion_command;
void insert_mip_filter_sensor_to_vehicle_rotation_quaternion_command(struct mip_serializer* serializer, const mip_filter_sensor_to_vehicle_rotation_quaternion_command* self);
void extract_mip_filter_sensor_to_vehicle_rotation_quaternion_command(struct mip_serializer* serializer, mip_filter_sensor_to_vehicle_rotation_quaternion_command* self);

struct mip_filter_sensor_to_vehicle_rotation_quaternion_response
{
    mip_quatf quat;
    
};
typedef struct mip_filter_sensor_to_vehicle_rotation_quaternion_response mip_filter_sensor_to_vehicle_rotation_quaternion_response;
void insert_mip_filter_sensor_to_vehicle_rotation_quaternion_response(struct mip_serializer* serializer, const mip_filter_sensor_to_vehicle_rotation_quaternion_response* self);
void extract_mip_filter_sensor_to_vehicle_rotation_quaternion_response(struct mip_serializer* serializer, mip_filter_sensor_to_vehicle_rotation_quaternion_response* self);

mip_cmd_result mip_filter_write_sensor_to_vehicle_rotation_quaternion(struct mip_interface* device, const float* quat);
mip_cmd_result mip_filter_read_sensor_to_vehicle_rotation_quaternion(struct mip_interface* device, float* quat_out);
mip_cmd_result mip_filter_save_sensor_to_vehicle_rotation_quaternion(struct mip_interface* device);
mip_cmd_result mip_filter_load_sensor_to_vehicle_rotation_quaternion(struct mip_interface* device);
mip_cmd_result mip_filter_default_sensor_to_vehicle_rotation_quaternion(struct mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_sensor_to_vehicle_offset  (0x0D,0x12) Sensor To Vehicle Offset [C]
/// Set the sensor to vehicle frame offset, expressed in the sensor frame.
/// 
/// This is a simple offset, not a lever arm.  It does not compensate for inertial effects experienced from being offset from the center of gravity/rotation of the vehicle.
/// It simply adds the offset to the position output to express it in the origin of the user's vehicle frame.
/// 
/// This offset affects the following output quantities:
/// Estimated LLH Position
/// 
/// The magnitude of the offset vector is limited to 10 meters
///
///@{

struct mip_filter_sensor_to_vehicle_offset_command
{
    mip_function_selector function;
    mip_vector3f offset; ///< [meters]
    
};
typedef struct mip_filter_sensor_to_vehicle_offset_command mip_filter_sensor_to_vehicle_offset_command;
void insert_mip_filter_sensor_to_vehicle_offset_command(struct mip_serializer* serializer, const mip_filter_sensor_to_vehicle_offset_command* self);
void extract_mip_filter_sensor_to_vehicle_offset_command(struct mip_serializer* serializer, mip_filter_sensor_to_vehicle_offset_command* self);

struct mip_filter_sensor_to_vehicle_offset_response
{
    mip_vector3f offset; ///< [meters]
    
};
typedef struct mip_filter_sensor_to_vehicle_offset_response mip_filter_sensor_to_vehicle_offset_response;
void insert_mip_filter_sensor_to_vehicle_offset_response(struct mip_serializer* serializer, const mip_filter_sensor_to_vehicle_offset_response* self);
void extract_mip_filter_sensor_to_vehicle_offset_response(struct mip_serializer* serializer, mip_filter_sensor_to_vehicle_offset_response* self);

mip_cmd_result mip_filter_write_sensor_to_vehicle_offset(struct mip_interface* device, const float* offset);
mip_cmd_result mip_filter_read_sensor_to_vehicle_offset(struct mip_interface* device, float* offset_out);
mip_cmd_result mip_filter_save_sensor_to_vehicle_offset(struct mip_interface* device);
mip_cmd_result mip_filter_load_sensor_to_vehicle_offset(struct mip_interface* device);
mip_cmd_result mip_filter_default_sensor_to_vehicle_offset(struct mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_antenna_offset  (0x0D,0x13) Antenna Offset [C]
/// Set the sensor to GNSS antenna offset.
/// 
/// This is expressed in the sensor frame, from the sensor origin to the GNSS antenna RF center.
/// 
/// The magnitude of the offset vector is limited to 10 meters
/// 
///
///@{

struct mip_filter_antenna_offset_command
{
    mip_function_selector function;
    mip_vector3f offset; ///< [meters]
    
};
typedef struct mip_filter_antenna_offset_command mip_filter_antenna_offset_command;
void insert_mip_filter_antenna_offset_command(struct mip_serializer* serializer, const mip_filter_antenna_offset_command* self);
void extract_mip_filter_antenna_offset_command(struct mip_serializer* serializer, mip_filter_antenna_offset_command* self);

struct mip_filter_antenna_offset_response
{
    mip_vector3f offset; ///< [meters]
    
};
typedef struct mip_filter_antenna_offset_response mip_filter_antenna_offset_response;
void insert_mip_filter_antenna_offset_response(struct mip_serializer* serializer, const mip_filter_antenna_offset_response* self);
void extract_mip_filter_antenna_offset_response(struct mip_serializer* serializer, mip_filter_antenna_offset_response* self);

mip_cmd_result mip_filter_write_antenna_offset(struct mip_interface* device, const float* offset);
mip_cmd_result mip_filter_read_antenna_offset(struct mip_interface* device, float* offset_out);
mip_cmd_result mip_filter_save_antenna_offset(struct mip_interface* device);
mip_cmd_result mip_filter_load_antenna_offset(struct mip_interface* device);
mip_cmd_result mip_filter_default_antenna_offset(struct mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_gnss_source  (0x0D,0x15) Gnss Source [C]
/// Control the source of GNSS information used to update the Kalman Filter.
/// 
/// Changing the GNSS source while the sensor is in the "running" state may temporarily place
/// it back in the "init" state until the new source of GNSS data is received.
/// 
///
///@{

typedef uint8_t mip_filter_gnss_source_command_source;
static const mip_filter_gnss_source_command_source MIP_FILTER_GNSS_SOURCE_COMMAND_SOURCE_ALL_INT = 1; ///<  All internal receivers
static const mip_filter_gnss_source_command_source MIP_FILTER_GNSS_SOURCE_COMMAND_SOURCE_EXT     = 2; ///<  External GNSS messages provided by user
static const mip_filter_gnss_source_command_source MIP_FILTER_GNSS_SOURCE_COMMAND_SOURCE_INT_1   = 3; ///<  Internal GNSS Receiver 1 only
static const mip_filter_gnss_source_command_source MIP_FILTER_GNSS_SOURCE_COMMAND_SOURCE_INT_2   = 4; ///<  Internal GNSS Receiver 2 only

struct mip_filter_gnss_source_command
{
    mip_function_selector function;
    mip_filter_gnss_source_command_source source;
    
};
typedef struct mip_filter_gnss_source_command mip_filter_gnss_source_command;
void insert_mip_filter_gnss_source_command(struct mip_serializer* serializer, const mip_filter_gnss_source_command* self);
void extract_mip_filter_gnss_source_command(struct mip_serializer* serializer, mip_filter_gnss_source_command* self);

void insert_mip_filter_gnss_source_command_source(struct mip_serializer* serializer, const mip_filter_gnss_source_command_source self);
void extract_mip_filter_gnss_source_command_source(struct mip_serializer* serializer, mip_filter_gnss_source_command_source* self);

struct mip_filter_gnss_source_response
{
    mip_filter_gnss_source_command_source source;
    
};
typedef struct mip_filter_gnss_source_response mip_filter_gnss_source_response;
void insert_mip_filter_gnss_source_response(struct mip_serializer* serializer, const mip_filter_gnss_source_response* self);
void extract_mip_filter_gnss_source_response(struct mip_serializer* serializer, mip_filter_gnss_source_response* self);

mip_cmd_result mip_filter_write_gnss_source(struct mip_interface* device, mip_filter_gnss_source_command_source source);
mip_cmd_result mip_filter_read_gnss_source(struct mip_interface* device, mip_filter_gnss_source_command_source* source_out);
mip_cmd_result mip_filter_save_gnss_source(struct mip_interface* device);
mip_cmd_result mip_filter_load_gnss_source(struct mip_interface* device);
mip_cmd_result mip_filter_default_gnss_source(struct mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_heading_source  (0x0D,0x18) Heading Source [C]
/// Control the source of heading information used to update the Kalman Filter.
/// 
/// 1. To use internal GNSS velocity vector for heading updates, the target application
/// must have minimal (preferably no) side-slip.  This option is good for wheeled vehicles.
/// 
/// 2. On some devices, when using GNSS velocity vector for heading updates, the X-axis of the device
/// must align with the direction of travel.  Please reference the user guide for your particular device to
/// determine if this limitation is applicable.
/// 
/// 3. When none is selected, the heading estimate can still converge if GNSS is available and sufficient dynamic motion
/// (change in direction of travel and acceleration) is experienced.  The heading may drift when: stationary, traveling
/// at a constant speed, or during a constant course over ground.
///
///@{

typedef uint8_t mip_filter_heading_source_command_source;
static const mip_filter_heading_source_command_source MIP_FILTER_HEADING_SOURCE_COMMAND_SOURCE_NONE                          = 0; ///<  See note 3
static const mip_filter_heading_source_command_source MIP_FILTER_HEADING_SOURCE_COMMAND_SOURCE_MAG                           = 1; ///<  
static const mip_filter_heading_source_command_source MIP_FILTER_HEADING_SOURCE_COMMAND_SOURCE_GNSS_VEL                      = 2; ///<  See notes 1,2
static const mip_filter_heading_source_command_source MIP_FILTER_HEADING_SOURCE_COMMAND_SOURCE_EXTERNAL                      = 3; ///<  
static const mip_filter_heading_source_command_source MIP_FILTER_HEADING_SOURCE_COMMAND_SOURCE_GNSS_VEL_AND_MAG              = 4; ///<  
static const mip_filter_heading_source_command_source MIP_FILTER_HEADING_SOURCE_COMMAND_SOURCE_GNSS_VEL_AND_EXTERNAL         = 5; ///<  
static const mip_filter_heading_source_command_source MIP_FILTER_HEADING_SOURCE_COMMAND_SOURCE_MAG_AND_EXTERNAL              = 6; ///<  
static const mip_filter_heading_source_command_source MIP_FILTER_HEADING_SOURCE_COMMAND_SOURCE_GNSS_VEL_AND_MAG_AND_EXTERNAL = 7; ///<  

struct mip_filter_heading_source_command
{
    mip_function_selector function;
    mip_filter_heading_source_command_source source;
    
};
typedef struct mip_filter_heading_source_command mip_filter_heading_source_command;
void insert_mip_filter_heading_source_command(struct mip_serializer* serializer, const mip_filter_heading_source_command* self);
void extract_mip_filter_heading_source_command(struct mip_serializer* serializer, mip_filter_heading_source_command* self);

void insert_mip_filter_heading_source_command_source(struct mip_serializer* serializer, const mip_filter_heading_source_command_source self);
void extract_mip_filter_heading_source_command_source(struct mip_serializer* serializer, mip_filter_heading_source_command_source* self);

struct mip_filter_heading_source_response
{
    mip_filter_heading_source_command_source source;
    
};
typedef struct mip_filter_heading_source_response mip_filter_heading_source_response;
void insert_mip_filter_heading_source_response(struct mip_serializer* serializer, const mip_filter_heading_source_response* self);
void extract_mip_filter_heading_source_response(struct mip_serializer* serializer, mip_filter_heading_source_response* self);

mip_cmd_result mip_filter_write_heading_source(struct mip_interface* device, mip_filter_heading_source_command_source source);
mip_cmd_result mip_filter_read_heading_source(struct mip_interface* device, mip_filter_heading_source_command_source* source_out);
mip_cmd_result mip_filter_save_heading_source(struct mip_interface* device);
mip_cmd_result mip_filter_load_heading_source(struct mip_interface* device);
mip_cmd_result mip_filter_default_heading_source(struct mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_auto_init_control  (0x0D,0x19) Auto Init Control [C]
/// Filter Auto-initialization Control
/// 
/// Enable/Disable automatic initialization upon device startup.
/// 
/// Possible enable values:
/// 
/// 0x00 - Disable auto-initialization
/// 0x01 - Enable auto-initialization
/// 
///
///@{

struct mip_filter_auto_init_control_command
{
    mip_function_selector function;
    uint8_t enable; ///< See above
    
};
typedef struct mip_filter_auto_init_control_command mip_filter_auto_init_control_command;
void insert_mip_filter_auto_init_control_command(struct mip_serializer* serializer, const mip_filter_auto_init_control_command* self);
void extract_mip_filter_auto_init_control_command(struct mip_serializer* serializer, mip_filter_auto_init_control_command* self);

struct mip_filter_auto_init_control_response
{
    uint8_t enable; ///< See above
    
};
typedef struct mip_filter_auto_init_control_response mip_filter_auto_init_control_response;
void insert_mip_filter_auto_init_control_response(struct mip_serializer* serializer, const mip_filter_auto_init_control_response* self);
void extract_mip_filter_auto_init_control_response(struct mip_serializer* serializer, mip_filter_auto_init_control_response* self);

mip_cmd_result mip_filter_write_auto_init_control(struct mip_interface* device, uint8_t enable);
mip_cmd_result mip_filter_read_auto_init_control(struct mip_interface* device, uint8_t* enable_out);
mip_cmd_result mip_filter_save_auto_init_control(struct mip_interface* device);
mip_cmd_result mip_filter_load_auto_init_control(struct mip_interface* device);
mip_cmd_result mip_filter_default_auto_init_control(struct mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_accel_noise  (0x0D,0x1A) Accel Noise [C]
/// Accelerometer Noise Standard Deviation
/// 
/// Each of the noise values must be greater than 0.0.
/// 
/// The noise value represents process noise in the Estimation Filter.
/// Changing this value modifies how the filter responds to dynamic input and can be used to tune the performance of the filter.
/// Default values provide good performance for most laboratory conditions.
///
///@{

struct mip_filter_accel_noise_command
{
    mip_function_selector function;
    mip_vector3f noise; ///< Accel Noise 1-sigma [meters/second^2]
    
};
typedef struct mip_filter_accel_noise_command mip_filter_accel_noise_command;
void insert_mip_filter_accel_noise_command(struct mip_serializer* serializer, const mip_filter_accel_noise_command* self);
void extract_mip_filter_accel_noise_command(struct mip_serializer* serializer, mip_filter_accel_noise_command* self);

struct mip_filter_accel_noise_response
{
    mip_vector3f noise; ///< Accel Noise 1-sigma [meters/second^2]
    
};
typedef struct mip_filter_accel_noise_response mip_filter_accel_noise_response;
void insert_mip_filter_accel_noise_response(struct mip_serializer* serializer, const mip_filter_accel_noise_response* self);
void extract_mip_filter_accel_noise_response(struct mip_serializer* serializer, mip_filter_accel_noise_response* self);

mip_cmd_result mip_filter_write_accel_noise(struct mip_interface* device, const float* noise);
mip_cmd_result mip_filter_read_accel_noise(struct mip_interface* device, float* noise_out);
mip_cmd_result mip_filter_save_accel_noise(struct mip_interface* device);
mip_cmd_result mip_filter_load_accel_noise(struct mip_interface* device);
mip_cmd_result mip_filter_default_accel_noise(struct mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_gyro_noise  (0x0D,0x1B) Gyro Noise [C]
/// Gyroscope Noise Standard Deviation
/// 
/// Each of the noise values must be greater than 0.0
/// 
/// The noise value represents process noise in the Estimation Filter.
/// Changing this value modifies how the filter responds to dynamic input and can be used to tune the performance of the filter.
/// Default values provide good performance for most laboratory conditions.
///
///@{

struct mip_filter_gyro_noise_command
{
    mip_function_selector function;
    mip_vector3f noise; ///< Gyro Noise 1-sigma [rad/second]
    
};
typedef struct mip_filter_gyro_noise_command mip_filter_gyro_noise_command;
void insert_mip_filter_gyro_noise_command(struct mip_serializer* serializer, const mip_filter_gyro_noise_command* self);
void extract_mip_filter_gyro_noise_command(struct mip_serializer* serializer, mip_filter_gyro_noise_command* self);

struct mip_filter_gyro_noise_response
{
    mip_vector3f noise; ///< Gyro Noise 1-sigma [rad/second]
    
};
typedef struct mip_filter_gyro_noise_response mip_filter_gyro_noise_response;
void insert_mip_filter_gyro_noise_response(struct mip_serializer* serializer, const mip_filter_gyro_noise_response* self);
void extract_mip_filter_gyro_noise_response(struct mip_serializer* serializer, mip_filter_gyro_noise_response* self);

mip_cmd_result mip_filter_write_gyro_noise(struct mip_interface* device, const float* noise);
mip_cmd_result mip_filter_read_gyro_noise(struct mip_interface* device, float* noise_out);
mip_cmd_result mip_filter_save_gyro_noise(struct mip_interface* device);
mip_cmd_result mip_filter_load_gyro_noise(struct mip_interface* device);
mip_cmd_result mip_filter_default_gyro_noise(struct mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_accel_bias_model  (0x0D,0x1C) Accel Bias Model [C]
/// Accelerometer Bias Model Parameters
/// 
/// Noise values must be greater than 0.0
/// 
///
///@{

struct mip_filter_accel_bias_model_command
{
    mip_function_selector function;
    mip_vector3f beta; ///< Accel Bias Beta [1/second]
    mip_vector3f noise; ///< Accel Noise 1-sigma [meters/second^2]
    
};
typedef struct mip_filter_accel_bias_model_command mip_filter_accel_bias_model_command;
void insert_mip_filter_accel_bias_model_command(struct mip_serializer* serializer, const mip_filter_accel_bias_model_command* self);
void extract_mip_filter_accel_bias_model_command(struct mip_serializer* serializer, mip_filter_accel_bias_model_command* self);

struct mip_filter_accel_bias_model_response
{
    mip_vector3f beta; ///< Accel Bias Beta [1/second]
    mip_vector3f noise; ///< Accel Noise 1-sigma [meters/second^2]
    
};
typedef struct mip_filter_accel_bias_model_response mip_filter_accel_bias_model_response;
void insert_mip_filter_accel_bias_model_response(struct mip_serializer* serializer, const mip_filter_accel_bias_model_response* self);
void extract_mip_filter_accel_bias_model_response(struct mip_serializer* serializer, mip_filter_accel_bias_model_response* self);

mip_cmd_result mip_filter_write_accel_bias_model(struct mip_interface* device, const float* beta, const float* noise);
mip_cmd_result mip_filter_read_accel_bias_model(struct mip_interface* device, float* beta_out, float* noise_out);
mip_cmd_result mip_filter_save_accel_bias_model(struct mip_interface* device);
mip_cmd_result mip_filter_load_accel_bias_model(struct mip_interface* device);
mip_cmd_result mip_filter_default_accel_bias_model(struct mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_gyro_bias_model  (0x0D,0x1D) Gyro Bias Model [C]
/// Gyroscope Bias Model Parameters
/// 
/// Noise values must be greater than 0.0
/// 
///
///@{

struct mip_filter_gyro_bias_model_command
{
    mip_function_selector function;
    mip_vector3f beta; ///< Gyro Bias Beta [1/second]
    mip_vector3f noise; ///< Gyro Noise 1-sigma [rad/second]
    
};
typedef struct mip_filter_gyro_bias_model_command mip_filter_gyro_bias_model_command;
void insert_mip_filter_gyro_bias_model_command(struct mip_serializer* serializer, const mip_filter_gyro_bias_model_command* self);
void extract_mip_filter_gyro_bias_model_command(struct mip_serializer* serializer, mip_filter_gyro_bias_model_command* self);

struct mip_filter_gyro_bias_model_response
{
    mip_vector3f beta; ///< Gyro Bias Beta [1/second]
    mip_vector3f noise; ///< Gyro Noise 1-sigma [rad/second]
    
};
typedef struct mip_filter_gyro_bias_model_response mip_filter_gyro_bias_model_response;
void insert_mip_filter_gyro_bias_model_response(struct mip_serializer* serializer, const mip_filter_gyro_bias_model_response* self);
void extract_mip_filter_gyro_bias_model_response(struct mip_serializer* serializer, mip_filter_gyro_bias_model_response* self);

mip_cmd_result mip_filter_write_gyro_bias_model(struct mip_interface* device, const float* beta, const float* noise);
mip_cmd_result mip_filter_read_gyro_bias_model(struct mip_interface* device, float* beta_out, float* noise_out);
mip_cmd_result mip_filter_save_gyro_bias_model(struct mip_interface* device);
mip_cmd_result mip_filter_load_gyro_bias_model(struct mip_interface* device);
mip_cmd_result mip_filter_default_gyro_bias_model(struct mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_altitude_aiding  (0x0D,0x47) Altitude Aiding [C]
/// Select altitude input for absolute altitude and/or vertical velocity. The primary altitude reading is always GNSS.
/// Aiding inputs are used to improve GNSS altitude readings when GNSS is available and to backup GNSS during outages.
/// 
/// Pressure altitude is based on "instant sea level pressure" which is dependent on location and weather conditions and can vary by more than 40 meters.
/// 
///
///@{

typedef uint8_t mip_filter_altitude_aiding_command_aiding_selector;
static const mip_filter_altitude_aiding_command_aiding_selector MIP_FILTER_ALTITUDE_AIDING_COMMAND_AIDING_SELECTOR_NONE    = 0; ///<  No altitude aiding
static const mip_filter_altitude_aiding_command_aiding_selector MIP_FILTER_ALTITUDE_AIDING_COMMAND_AIDING_SELECTOR_PRESURE = 1; ///<  Enable pressure sensor aiding

struct mip_filter_altitude_aiding_command
{
    mip_function_selector function;
    mip_filter_altitude_aiding_command_aiding_selector selector; ///< See above
    
};
typedef struct mip_filter_altitude_aiding_command mip_filter_altitude_aiding_command;
void insert_mip_filter_altitude_aiding_command(struct mip_serializer* serializer, const mip_filter_altitude_aiding_command* self);
void extract_mip_filter_altitude_aiding_command(struct mip_serializer* serializer, mip_filter_altitude_aiding_command* self);

void insert_mip_filter_altitude_aiding_command_aiding_selector(struct mip_serializer* serializer, const mip_filter_altitude_aiding_command_aiding_selector self);
void extract_mip_filter_altitude_aiding_command_aiding_selector(struct mip_serializer* serializer, mip_filter_altitude_aiding_command_aiding_selector* self);

struct mip_filter_altitude_aiding_response
{
    mip_filter_altitude_aiding_command_aiding_selector selector; ///< See above
    
};
typedef struct mip_filter_altitude_aiding_response mip_filter_altitude_aiding_response;
void insert_mip_filter_altitude_aiding_response(struct mip_serializer* serializer, const mip_filter_altitude_aiding_response* self);
void extract_mip_filter_altitude_aiding_response(struct mip_serializer* serializer, mip_filter_altitude_aiding_response* self);

mip_cmd_result mip_filter_write_altitude_aiding(struct mip_interface* device, mip_filter_altitude_aiding_command_aiding_selector selector);
mip_cmd_result mip_filter_read_altitude_aiding(struct mip_interface* device, mip_filter_altitude_aiding_command_aiding_selector* selector_out);
mip_cmd_result mip_filter_save_altitude_aiding(struct mip_interface* device);
mip_cmd_result mip_filter_load_altitude_aiding(struct mip_interface* device);
mip_cmd_result mip_filter_default_altitude_aiding(struct mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_pitch_roll_aiding  (0x0D,0x4B) Pitch Roll Aiding [C]
/// Select pitch/roll aiding input. Pitch/roll reading is always derived from GNSS corrected inertial solution.
/// Aiding inputs are used to improve that solution during periods of low dynamics and GNSS outages.
///
///@{

typedef uint8_t mip_filter_pitch_roll_aiding_command_aiding_source;
static const mip_filter_pitch_roll_aiding_command_aiding_source MIP_FILTER_PITCH_ROLL_AIDING_COMMAND_AIDING_SOURCE_NONE        = 0; ///<  No pitch/roll aiding
static const mip_filter_pitch_roll_aiding_command_aiding_source MIP_FILTER_PITCH_ROLL_AIDING_COMMAND_AIDING_SOURCE_GRAVITY_VEC = 1; ///<  Enable gravity vector aiding

struct mip_filter_pitch_roll_aiding_command
{
    mip_function_selector function;
    mip_filter_pitch_roll_aiding_command_aiding_source source; ///< Controls the aiding source
    
};
typedef struct mip_filter_pitch_roll_aiding_command mip_filter_pitch_roll_aiding_command;
void insert_mip_filter_pitch_roll_aiding_command(struct mip_serializer* serializer, const mip_filter_pitch_roll_aiding_command* self);
void extract_mip_filter_pitch_roll_aiding_command(struct mip_serializer* serializer, mip_filter_pitch_roll_aiding_command* self);

void insert_mip_filter_pitch_roll_aiding_command_aiding_source(struct mip_serializer* serializer, const mip_filter_pitch_roll_aiding_command_aiding_source self);
void extract_mip_filter_pitch_roll_aiding_command_aiding_source(struct mip_serializer* serializer, mip_filter_pitch_roll_aiding_command_aiding_source* self);

struct mip_filter_pitch_roll_aiding_response
{
    mip_filter_pitch_roll_aiding_command_aiding_source source; ///< Controls the aiding source
    
};
typedef struct mip_filter_pitch_roll_aiding_response mip_filter_pitch_roll_aiding_response;
void insert_mip_filter_pitch_roll_aiding_response(struct mip_serializer* serializer, const mip_filter_pitch_roll_aiding_response* self);
void extract_mip_filter_pitch_roll_aiding_response(struct mip_serializer* serializer, mip_filter_pitch_roll_aiding_response* self);

mip_cmd_result mip_filter_write_pitch_roll_aiding(struct mip_interface* device, mip_filter_pitch_roll_aiding_command_aiding_source source);
mip_cmd_result mip_filter_read_pitch_roll_aiding(struct mip_interface* device, mip_filter_pitch_roll_aiding_command_aiding_source* source_out);
mip_cmd_result mip_filter_save_pitch_roll_aiding(struct mip_interface* device);
mip_cmd_result mip_filter_load_pitch_roll_aiding(struct mip_interface* device);
mip_cmd_result mip_filter_default_pitch_roll_aiding(struct mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_auto_zupt  (0x0D,0x1E) Auto Zupt [C]
/// The ZUPT is triggered when the scalar magnitude of the GNSS reported velocity vector is equal-to or less than the threshold value.
/// The device will NACK threshold values that are less than zero (i.e.negative.)
///
///@{

struct mip_filter_auto_zupt_command
{
    mip_function_selector function;
    uint8_t enable; ///< 0 - Disable, 1 - Enable
    float threshold; ///< [meters/second]
    
};
typedef struct mip_filter_auto_zupt_command mip_filter_auto_zupt_command;
void insert_mip_filter_auto_zupt_command(struct mip_serializer* serializer, const mip_filter_auto_zupt_command* self);
void extract_mip_filter_auto_zupt_command(struct mip_serializer* serializer, mip_filter_auto_zupt_command* self);

struct mip_filter_auto_zupt_response
{
    uint8_t enable; ///< 0 - Disable, 1 - Enable
    float threshold; ///< [meters/second]
    
};
typedef struct mip_filter_auto_zupt_response mip_filter_auto_zupt_response;
void insert_mip_filter_auto_zupt_response(struct mip_serializer* serializer, const mip_filter_auto_zupt_response* self);
void extract_mip_filter_auto_zupt_response(struct mip_serializer* serializer, mip_filter_auto_zupt_response* self);

mip_cmd_result mip_filter_write_auto_zupt(struct mip_interface* device, uint8_t enable, float threshold);
mip_cmd_result mip_filter_read_auto_zupt(struct mip_interface* device, uint8_t* enable_out, float* threshold_out);
mip_cmd_result mip_filter_save_auto_zupt(struct mip_interface* device);
mip_cmd_result mip_filter_load_auto_zupt(struct mip_interface* device);
mip_cmd_result mip_filter_default_auto_zupt(struct mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_auto_angular_zupt  (0x0D,0x20) Auto Angular Zupt [C]
/// Zero Angular Rate Update
/// The ZUPT is triggered when the scalar magnitude of the angular rate vector is equal-to or less than the threshold value.
/// The device will NACK threshold values that are less than zero (i.e.negative.)
///
///@{

struct mip_filter_auto_angular_zupt_command
{
    mip_function_selector function;
    uint8_t enable; ///< 0 - Disable, 1 - Enable
    float threshold; ///< [radians/second]
    
};
typedef struct mip_filter_auto_angular_zupt_command mip_filter_auto_angular_zupt_command;
void insert_mip_filter_auto_angular_zupt_command(struct mip_serializer* serializer, const mip_filter_auto_angular_zupt_command* self);
void extract_mip_filter_auto_angular_zupt_command(struct mip_serializer* serializer, mip_filter_auto_angular_zupt_command* self);

struct mip_filter_auto_angular_zupt_response
{
    uint8_t enable; ///< 0 - Disable, 1 - Enable
    float threshold; ///< [radians/second]
    
};
typedef struct mip_filter_auto_angular_zupt_response mip_filter_auto_angular_zupt_response;
void insert_mip_filter_auto_angular_zupt_response(struct mip_serializer* serializer, const mip_filter_auto_angular_zupt_response* self);
void extract_mip_filter_auto_angular_zupt_response(struct mip_serializer* serializer, mip_filter_auto_angular_zupt_response* self);

mip_cmd_result mip_filter_write_auto_angular_zupt(struct mip_interface* device, uint8_t enable, float threshold);
mip_cmd_result mip_filter_read_auto_angular_zupt(struct mip_interface* device, uint8_t* enable_out, float* threshold_out);
mip_cmd_result mip_filter_save_auto_angular_zupt(struct mip_interface* device);
mip_cmd_result mip_filter_load_auto_angular_zupt(struct mip_interface* device);
mip_cmd_result mip_filter_default_auto_angular_zupt(struct mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_commanded_zupt  (0x0D,0x22) Commanded Zupt [C]
/// Please see the device user manual for the maximum rate of this message.
///
///@{

mip_cmd_result mip_filter_commanded_zupt(struct mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_commanded_angular_zupt  (0x0D,0x23) Commanded Angular Zupt [C]
/// Please see the device user manual for the maximum rate of this message.
///
///@{

mip_cmd_result mip_filter_commanded_angular_zupt(struct mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_mag_capture_auto_cal  (0x0D,0x27) Mag Capture Auto Cal [C]
/// This command captures the current value of the auto-calibration, applies it to the current fixed hard and soft iron calibration coefficients, and replaces the current fixed hard and soft iron calibration coefficients with the new values.
/// This may be used in place of (or in addition to) a manual hard and soft iron calibration utility. This command also resets the auto-calibration coefficients.
/// Function selector SAVE is the same as issuing the 0x0C, 0x3A and 0x0C, 0x3B commands with the SAVE function selector.
///
///@{

struct mip_filter_mag_capture_auto_cal_command
{
    mip_function_selector function;
    
};
typedef struct mip_filter_mag_capture_auto_cal_command mip_filter_mag_capture_auto_cal_command;
void insert_mip_filter_mag_capture_auto_cal_command(struct mip_serializer* serializer, const mip_filter_mag_capture_auto_cal_command* self);
void extract_mip_filter_mag_capture_auto_cal_command(struct mip_serializer* serializer, mip_filter_mag_capture_auto_cal_command* self);

mip_cmd_result mip_filter_write_mag_capture_auto_cal(struct mip_interface* device);
mip_cmd_result mip_filter_save_mag_capture_auto_cal(struct mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_gravity_noise  (0x0D,0x28) Gravity Noise [C]
/// Set the expected gravity noise 1-sigma values. This function can be used to tune the filter performance in the target application.
/// 
/// Note: Noise values must be greater than 0.0
/// 
/// The noise value represents process noise in the Estimation Filter. Changing this value modifies how the filter responds to dynamic input and can be used to tune filter performance.
/// Default values provide good performance for most laboratory conditions.
///
///@{

struct mip_filter_gravity_noise_command
{
    mip_function_selector function;
    mip_vector3f noise; ///< Gravity Noise 1-sigma [gauss]
    
};
typedef struct mip_filter_gravity_noise_command mip_filter_gravity_noise_command;
void insert_mip_filter_gravity_noise_command(struct mip_serializer* serializer, const mip_filter_gravity_noise_command* self);
void extract_mip_filter_gravity_noise_command(struct mip_serializer* serializer, mip_filter_gravity_noise_command* self);

struct mip_filter_gravity_noise_response
{
    mip_vector3f noise; ///< Gravity Noise 1-sigma [gauss]
    
};
typedef struct mip_filter_gravity_noise_response mip_filter_gravity_noise_response;
void insert_mip_filter_gravity_noise_response(struct mip_serializer* serializer, const mip_filter_gravity_noise_response* self);
void extract_mip_filter_gravity_noise_response(struct mip_serializer* serializer, mip_filter_gravity_noise_response* self);

mip_cmd_result mip_filter_write_gravity_noise(struct mip_interface* device, const float* noise);
mip_cmd_result mip_filter_read_gravity_noise(struct mip_interface* device, float* noise_out);
mip_cmd_result mip_filter_save_gravity_noise(struct mip_interface* device);
mip_cmd_result mip_filter_load_gravity_noise(struct mip_interface* device);
mip_cmd_result mip_filter_default_gravity_noise(struct mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_pressure_altitude_noise  (0x0D,0x29) Pressure Altitude Noise [C]
/// Set the expected pressure altitude noise 1-sigma values. This function can be used to tune the filter performance in the target application.
/// 
/// The noise value must be greater than 0.0
/// 
/// This noise value represents pressure altitude model noise in the Estimation Filter.
/// A lower value will increase responsiveness of the sensor to pressure changes, however height estimates will be more susceptible to error from air pressure fluctuations not due to changes in altitude. Default values provide good performance for most laboratory conditions.
///
///@{

struct mip_filter_pressure_altitude_noise_command
{
    mip_function_selector function;
    float noise; ///< Pressure Altitude Noise 1-sigma [m]
    
};
typedef struct mip_filter_pressure_altitude_noise_command mip_filter_pressure_altitude_noise_command;
void insert_mip_filter_pressure_altitude_noise_command(struct mip_serializer* serializer, const mip_filter_pressure_altitude_noise_command* self);
void extract_mip_filter_pressure_altitude_noise_command(struct mip_serializer* serializer, mip_filter_pressure_altitude_noise_command* self);

struct mip_filter_pressure_altitude_noise_response
{
    float noise; ///< Pressure Altitude Noise 1-sigma [m]
    
};
typedef struct mip_filter_pressure_altitude_noise_response mip_filter_pressure_altitude_noise_response;
void insert_mip_filter_pressure_altitude_noise_response(struct mip_serializer* serializer, const mip_filter_pressure_altitude_noise_response* self);
void extract_mip_filter_pressure_altitude_noise_response(struct mip_serializer* serializer, mip_filter_pressure_altitude_noise_response* self);

mip_cmd_result mip_filter_write_pressure_altitude_noise(struct mip_interface* device, float noise);
mip_cmd_result mip_filter_read_pressure_altitude_noise(struct mip_interface* device, float* noise_out);
mip_cmd_result mip_filter_save_pressure_altitude_noise(struct mip_interface* device);
mip_cmd_result mip_filter_load_pressure_altitude_noise(struct mip_interface* device);
mip_cmd_result mip_filter_default_pressure_altitude_noise(struct mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_hard_iron_offset_noise  (0x0D,0x2B) Hard Iron Offset Noise [C]
/// Set the expected hard iron offset noise 1-sigma values. This function can be used to tune the filter performance in the target application.
/// 
/// This function can be used to tune the filter performance in the target application.
/// 
/// Noise values must be greater than 0.0
/// 
/// The noise values represent process noise in the Estimation Filter.
/// Changing this value modifies how the filter responds to dynamic input and can be used to tune the performance of the filter. Default values provide good performance for most laboratory conditions.
///
///@{

struct mip_filter_hard_iron_offset_noise_command
{
    mip_function_selector function;
    mip_vector3f noise; ///< Hard Iron Offset Noise 1-sigma [gauss]
    
};
typedef struct mip_filter_hard_iron_offset_noise_command mip_filter_hard_iron_offset_noise_command;
void insert_mip_filter_hard_iron_offset_noise_command(struct mip_serializer* serializer, const mip_filter_hard_iron_offset_noise_command* self);
void extract_mip_filter_hard_iron_offset_noise_command(struct mip_serializer* serializer, mip_filter_hard_iron_offset_noise_command* self);

struct mip_filter_hard_iron_offset_noise_response
{
    mip_vector3f noise; ///< Hard Iron Offset Noise 1-sigma [gauss]
    
};
typedef struct mip_filter_hard_iron_offset_noise_response mip_filter_hard_iron_offset_noise_response;
void insert_mip_filter_hard_iron_offset_noise_response(struct mip_serializer* serializer, const mip_filter_hard_iron_offset_noise_response* self);
void extract_mip_filter_hard_iron_offset_noise_response(struct mip_serializer* serializer, mip_filter_hard_iron_offset_noise_response* self);

mip_cmd_result mip_filter_write_hard_iron_offset_noise(struct mip_interface* device, const float* noise);
mip_cmd_result mip_filter_read_hard_iron_offset_noise(struct mip_interface* device, float* noise_out);
mip_cmd_result mip_filter_save_hard_iron_offset_noise(struct mip_interface* device);
mip_cmd_result mip_filter_load_hard_iron_offset_noise(struct mip_interface* device);
mip_cmd_result mip_filter_default_hard_iron_offset_noise(struct mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_soft_iron_matrix_noise  (0x0D,0x2C) Soft Iron Matrix Noise [C]
/// Set the expected soft iron matrix noise 1-sigma values.
/// This function can be used to tune the filter performance in the target application.
/// 
/// Noise values must be greater than 0.0
/// 
/// The noise value represents process noise in the Estimation Filter.
/// Changing this value modifies how the filter responds to dynamic input and can be used to tune the performance of the filter. Default values provide good performance for most laboratory conditions.
///
///@{

struct mip_filter_soft_iron_matrix_noise_command
{
    mip_function_selector function;
    mip_matrix3f noise; ///< Soft Iron Matrix Noise 1-sigma [dimensionless]
    
};
typedef struct mip_filter_soft_iron_matrix_noise_command mip_filter_soft_iron_matrix_noise_command;
void insert_mip_filter_soft_iron_matrix_noise_command(struct mip_serializer* serializer, const mip_filter_soft_iron_matrix_noise_command* self);
void extract_mip_filter_soft_iron_matrix_noise_command(struct mip_serializer* serializer, mip_filter_soft_iron_matrix_noise_command* self);

struct mip_filter_soft_iron_matrix_noise_response
{
    mip_matrix3f noise; ///< Soft Iron Matrix Noise 1-sigma [dimensionless]
    
};
typedef struct mip_filter_soft_iron_matrix_noise_response mip_filter_soft_iron_matrix_noise_response;
void insert_mip_filter_soft_iron_matrix_noise_response(struct mip_serializer* serializer, const mip_filter_soft_iron_matrix_noise_response* self);
void extract_mip_filter_soft_iron_matrix_noise_response(struct mip_serializer* serializer, mip_filter_soft_iron_matrix_noise_response* self);

mip_cmd_result mip_filter_write_soft_iron_matrix_noise(struct mip_interface* device, const float* noise);
mip_cmd_result mip_filter_read_soft_iron_matrix_noise(struct mip_interface* device, float* noise_out);
mip_cmd_result mip_filter_save_soft_iron_matrix_noise(struct mip_interface* device);
mip_cmd_result mip_filter_load_soft_iron_matrix_noise(struct mip_interface* device);
mip_cmd_result mip_filter_default_soft_iron_matrix_noise(struct mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_mag_noise  (0x0D,0x42) Mag Noise [C]
/// Set the expected magnetometer noise 1-sigma values.
/// This function can be used to tune the filter performance in the target application.
/// 
/// Noise values must be greater than 0.0 (gauss)
/// 
/// The noise value represents process noise in the Estimation Filter.
/// Changing this value modifies how the filter responds to dynamic input and can be used to tune the performance of the filter. Default values provide good performance for most laboratory conditions
///
///@{

struct mip_filter_mag_noise_command
{
    mip_function_selector function;
    mip_vector3f noise; ///< Mag Noise 1-sigma [gauss]
    
};
typedef struct mip_filter_mag_noise_command mip_filter_mag_noise_command;
void insert_mip_filter_mag_noise_command(struct mip_serializer* serializer, const mip_filter_mag_noise_command* self);
void extract_mip_filter_mag_noise_command(struct mip_serializer* serializer, mip_filter_mag_noise_command* self);

struct mip_filter_mag_noise_response
{
    mip_vector3f noise; ///< Mag Noise 1-sigma [gauss]
    
};
typedef struct mip_filter_mag_noise_response mip_filter_mag_noise_response;
void insert_mip_filter_mag_noise_response(struct mip_serializer* serializer, const mip_filter_mag_noise_response* self);
void extract_mip_filter_mag_noise_response(struct mip_serializer* serializer, mip_filter_mag_noise_response* self);

mip_cmd_result mip_filter_write_mag_noise(struct mip_interface* device, const float* noise);
mip_cmd_result mip_filter_read_mag_noise(struct mip_interface* device, float* noise_out);
mip_cmd_result mip_filter_save_mag_noise(struct mip_interface* device);
mip_cmd_result mip_filter_load_mag_noise(struct mip_interface* device);
mip_cmd_result mip_filter_default_mag_noise(struct mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_inclination_source  (0x0D,0x4C) Inclination Source [C]
/// Set/Get the local magnetic field inclination angle source.
/// 
/// This can be used to correct for the local value of inclination (dip angle) of the earthmagnetic field.
/// Having a correct value is important for best performance of the auto-mag calibration feature. If you do not have an accurate inclination angle source, it is recommended that you leave the auto-mag calibration feature off.
/// 
///
///@{

struct mip_filter_inclination_source_command
{
    mip_function_selector function;
    mip_filter_mag_param_source source; ///< Inclination Source
    float inclination; ///< Inclination angle [radians] (only required if source = MANUAL)
    
};
typedef struct mip_filter_inclination_source_command mip_filter_inclination_source_command;
void insert_mip_filter_inclination_source_command(struct mip_serializer* serializer, const mip_filter_inclination_source_command* self);
void extract_mip_filter_inclination_source_command(struct mip_serializer* serializer, mip_filter_inclination_source_command* self);

struct mip_filter_inclination_source_response
{
    mip_filter_mag_param_source source; ///< Inclination Source
    float inclination; ///< Inclination angle [radians] (only required if source = MANUAL)
    
};
typedef struct mip_filter_inclination_source_response mip_filter_inclination_source_response;
void insert_mip_filter_inclination_source_response(struct mip_serializer* serializer, const mip_filter_inclination_source_response* self);
void extract_mip_filter_inclination_source_response(struct mip_serializer* serializer, mip_filter_inclination_source_response* self);

mip_cmd_result mip_filter_write_inclination_source(struct mip_interface* device, mip_filter_mag_param_source source, float inclination);
mip_cmd_result mip_filter_read_inclination_source(struct mip_interface* device, mip_filter_mag_param_source* source_out, float* inclination_out);
mip_cmd_result mip_filter_save_inclination_source(struct mip_interface* device);
mip_cmd_result mip_filter_load_inclination_source(struct mip_interface* device);
mip_cmd_result mip_filter_default_inclination_source(struct mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_magnetic_declination_source  (0x0D,0x43) Magnetic Declination Source [C]
/// Set/Get the local magnetic field declination angle source.
/// 
/// This can be used to correct for the local value of declination of the earthmagnetic field.
/// Having a correct value is important for best performance of the auto-mag calibration feature. If you do not have an accurate inclination angle source, it is recommended that you leave the auto-mag calibration feature off.
/// 
///
///@{

struct mip_filter_magnetic_declination_source_command
{
    mip_function_selector function;
    mip_filter_mag_param_source source; ///< Magnetic field declination angle source
    float declination; ///< Declination angle [radians] (only required if source = MANUAL)
    
};
typedef struct mip_filter_magnetic_declination_source_command mip_filter_magnetic_declination_source_command;
void insert_mip_filter_magnetic_declination_source_command(struct mip_serializer* serializer, const mip_filter_magnetic_declination_source_command* self);
void extract_mip_filter_magnetic_declination_source_command(struct mip_serializer* serializer, mip_filter_magnetic_declination_source_command* self);

struct mip_filter_magnetic_declination_source_response
{
    mip_filter_mag_param_source source; ///< Magnetic field declination angle source
    float declination; ///< Declination angle [radians] (only required if source = MANUAL)
    
};
typedef struct mip_filter_magnetic_declination_source_response mip_filter_magnetic_declination_source_response;
void insert_mip_filter_magnetic_declination_source_response(struct mip_serializer* serializer, const mip_filter_magnetic_declination_source_response* self);
void extract_mip_filter_magnetic_declination_source_response(struct mip_serializer* serializer, mip_filter_magnetic_declination_source_response* self);

mip_cmd_result mip_filter_write_magnetic_declination_source(struct mip_interface* device, mip_filter_mag_param_source source, float declination);
mip_cmd_result mip_filter_read_magnetic_declination_source(struct mip_interface* device, mip_filter_mag_param_source* source_out, float* declination_out);
mip_cmd_result mip_filter_save_magnetic_declination_source(struct mip_interface* device);
mip_cmd_result mip_filter_load_magnetic_declination_source(struct mip_interface* device);
mip_cmd_result mip_filter_default_magnetic_declination_source(struct mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_mag_field_magnitude_source  (0x0D,0x4D) Mag Field Magnitude Source [C]
/// Set/Get the local magnetic field magnitude source.
/// 
/// This is used to specify the local magnitude of the earth's magnetic field.
/// Having a correct value for magnitude is important for best performance of the auto-mag calibration feature and for the magnetometer adaptive magnitude. If you do not have an accurate value for the local magnetic field magnitude, it is recommended that you leave the auto-mag calibration feature off.
///
///@{

struct mip_filter_mag_field_magnitude_source_command
{
    mip_function_selector function;
    mip_filter_mag_param_source source; ///< Magnetic Field Magnitude Source
    float magnitude; ///< Magnitude [gauss] (only required if source = MANUAL)
    
};
typedef struct mip_filter_mag_field_magnitude_source_command mip_filter_mag_field_magnitude_source_command;
void insert_mip_filter_mag_field_magnitude_source_command(struct mip_serializer* serializer, const mip_filter_mag_field_magnitude_source_command* self);
void extract_mip_filter_mag_field_magnitude_source_command(struct mip_serializer* serializer, mip_filter_mag_field_magnitude_source_command* self);

struct mip_filter_mag_field_magnitude_source_response
{
    mip_filter_mag_param_source source; ///< Magnetic Field Magnitude Source
    float magnitude; ///< Magnitude [gauss] (only required if source = MANUAL)
    
};
typedef struct mip_filter_mag_field_magnitude_source_response mip_filter_mag_field_magnitude_source_response;
void insert_mip_filter_mag_field_magnitude_source_response(struct mip_serializer* serializer, const mip_filter_mag_field_magnitude_source_response* self);
void extract_mip_filter_mag_field_magnitude_source_response(struct mip_serializer* serializer, mip_filter_mag_field_magnitude_source_response* self);

mip_cmd_result mip_filter_write_mag_field_magnitude_source(struct mip_interface* device, mip_filter_mag_param_source source, float magnitude);
mip_cmd_result mip_filter_read_mag_field_magnitude_source(struct mip_interface* device, mip_filter_mag_param_source* source_out, float* magnitude_out);
mip_cmd_result mip_filter_save_mag_field_magnitude_source(struct mip_interface* device);
mip_cmd_result mip_filter_load_mag_field_magnitude_source(struct mip_interface* device);
mip_cmd_result mip_filter_default_mag_field_magnitude_source(struct mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_reference_position  (0x0D,0x26) Reference Position [C]
/// Set the Lat/Long/Alt reference position for the sensor.
/// 
/// This position is used by the sensor to calculate the WGS84 gravity and WMM2015 magnetic field parameters.
/// 
///
///@{

struct mip_filter_reference_position_command
{
    mip_function_selector function;
    bool enable; ///< enable/disable
    double latitude; ///< [degrees]
    double longitude; ///< [degrees]
    double altitude; ///< [meters]
    
};
typedef struct mip_filter_reference_position_command mip_filter_reference_position_command;
void insert_mip_filter_reference_position_command(struct mip_serializer* serializer, const mip_filter_reference_position_command* self);
void extract_mip_filter_reference_position_command(struct mip_serializer* serializer, mip_filter_reference_position_command* self);

struct mip_filter_reference_position_response
{
    bool enable; ///< enable/disable
    double latitude; ///< [degrees]
    double longitude; ///< [degrees]
    double altitude; ///< [meters]
    
};
typedef struct mip_filter_reference_position_response mip_filter_reference_position_response;
void insert_mip_filter_reference_position_response(struct mip_serializer* serializer, const mip_filter_reference_position_response* self);
void extract_mip_filter_reference_position_response(struct mip_serializer* serializer, mip_filter_reference_position_response* self);

mip_cmd_result mip_filter_write_reference_position(struct mip_interface* device, bool enable, double latitude, double longitude, double altitude);
mip_cmd_result mip_filter_read_reference_position(struct mip_interface* device, bool* enable_out, double* latitude_out, double* longitude_out, double* altitude_out);
mip_cmd_result mip_filter_save_reference_position(struct mip_interface* device);
mip_cmd_result mip_filter_load_reference_position(struct mip_interface* device);
mip_cmd_result mip_filter_default_reference_position(struct mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_accel_magnitude_error_adaptive_measurement  (0x0D,0x44) Accel Magnitude Error Adaptive Measurement [C]
/// Enable or disable the gravity magnitude error adaptive measurement.
/// This function can be used to tune the filter performance in the target application
/// 
/// Pick values that give you the least occurrence of invalid EF attitude output.
/// The default values are good for standard low dynamics applications.
/// Increase values for higher dynamic conditions, lower values for lower dynamic.
/// Too low a value will result in excessive heading errors.
/// Higher values increase heading errors when undergoing magnetic field anomalies caused by DC currents, magnets, steel structures,etc.
/// 
/// Adaptive measurements can be enabled/disabled without the need for providing the additional parameters.
/// In this case, only the function selector and enable value are required; all other parameters will remain at their previous values.
/// When “auto-adaptive” is selected, the filter and limit parameters are ignored.
/// Instead, aiding measurements which rely on the gravity vector will be automatically reweighted by the Kalman filter according to the perceived measurement quality.
/// 
///
///@{

struct mip_filter_accel_magnitude_error_adaptive_measurement_command
{
    mip_function_selector function;
    mip_filter_adaptive_measurement adaptive_measurement; ///< Adaptive measurement selector
    float frequency; ///< Low-pass filter curoff frequency [hertz]
    float low_limit; ///< [meters/second^2]
    float high_limit; ///< [meters/second^2]
    float low_limit_uncertainty; ///< 1-Sigma [meters/second^2]
    float high_limit_uncertainty; ///< 1-Sigma [meters/second^2]
    float minimum_uncertainty; ///< 1-Sigma [meters/second^2]
    
};
typedef struct mip_filter_accel_magnitude_error_adaptive_measurement_command mip_filter_accel_magnitude_error_adaptive_measurement_command;
void insert_mip_filter_accel_magnitude_error_adaptive_measurement_command(struct mip_serializer* serializer, const mip_filter_accel_magnitude_error_adaptive_measurement_command* self);
void extract_mip_filter_accel_magnitude_error_adaptive_measurement_command(struct mip_serializer* serializer, mip_filter_accel_magnitude_error_adaptive_measurement_command* self);

struct mip_filter_accel_magnitude_error_adaptive_measurement_response
{
    mip_filter_adaptive_measurement adaptive_measurement; ///< Adaptive measurement selector
    float frequency; ///< Low-pass filter curoff frequency [hertz]
    float low_limit; ///< [meters/second^2]
    float high_limit; ///< [meters/second^2]
    float low_limit_uncertainty; ///< 1-Sigma [meters/second^2]
    float high_limit_uncertainty; ///< 1-Sigma [meters/second^2]
    float minimum_uncertainty; ///< 1-Sigma [meters/second^2]
    
};
typedef struct mip_filter_accel_magnitude_error_adaptive_measurement_response mip_filter_accel_magnitude_error_adaptive_measurement_response;
void insert_mip_filter_accel_magnitude_error_adaptive_measurement_response(struct mip_serializer* serializer, const mip_filter_accel_magnitude_error_adaptive_measurement_response* self);
void extract_mip_filter_accel_magnitude_error_adaptive_measurement_response(struct mip_serializer* serializer, mip_filter_accel_magnitude_error_adaptive_measurement_response* self);

mip_cmd_result mip_filter_write_accel_magnitude_error_adaptive_measurement(struct mip_interface* device, mip_filter_adaptive_measurement adaptive_measurement, float frequency, float low_limit, float high_limit, float low_limit_uncertainty, float high_limit_uncertainty, float minimum_uncertainty);
mip_cmd_result mip_filter_read_accel_magnitude_error_adaptive_measurement(struct mip_interface* device, mip_filter_adaptive_measurement* adaptive_measurement_out, float* frequency_out, float* low_limit_out, float* high_limit_out, float* low_limit_uncertainty_out, float* high_limit_uncertainty_out, float* minimum_uncertainty_out);
mip_cmd_result mip_filter_save_accel_magnitude_error_adaptive_measurement(struct mip_interface* device);
mip_cmd_result mip_filter_load_accel_magnitude_error_adaptive_measurement(struct mip_interface* device);
mip_cmd_result mip_filter_default_accel_magnitude_error_adaptive_measurement(struct mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_mag_magnitude_error_adaptive_measurement  (0x0D,0x45) Mag Magnitude Error Adaptive Measurement [C]
/// Enable or disable the magnetometer magnitude error adaptive measurement.
/// This feature will reject magnetometer readings that are out of range of the thresholds specified (fixed adaptive) or calculated internally (auto-adaptive).
/// 
/// Pick values that give you the least occurrence of invalid EF attitude output.
/// The default values are good for standard low dynamics applications.
/// Increase values for higher dynamic conditions, lower values for lower dynamic.
/// Too low a value will result in excessive heading errors.
/// Higher values increase heading errors when undergoing magnetic field anomalies caused by DC currents, magnets, steel structures,etc.
/// 
///
///@{

struct mip_filter_mag_magnitude_error_adaptive_measurement_command
{
    mip_function_selector function;
    mip_filter_adaptive_measurement adaptive_measurement; ///< Adaptive measurement selector
    float frequency; ///< Low-pass filter curoff frequency [hertz]
    float low_limit; ///< [meters/second^2]
    float high_limit; ///< [meters/second^2]
    float low_limit_uncertainty; ///< 1-Sigma [meters/second^2]
    float high_limit_uncertainty; ///< 1-Sigma [meters/second^2]
    float minimum_uncertainty; ///< 1-Sigma [meters/second^2]
    
};
typedef struct mip_filter_mag_magnitude_error_adaptive_measurement_command mip_filter_mag_magnitude_error_adaptive_measurement_command;
void insert_mip_filter_mag_magnitude_error_adaptive_measurement_command(struct mip_serializer* serializer, const mip_filter_mag_magnitude_error_adaptive_measurement_command* self);
void extract_mip_filter_mag_magnitude_error_adaptive_measurement_command(struct mip_serializer* serializer, mip_filter_mag_magnitude_error_adaptive_measurement_command* self);

struct mip_filter_mag_magnitude_error_adaptive_measurement_response
{
    mip_filter_adaptive_measurement adaptive_measurement; ///< Adaptive measurement selector
    float frequency; ///< Low-pass filter curoff frequency [hertz]
    float low_limit; ///< [meters/second^2]
    float high_limit; ///< [meters/second^2]
    float low_limit_uncertainty; ///< 1-Sigma [meters/second^2]
    float high_limit_uncertainty; ///< 1-Sigma [meters/second^2]
    float minimum_uncertainty; ///< 1-Sigma [meters/second^2]
    
};
typedef struct mip_filter_mag_magnitude_error_adaptive_measurement_response mip_filter_mag_magnitude_error_adaptive_measurement_response;
void insert_mip_filter_mag_magnitude_error_adaptive_measurement_response(struct mip_serializer* serializer, const mip_filter_mag_magnitude_error_adaptive_measurement_response* self);
void extract_mip_filter_mag_magnitude_error_adaptive_measurement_response(struct mip_serializer* serializer, mip_filter_mag_magnitude_error_adaptive_measurement_response* self);

mip_cmd_result mip_filter_write_mag_magnitude_error_adaptive_measurement(struct mip_interface* device, mip_filter_adaptive_measurement adaptive_measurement, float frequency, float low_limit, float high_limit, float low_limit_uncertainty, float high_limit_uncertainty, float minimum_uncertainty);
mip_cmd_result mip_filter_read_mag_magnitude_error_adaptive_measurement(struct mip_interface* device, mip_filter_adaptive_measurement* adaptive_measurement_out, float* frequency_out, float* low_limit_out, float* high_limit_out, float* low_limit_uncertainty_out, float* high_limit_uncertainty_out, float* minimum_uncertainty_out);
mip_cmd_result mip_filter_save_mag_magnitude_error_adaptive_measurement(struct mip_interface* device);
mip_cmd_result mip_filter_load_mag_magnitude_error_adaptive_measurement(struct mip_interface* device);
mip_cmd_result mip_filter_default_mag_magnitude_error_adaptive_measurement(struct mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_mag_dip_angle_error_adaptive_measurement  (0x0D,0x46) Mag Dip Angle Error Adaptive Measurement [C]
/// Enable or disable the magnetometer dip angle error adaptive measurement.
/// This function can be used to tune the filter performance in the target application
/// 
/// Pick values that give you the least occurrence of invalid EF attitude output.
/// The default values are good for standard low dynamics applications.
/// Increase values for higher dynamic conditions, lower values for lower dynamic.
/// Too low a value will result in excessive heading errors.
/// Higher values increase heading errors when undergoing magnetic field anomalies caused by DC currents, magnets, steel structures,etc.
/// 
/// The magnetometer dip angle adaptive measurement is ignored if the auto-adaptive magnetometer magnitude or auto-adaptive accel magnitude options are selected.
/// 
///
///@{

struct mip_filter_mag_dip_angle_error_adaptive_measurement_command
{
    mip_function_selector function;
    bool enable; ///< Enable/Disable
    float frequency; ///< Low-pass filter curoff frequency [hertz]
    float high_limit; ///< [meters/second^2]
    float high_limit_uncertainty; ///< 1-Sigma [meters/second^2]
    float minimum_uncertainty; ///< 1-Sigma [meters/second^2]
    
};
typedef struct mip_filter_mag_dip_angle_error_adaptive_measurement_command mip_filter_mag_dip_angle_error_adaptive_measurement_command;
void insert_mip_filter_mag_dip_angle_error_adaptive_measurement_command(struct mip_serializer* serializer, const mip_filter_mag_dip_angle_error_adaptive_measurement_command* self);
void extract_mip_filter_mag_dip_angle_error_adaptive_measurement_command(struct mip_serializer* serializer, mip_filter_mag_dip_angle_error_adaptive_measurement_command* self);

struct mip_filter_mag_dip_angle_error_adaptive_measurement_response
{
    bool enable; ///< Enable/Disable
    float frequency; ///< Low-pass filter curoff frequency [hertz]
    float high_limit; ///< [meters/second^2]
    float high_limit_uncertainty; ///< 1-Sigma [meters/second^2]
    float minimum_uncertainty; ///< 1-Sigma [meters/second^2]
    
};
typedef struct mip_filter_mag_dip_angle_error_adaptive_measurement_response mip_filter_mag_dip_angle_error_adaptive_measurement_response;
void insert_mip_filter_mag_dip_angle_error_adaptive_measurement_response(struct mip_serializer* serializer, const mip_filter_mag_dip_angle_error_adaptive_measurement_response* self);
void extract_mip_filter_mag_dip_angle_error_adaptive_measurement_response(struct mip_serializer* serializer, mip_filter_mag_dip_angle_error_adaptive_measurement_response* self);

mip_cmd_result mip_filter_write_mag_dip_angle_error_adaptive_measurement(struct mip_interface* device, bool enable, float frequency, float high_limit, float high_limit_uncertainty, float minimum_uncertainty);
mip_cmd_result mip_filter_read_mag_dip_angle_error_adaptive_measurement(struct mip_interface* device, bool* enable_out, float* frequency_out, float* high_limit_out, float* high_limit_uncertainty_out, float* minimum_uncertainty_out);
mip_cmd_result mip_filter_save_mag_dip_angle_error_adaptive_measurement(struct mip_interface* device);
mip_cmd_result mip_filter_load_mag_dip_angle_error_adaptive_measurement(struct mip_interface* device);
mip_cmd_result mip_filter_default_mag_dip_angle_error_adaptive_measurement(struct mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_aiding_measurement_enable  (0x0D,0x50) Aiding Measurement Enable [C]
/// Enables / disables the specified aiding measurement source.
/// 
/// 
///
///@{

typedef uint16_t mip_filter_aiding_measurement_enable_command_aiding_source;
static const mip_filter_aiding_measurement_enable_command_aiding_source MIP_FILTER_AIDING_MEASUREMENT_ENABLE_COMMAND_AIDING_SOURCE_GNSS_POS_VEL          = 0;     ///<  GNSS Position and Velocity
static const mip_filter_aiding_measurement_enable_command_aiding_source MIP_FILTER_AIDING_MEASUREMENT_ENABLE_COMMAND_AIDING_SOURCE_GNSS_HEADING          = 1;     ///<  GNSS Heading (dual antenna)
static const mip_filter_aiding_measurement_enable_command_aiding_source MIP_FILTER_AIDING_MEASUREMENT_ENABLE_COMMAND_AIDING_SOURCE_ALTIMETER             = 2;     ///<  Pressure altimeter (built-in sensor)
static const mip_filter_aiding_measurement_enable_command_aiding_source MIP_FILTER_AIDING_MEASUREMENT_ENABLE_COMMAND_AIDING_SOURCE_SPEED                 = 3;     ///<  Speed sensor / Odometer
static const mip_filter_aiding_measurement_enable_command_aiding_source MIP_FILTER_AIDING_MEASUREMENT_ENABLE_COMMAND_AIDING_SOURCE_MAGNETOMETER          = 4;     ///<  Magnetometer (built-in sensor)
static const mip_filter_aiding_measurement_enable_command_aiding_source MIP_FILTER_AIDING_MEASUREMENT_ENABLE_COMMAND_AIDING_SOURCE_EXTERNAL_HEADING      = 5;     ///<  External heading input
static const mip_filter_aiding_measurement_enable_command_aiding_source MIP_FILTER_AIDING_MEASUREMENT_ENABLE_COMMAND_AIDING_SOURCE_EXTERNAL_ALTIMETER    = 6;     ///<  External pressure altimeter input
static const mip_filter_aiding_measurement_enable_command_aiding_source MIP_FILTER_AIDING_MEASUREMENT_ENABLE_COMMAND_AIDING_SOURCE_EXTERNAL_MAGNETOMETER = 7;     ///<  External magnetomer input
static const mip_filter_aiding_measurement_enable_command_aiding_source MIP_FILTER_AIDING_MEASUREMENT_ENABLE_COMMAND_AIDING_SOURCE_VEHICLE_FRAME_VEL     = 8;     ///<  External vehicle frame velocity input
static const mip_filter_aiding_measurement_enable_command_aiding_source MIP_FILTER_AIDING_MEASUREMENT_ENABLE_COMMAND_AIDING_SOURCE_ALL                   = 65535; ///<  Save/load/reset all options

struct mip_filter_aiding_measurement_enable_command
{
    mip_function_selector function;
    mip_filter_aiding_measurement_enable_command_aiding_source aiding_source; ///< Aiding measurement source
    bool enable; ///< Controls the aiding source
    
};
typedef struct mip_filter_aiding_measurement_enable_command mip_filter_aiding_measurement_enable_command;
void insert_mip_filter_aiding_measurement_enable_command(struct mip_serializer* serializer, const mip_filter_aiding_measurement_enable_command* self);
void extract_mip_filter_aiding_measurement_enable_command(struct mip_serializer* serializer, mip_filter_aiding_measurement_enable_command* self);

void insert_mip_filter_aiding_measurement_enable_command_aiding_source(struct mip_serializer* serializer, const mip_filter_aiding_measurement_enable_command_aiding_source self);
void extract_mip_filter_aiding_measurement_enable_command_aiding_source(struct mip_serializer* serializer, mip_filter_aiding_measurement_enable_command_aiding_source* self);

struct mip_filter_aiding_measurement_enable_response
{
    mip_filter_aiding_measurement_enable_command_aiding_source aiding_source; ///< Aiding measurement source
    bool enable; ///< Controls the aiding source
    
};
typedef struct mip_filter_aiding_measurement_enable_response mip_filter_aiding_measurement_enable_response;
void insert_mip_filter_aiding_measurement_enable_response(struct mip_serializer* serializer, const mip_filter_aiding_measurement_enable_response* self);
void extract_mip_filter_aiding_measurement_enable_response(struct mip_serializer* serializer, mip_filter_aiding_measurement_enable_response* self);

mip_cmd_result mip_filter_write_aiding_measurement_enable(struct mip_interface* device, mip_filter_aiding_measurement_enable_command_aiding_source aiding_source, bool enable);
mip_cmd_result mip_filter_read_aiding_measurement_enable(struct mip_interface* device, mip_filter_aiding_measurement_enable_command_aiding_source aiding_source, bool* enable_out);
mip_cmd_result mip_filter_save_aiding_measurement_enable(struct mip_interface* device, mip_filter_aiding_measurement_enable_command_aiding_source aiding_source);
mip_cmd_result mip_filter_load_aiding_measurement_enable(struct mip_interface* device, mip_filter_aiding_measurement_enable_command_aiding_source aiding_source);
mip_cmd_result mip_filter_default_aiding_measurement_enable(struct mip_interface* device, mip_filter_aiding_measurement_enable_command_aiding_source aiding_source);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_run  (0x0D,0x05) Run [C]
/// Manual run command.
/// 
/// If the initialization configuration has the "wait_for_run_command" option enabled, the filter will wait until it receives this command before commencing integration and enabling the Kalman filter. Prior to the receipt of this command, the filter will remain in the filter initialization mode.
///
///@{

mip_cmd_result mip_filter_run(struct mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_kinematic_constraint  (0x0D,0x51) Kinematic Constraint [C]
/// Controls kinematic constraint model selection for the navigation filter.
/// 
/// See manual for explanation of how the kinematic constraints are applied.
///
///@{

struct mip_filter_kinematic_constraint_command
{
    mip_function_selector function;
    uint8_t acceleration_constraint_selection; ///< Acceleration constraint: <br/> 0=None (default), <br/> 1=Zero-acceleration.
    uint8_t velocity_constraint_selection; ///< 0=None (default), <br/> 1=Zero-velocity, <br/> 2=Wheeled-vehicle. <br/>
    uint8_t angular_constraint_selection; ///< 0=None (default), 1=Zero-angular rate (ZUPT).
    
};
typedef struct mip_filter_kinematic_constraint_command mip_filter_kinematic_constraint_command;
void insert_mip_filter_kinematic_constraint_command(struct mip_serializer* serializer, const mip_filter_kinematic_constraint_command* self);
void extract_mip_filter_kinematic_constraint_command(struct mip_serializer* serializer, mip_filter_kinematic_constraint_command* self);

struct mip_filter_kinematic_constraint_response
{
    uint8_t acceleration_constraint_selection; ///< Acceleration constraint: <br/> 0=None (default), <br/> 1=Zero-acceleration.
    uint8_t velocity_constraint_selection; ///< 0=None (default), <br/> 1=Zero-velocity, <br/> 2=Wheeled-vehicle. <br/>
    uint8_t angular_constraint_selection; ///< 0=None (default), 1=Zero-angular rate (ZUPT).
    
};
typedef struct mip_filter_kinematic_constraint_response mip_filter_kinematic_constraint_response;
void insert_mip_filter_kinematic_constraint_response(struct mip_serializer* serializer, const mip_filter_kinematic_constraint_response* self);
void extract_mip_filter_kinematic_constraint_response(struct mip_serializer* serializer, mip_filter_kinematic_constraint_response* self);

mip_cmd_result mip_filter_write_kinematic_constraint(struct mip_interface* device, uint8_t acceleration_constraint_selection, uint8_t velocity_constraint_selection, uint8_t angular_constraint_selection);
mip_cmd_result mip_filter_read_kinematic_constraint(struct mip_interface* device, uint8_t* acceleration_constraint_selection_out, uint8_t* velocity_constraint_selection_out, uint8_t* angular_constraint_selection_out);
mip_cmd_result mip_filter_save_kinematic_constraint(struct mip_interface* device);
mip_cmd_result mip_filter_load_kinematic_constraint(struct mip_interface* device);
mip_cmd_result mip_filter_default_kinematic_constraint(struct mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_initialization_configuration  (0x0D,0x52) Initialization Configuration [C]
/// Controls the source and values used for initial conditions of the navigation solution.
/// 
/// Notes: Initial conditions are the position, velocity, and attitude of the platform used when the filter starts running or is reset.
/// For the user specified position array, the units are meters if the ECEF frame is selected, and degrees latitude, degrees longitude, and meters above ellipsoid if the latitude/longitude/height frame is selected.
/// For the user specified velocity array, the units are meters per second, but the reference frame depends on the reference frame selector (ECEF or NED).
///
///@{

typedef uint8_t mip_filter_initialization_configuration_command_alignment_selector;
static const mip_filter_initialization_configuration_command_alignment_selector MIP_FILTER_INITIALIZATION_CONFIGURATION_COMMAND_ALIGNMENT_SELECTOR_NONE         = 0x00;
static const mip_filter_initialization_configuration_command_alignment_selector MIP_FILTER_INITIALIZATION_CONFIGURATION_COMMAND_ALIGNMENT_SELECTOR_DUAL_ANTENNA = 0x01; ///<  Dual-antenna GNSS alignment
static const mip_filter_initialization_configuration_command_alignment_selector MIP_FILTER_INITIALIZATION_CONFIGURATION_COMMAND_ALIGNMENT_SELECTOR_KINEMATIC    = 0x02; ///<  GNSS kinematic alignment (GNSS velocity determines initial heading)
static const mip_filter_initialization_configuration_command_alignment_selector MIP_FILTER_INITIALIZATION_CONFIGURATION_COMMAND_ALIGNMENT_SELECTOR_MAGNETOMETER = 0x04; ///<  Magnetometer heading alignment (Internal magnetometer determines initial heading)
static const mip_filter_initialization_configuration_command_alignment_selector MIP_FILTER_INITIALIZATION_CONFIGURATION_COMMAND_ALIGNMENT_SELECTOR_EXTERNAL     = 0x08; ///<  External heading alignment (External heading input determines heading)
static const mip_filter_initialization_configuration_command_alignment_selector MIP_FILTER_INITIALIZATION_CONFIGURATION_COMMAND_ALIGNMENT_SELECTOR_ALL          = 0x0F;

typedef uint8_t mip_filter_initialization_configuration_command_initial_condition_source;
static const mip_filter_initialization_configuration_command_initial_condition_source MIP_FILTER_INITIALIZATION_CONFIGURATION_COMMAND_INITIAL_CONDITION_SOURCE_AUTO_POS_VEL_ATT        = 0; ///<  Automatic position, velocity and attitude
static const mip_filter_initialization_configuration_command_initial_condition_source MIP_FILTER_INITIALIZATION_CONFIGURATION_COMMAND_INITIAL_CONDITION_SOURCE_AUTO_POS_VEL_PITCH_ROLL = 1; ///<  Automatic position and velocity, automatic pitch and roll, and user-specified heading
static const mip_filter_initialization_configuration_command_initial_condition_source MIP_FILTER_INITIALIZATION_CONFIGURATION_COMMAND_INITIAL_CONDITION_SOURCE_AUTO_POS_VEL            = 2; ///<  Automatic position and velocity, with fully user-specified attitude
static const mip_filter_initialization_configuration_command_initial_condition_source MIP_FILTER_INITIALIZATION_CONFIGURATION_COMMAND_INITIAL_CONDITION_SOURCE_MANUAL                  = 3; ///<  User-specified position, velocity, and attitude.

struct mip_filter_initialization_configuration_command
{
    mip_function_selector function;
    uint8_t wait_for_run_command; ///< Initialize filter only after receiving "run" command
    mip_filter_initialization_configuration_command_initial_condition_source initial_cond_src; ///< Initial condition source:
    mip_filter_initialization_configuration_command_alignment_selector auto_heading_alignment_selector; ///< Bitfield specifying the allowed automatic heading alignment methods for automatic initial conditions. Bits are set to 1 to enable, and the correspond to the following: <br/>
    float initial_heading; ///< User-specified initial platform heading (degrees).
    float initial_pitch; ///< User-specified initial platform pitch (degrees)
    float initial_roll; ///< User-specified initial platform roll (degrees)
    mip_vector3f initial_position; ///< User-specified initial platform position (units determined by reference frame selector, see note.)
    mip_vector3f initial_velocity; ///< User-specified initial platform velocity (units determined by reference frame selector, see note.)
    mip_filter_reference_frame reference_frame_selector; ///< User-specified initial position/velocity reference frames
    
};
typedef struct mip_filter_initialization_configuration_command mip_filter_initialization_configuration_command;
void insert_mip_filter_initialization_configuration_command(struct mip_serializer* serializer, const mip_filter_initialization_configuration_command* self);
void extract_mip_filter_initialization_configuration_command(struct mip_serializer* serializer, mip_filter_initialization_configuration_command* self);

void insert_mip_filter_initialization_configuration_command_alignment_selector(struct mip_serializer* serializer, const mip_filter_initialization_configuration_command_alignment_selector self);
void extract_mip_filter_initialization_configuration_command_alignment_selector(struct mip_serializer* serializer, mip_filter_initialization_configuration_command_alignment_selector* self);

void insert_mip_filter_initialization_configuration_command_initial_condition_source(struct mip_serializer* serializer, const mip_filter_initialization_configuration_command_initial_condition_source self);
void extract_mip_filter_initialization_configuration_command_initial_condition_source(struct mip_serializer* serializer, mip_filter_initialization_configuration_command_initial_condition_source* self);

struct mip_filter_initialization_configuration_response
{
    uint8_t wait_for_run_command; ///< Initialize filter only after receiving "run" command
    mip_filter_initialization_configuration_command_initial_condition_source initial_cond_src; ///< Initial condition source:
    mip_filter_initialization_configuration_command_alignment_selector auto_heading_alignment_selector; ///< Bitfield specifying the allowed automatic heading alignment methods for automatic initial conditions. Bits are set to 1 to enable, and the correspond to the following: <br/>
    float initial_heading; ///< User-specified initial platform heading (degrees).
    float initial_pitch; ///< User-specified initial platform pitch (degrees)
    float initial_roll; ///< User-specified initial platform roll (degrees)
    mip_vector3f initial_position; ///< User-specified initial platform position (units determined by reference frame selector, see note.)
    mip_vector3f initial_velocity; ///< User-specified initial platform velocity (units determined by reference frame selector, see note.)
    mip_filter_reference_frame reference_frame_selector; ///< User-specified initial position/velocity reference frames
    
};
typedef struct mip_filter_initialization_configuration_response mip_filter_initialization_configuration_response;
void insert_mip_filter_initialization_configuration_response(struct mip_serializer* serializer, const mip_filter_initialization_configuration_response* self);
void extract_mip_filter_initialization_configuration_response(struct mip_serializer* serializer, mip_filter_initialization_configuration_response* self);

mip_cmd_result mip_filter_write_initialization_configuration(struct mip_interface* device, uint8_t wait_for_run_command, mip_filter_initialization_configuration_command_initial_condition_source initial_cond_src, mip_filter_initialization_configuration_command_alignment_selector auto_heading_alignment_selector, float initial_heading, float initial_pitch, float initial_roll, const float* initial_position, const float* initial_velocity, mip_filter_reference_frame reference_frame_selector);
mip_cmd_result mip_filter_read_initialization_configuration(struct mip_interface* device, uint8_t* wait_for_run_command_out, mip_filter_initialization_configuration_command_initial_condition_source* initial_cond_src_out, mip_filter_initialization_configuration_command_alignment_selector* auto_heading_alignment_selector_out, float* initial_heading_out, float* initial_pitch_out, float* initial_roll_out, float* initial_position_out, float* initial_velocity_out, mip_filter_reference_frame* reference_frame_selector_out);
mip_cmd_result mip_filter_save_initialization_configuration(struct mip_interface* device);
mip_cmd_result mip_filter_load_initialization_configuration(struct mip_interface* device);
mip_cmd_result mip_filter_default_initialization_configuration(struct mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_adaptive_filter_options  (0x0D,0x53) Adaptive Filter Options [C]
/// Configures the basic setup for auto-adaptive filtering. See product manual for a detailed description of this feature.
///
///@{

struct mip_filter_adaptive_filter_options_command
{
    mip_function_selector function;
    uint8_t level; ///< Auto-adaptive operating level: <br/> 0=Off, <br/> 1=Conservative, <br/> 2=Moderate (default), <br/> 3=Aggressive.
    uint16_t time_limit; ///< Maximum duration of measurement rejection before entering recovery mode    (ms)
    
};
typedef struct mip_filter_adaptive_filter_options_command mip_filter_adaptive_filter_options_command;
void insert_mip_filter_adaptive_filter_options_command(struct mip_serializer* serializer, const mip_filter_adaptive_filter_options_command* self);
void extract_mip_filter_adaptive_filter_options_command(struct mip_serializer* serializer, mip_filter_adaptive_filter_options_command* self);

struct mip_filter_adaptive_filter_options_response
{
    uint8_t level; ///< Auto-adaptive operating level: <br/> 0=Off, <br/> 1=Conservative, <br/> 2=Moderate (default), <br/> 3=Aggressive.
    uint16_t time_limit; ///< Maximum duration of measurement rejection before entering recovery mode    (ms)
    
};
typedef struct mip_filter_adaptive_filter_options_response mip_filter_adaptive_filter_options_response;
void insert_mip_filter_adaptive_filter_options_response(struct mip_serializer* serializer, const mip_filter_adaptive_filter_options_response* self);
void extract_mip_filter_adaptive_filter_options_response(struct mip_serializer* serializer, mip_filter_adaptive_filter_options_response* self);

mip_cmd_result mip_filter_write_adaptive_filter_options(struct mip_interface* device, uint8_t level, uint16_t time_limit);
mip_cmd_result mip_filter_read_adaptive_filter_options(struct mip_interface* device, uint8_t* level_out, uint16_t* time_limit_out);
mip_cmd_result mip_filter_save_adaptive_filter_options(struct mip_interface* device);
mip_cmd_result mip_filter_load_adaptive_filter_options(struct mip_interface* device);
mip_cmd_result mip_filter_default_adaptive_filter_options(struct mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_multi_antenna_offset  (0x0D,0x54) Multi Antenna Offset [C]
/// Set the antenna lever arm.
/// 
/// This command works with devices that utilize multiple antennas.
/// <br/><br/><b>Offset Limit</b>: 10 m magnitude (default)
///
///@{

struct mip_filter_multi_antenna_offset_command
{
    mip_function_selector function;
    uint8_t receiver_id; ///< Receiver: 1, 2, etc...
    mip_vector3f antenna_offset; ///< Antenna lever arm offset vector in the vehicle frame (m)
    
};
typedef struct mip_filter_multi_antenna_offset_command mip_filter_multi_antenna_offset_command;
void insert_mip_filter_multi_antenna_offset_command(struct mip_serializer* serializer, const mip_filter_multi_antenna_offset_command* self);
void extract_mip_filter_multi_antenna_offset_command(struct mip_serializer* serializer, mip_filter_multi_antenna_offset_command* self);

struct mip_filter_multi_antenna_offset_response
{
    uint8_t receiver_id;
    mip_vector3f antenna_offset;
    
};
typedef struct mip_filter_multi_antenna_offset_response mip_filter_multi_antenna_offset_response;
void insert_mip_filter_multi_antenna_offset_response(struct mip_serializer* serializer, const mip_filter_multi_antenna_offset_response* self);
void extract_mip_filter_multi_antenna_offset_response(struct mip_serializer* serializer, mip_filter_multi_antenna_offset_response* self);

mip_cmd_result mip_filter_write_multi_antenna_offset(struct mip_interface* device, uint8_t receiver_id, const float* antenna_offset);
mip_cmd_result mip_filter_read_multi_antenna_offset(struct mip_interface* device, uint8_t receiver_id, float* antenna_offset_out);
mip_cmd_result mip_filter_save_multi_antenna_offset(struct mip_interface* device, uint8_t receiver_id);
mip_cmd_result mip_filter_load_multi_antenna_offset(struct mip_interface* device, uint8_t receiver_id);
mip_cmd_result mip_filter_default_multi_antenna_offset(struct mip_interface* device, uint8_t receiver_id);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_rel_pos_configuration  (0x0D,0x55) Rel Pos Configuration [C]
/// Configure the reference location for filter relative positioning outputs
///
///@{

struct mip_filter_rel_pos_configuration_command
{
    mip_function_selector function;
    uint8_t source; ///< 0 - auto (RTK base station), 1 - manual
    mip_filter_reference_frame reference_frame_selector; ///< ECEF or LLH
    mip_vector3d reference_coordinates; ///< reference coordinates, units determined by source selection
    
};
typedef struct mip_filter_rel_pos_configuration_command mip_filter_rel_pos_configuration_command;
void insert_mip_filter_rel_pos_configuration_command(struct mip_serializer* serializer, const mip_filter_rel_pos_configuration_command* self);
void extract_mip_filter_rel_pos_configuration_command(struct mip_serializer* serializer, mip_filter_rel_pos_configuration_command* self);

struct mip_filter_rel_pos_configuration_response
{
    uint8_t source; ///< 0 - auto (RTK base station), 1 - manual
    mip_filter_reference_frame reference_frame_selector; ///< ECEF or LLH
    mip_vector3d reference_coordinates; ///< reference coordinates, units determined by source selection
    
};
typedef struct mip_filter_rel_pos_configuration_response mip_filter_rel_pos_configuration_response;
void insert_mip_filter_rel_pos_configuration_response(struct mip_serializer* serializer, const mip_filter_rel_pos_configuration_response* self);
void extract_mip_filter_rel_pos_configuration_response(struct mip_serializer* serializer, mip_filter_rel_pos_configuration_response* self);

mip_cmd_result mip_filter_write_rel_pos_configuration(struct mip_interface* device, uint8_t source, mip_filter_reference_frame reference_frame_selector, const double* reference_coordinates);
mip_cmd_result mip_filter_read_rel_pos_configuration(struct mip_interface* device, uint8_t* source_out, mip_filter_reference_frame* reference_frame_selector_out, double* reference_coordinates_out);
mip_cmd_result mip_filter_save_rel_pos_configuration(struct mip_interface* device);
mip_cmd_result mip_filter_load_rel_pos_configuration(struct mip_interface* device);
mip_cmd_result mip_filter_default_rel_pos_configuration(struct mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_ref_point_lever_arm  (0x0D,0x56) Ref Point Lever Arm [C]
/// Lever arm offset with respect to the sensor for the indicated point of reference.
/// This is used to change the location of the indicated point of reference, and will affect filter position and velocity outputs.
/// Changing this setting from default will result in a global position offset that depends on vehicle attitude,
/// and a velocity offset that depends on vehicle attitude and angular rate.
/// <br/>The lever arm is defined by a 3-element vector that points from the sensor to the desired reference point, with (x,y,z) components given in the vehicle's reference frame.
/// <br/><br/>Note, if the reference point selector is set to VEH (1), this setting will affect the following data fields: (0x82, 0x01), (0x82, 0x02), (0x82, 0x40), (0x82, 0x41), and (0x82, 42)
/// <br/><br/><b>Offset Limits</b>
/// <br/>Reference Point VEH (1): 10 m magnitude (default)
///
///@{

typedef uint8_t mip_filter_ref_point_lever_arm_command_reference_point_selector;
static const mip_filter_ref_point_lever_arm_command_reference_point_selector MIP_FILTER_REF_POINT_LEVER_ARM_COMMAND_REFERENCE_POINT_SELECTOR_VEH = 1; ///<  Defines the origin of the vehicle

struct mip_filter_ref_point_lever_arm_command
{
    mip_function_selector function;
    mip_filter_ref_point_lever_arm_command_reference_point_selector ref_point_sel; ///< Reserved, must be 1
    mip_vector3f lever_arm_offset; ///< [m] Lever arm offset vector in the vehicle's reference frame.
    
};
typedef struct mip_filter_ref_point_lever_arm_command mip_filter_ref_point_lever_arm_command;
void insert_mip_filter_ref_point_lever_arm_command(struct mip_serializer* serializer, const mip_filter_ref_point_lever_arm_command* self);
void extract_mip_filter_ref_point_lever_arm_command(struct mip_serializer* serializer, mip_filter_ref_point_lever_arm_command* self);

void insert_mip_filter_ref_point_lever_arm_command_reference_point_selector(struct mip_serializer* serializer, const mip_filter_ref_point_lever_arm_command_reference_point_selector self);
void extract_mip_filter_ref_point_lever_arm_command_reference_point_selector(struct mip_serializer* serializer, mip_filter_ref_point_lever_arm_command_reference_point_selector* self);

struct mip_filter_ref_point_lever_arm_response
{
    mip_filter_ref_point_lever_arm_command_reference_point_selector ref_point_sel; ///< Reserved, must be 1
    mip_vector3f lever_arm_offset; ///< [m] Lever arm offset vector in the vehicle's reference frame.
    
};
typedef struct mip_filter_ref_point_lever_arm_response mip_filter_ref_point_lever_arm_response;
void insert_mip_filter_ref_point_lever_arm_response(struct mip_serializer* serializer, const mip_filter_ref_point_lever_arm_response* self);
void extract_mip_filter_ref_point_lever_arm_response(struct mip_serializer* serializer, mip_filter_ref_point_lever_arm_response* self);

mip_cmd_result mip_filter_write_ref_point_lever_arm(struct mip_interface* device, mip_filter_ref_point_lever_arm_command_reference_point_selector ref_point_sel, const float* lever_arm_offset);
mip_cmd_result mip_filter_read_ref_point_lever_arm(struct mip_interface* device, mip_filter_ref_point_lever_arm_command_reference_point_selector* ref_point_sel_out, float* lever_arm_offset_out);
mip_cmd_result mip_filter_save_ref_point_lever_arm(struct mip_interface* device);
mip_cmd_result mip_filter_load_ref_point_lever_arm(struct mip_interface* device);
mip_cmd_result mip_filter_default_ref_point_lever_arm(struct mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_speed_measurement  (0x0D,0x60) Speed Measurement [C]
/// Speed aiding measurement, where speed is defined as rate of motion along the vehicle's x-axis direction.
/// Can be used by an external odometer/speedometer, for example.
/// This command cannot be used if the internal odometer is configured.
///
///@{

struct mip_filter_speed_measurement_command
{
    uint8_t source; ///< Reserved, must be 1.
    float time_of_week; ///< GPS time of week when speed was sampled
    float speed; ///< Estimated speed along vehicle's x-axis (may be positive or negative) [meters/second]
    float speed_uncertainty; ///< Estimated uncertainty in the speed measurement (1-sigma value) [meters/second]
    
};
typedef struct mip_filter_speed_measurement_command mip_filter_speed_measurement_command;
void insert_mip_filter_speed_measurement_command(struct mip_serializer* serializer, const mip_filter_speed_measurement_command* self);
void extract_mip_filter_speed_measurement_command(struct mip_serializer* serializer, mip_filter_speed_measurement_command* self);

mip_cmd_result mip_filter_speed_measurement(struct mip_interface* device, uint8_t source, float time_of_week, float speed, float speed_uncertainty);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_speed_lever_arm  (0x0D,0x61) Speed Lever Arm [C]
/// Lever arm offset for speed measurements.
/// This is used to compensate for an off-center measurement point
/// having a different speed due to rotation of the vehicle.
/// The typical use case for this would be an odometer attached to a wheel
/// on a standard 4-wheeled vehicle. If the odometer is on the left wheel,
/// it will report higher speed on right turns and lower speed on left turns.
/// This is because the outside edge of the curve is longer than the inside edge.
///
///@{

struct mip_filter_speed_lever_arm_command
{
    mip_function_selector function;
    uint8_t source; ///< Reserved, must be 1.
    mip_vector3f lever_arm_offset; ///< [m] Lever arm offset vector in the vehicle's reference frame.
    
};
typedef struct mip_filter_speed_lever_arm_command mip_filter_speed_lever_arm_command;
void insert_mip_filter_speed_lever_arm_command(struct mip_serializer* serializer, const mip_filter_speed_lever_arm_command* self);
void extract_mip_filter_speed_lever_arm_command(struct mip_serializer* serializer, mip_filter_speed_lever_arm_command* self);

struct mip_filter_speed_lever_arm_response
{
    uint8_t source; ///< Reserved, must be 1.
    mip_vector3f lever_arm_offset; ///< [m] Lever arm offset vector in the vehicle's reference frame.
    
};
typedef struct mip_filter_speed_lever_arm_response mip_filter_speed_lever_arm_response;
void insert_mip_filter_speed_lever_arm_response(struct mip_serializer* serializer, const mip_filter_speed_lever_arm_response* self);
void extract_mip_filter_speed_lever_arm_response(struct mip_serializer* serializer, mip_filter_speed_lever_arm_response* self);

mip_cmd_result mip_filter_write_speed_lever_arm(struct mip_interface* device, uint8_t source, const float* lever_arm_offset);
mip_cmd_result mip_filter_read_speed_lever_arm(struct mip_interface* device, uint8_t source, float* lever_arm_offset_out);
mip_cmd_result mip_filter_save_speed_lever_arm(struct mip_interface* device, uint8_t source);
mip_cmd_result mip_filter_load_speed_lever_arm(struct mip_interface* device, uint8_t source);
mip_cmd_result mip_filter_default_speed_lever_arm(struct mip_interface* device, uint8_t source);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_wheeled_vehicle_constraint_control  (0x0D,0x63) Wheeled Vehicle Constraint Control [C]
/// Configure the wheeled vehicle kinematic constraint.
/// 
/// When enabled, the filter uses the assumption that velocity is constrained to the primary vehicle axis.
/// By convention, the primary vehicle axis is the vehicle X-axis (note: the sensor may be physically installed in
/// any orientation on the vehicle if the appropriate mounting transformation has been specified).
/// This constraint will typically improve heading estimates for vehicles where the assumption is valid, such
/// as an automobile, particularly when GNSS coverage is intermittent.
///
///@{

struct mip_filter_wheeled_vehicle_constraint_control_command
{
    mip_function_selector function;
    uint8_t enable; ///< 0 - Disable, 1 - Enable
    
};
typedef struct mip_filter_wheeled_vehicle_constraint_control_command mip_filter_wheeled_vehicle_constraint_control_command;
void insert_mip_filter_wheeled_vehicle_constraint_control_command(struct mip_serializer* serializer, const mip_filter_wheeled_vehicle_constraint_control_command* self);
void extract_mip_filter_wheeled_vehicle_constraint_control_command(struct mip_serializer* serializer, mip_filter_wheeled_vehicle_constraint_control_command* self);

struct mip_filter_wheeled_vehicle_constraint_control_response
{
    uint8_t enable; ///< 0 - Disable, 1 - Enable
    
};
typedef struct mip_filter_wheeled_vehicle_constraint_control_response mip_filter_wheeled_vehicle_constraint_control_response;
void insert_mip_filter_wheeled_vehicle_constraint_control_response(struct mip_serializer* serializer, const mip_filter_wheeled_vehicle_constraint_control_response* self);
void extract_mip_filter_wheeled_vehicle_constraint_control_response(struct mip_serializer* serializer, mip_filter_wheeled_vehicle_constraint_control_response* self);

mip_cmd_result mip_filter_write_wheeled_vehicle_constraint_control(struct mip_interface* device, uint8_t enable);
mip_cmd_result mip_filter_read_wheeled_vehicle_constraint_control(struct mip_interface* device, uint8_t* enable_out);
mip_cmd_result mip_filter_save_wheeled_vehicle_constraint_control(struct mip_interface* device);
mip_cmd_result mip_filter_load_wheeled_vehicle_constraint_control(struct mip_interface* device);
mip_cmd_result mip_filter_default_wheeled_vehicle_constraint_control(struct mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_vertical_gyro_constraint_control  (0x0D,0x62) Vertical Gyro Constraint Control [C]
/// Configure the vertical gyro kinematic constraint.
/// 
/// When enabled and no valid GNSS measurements are available, the filter uses the accelerometers to track pitch
/// and roll under the assumption that the sensor platform is not undergoing linear acceleration.
/// This constraint is useful to maintain accurate pitch and roll during GNSS signal outages.
///
///@{

struct mip_filter_vertical_gyro_constraint_control_command
{
    mip_function_selector function;
    uint8_t enable; ///< 0 - Disable, 1 - Enable
    
};
typedef struct mip_filter_vertical_gyro_constraint_control_command mip_filter_vertical_gyro_constraint_control_command;
void insert_mip_filter_vertical_gyro_constraint_control_command(struct mip_serializer* serializer, const mip_filter_vertical_gyro_constraint_control_command* self);
void extract_mip_filter_vertical_gyro_constraint_control_command(struct mip_serializer* serializer, mip_filter_vertical_gyro_constraint_control_command* self);

struct mip_filter_vertical_gyro_constraint_control_response
{
    uint8_t enable; ///< 0 - Disable, 1 - Enable
    
};
typedef struct mip_filter_vertical_gyro_constraint_control_response mip_filter_vertical_gyro_constraint_control_response;
void insert_mip_filter_vertical_gyro_constraint_control_response(struct mip_serializer* serializer, const mip_filter_vertical_gyro_constraint_control_response* self);
void extract_mip_filter_vertical_gyro_constraint_control_response(struct mip_serializer* serializer, mip_filter_vertical_gyro_constraint_control_response* self);

mip_cmd_result mip_filter_write_vertical_gyro_constraint_control(struct mip_interface* device, uint8_t enable);
mip_cmd_result mip_filter_read_vertical_gyro_constraint_control(struct mip_interface* device, uint8_t* enable_out);
mip_cmd_result mip_filter_save_vertical_gyro_constraint_control(struct mip_interface* device);
mip_cmd_result mip_filter_load_vertical_gyro_constraint_control(struct mip_interface* device);
mip_cmd_result mip_filter_default_vertical_gyro_constraint_control(struct mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_gnss_antenna_cal_control  (0x0D,0x64) Gnss Antenna Cal Control [C]
/// Configure the GNSS antenna lever arm calibration.
/// 
/// When enabled, the filter will enable lever arm error tracking, up to the maximum offset specified.
///
///@{

struct mip_filter_gnss_antenna_cal_control_command
{
    mip_function_selector function;
    uint8_t enable; ///< 0 - Disable, 1 - Enable
    float max_offset; ///< Maximum absolute value of lever arm offset error in the vehicle frame [meters]. See device user manual for the valid range of this parameter.
    
};
typedef struct mip_filter_gnss_antenna_cal_control_command mip_filter_gnss_antenna_cal_control_command;
void insert_mip_filter_gnss_antenna_cal_control_command(struct mip_serializer* serializer, const mip_filter_gnss_antenna_cal_control_command* self);
void extract_mip_filter_gnss_antenna_cal_control_command(struct mip_serializer* serializer, mip_filter_gnss_antenna_cal_control_command* self);

struct mip_filter_gnss_antenna_cal_control_response
{
    uint8_t enable; ///< 0 - Disable, 1 - Enable
    float max_offset; ///< Maximum absolute value of lever arm offset error in the vehicle frame [meters]. See device user manual for the valid range of this parameter.
    
};
typedef struct mip_filter_gnss_antenna_cal_control_response mip_filter_gnss_antenna_cal_control_response;
void insert_mip_filter_gnss_antenna_cal_control_response(struct mip_serializer* serializer, const mip_filter_gnss_antenna_cal_control_response* self);
void extract_mip_filter_gnss_antenna_cal_control_response(struct mip_serializer* serializer, mip_filter_gnss_antenna_cal_control_response* self);

mip_cmd_result mip_filter_write_gnss_antenna_cal_control(struct mip_interface* device, uint8_t enable, float max_offset);
mip_cmd_result mip_filter_read_gnss_antenna_cal_control(struct mip_interface* device, uint8_t* enable_out, float* max_offset_out);
mip_cmd_result mip_filter_save_gnss_antenna_cal_control(struct mip_interface* device);
mip_cmd_result mip_filter_load_gnss_antenna_cal_control(struct mip_interface* device);
mip_cmd_result mip_filter_default_gnss_antenna_cal_control(struct mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_set_initial_heading  (0x0D,0x03) Set Initial Heading [C]
/// Set the initial heading angle.
/// 
/// The estimation filter will reset the heading estimate to provided value. If the product supports magnetometer aiding and this feature has been enabled, the heading
/// argument will be ignored and the filter will initialize using the inferred magnetic heading.
///
///@{

struct mip_filter_set_initial_heading_command
{
    float heading; ///< Initial heading in radians [-pi, pi]
    
};
typedef struct mip_filter_set_initial_heading_command mip_filter_set_initial_heading_command;
void insert_mip_filter_set_initial_heading_command(struct mip_serializer* serializer, const mip_filter_set_initial_heading_command* self);
void extract_mip_filter_set_initial_heading_command(struct mip_serializer* serializer, mip_filter_set_initial_heading_command* self);

mip_cmd_result mip_filter_set_initial_heading(struct mip_interface* device, float heading);

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


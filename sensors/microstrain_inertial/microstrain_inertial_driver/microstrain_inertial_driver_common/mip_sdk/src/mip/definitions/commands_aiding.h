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
///@defgroup aiding_commands_c  Aiding Commands [C]
///
///@{

////////////////////////////////////////////////////////////////////////////////
// Descriptors
////////////////////////////////////////////////////////////////////////////////

enum 
{
    MIP_AIDING_CMD_DESC_SET                = 0x13,
    
    MIP_CMD_DESC_AIDING_FRAME_CONFIG       = 0x01,
    MIP_CMD_DESC_AIDING_LOCAL_FRAME        = 0x03,
    MIP_CMD_DESC_AIDING_ECHO_CONTROL       = 0x1F,
    MIP_CMD_DESC_AIDING_POS_LOCAL          = 0x20,
    MIP_CMD_DESC_AIDING_POS_ECEF           = 0x21,
    MIP_CMD_DESC_AIDING_POS_LLH            = 0x22,
    MIP_CMD_DESC_AIDING_HEIGHT_ABS         = 0x23,
    MIP_CMD_DESC_AIDING_HEIGHT_REL         = 0x24,
    MIP_CMD_DESC_AIDING_VEL_ECEF           = 0x28,
    MIP_CMD_DESC_AIDING_VEL_NED            = 0x29,
    MIP_CMD_DESC_AIDING_VEL_ODOM           = 0x2A,
    MIP_CMD_DESC_AIDING_WHEELSPEED         = 0x2B,
    MIP_CMD_DESC_AIDING_HEADING_TRUE       = 0x31,
    MIP_CMD_DESC_AIDING_MAGNETIC_FIELD     = 0x32,
    MIP_CMD_DESC_AIDING_PRESSURE           = 0x33,
    MIP_CMD_DESC_AIDING_DELTA_POSITION     = 0x38,
    MIP_CMD_DESC_AIDING_DELTA_ATTITUDE     = 0x39,
    MIP_CMD_DESC_AIDING_LOCAL_ANGULAR_RATE = 0x3A,
    
    MIP_REPLY_DESC_AIDING_FRAME_CONFIG     = 0x81,
    MIP_REPLY_DESC_AIDING_ECHO_CONTROL     = 0x9F,
};

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

typedef uint8_t mip_time_timebase;
static const mip_time_timebase MIP_TIME_TIMEBASE_INTERNAL_REFERENCE = 1; ///<  Timestamp provided is with respect to internal clock.
static const mip_time_timebase MIP_TIME_TIMEBASE_EXTERNAL_TIME      = 2; ///<  Timestamp provided is with respect to external clock, synced by PPS source.
static const mip_time_timebase MIP_TIME_TIMEBASE_TIME_OF_ARRIVAL    = 3; ///<  Timestamp provided is a fixed latency relative to time of message arrival.

struct mip_time
{
    mip_time_timebase timebase; ///< Timebase reference, e.g. internal, external, GPS, UTC, etc.
    uint8_t reserved; ///< Reserved, set to 0x01.
    uint64_t nanoseconds; ///< Nanoseconds since the timebase epoch.
    
};
typedef struct mip_time mip_time;
void insert_mip_time(struct mip_serializer* serializer, const mip_time* self);
void extract_mip_time(struct mip_serializer* serializer, mip_time* self);

void insert_mip_time_timebase(struct mip_serializer* serializer, const mip_time_timebase self);
void extract_mip_time_timebase(struct mip_serializer* serializer, mip_time_timebase* self);


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
///@defgroup c_aiding_frame_config  (0x13,0x01) Frame Config [C]
/// Defines an aiding frame associated with a specific sensor frame ID.  The frame ID used in this command
/// should mirror the frame ID used in the aiding command (if that aiding measurement is measured in this reference frame)
/// 
/// This transform satisfies the following relationship:
/// 
/// EQSTART p^{veh} = R p^{sensor_frame} + t EQEND<br/>
/// 
/// Where:<br/>
/// EQSTART R EQEND is rotation matrix defined by the rotation component and EQSTART t EQEND is the translation vector<br/><br/>
/// EQSTART p^{sensor_frame} EQEND is a 3-element position vector expressed in the external sensor frame<br/>
/// EQSTART p^{veh} EQEND is a 3-element position vector expressed in the vehicle frame<br/>
/// 
/// Rotation can be defined using Euler angles OR quaternions.  If Format selector is set to Euler Angles, the fourth element
/// in the rotation vector is ignored and should be set to 0.
/// 
/// When the tracking_enabled flag is 1, the Kalman filter will track errors in the provided frame definition; when 0, no errors are tracked.
/// 
/// Example: GNSS antenna lever arm
/// 
/// Frame ID: 1
/// Format: 1 (Euler)
/// Translation: [0,1,] (GNSS with a 1 meter Y offset in the vehicle frame)
/// Rotation: [0,0,0,0] (Rotational component is not relevant for GNSS measurements, set to zero)
/// 
///
///@{

typedef uint8_t mip_aiding_frame_config_command_format;
static const mip_aiding_frame_config_command_format MIP_AIDING_FRAME_CONFIG_COMMAND_FORMAT_EULER      = 1; ///<  Translation vector followed by euler angles (roll, pitch, yaw).
static const mip_aiding_frame_config_command_format MIP_AIDING_FRAME_CONFIG_COMMAND_FORMAT_QUATERNION = 2; ///<  Translation vector followed by quaternion (w, x, y, z).

union mip_aiding_frame_config_command_rotation
{
    mip_vector3f euler;
    mip_quatf quaternion;
};
typedef union mip_aiding_frame_config_command_rotation mip_aiding_frame_config_command_rotation;

struct mip_aiding_frame_config_command
{
    mip_function_selector function;
    uint8_t frame_id; ///< Reference frame number. Limit 4.
    mip_aiding_frame_config_command_format format; ///< Format of the transformation.
    bool tracking_enabled; ///< If enabled, the Kalman filter will track errors.
    mip_vector3f translation; ///< Translation X, Y, and Z.
    mip_aiding_frame_config_command_rotation rotation; ///< Rotation as specified by format.
    
};
typedef struct mip_aiding_frame_config_command mip_aiding_frame_config_command;
void insert_mip_aiding_frame_config_command(struct mip_serializer* serializer, const mip_aiding_frame_config_command* self);
void extract_mip_aiding_frame_config_command(struct mip_serializer* serializer, mip_aiding_frame_config_command* self);

void insert_mip_aiding_frame_config_command_format(struct mip_serializer* serializer, const mip_aiding_frame_config_command_format self);
void extract_mip_aiding_frame_config_command_format(struct mip_serializer* serializer, mip_aiding_frame_config_command_format* self);

struct mip_aiding_frame_config_response
{
    uint8_t frame_id; ///< Reference frame number. Limit 4.
    mip_aiding_frame_config_command_format format; ///< Format of the transformation.
    bool tracking_enabled; ///< If enabled, the Kalman filter will track errors.
    mip_vector3f translation; ///< Translation X, Y, and Z.
    mip_aiding_frame_config_command_rotation rotation; ///< Rotation as specified by format.
    
};
typedef struct mip_aiding_frame_config_response mip_aiding_frame_config_response;
void insert_mip_aiding_frame_config_response(struct mip_serializer* serializer, const mip_aiding_frame_config_response* self);
void extract_mip_aiding_frame_config_response(struct mip_serializer* serializer, mip_aiding_frame_config_response* self);

mip_cmd_result mip_aiding_write_frame_config(struct mip_interface* device, uint8_t frame_id, mip_aiding_frame_config_command_format format, bool tracking_enabled, const float* translation, const mip_aiding_frame_config_command_rotation* rotation);
mip_cmd_result mip_aiding_read_frame_config(struct mip_interface* device, uint8_t frame_id, mip_aiding_frame_config_command_format format, bool* tracking_enabled_out, float* translation_out, mip_aiding_frame_config_command_rotation* rotation_out);
mip_cmd_result mip_aiding_save_frame_config(struct mip_interface* device, uint8_t frame_id);
mip_cmd_result mip_aiding_load_frame_config(struct mip_interface* device, uint8_t frame_id);
mip_cmd_result mip_aiding_default_frame_config(struct mip_interface* device, uint8_t frame_id);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_aiding_aiding_echo_control  (0x13,0x1F) Aiding Echo Control [C]
/// Controls command response behavior to external aiding commands
///
///@{

typedef uint8_t mip_aiding_aiding_echo_control_command_mode;
static const mip_aiding_aiding_echo_control_command_mode MIP_AIDING_AIDING_ECHO_CONTROL_COMMAND_MODE_SUPPRESS_ACK = 0; ///<  Suppresses the usual command ack field for aiding messages.
static const mip_aiding_aiding_echo_control_command_mode MIP_AIDING_AIDING_ECHO_CONTROL_COMMAND_MODE_STANDARD     = 1; ///<  Normal ack/nack behavior.
static const mip_aiding_aiding_echo_control_command_mode MIP_AIDING_AIDING_ECHO_CONTROL_COMMAND_MODE_RESPONSE     = 2; ///<  Echo the data back as a response.

struct mip_aiding_aiding_echo_control_command
{
    mip_function_selector function;
    mip_aiding_aiding_echo_control_command_mode mode; ///< Controls data echoing.
    
};
typedef struct mip_aiding_aiding_echo_control_command mip_aiding_aiding_echo_control_command;
void insert_mip_aiding_aiding_echo_control_command(struct mip_serializer* serializer, const mip_aiding_aiding_echo_control_command* self);
void extract_mip_aiding_aiding_echo_control_command(struct mip_serializer* serializer, mip_aiding_aiding_echo_control_command* self);

void insert_mip_aiding_aiding_echo_control_command_mode(struct mip_serializer* serializer, const mip_aiding_aiding_echo_control_command_mode self);
void extract_mip_aiding_aiding_echo_control_command_mode(struct mip_serializer* serializer, mip_aiding_aiding_echo_control_command_mode* self);

struct mip_aiding_aiding_echo_control_response
{
    mip_aiding_aiding_echo_control_command_mode mode; ///< Controls data echoing.
    
};
typedef struct mip_aiding_aiding_echo_control_response mip_aiding_aiding_echo_control_response;
void insert_mip_aiding_aiding_echo_control_response(struct mip_serializer* serializer, const mip_aiding_aiding_echo_control_response* self);
void extract_mip_aiding_aiding_echo_control_response(struct mip_serializer* serializer, mip_aiding_aiding_echo_control_response* self);

mip_cmd_result mip_aiding_write_aiding_echo_control(struct mip_interface* device, mip_aiding_aiding_echo_control_command_mode mode);
mip_cmd_result mip_aiding_read_aiding_echo_control(struct mip_interface* device, mip_aiding_aiding_echo_control_command_mode* mode_out);
mip_cmd_result mip_aiding_save_aiding_echo_control(struct mip_interface* device);
mip_cmd_result mip_aiding_load_aiding_echo_control(struct mip_interface* device);
mip_cmd_result mip_aiding_default_aiding_echo_control(struct mip_interface* device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_aiding_ecef_pos  (0x13,0x21) Ecef Pos [C]
/// Cartesian vector position aiding command. Coordinates are given in the WGS84 ECEF system.
///
///@{

typedef uint16_t mip_aiding_ecef_pos_command_valid_flags;
static const mip_aiding_ecef_pos_command_valid_flags MIP_AIDING_ECEF_POS_COMMAND_VALID_FLAGS_NONE = 0x0000;
static const mip_aiding_ecef_pos_command_valid_flags MIP_AIDING_ECEF_POS_COMMAND_VALID_FLAGS_X    = 0x0001; ///<  
static const mip_aiding_ecef_pos_command_valid_flags MIP_AIDING_ECEF_POS_COMMAND_VALID_FLAGS_Y    = 0x0002; ///<  
static const mip_aiding_ecef_pos_command_valid_flags MIP_AIDING_ECEF_POS_COMMAND_VALID_FLAGS_Z    = 0x0004; ///<  
static const mip_aiding_ecef_pos_command_valid_flags MIP_AIDING_ECEF_POS_COMMAND_VALID_FLAGS_ALL  = 0x0007;

struct mip_aiding_ecef_pos_command
{
    mip_time time; ///< Timestamp of the measurement.
    uint8_t frame_id; ///< Source ID for this estimate (source_id == 0 indicates this sensor, source_id > 0 indicates an external estimate).
    mip_vector3d position; ///< ECEF position [m].
    mip_vector3f uncertainty; ///< ECEF position uncertainty [m]. Cannot be 0 unless the corresponding valid flags are 0.
    mip_aiding_ecef_pos_command_valid_flags valid_flags; ///< Valid flags. Axes with 0 will be completely ignored.
    
};
typedef struct mip_aiding_ecef_pos_command mip_aiding_ecef_pos_command;
void insert_mip_aiding_ecef_pos_command(struct mip_serializer* serializer, const mip_aiding_ecef_pos_command* self);
void extract_mip_aiding_ecef_pos_command(struct mip_serializer* serializer, mip_aiding_ecef_pos_command* self);

void insert_mip_aiding_ecef_pos_command_valid_flags(struct mip_serializer* serializer, const mip_aiding_ecef_pos_command_valid_flags self);
void extract_mip_aiding_ecef_pos_command_valid_flags(struct mip_serializer* serializer, mip_aiding_ecef_pos_command_valid_flags* self);

mip_cmd_result mip_aiding_ecef_pos(struct mip_interface* device, const mip_time* time, uint8_t frame_id, const double* position, const float* uncertainty, mip_aiding_ecef_pos_command_valid_flags valid_flags);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_aiding_llh_pos  (0x13,0x22) Llh Pos [C]
/// Geodetic position aiding command. Coordinates are given in WGS84 geodetic latitude, longitude, and height above the ellipsoid.
/// Uncertainty is given in NED coordinates, which are parallel to incremental changes in latitude, longitude, and height.
///
///@{

typedef uint16_t mip_aiding_llh_pos_command_valid_flags;
static const mip_aiding_llh_pos_command_valid_flags MIP_AIDING_LLH_POS_COMMAND_VALID_FLAGS_NONE      = 0x0000;
static const mip_aiding_llh_pos_command_valid_flags MIP_AIDING_LLH_POS_COMMAND_VALID_FLAGS_LATITUDE  = 0x0001; ///<  
static const mip_aiding_llh_pos_command_valid_flags MIP_AIDING_LLH_POS_COMMAND_VALID_FLAGS_LONGITUDE = 0x0002; ///<  
static const mip_aiding_llh_pos_command_valid_flags MIP_AIDING_LLH_POS_COMMAND_VALID_FLAGS_HEIGHT    = 0x0004; ///<  
static const mip_aiding_llh_pos_command_valid_flags MIP_AIDING_LLH_POS_COMMAND_VALID_FLAGS_ALL       = 0x0007;

struct mip_aiding_llh_pos_command
{
    mip_time time; ///< Timestamp of the measurement.
    uint8_t frame_id; ///< Source ID for this estimate (source_id == 0 indicates this sensor, source_id > 0 indicates an external estimate).
    double latitude; ///< [deg]
    double longitude; ///< [deg]
    double height; ///< [m]
    mip_vector3f uncertainty; ///< NED position uncertainty. Cannot be 0 unless the corresponding valid flags are 0.
    mip_aiding_llh_pos_command_valid_flags valid_flags; ///< Valid flags. Axes with 0 will be completely ignored.
    
};
typedef struct mip_aiding_llh_pos_command mip_aiding_llh_pos_command;
void insert_mip_aiding_llh_pos_command(struct mip_serializer* serializer, const mip_aiding_llh_pos_command* self);
void extract_mip_aiding_llh_pos_command(struct mip_serializer* serializer, mip_aiding_llh_pos_command* self);

void insert_mip_aiding_llh_pos_command_valid_flags(struct mip_serializer* serializer, const mip_aiding_llh_pos_command_valid_flags self);
void extract_mip_aiding_llh_pos_command_valid_flags(struct mip_serializer* serializer, mip_aiding_llh_pos_command_valid_flags* self);

mip_cmd_result mip_aiding_llh_pos(struct mip_interface* device, const mip_time* time, uint8_t frame_id, double latitude, double longitude, double height, const float* uncertainty, mip_aiding_llh_pos_command_valid_flags valid_flags);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_aiding_height  (0x13,0x23) Height [C]
/// Estimated value of height.
///
///@{

struct mip_aiding_height_command
{
    mip_time time; ///< Timestamp of the measurement.
    uint8_t frame_id; ///< Source ID for this estimate (source_id == 0 indicates this sensor, source_id > 0 indicates an external estimate).
    float height; ///< [m]
    float uncertainty; ///< [m]
    uint16_t valid_flags;
    
};
typedef struct mip_aiding_height_command mip_aiding_height_command;
void insert_mip_aiding_height_command(struct mip_serializer* serializer, const mip_aiding_height_command* self);
void extract_mip_aiding_height_command(struct mip_serializer* serializer, mip_aiding_height_command* self);

mip_cmd_result mip_aiding_height(struct mip_interface* device, const mip_time* time, uint8_t frame_id, float height, float uncertainty, uint16_t valid_flags);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_aiding_ecef_vel  (0x13,0x28) Ecef Vel [C]
/// ECEF velocity aiding command. Coordinates are given in the WGS84 ECEF frame.
///
///@{

typedef uint16_t mip_aiding_ecef_vel_command_valid_flags;
static const mip_aiding_ecef_vel_command_valid_flags MIP_AIDING_ECEF_VEL_COMMAND_VALID_FLAGS_NONE = 0x0000;
static const mip_aiding_ecef_vel_command_valid_flags MIP_AIDING_ECEF_VEL_COMMAND_VALID_FLAGS_X    = 0x0001; ///<  
static const mip_aiding_ecef_vel_command_valid_flags MIP_AIDING_ECEF_VEL_COMMAND_VALID_FLAGS_Y    = 0x0002; ///<  
static const mip_aiding_ecef_vel_command_valid_flags MIP_AIDING_ECEF_VEL_COMMAND_VALID_FLAGS_Z    = 0x0004; ///<  
static const mip_aiding_ecef_vel_command_valid_flags MIP_AIDING_ECEF_VEL_COMMAND_VALID_FLAGS_ALL  = 0x0007;

struct mip_aiding_ecef_vel_command
{
    mip_time time; ///< Timestamp of the measurement.
    uint8_t frame_id; ///< Source ID for this estimate (source_id == 0 indicates this sensor, source_id > 0 indicates an external estimate).
    mip_vector3f velocity; ///< ECEF velocity [m/s].
    mip_vector3f uncertainty; ///< ECEF velocity uncertainty [m/s]. Cannot be 0 unless the corresponding valid flags are 0.
    mip_aiding_ecef_vel_command_valid_flags valid_flags; ///< Valid flags. Axes with 0 will be completely ignored.
    
};
typedef struct mip_aiding_ecef_vel_command mip_aiding_ecef_vel_command;
void insert_mip_aiding_ecef_vel_command(struct mip_serializer* serializer, const mip_aiding_ecef_vel_command* self);
void extract_mip_aiding_ecef_vel_command(struct mip_serializer* serializer, mip_aiding_ecef_vel_command* self);

void insert_mip_aiding_ecef_vel_command_valid_flags(struct mip_serializer* serializer, const mip_aiding_ecef_vel_command_valid_flags self);
void extract_mip_aiding_ecef_vel_command_valid_flags(struct mip_serializer* serializer, mip_aiding_ecef_vel_command_valid_flags* self);

mip_cmd_result mip_aiding_ecef_vel(struct mip_interface* device, const mip_time* time, uint8_t frame_id, const float* velocity, const float* uncertainty, mip_aiding_ecef_vel_command_valid_flags valid_flags);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_aiding_ned_vel  (0x13,0x29) Ned Vel [C]
/// NED velocity aiding command. Coordinates are given in the local North-East-Down frame.
///
///@{

typedef uint16_t mip_aiding_ned_vel_command_valid_flags;
static const mip_aiding_ned_vel_command_valid_flags MIP_AIDING_NED_VEL_COMMAND_VALID_FLAGS_NONE = 0x0000;
static const mip_aiding_ned_vel_command_valid_flags MIP_AIDING_NED_VEL_COMMAND_VALID_FLAGS_X    = 0x0001; ///<  
static const mip_aiding_ned_vel_command_valid_flags MIP_AIDING_NED_VEL_COMMAND_VALID_FLAGS_Y    = 0x0002; ///<  
static const mip_aiding_ned_vel_command_valid_flags MIP_AIDING_NED_VEL_COMMAND_VALID_FLAGS_Z    = 0x0004; ///<  
static const mip_aiding_ned_vel_command_valid_flags MIP_AIDING_NED_VEL_COMMAND_VALID_FLAGS_ALL  = 0x0007;

struct mip_aiding_ned_vel_command
{
    mip_time time; ///< Timestamp of the measurement.
    uint8_t frame_id; ///< Source ID for this estimate (source_id == 0 indicates this sensor, source_id > 0 indicates an external estimate).
    mip_vector3f velocity; ///< NED velocity [m/s].
    mip_vector3f uncertainty; ///< NED velocity uncertainty [m/s]. Cannot be 0 unless the corresponding valid flags are 0.
    mip_aiding_ned_vel_command_valid_flags valid_flags; ///< Valid flags. Axes with 0 will be completely ignored.
    
};
typedef struct mip_aiding_ned_vel_command mip_aiding_ned_vel_command;
void insert_mip_aiding_ned_vel_command(struct mip_serializer* serializer, const mip_aiding_ned_vel_command* self);
void extract_mip_aiding_ned_vel_command(struct mip_serializer* serializer, mip_aiding_ned_vel_command* self);

void insert_mip_aiding_ned_vel_command_valid_flags(struct mip_serializer* serializer, const mip_aiding_ned_vel_command_valid_flags self);
void extract_mip_aiding_ned_vel_command_valid_flags(struct mip_serializer* serializer, mip_aiding_ned_vel_command_valid_flags* self);

mip_cmd_result mip_aiding_ned_vel(struct mip_interface* device, const mip_time* time, uint8_t frame_id, const float* velocity, const float* uncertainty, mip_aiding_ned_vel_command_valid_flags valid_flags);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_aiding_vehicle_fixed_frame_velocity  (0x13,0x2A) Vehicle Fixed Frame Velocity [C]
/// Estimate of velocity of the vehicle in the frame associated
/// with the given sensor ID.
///
///@{

typedef uint16_t mip_aiding_vehicle_fixed_frame_velocity_command_valid_flags;
static const mip_aiding_vehicle_fixed_frame_velocity_command_valid_flags MIP_AIDING_VEHICLE_FIXED_FRAME_VELOCITY_COMMAND_VALID_FLAGS_NONE = 0x0000;
static const mip_aiding_vehicle_fixed_frame_velocity_command_valid_flags MIP_AIDING_VEHICLE_FIXED_FRAME_VELOCITY_COMMAND_VALID_FLAGS_X    = 0x0001; ///<  
static const mip_aiding_vehicle_fixed_frame_velocity_command_valid_flags MIP_AIDING_VEHICLE_FIXED_FRAME_VELOCITY_COMMAND_VALID_FLAGS_Y    = 0x0002; ///<  
static const mip_aiding_vehicle_fixed_frame_velocity_command_valid_flags MIP_AIDING_VEHICLE_FIXED_FRAME_VELOCITY_COMMAND_VALID_FLAGS_Z    = 0x0004; ///<  
static const mip_aiding_vehicle_fixed_frame_velocity_command_valid_flags MIP_AIDING_VEHICLE_FIXED_FRAME_VELOCITY_COMMAND_VALID_FLAGS_ALL  = 0x0007;

struct mip_aiding_vehicle_fixed_frame_velocity_command
{
    mip_time time; ///< Timestamp of the measurement.
    uint8_t frame_id; ///< Source ID for this estimate (source_id == 0 indicates this sensor, source_id > 0 indicates an external estimate).
    mip_vector3f velocity; ///< [m/s]
    mip_vector3f uncertainty; ///< [m/s] 1-sigma uncertainty. Cannot be 0 unless the corresponding valid flags are 0.
    mip_aiding_vehicle_fixed_frame_velocity_command_valid_flags valid_flags; ///< Valid flags. Axes with 0 will be completely ignored.
    
};
typedef struct mip_aiding_vehicle_fixed_frame_velocity_command mip_aiding_vehicle_fixed_frame_velocity_command;
void insert_mip_aiding_vehicle_fixed_frame_velocity_command(struct mip_serializer* serializer, const mip_aiding_vehicle_fixed_frame_velocity_command* self);
void extract_mip_aiding_vehicle_fixed_frame_velocity_command(struct mip_serializer* serializer, mip_aiding_vehicle_fixed_frame_velocity_command* self);

void insert_mip_aiding_vehicle_fixed_frame_velocity_command_valid_flags(struct mip_serializer* serializer, const mip_aiding_vehicle_fixed_frame_velocity_command_valid_flags self);
void extract_mip_aiding_vehicle_fixed_frame_velocity_command_valid_flags(struct mip_serializer* serializer, mip_aiding_vehicle_fixed_frame_velocity_command_valid_flags* self);

mip_cmd_result mip_aiding_vehicle_fixed_frame_velocity(struct mip_interface* device, const mip_time* time, uint8_t frame_id, const float* velocity, const float* uncertainty, mip_aiding_vehicle_fixed_frame_velocity_command_valid_flags valid_flags);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_aiding_true_heading  (0x13,0x31) True Heading [C]
///
///@{

struct mip_aiding_true_heading_command
{
    mip_time time; ///< Timestamp of the measurement.
    uint8_t frame_id; ///< Source ID for this estimate (source_id == 0 indicates this sensor, source_id > 0 indicates an external estimate).
    float heading; ///< Heading [radians]. Range +/- Pi.
    float uncertainty; ///< Cannot be 0 unless the valid flags are 0.
    uint16_t valid_flags;
    
};
typedef struct mip_aiding_true_heading_command mip_aiding_true_heading_command;
void insert_mip_aiding_true_heading_command(struct mip_serializer* serializer, const mip_aiding_true_heading_command* self);
void extract_mip_aiding_true_heading_command(struct mip_serializer* serializer, mip_aiding_true_heading_command* self);

mip_cmd_result mip_aiding_true_heading(struct mip_interface* device, const mip_time* time, uint8_t frame_id, float heading, float uncertainty, uint16_t valid_flags);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_aiding_magnetic_field  (0x13,0x32) Magnetic Field [C]
/// Estimate of magnetic field in the frame associated with the given sensor ID.
///
///@{

typedef uint16_t mip_aiding_magnetic_field_command_valid_flags;
static const mip_aiding_magnetic_field_command_valid_flags MIP_AIDING_MAGNETIC_FIELD_COMMAND_VALID_FLAGS_NONE = 0x0000;
static const mip_aiding_magnetic_field_command_valid_flags MIP_AIDING_MAGNETIC_FIELD_COMMAND_VALID_FLAGS_X    = 0x0001; ///<  
static const mip_aiding_magnetic_field_command_valid_flags MIP_AIDING_MAGNETIC_FIELD_COMMAND_VALID_FLAGS_Y    = 0x0002; ///<  
static const mip_aiding_magnetic_field_command_valid_flags MIP_AIDING_MAGNETIC_FIELD_COMMAND_VALID_FLAGS_Z    = 0x0004; ///<  
static const mip_aiding_magnetic_field_command_valid_flags MIP_AIDING_MAGNETIC_FIELD_COMMAND_VALID_FLAGS_ALL  = 0x0007;

struct mip_aiding_magnetic_field_command
{
    mip_time time; ///< Timestamp of the measurement.
    uint8_t frame_id; ///< Source ID for this estimate (source_id == 0 indicates this sensor, source_id > 0 indicates an external estimate).
    mip_vector3f magnetic_field; ///< [G]
    mip_vector3f uncertainty; ///< [G] 1-sigma uncertainty. Cannot be 0 unless the corresponding valid flags are 0.
    mip_aiding_magnetic_field_command_valid_flags valid_flags; ///< Valid flags. Axes with 0 will be completely ignored.
    
};
typedef struct mip_aiding_magnetic_field_command mip_aiding_magnetic_field_command;
void insert_mip_aiding_magnetic_field_command(struct mip_serializer* serializer, const mip_aiding_magnetic_field_command* self);
void extract_mip_aiding_magnetic_field_command(struct mip_serializer* serializer, mip_aiding_magnetic_field_command* self);

void insert_mip_aiding_magnetic_field_command_valid_flags(struct mip_serializer* serializer, const mip_aiding_magnetic_field_command_valid_flags self);
void extract_mip_aiding_magnetic_field_command_valid_flags(struct mip_serializer* serializer, mip_aiding_magnetic_field_command_valid_flags* self);

mip_cmd_result mip_aiding_magnetic_field(struct mip_interface* device, const mip_time* time, uint8_t frame_id, const float* magnetic_field, const float* uncertainty, mip_aiding_magnetic_field_command_valid_flags valid_flags);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_aiding_pressure  (0x13,0x33) Pressure [C]
/// Estimated value of air pressure.
///
///@{

struct mip_aiding_pressure_command
{
    mip_time time; ///< Timestamp of the measurement.
    uint8_t frame_id; ///< Source ID for this estimate (source_id == 0 indicates this sensor, source_id > 0 indicates an external estimate).
    float pressure; ///< [mbar]
    float uncertainty; ///< [mbar] 1-sigma uncertainty. Cannot be 0 unless the valid flags are 0.
    uint16_t valid_flags;
    
};
typedef struct mip_aiding_pressure_command mip_aiding_pressure_command;
void insert_mip_aiding_pressure_command(struct mip_serializer* serializer, const mip_aiding_pressure_command* self);
void extract_mip_aiding_pressure_command(struct mip_serializer* serializer, mip_aiding_pressure_command* self);

mip_cmd_result mip_aiding_pressure(struct mip_interface* device, const mip_time* time, uint8_t frame_id, float pressure, float uncertainty, uint16_t valid_flags);

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


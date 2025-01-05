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
///@addtogroup MipData_c  MIP Data [C]
///@{
///@defgroup filter_data_c  Filter Data [C]
///
///@{

////////////////////////////////////////////////////////////////////////////////
// Descriptors
////////////////////////////////////////////////////////////////////////////////

enum 
{
    MIP_FILTER_DATA_DESC_SET                                         = 0x82,
    
    MIP_DATA_DESC_FILTER_POS_LLH                                     = 0x01,
    MIP_DATA_DESC_FILTER_VEL_NED                                     = 0x02,
    MIP_DATA_DESC_FILTER_ATT_QUATERNION                              = 0x03,
    MIP_DATA_DESC_FILTER_ATT_MATRIX                                  = 0x04,
    MIP_DATA_DESC_FILTER_ATT_EULER_ANGLES                            = 0x05,
    MIP_DATA_DESC_FILTER_GYRO_BIAS                                   = 0x06,
    MIP_DATA_DESC_FILTER_ACCEL_BIAS                                  = 0x07,
    MIP_DATA_DESC_FILTER_POS_UNCERTAINTY                             = 0x08,
    MIP_DATA_DESC_FILTER_VEL_UNCERTAINTY                             = 0x09,
    MIP_DATA_DESC_FILTER_ATT_UNCERTAINTY_EULER                       = 0x0A,
    MIP_DATA_DESC_FILTER_GYRO_BIAS_UNCERTAINTY                       = 0x0B,
    MIP_DATA_DESC_FILTER_ACCEL_BIAS_UNCERTAINTY                      = 0x0C,
    MIP_DATA_DESC_FILTER_LINEAR_ACCELERATION                         = 0x0D,
    MIP_DATA_DESC_FILTER_COMPENSATED_ANGULAR_RATE                    = 0x0E,
    MIP_DATA_DESC_FILTER_WGS84_GRAVITY                               = 0x0F,
    MIP_DATA_DESC_FILTER_FILTER_STATUS                               = 0x10,
    MIP_DATA_DESC_FILTER_FILTER_TIMESTAMP                            = 0x11,
    MIP_DATA_DESC_FILTER_ATT_UNCERTAINTY_QUATERNION                  = 0x12,
    MIP_DATA_DESC_FILTER_GRAVITY_VECTOR                              = 0x13,
    MIP_DATA_DESC_FILTER_HEADING_UPDATE_STATE                        = 0x14,
    MIP_DATA_DESC_FILTER_MAGNETIC_MODEL                              = 0x15,
    MIP_DATA_DESC_FILTER_GYRO_SCALE_FACTOR                           = 0x16,
    MIP_DATA_DESC_FILTER_ACCEL_SCALE_FACTOR                          = 0x17,
    MIP_DATA_DESC_FILTER_GYRO_SCALE_FACTOR_UNCERTAINTY               = 0x18,
    MIP_DATA_DESC_FILTER_ACCEL_SCALE_FACTOR_UNCERTAINTY              = 0x19,
    MIP_DATA_DESC_FILTER_MAG_BIAS                                    = 0x1A,
    MIP_DATA_DESC_FILTER_MAG_BIAS_UNCERTAINTY                        = 0x1B,
    MIP_DATA_DESC_FILTER_COMPENSATED_ACCELERATION                    = 0x1C,
    MIP_DATA_DESC_FILTER_STANDARD_ATMOSPHERE_DATA                    = 0x20,
    MIP_DATA_DESC_FILTER_PRESSURE_ALTITUDE_DATA                      = 0x21,
    MIP_DATA_DESC_FILTER_DENSITY_ALTITUDE_DATA                       = 0x22,
    MIP_DATA_DESC_FILTER_MAG_SCALE_FACTOR                            = 0x23,
    MIP_DATA_DESC_FILTER_MAG_SCALE_FACTOR_UNCERTAINTY                = 0x24,
    MIP_DATA_DESC_FILTER_MAG_COMPENSATION_OFFSET                     = 0x25,
    MIP_DATA_DESC_FILTER_MAG_COMPENSATION_MATRIX                     = 0x26,
    MIP_DATA_DESC_FILTER_COMPENSATED_MAGNETOMETER                    = 0x27,
    MIP_DATA_DESC_FILTER_MAG_COMPENSATION_OFFSET_UNCERTAINTY         = 0x28,
    MIP_DATA_DESC_FILTER_MAG_COMPENSATION_MATRIX_UNCERTAINTY         = 0x29,
    MIP_DATA_DESC_FILTER_MAG_COVARIANCE                              = 0x2A,
    MIP_DATA_DESC_FILTER_GRAVITY_COVARIANCE                          = 0x2B,
    MIP_DATA_DESC_FILTER_MAG_RESIDUAL                                = 0x2C,
    MIP_DATA_DESC_FILTER_MAG_FILTERED_RESIDUAL                       = 0x2D,
    MIP_DATA_DESC_FILTER_ANTENNA_OFFSET_CORRECTION                   = 0x30,
    MIP_DATA_DESC_FILTER_ANTENNA_OFFSET_CORRECTION_UNCERTAINTY       = 0x31,
    MIP_DATA_DESC_FILTER_CLOCK_CORRECTION                            = 0x32,
    MIP_DATA_DESC_FILTER_CLOCK_CORRECTION_UNCERTAINTY                = 0x33,
    MIP_DATA_DESC_FILTER_MULTI_ANTENNA_OFFSET_CORRECTION             = 0x34,
    MIP_DATA_DESC_FILTER_MULTI_ANTENNA_OFFSET_CORRECTION_UNCERTAINTY = 0x35,
    MIP_DATA_DESC_FILTER_ECEF_POS_UNCERTAINTY                        = 0x36,
    MIP_DATA_DESC_FILTER_ECEF_VEL_UNCERTAINTY                        = 0x37,
    MIP_DATA_DESC_FILTER_ECEF_POS                                    = 0x40,
    MIP_DATA_DESC_FILTER_ECEF_VEL                                    = 0x41,
    MIP_DATA_DESC_FILTER_REL_POS_NED                                 = 0x42,
    MIP_DATA_DESC_FILTER_GNSS_POS_AID_STATUS                         = 0x43,
    MIP_DATA_DESC_FILTER_GNSS_ATT_AID_STATUS                         = 0x44,
    MIP_DATA_DESC_FILTER_HEAD_AID_STATUS                             = 0x45,
    MIP_DATA_DESC_FILTER_AID_MEAS_SUMMARY                            = 0x46,
    MIP_DATA_DESC_FILTER_ODOMETER_SCALE_FACTOR_ERROR                 = 0x47,
    MIP_DATA_DESC_FILTER_ODOMETER_SCALE_FACTOR_ERROR_UNCERTAINTY     = 0x48,
    MIP_DATA_DESC_FILTER_GNSS_DUAL_ANTENNA_STATUS                    = 0x49,
    MIP_DATA_DESC_FILTER_FRAME_CONFIG_ERROR                          = 0x50,
    MIP_DATA_DESC_FILTER_FRAME_CONFIG_ERROR_UNCERTAINTY              = 0x51,
    
};

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

typedef uint16_t mip_filter_mode;
static const mip_filter_mode MIP_FILTER_MODE_GX5_STARTUP            = 0; ///<  
static const mip_filter_mode MIP_FILTER_MODE_GX5_INIT               = 1; ///<  
static const mip_filter_mode MIP_FILTER_MODE_GX5_RUN_SOLUTION_VALID = 2; ///<  
static const mip_filter_mode MIP_FILTER_MODE_GX5_RUN_SOLUTION_ERROR = 3; ///<  
static const mip_filter_mode MIP_FILTER_MODE_INIT                   = 1; ///<  
static const mip_filter_mode MIP_FILTER_MODE_VERT_GYRO              = 2; ///<  
static const mip_filter_mode MIP_FILTER_MODE_AHRS                   = 3; ///<  
static const mip_filter_mode MIP_FILTER_MODE_FULL_NAV               = 4; ///<  

void insert_mip_filter_mode(struct mip_serializer* serializer, const mip_filter_mode self);
void extract_mip_filter_mode(struct mip_serializer* serializer, mip_filter_mode* self);

typedef uint16_t mip_filter_dynamics_mode;
static const mip_filter_dynamics_mode MIP_FILTER_DYNAMICS_MODE_GX5_PORTABLE   = 1; ///<  
static const mip_filter_dynamics_mode MIP_FILTER_DYNAMICS_MODE_GX5_AUTOMOTIVE = 2; ///<  
static const mip_filter_dynamics_mode MIP_FILTER_DYNAMICS_MODE_GX5_AIRBORNE   = 3; ///<  
static const mip_filter_dynamics_mode MIP_FILTER_DYNAMICS_MODE_GQ7_DEFAULT    = 1; ///<  

void insert_mip_filter_dynamics_mode(struct mip_serializer* serializer, const mip_filter_dynamics_mode self);
void extract_mip_filter_dynamics_mode(struct mip_serializer* serializer, mip_filter_dynamics_mode* self);

typedef uint16_t mip_filter_status_flags;
static const mip_filter_status_flags MIP_FILTER_STATUS_FLAGS_NONE                                           = 0x0000;
static const mip_filter_status_flags MIP_FILTER_STATUS_FLAGS_GX5_INIT_NO_ATTITUDE                           = 0x1000; ///<  
static const mip_filter_status_flags MIP_FILTER_STATUS_FLAGS_GX5_INIT_NO_POSITION_VELOCITY                  = 0x2000; ///<  
static const mip_filter_status_flags MIP_FILTER_STATUS_FLAGS_GX5_RUN_IMU_UNAVAILABLE                        = 0x0001; ///<  
static const mip_filter_status_flags MIP_FILTER_STATUS_FLAGS_GX5_RUN_GPS_UNAVAILABLE                        = 0x0002; ///<  
static const mip_filter_status_flags MIP_FILTER_STATUS_FLAGS_GX5_RUN_MATRIX_SINGULARITY                     = 0x0008; ///<  
static const mip_filter_status_flags MIP_FILTER_STATUS_FLAGS_GX5_RUN_POSITION_COVARIANCE_WARNING            = 0x0010; ///<  
static const mip_filter_status_flags MIP_FILTER_STATUS_FLAGS_GX5_RUN_VELOCITY_COVARIANCE_WARNING            = 0x0020; ///<  
static const mip_filter_status_flags MIP_FILTER_STATUS_FLAGS_GX5_RUN_ATTITUDE_COVARIANCE_WARNING            = 0x0040; ///<  
static const mip_filter_status_flags MIP_FILTER_STATUS_FLAGS_GX5_RUN_NAN_IN_SOLUTION_WARNING                = 0x0080; ///<  
static const mip_filter_status_flags MIP_FILTER_STATUS_FLAGS_GX5_RUN_GYRO_BIAS_EST_HIGH_WARNING             = 0x0100; ///<  
static const mip_filter_status_flags MIP_FILTER_STATUS_FLAGS_GX5_RUN_ACCEL_BIAS_EST_HIGH_WARNING            = 0x0200; ///<  
static const mip_filter_status_flags MIP_FILTER_STATUS_FLAGS_GX5_RUN_GYRO_SCALE_FACTOR_EST_HIGH_WARNING     = 0x0400; ///<  
static const mip_filter_status_flags MIP_FILTER_STATUS_FLAGS_GX5_RUN_ACCEL_SCALE_FACTOR_EST_HIGH_WARNING    = 0x0800; ///<  
static const mip_filter_status_flags MIP_FILTER_STATUS_FLAGS_GX5_RUN_MAG_BIAS_EST_HIGH_WARNING              = 0x1000; ///<  
static const mip_filter_status_flags MIP_FILTER_STATUS_FLAGS_GX5_RUN_ANT_OFFSET_CORRECTION_EST_HIGH_WARNING = 0x2000; ///<  
static const mip_filter_status_flags MIP_FILTER_STATUS_FLAGS_GX5_RUN_MAG_HARD_IRON_EST_HIGH_WARNING         = 0x4000; ///<  
static const mip_filter_status_flags MIP_FILTER_STATUS_FLAGS_GX5_RUN_MAG_SOFT_IRON_EST_HIGH_WARNING         = 0x8000; ///<  
static const mip_filter_status_flags MIP_FILTER_STATUS_FLAGS_GQ7_FILTER_CONDITION                           = 0x0003; ///<  
static const mip_filter_status_flags MIP_FILTER_STATUS_FLAGS_GQ7_ROLL_PITCH_WARNING                         = 0x0004; ///<  
static const mip_filter_status_flags MIP_FILTER_STATUS_FLAGS_GQ7_HEADING_WARNING                            = 0x0008; ///<  
static const mip_filter_status_flags MIP_FILTER_STATUS_FLAGS_GQ7_POSITION_WARNING                           = 0x0010; ///<  
static const mip_filter_status_flags MIP_FILTER_STATUS_FLAGS_GQ7_VELOCITY_WARNING                           = 0x0020; ///<  
static const mip_filter_status_flags MIP_FILTER_STATUS_FLAGS_GQ7_IMU_BIAS_WARNING                           = 0x0040; ///<  
static const mip_filter_status_flags MIP_FILTER_STATUS_FLAGS_GQ7_GNSS_CLK_WARNING                           = 0x0080; ///<  
static const mip_filter_status_flags MIP_FILTER_STATUS_FLAGS_GQ7_ANTENNA_LEVER_ARM_WARNING                  = 0x0100; ///<  
static const mip_filter_status_flags MIP_FILTER_STATUS_FLAGS_GQ7_MOUNTING_TRANSFORM_WARNING                 = 0x0200; ///<  
static const mip_filter_status_flags MIP_FILTER_STATUS_FLAGS_GQ7_TIME_SYNC_WARNING                          = 0x0400; ///<  No time synchronization pulse detected
static const mip_filter_status_flags MIP_FILTER_STATUS_FLAGS_GQ7_SOLUTION_ERROR                             = 0xF000; ///<  Filter computation warning flags. If any bits 12-15 are set, and all filter outputs will be invalid.
static const mip_filter_status_flags MIP_FILTER_STATUS_FLAGS_ALL                                            = 0xFFFF;

void insert_mip_filter_status_flags(struct mip_serializer* serializer, const mip_filter_status_flags self);
void extract_mip_filter_status_flags(struct mip_serializer* serializer, mip_filter_status_flags* self);

typedef uint8_t mip_filter_aiding_measurement_type;
static const mip_filter_aiding_measurement_type MIP_FILTER_AIDING_MEASUREMENT_TYPE_GNSS              = 1;  ///<  
static const mip_filter_aiding_measurement_type MIP_FILTER_AIDING_MEASUREMENT_TYPE_DUAL_ANTENNA      = 2;  ///<  
static const mip_filter_aiding_measurement_type MIP_FILTER_AIDING_MEASUREMENT_TYPE_HEADING           = 3;  ///<  
static const mip_filter_aiding_measurement_type MIP_FILTER_AIDING_MEASUREMENT_TYPE_PRESSURE          = 4;  ///<  
static const mip_filter_aiding_measurement_type MIP_FILTER_AIDING_MEASUREMENT_TYPE_MAGNETOMETER      = 5;  ///<  
static const mip_filter_aiding_measurement_type MIP_FILTER_AIDING_MEASUREMENT_TYPE_SPEED             = 6;  ///<  
static const mip_filter_aiding_measurement_type MIP_FILTER_AIDING_MEASUREMENT_TYPE_POS_ECEF          = 33; ///<  
static const mip_filter_aiding_measurement_type MIP_FILTER_AIDING_MEASUREMENT_TYPE_POS_LLH           = 34; ///<  
static const mip_filter_aiding_measurement_type MIP_FILTER_AIDING_MEASUREMENT_TYPE_VEL_ECEF          = 40; ///<  
static const mip_filter_aiding_measurement_type MIP_FILTER_AIDING_MEASUREMENT_TYPE_VEL_NED           = 41; ///<  
static const mip_filter_aiding_measurement_type MIP_FILTER_AIDING_MEASUREMENT_TYPE_VEL_VEHICLE_FRAME = 42; ///<  
static const mip_filter_aiding_measurement_type MIP_FILTER_AIDING_MEASUREMENT_TYPE_HEADING_TRUE      = 49; ///<  

void insert_mip_filter_aiding_measurement_type(struct mip_serializer* serializer, const mip_filter_aiding_measurement_type self);
void extract_mip_filter_aiding_measurement_type(struct mip_serializer* serializer, mip_filter_aiding_measurement_type* self);

typedef uint8_t mip_filter_measurement_indicator;
static const mip_filter_measurement_indicator MIP_FILTER_MEASUREMENT_INDICATOR_NONE                  = 0x00;
static const mip_filter_measurement_indicator MIP_FILTER_MEASUREMENT_INDICATOR_ENABLED               = 0x01; ///<  
static const mip_filter_measurement_indicator MIP_FILTER_MEASUREMENT_INDICATOR_USED                  = 0x02; ///<  
static const mip_filter_measurement_indicator MIP_FILTER_MEASUREMENT_INDICATOR_RESIDUAL_HIGH_WARNING = 0x04; ///<  
static const mip_filter_measurement_indicator MIP_FILTER_MEASUREMENT_INDICATOR_SAMPLE_TIME_WARNING   = 0x08; ///<  
static const mip_filter_measurement_indicator MIP_FILTER_MEASUREMENT_INDICATOR_CONFIGURATION_ERROR   = 0x10; ///<  
static const mip_filter_measurement_indicator MIP_FILTER_MEASUREMENT_INDICATOR_MAX_NUM_MEAS_EXCEEDED = 0x20; ///<  
static const mip_filter_measurement_indicator MIP_FILTER_MEASUREMENT_INDICATOR_ALL                   = 0x3F;

void insert_mip_filter_measurement_indicator(struct mip_serializer* serializer, const mip_filter_measurement_indicator self);
void extract_mip_filter_measurement_indicator(struct mip_serializer* serializer, mip_filter_measurement_indicator* self);

typedef uint16_t mip_gnss_aid_status_flags;
static const mip_gnss_aid_status_flags MIP_GNSS_AID_STATUS_FLAGS_NONE           = 0x0000;
static const mip_gnss_aid_status_flags MIP_GNSS_AID_STATUS_FLAGS_TIGHT_COUPLING = 0x0001; ///<  If 1, the Kalman filter is processing raw range information from this GNSS module
static const mip_gnss_aid_status_flags MIP_GNSS_AID_STATUS_FLAGS_DIFFERENTIAL   = 0x0002; ///<  If 1, the Kalman filter is processing RTK corrections from this GNSS module
static const mip_gnss_aid_status_flags MIP_GNSS_AID_STATUS_FLAGS_INTEGER_FIX    = 0x0004; ///<  If 1, the Kalman filter has an RTK integer fix from this GNSS module, indicating the best position performance possible
static const mip_gnss_aid_status_flags MIP_GNSS_AID_STATUS_FLAGS_GPS_L1         = 0x0008; ///<  If 1, the Kalman filter is using GPS L1 measurements
static const mip_gnss_aid_status_flags MIP_GNSS_AID_STATUS_FLAGS_GPS_L2         = 0x0010; ///<  If 1, the Kalman filter is using GPS L2 measurements
static const mip_gnss_aid_status_flags MIP_GNSS_AID_STATUS_FLAGS_GPS_L5         = 0x0020; ///<  If 1, the Kalman filter is using GPS L5 measurements (not available on the GQ7)
static const mip_gnss_aid_status_flags MIP_GNSS_AID_STATUS_FLAGS_GLO_L1         = 0x0040; ///<  If 1, the Kalman filter is using GLONASS L1 measurements
static const mip_gnss_aid_status_flags MIP_GNSS_AID_STATUS_FLAGS_GLO_L2         = 0x0080; ///<  If 1, the Kalman filter is using GLONASS L2 measurements
static const mip_gnss_aid_status_flags MIP_GNSS_AID_STATUS_FLAGS_GAL_E1         = 0x0100; ///<  If 1, the Kalman filter is using Galileo E1 measurements
static const mip_gnss_aid_status_flags MIP_GNSS_AID_STATUS_FLAGS_GAL_E5         = 0x0200; ///<  If 1, the Kalman filter is using Galileo E5 measurements
static const mip_gnss_aid_status_flags MIP_GNSS_AID_STATUS_FLAGS_GAL_E6         = 0x0400; ///<  If 1, the Kalman filter is using Galileo E6 measurements
static const mip_gnss_aid_status_flags MIP_GNSS_AID_STATUS_FLAGS_BEI_B1         = 0x0800; ///<  If 1, the Kalman filter is using Beidou B1 measurements (not enabled on GQ7 currently)
static const mip_gnss_aid_status_flags MIP_GNSS_AID_STATUS_FLAGS_BEI_B2         = 0x1000; ///<  If 1, the Kalman filter is using Beidou B2 measurements (not enabled on GQ7 currently)
static const mip_gnss_aid_status_flags MIP_GNSS_AID_STATUS_FLAGS_BEI_B3         = 0x2000; ///<  If 1, the Kalman filter is using Beidou B3 measurements (not available on the GQ7)
static const mip_gnss_aid_status_flags MIP_GNSS_AID_STATUS_FLAGS_NO_FIX         = 0x4000; ///<  If 1, this GNSS module is reporting no position fix
static const mip_gnss_aid_status_flags MIP_GNSS_AID_STATUS_FLAGS_CONFIG_ERROR   = 0x8000; ///<  If 1, there is likely an issue with the antenna offset for this GNSS module
static const mip_gnss_aid_status_flags MIP_GNSS_AID_STATUS_FLAGS_ALL            = 0xFFFF;

void insert_mip_gnss_aid_status_flags(struct mip_serializer* serializer, const mip_gnss_aid_status_flags self);
void extract_mip_gnss_aid_status_flags(struct mip_serializer* serializer, mip_gnss_aid_status_flags* self);


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_position_llh  (0x82,0x01) Position Llh [C]
/// Filter reported position in the WGS84 geodetic frame.
///
///@{

struct mip_filter_position_llh_data
{
    double latitude; ///< [degrees]
    double longitude; ///< [degrees]
    double ellipsoid_height; ///< [meters]
    uint16_t valid_flags; ///< 0 - Invalid, 1 - valid
    
};
typedef struct mip_filter_position_llh_data mip_filter_position_llh_data;
void insert_mip_filter_position_llh_data(struct mip_serializer* serializer, const mip_filter_position_llh_data* self);
void extract_mip_filter_position_llh_data(struct mip_serializer* serializer, mip_filter_position_llh_data* self);
bool extract_mip_filter_position_llh_data_from_field(const struct mip_field* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_velocity_ned  (0x82,0x02) Velocity Ned [C]
/// Filter reported velocity in the NED local-level frame.
///
///@{

struct mip_filter_velocity_ned_data
{
    float north; ///< [meters/second]
    float east; ///< [meters/second]
    float down; ///< [meters/second]
    uint16_t valid_flags; ///< 0 - Invalid, 1 - valid
    
};
typedef struct mip_filter_velocity_ned_data mip_filter_velocity_ned_data;
void insert_mip_filter_velocity_ned_data(struct mip_serializer* serializer, const mip_filter_velocity_ned_data* self);
void extract_mip_filter_velocity_ned_data(struct mip_serializer* serializer, mip_filter_velocity_ned_data* self);
bool extract_mip_filter_velocity_ned_data_from_field(const struct mip_field* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_attitude_quaternion  (0x82,0x03) Attitude Quaternion [C]
/// 4x1 vector representation of the quaternion describing the orientation of the device with respect to the NED local-level frame.
/// This quaternion satisfies the following relationship:
/// 
/// EQSTART p^{veh} = q^{-1} p^{ned} q EQEND<br/>
/// 
/// Where:<br/>
/// EQSTART q = (q_w, q_x, q_y, q_z) EQEND is the quaternion describing the rotation. <br/>
/// EQSTART p^ned = (0, v^{ned}_x, v^{ned}_y, v^{ned}_z) EQEND and EQSTART v^{ned} EQEND is a 3-element vector expressed in the NED frame.<br/>
/// EQSTART p^veh = (0, v^{veh}_x, v^{veh}_y, v^{veh}_z) EQEND and EQSTART v^{veh} EQEND is a 3-element vector expressed in the vehicle frame.<br/>
///
///@{

struct mip_filter_attitude_quaternion_data
{
    mip_quatf q; ///< Quaternion elements EQSTART q = (q_w, q_x, q_y, q_z) EQEND
    uint16_t valid_flags; ///< 0 - invalid, 1 - valid
    
};
typedef struct mip_filter_attitude_quaternion_data mip_filter_attitude_quaternion_data;
void insert_mip_filter_attitude_quaternion_data(struct mip_serializer* serializer, const mip_filter_attitude_quaternion_data* self);
void extract_mip_filter_attitude_quaternion_data(struct mip_serializer* serializer, mip_filter_attitude_quaternion_data* self);
bool extract_mip_filter_attitude_quaternion_data_from_field(const struct mip_field* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_attitude_dcm  (0x82,0x04) Attitude Dcm [C]
/// 3x3 Direction Cosine Matrix EQSTART M_{ned}^{veh} EQEND describing the orientation of the device with respect to the NED local-level frame.
/// This matrix satisfies the following relationship:
/// 
/// EQSTART v^{veh} = M_{ned}^{veh} v^{ned} EQEND<br/>
/// 
/// Where:<br/>
/// 
/// EQSTART v^{ned} EQEND is a 3-element vector expressed in the NED frame. <br/>
/// EQSTART v^{veh} EQEND is the same 3-element vector expressed in the vehicle frame.  <br/>
/// <br/>
/// The matrix elements are stored is row-major order: EQSTART M_{ned}^{veh} = \begin{bmatrix} M_{11}, M_{12}, M_{13}, M_{21}, M_{22}, M_{23}, M_{31}, M_{32}, M_{33} \end{bmatrix} EQEND
///
///@{

struct mip_filter_attitude_dcm_data
{
    mip_matrix3f dcm; ///< Matrix elements in row-major order.
    uint16_t valid_flags; ///< 0 - invalid, 1 - valid
    
};
typedef struct mip_filter_attitude_dcm_data mip_filter_attitude_dcm_data;
void insert_mip_filter_attitude_dcm_data(struct mip_serializer* serializer, const mip_filter_attitude_dcm_data* self);
void extract_mip_filter_attitude_dcm_data(struct mip_serializer* serializer, mip_filter_attitude_dcm_data* self);
bool extract_mip_filter_attitude_dcm_data_from_field(const struct mip_field* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_euler_angles  (0x82,0x05) Euler Angles [C]
/// Filter reported Euler angles describing the orientation of the device with respect to the NED local-level frame.
/// The Euler angles are reported in 3-2-1 (Yaw-Pitch-Roll, AKA Aircraft) order.
///
///@{

struct mip_filter_euler_angles_data
{
    float roll; ///< [radians]
    float pitch; ///< [radians]
    float yaw; ///< [radians]
    uint16_t valid_flags; ///< 0 - invalid, 1 - valid
    
};
typedef struct mip_filter_euler_angles_data mip_filter_euler_angles_data;
void insert_mip_filter_euler_angles_data(struct mip_serializer* serializer, const mip_filter_euler_angles_data* self);
void extract_mip_filter_euler_angles_data(struct mip_serializer* serializer, mip_filter_euler_angles_data* self);
bool extract_mip_filter_euler_angles_data_from_field(const struct mip_field* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_gyro_bias  (0x82,0x06) Gyro Bias [C]
/// Filter reported gyro bias expressed in the sensor frame.
///
///@{

struct mip_filter_gyro_bias_data
{
    mip_vector3f bias; ///< (x, y, z) [radians/second]
    uint16_t valid_flags; ///< 0 - invalid, 1 - valid
    
};
typedef struct mip_filter_gyro_bias_data mip_filter_gyro_bias_data;
void insert_mip_filter_gyro_bias_data(struct mip_serializer* serializer, const mip_filter_gyro_bias_data* self);
void extract_mip_filter_gyro_bias_data(struct mip_serializer* serializer, mip_filter_gyro_bias_data* self);
bool extract_mip_filter_gyro_bias_data_from_field(const struct mip_field* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_accel_bias  (0x82,0x07) Accel Bias [C]
/// Filter reported accelerometer bias expressed in the sensor frame.
///
///@{

struct mip_filter_accel_bias_data
{
    mip_vector3f bias; ///< (x, y, z) [meters/second^2]
    uint16_t valid_flags; ///< 0 - invalid, 1 - valid
    
};
typedef struct mip_filter_accel_bias_data mip_filter_accel_bias_data;
void insert_mip_filter_accel_bias_data(struct mip_serializer* serializer, const mip_filter_accel_bias_data* self);
void extract_mip_filter_accel_bias_data(struct mip_serializer* serializer, mip_filter_accel_bias_data* self);
bool extract_mip_filter_accel_bias_data_from_field(const struct mip_field* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_position_llh_uncertainty  (0x82,0x08) Position Llh Uncertainty [C]
/// Filter reported 1-sigma position uncertainty in the NED local-level frame.
///
///@{

struct mip_filter_position_llh_uncertainty_data
{
    float north; ///< [meters]
    float east; ///< [meters]
    float down; ///< [meters]
    uint16_t valid_flags; ///< 0 - invalid, 1 - valid
    
};
typedef struct mip_filter_position_llh_uncertainty_data mip_filter_position_llh_uncertainty_data;
void insert_mip_filter_position_llh_uncertainty_data(struct mip_serializer* serializer, const mip_filter_position_llh_uncertainty_data* self);
void extract_mip_filter_position_llh_uncertainty_data(struct mip_serializer* serializer, mip_filter_position_llh_uncertainty_data* self);
bool extract_mip_filter_position_llh_uncertainty_data_from_field(const struct mip_field* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_velocity_ned_uncertainty  (0x82,0x09) Velocity Ned Uncertainty [C]
/// Filter reported 1-sigma velocity uncertainties in the NED local-level frame.
///
///@{

struct mip_filter_velocity_ned_uncertainty_data
{
    float north; ///< [meters/second]
    float east; ///< [meters/second]
    float down; ///< [meters/second]
    uint16_t valid_flags; ///< 0 - invalid, 1 - valid
    
};
typedef struct mip_filter_velocity_ned_uncertainty_data mip_filter_velocity_ned_uncertainty_data;
void insert_mip_filter_velocity_ned_uncertainty_data(struct mip_serializer* serializer, const mip_filter_velocity_ned_uncertainty_data* self);
void extract_mip_filter_velocity_ned_uncertainty_data(struct mip_serializer* serializer, mip_filter_velocity_ned_uncertainty_data* self);
bool extract_mip_filter_velocity_ned_uncertainty_data_from_field(const struct mip_field* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_euler_angles_uncertainty  (0x82,0x0A) Euler Angles Uncertainty [C]
/// Filter reported 1-sigma Euler angle uncertainties.
/// The uncertainties are reported in 3-2-1 (Yaw-Pitch-Roll, AKA Aircraft) order.
///
///@{

struct mip_filter_euler_angles_uncertainty_data
{
    float roll; ///< [radians]
    float pitch; ///< [radians]
    float yaw; ///< [radians]
    uint16_t valid_flags; ///< 0 - invalid, 1 - valid
    
};
typedef struct mip_filter_euler_angles_uncertainty_data mip_filter_euler_angles_uncertainty_data;
void insert_mip_filter_euler_angles_uncertainty_data(struct mip_serializer* serializer, const mip_filter_euler_angles_uncertainty_data* self);
void extract_mip_filter_euler_angles_uncertainty_data(struct mip_serializer* serializer, mip_filter_euler_angles_uncertainty_data* self);
bool extract_mip_filter_euler_angles_uncertainty_data_from_field(const struct mip_field* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_gyro_bias_uncertainty  (0x82,0x0B) Gyro Bias Uncertainty [C]
/// Filter reported 1-sigma gyro bias uncertainties expressed in the sensor frame.
///
///@{

struct mip_filter_gyro_bias_uncertainty_data
{
    mip_vector3f bias_uncert; ///< (x,y,z) [radians/sec]
    uint16_t valid_flags; ///< 0 - invalid, 1 - valid
    
};
typedef struct mip_filter_gyro_bias_uncertainty_data mip_filter_gyro_bias_uncertainty_data;
void insert_mip_filter_gyro_bias_uncertainty_data(struct mip_serializer* serializer, const mip_filter_gyro_bias_uncertainty_data* self);
void extract_mip_filter_gyro_bias_uncertainty_data(struct mip_serializer* serializer, mip_filter_gyro_bias_uncertainty_data* self);
bool extract_mip_filter_gyro_bias_uncertainty_data_from_field(const struct mip_field* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_accel_bias_uncertainty  (0x82,0x0C) Accel Bias Uncertainty [C]
/// Filter reported 1-sigma accelerometer bias uncertainties expressed in the sensor frame.
///
///@{

struct mip_filter_accel_bias_uncertainty_data
{
    mip_vector3f bias_uncert; ///< (x,y,z) [meters/second^2]
    uint16_t valid_flags; ///< 0 - invalid, 1 - valid
    
};
typedef struct mip_filter_accel_bias_uncertainty_data mip_filter_accel_bias_uncertainty_data;
void insert_mip_filter_accel_bias_uncertainty_data(struct mip_serializer* serializer, const mip_filter_accel_bias_uncertainty_data* self);
void extract_mip_filter_accel_bias_uncertainty_data(struct mip_serializer* serializer, mip_filter_accel_bias_uncertainty_data* self);
bool extract_mip_filter_accel_bias_uncertainty_data_from_field(const struct mip_field* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_timestamp  (0x82,0x11) Timestamp [C]
/// GPS timestamp of the Filter data
/// 
/// Should the PPS become unavailable, the device will revert to its internal clock, which will cause the reported time to drift from true GPS time.
/// Upon recovering from a PPS outage, the user should expect a jump in the reported GPS time due to the accumulation of internal clock error.
/// If synchronization to an external clock or onboard GNSS receiver (for products that have one) is disabled, this time is equivalent to internal system time.
/// 
/// Note: this data field may be deprecated in the future. The more flexible shared data field (0x82, 0xD3) should be used instead.
///
///@{

struct mip_filter_timestamp_data
{
    double tow; ///< GPS Time of Week [seconds]
    uint16_t week_number; ///< GPS Week Number since 1980 [weeks]
    uint16_t valid_flags; ///< 0 - invalid, 1 - valid
    
};
typedef struct mip_filter_timestamp_data mip_filter_timestamp_data;
void insert_mip_filter_timestamp_data(struct mip_serializer* serializer, const mip_filter_timestamp_data* self);
void extract_mip_filter_timestamp_data(struct mip_serializer* serializer, mip_filter_timestamp_data* self);
bool extract_mip_filter_timestamp_data_from_field(const struct mip_field* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_status  (0x82,0x10) Status [C]
/// Device-specific filter status indicators.
///
///@{

struct mip_filter_status_data
{
    mip_filter_mode filter_state; ///< Device-specific filter state.  Please consult the user manual for definition.
    mip_filter_dynamics_mode dynamics_mode; ///< Device-specific dynamics mode. Please consult the user manual for definition.
    mip_filter_status_flags status_flags; ///< Device-specific status flags.  Please consult the user manual for definition.
    
};
typedef struct mip_filter_status_data mip_filter_status_data;
void insert_mip_filter_status_data(struct mip_serializer* serializer, const mip_filter_status_data* self);
void extract_mip_filter_status_data(struct mip_serializer* serializer, mip_filter_status_data* self);
bool extract_mip_filter_status_data_from_field(const struct mip_field* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_linear_accel  (0x82,0x0D) Linear Accel [C]
/// Filter-compensated linear acceleration expressed in the vehicle frame.
/// Note: The estimated gravity has been removed from this data leaving only linear acceleration.
///
///@{

struct mip_filter_linear_accel_data
{
    mip_vector3f accel; ///< (x,y,z) [meters/second^2]
    uint16_t valid_flags; ///< 0 - invalid, 1 - valid
    
};
typedef struct mip_filter_linear_accel_data mip_filter_linear_accel_data;
void insert_mip_filter_linear_accel_data(struct mip_serializer* serializer, const mip_filter_linear_accel_data* self);
void extract_mip_filter_linear_accel_data(struct mip_serializer* serializer, mip_filter_linear_accel_data* self);
bool extract_mip_filter_linear_accel_data_from_field(const struct mip_field* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_gravity_vector  (0x82,0x13) Gravity Vector [C]
/// Filter reported gravity vector expressed in the vehicle frame.
///
///@{

struct mip_filter_gravity_vector_data
{
    mip_vector3f gravity; ///< (x, y, z) [meters/second^2]
    uint16_t valid_flags; ///< 0 - invalid, 1 - valid
    
};
typedef struct mip_filter_gravity_vector_data mip_filter_gravity_vector_data;
void insert_mip_filter_gravity_vector_data(struct mip_serializer* serializer, const mip_filter_gravity_vector_data* self);
void extract_mip_filter_gravity_vector_data(struct mip_serializer* serializer, mip_filter_gravity_vector_data* self);
bool extract_mip_filter_gravity_vector_data_from_field(const struct mip_field* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_comp_accel  (0x82,0x1C) Comp Accel [C]
/// Filter-compensated acceleration expressed in the vehicle frame.
///
///@{

struct mip_filter_comp_accel_data
{
    mip_vector3f accel; ///< (x,y,z) [meters/second^2]
    uint16_t valid_flags; ///< 0 - invalid, 1 - valid
    
};
typedef struct mip_filter_comp_accel_data mip_filter_comp_accel_data;
void insert_mip_filter_comp_accel_data(struct mip_serializer* serializer, const mip_filter_comp_accel_data* self);
void extract_mip_filter_comp_accel_data(struct mip_serializer* serializer, mip_filter_comp_accel_data* self);
bool extract_mip_filter_comp_accel_data_from_field(const struct mip_field* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_comp_angular_rate  (0x82,0x0E) Comp Angular Rate [C]
/// Filter-compensated angular rate expressed in the vehicle frame.
///
///@{

struct mip_filter_comp_angular_rate_data
{
    mip_vector3f gyro; ///< (x, y, z) [radians/second]
    uint16_t valid_flags; ///< 0 - invalid, 1 - valid
    
};
typedef struct mip_filter_comp_angular_rate_data mip_filter_comp_angular_rate_data;
void insert_mip_filter_comp_angular_rate_data(struct mip_serializer* serializer, const mip_filter_comp_angular_rate_data* self);
void extract_mip_filter_comp_angular_rate_data(struct mip_serializer* serializer, mip_filter_comp_angular_rate_data* self);
bool extract_mip_filter_comp_angular_rate_data_from_field(const struct mip_field* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_quaternion_attitude_uncertainty  (0x82,0x12) Quaternion Attitude Uncertainty [C]
/// Filter reported quaternion uncertainties.
///
///@{

struct mip_filter_quaternion_attitude_uncertainty_data
{
    mip_quatf q; ///< [dimensionless]
    uint16_t valid_flags; ///< 0 - invalid, 1 - valid
    
};
typedef struct mip_filter_quaternion_attitude_uncertainty_data mip_filter_quaternion_attitude_uncertainty_data;
void insert_mip_filter_quaternion_attitude_uncertainty_data(struct mip_serializer* serializer, const mip_filter_quaternion_attitude_uncertainty_data* self);
void extract_mip_filter_quaternion_attitude_uncertainty_data(struct mip_serializer* serializer, mip_filter_quaternion_attitude_uncertainty_data* self);
bool extract_mip_filter_quaternion_attitude_uncertainty_data_from_field(const struct mip_field* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_wgs84_gravity_mag  (0x82,0x0F) Wgs84 Gravity Mag [C]
/// Filter reported WGS84 gravity magnitude.
///
///@{

struct mip_filter_wgs84_gravity_mag_data
{
    float magnitude; ///< [meters/second^2]
    uint16_t valid_flags; ///< 0 - invalid, 1 - valid
    
};
typedef struct mip_filter_wgs84_gravity_mag_data mip_filter_wgs84_gravity_mag_data;
void insert_mip_filter_wgs84_gravity_mag_data(struct mip_serializer* serializer, const mip_filter_wgs84_gravity_mag_data* self);
void extract_mip_filter_wgs84_gravity_mag_data(struct mip_serializer* serializer, mip_filter_wgs84_gravity_mag_data* self);
bool extract_mip_filter_wgs84_gravity_mag_data_from_field(const struct mip_field* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_heading_update_state  (0x82,0x14) Heading Update State [C]
/// Filter reported heading update state.
/// 
/// Heading updates can be applied from the sources listed below.  Note, some of these sources may be combined.
/// The heading value is always relative to true north.
///
///@{

typedef uint16_t mip_filter_heading_update_state_data_heading_source;
static const mip_filter_heading_update_state_data_heading_source MIP_FILTER_HEADING_UPDATE_STATE_DATA_HEADING_SOURCE_NONE                 = 0; ///<  
static const mip_filter_heading_update_state_data_heading_source MIP_FILTER_HEADING_UPDATE_STATE_DATA_HEADING_SOURCE_MAGNETOMETER         = 1; ///<  
static const mip_filter_heading_update_state_data_heading_source MIP_FILTER_HEADING_UPDATE_STATE_DATA_HEADING_SOURCE_GNSS_VELOCITY_VECTOR = 2; ///<  
static const mip_filter_heading_update_state_data_heading_source MIP_FILTER_HEADING_UPDATE_STATE_DATA_HEADING_SOURCE_EXTERNAL             = 4; ///<  
static const mip_filter_heading_update_state_data_heading_source MIP_FILTER_HEADING_UPDATE_STATE_DATA_HEADING_SOURCE_DUAL_ANTENNA         = 8; ///<  

struct mip_filter_heading_update_state_data
{
    float heading; ///< [radians]
    float heading_1sigma; ///< [radians]
    mip_filter_heading_update_state_data_heading_source source;
    uint16_t valid_flags; ///< 1 if a valid heading update was received in 2 seconds, 0 otherwise.
    
};
typedef struct mip_filter_heading_update_state_data mip_filter_heading_update_state_data;
void insert_mip_filter_heading_update_state_data(struct mip_serializer* serializer, const mip_filter_heading_update_state_data* self);
void extract_mip_filter_heading_update_state_data(struct mip_serializer* serializer, mip_filter_heading_update_state_data* self);
bool extract_mip_filter_heading_update_state_data_from_field(const struct mip_field* field, void* ptr);

void insert_mip_filter_heading_update_state_data_heading_source(struct mip_serializer* serializer, const mip_filter_heading_update_state_data_heading_source self);
void extract_mip_filter_heading_update_state_data_heading_source(struct mip_serializer* serializer, mip_filter_heading_update_state_data_heading_source* self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_magnetic_model  (0x82,0x15) Magnetic Model [C]
/// The World Magnetic Model is used for this data. Please refer to the device user manual for the current version of the model.
/// A valid GNSS location is required for the model to be valid.
///
///@{

struct mip_filter_magnetic_model_data
{
    float intensity_north; ///< [Gauss]
    float intensity_east; ///< [Gauss]
    float intensity_down; ///< [Gauss]
    float inclination; ///< [radians]
    float declination; ///< [radians]
    uint16_t valid_flags; ///< 0 - invalid, 1 - valid
    
};
typedef struct mip_filter_magnetic_model_data mip_filter_magnetic_model_data;
void insert_mip_filter_magnetic_model_data(struct mip_serializer* serializer, const mip_filter_magnetic_model_data* self);
void extract_mip_filter_magnetic_model_data(struct mip_serializer* serializer, mip_filter_magnetic_model_data* self);
bool extract_mip_filter_magnetic_model_data_from_field(const struct mip_field* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_accel_scale_factor  (0x82,0x17) Accel Scale Factor [C]
/// Filter reported accelerometer scale factor expressed in the sensor frame.
///
///@{

struct mip_filter_accel_scale_factor_data
{
    mip_vector3f scale_factor; ///< (x,y,z) [dimensionless]
    uint16_t valid_flags; ///< 0 - invalid, 1 - valid
    
};
typedef struct mip_filter_accel_scale_factor_data mip_filter_accel_scale_factor_data;
void insert_mip_filter_accel_scale_factor_data(struct mip_serializer* serializer, const mip_filter_accel_scale_factor_data* self);
void extract_mip_filter_accel_scale_factor_data(struct mip_serializer* serializer, mip_filter_accel_scale_factor_data* self);
bool extract_mip_filter_accel_scale_factor_data_from_field(const struct mip_field* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_accel_scale_factor_uncertainty  (0x82,0x19) Accel Scale Factor Uncertainty [C]
/// Filter reported 1-sigma accelerometer scale factor uncertainty expressed in the sensor frame.
///
///@{

struct mip_filter_accel_scale_factor_uncertainty_data
{
    mip_vector3f scale_factor_uncert; ///< (x,y,z) [dimensionless]
    uint16_t valid_flags; ///< 0 - invalid, 1 - valid
    
};
typedef struct mip_filter_accel_scale_factor_uncertainty_data mip_filter_accel_scale_factor_uncertainty_data;
void insert_mip_filter_accel_scale_factor_uncertainty_data(struct mip_serializer* serializer, const mip_filter_accel_scale_factor_uncertainty_data* self);
void extract_mip_filter_accel_scale_factor_uncertainty_data(struct mip_serializer* serializer, mip_filter_accel_scale_factor_uncertainty_data* self);
bool extract_mip_filter_accel_scale_factor_uncertainty_data_from_field(const struct mip_field* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_gyro_scale_factor  (0x82,0x16) Gyro Scale Factor [C]
/// Filter reported gyro scale factor expressed in the sensor frame.
///
///@{

struct mip_filter_gyro_scale_factor_data
{
    mip_vector3f scale_factor; ///< (x,y,z) [dimensionless]
    uint16_t valid_flags; ///< 0 - invalid, 1 - valid
    
};
typedef struct mip_filter_gyro_scale_factor_data mip_filter_gyro_scale_factor_data;
void insert_mip_filter_gyro_scale_factor_data(struct mip_serializer* serializer, const mip_filter_gyro_scale_factor_data* self);
void extract_mip_filter_gyro_scale_factor_data(struct mip_serializer* serializer, mip_filter_gyro_scale_factor_data* self);
bool extract_mip_filter_gyro_scale_factor_data_from_field(const struct mip_field* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_gyro_scale_factor_uncertainty  (0x82,0x18) Gyro Scale Factor Uncertainty [C]
/// Filter reported 1-sigma gyro scale factor uncertainty expressed in the sensor frame.
///
///@{

struct mip_filter_gyro_scale_factor_uncertainty_data
{
    mip_vector3f scale_factor_uncert; ///< (x,y,z) [dimensionless]
    uint16_t valid_flags; ///< 0 - invalid, 1 - valid
    
};
typedef struct mip_filter_gyro_scale_factor_uncertainty_data mip_filter_gyro_scale_factor_uncertainty_data;
void insert_mip_filter_gyro_scale_factor_uncertainty_data(struct mip_serializer* serializer, const mip_filter_gyro_scale_factor_uncertainty_data* self);
void extract_mip_filter_gyro_scale_factor_uncertainty_data(struct mip_serializer* serializer, mip_filter_gyro_scale_factor_uncertainty_data* self);
bool extract_mip_filter_gyro_scale_factor_uncertainty_data_from_field(const struct mip_field* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_mag_bias  (0x82,0x1A) Mag Bias [C]
/// Filter reported magnetometer bias expressed in the sensor frame.
///
///@{

struct mip_filter_mag_bias_data
{
    mip_vector3f bias; ///< (x,y,z) [Gauss]
    uint16_t valid_flags; ///< 0 - invalid, 1 - valid
    
};
typedef struct mip_filter_mag_bias_data mip_filter_mag_bias_data;
void insert_mip_filter_mag_bias_data(struct mip_serializer* serializer, const mip_filter_mag_bias_data* self);
void extract_mip_filter_mag_bias_data(struct mip_serializer* serializer, mip_filter_mag_bias_data* self);
bool extract_mip_filter_mag_bias_data_from_field(const struct mip_field* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_mag_bias_uncertainty  (0x82,0x1B) Mag Bias Uncertainty [C]
/// Filter reported 1-sigma magnetometer bias uncertainty expressed in the sensor frame.
///
///@{

struct mip_filter_mag_bias_uncertainty_data
{
    mip_vector3f bias_uncert; ///< (x,y,z) [Gauss]
    uint16_t valid_flags; ///< 0 - invalid, 1 - valid
    
};
typedef struct mip_filter_mag_bias_uncertainty_data mip_filter_mag_bias_uncertainty_data;
void insert_mip_filter_mag_bias_uncertainty_data(struct mip_serializer* serializer, const mip_filter_mag_bias_uncertainty_data* self);
void extract_mip_filter_mag_bias_uncertainty_data(struct mip_serializer* serializer, mip_filter_mag_bias_uncertainty_data* self);
bool extract_mip_filter_mag_bias_uncertainty_data_from_field(const struct mip_field* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_standard_atmosphere  (0x82,0x20) Standard Atmosphere [C]
/// Filter reported standard atmosphere parameters.
/// 
/// The US 1976 Standard Atmosphere Model is used. A valid GNSS location is required for the model to be valid.
///
///@{

struct mip_filter_standard_atmosphere_data
{
    float geometric_altitude; ///< Input into calculation [meters]
    float geopotential_altitude; ///< [meters]
    float standard_temperature; ///< [degC]
    float standard_pressure; ///< [milliBar]
    float standard_density; ///< [kilogram/meter^3]
    uint16_t valid_flags; ///< 0 - invalid, 1 - valid
    
};
typedef struct mip_filter_standard_atmosphere_data mip_filter_standard_atmosphere_data;
void insert_mip_filter_standard_atmosphere_data(struct mip_serializer* serializer, const mip_filter_standard_atmosphere_data* self);
void extract_mip_filter_standard_atmosphere_data(struct mip_serializer* serializer, mip_filter_standard_atmosphere_data* self);
bool extract_mip_filter_standard_atmosphere_data_from_field(const struct mip_field* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_pressure_altitude  (0x82,0x21) Pressure Altitude [C]
/// Filter reported pressure altitude.
/// 
/// The US 1976 Standard Atmosphere Model is used to calculate the pressure altitude in meters.
/// A valid pressure sensor reading is required for the pressure altitude to be valid.
/// The minimum pressure reading supported by the model is 0.0037 mBar, corresponding to an altitude of 84,852 meters.
///
///@{

struct mip_filter_pressure_altitude_data
{
    float pressure_altitude; ///< [meters]
    uint16_t valid_flags; ///< 0 - invalid, 1 - valid
    
};
typedef struct mip_filter_pressure_altitude_data mip_filter_pressure_altitude_data;
void insert_mip_filter_pressure_altitude_data(struct mip_serializer* serializer, const mip_filter_pressure_altitude_data* self);
void extract_mip_filter_pressure_altitude_data(struct mip_serializer* serializer, mip_filter_pressure_altitude_data* self);
bool extract_mip_filter_pressure_altitude_data_from_field(const struct mip_field* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_density_altitude  (0x82,0x22) Density Altitude [C]
///
///@{

struct mip_filter_density_altitude_data
{
    float density_altitude; ///< m
    uint16_t valid_flags; ///< 0 - invalid, 1 - valid
    
};
typedef struct mip_filter_density_altitude_data mip_filter_density_altitude_data;
void insert_mip_filter_density_altitude_data(struct mip_serializer* serializer, const mip_filter_density_altitude_data* self);
void extract_mip_filter_density_altitude_data(struct mip_serializer* serializer, mip_filter_density_altitude_data* self);
bool extract_mip_filter_density_altitude_data_from_field(const struct mip_field* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_antenna_offset_correction  (0x82,0x30) Antenna Offset Correction [C]
/// Filter reported GNSS antenna offset in vehicle frame.
/// 
/// This offset added to any previously stored offset vector to compensate for errors in definition.
///
///@{

struct mip_filter_antenna_offset_correction_data
{
    mip_vector3f offset; ///< (x,y,z) [meters]
    uint16_t valid_flags; ///< 0 - invalid, 1 - valid
    
};
typedef struct mip_filter_antenna_offset_correction_data mip_filter_antenna_offset_correction_data;
void insert_mip_filter_antenna_offset_correction_data(struct mip_serializer* serializer, const mip_filter_antenna_offset_correction_data* self);
void extract_mip_filter_antenna_offset_correction_data(struct mip_serializer* serializer, mip_filter_antenna_offset_correction_data* self);
bool extract_mip_filter_antenna_offset_correction_data_from_field(const struct mip_field* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_antenna_offset_correction_uncertainty  (0x82,0x31) Antenna Offset Correction Uncertainty [C]
/// Filter reported 1-sigma GNSS antenna offset uncertainties in vehicle frame.
///
///@{

struct mip_filter_antenna_offset_correction_uncertainty_data
{
    mip_vector3f offset_uncert; ///< (x,y,z) [meters]
    uint16_t valid_flags; ///< 0 - invalid, 1 - valid
    
};
typedef struct mip_filter_antenna_offset_correction_uncertainty_data mip_filter_antenna_offset_correction_uncertainty_data;
void insert_mip_filter_antenna_offset_correction_uncertainty_data(struct mip_serializer* serializer, const mip_filter_antenna_offset_correction_uncertainty_data* self);
void extract_mip_filter_antenna_offset_correction_uncertainty_data(struct mip_serializer* serializer, mip_filter_antenna_offset_correction_uncertainty_data* self);
bool extract_mip_filter_antenna_offset_correction_uncertainty_data_from_field(const struct mip_field* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_multi_antenna_offset_correction  (0x82,0x34) Multi Antenna Offset Correction [C]
/// Filter reported GNSS antenna offset in vehicle frame.
/// 
/// This offset added to any previously stored offset vector to compensate for errors in definition.
///
///@{

struct mip_filter_multi_antenna_offset_correction_data
{
    uint8_t receiver_id; ///< Receiver ID for the receiver to which the antenna is attached
    mip_vector3f offset; ///< (x,y,z) [meters]
    uint16_t valid_flags; ///< 0 - invalid, 1 - valid
    
};
typedef struct mip_filter_multi_antenna_offset_correction_data mip_filter_multi_antenna_offset_correction_data;
void insert_mip_filter_multi_antenna_offset_correction_data(struct mip_serializer* serializer, const mip_filter_multi_antenna_offset_correction_data* self);
void extract_mip_filter_multi_antenna_offset_correction_data(struct mip_serializer* serializer, mip_filter_multi_antenna_offset_correction_data* self);
bool extract_mip_filter_multi_antenna_offset_correction_data_from_field(const struct mip_field* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_multi_antenna_offset_correction_uncertainty  (0x82,0x35) Multi Antenna Offset Correction Uncertainty [C]
/// Filter reported 1-sigma GNSS antenna offset uncertainties in vehicle frame.
///
///@{

struct mip_filter_multi_antenna_offset_correction_uncertainty_data
{
    uint8_t receiver_id; ///< Receiver ID for the receiver to which the antenna is attached
    mip_vector3f offset_uncert; ///< (x,y,z) [meters]
    uint16_t valid_flags; ///< 0 - invalid, 1 - valid
    
};
typedef struct mip_filter_multi_antenna_offset_correction_uncertainty_data mip_filter_multi_antenna_offset_correction_uncertainty_data;
void insert_mip_filter_multi_antenna_offset_correction_uncertainty_data(struct mip_serializer* serializer, const mip_filter_multi_antenna_offset_correction_uncertainty_data* self);
void extract_mip_filter_multi_antenna_offset_correction_uncertainty_data(struct mip_serializer* serializer, mip_filter_multi_antenna_offset_correction_uncertainty_data* self);
bool extract_mip_filter_multi_antenna_offset_correction_uncertainty_data_from_field(const struct mip_field* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_magnetometer_offset  (0x82,0x25) Magnetometer Offset [C]
/// Filter reported magnetometer hard iron offset in sensor frame.
/// 
/// This offset added to any previously stored hard iron offset vector to compensate for magnetometer in-run bias errors.
///
///@{

struct mip_filter_magnetometer_offset_data
{
    mip_vector3f hard_iron; ///< (x,y,z) [Gauss]
    uint16_t valid_flags; ///< 0 - invalid, 1 - valid
    
};
typedef struct mip_filter_magnetometer_offset_data mip_filter_magnetometer_offset_data;
void insert_mip_filter_magnetometer_offset_data(struct mip_serializer* serializer, const mip_filter_magnetometer_offset_data* self);
void extract_mip_filter_magnetometer_offset_data(struct mip_serializer* serializer, mip_filter_magnetometer_offset_data* self);
bool extract_mip_filter_magnetometer_offset_data_from_field(const struct mip_field* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_magnetometer_matrix  (0x82,0x26) Magnetometer Matrix [C]
/// Filter reported magnetometer soft iron matrix in sensor frame.
/// 
/// This matrix is post multiplied to any previously stored soft iron matrix to compensate for magnetometer in-run errors.
///
///@{

struct mip_filter_magnetometer_matrix_data
{
    mip_matrix3f soft_iron; ///< Row-major [dimensionless]
    uint16_t valid_flags; ///< 0 - invalid, 1 - valid
    
};
typedef struct mip_filter_magnetometer_matrix_data mip_filter_magnetometer_matrix_data;
void insert_mip_filter_magnetometer_matrix_data(struct mip_serializer* serializer, const mip_filter_magnetometer_matrix_data* self);
void extract_mip_filter_magnetometer_matrix_data(struct mip_serializer* serializer, mip_filter_magnetometer_matrix_data* self);
bool extract_mip_filter_magnetometer_matrix_data_from_field(const struct mip_field* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_magnetometer_offset_uncertainty  (0x82,0x28) Magnetometer Offset Uncertainty [C]
/// Filter reported 1-sigma magnetometer hard iron offset uncertainties in sensor frame.
///
///@{

struct mip_filter_magnetometer_offset_uncertainty_data
{
    mip_vector3f hard_iron_uncertainty; ///< (x,y,z) [Gauss]
    uint16_t valid_flags; ///< 0 - invalid, 1 - valid
    
};
typedef struct mip_filter_magnetometer_offset_uncertainty_data mip_filter_magnetometer_offset_uncertainty_data;
void insert_mip_filter_magnetometer_offset_uncertainty_data(struct mip_serializer* serializer, const mip_filter_magnetometer_offset_uncertainty_data* self);
void extract_mip_filter_magnetometer_offset_uncertainty_data(struct mip_serializer* serializer, mip_filter_magnetometer_offset_uncertainty_data* self);
bool extract_mip_filter_magnetometer_offset_uncertainty_data_from_field(const struct mip_field* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_magnetometer_matrix_uncertainty  (0x82,0x29) Magnetometer Matrix Uncertainty [C]
/// Filter reported 1-sigma magnetometer soft iron matrix uncertainties in sensor frame.
///
///@{

struct mip_filter_magnetometer_matrix_uncertainty_data
{
    mip_matrix3f soft_iron_uncertainty; ///< Row-major [dimensionless]
    uint16_t valid_flags; ///< 0 - invalid, 1 - valid
    
};
typedef struct mip_filter_magnetometer_matrix_uncertainty_data mip_filter_magnetometer_matrix_uncertainty_data;
void insert_mip_filter_magnetometer_matrix_uncertainty_data(struct mip_serializer* serializer, const mip_filter_magnetometer_matrix_uncertainty_data* self);
void extract_mip_filter_magnetometer_matrix_uncertainty_data(struct mip_serializer* serializer, mip_filter_magnetometer_matrix_uncertainty_data* self);
bool extract_mip_filter_magnetometer_matrix_uncertainty_data_from_field(const struct mip_field* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_magnetometer_covariance_matrix  (0x82,0x2A) Magnetometer Covariance Matrix [C]
///
///@{

struct mip_filter_magnetometer_covariance_matrix_data
{
    mip_matrix3f covariance;
    uint16_t valid_flags; ///< 0 - invalid, 1 - valid
    
};
typedef struct mip_filter_magnetometer_covariance_matrix_data mip_filter_magnetometer_covariance_matrix_data;
void insert_mip_filter_magnetometer_covariance_matrix_data(struct mip_serializer* serializer, const mip_filter_magnetometer_covariance_matrix_data* self);
void extract_mip_filter_magnetometer_covariance_matrix_data(struct mip_serializer* serializer, mip_filter_magnetometer_covariance_matrix_data* self);
bool extract_mip_filter_magnetometer_covariance_matrix_data_from_field(const struct mip_field* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_magnetometer_residual_vector  (0x82,0x2C) Magnetometer Residual Vector [C]
/// Filter reported magnetometer measurement residuals in vehicle frame.
///
///@{

struct mip_filter_magnetometer_residual_vector_data
{
    mip_vector3f residual; ///< (x,y,z) [Gauss]
    uint16_t valid_flags; ///< 0 - invalid, 1 - valid
    
};
typedef struct mip_filter_magnetometer_residual_vector_data mip_filter_magnetometer_residual_vector_data;
void insert_mip_filter_magnetometer_residual_vector_data(struct mip_serializer* serializer, const mip_filter_magnetometer_residual_vector_data* self);
void extract_mip_filter_magnetometer_residual_vector_data(struct mip_serializer* serializer, mip_filter_magnetometer_residual_vector_data* self);
bool extract_mip_filter_magnetometer_residual_vector_data_from_field(const struct mip_field* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_clock_correction  (0x82,0x32) Clock Correction [C]
/// Filter reported GNSS receiver clock error parameters.
///
///@{

struct mip_filter_clock_correction_data
{
    uint8_t receiver_id; ///< 1, 2, etc.
    float bias; ///< [seconds]
    float bias_drift; ///< [seconds/second]
    uint16_t valid_flags; ///< 0 - invalid, 1 - valid
    
};
typedef struct mip_filter_clock_correction_data mip_filter_clock_correction_data;
void insert_mip_filter_clock_correction_data(struct mip_serializer* serializer, const mip_filter_clock_correction_data* self);
void extract_mip_filter_clock_correction_data(struct mip_serializer* serializer, mip_filter_clock_correction_data* self);
bool extract_mip_filter_clock_correction_data_from_field(const struct mip_field* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_clock_correction_uncertainty  (0x82,0x33) Clock Correction Uncertainty [C]
/// Filter reported 1-sigma GNSS receiver clock error parameters.
///
///@{

struct mip_filter_clock_correction_uncertainty_data
{
    uint8_t receiver_id; ///< 1, 2, etc.
    float bias_uncertainty; ///< [seconds]
    float bias_drift_uncertainty; ///< [seconds/second]
    uint16_t valid_flags; ///< 0 - invalid, 1 - valid
    
};
typedef struct mip_filter_clock_correction_uncertainty_data mip_filter_clock_correction_uncertainty_data;
void insert_mip_filter_clock_correction_uncertainty_data(struct mip_serializer* serializer, const mip_filter_clock_correction_uncertainty_data* self);
void extract_mip_filter_clock_correction_uncertainty_data(struct mip_serializer* serializer, mip_filter_clock_correction_uncertainty_data* self);
bool extract_mip_filter_clock_correction_uncertainty_data_from_field(const struct mip_field* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_gnss_pos_aid_status  (0x82,0x43) Gnss Pos Aid Status [C]
/// Filter reported GNSS position aiding status
///
///@{

struct mip_filter_gnss_pos_aid_status_data
{
    uint8_t receiver_id;
    float time_of_week; ///< Last GNSS aiding measurement time of week [seconds]
    mip_gnss_aid_status_flags status; ///< Aiding measurement status bitfield
    uint8_t reserved[8];
    
};
typedef struct mip_filter_gnss_pos_aid_status_data mip_filter_gnss_pos_aid_status_data;
void insert_mip_filter_gnss_pos_aid_status_data(struct mip_serializer* serializer, const mip_filter_gnss_pos_aid_status_data* self);
void extract_mip_filter_gnss_pos_aid_status_data(struct mip_serializer* serializer, mip_filter_gnss_pos_aid_status_data* self);
bool extract_mip_filter_gnss_pos_aid_status_data_from_field(const struct mip_field* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_gnss_att_aid_status  (0x82,0x44) Gnss Att Aid Status [C]
/// Filter reported dual antenna GNSS attitude aiding status
///
///@{

struct mip_filter_gnss_att_aid_status_data
{
    float time_of_week; ///< Last valid aiding measurement time of week [seconds] [processed instead of measured?]
    mip_gnss_aid_status_flags status; ///< Last valid aiding measurement status bitfield
    uint8_t reserved[8];
    
};
typedef struct mip_filter_gnss_att_aid_status_data mip_filter_gnss_att_aid_status_data;
void insert_mip_filter_gnss_att_aid_status_data(struct mip_serializer* serializer, const mip_filter_gnss_att_aid_status_data* self);
void extract_mip_filter_gnss_att_aid_status_data(struct mip_serializer* serializer, mip_filter_gnss_att_aid_status_data* self);
bool extract_mip_filter_gnss_att_aid_status_data_from_field(const struct mip_field* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_head_aid_status  (0x82,0x45) Head Aid Status [C]
/// Filter reported GNSS heading aiding status
///
///@{

typedef uint8_t mip_filter_head_aid_status_data_heading_aid_type;
static const mip_filter_head_aid_status_data_heading_aid_type MIP_FILTER_HEAD_AID_STATUS_DATA_HEADING_AID_TYPE_DUAL_ANTENNA     = 1; ///<  
static const mip_filter_head_aid_status_data_heading_aid_type MIP_FILTER_HEAD_AID_STATUS_DATA_HEADING_AID_TYPE_EXTERNAL_MESSAGE = 2; ///<  

struct mip_filter_head_aid_status_data
{
    float time_of_week; ///< Last valid aiding measurement time of week [seconds] [processed instead of measured?]
    mip_filter_head_aid_status_data_heading_aid_type type; ///< 1 - Dual antenna, 2 - External heading message (user supplied)
    float reserved[2];
    
};
typedef struct mip_filter_head_aid_status_data mip_filter_head_aid_status_data;
void insert_mip_filter_head_aid_status_data(struct mip_serializer* serializer, const mip_filter_head_aid_status_data* self);
void extract_mip_filter_head_aid_status_data(struct mip_serializer* serializer, mip_filter_head_aid_status_data* self);
bool extract_mip_filter_head_aid_status_data_from_field(const struct mip_field* field, void* ptr);

void insert_mip_filter_head_aid_status_data_heading_aid_type(struct mip_serializer* serializer, const mip_filter_head_aid_status_data_heading_aid_type self);
void extract_mip_filter_head_aid_status_data_heading_aid_type(struct mip_serializer* serializer, mip_filter_head_aid_status_data_heading_aid_type* self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_rel_pos_ned  (0x82,0x42) Rel Pos Ned [C]
/// Filter reported relative position, with respect to configured reference position
///
///@{

struct mip_filter_rel_pos_ned_data
{
    mip_vector3d relative_position; ///< [meters, NED]
    uint16_t valid_flags; ///< 0 - invalid, 1 - valid
    
};
typedef struct mip_filter_rel_pos_ned_data mip_filter_rel_pos_ned_data;
void insert_mip_filter_rel_pos_ned_data(struct mip_serializer* serializer, const mip_filter_rel_pos_ned_data* self);
void extract_mip_filter_rel_pos_ned_data(struct mip_serializer* serializer, mip_filter_rel_pos_ned_data* self);
bool extract_mip_filter_rel_pos_ned_data_from_field(const struct mip_field* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_ecef_pos  (0x82,0x40) Ecef Pos [C]
/// Filter reported ECEF position
///
///@{

struct mip_filter_ecef_pos_data
{
    mip_vector3d position_ecef; ///< [meters, ECEF]
    uint16_t valid_flags; ///< 0 - invalid, 1 valid
    
};
typedef struct mip_filter_ecef_pos_data mip_filter_ecef_pos_data;
void insert_mip_filter_ecef_pos_data(struct mip_serializer* serializer, const mip_filter_ecef_pos_data* self);
void extract_mip_filter_ecef_pos_data(struct mip_serializer* serializer, mip_filter_ecef_pos_data* self);
bool extract_mip_filter_ecef_pos_data_from_field(const struct mip_field* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_ecef_vel  (0x82,0x41) Ecef Vel [C]
/// Filter reported ECEF velocity
///
///@{

struct mip_filter_ecef_vel_data
{
    mip_vector3f velocity_ecef; ///< [meters/second, ECEF]
    uint16_t valid_flags; ///< 0 - invalid, 1 valid
    
};
typedef struct mip_filter_ecef_vel_data mip_filter_ecef_vel_data;
void insert_mip_filter_ecef_vel_data(struct mip_serializer* serializer, const mip_filter_ecef_vel_data* self);
void extract_mip_filter_ecef_vel_data(struct mip_serializer* serializer, mip_filter_ecef_vel_data* self);
bool extract_mip_filter_ecef_vel_data_from_field(const struct mip_field* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_ecef_pos_uncertainty  (0x82,0x36) Ecef Pos Uncertainty [C]
/// Filter reported 1-sigma position uncertainty in the ECEF frame.
///
///@{

struct mip_filter_ecef_pos_uncertainty_data
{
    mip_vector3f pos_uncertainty; ///< [meters]
    uint16_t valid_flags; ///< 0 - invalid, 1 - valid
    
};
typedef struct mip_filter_ecef_pos_uncertainty_data mip_filter_ecef_pos_uncertainty_data;
void insert_mip_filter_ecef_pos_uncertainty_data(struct mip_serializer* serializer, const mip_filter_ecef_pos_uncertainty_data* self);
void extract_mip_filter_ecef_pos_uncertainty_data(struct mip_serializer* serializer, mip_filter_ecef_pos_uncertainty_data* self);
bool extract_mip_filter_ecef_pos_uncertainty_data_from_field(const struct mip_field* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_ecef_vel_uncertainty  (0x82,0x37) Ecef Vel Uncertainty [C]
/// Filter reported 1-sigma velocity uncertainties in the ECEF frame.
///
///@{

struct mip_filter_ecef_vel_uncertainty_data
{
    mip_vector3f vel_uncertainty; ///< [meters/second]
    uint16_t valid_flags; ///< 0 - invalid, 1 - valid
    
};
typedef struct mip_filter_ecef_vel_uncertainty_data mip_filter_ecef_vel_uncertainty_data;
void insert_mip_filter_ecef_vel_uncertainty_data(struct mip_serializer* serializer, const mip_filter_ecef_vel_uncertainty_data* self);
void extract_mip_filter_ecef_vel_uncertainty_data(struct mip_serializer* serializer, mip_filter_ecef_vel_uncertainty_data* self);
bool extract_mip_filter_ecef_vel_uncertainty_data_from_field(const struct mip_field* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_aiding_measurement_summary  (0x82,0x46) Aiding Measurement Summary [C]
/// Filter reported aiding measurement summary. This message contains a summary of the specified aiding measurement over the previous measurement interval ending at the specified time.
///
///@{

struct mip_filter_aiding_measurement_summary_data
{
    float time_of_week; ///< [seconds]
    uint8_t source;
    mip_filter_aiding_measurement_type type; ///< (see product manual for supported types) Note: values 0x20 and above correspond to commanded aiding measurements in the 0x13 Aiding command set.
    mip_filter_measurement_indicator indicator;
    
};
typedef struct mip_filter_aiding_measurement_summary_data mip_filter_aiding_measurement_summary_data;
void insert_mip_filter_aiding_measurement_summary_data(struct mip_serializer* serializer, const mip_filter_aiding_measurement_summary_data* self);
void extract_mip_filter_aiding_measurement_summary_data(struct mip_serializer* serializer, mip_filter_aiding_measurement_summary_data* self);
bool extract_mip_filter_aiding_measurement_summary_data_from_field(const struct mip_field* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_odometer_scale_factor_error  (0x82,0x47) Odometer Scale Factor Error [C]
/// Filter reported odometer scale factor error. The total scale factor estimate is the user indicated scale factor, plus the user indicated scale factor times the scale factor error.
///
///@{

struct mip_filter_odometer_scale_factor_error_data
{
    float scale_factor_error; ///< [dimensionless]
    uint16_t valid_flags; ///< 0 - invalid, 1 - valid
    
};
typedef struct mip_filter_odometer_scale_factor_error_data mip_filter_odometer_scale_factor_error_data;
void insert_mip_filter_odometer_scale_factor_error_data(struct mip_serializer* serializer, const mip_filter_odometer_scale_factor_error_data* self);
void extract_mip_filter_odometer_scale_factor_error_data(struct mip_serializer* serializer, mip_filter_odometer_scale_factor_error_data* self);
bool extract_mip_filter_odometer_scale_factor_error_data_from_field(const struct mip_field* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_odometer_scale_factor_error_uncertainty  (0x82,0x48) Odometer Scale Factor Error Uncertainty [C]
/// Filter reported odometer scale factor error uncertainty.
///
///@{

struct mip_filter_odometer_scale_factor_error_uncertainty_data
{
    float scale_factor_error_uncertainty; ///< [dimensionless]
    uint16_t valid_flags; ///< 0 - invalid, 1 - valid
    
};
typedef struct mip_filter_odometer_scale_factor_error_uncertainty_data mip_filter_odometer_scale_factor_error_uncertainty_data;
void insert_mip_filter_odometer_scale_factor_error_uncertainty_data(struct mip_serializer* serializer, const mip_filter_odometer_scale_factor_error_uncertainty_data* self);
void extract_mip_filter_odometer_scale_factor_error_uncertainty_data(struct mip_serializer* serializer, mip_filter_odometer_scale_factor_error_uncertainty_data* self);
bool extract_mip_filter_odometer_scale_factor_error_uncertainty_data_from_field(const struct mip_field* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_gnss_dual_antenna_status  (0x82,0x49) Gnss Dual Antenna Status [C]
/// Summary information for status of GNSS dual antenna heading estimate.
///
///@{

typedef uint8_t mip_filter_gnss_dual_antenna_status_data_fix_type;
static const mip_filter_gnss_dual_antenna_status_data_fix_type MIP_FILTER_GNSS_DUAL_ANTENNA_STATUS_DATA_FIX_TYPE_FIX_NONE     = 0; ///<  
static const mip_filter_gnss_dual_antenna_status_data_fix_type MIP_FILTER_GNSS_DUAL_ANTENNA_STATUS_DATA_FIX_TYPE_FIX_DA_FLOAT = 1; ///<  
static const mip_filter_gnss_dual_antenna_status_data_fix_type MIP_FILTER_GNSS_DUAL_ANTENNA_STATUS_DATA_FIX_TYPE_FIX_DA_FIXED = 2; ///<  

typedef uint16_t mip_filter_gnss_dual_antenna_status_data_dual_antenna_status_flags;
static const mip_filter_gnss_dual_antenna_status_data_dual_antenna_status_flags MIP_FILTER_GNSS_DUAL_ANTENNA_STATUS_DATA_DUAL_ANTENNA_STATUS_FLAGS_NONE                  = 0x0000;
static const mip_filter_gnss_dual_antenna_status_data_dual_antenna_status_flags MIP_FILTER_GNSS_DUAL_ANTENNA_STATUS_DATA_DUAL_ANTENNA_STATUS_FLAGS_RCV_1_DATA_VALID      = 0x0001; ///<  
static const mip_filter_gnss_dual_antenna_status_data_dual_antenna_status_flags MIP_FILTER_GNSS_DUAL_ANTENNA_STATUS_DATA_DUAL_ANTENNA_STATUS_FLAGS_RCV_2_DATA_VALID      = 0x0002; ///<  
static const mip_filter_gnss_dual_antenna_status_data_dual_antenna_status_flags MIP_FILTER_GNSS_DUAL_ANTENNA_STATUS_DATA_DUAL_ANTENNA_STATUS_FLAGS_ANTENNA_OFFSETS_VALID = 0x0004; ///<  
static const mip_filter_gnss_dual_antenna_status_data_dual_antenna_status_flags MIP_FILTER_GNSS_DUAL_ANTENNA_STATUS_DATA_DUAL_ANTENNA_STATUS_FLAGS_ALL                   = 0x0007;

struct mip_filter_gnss_dual_antenna_status_data
{
    float time_of_week; ///< Last dual-antenna GNSS aiding measurement time of week [seconds]
    float heading; ///< [radians]
    float heading_unc; ///< [radians]
    mip_filter_gnss_dual_antenna_status_data_fix_type fix_type; ///< Fix type indicator
    mip_filter_gnss_dual_antenna_status_data_dual_antenna_status_flags status_flags;
    uint16_t valid_flags; ///< 0 - invalid, 1 - valid
    
};
typedef struct mip_filter_gnss_dual_antenna_status_data mip_filter_gnss_dual_antenna_status_data;
void insert_mip_filter_gnss_dual_antenna_status_data(struct mip_serializer* serializer, const mip_filter_gnss_dual_antenna_status_data* self);
void extract_mip_filter_gnss_dual_antenna_status_data(struct mip_serializer* serializer, mip_filter_gnss_dual_antenna_status_data* self);
bool extract_mip_filter_gnss_dual_antenna_status_data_from_field(const struct mip_field* field, void* ptr);

void insert_mip_filter_gnss_dual_antenna_status_data_fix_type(struct mip_serializer* serializer, const mip_filter_gnss_dual_antenna_status_data_fix_type self);
void extract_mip_filter_gnss_dual_antenna_status_data_fix_type(struct mip_serializer* serializer, mip_filter_gnss_dual_antenna_status_data_fix_type* self);

void insert_mip_filter_gnss_dual_antenna_status_data_dual_antenna_status_flags(struct mip_serializer* serializer, const mip_filter_gnss_dual_antenna_status_data_dual_antenna_status_flags self);
void extract_mip_filter_gnss_dual_antenna_status_data_dual_antenna_status_flags(struct mip_serializer* serializer, mip_filter_gnss_dual_antenna_status_data_dual_antenna_status_flags* self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_aiding_frame_config_error  (0x82,0x50) Aiding Frame Config Error [C]
/// Filter reported aiding source frame configuration error
/// 
/// These estimates are used to compensate for small errors to the user-supplied aiding frame configurations (set with (0x13, 0x01) command ).
///
///@{

struct mip_filter_aiding_frame_config_error_data
{
    uint8_t frame_id; ///< Frame ID for the receiver to which the antenna is attached
    mip_vector3f translation; ///< Translation config X, Y, and Z (m).
    mip_quatf attitude; ///< Attitude quaternion
    
};
typedef struct mip_filter_aiding_frame_config_error_data mip_filter_aiding_frame_config_error_data;
void insert_mip_filter_aiding_frame_config_error_data(struct mip_serializer* serializer, const mip_filter_aiding_frame_config_error_data* self);
void extract_mip_filter_aiding_frame_config_error_data(struct mip_serializer* serializer, mip_filter_aiding_frame_config_error_data* self);
bool extract_mip_filter_aiding_frame_config_error_data_from_field(const struct mip_field* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_filter_aiding_frame_config_error_uncertainty  (0x82,0x51) Aiding Frame Config Error Uncertainty [C]
/// Filter reported aiding source frame configuration error uncertainty
/// 
/// These estimates are used to compensate for small errors to the user-supplied aiding frame configurations (set with (0x13, 0x01) command ).
///
///@{

struct mip_filter_aiding_frame_config_error_uncertainty_data
{
    uint8_t frame_id; ///< Frame ID for the receiver to which the antenna is attached
    mip_vector3f translation_unc; ///< Translation uncertaint X, Y, and Z (m).
    mip_vector3f attitude_unc; ///< Attitude uncertainty, X, Y, and Z (radians).
    
};
typedef struct mip_filter_aiding_frame_config_error_uncertainty_data mip_filter_aiding_frame_config_error_uncertainty_data;
void insert_mip_filter_aiding_frame_config_error_uncertainty_data(struct mip_serializer* serializer, const mip_filter_aiding_frame_config_error_uncertainty_data* self);
void extract_mip_filter_aiding_frame_config_error_uncertainty_data(struct mip_serializer* serializer, mip_filter_aiding_frame_config_error_uncertainty_data* self);
bool extract_mip_filter_aiding_frame_config_error_uncertainty_data_from_field(const struct mip_field* field, void* ptr);


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


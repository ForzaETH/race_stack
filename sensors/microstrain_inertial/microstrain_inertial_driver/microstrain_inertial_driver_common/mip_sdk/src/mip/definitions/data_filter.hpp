#pragma once

#include "common.h"
#include "descriptors.h"
#include "../mip_result.h"

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

namespace mip {
class Serializer;

namespace C {
struct mip_interface;
} // namespace C

namespace data_filter {

////////////////////////////////////////////////////////////////////////////////
///@addtogroup MipData_cpp  MIP Data [CPP]
///@{
///@defgroup filter_data_cpp  Filter Data [CPP]
///
///@{

////////////////////////////////////////////////////////////////////////////////
// Descriptors
////////////////////////////////////////////////////////////////////////////////

enum 
{
    DESCRIPTOR_SET                                   = 0x82,
    
    DATA_POS_LLH                                     = 0x01,
    DATA_VEL_NED                                     = 0x02,
    DATA_ATT_QUATERNION                              = 0x03,
    DATA_ATT_MATRIX                                  = 0x04,
    DATA_ATT_EULER_ANGLES                            = 0x05,
    DATA_GYRO_BIAS                                   = 0x06,
    DATA_ACCEL_BIAS                                  = 0x07,
    DATA_POS_UNCERTAINTY                             = 0x08,
    DATA_VEL_UNCERTAINTY                             = 0x09,
    DATA_ATT_UNCERTAINTY_EULER                       = 0x0A,
    DATA_GYRO_BIAS_UNCERTAINTY                       = 0x0B,
    DATA_ACCEL_BIAS_UNCERTAINTY                      = 0x0C,
    DATA_LINEAR_ACCELERATION                         = 0x0D,
    DATA_COMPENSATED_ANGULAR_RATE                    = 0x0E,
    DATA_WGS84_GRAVITY                               = 0x0F,
    DATA_FILTER_STATUS                               = 0x10,
    DATA_FILTER_TIMESTAMP                            = 0x11,
    DATA_ATT_UNCERTAINTY_QUATERNION                  = 0x12,
    DATA_GRAVITY_VECTOR                              = 0x13,
    DATA_HEADING_UPDATE_STATE                        = 0x14,
    DATA_MAGNETIC_MODEL                              = 0x15,
    DATA_GYRO_SCALE_FACTOR                           = 0x16,
    DATA_ACCEL_SCALE_FACTOR                          = 0x17,
    DATA_GYRO_SCALE_FACTOR_UNCERTAINTY               = 0x18,
    DATA_ACCEL_SCALE_FACTOR_UNCERTAINTY              = 0x19,
    DATA_MAG_BIAS                                    = 0x1A,
    DATA_MAG_BIAS_UNCERTAINTY                        = 0x1B,
    DATA_COMPENSATED_ACCELERATION                    = 0x1C,
    DATA_STANDARD_ATMOSPHERE_DATA                    = 0x20,
    DATA_PRESSURE_ALTITUDE_DATA                      = 0x21,
    DATA_DENSITY_ALTITUDE_DATA                       = 0x22,
    DATA_MAG_SCALE_FACTOR                            = 0x23,
    DATA_MAG_SCALE_FACTOR_UNCERTAINTY                = 0x24,
    DATA_MAG_COMPENSATION_OFFSET                     = 0x25,
    DATA_MAG_COMPENSATION_MATRIX                     = 0x26,
    DATA_COMPENSATED_MAGNETOMETER                    = 0x27,
    DATA_MAG_COMPENSATION_OFFSET_UNCERTAINTY         = 0x28,
    DATA_MAG_COMPENSATION_MATRIX_UNCERTAINTY         = 0x29,
    DATA_MAG_COVARIANCE                              = 0x2A,
    DATA_GRAVITY_COVARIANCE                          = 0x2B,
    DATA_MAG_RESIDUAL                                = 0x2C,
    DATA_MAG_FILTERED_RESIDUAL                       = 0x2D,
    DATA_ANTENNA_OFFSET_CORRECTION                   = 0x30,
    DATA_ANTENNA_OFFSET_CORRECTION_UNCERTAINTY       = 0x31,
    DATA_CLOCK_CORRECTION                            = 0x32,
    DATA_CLOCK_CORRECTION_UNCERTAINTY                = 0x33,
    DATA_MULTI_ANTENNA_OFFSET_CORRECTION             = 0x34,
    DATA_MULTI_ANTENNA_OFFSET_CORRECTION_UNCERTAINTY = 0x35,
    DATA_ECEF_POS_UNCERTAINTY                        = 0x36,
    DATA_ECEF_VEL_UNCERTAINTY                        = 0x37,
    DATA_ECEF_POS                                    = 0x40,
    DATA_ECEF_VEL                                    = 0x41,
    DATA_REL_POS_NED                                 = 0x42,
    DATA_GNSS_POS_AID_STATUS                         = 0x43,
    DATA_GNSS_ATT_AID_STATUS                         = 0x44,
    DATA_HEAD_AID_STATUS                             = 0x45,
    DATA_AID_MEAS_SUMMARY                            = 0x46,
    DATA_ODOMETER_SCALE_FACTOR_ERROR                 = 0x47,
    DATA_ODOMETER_SCALE_FACTOR_ERROR_UNCERTAINTY     = 0x48,
    DATA_GNSS_DUAL_ANTENNA_STATUS                    = 0x49,
    DATA_FRAME_CONFIG_ERROR                          = 0x50,
    DATA_FRAME_CONFIG_ERROR_UNCERTAINTY              = 0x51,
    
};

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

enum class FilterMode : uint16_t
{
    GX5_STARTUP            = 0,  ///<  
    GX5_INIT               = 1,  ///<  
    GX5_RUN_SOLUTION_VALID = 2,  ///<  
    GX5_RUN_SOLUTION_ERROR = 3,  ///<  
    INIT                   = 1,  ///<  
    VERT_GYRO              = 2,  ///<  
    AHRS                   = 3,  ///<  
    FULL_NAV               = 4,  ///<  
};

enum class FilterDynamicsMode : uint16_t
{
    GX5_PORTABLE   = 1,  ///<  
    GX5_AUTOMOTIVE = 2,  ///<  
    GX5_AIRBORNE   = 3,  ///<  
    GQ7_DEFAULT    = 1,  ///<  
};

struct FilterStatusFlags : Bitfield<FilterStatusFlags>
{
    enum _enumType : uint16_t
    {
        NONE                                           = 0x0000,
        GX5_INIT_NO_ATTITUDE                           = 0x1000,  ///<  
        GX5_INIT_NO_POSITION_VELOCITY                  = 0x2000,  ///<  
        GX5_RUN_IMU_UNAVAILABLE                        = 0x0001,  ///<  
        GX5_RUN_GPS_UNAVAILABLE                        = 0x0002,  ///<  
        GX5_RUN_MATRIX_SINGULARITY                     = 0x0008,  ///<  
        GX5_RUN_POSITION_COVARIANCE_WARNING            = 0x0010,  ///<  
        GX5_RUN_VELOCITY_COVARIANCE_WARNING            = 0x0020,  ///<  
        GX5_RUN_ATTITUDE_COVARIANCE_WARNING            = 0x0040,  ///<  
        GX5_RUN_NAN_IN_SOLUTION_WARNING                = 0x0080,  ///<  
        GX5_RUN_GYRO_BIAS_EST_HIGH_WARNING             = 0x0100,  ///<  
        GX5_RUN_ACCEL_BIAS_EST_HIGH_WARNING            = 0x0200,  ///<  
        GX5_RUN_GYRO_SCALE_FACTOR_EST_HIGH_WARNING     = 0x0400,  ///<  
        GX5_RUN_ACCEL_SCALE_FACTOR_EST_HIGH_WARNING    = 0x0800,  ///<  
        GX5_RUN_MAG_BIAS_EST_HIGH_WARNING              = 0x1000,  ///<  
        GX5_RUN_ANT_OFFSET_CORRECTION_EST_HIGH_WARNING = 0x2000,  ///<  
        GX5_RUN_MAG_HARD_IRON_EST_HIGH_WARNING         = 0x4000,  ///<  
        GX5_RUN_MAG_SOFT_IRON_EST_HIGH_WARNING         = 0x8000,  ///<  
        GQ7_FILTER_CONDITION                           = 0x0003,  ///<  
        GQ7_ROLL_PITCH_WARNING                         = 0x0004,  ///<  
        GQ7_HEADING_WARNING                            = 0x0008,  ///<  
        GQ7_POSITION_WARNING                           = 0x0010,  ///<  
        GQ7_VELOCITY_WARNING                           = 0x0020,  ///<  
        GQ7_IMU_BIAS_WARNING                           = 0x0040,  ///<  
        GQ7_GNSS_CLK_WARNING                           = 0x0080,  ///<  
        GQ7_ANTENNA_LEVER_ARM_WARNING                  = 0x0100,  ///<  
        GQ7_MOUNTING_TRANSFORM_WARNING                 = 0x0200,  ///<  
        GQ7_TIME_SYNC_WARNING                          = 0x0400,  ///<  No time synchronization pulse detected
        GQ7_SOLUTION_ERROR                             = 0xF000,  ///<  Filter computation warning flags. If any bits 12-15 are set, and all filter outputs will be invalid.
        ALL                                            = 0xFFFF,
    };
    uint16_t value = NONE;
    
    FilterStatusFlags() : value(NONE) {}
    FilterStatusFlags(int val) : value((uint16_t)val) {}
    operator uint16_t() const { return value; }
    FilterStatusFlags& operator=(uint16_t val) { value = val; return *this; }
    FilterStatusFlags& operator=(int val) { value = uint16_t(val); return *this; }
    FilterStatusFlags& operator|=(uint16_t val) { return *this = value | val; }
    FilterStatusFlags& operator&=(uint16_t val) { return *this = value & val; }
    
    bool gx5InitNoAttitude() const { return (value & GX5_INIT_NO_ATTITUDE) > 0; }
    void gx5InitNoAttitude(bool val) { if(val) value |= GX5_INIT_NO_ATTITUDE; else value &= ~GX5_INIT_NO_ATTITUDE; }
    bool gx5InitNoPositionVelocity() const { return (value & GX5_INIT_NO_POSITION_VELOCITY) > 0; }
    void gx5InitNoPositionVelocity(bool val) { if(val) value |= GX5_INIT_NO_POSITION_VELOCITY; else value &= ~GX5_INIT_NO_POSITION_VELOCITY; }
    bool gx5RunImuUnavailable() const { return (value & GX5_RUN_IMU_UNAVAILABLE) > 0; }
    void gx5RunImuUnavailable(bool val) { if(val) value |= GX5_RUN_IMU_UNAVAILABLE; else value &= ~GX5_RUN_IMU_UNAVAILABLE; }
    bool gx5RunGpsUnavailable() const { return (value & GX5_RUN_GPS_UNAVAILABLE) > 0; }
    void gx5RunGpsUnavailable(bool val) { if(val) value |= GX5_RUN_GPS_UNAVAILABLE; else value &= ~GX5_RUN_GPS_UNAVAILABLE; }
    bool gx5RunMatrixSingularity() const { return (value & GX5_RUN_MATRIX_SINGULARITY) > 0; }
    void gx5RunMatrixSingularity(bool val) { if(val) value |= GX5_RUN_MATRIX_SINGULARITY; else value &= ~GX5_RUN_MATRIX_SINGULARITY; }
    bool gx5RunPositionCovarianceWarning() const { return (value & GX5_RUN_POSITION_COVARIANCE_WARNING) > 0; }
    void gx5RunPositionCovarianceWarning(bool val) { if(val) value |= GX5_RUN_POSITION_COVARIANCE_WARNING; else value &= ~GX5_RUN_POSITION_COVARIANCE_WARNING; }
    bool gx5RunVelocityCovarianceWarning() const { return (value & GX5_RUN_VELOCITY_COVARIANCE_WARNING) > 0; }
    void gx5RunVelocityCovarianceWarning(bool val) { if(val) value |= GX5_RUN_VELOCITY_COVARIANCE_WARNING; else value &= ~GX5_RUN_VELOCITY_COVARIANCE_WARNING; }
    bool gx5RunAttitudeCovarianceWarning() const { return (value & GX5_RUN_ATTITUDE_COVARIANCE_WARNING) > 0; }
    void gx5RunAttitudeCovarianceWarning(bool val) { if(val) value |= GX5_RUN_ATTITUDE_COVARIANCE_WARNING; else value &= ~GX5_RUN_ATTITUDE_COVARIANCE_WARNING; }
    bool gx5RunNanInSolutionWarning() const { return (value & GX5_RUN_NAN_IN_SOLUTION_WARNING) > 0; }
    void gx5RunNanInSolutionWarning(bool val) { if(val) value |= GX5_RUN_NAN_IN_SOLUTION_WARNING; else value &= ~GX5_RUN_NAN_IN_SOLUTION_WARNING; }
    bool gx5RunGyroBiasEstHighWarning() const { return (value & GX5_RUN_GYRO_BIAS_EST_HIGH_WARNING) > 0; }
    void gx5RunGyroBiasEstHighWarning(bool val) { if(val) value |= GX5_RUN_GYRO_BIAS_EST_HIGH_WARNING; else value &= ~GX5_RUN_GYRO_BIAS_EST_HIGH_WARNING; }
    bool gx5RunAccelBiasEstHighWarning() const { return (value & GX5_RUN_ACCEL_BIAS_EST_HIGH_WARNING) > 0; }
    void gx5RunAccelBiasEstHighWarning(bool val) { if(val) value |= GX5_RUN_ACCEL_BIAS_EST_HIGH_WARNING; else value &= ~GX5_RUN_ACCEL_BIAS_EST_HIGH_WARNING; }
    bool gx5RunGyroScaleFactorEstHighWarning() const { return (value & GX5_RUN_GYRO_SCALE_FACTOR_EST_HIGH_WARNING) > 0; }
    void gx5RunGyroScaleFactorEstHighWarning(bool val) { if(val) value |= GX5_RUN_GYRO_SCALE_FACTOR_EST_HIGH_WARNING; else value &= ~GX5_RUN_GYRO_SCALE_FACTOR_EST_HIGH_WARNING; }
    bool gx5RunAccelScaleFactorEstHighWarning() const { return (value & GX5_RUN_ACCEL_SCALE_FACTOR_EST_HIGH_WARNING) > 0; }
    void gx5RunAccelScaleFactorEstHighWarning(bool val) { if(val) value |= GX5_RUN_ACCEL_SCALE_FACTOR_EST_HIGH_WARNING; else value &= ~GX5_RUN_ACCEL_SCALE_FACTOR_EST_HIGH_WARNING; }
    bool gx5RunMagBiasEstHighWarning() const { return (value & GX5_RUN_MAG_BIAS_EST_HIGH_WARNING) > 0; }
    void gx5RunMagBiasEstHighWarning(bool val) { if(val) value |= GX5_RUN_MAG_BIAS_EST_HIGH_WARNING; else value &= ~GX5_RUN_MAG_BIAS_EST_HIGH_WARNING; }
    bool gx5RunAntOffsetCorrectionEstHighWarning() const { return (value & GX5_RUN_ANT_OFFSET_CORRECTION_EST_HIGH_WARNING) > 0; }
    void gx5RunAntOffsetCorrectionEstHighWarning(bool val) { if(val) value |= GX5_RUN_ANT_OFFSET_CORRECTION_EST_HIGH_WARNING; else value &= ~GX5_RUN_ANT_OFFSET_CORRECTION_EST_HIGH_WARNING; }
    bool gx5RunMagHardIronEstHighWarning() const { return (value & GX5_RUN_MAG_HARD_IRON_EST_HIGH_WARNING) > 0; }
    void gx5RunMagHardIronEstHighWarning(bool val) { if(val) value |= GX5_RUN_MAG_HARD_IRON_EST_HIGH_WARNING; else value &= ~GX5_RUN_MAG_HARD_IRON_EST_HIGH_WARNING; }
    bool gx5RunMagSoftIronEstHighWarning() const { return (value & GX5_RUN_MAG_SOFT_IRON_EST_HIGH_WARNING) > 0; }
    void gx5RunMagSoftIronEstHighWarning(bool val) { if(val) value |= GX5_RUN_MAG_SOFT_IRON_EST_HIGH_WARNING; else value &= ~GX5_RUN_MAG_SOFT_IRON_EST_HIGH_WARNING; }
    uint16_t gq7FilterCondition() const { return (value & GQ7_FILTER_CONDITION) >> 0; }
    void gq7FilterCondition(uint16_t val) { value = (value & ~GQ7_FILTER_CONDITION) | (val << 0); }
    bool gq7RollPitchWarning() const { return (value & GQ7_ROLL_PITCH_WARNING) > 0; }
    void gq7RollPitchWarning(bool val) { if(val) value |= GQ7_ROLL_PITCH_WARNING; else value &= ~GQ7_ROLL_PITCH_WARNING; }
    bool gq7HeadingWarning() const { return (value & GQ7_HEADING_WARNING) > 0; }
    void gq7HeadingWarning(bool val) { if(val) value |= GQ7_HEADING_WARNING; else value &= ~GQ7_HEADING_WARNING; }
    bool gq7PositionWarning() const { return (value & GQ7_POSITION_WARNING) > 0; }
    void gq7PositionWarning(bool val) { if(val) value |= GQ7_POSITION_WARNING; else value &= ~GQ7_POSITION_WARNING; }
    bool gq7VelocityWarning() const { return (value & GQ7_VELOCITY_WARNING) > 0; }
    void gq7VelocityWarning(bool val) { if(val) value |= GQ7_VELOCITY_WARNING; else value &= ~GQ7_VELOCITY_WARNING; }
    bool gq7ImuBiasWarning() const { return (value & GQ7_IMU_BIAS_WARNING) > 0; }
    void gq7ImuBiasWarning(bool val) { if(val) value |= GQ7_IMU_BIAS_WARNING; else value &= ~GQ7_IMU_BIAS_WARNING; }
    bool gq7GnssClkWarning() const { return (value & GQ7_GNSS_CLK_WARNING) > 0; }
    void gq7GnssClkWarning(bool val) { if(val) value |= GQ7_GNSS_CLK_WARNING; else value &= ~GQ7_GNSS_CLK_WARNING; }
    bool gq7AntennaLeverArmWarning() const { return (value & GQ7_ANTENNA_LEVER_ARM_WARNING) > 0; }
    void gq7AntennaLeverArmWarning(bool val) { if(val) value |= GQ7_ANTENNA_LEVER_ARM_WARNING; else value &= ~GQ7_ANTENNA_LEVER_ARM_WARNING; }
    bool gq7MountingTransformWarning() const { return (value & GQ7_MOUNTING_TRANSFORM_WARNING) > 0; }
    void gq7MountingTransformWarning(bool val) { if(val) value |= GQ7_MOUNTING_TRANSFORM_WARNING; else value &= ~GQ7_MOUNTING_TRANSFORM_WARNING; }
    bool gq7TimeSyncWarning() const { return (value & GQ7_TIME_SYNC_WARNING) > 0; }
    void gq7TimeSyncWarning(bool val) { if(val) value |= GQ7_TIME_SYNC_WARNING; else value &= ~GQ7_TIME_SYNC_WARNING; }
    uint16_t gq7SolutionError() const { return (value & GQ7_SOLUTION_ERROR) >> 12; }
    void gq7SolutionError(uint16_t val) { value = (value & ~GQ7_SOLUTION_ERROR) | (val << 12); }
    
    bool allSet() const { return value == ALL; }
    void setAll() { value |= ALL; }
};

enum class FilterAidingMeasurementType : uint8_t
{
    GNSS              = 1,  ///<  
    DUAL_ANTENNA      = 2,  ///<  
    HEADING           = 3,  ///<  
    PRESSURE          = 4,  ///<  
    MAGNETOMETER      = 5,  ///<  
    SPEED             = 6,  ///<  
    POS_ECEF          = 33,  ///<  
    POS_LLH           = 34,  ///<  
    VEL_ECEF          = 40,  ///<  
    VEL_NED           = 41,  ///<  
    VEL_VEHICLE_FRAME = 42,  ///<  
    HEADING_TRUE      = 49,  ///<  
};

struct FilterMeasurementIndicator : Bitfield<FilterMeasurementIndicator>
{
    enum _enumType : uint8_t
    {
        NONE                  = 0x00,
        ENABLED               = 0x01,  ///<  
        USED                  = 0x02,  ///<  
        RESIDUAL_HIGH_WARNING = 0x04,  ///<  
        SAMPLE_TIME_WARNING   = 0x08,  ///<  
        CONFIGURATION_ERROR   = 0x10,  ///<  
        MAX_NUM_MEAS_EXCEEDED = 0x20,  ///<  
        ALL                   = 0x3F,
    };
    uint8_t value = NONE;
    
    FilterMeasurementIndicator() : value(NONE) {}
    FilterMeasurementIndicator(int val) : value((uint8_t)val) {}
    operator uint8_t() const { return value; }
    FilterMeasurementIndicator& operator=(uint8_t val) { value = val; return *this; }
    FilterMeasurementIndicator& operator=(int val) { value = uint8_t(val); return *this; }
    FilterMeasurementIndicator& operator|=(uint8_t val) { return *this = value | val; }
    FilterMeasurementIndicator& operator&=(uint8_t val) { return *this = value & val; }
    
    bool enabled() const { return (value & ENABLED) > 0; }
    void enabled(bool val) { if(val) value |= ENABLED; else value &= ~ENABLED; }
    bool used() const { return (value & USED) > 0; }
    void used(bool val) { if(val) value |= USED; else value &= ~USED; }
    bool residualHighWarning() const { return (value & RESIDUAL_HIGH_WARNING) > 0; }
    void residualHighWarning(bool val) { if(val) value |= RESIDUAL_HIGH_WARNING; else value &= ~RESIDUAL_HIGH_WARNING; }
    bool sampleTimeWarning() const { return (value & SAMPLE_TIME_WARNING) > 0; }
    void sampleTimeWarning(bool val) { if(val) value |= SAMPLE_TIME_WARNING; else value &= ~SAMPLE_TIME_WARNING; }
    bool configurationError() const { return (value & CONFIGURATION_ERROR) > 0; }
    void configurationError(bool val) { if(val) value |= CONFIGURATION_ERROR; else value &= ~CONFIGURATION_ERROR; }
    bool maxNumMeasExceeded() const { return (value & MAX_NUM_MEAS_EXCEEDED) > 0; }
    void maxNumMeasExceeded(bool val) { if(val) value |= MAX_NUM_MEAS_EXCEEDED; else value &= ~MAX_NUM_MEAS_EXCEEDED; }
    
    bool allSet() const { return value == ALL; }
    void setAll() { value |= ALL; }
};

struct GnssAidStatusFlags : Bitfield<GnssAidStatusFlags>
{
    enum _enumType : uint16_t
    {
        NONE           = 0x0000,
        TIGHT_COUPLING = 0x0001,  ///<  If 1, the Kalman filter is processing raw range information from this GNSS module
        DIFFERENTIAL   = 0x0002,  ///<  If 1, the Kalman filter is processing RTK corrections from this GNSS module
        INTEGER_FIX    = 0x0004,  ///<  If 1, the Kalman filter has an RTK integer fix from this GNSS module, indicating the best position performance possible
        GPS_L1         = 0x0008,  ///<  If 1, the Kalman filter is using GPS L1 measurements
        GPS_L2         = 0x0010,  ///<  If 1, the Kalman filter is using GPS L2 measurements
        GPS_L5         = 0x0020,  ///<  If 1, the Kalman filter is using GPS L5 measurements (not available on the GQ7)
        GLO_L1         = 0x0040,  ///<  If 1, the Kalman filter is using GLONASS L1 measurements
        GLO_L2         = 0x0080,  ///<  If 1, the Kalman filter is using GLONASS L2 measurements
        GAL_E1         = 0x0100,  ///<  If 1, the Kalman filter is using Galileo E1 measurements
        GAL_E5         = 0x0200,  ///<  If 1, the Kalman filter is using Galileo E5 measurements
        GAL_E6         = 0x0400,  ///<  If 1, the Kalman filter is using Galileo E6 measurements
        BEI_B1         = 0x0800,  ///<  If 1, the Kalman filter is using Beidou B1 measurements (not enabled on GQ7 currently)
        BEI_B2         = 0x1000,  ///<  If 1, the Kalman filter is using Beidou B2 measurements (not enabled on GQ7 currently)
        BEI_B3         = 0x2000,  ///<  If 1, the Kalman filter is using Beidou B3 measurements (not available on the GQ7)
        NO_FIX         = 0x4000,  ///<  If 1, this GNSS module is reporting no position fix
        CONFIG_ERROR   = 0x8000,  ///<  If 1, there is likely an issue with the antenna offset for this GNSS module
        ALL            = 0xFFFF,
    };
    uint16_t value = NONE;
    
    GnssAidStatusFlags() : value(NONE) {}
    GnssAidStatusFlags(int val) : value((uint16_t)val) {}
    operator uint16_t() const { return value; }
    GnssAidStatusFlags& operator=(uint16_t val) { value = val; return *this; }
    GnssAidStatusFlags& operator=(int val) { value = uint16_t(val); return *this; }
    GnssAidStatusFlags& operator|=(uint16_t val) { return *this = value | val; }
    GnssAidStatusFlags& operator&=(uint16_t val) { return *this = value & val; }
    
    bool tightCoupling() const { return (value & TIGHT_COUPLING) > 0; }
    void tightCoupling(bool val) { if(val) value |= TIGHT_COUPLING; else value &= ~TIGHT_COUPLING; }
    bool differential() const { return (value & DIFFERENTIAL) > 0; }
    void differential(bool val) { if(val) value |= DIFFERENTIAL; else value &= ~DIFFERENTIAL; }
    bool integerFix() const { return (value & INTEGER_FIX) > 0; }
    void integerFix(bool val) { if(val) value |= INTEGER_FIX; else value &= ~INTEGER_FIX; }
    bool gpsL1() const { return (value & GPS_L1) > 0; }
    void gpsL1(bool val) { if(val) value |= GPS_L1; else value &= ~GPS_L1; }
    bool gpsL2() const { return (value & GPS_L2) > 0; }
    void gpsL2(bool val) { if(val) value |= GPS_L2; else value &= ~GPS_L2; }
    bool gpsL5() const { return (value & GPS_L5) > 0; }
    void gpsL5(bool val) { if(val) value |= GPS_L5; else value &= ~GPS_L5; }
    bool gloL1() const { return (value & GLO_L1) > 0; }
    void gloL1(bool val) { if(val) value |= GLO_L1; else value &= ~GLO_L1; }
    bool gloL2() const { return (value & GLO_L2) > 0; }
    void gloL2(bool val) { if(val) value |= GLO_L2; else value &= ~GLO_L2; }
    bool galE1() const { return (value & GAL_E1) > 0; }
    void galE1(bool val) { if(val) value |= GAL_E1; else value &= ~GAL_E1; }
    bool galE5() const { return (value & GAL_E5) > 0; }
    void galE5(bool val) { if(val) value |= GAL_E5; else value &= ~GAL_E5; }
    bool galE6() const { return (value & GAL_E6) > 0; }
    void galE6(bool val) { if(val) value |= GAL_E6; else value &= ~GAL_E6; }
    bool beiB1() const { return (value & BEI_B1) > 0; }
    void beiB1(bool val) { if(val) value |= BEI_B1; else value &= ~BEI_B1; }
    bool beiB2() const { return (value & BEI_B2) > 0; }
    void beiB2(bool val) { if(val) value |= BEI_B2; else value &= ~BEI_B2; }
    bool beiB3() const { return (value & BEI_B3) > 0; }
    void beiB3(bool val) { if(val) value |= BEI_B3; else value &= ~BEI_B3; }
    bool noFix() const { return (value & NO_FIX) > 0; }
    void noFix(bool val) { if(val) value |= NO_FIX; else value &= ~NO_FIX; }
    bool configError() const { return (value & CONFIG_ERROR) > 0; }
    void configError(bool val) { if(val) value |= CONFIG_ERROR; else value &= ~CONFIG_ERROR; }
    
    bool allSet() const { return value == ALL; }
    void setAll() { value |= ALL; }
};


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_position_llh  (0x82,0x01) Position Llh [CPP]
/// Filter reported position in the WGS84 geodetic frame.
///
///@{

struct PositionLlh
{
    double latitude = 0; ///< [degrees]
    double longitude = 0; ///< [degrees]
    double ellipsoid_height = 0; ///< [meters]
    uint16_t valid_flags = 0; ///< 0 - Invalid, 1 - valid
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_POS_LLH;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "PositionLlh";
    static constexpr const char* DOC_NAME = "LLH Position";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(latitude,longitude,ellipsoid_height,valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(latitude),std::ref(longitude),std::ref(ellipsoid_height),std::ref(valid_flags));
    }
};
void insert(Serializer& serializer, const PositionLlh& self);
void extract(Serializer& serializer, PositionLlh& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_velocity_ned  (0x82,0x02) Velocity Ned [CPP]
/// Filter reported velocity in the NED local-level frame.
///
///@{

struct VelocityNed
{
    float north = 0; ///< [meters/second]
    float east = 0; ///< [meters/second]
    float down = 0; ///< [meters/second]
    uint16_t valid_flags = 0; ///< 0 - Invalid, 1 - valid
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_VEL_NED;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "VelocityNed";
    static constexpr const char* DOC_NAME = "VelocityNed";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(north,east,down,valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(north),std::ref(east),std::ref(down),std::ref(valid_flags));
    }
};
void insert(Serializer& serializer, const VelocityNed& self);
void extract(Serializer& serializer, VelocityNed& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_attitude_quaternion  (0x82,0x03) Attitude Quaternion [CPP]
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

struct AttitudeQuaternion
{
    Quatf q; ///< Quaternion elements EQSTART q = (q_w, q_x, q_y, q_z) EQEND
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_ATT_QUATERNION;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "AttitudeQuaternion";
    static constexpr const char* DOC_NAME = "AttitudeQuaternion";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(q[0],q[1],q[2],q[3],valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(q[0]),std::ref(q[1]),std::ref(q[2]),std::ref(q[3]),std::ref(valid_flags));
    }
};
void insert(Serializer& serializer, const AttitudeQuaternion& self);
void extract(Serializer& serializer, AttitudeQuaternion& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_attitude_dcm  (0x82,0x04) Attitude Dcm [CPP]
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

struct AttitudeDcm
{
    Matrix3f dcm; ///< Matrix elements in row-major order.
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_ATT_MATRIX;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "AttitudeDcm";
    static constexpr const char* DOC_NAME = "AttitudeDcm";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(dcm[0],dcm[1],dcm[2],dcm[3],dcm[4],dcm[5],dcm[6],dcm[7],dcm[8],valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(dcm[0]),std::ref(dcm[1]),std::ref(dcm[2]),std::ref(dcm[3]),std::ref(dcm[4]),std::ref(dcm[5]),std::ref(dcm[6]),std::ref(dcm[7]),std::ref(dcm[8]),std::ref(valid_flags));
    }
};
void insert(Serializer& serializer, const AttitudeDcm& self);
void extract(Serializer& serializer, AttitudeDcm& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_euler_angles  (0x82,0x05) Euler Angles [CPP]
/// Filter reported Euler angles describing the orientation of the device with respect to the NED local-level frame.
/// The Euler angles are reported in 3-2-1 (Yaw-Pitch-Roll, AKA Aircraft) order.
///
///@{

struct EulerAngles
{
    float roll = 0; ///< [radians]
    float pitch = 0; ///< [radians]
    float yaw = 0; ///< [radians]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_ATT_EULER_ANGLES;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "EulerAngles";
    static constexpr const char* DOC_NAME = "EulerAngles";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(roll,pitch,yaw,valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(roll),std::ref(pitch),std::ref(yaw),std::ref(valid_flags));
    }
};
void insert(Serializer& serializer, const EulerAngles& self);
void extract(Serializer& serializer, EulerAngles& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_gyro_bias  (0x82,0x06) Gyro Bias [CPP]
/// Filter reported gyro bias expressed in the sensor frame.
///
///@{

struct GyroBias
{
    Vector3f bias; ///< (x, y, z) [radians/second]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_GYRO_BIAS;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "GyroBias";
    static constexpr const char* DOC_NAME = "GyroBias";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(bias[0],bias[1],bias[2],valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(bias[0]),std::ref(bias[1]),std::ref(bias[2]),std::ref(valid_flags));
    }
};
void insert(Serializer& serializer, const GyroBias& self);
void extract(Serializer& serializer, GyroBias& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_accel_bias  (0x82,0x07) Accel Bias [CPP]
/// Filter reported accelerometer bias expressed in the sensor frame.
///
///@{

struct AccelBias
{
    Vector3f bias; ///< (x, y, z) [meters/second^2]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_ACCEL_BIAS;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "AccelBias";
    static constexpr const char* DOC_NAME = "AccelBias";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(bias[0],bias[1],bias[2],valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(bias[0]),std::ref(bias[1]),std::ref(bias[2]),std::ref(valid_flags));
    }
};
void insert(Serializer& serializer, const AccelBias& self);
void extract(Serializer& serializer, AccelBias& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_position_llh_uncertainty  (0x82,0x08) Position Llh Uncertainty [CPP]
/// Filter reported 1-sigma position uncertainty in the NED local-level frame.
///
///@{

struct PositionLlhUncertainty
{
    float north = 0; ///< [meters]
    float east = 0; ///< [meters]
    float down = 0; ///< [meters]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_POS_UNCERTAINTY;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "PositionLlhUncertainty";
    static constexpr const char* DOC_NAME = "LLH Position Uncertainty";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(north,east,down,valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(north),std::ref(east),std::ref(down),std::ref(valid_flags));
    }
};
void insert(Serializer& serializer, const PositionLlhUncertainty& self);
void extract(Serializer& serializer, PositionLlhUncertainty& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_velocity_ned_uncertainty  (0x82,0x09) Velocity Ned Uncertainty [CPP]
/// Filter reported 1-sigma velocity uncertainties in the NED local-level frame.
///
///@{

struct VelocityNedUncertainty
{
    float north = 0; ///< [meters/second]
    float east = 0; ///< [meters/second]
    float down = 0; ///< [meters/second]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_VEL_UNCERTAINTY;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "VelocityNedUncertainty";
    static constexpr const char* DOC_NAME = "NED Velocity Uncertainty";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(north,east,down,valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(north),std::ref(east),std::ref(down),std::ref(valid_flags));
    }
};
void insert(Serializer& serializer, const VelocityNedUncertainty& self);
void extract(Serializer& serializer, VelocityNedUncertainty& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_euler_angles_uncertainty  (0x82,0x0A) Euler Angles Uncertainty [CPP]
/// Filter reported 1-sigma Euler angle uncertainties.
/// The uncertainties are reported in 3-2-1 (Yaw-Pitch-Roll, AKA Aircraft) order.
///
///@{

struct EulerAnglesUncertainty
{
    float roll = 0; ///< [radians]
    float pitch = 0; ///< [radians]
    float yaw = 0; ///< [radians]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_ATT_UNCERTAINTY_EULER;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "EulerAnglesUncertainty";
    static constexpr const char* DOC_NAME = "EulerAnglesUncertainty";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(roll,pitch,yaw,valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(roll),std::ref(pitch),std::ref(yaw),std::ref(valid_flags));
    }
};
void insert(Serializer& serializer, const EulerAnglesUncertainty& self);
void extract(Serializer& serializer, EulerAnglesUncertainty& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_gyro_bias_uncertainty  (0x82,0x0B) Gyro Bias Uncertainty [CPP]
/// Filter reported 1-sigma gyro bias uncertainties expressed in the sensor frame.
///
///@{

struct GyroBiasUncertainty
{
    Vector3f bias_uncert; ///< (x,y,z) [radians/sec]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_GYRO_BIAS_UNCERTAINTY;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "GyroBiasUncertainty";
    static constexpr const char* DOC_NAME = "GyroBiasUncertainty";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(bias_uncert[0],bias_uncert[1],bias_uncert[2],valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(bias_uncert[0]),std::ref(bias_uncert[1]),std::ref(bias_uncert[2]),std::ref(valid_flags));
    }
};
void insert(Serializer& serializer, const GyroBiasUncertainty& self);
void extract(Serializer& serializer, GyroBiasUncertainty& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_accel_bias_uncertainty  (0x82,0x0C) Accel Bias Uncertainty [CPP]
/// Filter reported 1-sigma accelerometer bias uncertainties expressed in the sensor frame.
///
///@{

struct AccelBiasUncertainty
{
    Vector3f bias_uncert; ///< (x,y,z) [meters/second^2]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_ACCEL_BIAS_UNCERTAINTY;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "AccelBiasUncertainty";
    static constexpr const char* DOC_NAME = "AccelBiasUncertainty";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(bias_uncert[0],bias_uncert[1],bias_uncert[2],valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(bias_uncert[0]),std::ref(bias_uncert[1]),std::ref(bias_uncert[2]),std::ref(valid_flags));
    }
};
void insert(Serializer& serializer, const AccelBiasUncertainty& self);
void extract(Serializer& serializer, AccelBiasUncertainty& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_timestamp  (0x82,0x11) Timestamp [CPP]
/// GPS timestamp of the Filter data
/// 
/// Should the PPS become unavailable, the device will revert to its internal clock, which will cause the reported time to drift from true GPS time.
/// Upon recovering from a PPS outage, the user should expect a jump in the reported GPS time due to the accumulation of internal clock error.
/// If synchronization to an external clock or onboard GNSS receiver (for products that have one) is disabled, this time is equivalent to internal system time.
/// 
/// Note: this data field may be deprecated in the future. The more flexible shared data field (0x82, 0xD3) should be used instead.
///
///@{

struct Timestamp
{
    double tow = 0; ///< GPS Time of Week [seconds]
    uint16_t week_number = 0; ///< GPS Week Number since 1980 [weeks]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_FILTER_TIMESTAMP;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "Timestamp";
    static constexpr const char* DOC_NAME = "Timestamp";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(tow,week_number,valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(tow),std::ref(week_number),std::ref(valid_flags));
    }
};
void insert(Serializer& serializer, const Timestamp& self);
void extract(Serializer& serializer, Timestamp& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_status  (0x82,0x10) Status [CPP]
/// Device-specific filter status indicators.
///
///@{

struct Status
{
    FilterMode filter_state = static_cast<FilterMode>(0); ///< Device-specific filter state.  Please consult the user manual for definition.
    FilterDynamicsMode dynamics_mode = static_cast<FilterDynamicsMode>(0); ///< Device-specific dynamics mode. Please consult the user manual for definition.
    FilterStatusFlags status_flags; ///< Device-specific status flags.  Please consult the user manual for definition.
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_FILTER_STATUS;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "Status";
    static constexpr const char* DOC_NAME = "Status";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(filter_state,dynamics_mode,status_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(filter_state),std::ref(dynamics_mode),std::ref(status_flags));
    }
};
void insert(Serializer& serializer, const Status& self);
void extract(Serializer& serializer, Status& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_linear_accel  (0x82,0x0D) Linear Accel [CPP]
/// Filter-compensated linear acceleration expressed in the vehicle frame.
/// Note: The estimated gravity has been removed from this data leaving only linear acceleration.
///
///@{

struct LinearAccel
{
    Vector3f accel; ///< (x,y,z) [meters/second^2]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_LINEAR_ACCELERATION;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "LinearAccel";
    static constexpr const char* DOC_NAME = "LinearAccel";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(accel[0],accel[1],accel[2],valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(accel[0]),std::ref(accel[1]),std::ref(accel[2]),std::ref(valid_flags));
    }
};
void insert(Serializer& serializer, const LinearAccel& self);
void extract(Serializer& serializer, LinearAccel& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_gravity_vector  (0x82,0x13) Gravity Vector [CPP]
/// Filter reported gravity vector expressed in the vehicle frame.
///
///@{

struct GravityVector
{
    Vector3f gravity; ///< (x, y, z) [meters/second^2]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_GRAVITY_VECTOR;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "GravityVector";
    static constexpr const char* DOC_NAME = "GravityVector";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(gravity[0],gravity[1],gravity[2],valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(gravity[0]),std::ref(gravity[1]),std::ref(gravity[2]),std::ref(valid_flags));
    }
};
void insert(Serializer& serializer, const GravityVector& self);
void extract(Serializer& serializer, GravityVector& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_comp_accel  (0x82,0x1C) Comp Accel [CPP]
/// Filter-compensated acceleration expressed in the vehicle frame.
///
///@{

struct CompAccel
{
    Vector3f accel; ///< (x,y,z) [meters/second^2]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_COMPENSATED_ACCELERATION;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "CompAccel";
    static constexpr const char* DOC_NAME = "Compensated Acceleration";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(accel[0],accel[1],accel[2],valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(accel[0]),std::ref(accel[1]),std::ref(accel[2]),std::ref(valid_flags));
    }
};
void insert(Serializer& serializer, const CompAccel& self);
void extract(Serializer& serializer, CompAccel& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_comp_angular_rate  (0x82,0x0E) Comp Angular Rate [CPP]
/// Filter-compensated angular rate expressed in the vehicle frame.
///
///@{

struct CompAngularRate
{
    Vector3f gyro; ///< (x, y, z) [radians/second]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_COMPENSATED_ANGULAR_RATE;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "CompAngularRate";
    static constexpr const char* DOC_NAME = "CompAngularRate";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(gyro[0],gyro[1],gyro[2],valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(gyro[0]),std::ref(gyro[1]),std::ref(gyro[2]),std::ref(valid_flags));
    }
};
void insert(Serializer& serializer, const CompAngularRate& self);
void extract(Serializer& serializer, CompAngularRate& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_quaternion_attitude_uncertainty  (0x82,0x12) Quaternion Attitude Uncertainty [CPP]
/// Filter reported quaternion uncertainties.
///
///@{

struct QuaternionAttitudeUncertainty
{
    Quatf q; ///< [dimensionless]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_ATT_UNCERTAINTY_QUATERNION;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "QuaternionAttitudeUncertainty";
    static constexpr const char* DOC_NAME = "QuaternionAttitudeUncertainty";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(q[0],q[1],q[2],q[3],valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(q[0]),std::ref(q[1]),std::ref(q[2]),std::ref(q[3]),std::ref(valid_flags));
    }
};
void insert(Serializer& serializer, const QuaternionAttitudeUncertainty& self);
void extract(Serializer& serializer, QuaternionAttitudeUncertainty& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_wgs84_gravity_mag  (0x82,0x0F) Wgs84 Gravity Mag [CPP]
/// Filter reported WGS84 gravity magnitude.
///
///@{

struct Wgs84GravityMag
{
    float magnitude = 0; ///< [meters/second^2]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_WGS84_GRAVITY;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "Wgs84GravityMag";
    static constexpr const char* DOC_NAME = "Wgs84GravityMag";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(magnitude,valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(magnitude),std::ref(valid_flags));
    }
};
void insert(Serializer& serializer, const Wgs84GravityMag& self);
void extract(Serializer& serializer, Wgs84GravityMag& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_heading_update_state  (0x82,0x14) Heading Update State [CPP]
/// Filter reported heading update state.
/// 
/// Heading updates can be applied from the sources listed below.  Note, some of these sources may be combined.
/// The heading value is always relative to true north.
///
///@{

struct HeadingUpdateState
{
    enum class HeadingSource : uint16_t
    {
        NONE                 = 0,  ///<  
        MAGNETOMETER         = 1,  ///<  
        GNSS_VELOCITY_VECTOR = 2,  ///<  
        EXTERNAL             = 4,  ///<  
        DUAL_ANTENNA         = 8,  ///<  
    };
    
    float heading = 0; ///< [radians]
    float heading_1sigma = 0; ///< [radians]
    HeadingSource source = static_cast<HeadingSource>(0);
    uint16_t valid_flags = 0; ///< 1 if a valid heading update was received in 2 seconds, 0 otherwise.
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_HEADING_UPDATE_STATE;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "HeadingUpdateState";
    static constexpr const char* DOC_NAME = "HeadingUpdateState";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(heading,heading_1sigma,source,valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(heading),std::ref(heading_1sigma),std::ref(source),std::ref(valid_flags));
    }
};
void insert(Serializer& serializer, const HeadingUpdateState& self);
void extract(Serializer& serializer, HeadingUpdateState& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_magnetic_model  (0x82,0x15) Magnetic Model [CPP]
/// The World Magnetic Model is used for this data. Please refer to the device user manual for the current version of the model.
/// A valid GNSS location is required for the model to be valid.
///
///@{

struct MagneticModel
{
    float intensity_north = 0; ///< [Gauss]
    float intensity_east = 0; ///< [Gauss]
    float intensity_down = 0; ///< [Gauss]
    float inclination = 0; ///< [radians]
    float declination = 0; ///< [radians]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_MAGNETIC_MODEL;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "MagneticModel";
    static constexpr const char* DOC_NAME = "MagneticModel";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(intensity_north,intensity_east,intensity_down,inclination,declination,valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(intensity_north),std::ref(intensity_east),std::ref(intensity_down),std::ref(inclination),std::ref(declination),std::ref(valid_flags));
    }
};
void insert(Serializer& serializer, const MagneticModel& self);
void extract(Serializer& serializer, MagneticModel& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_accel_scale_factor  (0x82,0x17) Accel Scale Factor [CPP]
/// Filter reported accelerometer scale factor expressed in the sensor frame.
///
///@{

struct AccelScaleFactor
{
    Vector3f scale_factor; ///< (x,y,z) [dimensionless]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_ACCEL_SCALE_FACTOR;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "AccelScaleFactor";
    static constexpr const char* DOC_NAME = "AccelScaleFactor";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(scale_factor[0],scale_factor[1],scale_factor[2],valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(scale_factor[0]),std::ref(scale_factor[1]),std::ref(scale_factor[2]),std::ref(valid_flags));
    }
};
void insert(Serializer& serializer, const AccelScaleFactor& self);
void extract(Serializer& serializer, AccelScaleFactor& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_accel_scale_factor_uncertainty  (0x82,0x19) Accel Scale Factor Uncertainty [CPP]
/// Filter reported 1-sigma accelerometer scale factor uncertainty expressed in the sensor frame.
///
///@{

struct AccelScaleFactorUncertainty
{
    Vector3f scale_factor_uncert; ///< (x,y,z) [dimensionless]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_ACCEL_SCALE_FACTOR_UNCERTAINTY;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "AccelScaleFactorUncertainty";
    static constexpr const char* DOC_NAME = "AccelScaleFactorUncertainty";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(scale_factor_uncert[0],scale_factor_uncert[1],scale_factor_uncert[2],valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(scale_factor_uncert[0]),std::ref(scale_factor_uncert[1]),std::ref(scale_factor_uncert[2]),std::ref(valid_flags));
    }
};
void insert(Serializer& serializer, const AccelScaleFactorUncertainty& self);
void extract(Serializer& serializer, AccelScaleFactorUncertainty& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_gyro_scale_factor  (0x82,0x16) Gyro Scale Factor [CPP]
/// Filter reported gyro scale factor expressed in the sensor frame.
///
///@{

struct GyroScaleFactor
{
    Vector3f scale_factor; ///< (x,y,z) [dimensionless]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_GYRO_SCALE_FACTOR;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "GyroScaleFactor";
    static constexpr const char* DOC_NAME = "GyroScaleFactor";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(scale_factor[0],scale_factor[1],scale_factor[2],valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(scale_factor[0]),std::ref(scale_factor[1]),std::ref(scale_factor[2]),std::ref(valid_flags));
    }
};
void insert(Serializer& serializer, const GyroScaleFactor& self);
void extract(Serializer& serializer, GyroScaleFactor& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_gyro_scale_factor_uncertainty  (0x82,0x18) Gyro Scale Factor Uncertainty [CPP]
/// Filter reported 1-sigma gyro scale factor uncertainty expressed in the sensor frame.
///
///@{

struct GyroScaleFactorUncertainty
{
    Vector3f scale_factor_uncert; ///< (x,y,z) [dimensionless]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_GYRO_SCALE_FACTOR_UNCERTAINTY;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "GyroScaleFactorUncertainty";
    static constexpr const char* DOC_NAME = "GyroScaleFactorUncertainty";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(scale_factor_uncert[0],scale_factor_uncert[1],scale_factor_uncert[2],valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(scale_factor_uncert[0]),std::ref(scale_factor_uncert[1]),std::ref(scale_factor_uncert[2]),std::ref(valid_flags));
    }
};
void insert(Serializer& serializer, const GyroScaleFactorUncertainty& self);
void extract(Serializer& serializer, GyroScaleFactorUncertainty& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_mag_bias  (0x82,0x1A) Mag Bias [CPP]
/// Filter reported magnetometer bias expressed in the sensor frame.
///
///@{

struct MagBias
{
    Vector3f bias; ///< (x,y,z) [Gauss]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_MAG_BIAS;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "MagBias";
    static constexpr const char* DOC_NAME = "MagBias";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(bias[0],bias[1],bias[2],valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(bias[0]),std::ref(bias[1]),std::ref(bias[2]),std::ref(valid_flags));
    }
};
void insert(Serializer& serializer, const MagBias& self);
void extract(Serializer& serializer, MagBias& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_mag_bias_uncertainty  (0x82,0x1B) Mag Bias Uncertainty [CPP]
/// Filter reported 1-sigma magnetometer bias uncertainty expressed in the sensor frame.
///
///@{

struct MagBiasUncertainty
{
    Vector3f bias_uncert; ///< (x,y,z) [Gauss]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_MAG_BIAS_UNCERTAINTY;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "MagBiasUncertainty";
    static constexpr const char* DOC_NAME = "MagBiasUncertainty";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(bias_uncert[0],bias_uncert[1],bias_uncert[2],valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(bias_uncert[0]),std::ref(bias_uncert[1]),std::ref(bias_uncert[2]),std::ref(valid_flags));
    }
};
void insert(Serializer& serializer, const MagBiasUncertainty& self);
void extract(Serializer& serializer, MagBiasUncertainty& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_standard_atmosphere  (0x82,0x20) Standard Atmosphere [CPP]
/// Filter reported standard atmosphere parameters.
/// 
/// The US 1976 Standard Atmosphere Model is used. A valid GNSS location is required for the model to be valid.
///
///@{

struct StandardAtmosphere
{
    float geometric_altitude = 0; ///< Input into calculation [meters]
    float geopotential_altitude = 0; ///< [meters]
    float standard_temperature = 0; ///< [degC]
    float standard_pressure = 0; ///< [milliBar]
    float standard_density = 0; ///< [kilogram/meter^3]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_STANDARD_ATMOSPHERE_DATA;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "StandardAtmosphere";
    static constexpr const char* DOC_NAME = "StandardAtmosphere";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(geometric_altitude,geopotential_altitude,standard_temperature,standard_pressure,standard_density,valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(geometric_altitude),std::ref(geopotential_altitude),std::ref(standard_temperature),std::ref(standard_pressure),std::ref(standard_density),std::ref(valid_flags));
    }
};
void insert(Serializer& serializer, const StandardAtmosphere& self);
void extract(Serializer& serializer, StandardAtmosphere& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_pressure_altitude  (0x82,0x21) Pressure Altitude [CPP]
/// Filter reported pressure altitude.
/// 
/// The US 1976 Standard Atmosphere Model is used to calculate the pressure altitude in meters.
/// A valid pressure sensor reading is required for the pressure altitude to be valid.
/// The minimum pressure reading supported by the model is 0.0037 mBar, corresponding to an altitude of 84,852 meters.
///
///@{

struct PressureAltitude
{
    float pressure_altitude = 0; ///< [meters]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_PRESSURE_ALTITUDE_DATA;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "PressureAltitude";
    static constexpr const char* DOC_NAME = "PressureAltitude";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(pressure_altitude,valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(pressure_altitude),std::ref(valid_flags));
    }
};
void insert(Serializer& serializer, const PressureAltitude& self);
void extract(Serializer& serializer, PressureAltitude& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_density_altitude  (0x82,0x22) Density Altitude [CPP]
///
///@{

struct DensityAltitude
{
    float density_altitude = 0; ///< m
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_DENSITY_ALTITUDE_DATA;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "DensityAltitude";
    static constexpr const char* DOC_NAME = "DensityAltitude";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(density_altitude,valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(density_altitude),std::ref(valid_flags));
    }
};
void insert(Serializer& serializer, const DensityAltitude& self);
void extract(Serializer& serializer, DensityAltitude& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_antenna_offset_correction  (0x82,0x30) Antenna Offset Correction [CPP]
/// Filter reported GNSS antenna offset in vehicle frame.
/// 
/// This offset added to any previously stored offset vector to compensate for errors in definition.
///
///@{

struct AntennaOffsetCorrection
{
    Vector3f offset; ///< (x,y,z) [meters]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_ANTENNA_OFFSET_CORRECTION;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "AntennaOffsetCorrection";
    static constexpr const char* DOC_NAME = "AntennaOffsetCorrection";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(offset[0],offset[1],offset[2],valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(offset[0]),std::ref(offset[1]),std::ref(offset[2]),std::ref(valid_flags));
    }
};
void insert(Serializer& serializer, const AntennaOffsetCorrection& self);
void extract(Serializer& serializer, AntennaOffsetCorrection& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_antenna_offset_correction_uncertainty  (0x82,0x31) Antenna Offset Correction Uncertainty [CPP]
/// Filter reported 1-sigma GNSS antenna offset uncertainties in vehicle frame.
///
///@{

struct AntennaOffsetCorrectionUncertainty
{
    Vector3f offset_uncert; ///< (x,y,z) [meters]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_ANTENNA_OFFSET_CORRECTION_UNCERTAINTY;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "AntennaOffsetCorrectionUncertainty";
    static constexpr const char* DOC_NAME = "AntennaOffsetCorrectionUncertainty";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(offset_uncert[0],offset_uncert[1],offset_uncert[2],valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(offset_uncert[0]),std::ref(offset_uncert[1]),std::ref(offset_uncert[2]),std::ref(valid_flags));
    }
};
void insert(Serializer& serializer, const AntennaOffsetCorrectionUncertainty& self);
void extract(Serializer& serializer, AntennaOffsetCorrectionUncertainty& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_multi_antenna_offset_correction  (0x82,0x34) Multi Antenna Offset Correction [CPP]
/// Filter reported GNSS antenna offset in vehicle frame.
/// 
/// This offset added to any previously stored offset vector to compensate for errors in definition.
///
///@{

struct MultiAntennaOffsetCorrection
{
    uint8_t receiver_id = 0; ///< Receiver ID for the receiver to which the antenna is attached
    Vector3f offset; ///< (x,y,z) [meters]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_MULTI_ANTENNA_OFFSET_CORRECTION;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "MultiAntennaOffsetCorrection";
    static constexpr const char* DOC_NAME = "MultiAntennaOffsetCorrection";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(receiver_id,offset[0],offset[1],offset[2],valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(receiver_id),std::ref(offset[0]),std::ref(offset[1]),std::ref(offset[2]),std::ref(valid_flags));
    }
};
void insert(Serializer& serializer, const MultiAntennaOffsetCorrection& self);
void extract(Serializer& serializer, MultiAntennaOffsetCorrection& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_multi_antenna_offset_correction_uncertainty  (0x82,0x35) Multi Antenna Offset Correction Uncertainty [CPP]
/// Filter reported 1-sigma GNSS antenna offset uncertainties in vehicle frame.
///
///@{

struct MultiAntennaOffsetCorrectionUncertainty
{
    uint8_t receiver_id = 0; ///< Receiver ID for the receiver to which the antenna is attached
    Vector3f offset_uncert; ///< (x,y,z) [meters]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_MULTI_ANTENNA_OFFSET_CORRECTION_UNCERTAINTY;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "MultiAntennaOffsetCorrectionUncertainty";
    static constexpr const char* DOC_NAME = "MultiAntennaOffsetCorrectionUncertainty";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(receiver_id,offset_uncert[0],offset_uncert[1],offset_uncert[2],valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(receiver_id),std::ref(offset_uncert[0]),std::ref(offset_uncert[1]),std::ref(offset_uncert[2]),std::ref(valid_flags));
    }
};
void insert(Serializer& serializer, const MultiAntennaOffsetCorrectionUncertainty& self);
void extract(Serializer& serializer, MultiAntennaOffsetCorrectionUncertainty& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_magnetometer_offset  (0x82,0x25) Magnetometer Offset [CPP]
/// Filter reported magnetometer hard iron offset in sensor frame.
/// 
/// This offset added to any previously stored hard iron offset vector to compensate for magnetometer in-run bias errors.
///
///@{

struct MagnetometerOffset
{
    Vector3f hard_iron; ///< (x,y,z) [Gauss]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_MAG_COMPENSATION_OFFSET;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "MagnetometerOffset";
    static constexpr const char* DOC_NAME = "MagnetometerOffset";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(hard_iron[0],hard_iron[1],hard_iron[2],valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(hard_iron[0]),std::ref(hard_iron[1]),std::ref(hard_iron[2]),std::ref(valid_flags));
    }
};
void insert(Serializer& serializer, const MagnetometerOffset& self);
void extract(Serializer& serializer, MagnetometerOffset& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_magnetometer_matrix  (0x82,0x26) Magnetometer Matrix [CPP]
/// Filter reported magnetometer soft iron matrix in sensor frame.
/// 
/// This matrix is post multiplied to any previously stored soft iron matrix to compensate for magnetometer in-run errors.
///
///@{

struct MagnetometerMatrix
{
    Matrix3f soft_iron; ///< Row-major [dimensionless]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_MAG_COMPENSATION_MATRIX;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "MagnetometerMatrix";
    static constexpr const char* DOC_NAME = "MagnetometerMatrix";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(soft_iron[0],soft_iron[1],soft_iron[2],soft_iron[3],soft_iron[4],soft_iron[5],soft_iron[6],soft_iron[7],soft_iron[8],valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(soft_iron[0]),std::ref(soft_iron[1]),std::ref(soft_iron[2]),std::ref(soft_iron[3]),std::ref(soft_iron[4]),std::ref(soft_iron[5]),std::ref(soft_iron[6]),std::ref(soft_iron[7]),std::ref(soft_iron[8]),std::ref(valid_flags));
    }
};
void insert(Serializer& serializer, const MagnetometerMatrix& self);
void extract(Serializer& serializer, MagnetometerMatrix& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_magnetometer_offset_uncertainty  (0x82,0x28) Magnetometer Offset Uncertainty [CPP]
/// Filter reported 1-sigma magnetometer hard iron offset uncertainties in sensor frame.
///
///@{

struct MagnetometerOffsetUncertainty
{
    Vector3f hard_iron_uncertainty; ///< (x,y,z) [Gauss]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_MAG_COMPENSATION_OFFSET_UNCERTAINTY;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "MagnetometerOffsetUncertainty";
    static constexpr const char* DOC_NAME = "MagnetometerOffsetUncertainty";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(hard_iron_uncertainty[0],hard_iron_uncertainty[1],hard_iron_uncertainty[2],valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(hard_iron_uncertainty[0]),std::ref(hard_iron_uncertainty[1]),std::ref(hard_iron_uncertainty[2]),std::ref(valid_flags));
    }
};
void insert(Serializer& serializer, const MagnetometerOffsetUncertainty& self);
void extract(Serializer& serializer, MagnetometerOffsetUncertainty& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_magnetometer_matrix_uncertainty  (0x82,0x29) Magnetometer Matrix Uncertainty [CPP]
/// Filter reported 1-sigma magnetometer soft iron matrix uncertainties in sensor frame.
///
///@{

struct MagnetometerMatrixUncertainty
{
    Matrix3f soft_iron_uncertainty; ///< Row-major [dimensionless]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_MAG_COMPENSATION_MATRIX_UNCERTAINTY;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "MagnetometerMatrixUncertainty";
    static constexpr const char* DOC_NAME = "MagnetometerMatrixUncertainty";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(soft_iron_uncertainty[0],soft_iron_uncertainty[1],soft_iron_uncertainty[2],soft_iron_uncertainty[3],soft_iron_uncertainty[4],soft_iron_uncertainty[5],soft_iron_uncertainty[6],soft_iron_uncertainty[7],soft_iron_uncertainty[8],valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(soft_iron_uncertainty[0]),std::ref(soft_iron_uncertainty[1]),std::ref(soft_iron_uncertainty[2]),std::ref(soft_iron_uncertainty[3]),std::ref(soft_iron_uncertainty[4]),std::ref(soft_iron_uncertainty[5]),std::ref(soft_iron_uncertainty[6]),std::ref(soft_iron_uncertainty[7]),std::ref(soft_iron_uncertainty[8]),std::ref(valid_flags));
    }
};
void insert(Serializer& serializer, const MagnetometerMatrixUncertainty& self);
void extract(Serializer& serializer, MagnetometerMatrixUncertainty& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_magnetometer_covariance_matrix  (0x82,0x2A) Magnetometer Covariance Matrix [CPP]
///
///@{

struct MagnetometerCovarianceMatrix
{
    Matrix3f covariance;
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_MAG_COVARIANCE;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "MagnetometerCovarianceMatrix";
    static constexpr const char* DOC_NAME = "MagnetometerCovarianceMatrix";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(covariance[0],covariance[1],covariance[2],covariance[3],covariance[4],covariance[5],covariance[6],covariance[7],covariance[8],valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(covariance[0]),std::ref(covariance[1]),std::ref(covariance[2]),std::ref(covariance[3]),std::ref(covariance[4]),std::ref(covariance[5]),std::ref(covariance[6]),std::ref(covariance[7]),std::ref(covariance[8]),std::ref(valid_flags));
    }
};
void insert(Serializer& serializer, const MagnetometerCovarianceMatrix& self);
void extract(Serializer& serializer, MagnetometerCovarianceMatrix& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_magnetometer_residual_vector  (0x82,0x2C) Magnetometer Residual Vector [CPP]
/// Filter reported magnetometer measurement residuals in vehicle frame.
///
///@{

struct MagnetometerResidualVector
{
    Vector3f residual; ///< (x,y,z) [Gauss]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_MAG_RESIDUAL;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "MagnetometerResidualVector";
    static constexpr const char* DOC_NAME = "MagnetometerResidualVector";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(residual[0],residual[1],residual[2],valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(residual[0]),std::ref(residual[1]),std::ref(residual[2]),std::ref(valid_flags));
    }
};
void insert(Serializer& serializer, const MagnetometerResidualVector& self);
void extract(Serializer& serializer, MagnetometerResidualVector& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_clock_correction  (0x82,0x32) Clock Correction [CPP]
/// Filter reported GNSS receiver clock error parameters.
///
///@{

struct ClockCorrection
{
    uint8_t receiver_id = 0; ///< 1, 2, etc.
    float bias = 0; ///< [seconds]
    float bias_drift = 0; ///< [seconds/second]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_CLOCK_CORRECTION;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "ClockCorrection";
    static constexpr const char* DOC_NAME = "ClockCorrection";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(receiver_id,bias,bias_drift,valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(receiver_id),std::ref(bias),std::ref(bias_drift),std::ref(valid_flags));
    }
};
void insert(Serializer& serializer, const ClockCorrection& self);
void extract(Serializer& serializer, ClockCorrection& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_clock_correction_uncertainty  (0x82,0x33) Clock Correction Uncertainty [CPP]
/// Filter reported 1-sigma GNSS receiver clock error parameters.
///
///@{

struct ClockCorrectionUncertainty
{
    uint8_t receiver_id = 0; ///< 1, 2, etc.
    float bias_uncertainty = 0; ///< [seconds]
    float bias_drift_uncertainty = 0; ///< [seconds/second]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_CLOCK_CORRECTION_UNCERTAINTY;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "ClockCorrectionUncertainty";
    static constexpr const char* DOC_NAME = "ClockCorrectionUncertainty";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(receiver_id,bias_uncertainty,bias_drift_uncertainty,valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(receiver_id),std::ref(bias_uncertainty),std::ref(bias_drift_uncertainty),std::ref(valid_flags));
    }
};
void insert(Serializer& serializer, const ClockCorrectionUncertainty& self);
void extract(Serializer& serializer, ClockCorrectionUncertainty& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_gnss_pos_aid_status  (0x82,0x43) Gnss Pos Aid Status [CPP]
/// Filter reported GNSS position aiding status
///
///@{

struct GnssPosAidStatus
{
    uint8_t receiver_id = 0;
    float time_of_week = 0; ///< Last GNSS aiding measurement time of week [seconds]
    GnssAidStatusFlags status; ///< Aiding measurement status bitfield
    uint8_t reserved[8] = {0};
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_GNSS_POS_AID_STATUS;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "GnssPosAidStatus";
    static constexpr const char* DOC_NAME = "GNSS Position Aiding Status";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(receiver_id,time_of_week,status,reserved);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(receiver_id),std::ref(time_of_week),std::ref(status),std::ref(reserved));
    }
};
void insert(Serializer& serializer, const GnssPosAidStatus& self);
void extract(Serializer& serializer, GnssPosAidStatus& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_gnss_att_aid_status  (0x82,0x44) Gnss Att Aid Status [CPP]
/// Filter reported dual antenna GNSS attitude aiding status
///
///@{

struct GnssAttAidStatus
{
    float time_of_week = 0; ///< Last valid aiding measurement time of week [seconds] [processed instead of measured?]
    GnssAidStatusFlags status; ///< Last valid aiding measurement status bitfield
    uint8_t reserved[8] = {0};
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_GNSS_ATT_AID_STATUS;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "GnssAttAidStatus";
    static constexpr const char* DOC_NAME = "GNSS Attitude Aiding Status";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(time_of_week,status,reserved);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(time_of_week),std::ref(status),std::ref(reserved));
    }
};
void insert(Serializer& serializer, const GnssAttAidStatus& self);
void extract(Serializer& serializer, GnssAttAidStatus& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_head_aid_status  (0x82,0x45) Head Aid Status [CPP]
/// Filter reported GNSS heading aiding status
///
///@{

struct HeadAidStatus
{
    enum class HeadingAidType : uint8_t
    {
        DUAL_ANTENNA     = 1,  ///<  
        EXTERNAL_MESSAGE = 2,  ///<  
    };
    
    float time_of_week = 0; ///< Last valid aiding measurement time of week [seconds] [processed instead of measured?]
    HeadingAidType type = static_cast<HeadingAidType>(0); ///< 1 - Dual antenna, 2 - External heading message (user supplied)
    float reserved[2] = {0};
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_HEAD_AID_STATUS;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "HeadAidStatus";
    static constexpr const char* DOC_NAME = "HeadAidStatus";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(time_of_week,type,reserved);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(time_of_week),std::ref(type),std::ref(reserved));
    }
};
void insert(Serializer& serializer, const HeadAidStatus& self);
void extract(Serializer& serializer, HeadAidStatus& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_rel_pos_ned  (0x82,0x42) Rel Pos Ned [CPP]
/// Filter reported relative position, with respect to configured reference position
///
///@{

struct RelPosNed
{
    Vector3d relative_position; ///< [meters, NED]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_REL_POS_NED;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "RelPosNed";
    static constexpr const char* DOC_NAME = "NED Relative Position";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(relative_position[0],relative_position[1],relative_position[2],valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(relative_position[0]),std::ref(relative_position[1]),std::ref(relative_position[2]),std::ref(valid_flags));
    }
};
void insert(Serializer& serializer, const RelPosNed& self);
void extract(Serializer& serializer, RelPosNed& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_ecef_pos  (0x82,0x40) Ecef Pos [CPP]
/// Filter reported ECEF position
///
///@{

struct EcefPos
{
    Vector3d position_ecef; ///< [meters, ECEF]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 valid
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_ECEF_POS;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "EcefPos";
    static constexpr const char* DOC_NAME = "ECEF Position";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(position_ecef[0],position_ecef[1],position_ecef[2],valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(position_ecef[0]),std::ref(position_ecef[1]),std::ref(position_ecef[2]),std::ref(valid_flags));
    }
};
void insert(Serializer& serializer, const EcefPos& self);
void extract(Serializer& serializer, EcefPos& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_ecef_vel  (0x82,0x41) Ecef Vel [CPP]
/// Filter reported ECEF velocity
///
///@{

struct EcefVel
{
    Vector3f velocity_ecef; ///< [meters/second, ECEF]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 valid
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_ECEF_VEL;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "EcefVel";
    static constexpr const char* DOC_NAME = "ECEF Velocity";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(velocity_ecef[0],velocity_ecef[1],velocity_ecef[2],valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(velocity_ecef[0]),std::ref(velocity_ecef[1]),std::ref(velocity_ecef[2]),std::ref(valid_flags));
    }
};
void insert(Serializer& serializer, const EcefVel& self);
void extract(Serializer& serializer, EcefVel& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_ecef_pos_uncertainty  (0x82,0x36) Ecef Pos Uncertainty [CPP]
/// Filter reported 1-sigma position uncertainty in the ECEF frame.
///
///@{

struct EcefPosUncertainty
{
    Vector3f pos_uncertainty; ///< [meters]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_ECEF_POS_UNCERTAINTY;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "EcefPosUncertainty";
    static constexpr const char* DOC_NAME = "ECEF Position Uncertainty";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(pos_uncertainty[0],pos_uncertainty[1],pos_uncertainty[2],valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(pos_uncertainty[0]),std::ref(pos_uncertainty[1]),std::ref(pos_uncertainty[2]),std::ref(valid_flags));
    }
};
void insert(Serializer& serializer, const EcefPosUncertainty& self);
void extract(Serializer& serializer, EcefPosUncertainty& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_ecef_vel_uncertainty  (0x82,0x37) Ecef Vel Uncertainty [CPP]
/// Filter reported 1-sigma velocity uncertainties in the ECEF frame.
///
///@{

struct EcefVelUncertainty
{
    Vector3f vel_uncertainty; ///< [meters/second]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_ECEF_VEL_UNCERTAINTY;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "EcefVelUncertainty";
    static constexpr const char* DOC_NAME = "ECEF Velocity Uncertainty";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(vel_uncertainty[0],vel_uncertainty[1],vel_uncertainty[2],valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(vel_uncertainty[0]),std::ref(vel_uncertainty[1]),std::ref(vel_uncertainty[2]),std::ref(valid_flags));
    }
};
void insert(Serializer& serializer, const EcefVelUncertainty& self);
void extract(Serializer& serializer, EcefVelUncertainty& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_aiding_measurement_summary  (0x82,0x46) Aiding Measurement Summary [CPP]
/// Filter reported aiding measurement summary. This message contains a summary of the specified aiding measurement over the previous measurement interval ending at the specified time.
///
///@{

struct AidingMeasurementSummary
{
    float time_of_week = 0; ///< [seconds]
    uint8_t source = 0;
    FilterAidingMeasurementType type = static_cast<FilterAidingMeasurementType>(0); ///< (see product manual for supported types) Note: values 0x20 and above correspond to commanded aiding measurements in the 0x13 Aiding command set.
    FilterMeasurementIndicator indicator;
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_AID_MEAS_SUMMARY;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "AidingMeasurementSummary";
    static constexpr const char* DOC_NAME = "AidingMeasurementSummary";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(time_of_week,source,type,indicator);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(time_of_week),std::ref(source),std::ref(type),std::ref(indicator));
    }
};
void insert(Serializer& serializer, const AidingMeasurementSummary& self);
void extract(Serializer& serializer, AidingMeasurementSummary& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_odometer_scale_factor_error  (0x82,0x47) Odometer Scale Factor Error [CPP]
/// Filter reported odometer scale factor error. The total scale factor estimate is the user indicated scale factor, plus the user indicated scale factor times the scale factor error.
///
///@{

struct OdometerScaleFactorError
{
    float scale_factor_error = 0; ///< [dimensionless]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_ODOMETER_SCALE_FACTOR_ERROR;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "OdometerScaleFactorError";
    static constexpr const char* DOC_NAME = "Odometer Scale Factor Error";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(scale_factor_error,valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(scale_factor_error),std::ref(valid_flags));
    }
};
void insert(Serializer& serializer, const OdometerScaleFactorError& self);
void extract(Serializer& serializer, OdometerScaleFactorError& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_odometer_scale_factor_error_uncertainty  (0x82,0x48) Odometer Scale Factor Error Uncertainty [CPP]
/// Filter reported odometer scale factor error uncertainty.
///
///@{

struct OdometerScaleFactorErrorUncertainty
{
    float scale_factor_error_uncertainty = 0; ///< [dimensionless]
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_ODOMETER_SCALE_FACTOR_ERROR_UNCERTAINTY;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "OdometerScaleFactorErrorUncertainty";
    static constexpr const char* DOC_NAME = "Odometer Scale Factor Error Uncertainty";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(scale_factor_error_uncertainty,valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(scale_factor_error_uncertainty),std::ref(valid_flags));
    }
};
void insert(Serializer& serializer, const OdometerScaleFactorErrorUncertainty& self);
void extract(Serializer& serializer, OdometerScaleFactorErrorUncertainty& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_gnss_dual_antenna_status  (0x82,0x49) Gnss Dual Antenna Status [CPP]
/// Summary information for status of GNSS dual antenna heading estimate.
///
///@{

struct GnssDualAntennaStatus
{
    enum class FixType : uint8_t
    {
        FIX_NONE     = 0,  ///<  
        FIX_DA_FLOAT = 1,  ///<  
        FIX_DA_FIXED = 2,  ///<  
    };
    
    struct DualAntennaStatusFlags : Bitfield<DualAntennaStatusFlags>
    {
        enum _enumType : uint16_t
        {
            NONE                  = 0x0000,
            RCV_1_DATA_VALID      = 0x0001,  ///<  
            RCV_2_DATA_VALID      = 0x0002,  ///<  
            ANTENNA_OFFSETS_VALID = 0x0004,  ///<  
            ALL                   = 0x0007,
        };
        uint16_t value = NONE;
        
        DualAntennaStatusFlags() : value(NONE) {}
        DualAntennaStatusFlags(int val) : value((uint16_t)val) {}
        operator uint16_t() const { return value; }
        DualAntennaStatusFlags& operator=(uint16_t val) { value = val; return *this; }
        DualAntennaStatusFlags& operator=(int val) { value = uint16_t(val); return *this; }
        DualAntennaStatusFlags& operator|=(uint16_t val) { return *this = value | val; }
        DualAntennaStatusFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        bool rcv1DataValid() const { return (value & RCV_1_DATA_VALID) > 0; }
        void rcv1DataValid(bool val) { if(val) value |= RCV_1_DATA_VALID; else value &= ~RCV_1_DATA_VALID; }
        bool rcv2DataValid() const { return (value & RCV_2_DATA_VALID) > 0; }
        void rcv2DataValid(bool val) { if(val) value |= RCV_2_DATA_VALID; else value &= ~RCV_2_DATA_VALID; }
        bool antennaOffsetsValid() const { return (value & ANTENNA_OFFSETS_VALID) > 0; }
        void antennaOffsetsValid(bool val) { if(val) value |= ANTENNA_OFFSETS_VALID; else value &= ~ANTENNA_OFFSETS_VALID; }
        
        bool allSet() const { return value == ALL; }
        void setAll() { value |= ALL; }
    };
    
    float time_of_week = 0; ///< Last dual-antenna GNSS aiding measurement time of week [seconds]
    float heading = 0; ///< [radians]
    float heading_unc = 0; ///< [radians]
    FixType fix_type = static_cast<FixType>(0); ///< Fix type indicator
    DualAntennaStatusFlags status_flags;
    uint16_t valid_flags = 0; ///< 0 - invalid, 1 - valid
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_GNSS_DUAL_ANTENNA_STATUS;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "GnssDualAntennaStatus";
    static constexpr const char* DOC_NAME = "GNSS Dual Antenna Status";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(time_of_week,heading,heading_unc,fix_type,status_flags,valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(time_of_week),std::ref(heading),std::ref(heading_unc),std::ref(fix_type),std::ref(status_flags),std::ref(valid_flags));
    }
};
void insert(Serializer& serializer, const GnssDualAntennaStatus& self);
void extract(Serializer& serializer, GnssDualAntennaStatus& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_aiding_frame_config_error  (0x82,0x50) Aiding Frame Config Error [CPP]
/// Filter reported aiding source frame configuration error
/// 
/// These estimates are used to compensate for small errors to the user-supplied aiding frame configurations (set with (0x13, 0x01) command ).
///
///@{

struct AidingFrameConfigError
{
    uint8_t frame_id = 0; ///< Frame ID for the receiver to which the antenna is attached
    Vector3f translation; ///< Translation config X, Y, and Z (m).
    Quatf attitude; ///< Attitude quaternion
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_FRAME_CONFIG_ERROR;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "AidingFrameConfigError";
    static constexpr const char* DOC_NAME = "Aiding Frame Configuration Error";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(frame_id,translation[0],translation[1],translation[2],attitude[0],attitude[1],attitude[2],attitude[3]);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(frame_id),std::ref(translation[0]),std::ref(translation[1]),std::ref(translation[2]),std::ref(attitude[0]),std::ref(attitude[1]),std::ref(attitude[2]),std::ref(attitude[3]));
    }
};
void insert(Serializer& serializer, const AidingFrameConfigError& self);
void extract(Serializer& serializer, AidingFrameConfigError& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_aiding_frame_config_error_uncertainty  (0x82,0x51) Aiding Frame Config Error Uncertainty [CPP]
/// Filter reported aiding source frame configuration error uncertainty
/// 
/// These estimates are used to compensate for small errors to the user-supplied aiding frame configurations (set with (0x13, 0x01) command ).
///
///@{

struct AidingFrameConfigErrorUncertainty
{
    uint8_t frame_id = 0; ///< Frame ID for the receiver to which the antenna is attached
    Vector3f translation_unc; ///< Translation uncertaint X, Y, and Z (m).
    Vector3f attitude_unc; ///< Attitude uncertainty, X, Y, and Z (radians).
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_filter::DATA_FRAME_CONFIG_ERROR_UNCERTAINTY;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "AidingFrameConfigErrorUncertainty";
    static constexpr const char* DOC_NAME = "Aiding Frame Configuration Error Uncertainty";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(frame_id,translation_unc[0],translation_unc[1],translation_unc[2],attitude_unc[0],attitude_unc[1],attitude_unc[2]);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(frame_id),std::ref(translation_unc[0]),std::ref(translation_unc[1]),std::ref(translation_unc[2]),std::ref(attitude_unc[0]),std::ref(attitude_unc[1]),std::ref(attitude_unc[2]));
    }
};
void insert(Serializer& serializer, const AidingFrameConfigErrorUncertainty& self);
void extract(Serializer& serializer, AidingFrameConfigErrorUncertainty& self);


///@}
///

///@}
///@}
///
////////////////////////////////////////////////////////////////////////////////
} // namespace data_filter
} // namespace mip


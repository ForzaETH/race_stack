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

namespace commands_filter {

////////////////////////////////////////////////////////////////////////////////
///@addtogroup MipCommands_cpp  MIP Commands [CPP]
///@{
///@defgroup filter_commands_cpp  Filter Commands [CPP]
///
///@{

////////////////////////////////////////////////////////////////////////////////
// Descriptors
////////////////////////////////////////////////////////////////////////////////

enum 
{
    DESCRIPTOR_SET                                            = 0x0D,
    
    CMD_RESET_FILTER                                          = 0x01,
    CMD_SET_INITIAL_ATTITUDE                                  = 0x02,
    CMD_SET_INITIAL_HEADING                                   = 0x03,
    CMD_SET_INITIAL_HEADING_FROM_MAG                          = 0x04,
    CMD_RUN                                                   = 0x05,
    CMD_SELECT_FILTER                                         = 0x0F,
    CMD_VEHICLE_DYNAMICS_MODE                                 = 0x10,
    CMD_SENSOR2VEHICLE_ROTATION_EULER                         = 0x11,
    CMD_SENSOR2VEHICLE_OFFSET                                 = 0x12,
    CMD_ANTENNA_OFFSET                                        = 0x13,
    CMD_ESTIMATION_CONTROL_FLAGS                              = 0x14,
    CMD_GNSS_SOURCE_CONTROL                                   = 0x15,
    CMD_EXTERNAL_GNSS_UPDATE                                  = 0x16,
    CMD_EXTERNAL_HEADING_UPDATE                               = 0x17,
    CMD_HEADING_UPDATE_CONTROL                                = 0x18,
    CMD_AUTOINIT_CONTROL                                      = 0x19,
    CMD_ACCEL_NOISE                                           = 0x1A,
    CMD_GYRO_NOISE                                            = 0x1B,
    CMD_ACCEL_BIAS_MODEL                                      = 0x1C,
    CMD_GYRO_BIAS_MODEL                                       = 0x1D,
    CMD_ZUPT_CONTROL                                          = 0x1E,
    CMD_EXTERNAL_HEADING_UPDATE_WITH_TIME                     = 0x1F,
    CMD_ANGULAR_ZUPT_CONTROL                                  = 0x20,
    CMD_TARE_ORIENTATION                                      = 0x21,
    CMD_COMMANDED_ZUPT                                        = 0x22,
    CMD_COMMANDED_ANGULAR_ZUPT                                = 0x23,
    CMD_AUTO_HEADING_UPDATE_CONTROL                           = 0x24,
    CMD_MAG_AUTO_CALIBRATION_CONTROL                          = 0x25,
    CMD_MAG_CAPTURE_AUTO_CALIBRATION                          = 0x27,
    CMD_GRAVITY_NOISE                                         = 0x28,
    CMD_PRESSURE_NOISE                                        = 0x29,
    CMD_GRAVITY_NOISE_MINIMUM                                 = 0x2A,
    CMD_HARD_IRON_OFFSET_NOISE                                = 0x2B,
    CMD_SOFT_IRON_MATRIX_NOISE                                = 0x2C,
    CMD_LOW_PASS_SENSOR_FILTER                                = 0x30,
    CMD_MAG_NOISE                                             = 0x42,
    CMD_DECLINATION_SOURCE                                    = 0x43,
    CMD_HOT_START_CONTROL                                     = 0x48,
    CMD_SECONDARY_VELOCITY_AIDING_CONTROL                     = 0x4A,
    CMD_INCLINATION_SOURCE                                    = 0x4C,
    CMD_MAGNETIC_MAGNITUDE_SOURCE                             = 0x4D,
    CMD_SENSOR2VEHICLE_ROTATION_DCM                           = 0x4E,
    CMD_SENSOR2VEHICLE_ROTATION_QUATERNION                    = 0x4F,
    CMD_REFERENCE_POSITION                                    = 0x26,
    CMD_ENABLE_MEASUREMENT                                    = 0x41,
    CMD_ACCEL_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL    = 0x44,
    CMD_MAG_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL      = 0x45,
    CMD_MAG_DIP_ANGLE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL      = 0x46,
    CMD_ALTITUDE_AIDING_CONTROL                               = 0x47,
    CMD_SECONDARY_PITCH_ROLL_AIDING_CONTROL                   = 0x4B,
    CMD_AIDING_MEASUREMENT_ENABLE                             = 0x50,
    CMD_KINEMATIC_CONSTRAINT                                  = 0x51,
    CMD_INITIALIZATION_CONFIGURATION                          = 0x52,
    CMD_ADAPTIVE_FILTER_OPTIONS                               = 0x53,
    CMD_MULTI_ANTENNA_OFFSET                                  = 0x54,
    CMD_REL_POS_CONFIGURATION                                 = 0x55,
    CMD_REF_POINT_LEVER_ARM                                   = 0x56,
    CMD_SPEED_MEASUREMENT                                     = 0x60,
    CMD_SPEED_LEVER_ARM                                       = 0x61,
    CMD_GYRO_CONSTRAINT_CONTROL                               = 0x62,
    CMD_VEHICLE_CONSTRAINT_CONTROL                            = 0x63,
    CMD_ANTENNA_CALIBRATION_CONTROL                           = 0x64,
    CMD_TO_VEHICLE_CALIBRATION_CONTROL                        = 0x65,
    
    REPLY_VEHICLE_DYNAMICS_MODE                               = 0x80,
    REPLY_SENSOR2VEHICLE_ROTATION_EULER                       = 0x81,
    REPLY_SENSOR2VEHICLE_OFFSET                               = 0x82,
    REPLY_ANTENNA_OFFSET                                      = 0x83,
    REPLY_ESTIMATION_CONTROL_FLAGS                            = 0x84,
    REPLY_GNSS_SOURCE_CONTROL                                 = 0x86,
    REPLY_HEADING_UPDATE_CONTROL                              = 0x87,
    REPLY_AUTOINIT_CONTROL                                    = 0x88,
    REPLY_ACCEL_NOISE                                         = 0x89,
    REPLY_GYRO_NOISE                                          = 0x8A,
    REPLY_MAG_NOISE                                           = 0xB1,
    REPLY_ACCEL_BIAS_MODEL                                    = 0x8B,
    REPLY_GYRO_BIAS_MODEL                                     = 0x8C,
    REPLY_ZUPT_CONTROL                                        = 0x8D,
    REPLY_ANGULAR_ZUPT_CONTROL                                = 0x8E,
    REPLY_SELECT_FILTER                                       = 0x8F,
    REPLY_GRAVITY_NOISE                                       = 0x93,
    REPLY_PRESSURE_NOISE                                      = 0x94,
    REPLY_GRAVITY_NOISE_MINIMUM                               = 0x95,
    REPLY_HARD_IRON_OFFSET_NOISE                              = 0x96,
    REPLY_SOFT_IRON_MATRIX_NOISE                              = 0x97,
    REPLY_LOW_PASS_SENSOR_FILTER                              = 0xA0,
    REPLY_SET_INITIAL_HEADING                                 = 0x98,
    REPLY_REFERENCE_POSITION                                  = 0x90,
    REPLY_AUTO_HEADING_UPDATE_CONTROL                         = 0x91,
    REPLY_MAG_AUTO_CALIBRATION_CONTROL                        = 0x92,
    REPLY_ENABLE_MEASUREMENT                                  = 0xB0,
    REPLY_DECLINATION_SOURCE                                  = 0xB2,
    REPLY_ACCEL_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL  = 0xB3,
    REPLY_MAG_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL    = 0xB4,
    REPLY_MAG_DIP_ANGLE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL    = 0xB5,
    REPLY_MAG_ANGULAR_RATE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL = 0xB6,
    REPLY_ALTITUDE_AIDING_CONTROL                             = 0xB7,
    REPLY_HOT_START_CONTROL                                   = 0xB8,
    REPLY_SECONDARY_VELOCITY_AIDING_CONTROL                   = 0xBA,
    REPLY_SECONDARY_PITCH_ROLL_AIDING_CONTROL                 = 0xBB,
    REPLY_INCLINATION_SOURCE                                  = 0xBC,
    REPLY_MAGNETIC_MAGNITUDE_SOURCE                           = 0xBD,
    REPLY_SENSOR2VEHICLE_ROTATION_DCM                         = 0xBE,
    REPLY_SENSOR2VEHICLE_ROTATION_QUATERNION                  = 0xBF,
    REPLY_AIDING_MEASUREMENT_ENABLE                           = 0xD0,
    REPLY_KINEMATIC_CONSTRAINT                                = 0xD1,
    REPLY_INITIALIZATION_CONFIGURATION                        = 0xD2,
    REPLY_ADAPTIVE_FILTER_OPTIONS                             = 0xD3,
    REPLY_MULTI_ANTENNA_OFFSET                                = 0xD4,
    REPLY_REL_POS_CONFIGURATION                               = 0xD5,
    REPLY_SPEED_MEASUREMENT                                   = 0xE0,
    REPLY_GYRO_CONSTRAINT_CONTROL                             = 0xE2,
    REPLY_VEHICLE_CONSTRAINT_CONTROL                          = 0xE3,
    REPLY_ANTENNA_CALIBRATION_CONTROL                         = 0xE4,
    REPLY_TARE_ORIENTATION                                    = 0xA1,
    REPLY_REF_POINT_LEVER_ARM                                 = 0xD6,
    REPLY_SPEED_LEVER_ARM                                     = 0xE1,
};

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

enum class FilterReferenceFrame : uint8_t
{
    ECEF = 1,  ///<  WGS84 Earth-fixed, earth centered coordinates
    LLH  = 2,  ///<  WGS84 Latitude, longitude, and height above ellipsoid
};

enum class FilterMagParamSource : uint8_t
{
    NONE   = 1,  ///<  No source. See command documentation for default behavior
    WMM    = 2,  ///<  Magnetic field is assumed to conform to the World Magnetic Model, calculated using current location estimate as an input to the model.
    MANUAL = 3,  ///<  Magnetic field is assumed to have the parameter specified by the user.
};

enum class FilterAdaptiveMeasurement : uint8_t
{
    DISABLED = 0,  ///<  No adaptive measurement
    FIXED    = 1,  ///<  Enable fixed adaptive measurement (use specified limits)
    AUTO     = 2,  ///<  Enable auto adaptive measurement
};


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_reset  (0x0D,0x01) Reset [CPP]
/// Resets the filter to the initialization state.
/// 
/// If the auto-initialization feature is disabled, the initial attitude or heading must be set in
/// order to enter the run state after a reset.
///
///@{

struct Reset
{
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_RESET_FILTER;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "Reset";
    static constexpr const char* DOC_NAME = "Reset Navigation Filter";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple();
    }
    
    auto as_tuple()
    {
        return std::make_tuple();
    }
    typedef void Response;
};
void insert(Serializer& serializer, const Reset& self);
void extract(Serializer& serializer, Reset& self);

TypedResult<Reset> reset(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_set_initial_attitude  (0x0D,0x02) Set Initial Attitude [CPP]
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

struct SetInitialAttitude
{
    float roll = 0; ///< [radians]
    float pitch = 0; ///< [radians]
    float heading = 0; ///< [radians]
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_SET_INITIAL_ATTITUDE;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "SetInitialAttitude";
    static constexpr const char* DOC_NAME = "Set Initial Attitude";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(roll,pitch,heading);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(roll),std::ref(pitch),std::ref(heading));
    }
    typedef void Response;
};
void insert(Serializer& serializer, const SetInitialAttitude& self);
void extract(Serializer& serializer, SetInitialAttitude& self);

TypedResult<SetInitialAttitude> setInitialAttitude(C::mip_interface& device, float roll, float pitch, float heading);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_estimation_control  (0x0D,0x14) Estimation Control [CPP]
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

struct EstimationControl
{
    struct EnableFlags : Bitfield<EnableFlags>
    {
        enum _enumType : uint16_t
        {
            NONE               = 0x0000,
            GYRO_BIAS          = 0x0001,  ///<  
            ACCEL_BIAS         = 0x0002,  ///<  
            GYRO_SCALE_FACTOR  = 0x0004,  ///<  
            ACCEL_SCALE_FACTOR = 0x0008,  ///<  
            ANTENNA_OFFSET     = 0x0010,  ///<  
            AUTO_MAG_HARD_IRON = 0x0020,  ///<  
            AUTO_MAG_SOFT_IRON = 0x0040,  ///<  
            ALL                = 0x007F,
        };
        uint16_t value = NONE;
        
        EnableFlags() : value(NONE) {}
        EnableFlags(int val) : value((uint16_t)val) {}
        operator uint16_t() const { return value; }
        EnableFlags& operator=(uint16_t val) { value = val; return *this; }
        EnableFlags& operator=(int val) { value = uint16_t(val); return *this; }
        EnableFlags& operator|=(uint16_t val) { return *this = value | val; }
        EnableFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        bool gyroBias() const { return (value & GYRO_BIAS) > 0; }
        void gyroBias(bool val) { if(val) value |= GYRO_BIAS; else value &= ~GYRO_BIAS; }
        bool accelBias() const { return (value & ACCEL_BIAS) > 0; }
        void accelBias(bool val) { if(val) value |= ACCEL_BIAS; else value &= ~ACCEL_BIAS; }
        bool gyroScaleFactor() const { return (value & GYRO_SCALE_FACTOR) > 0; }
        void gyroScaleFactor(bool val) { if(val) value |= GYRO_SCALE_FACTOR; else value &= ~GYRO_SCALE_FACTOR; }
        bool accelScaleFactor() const { return (value & ACCEL_SCALE_FACTOR) > 0; }
        void accelScaleFactor(bool val) { if(val) value |= ACCEL_SCALE_FACTOR; else value &= ~ACCEL_SCALE_FACTOR; }
        bool antennaOffset() const { return (value & ANTENNA_OFFSET) > 0; }
        void antennaOffset(bool val) { if(val) value |= ANTENNA_OFFSET; else value &= ~ANTENNA_OFFSET; }
        bool autoMagHardIron() const { return (value & AUTO_MAG_HARD_IRON) > 0; }
        void autoMagHardIron(bool val) { if(val) value |= AUTO_MAG_HARD_IRON; else value &= ~AUTO_MAG_HARD_IRON; }
        bool autoMagSoftIron() const { return (value & AUTO_MAG_SOFT_IRON) > 0; }
        void autoMagSoftIron(bool val) { if(val) value |= AUTO_MAG_SOFT_IRON; else value &= ~AUTO_MAG_SOFT_IRON; }
        
        bool allSet() const { return value == ALL; }
        void setAll() { value |= ALL; }
    };
    
    FunctionSelector function = static_cast<FunctionSelector>(0);
    EnableFlags enable; ///< See above
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_ESTIMATION_CONTROL_FLAGS;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "EstimationControl";
    static constexpr const char* DOC_NAME = "Estimation Control Flags";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    static constexpr const uint32_t WRITE_PARAMS   = 0x8001;
    static constexpr const uint32_t READ_PARAMS    = 0x8000;
    static constexpr const uint32_t SAVE_PARAMS    = 0x8000;
    static constexpr const uint32_t LOAD_PARAMS    = 0x8000;
    static constexpr const uint32_t DEFAULT_PARAMS = 0x8000;
    static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(enable);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(enable));
    }
    
    static EstimationControl create_sld_all(::mip::FunctionSelector function)
    {
        EstimationControl cmd;
        cmd.function = function;
        return cmd;
    }
    
    struct Response
    {
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_ESTIMATION_CONTROL_FLAGS;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "EstimationControl::Response";
        static constexpr const char* DOC_NAME = "Estimation Control Flags Response";
        
        static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
        static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
        EnableFlags enable; ///< See above
        
        
        auto as_tuple()
        {
            return std::make_tuple(std::ref(enable));
        }
    };
};
void insert(Serializer& serializer, const EstimationControl& self);
void extract(Serializer& serializer, EstimationControl& self);

void insert(Serializer& serializer, const EstimationControl::Response& self);
void extract(Serializer& serializer, EstimationControl::Response& self);

TypedResult<EstimationControl> writeEstimationControl(C::mip_interface& device, EstimationControl::EnableFlags enable);
TypedResult<EstimationControl> readEstimationControl(C::mip_interface& device, EstimationControl::EnableFlags* enableOut);
TypedResult<EstimationControl> saveEstimationControl(C::mip_interface& device);
TypedResult<EstimationControl> loadEstimationControl(C::mip_interface& device);
TypedResult<EstimationControl> defaultEstimationControl(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_external_gnss_update  (0x0D,0x16) External Gnss Update [CPP]
/// Provide a filter measurement from an external GNSS
/// 
/// The GNSS source control must be set to "external" for this command to succeed, otherwise it will be NACK'd.
/// Please refer to your device user manual for information on the maximum rate of this message.
/// 
///
///@{

struct ExternalGnssUpdate
{
    double gps_time = 0; ///< [seconds]
    uint16_t gps_week = 0; ///< [GPS week number, not modulus 1024]
    double latitude = 0; ///< [degrees]
    double longitude = 0; ///< [degrees]
    double height = 0; ///< Above WGS84 ellipsoid [meters]
    Vector3f velocity; ///< NED Frame [meters/second]
    Vector3f pos_uncertainty; ///< NED Frame, 1-sigma [meters]
    Vector3f vel_uncertainty; ///< NED Frame, 1-sigma [meters/second]
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_EXTERNAL_GNSS_UPDATE;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "ExternalGnssUpdate";
    static constexpr const char* DOC_NAME = "External GNSS Update";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(gps_time,gps_week,latitude,longitude,height,velocity,pos_uncertainty,vel_uncertainty);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(gps_time),std::ref(gps_week),std::ref(latitude),std::ref(longitude),std::ref(height),std::ref(velocity),std::ref(pos_uncertainty),std::ref(vel_uncertainty));
    }
    typedef void Response;
};
void insert(Serializer& serializer, const ExternalGnssUpdate& self);
void extract(Serializer& serializer, ExternalGnssUpdate& self);

TypedResult<ExternalGnssUpdate> externalGnssUpdate(C::mip_interface& device, double gpsTime, uint16_t gpsWeek, double latitude, double longitude, double height, const float* velocity, const float* posUncertainty, const float* velUncertainty);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_external_heading_update  (0x0D,0x17) External Heading Update [CPP]
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

struct ExternalHeadingUpdate
{
    float heading = 0; ///< Bounded by +-PI [radians]
    float heading_uncertainty = 0; ///< 1-sigma [radians]
    uint8_t type = 0; ///< 1 - True, 2 - Magnetic
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_EXTERNAL_HEADING_UPDATE;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "ExternalHeadingUpdate";
    static constexpr const char* DOC_NAME = "External Heading Update";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(heading,heading_uncertainty,type);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(heading),std::ref(heading_uncertainty),std::ref(type));
    }
    typedef void Response;
};
void insert(Serializer& serializer, const ExternalHeadingUpdate& self);
void extract(Serializer& serializer, ExternalHeadingUpdate& self);

TypedResult<ExternalHeadingUpdate> externalHeadingUpdate(C::mip_interface& device, float heading, float headingUncertainty, uint8_t type);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_external_heading_update_with_time  (0x0D,0x1F) External Heading Update With Time [CPP]
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

struct ExternalHeadingUpdateWithTime
{
    double gps_time = 0; ///< [seconds]
    uint16_t gps_week = 0; ///< [GPS week number, not modulus 1024]
    float heading = 0; ///< Relative to true north, bounded by +-PI [radians]
    float heading_uncertainty = 0; ///< 1-sigma [radians]
    uint8_t type = 0; ///< 1 - True, 2 - Magnetic
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_EXTERNAL_HEADING_UPDATE_WITH_TIME;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "ExternalHeadingUpdateWithTime";
    static constexpr const char* DOC_NAME = "External Heading Update With Time";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(gps_time,gps_week,heading,heading_uncertainty,type);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(gps_time),std::ref(gps_week),std::ref(heading),std::ref(heading_uncertainty),std::ref(type));
    }
    typedef void Response;
};
void insert(Serializer& serializer, const ExternalHeadingUpdateWithTime& self);
void extract(Serializer& serializer, ExternalHeadingUpdateWithTime& self);

TypedResult<ExternalHeadingUpdateWithTime> externalHeadingUpdateWithTime(C::mip_interface& device, double gpsTime, uint16_t gpsWeek, float heading, float headingUncertainty, uint8_t type);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_tare_orientation  (0x0D,0x21) Tare Orientation [CPP]
/// Tare the device orientation.
/// 
/// This function uses the current device orientation relative to the NED frame as the current sensor to vehicle transformation.
/// This command is provided as a convenient way to set the sensor to vehicle frame transformation.
/// The filter must be initialized and have a valid attitude output. If the attitude is not valid, an error will be returned.
///
///@{

struct TareOrientation
{
    struct MipTareAxes : Bitfield<MipTareAxes>
    {
        enum _enumType : uint8_t
        {
            NONE  = 0x0,
            ROLL  = 0x1,  ///<  
            PITCH = 0x2,  ///<  
            YAW   = 0x4,  ///<  
            ALL   = 0x7,
        };
        uint8_t value = NONE;
        
        MipTareAxes() : value(NONE) {}
        MipTareAxes(int val) : value((uint8_t)val) {}
        operator uint8_t() const { return value; }
        MipTareAxes& operator=(uint8_t val) { value = val; return *this; }
        MipTareAxes& operator=(int val) { value = uint8_t(val); return *this; }
        MipTareAxes& operator|=(uint8_t val) { return *this = value | val; }
        MipTareAxes& operator&=(uint8_t val) { return *this = value & val; }
        
        bool roll() const { return (value & ROLL) > 0; }
        void roll(bool val) { if(val) value |= ROLL; else value &= ~ROLL; }
        bool pitch() const { return (value & PITCH) > 0; }
        void pitch(bool val) { if(val) value |= PITCH; else value &= ~PITCH; }
        bool yaw() const { return (value & YAW) > 0; }
        void yaw(bool val) { if(val) value |= YAW; else value &= ~YAW; }
        
        bool allSet() const { return value == ALL; }
        void setAll() { value |= ALL; }
    };
    
    FunctionSelector function = static_cast<FunctionSelector>(0);
    MipTareAxes axes; ///< Axes to tare
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_TARE_ORIENTATION;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "TareOrientation";
    static constexpr const char* DOC_NAME = "Tare Sensor Orientation";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    static constexpr const uint32_t WRITE_PARAMS   = 0x8001;
    static constexpr const uint32_t READ_PARAMS    = 0x8000;
    static constexpr const uint32_t SAVE_PARAMS    = 0x8000;
    static constexpr const uint32_t LOAD_PARAMS    = 0x8000;
    static constexpr const uint32_t DEFAULT_PARAMS = 0x8000;
    static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(axes);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(axes));
    }
    
    static TareOrientation create_sld_all(::mip::FunctionSelector function)
    {
        TareOrientation cmd;
        cmd.function = function;
        return cmd;
    }
    
    struct Response
    {
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_TARE_ORIENTATION;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "TareOrientation::Response";
        static constexpr const char* DOC_NAME = "Tare Sensor Orientation Response";
        
        static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
        static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
        MipTareAxes axes; ///< Axes to tare
        
        
        auto as_tuple()
        {
            return std::make_tuple(std::ref(axes));
        }
    };
};
void insert(Serializer& serializer, const TareOrientation& self);
void extract(Serializer& serializer, TareOrientation& self);

void insert(Serializer& serializer, const TareOrientation::Response& self);
void extract(Serializer& serializer, TareOrientation::Response& self);

TypedResult<TareOrientation> writeTareOrientation(C::mip_interface& device, TareOrientation::MipTareAxes axes);
TypedResult<TareOrientation> readTareOrientation(C::mip_interface& device, TareOrientation::MipTareAxes* axesOut);
TypedResult<TareOrientation> saveTareOrientation(C::mip_interface& device);
TypedResult<TareOrientation> loadTareOrientation(C::mip_interface& device);
TypedResult<TareOrientation> defaultTareOrientation(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_vehicle_dynamics_mode  (0x0D,0x10) Vehicle Dynamics Mode [CPP]
/// Controls the vehicle dynamics mode.
///
///@{

struct VehicleDynamicsMode
{
    enum class DynamicsMode : uint8_t
    {
        PORTABLE        = 1,  ///<  
        AUTOMOTIVE      = 2,  ///<  
        AIRBORNE        = 3,  ///<  
        AIRBORNE_HIGH_G = 4,  ///<  
    };
    
    FunctionSelector function = static_cast<FunctionSelector>(0);
    DynamicsMode mode = static_cast<DynamicsMode>(0);
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_VEHICLE_DYNAMICS_MODE;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "VehicleDynamicsMode";
    static constexpr const char* DOC_NAME = "Vehicle Dynamics Mode";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    static constexpr const uint32_t WRITE_PARAMS   = 0x8001;
    static constexpr const uint32_t READ_PARAMS    = 0x8000;
    static constexpr const uint32_t SAVE_PARAMS    = 0x8000;
    static constexpr const uint32_t LOAD_PARAMS    = 0x8000;
    static constexpr const uint32_t DEFAULT_PARAMS = 0x8000;
    static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(mode);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(mode));
    }
    
    static VehicleDynamicsMode create_sld_all(::mip::FunctionSelector function)
    {
        VehicleDynamicsMode cmd;
        cmd.function = function;
        return cmd;
    }
    
    struct Response
    {
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_VEHICLE_DYNAMICS_MODE;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "VehicleDynamicsMode::Response";
        static constexpr const char* DOC_NAME = "Vehicle Dynamics Mode Response";
        
        static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
        static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
        DynamicsMode mode = static_cast<DynamicsMode>(0);
        
        
        auto as_tuple()
        {
            return std::make_tuple(std::ref(mode));
        }
    };
};
void insert(Serializer& serializer, const VehicleDynamicsMode& self);
void extract(Serializer& serializer, VehicleDynamicsMode& self);

void insert(Serializer& serializer, const VehicleDynamicsMode::Response& self);
void extract(Serializer& serializer, VehicleDynamicsMode::Response& self);

TypedResult<VehicleDynamicsMode> writeVehicleDynamicsMode(C::mip_interface& device, VehicleDynamicsMode::DynamicsMode mode);
TypedResult<VehicleDynamicsMode> readVehicleDynamicsMode(C::mip_interface& device, VehicleDynamicsMode::DynamicsMode* modeOut);
TypedResult<VehicleDynamicsMode> saveVehicleDynamicsMode(C::mip_interface& device);
TypedResult<VehicleDynamicsMode> loadVehicleDynamicsMode(C::mip_interface& device);
TypedResult<VehicleDynamicsMode> defaultVehicleDynamicsMode(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_sensor_to_vehicle_rotation_euler  (0x0D,0x11) Sensor To Vehicle Rotation Euler [CPP]
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

struct SensorToVehicleRotationEuler
{
    FunctionSelector function = static_cast<FunctionSelector>(0);
    float roll = 0; ///< [radians]
    float pitch = 0; ///< [radians]
    float yaw = 0; ///< [radians]
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_SENSOR2VEHICLE_ROTATION_EULER;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "SensorToVehicleRotationEuler";
    static constexpr const char* DOC_NAME = "Sensor to Vehicle Frame Rotation Euler";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    static constexpr const uint32_t WRITE_PARAMS   = 0x8007;
    static constexpr const uint32_t READ_PARAMS    = 0x8000;
    static constexpr const uint32_t SAVE_PARAMS    = 0x8000;
    static constexpr const uint32_t LOAD_PARAMS    = 0x8000;
    static constexpr const uint32_t DEFAULT_PARAMS = 0x8000;
    static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(roll,pitch,yaw);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(roll),std::ref(pitch),std::ref(yaw));
    }
    
    static SensorToVehicleRotationEuler create_sld_all(::mip::FunctionSelector function)
    {
        SensorToVehicleRotationEuler cmd;
        cmd.function = function;
        return cmd;
    }
    
    struct Response
    {
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_SENSOR2VEHICLE_ROTATION_EULER;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "SensorToVehicleRotationEuler::Response";
        static constexpr const char* DOC_NAME = "Sensor to Vehicle Frame Rotation Euler Response";
        
        static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
        static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
        float roll = 0; ///< [radians]
        float pitch = 0; ///< [radians]
        float yaw = 0; ///< [radians]
        
        
        auto as_tuple()
        {
            return std::make_tuple(std::ref(roll),std::ref(pitch),std::ref(yaw));
        }
    };
};
void insert(Serializer& serializer, const SensorToVehicleRotationEuler& self);
void extract(Serializer& serializer, SensorToVehicleRotationEuler& self);

void insert(Serializer& serializer, const SensorToVehicleRotationEuler::Response& self);
void extract(Serializer& serializer, SensorToVehicleRotationEuler::Response& self);

TypedResult<SensorToVehicleRotationEuler> writeSensorToVehicleRotationEuler(C::mip_interface& device, float roll, float pitch, float yaw);
TypedResult<SensorToVehicleRotationEuler> readSensorToVehicleRotationEuler(C::mip_interface& device, float* rollOut, float* pitchOut, float* yawOut);
TypedResult<SensorToVehicleRotationEuler> saveSensorToVehicleRotationEuler(C::mip_interface& device);
TypedResult<SensorToVehicleRotationEuler> loadSensorToVehicleRotationEuler(C::mip_interface& device);
TypedResult<SensorToVehicleRotationEuler> defaultSensorToVehicleRotationEuler(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_sensor_to_vehicle_rotation_dcm  (0x0D,0x4E) Sensor To Vehicle Rotation Dcm [CPP]
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

struct SensorToVehicleRotationDcm
{
    FunctionSelector function = static_cast<FunctionSelector>(0);
    Matrix3f dcm;
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_SENSOR2VEHICLE_ROTATION_DCM;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "SensorToVehicleRotationDcm";
    static constexpr const char* DOC_NAME = "Sensor to Vehicle Frame Rotation DCM";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    static constexpr const uint32_t WRITE_PARAMS   = 0x8001;
    static constexpr const uint32_t READ_PARAMS    = 0x8000;
    static constexpr const uint32_t SAVE_PARAMS    = 0x8000;
    static constexpr const uint32_t LOAD_PARAMS    = 0x8000;
    static constexpr const uint32_t DEFAULT_PARAMS = 0x8000;
    static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(dcm);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(dcm));
    }
    
    static SensorToVehicleRotationDcm create_sld_all(::mip::FunctionSelector function)
    {
        SensorToVehicleRotationDcm cmd;
        cmd.function = function;
        return cmd;
    }
    
    struct Response
    {
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_SENSOR2VEHICLE_ROTATION_DCM;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "SensorToVehicleRotationDcm::Response";
        static constexpr const char* DOC_NAME = "Sensor to Vehicle Frame Rotation DCM Response";
        
        static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
        static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
        Matrix3f dcm;
        
        
        auto as_tuple()
        {
            return std::make_tuple(std::ref(dcm));
        }
    };
};
void insert(Serializer& serializer, const SensorToVehicleRotationDcm& self);
void extract(Serializer& serializer, SensorToVehicleRotationDcm& self);

void insert(Serializer& serializer, const SensorToVehicleRotationDcm::Response& self);
void extract(Serializer& serializer, SensorToVehicleRotationDcm::Response& self);

TypedResult<SensorToVehicleRotationDcm> writeSensorToVehicleRotationDcm(C::mip_interface& device, const float* dcm);
TypedResult<SensorToVehicleRotationDcm> readSensorToVehicleRotationDcm(C::mip_interface& device, float* dcmOut);
TypedResult<SensorToVehicleRotationDcm> saveSensorToVehicleRotationDcm(C::mip_interface& device);
TypedResult<SensorToVehicleRotationDcm> loadSensorToVehicleRotationDcm(C::mip_interface& device);
TypedResult<SensorToVehicleRotationDcm> defaultSensorToVehicleRotationDcm(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_sensor_to_vehicle_rotation_quaternion  (0x0D,0x4F) Sensor To Vehicle Rotation Quaternion [CPP]
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

struct SensorToVehicleRotationQuaternion
{
    FunctionSelector function = static_cast<FunctionSelector>(0);
    Quatf quat;
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_SENSOR2VEHICLE_ROTATION_QUATERNION;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "SensorToVehicleRotationQuaternion";
    static constexpr const char* DOC_NAME = "Sensor to Vehicle Frame Rotation Quaternion";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    static constexpr const uint32_t WRITE_PARAMS   = 0x8001;
    static constexpr const uint32_t READ_PARAMS    = 0x8000;
    static constexpr const uint32_t SAVE_PARAMS    = 0x8000;
    static constexpr const uint32_t LOAD_PARAMS    = 0x8000;
    static constexpr const uint32_t DEFAULT_PARAMS = 0x8000;
    static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(quat);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(quat));
    }
    
    static SensorToVehicleRotationQuaternion create_sld_all(::mip::FunctionSelector function)
    {
        SensorToVehicleRotationQuaternion cmd;
        cmd.function = function;
        return cmd;
    }
    
    struct Response
    {
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_SENSOR2VEHICLE_ROTATION_QUATERNION;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "SensorToVehicleRotationQuaternion::Response";
        static constexpr const char* DOC_NAME = "Sensor to Vehicle Frame Rotation Quaternion Response";
        
        static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
        static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
        Quatf quat;
        
        
        auto as_tuple()
        {
            return std::make_tuple(std::ref(quat));
        }
    };
};
void insert(Serializer& serializer, const SensorToVehicleRotationQuaternion& self);
void extract(Serializer& serializer, SensorToVehicleRotationQuaternion& self);

void insert(Serializer& serializer, const SensorToVehicleRotationQuaternion::Response& self);
void extract(Serializer& serializer, SensorToVehicleRotationQuaternion::Response& self);

TypedResult<SensorToVehicleRotationQuaternion> writeSensorToVehicleRotationQuaternion(C::mip_interface& device, const float* quat);
TypedResult<SensorToVehicleRotationQuaternion> readSensorToVehicleRotationQuaternion(C::mip_interface& device, float* quatOut);
TypedResult<SensorToVehicleRotationQuaternion> saveSensorToVehicleRotationQuaternion(C::mip_interface& device);
TypedResult<SensorToVehicleRotationQuaternion> loadSensorToVehicleRotationQuaternion(C::mip_interface& device);
TypedResult<SensorToVehicleRotationQuaternion> defaultSensorToVehicleRotationQuaternion(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_sensor_to_vehicle_offset  (0x0D,0x12) Sensor To Vehicle Offset [CPP]
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

struct SensorToVehicleOffset
{
    FunctionSelector function = static_cast<FunctionSelector>(0);
    Vector3f offset; ///< [meters]
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_SENSOR2VEHICLE_OFFSET;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "SensorToVehicleOffset";
    static constexpr const char* DOC_NAME = "Sensor to Vehicle Frame Offset";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    static constexpr const uint32_t WRITE_PARAMS   = 0x8001;
    static constexpr const uint32_t READ_PARAMS    = 0x8000;
    static constexpr const uint32_t SAVE_PARAMS    = 0x8000;
    static constexpr const uint32_t LOAD_PARAMS    = 0x8000;
    static constexpr const uint32_t DEFAULT_PARAMS = 0x8000;
    static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(offset);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(offset));
    }
    
    static SensorToVehicleOffset create_sld_all(::mip::FunctionSelector function)
    {
        SensorToVehicleOffset cmd;
        cmd.function = function;
        return cmd;
    }
    
    struct Response
    {
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_SENSOR2VEHICLE_OFFSET;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "SensorToVehicleOffset::Response";
        static constexpr const char* DOC_NAME = "Sensor to Vehicle Frame Offset Response";
        
        static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
        static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
        Vector3f offset; ///< [meters]
        
        
        auto as_tuple()
        {
            return std::make_tuple(std::ref(offset));
        }
    };
};
void insert(Serializer& serializer, const SensorToVehicleOffset& self);
void extract(Serializer& serializer, SensorToVehicleOffset& self);

void insert(Serializer& serializer, const SensorToVehicleOffset::Response& self);
void extract(Serializer& serializer, SensorToVehicleOffset::Response& self);

TypedResult<SensorToVehicleOffset> writeSensorToVehicleOffset(C::mip_interface& device, const float* offset);
TypedResult<SensorToVehicleOffset> readSensorToVehicleOffset(C::mip_interface& device, float* offsetOut);
TypedResult<SensorToVehicleOffset> saveSensorToVehicleOffset(C::mip_interface& device);
TypedResult<SensorToVehicleOffset> loadSensorToVehicleOffset(C::mip_interface& device);
TypedResult<SensorToVehicleOffset> defaultSensorToVehicleOffset(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_antenna_offset  (0x0D,0x13) Antenna Offset [CPP]
/// Set the sensor to GNSS antenna offset.
/// 
/// This is expressed in the sensor frame, from the sensor origin to the GNSS antenna RF center.
/// 
/// The magnitude of the offset vector is limited to 10 meters
/// 
///
///@{

struct AntennaOffset
{
    FunctionSelector function = static_cast<FunctionSelector>(0);
    Vector3f offset; ///< [meters]
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_ANTENNA_OFFSET;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "AntennaOffset";
    static constexpr const char* DOC_NAME = "GNSS Antenna Offset Control";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    static constexpr const uint32_t WRITE_PARAMS   = 0x8001;
    static constexpr const uint32_t READ_PARAMS    = 0x8000;
    static constexpr const uint32_t SAVE_PARAMS    = 0x8000;
    static constexpr const uint32_t LOAD_PARAMS    = 0x8000;
    static constexpr const uint32_t DEFAULT_PARAMS = 0x8000;
    static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(offset);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(offset));
    }
    
    static AntennaOffset create_sld_all(::mip::FunctionSelector function)
    {
        AntennaOffset cmd;
        cmd.function = function;
        return cmd;
    }
    
    struct Response
    {
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_ANTENNA_OFFSET;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "AntennaOffset::Response";
        static constexpr const char* DOC_NAME = "GNSS Antenna Offset Control Response";
        
        static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
        static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
        Vector3f offset; ///< [meters]
        
        
        auto as_tuple()
        {
            return std::make_tuple(std::ref(offset));
        }
    };
};
void insert(Serializer& serializer, const AntennaOffset& self);
void extract(Serializer& serializer, AntennaOffset& self);

void insert(Serializer& serializer, const AntennaOffset::Response& self);
void extract(Serializer& serializer, AntennaOffset::Response& self);

TypedResult<AntennaOffset> writeAntennaOffset(C::mip_interface& device, const float* offset);
TypedResult<AntennaOffset> readAntennaOffset(C::mip_interface& device, float* offsetOut);
TypedResult<AntennaOffset> saveAntennaOffset(C::mip_interface& device);
TypedResult<AntennaOffset> loadAntennaOffset(C::mip_interface& device);
TypedResult<AntennaOffset> defaultAntennaOffset(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_gnss_source  (0x0D,0x15) Gnss Source [CPP]
/// Control the source of GNSS information used to update the Kalman Filter.
/// 
/// Changing the GNSS source while the sensor is in the "running" state may temporarily place
/// it back in the "init" state until the new source of GNSS data is received.
/// 
///
///@{

struct GnssSource
{
    enum class Source : uint8_t
    {
        ALL_INT = 1,  ///<  All internal receivers
        EXT     = 2,  ///<  External GNSS messages provided by user
        INT_1   = 3,  ///<  Internal GNSS Receiver 1 only
        INT_2   = 4,  ///<  Internal GNSS Receiver 2 only
    };
    
    FunctionSelector function = static_cast<FunctionSelector>(0);
    Source source = static_cast<Source>(0);
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_GNSS_SOURCE_CONTROL;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "GnssSource";
    static constexpr const char* DOC_NAME = "GNSS Aiding Source Control";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    static constexpr const uint32_t WRITE_PARAMS   = 0x8001;
    static constexpr const uint32_t READ_PARAMS    = 0x8000;
    static constexpr const uint32_t SAVE_PARAMS    = 0x8000;
    static constexpr const uint32_t LOAD_PARAMS    = 0x8000;
    static constexpr const uint32_t DEFAULT_PARAMS = 0x8000;
    static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(source);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(source));
    }
    
    static GnssSource create_sld_all(::mip::FunctionSelector function)
    {
        GnssSource cmd;
        cmd.function = function;
        return cmd;
    }
    
    struct Response
    {
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_GNSS_SOURCE_CONTROL;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "GnssSource::Response";
        static constexpr const char* DOC_NAME = "GNSS Aiding Source Control Response";
        
        static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
        static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
        Source source = static_cast<Source>(0);
        
        
        auto as_tuple()
        {
            return std::make_tuple(std::ref(source));
        }
    };
};
void insert(Serializer& serializer, const GnssSource& self);
void extract(Serializer& serializer, GnssSource& self);

void insert(Serializer& serializer, const GnssSource::Response& self);
void extract(Serializer& serializer, GnssSource::Response& self);

TypedResult<GnssSource> writeGnssSource(C::mip_interface& device, GnssSource::Source source);
TypedResult<GnssSource> readGnssSource(C::mip_interface& device, GnssSource::Source* sourceOut);
TypedResult<GnssSource> saveGnssSource(C::mip_interface& device);
TypedResult<GnssSource> loadGnssSource(C::mip_interface& device);
TypedResult<GnssSource> defaultGnssSource(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_heading_source  (0x0D,0x18) Heading Source [CPP]
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

struct HeadingSource
{
    enum class Source : uint8_t
    {
        NONE                          = 0,  ///<  See note 3
        MAG                           = 1,  ///<  
        GNSS_VEL                      = 2,  ///<  See notes 1,2
        EXTERNAL                      = 3,  ///<  
        GNSS_VEL_AND_MAG              = 4,  ///<  
        GNSS_VEL_AND_EXTERNAL         = 5,  ///<  
        MAG_AND_EXTERNAL              = 6,  ///<  
        GNSS_VEL_AND_MAG_AND_EXTERNAL = 7,  ///<  
    };
    
    FunctionSelector function = static_cast<FunctionSelector>(0);
    Source source = static_cast<Source>(0);
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_HEADING_UPDATE_CONTROL;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "HeadingSource";
    static constexpr const char* DOC_NAME = "Heading Aiding Source Control";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    static constexpr const uint32_t WRITE_PARAMS   = 0x8001;
    static constexpr const uint32_t READ_PARAMS    = 0x8000;
    static constexpr const uint32_t SAVE_PARAMS    = 0x8000;
    static constexpr const uint32_t LOAD_PARAMS    = 0x8000;
    static constexpr const uint32_t DEFAULT_PARAMS = 0x8000;
    static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(source);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(source));
    }
    
    static HeadingSource create_sld_all(::mip::FunctionSelector function)
    {
        HeadingSource cmd;
        cmd.function = function;
        return cmd;
    }
    
    struct Response
    {
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_HEADING_UPDATE_CONTROL;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "HeadingSource::Response";
        static constexpr const char* DOC_NAME = "Heading Aiding Source Control Response";
        
        static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
        static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
        Source source = static_cast<Source>(0);
        
        
        auto as_tuple()
        {
            return std::make_tuple(std::ref(source));
        }
    };
};
void insert(Serializer& serializer, const HeadingSource& self);
void extract(Serializer& serializer, HeadingSource& self);

void insert(Serializer& serializer, const HeadingSource::Response& self);
void extract(Serializer& serializer, HeadingSource::Response& self);

TypedResult<HeadingSource> writeHeadingSource(C::mip_interface& device, HeadingSource::Source source);
TypedResult<HeadingSource> readHeadingSource(C::mip_interface& device, HeadingSource::Source* sourceOut);
TypedResult<HeadingSource> saveHeadingSource(C::mip_interface& device);
TypedResult<HeadingSource> loadHeadingSource(C::mip_interface& device);
TypedResult<HeadingSource> defaultHeadingSource(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_auto_init_control  (0x0D,0x19) Auto Init Control [CPP]
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

struct AutoInitControl
{
    FunctionSelector function = static_cast<FunctionSelector>(0);
    uint8_t enable = 0; ///< See above
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_AUTOINIT_CONTROL;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "AutoInitControl";
    static constexpr const char* DOC_NAME = "Auto-initialization Control";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    static constexpr const uint32_t WRITE_PARAMS   = 0x8001;
    static constexpr const uint32_t READ_PARAMS    = 0x8000;
    static constexpr const uint32_t SAVE_PARAMS    = 0x8000;
    static constexpr const uint32_t LOAD_PARAMS    = 0x8000;
    static constexpr const uint32_t DEFAULT_PARAMS = 0x8000;
    static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(enable);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(enable));
    }
    
    static AutoInitControl create_sld_all(::mip::FunctionSelector function)
    {
        AutoInitControl cmd;
        cmd.function = function;
        return cmd;
    }
    
    struct Response
    {
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_AUTOINIT_CONTROL;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "AutoInitControl::Response";
        static constexpr const char* DOC_NAME = "Auto-initialization Control Response";
        
        static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
        static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
        uint8_t enable = 0; ///< See above
        
        
        auto as_tuple()
        {
            return std::make_tuple(std::ref(enable));
        }
    };
};
void insert(Serializer& serializer, const AutoInitControl& self);
void extract(Serializer& serializer, AutoInitControl& self);

void insert(Serializer& serializer, const AutoInitControl::Response& self);
void extract(Serializer& serializer, AutoInitControl::Response& self);

TypedResult<AutoInitControl> writeAutoInitControl(C::mip_interface& device, uint8_t enable);
TypedResult<AutoInitControl> readAutoInitControl(C::mip_interface& device, uint8_t* enableOut);
TypedResult<AutoInitControl> saveAutoInitControl(C::mip_interface& device);
TypedResult<AutoInitControl> loadAutoInitControl(C::mip_interface& device);
TypedResult<AutoInitControl> defaultAutoInitControl(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_accel_noise  (0x0D,0x1A) Accel Noise [CPP]
/// Accelerometer Noise Standard Deviation
/// 
/// Each of the noise values must be greater than 0.0.
/// 
/// The noise value represents process noise in the Estimation Filter.
/// Changing this value modifies how the filter responds to dynamic input and can be used to tune the performance of the filter.
/// Default values provide good performance for most laboratory conditions.
///
///@{

struct AccelNoise
{
    FunctionSelector function = static_cast<FunctionSelector>(0);
    Vector3f noise; ///< Accel Noise 1-sigma [meters/second^2]
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_ACCEL_NOISE;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "AccelNoise";
    static constexpr const char* DOC_NAME = "Accelerometer Noise Standard Deviation";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    static constexpr const uint32_t WRITE_PARAMS   = 0x8001;
    static constexpr const uint32_t READ_PARAMS    = 0x8000;
    static constexpr const uint32_t SAVE_PARAMS    = 0x8000;
    static constexpr const uint32_t LOAD_PARAMS    = 0x8000;
    static constexpr const uint32_t DEFAULT_PARAMS = 0x8000;
    static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(noise);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(noise));
    }
    
    static AccelNoise create_sld_all(::mip::FunctionSelector function)
    {
        AccelNoise cmd;
        cmd.function = function;
        return cmd;
    }
    
    struct Response
    {
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_ACCEL_NOISE;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "AccelNoise::Response";
        static constexpr const char* DOC_NAME = "Accelerometer Noise Standard Deviation Response";
        
        static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
        static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
        Vector3f noise; ///< Accel Noise 1-sigma [meters/second^2]
        
        
        auto as_tuple()
        {
            return std::make_tuple(std::ref(noise));
        }
    };
};
void insert(Serializer& serializer, const AccelNoise& self);
void extract(Serializer& serializer, AccelNoise& self);

void insert(Serializer& serializer, const AccelNoise::Response& self);
void extract(Serializer& serializer, AccelNoise::Response& self);

TypedResult<AccelNoise> writeAccelNoise(C::mip_interface& device, const float* noise);
TypedResult<AccelNoise> readAccelNoise(C::mip_interface& device, float* noiseOut);
TypedResult<AccelNoise> saveAccelNoise(C::mip_interface& device);
TypedResult<AccelNoise> loadAccelNoise(C::mip_interface& device);
TypedResult<AccelNoise> defaultAccelNoise(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_gyro_noise  (0x0D,0x1B) Gyro Noise [CPP]
/// Gyroscope Noise Standard Deviation
/// 
/// Each of the noise values must be greater than 0.0
/// 
/// The noise value represents process noise in the Estimation Filter.
/// Changing this value modifies how the filter responds to dynamic input and can be used to tune the performance of the filter.
/// Default values provide good performance for most laboratory conditions.
///
///@{

struct GyroNoise
{
    FunctionSelector function = static_cast<FunctionSelector>(0);
    Vector3f noise; ///< Gyro Noise 1-sigma [rad/second]
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_GYRO_NOISE;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "GyroNoise";
    static constexpr const char* DOC_NAME = "Gyroscope Noise Standard Deviation";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    static constexpr const uint32_t WRITE_PARAMS   = 0x8001;
    static constexpr const uint32_t READ_PARAMS    = 0x8000;
    static constexpr const uint32_t SAVE_PARAMS    = 0x8000;
    static constexpr const uint32_t LOAD_PARAMS    = 0x8000;
    static constexpr const uint32_t DEFAULT_PARAMS = 0x8000;
    static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(noise);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(noise));
    }
    
    static GyroNoise create_sld_all(::mip::FunctionSelector function)
    {
        GyroNoise cmd;
        cmd.function = function;
        return cmd;
    }
    
    struct Response
    {
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_GYRO_NOISE;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "GyroNoise::Response";
        static constexpr const char* DOC_NAME = "Gyroscope Noise Standard Deviation Response";
        
        static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
        static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
        Vector3f noise; ///< Gyro Noise 1-sigma [rad/second]
        
        
        auto as_tuple()
        {
            return std::make_tuple(std::ref(noise));
        }
    };
};
void insert(Serializer& serializer, const GyroNoise& self);
void extract(Serializer& serializer, GyroNoise& self);

void insert(Serializer& serializer, const GyroNoise::Response& self);
void extract(Serializer& serializer, GyroNoise::Response& self);

TypedResult<GyroNoise> writeGyroNoise(C::mip_interface& device, const float* noise);
TypedResult<GyroNoise> readGyroNoise(C::mip_interface& device, float* noiseOut);
TypedResult<GyroNoise> saveGyroNoise(C::mip_interface& device);
TypedResult<GyroNoise> loadGyroNoise(C::mip_interface& device);
TypedResult<GyroNoise> defaultGyroNoise(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_accel_bias_model  (0x0D,0x1C) Accel Bias Model [CPP]
/// Accelerometer Bias Model Parameters
/// 
/// Noise values must be greater than 0.0
/// 
///
///@{

struct AccelBiasModel
{
    FunctionSelector function = static_cast<FunctionSelector>(0);
    Vector3f beta; ///< Accel Bias Beta [1/second]
    Vector3f noise; ///< Accel Noise 1-sigma [meters/second^2]
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_ACCEL_BIAS_MODEL;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "AccelBiasModel";
    static constexpr const char* DOC_NAME = "Accelerometer Bias Model Parameters";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    static constexpr const uint32_t WRITE_PARAMS   = 0x8003;
    static constexpr const uint32_t READ_PARAMS    = 0x8000;
    static constexpr const uint32_t SAVE_PARAMS    = 0x8000;
    static constexpr const uint32_t LOAD_PARAMS    = 0x8000;
    static constexpr const uint32_t DEFAULT_PARAMS = 0x8000;
    static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(beta,noise);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(beta),std::ref(noise));
    }
    
    static AccelBiasModel create_sld_all(::mip::FunctionSelector function)
    {
        AccelBiasModel cmd;
        cmd.function = function;
        return cmd;
    }
    
    struct Response
    {
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_ACCEL_BIAS_MODEL;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "AccelBiasModel::Response";
        static constexpr const char* DOC_NAME = "Accelerometer Bias Model Parameters Response";
        
        static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
        static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
        Vector3f beta; ///< Accel Bias Beta [1/second]
        Vector3f noise; ///< Accel Noise 1-sigma [meters/second^2]
        
        
        auto as_tuple()
        {
            return std::make_tuple(std::ref(beta),std::ref(noise));
        }
    };
};
void insert(Serializer& serializer, const AccelBiasModel& self);
void extract(Serializer& serializer, AccelBiasModel& self);

void insert(Serializer& serializer, const AccelBiasModel::Response& self);
void extract(Serializer& serializer, AccelBiasModel::Response& self);

TypedResult<AccelBiasModel> writeAccelBiasModel(C::mip_interface& device, const float* beta, const float* noise);
TypedResult<AccelBiasModel> readAccelBiasModel(C::mip_interface& device, float* betaOut, float* noiseOut);
TypedResult<AccelBiasModel> saveAccelBiasModel(C::mip_interface& device);
TypedResult<AccelBiasModel> loadAccelBiasModel(C::mip_interface& device);
TypedResult<AccelBiasModel> defaultAccelBiasModel(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_gyro_bias_model  (0x0D,0x1D) Gyro Bias Model [CPP]
/// Gyroscope Bias Model Parameters
/// 
/// Noise values must be greater than 0.0
/// 
///
///@{

struct GyroBiasModel
{
    FunctionSelector function = static_cast<FunctionSelector>(0);
    Vector3f beta; ///< Gyro Bias Beta [1/second]
    Vector3f noise; ///< Gyro Noise 1-sigma [rad/second]
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_GYRO_BIAS_MODEL;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "GyroBiasModel";
    static constexpr const char* DOC_NAME = "Gyroscope Bias Model Parameters";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    static constexpr const uint32_t WRITE_PARAMS   = 0x8003;
    static constexpr const uint32_t READ_PARAMS    = 0x8000;
    static constexpr const uint32_t SAVE_PARAMS    = 0x8000;
    static constexpr const uint32_t LOAD_PARAMS    = 0x8000;
    static constexpr const uint32_t DEFAULT_PARAMS = 0x8000;
    static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(beta,noise);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(beta),std::ref(noise));
    }
    
    static GyroBiasModel create_sld_all(::mip::FunctionSelector function)
    {
        GyroBiasModel cmd;
        cmd.function = function;
        return cmd;
    }
    
    struct Response
    {
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_GYRO_BIAS_MODEL;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "GyroBiasModel::Response";
        static constexpr const char* DOC_NAME = "Gyroscope Bias Model Parameters Response";
        
        static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
        static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
        Vector3f beta; ///< Gyro Bias Beta [1/second]
        Vector3f noise; ///< Gyro Noise 1-sigma [rad/second]
        
        
        auto as_tuple()
        {
            return std::make_tuple(std::ref(beta),std::ref(noise));
        }
    };
};
void insert(Serializer& serializer, const GyroBiasModel& self);
void extract(Serializer& serializer, GyroBiasModel& self);

void insert(Serializer& serializer, const GyroBiasModel::Response& self);
void extract(Serializer& serializer, GyroBiasModel::Response& self);

TypedResult<GyroBiasModel> writeGyroBiasModel(C::mip_interface& device, const float* beta, const float* noise);
TypedResult<GyroBiasModel> readGyroBiasModel(C::mip_interface& device, float* betaOut, float* noiseOut);
TypedResult<GyroBiasModel> saveGyroBiasModel(C::mip_interface& device);
TypedResult<GyroBiasModel> loadGyroBiasModel(C::mip_interface& device);
TypedResult<GyroBiasModel> defaultGyroBiasModel(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_altitude_aiding  (0x0D,0x47) Altitude Aiding [CPP]
/// Select altitude input for absolute altitude and/or vertical velocity. The primary altitude reading is always GNSS.
/// Aiding inputs are used to improve GNSS altitude readings when GNSS is available and to backup GNSS during outages.
/// 
/// Pressure altitude is based on "instant sea level pressure" which is dependent on location and weather conditions and can vary by more than 40 meters.
/// 
///
///@{

struct AltitudeAiding
{
    enum class AidingSelector : uint8_t
    {
        NONE    = 0,  ///<  No altitude aiding
        PRESURE = 1,  ///<  Enable pressure sensor aiding
    };
    
    FunctionSelector function = static_cast<FunctionSelector>(0);
    AidingSelector selector = static_cast<AidingSelector>(0); ///< See above
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_ALTITUDE_AIDING_CONTROL;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "AltitudeAiding";
    static constexpr const char* DOC_NAME = "Altitude Aiding Control";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    static constexpr const uint32_t WRITE_PARAMS   = 0x8001;
    static constexpr const uint32_t READ_PARAMS    = 0x8000;
    static constexpr const uint32_t SAVE_PARAMS    = 0x8000;
    static constexpr const uint32_t LOAD_PARAMS    = 0x8000;
    static constexpr const uint32_t DEFAULT_PARAMS = 0x8000;
    static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(selector);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(selector));
    }
    
    static AltitudeAiding create_sld_all(::mip::FunctionSelector function)
    {
        AltitudeAiding cmd;
        cmd.function = function;
        return cmd;
    }
    
    struct Response
    {
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_ALTITUDE_AIDING_CONTROL;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "AltitudeAiding::Response";
        static constexpr const char* DOC_NAME = "Altitude Aiding Control Response";
        
        static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
        static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
        AidingSelector selector = static_cast<AidingSelector>(0); ///< See above
        
        
        auto as_tuple()
        {
            return std::make_tuple(std::ref(selector));
        }
    };
};
void insert(Serializer& serializer, const AltitudeAiding& self);
void extract(Serializer& serializer, AltitudeAiding& self);

void insert(Serializer& serializer, const AltitudeAiding::Response& self);
void extract(Serializer& serializer, AltitudeAiding::Response& self);

TypedResult<AltitudeAiding> writeAltitudeAiding(C::mip_interface& device, AltitudeAiding::AidingSelector selector);
TypedResult<AltitudeAiding> readAltitudeAiding(C::mip_interface& device, AltitudeAiding::AidingSelector* selectorOut);
TypedResult<AltitudeAiding> saveAltitudeAiding(C::mip_interface& device);
TypedResult<AltitudeAiding> loadAltitudeAiding(C::mip_interface& device);
TypedResult<AltitudeAiding> defaultAltitudeAiding(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_pitch_roll_aiding  (0x0D,0x4B) Pitch Roll Aiding [CPP]
/// Select pitch/roll aiding input. Pitch/roll reading is always derived from GNSS corrected inertial solution.
/// Aiding inputs are used to improve that solution during periods of low dynamics and GNSS outages.
///
///@{

struct PitchRollAiding
{
    enum class AidingSource : uint8_t
    {
        NONE        = 0,  ///<  No pitch/roll aiding
        GRAVITY_VEC = 1,  ///<  Enable gravity vector aiding
    };
    
    FunctionSelector function = static_cast<FunctionSelector>(0);
    AidingSource source = static_cast<AidingSource>(0); ///< Controls the aiding source
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_SECONDARY_PITCH_ROLL_AIDING_CONTROL;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "PitchRollAiding";
    static constexpr const char* DOC_NAME = "Pitch/Roll Aiding Control";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    static constexpr const uint32_t WRITE_PARAMS   = 0x8001;
    static constexpr const uint32_t READ_PARAMS    = 0x8000;
    static constexpr const uint32_t SAVE_PARAMS    = 0x8000;
    static constexpr const uint32_t LOAD_PARAMS    = 0x8000;
    static constexpr const uint32_t DEFAULT_PARAMS = 0x8000;
    static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(source);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(source));
    }
    
    static PitchRollAiding create_sld_all(::mip::FunctionSelector function)
    {
        PitchRollAiding cmd;
        cmd.function = function;
        return cmd;
    }
    
    struct Response
    {
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_SECONDARY_PITCH_ROLL_AIDING_CONTROL;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "PitchRollAiding::Response";
        static constexpr const char* DOC_NAME = "Pitch/Roll Aiding Control Response";
        
        static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
        static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
        AidingSource source = static_cast<AidingSource>(0); ///< Controls the aiding source
        
        
        auto as_tuple()
        {
            return std::make_tuple(std::ref(source));
        }
    };
};
void insert(Serializer& serializer, const PitchRollAiding& self);
void extract(Serializer& serializer, PitchRollAiding& self);

void insert(Serializer& serializer, const PitchRollAiding::Response& self);
void extract(Serializer& serializer, PitchRollAiding::Response& self);

TypedResult<PitchRollAiding> writePitchRollAiding(C::mip_interface& device, PitchRollAiding::AidingSource source);
TypedResult<PitchRollAiding> readPitchRollAiding(C::mip_interface& device, PitchRollAiding::AidingSource* sourceOut);
TypedResult<PitchRollAiding> savePitchRollAiding(C::mip_interface& device);
TypedResult<PitchRollAiding> loadPitchRollAiding(C::mip_interface& device);
TypedResult<PitchRollAiding> defaultPitchRollAiding(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_auto_zupt  (0x0D,0x1E) Auto Zupt [CPP]
/// The ZUPT is triggered when the scalar magnitude of the GNSS reported velocity vector is equal-to or less than the threshold value.
/// The device will NACK threshold values that are less than zero (i.e.negative.)
///
///@{

struct AutoZupt
{
    FunctionSelector function = static_cast<FunctionSelector>(0);
    uint8_t enable = 0; ///< 0 - Disable, 1 - Enable
    float threshold = 0; ///< [meters/second]
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_ZUPT_CONTROL;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "AutoZupt";
    static constexpr const char* DOC_NAME = "Zero Velocity Update Control";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    static constexpr const uint32_t WRITE_PARAMS   = 0x8003;
    static constexpr const uint32_t READ_PARAMS    = 0x8000;
    static constexpr const uint32_t SAVE_PARAMS    = 0x8000;
    static constexpr const uint32_t LOAD_PARAMS    = 0x8000;
    static constexpr const uint32_t DEFAULT_PARAMS = 0x8000;
    static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(enable,threshold);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(enable),std::ref(threshold));
    }
    
    static AutoZupt create_sld_all(::mip::FunctionSelector function)
    {
        AutoZupt cmd;
        cmd.function = function;
        return cmd;
    }
    
    struct Response
    {
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_ZUPT_CONTROL;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "AutoZupt::Response";
        static constexpr const char* DOC_NAME = "Zero Velocity Update Control Response";
        
        static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
        static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
        uint8_t enable = 0; ///< 0 - Disable, 1 - Enable
        float threshold = 0; ///< [meters/second]
        
        
        auto as_tuple()
        {
            return std::make_tuple(std::ref(enable),std::ref(threshold));
        }
    };
};
void insert(Serializer& serializer, const AutoZupt& self);
void extract(Serializer& serializer, AutoZupt& self);

void insert(Serializer& serializer, const AutoZupt::Response& self);
void extract(Serializer& serializer, AutoZupt::Response& self);

TypedResult<AutoZupt> writeAutoZupt(C::mip_interface& device, uint8_t enable, float threshold);
TypedResult<AutoZupt> readAutoZupt(C::mip_interface& device, uint8_t* enableOut, float* thresholdOut);
TypedResult<AutoZupt> saveAutoZupt(C::mip_interface& device);
TypedResult<AutoZupt> loadAutoZupt(C::mip_interface& device);
TypedResult<AutoZupt> defaultAutoZupt(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_auto_angular_zupt  (0x0D,0x20) Auto Angular Zupt [CPP]
/// Zero Angular Rate Update
/// The ZUPT is triggered when the scalar magnitude of the angular rate vector is equal-to or less than the threshold value.
/// The device will NACK threshold values that are less than zero (i.e.negative.)
///
///@{

struct AutoAngularZupt
{
    FunctionSelector function = static_cast<FunctionSelector>(0);
    uint8_t enable = 0; ///< 0 - Disable, 1 - Enable
    float threshold = 0; ///< [radians/second]
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_ANGULAR_ZUPT_CONTROL;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "AutoAngularZupt";
    static constexpr const char* DOC_NAME = "Zero Angular Rate Update Control";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    static constexpr const uint32_t WRITE_PARAMS   = 0x8003;
    static constexpr const uint32_t READ_PARAMS    = 0x8000;
    static constexpr const uint32_t SAVE_PARAMS    = 0x8000;
    static constexpr const uint32_t LOAD_PARAMS    = 0x8000;
    static constexpr const uint32_t DEFAULT_PARAMS = 0x8000;
    static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(enable,threshold);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(enable),std::ref(threshold));
    }
    
    static AutoAngularZupt create_sld_all(::mip::FunctionSelector function)
    {
        AutoAngularZupt cmd;
        cmd.function = function;
        return cmd;
    }
    
    struct Response
    {
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_ANGULAR_ZUPT_CONTROL;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "AutoAngularZupt::Response";
        static constexpr const char* DOC_NAME = "Zero Angular Rate Update Control Response";
        
        static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
        static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
        uint8_t enable = 0; ///< 0 - Disable, 1 - Enable
        float threshold = 0; ///< [radians/second]
        
        
        auto as_tuple()
        {
            return std::make_tuple(std::ref(enable),std::ref(threshold));
        }
    };
};
void insert(Serializer& serializer, const AutoAngularZupt& self);
void extract(Serializer& serializer, AutoAngularZupt& self);

void insert(Serializer& serializer, const AutoAngularZupt::Response& self);
void extract(Serializer& serializer, AutoAngularZupt::Response& self);

TypedResult<AutoAngularZupt> writeAutoAngularZupt(C::mip_interface& device, uint8_t enable, float threshold);
TypedResult<AutoAngularZupt> readAutoAngularZupt(C::mip_interface& device, uint8_t* enableOut, float* thresholdOut);
TypedResult<AutoAngularZupt> saveAutoAngularZupt(C::mip_interface& device);
TypedResult<AutoAngularZupt> loadAutoAngularZupt(C::mip_interface& device);
TypedResult<AutoAngularZupt> defaultAutoAngularZupt(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_commanded_zupt  (0x0D,0x22) Commanded Zupt [CPP]
/// Please see the device user manual for the maximum rate of this message.
///
///@{

struct CommandedZupt
{
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_COMMANDED_ZUPT;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "CommandedZupt";
    static constexpr const char* DOC_NAME = "Commanded Zero Velocity Update";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple();
    }
    
    auto as_tuple()
    {
        return std::make_tuple();
    }
    typedef void Response;
};
void insert(Serializer& serializer, const CommandedZupt& self);
void extract(Serializer& serializer, CommandedZupt& self);

TypedResult<CommandedZupt> commandedZupt(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_commanded_angular_zupt  (0x0D,0x23) Commanded Angular Zupt [CPP]
/// Please see the device user manual for the maximum rate of this message.
///
///@{

struct CommandedAngularZupt
{
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_COMMANDED_ANGULAR_ZUPT;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "CommandedAngularZupt";
    static constexpr const char* DOC_NAME = "Commanded Zero Angular Rate Update";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple();
    }
    
    auto as_tuple()
    {
        return std::make_tuple();
    }
    typedef void Response;
};
void insert(Serializer& serializer, const CommandedAngularZupt& self);
void extract(Serializer& serializer, CommandedAngularZupt& self);

TypedResult<CommandedAngularZupt> commandedAngularZupt(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_mag_capture_auto_cal  (0x0D,0x27) Mag Capture Auto Cal [CPP]
/// This command captures the current value of the auto-calibration, applies it to the current fixed hard and soft iron calibration coefficients, and replaces the current fixed hard and soft iron calibration coefficients with the new values.
/// This may be used in place of (or in addition to) a manual hard and soft iron calibration utility. This command also resets the auto-calibration coefficients.
/// Function selector SAVE is the same as issuing the 0x0C, 0x3A and 0x0C, 0x3B commands with the SAVE function selector.
///
///@{

struct MagCaptureAutoCal
{
    FunctionSelector function = static_cast<FunctionSelector>(0);
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_MAG_CAPTURE_AUTO_CALIBRATION;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "MagCaptureAutoCal";
    static constexpr const char* DOC_NAME = "Magnetometer Capture Auto Calibration";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    static constexpr const uint32_t WRITE_PARAMS   = 0x8000;
    static constexpr const uint32_t READ_PARAMS    = 0x0000;
    static constexpr const uint32_t SAVE_PARAMS    = 0x8000;
    static constexpr const uint32_t LOAD_PARAMS    = 0x0000;
    static constexpr const uint32_t DEFAULT_PARAMS = 0x0000;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple();
    }
    
    auto as_tuple()
    {
        return std::make_tuple();
    }
    
    static MagCaptureAutoCal create_sld_all(::mip::FunctionSelector function)
    {
        MagCaptureAutoCal cmd;
        cmd.function = function;
        return cmd;
    }
    
    typedef void Response;
};
void insert(Serializer& serializer, const MagCaptureAutoCal& self);
void extract(Serializer& serializer, MagCaptureAutoCal& self);

TypedResult<MagCaptureAutoCal> writeMagCaptureAutoCal(C::mip_interface& device);
TypedResult<MagCaptureAutoCal> saveMagCaptureAutoCal(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_gravity_noise  (0x0D,0x28) Gravity Noise [CPP]
/// Set the expected gravity noise 1-sigma values. This function can be used to tune the filter performance in the target application.
/// 
/// Note: Noise values must be greater than 0.0
/// 
/// The noise value represents process noise in the Estimation Filter. Changing this value modifies how the filter responds to dynamic input and can be used to tune filter performance.
/// Default values provide good performance for most laboratory conditions.
///
///@{

struct GravityNoise
{
    FunctionSelector function = static_cast<FunctionSelector>(0);
    Vector3f noise; ///< Gravity Noise 1-sigma [gauss]
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_GRAVITY_NOISE;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "GravityNoise";
    static constexpr const char* DOC_NAME = "Gravity Noise Standard Deviation";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    static constexpr const uint32_t WRITE_PARAMS   = 0x8001;
    static constexpr const uint32_t READ_PARAMS    = 0x8000;
    static constexpr const uint32_t SAVE_PARAMS    = 0x8000;
    static constexpr const uint32_t LOAD_PARAMS    = 0x8000;
    static constexpr const uint32_t DEFAULT_PARAMS = 0x8000;
    static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(noise);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(noise));
    }
    
    static GravityNoise create_sld_all(::mip::FunctionSelector function)
    {
        GravityNoise cmd;
        cmd.function = function;
        return cmd;
    }
    
    struct Response
    {
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_GRAVITY_NOISE;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "GravityNoise::Response";
        static constexpr const char* DOC_NAME = "Gravity Noise Standard Deviation Response";
        
        static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
        static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
        Vector3f noise; ///< Gravity Noise 1-sigma [gauss]
        
        
        auto as_tuple()
        {
            return std::make_tuple(std::ref(noise));
        }
    };
};
void insert(Serializer& serializer, const GravityNoise& self);
void extract(Serializer& serializer, GravityNoise& self);

void insert(Serializer& serializer, const GravityNoise::Response& self);
void extract(Serializer& serializer, GravityNoise::Response& self);

TypedResult<GravityNoise> writeGravityNoise(C::mip_interface& device, const float* noise);
TypedResult<GravityNoise> readGravityNoise(C::mip_interface& device, float* noiseOut);
TypedResult<GravityNoise> saveGravityNoise(C::mip_interface& device);
TypedResult<GravityNoise> loadGravityNoise(C::mip_interface& device);
TypedResult<GravityNoise> defaultGravityNoise(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_pressure_altitude_noise  (0x0D,0x29) Pressure Altitude Noise [CPP]
/// Set the expected pressure altitude noise 1-sigma values. This function can be used to tune the filter performance in the target application.
/// 
/// The noise value must be greater than 0.0
/// 
/// This noise value represents pressure altitude model noise in the Estimation Filter.
/// A lower value will increase responsiveness of the sensor to pressure changes, however height estimates will be more susceptible to error from air pressure fluctuations not due to changes in altitude. Default values provide good performance for most laboratory conditions.
///
///@{

struct PressureAltitudeNoise
{
    FunctionSelector function = static_cast<FunctionSelector>(0);
    float noise = 0; ///< Pressure Altitude Noise 1-sigma [m]
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_PRESSURE_NOISE;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "PressureAltitudeNoise";
    static constexpr const char* DOC_NAME = "Pressure Altitude Noise Standard Deviation";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    static constexpr const uint32_t WRITE_PARAMS   = 0x8001;
    static constexpr const uint32_t READ_PARAMS    = 0x8000;
    static constexpr const uint32_t SAVE_PARAMS    = 0x8000;
    static constexpr const uint32_t LOAD_PARAMS    = 0x8000;
    static constexpr const uint32_t DEFAULT_PARAMS = 0x8000;
    static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(noise);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(noise));
    }
    
    static PressureAltitudeNoise create_sld_all(::mip::FunctionSelector function)
    {
        PressureAltitudeNoise cmd;
        cmd.function = function;
        return cmd;
    }
    
    struct Response
    {
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_PRESSURE_NOISE;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "PressureAltitudeNoise::Response";
        static constexpr const char* DOC_NAME = "Pressure Altitude Noise Standard Deviation Response";
        
        static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
        static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
        float noise = 0; ///< Pressure Altitude Noise 1-sigma [m]
        
        
        auto as_tuple()
        {
            return std::make_tuple(std::ref(noise));
        }
    };
};
void insert(Serializer& serializer, const PressureAltitudeNoise& self);
void extract(Serializer& serializer, PressureAltitudeNoise& self);

void insert(Serializer& serializer, const PressureAltitudeNoise::Response& self);
void extract(Serializer& serializer, PressureAltitudeNoise::Response& self);

TypedResult<PressureAltitudeNoise> writePressureAltitudeNoise(C::mip_interface& device, float noise);
TypedResult<PressureAltitudeNoise> readPressureAltitudeNoise(C::mip_interface& device, float* noiseOut);
TypedResult<PressureAltitudeNoise> savePressureAltitudeNoise(C::mip_interface& device);
TypedResult<PressureAltitudeNoise> loadPressureAltitudeNoise(C::mip_interface& device);
TypedResult<PressureAltitudeNoise> defaultPressureAltitudeNoise(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_hard_iron_offset_noise  (0x0D,0x2B) Hard Iron Offset Noise [CPP]
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

struct HardIronOffsetNoise
{
    FunctionSelector function = static_cast<FunctionSelector>(0);
    Vector3f noise; ///< Hard Iron Offset Noise 1-sigma [gauss]
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_HARD_IRON_OFFSET_NOISE;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "HardIronOffsetNoise";
    static constexpr const char* DOC_NAME = "Hard Iron Offset Process Noise";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    static constexpr const uint32_t WRITE_PARAMS   = 0x8001;
    static constexpr const uint32_t READ_PARAMS    = 0x8000;
    static constexpr const uint32_t SAVE_PARAMS    = 0x8000;
    static constexpr const uint32_t LOAD_PARAMS    = 0x8000;
    static constexpr const uint32_t DEFAULT_PARAMS = 0x8000;
    static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(noise);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(noise));
    }
    
    static HardIronOffsetNoise create_sld_all(::mip::FunctionSelector function)
    {
        HardIronOffsetNoise cmd;
        cmd.function = function;
        return cmd;
    }
    
    struct Response
    {
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_HARD_IRON_OFFSET_NOISE;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "HardIronOffsetNoise::Response";
        static constexpr const char* DOC_NAME = "Hard Iron Offset Process Noise Response";
        
        static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
        static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
        Vector3f noise; ///< Hard Iron Offset Noise 1-sigma [gauss]
        
        
        auto as_tuple()
        {
            return std::make_tuple(std::ref(noise));
        }
    };
};
void insert(Serializer& serializer, const HardIronOffsetNoise& self);
void extract(Serializer& serializer, HardIronOffsetNoise& self);

void insert(Serializer& serializer, const HardIronOffsetNoise::Response& self);
void extract(Serializer& serializer, HardIronOffsetNoise::Response& self);

TypedResult<HardIronOffsetNoise> writeHardIronOffsetNoise(C::mip_interface& device, const float* noise);
TypedResult<HardIronOffsetNoise> readHardIronOffsetNoise(C::mip_interface& device, float* noiseOut);
TypedResult<HardIronOffsetNoise> saveHardIronOffsetNoise(C::mip_interface& device);
TypedResult<HardIronOffsetNoise> loadHardIronOffsetNoise(C::mip_interface& device);
TypedResult<HardIronOffsetNoise> defaultHardIronOffsetNoise(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_soft_iron_matrix_noise  (0x0D,0x2C) Soft Iron Matrix Noise [CPP]
/// Set the expected soft iron matrix noise 1-sigma values.
/// This function can be used to tune the filter performance in the target application.
/// 
/// Noise values must be greater than 0.0
/// 
/// The noise value represents process noise in the Estimation Filter.
/// Changing this value modifies how the filter responds to dynamic input and can be used to tune the performance of the filter. Default values provide good performance for most laboratory conditions.
///
///@{

struct SoftIronMatrixNoise
{
    FunctionSelector function = static_cast<FunctionSelector>(0);
    Matrix3f noise; ///< Soft Iron Matrix Noise 1-sigma [dimensionless]
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_SOFT_IRON_MATRIX_NOISE;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "SoftIronMatrixNoise";
    static constexpr const char* DOC_NAME = "Soft Iron Offset Process Noise";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    static constexpr const uint32_t WRITE_PARAMS   = 0x8001;
    static constexpr const uint32_t READ_PARAMS    = 0x8000;
    static constexpr const uint32_t SAVE_PARAMS    = 0x8000;
    static constexpr const uint32_t LOAD_PARAMS    = 0x8000;
    static constexpr const uint32_t DEFAULT_PARAMS = 0x8000;
    static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(noise);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(noise));
    }
    
    static SoftIronMatrixNoise create_sld_all(::mip::FunctionSelector function)
    {
        SoftIronMatrixNoise cmd;
        cmd.function = function;
        return cmd;
    }
    
    struct Response
    {
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_SOFT_IRON_MATRIX_NOISE;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "SoftIronMatrixNoise::Response";
        static constexpr const char* DOC_NAME = "Soft Iron Offset Process Noise Response";
        
        static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
        static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
        Matrix3f noise; ///< Soft Iron Matrix Noise 1-sigma [dimensionless]
        
        
        auto as_tuple()
        {
            return std::make_tuple(std::ref(noise));
        }
    };
};
void insert(Serializer& serializer, const SoftIronMatrixNoise& self);
void extract(Serializer& serializer, SoftIronMatrixNoise& self);

void insert(Serializer& serializer, const SoftIronMatrixNoise::Response& self);
void extract(Serializer& serializer, SoftIronMatrixNoise::Response& self);

TypedResult<SoftIronMatrixNoise> writeSoftIronMatrixNoise(C::mip_interface& device, const float* noise);
TypedResult<SoftIronMatrixNoise> readSoftIronMatrixNoise(C::mip_interface& device, float* noiseOut);
TypedResult<SoftIronMatrixNoise> saveSoftIronMatrixNoise(C::mip_interface& device);
TypedResult<SoftIronMatrixNoise> loadSoftIronMatrixNoise(C::mip_interface& device);
TypedResult<SoftIronMatrixNoise> defaultSoftIronMatrixNoise(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_mag_noise  (0x0D,0x42) Mag Noise [CPP]
/// Set the expected magnetometer noise 1-sigma values.
/// This function can be used to tune the filter performance in the target application.
/// 
/// Noise values must be greater than 0.0 (gauss)
/// 
/// The noise value represents process noise in the Estimation Filter.
/// Changing this value modifies how the filter responds to dynamic input and can be used to tune the performance of the filter. Default values provide good performance for most laboratory conditions
///
///@{

struct MagNoise
{
    FunctionSelector function = static_cast<FunctionSelector>(0);
    Vector3f noise; ///< Mag Noise 1-sigma [gauss]
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_MAG_NOISE;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "MagNoise";
    static constexpr const char* DOC_NAME = "Magnetometer Noise Standard Deviation";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    static constexpr const uint32_t WRITE_PARAMS   = 0x8001;
    static constexpr const uint32_t READ_PARAMS    = 0x8000;
    static constexpr const uint32_t SAVE_PARAMS    = 0x8000;
    static constexpr const uint32_t LOAD_PARAMS    = 0x8000;
    static constexpr const uint32_t DEFAULT_PARAMS = 0x8000;
    static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(noise);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(noise));
    }
    
    static MagNoise create_sld_all(::mip::FunctionSelector function)
    {
        MagNoise cmd;
        cmd.function = function;
        return cmd;
    }
    
    struct Response
    {
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_MAG_NOISE;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "MagNoise::Response";
        static constexpr const char* DOC_NAME = "Magnetometer Noise Standard Deviation Response";
        
        static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
        static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
        Vector3f noise; ///< Mag Noise 1-sigma [gauss]
        
        
        auto as_tuple()
        {
            return std::make_tuple(std::ref(noise));
        }
    };
};
void insert(Serializer& serializer, const MagNoise& self);
void extract(Serializer& serializer, MagNoise& self);

void insert(Serializer& serializer, const MagNoise::Response& self);
void extract(Serializer& serializer, MagNoise::Response& self);

TypedResult<MagNoise> writeMagNoise(C::mip_interface& device, const float* noise);
TypedResult<MagNoise> readMagNoise(C::mip_interface& device, float* noiseOut);
TypedResult<MagNoise> saveMagNoise(C::mip_interface& device);
TypedResult<MagNoise> loadMagNoise(C::mip_interface& device);
TypedResult<MagNoise> defaultMagNoise(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_inclination_source  (0x0D,0x4C) Inclination Source [CPP]
/// Set/Get the local magnetic field inclination angle source.
/// 
/// This can be used to correct for the local value of inclination (dip angle) of the earthmagnetic field.
/// Having a correct value is important for best performance of the auto-mag calibration feature. If you do not have an accurate inclination angle source, it is recommended that you leave the auto-mag calibration feature off.
/// 
///
///@{

struct InclinationSource
{
    FunctionSelector function = static_cast<FunctionSelector>(0);
    FilterMagParamSource source = static_cast<FilterMagParamSource>(0); ///< Inclination Source
    float inclination = 0; ///< Inclination angle [radians] (only required if source = MANUAL)
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_INCLINATION_SOURCE;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "InclinationSource";
    static constexpr const char* DOC_NAME = "Inclination Source";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    static constexpr const uint32_t WRITE_PARAMS   = 0x8003;
    static constexpr const uint32_t READ_PARAMS    = 0x8000;
    static constexpr const uint32_t SAVE_PARAMS    = 0x8000;
    static constexpr const uint32_t LOAD_PARAMS    = 0x8000;
    static constexpr const uint32_t DEFAULT_PARAMS = 0x8000;
    static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(source,inclination);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(source),std::ref(inclination));
    }
    
    static InclinationSource create_sld_all(::mip::FunctionSelector function)
    {
        InclinationSource cmd;
        cmd.function = function;
        return cmd;
    }
    
    struct Response
    {
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_INCLINATION_SOURCE;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "InclinationSource::Response";
        static constexpr const char* DOC_NAME = "Inclination Source Response";
        
        static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
        static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
        FilterMagParamSource source = static_cast<FilterMagParamSource>(0); ///< Inclination Source
        float inclination = 0; ///< Inclination angle [radians] (only required if source = MANUAL)
        
        
        auto as_tuple()
        {
            return std::make_tuple(std::ref(source),std::ref(inclination));
        }
    };
};
void insert(Serializer& serializer, const InclinationSource& self);
void extract(Serializer& serializer, InclinationSource& self);

void insert(Serializer& serializer, const InclinationSource::Response& self);
void extract(Serializer& serializer, InclinationSource::Response& self);

TypedResult<InclinationSource> writeInclinationSource(C::mip_interface& device, FilterMagParamSource source, float inclination);
TypedResult<InclinationSource> readInclinationSource(C::mip_interface& device, FilterMagParamSource* sourceOut, float* inclinationOut);
TypedResult<InclinationSource> saveInclinationSource(C::mip_interface& device);
TypedResult<InclinationSource> loadInclinationSource(C::mip_interface& device);
TypedResult<InclinationSource> defaultInclinationSource(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_magnetic_declination_source  (0x0D,0x43) Magnetic Declination Source [CPP]
/// Set/Get the local magnetic field declination angle source.
/// 
/// This can be used to correct for the local value of declination of the earthmagnetic field.
/// Having a correct value is important for best performance of the auto-mag calibration feature. If you do not have an accurate inclination angle source, it is recommended that you leave the auto-mag calibration feature off.
/// 
///
///@{

struct MagneticDeclinationSource
{
    FunctionSelector function = static_cast<FunctionSelector>(0);
    FilterMagParamSource source = static_cast<FilterMagParamSource>(0); ///< Magnetic field declination angle source
    float declination = 0; ///< Declination angle [radians] (only required if source = MANUAL)
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_DECLINATION_SOURCE;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "MagneticDeclinationSource";
    static constexpr const char* DOC_NAME = "Magnetic Field Declination Source Control";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    static constexpr const uint32_t WRITE_PARAMS   = 0x8003;
    static constexpr const uint32_t READ_PARAMS    = 0x8000;
    static constexpr const uint32_t SAVE_PARAMS    = 0x8000;
    static constexpr const uint32_t LOAD_PARAMS    = 0x8000;
    static constexpr const uint32_t DEFAULT_PARAMS = 0x8000;
    static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(source,declination);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(source),std::ref(declination));
    }
    
    static MagneticDeclinationSource create_sld_all(::mip::FunctionSelector function)
    {
        MagneticDeclinationSource cmd;
        cmd.function = function;
        return cmd;
    }
    
    struct Response
    {
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_DECLINATION_SOURCE;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "MagneticDeclinationSource::Response";
        static constexpr const char* DOC_NAME = "Magnetic Field Declination Source Control Response";
        
        static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
        static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
        FilterMagParamSource source = static_cast<FilterMagParamSource>(0); ///< Magnetic field declination angle source
        float declination = 0; ///< Declination angle [radians] (only required if source = MANUAL)
        
        
        auto as_tuple()
        {
            return std::make_tuple(std::ref(source),std::ref(declination));
        }
    };
};
void insert(Serializer& serializer, const MagneticDeclinationSource& self);
void extract(Serializer& serializer, MagneticDeclinationSource& self);

void insert(Serializer& serializer, const MagneticDeclinationSource::Response& self);
void extract(Serializer& serializer, MagneticDeclinationSource::Response& self);

TypedResult<MagneticDeclinationSource> writeMagneticDeclinationSource(C::mip_interface& device, FilterMagParamSource source, float declination);
TypedResult<MagneticDeclinationSource> readMagneticDeclinationSource(C::mip_interface& device, FilterMagParamSource* sourceOut, float* declinationOut);
TypedResult<MagneticDeclinationSource> saveMagneticDeclinationSource(C::mip_interface& device);
TypedResult<MagneticDeclinationSource> loadMagneticDeclinationSource(C::mip_interface& device);
TypedResult<MagneticDeclinationSource> defaultMagneticDeclinationSource(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_mag_field_magnitude_source  (0x0D,0x4D) Mag Field Magnitude Source [CPP]
/// Set/Get the local magnetic field magnitude source.
/// 
/// This is used to specify the local magnitude of the earth's magnetic field.
/// Having a correct value for magnitude is important for best performance of the auto-mag calibration feature and for the magnetometer adaptive magnitude. If you do not have an accurate value for the local magnetic field magnitude, it is recommended that you leave the auto-mag calibration feature off.
///
///@{

struct MagFieldMagnitudeSource
{
    FunctionSelector function = static_cast<FunctionSelector>(0);
    FilterMagParamSource source = static_cast<FilterMagParamSource>(0); ///< Magnetic Field Magnitude Source
    float magnitude = 0; ///< Magnitude [gauss] (only required if source = MANUAL)
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_MAGNETIC_MAGNITUDE_SOURCE;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "MagFieldMagnitudeSource";
    static constexpr const char* DOC_NAME = "Magnetic Field Magnitude Source";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    static constexpr const uint32_t WRITE_PARAMS   = 0x8003;
    static constexpr const uint32_t READ_PARAMS    = 0x8000;
    static constexpr const uint32_t SAVE_PARAMS    = 0x8000;
    static constexpr const uint32_t LOAD_PARAMS    = 0x8000;
    static constexpr const uint32_t DEFAULT_PARAMS = 0x8000;
    static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(source,magnitude);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(source),std::ref(magnitude));
    }
    
    static MagFieldMagnitudeSource create_sld_all(::mip::FunctionSelector function)
    {
        MagFieldMagnitudeSource cmd;
        cmd.function = function;
        return cmd;
    }
    
    struct Response
    {
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_MAGNETIC_MAGNITUDE_SOURCE;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "MagFieldMagnitudeSource::Response";
        static constexpr const char* DOC_NAME = "Magnetic Field Magnitude Source Response";
        
        static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
        static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
        FilterMagParamSource source = static_cast<FilterMagParamSource>(0); ///< Magnetic Field Magnitude Source
        float magnitude = 0; ///< Magnitude [gauss] (only required if source = MANUAL)
        
        
        auto as_tuple()
        {
            return std::make_tuple(std::ref(source),std::ref(magnitude));
        }
    };
};
void insert(Serializer& serializer, const MagFieldMagnitudeSource& self);
void extract(Serializer& serializer, MagFieldMagnitudeSource& self);

void insert(Serializer& serializer, const MagFieldMagnitudeSource::Response& self);
void extract(Serializer& serializer, MagFieldMagnitudeSource::Response& self);

TypedResult<MagFieldMagnitudeSource> writeMagFieldMagnitudeSource(C::mip_interface& device, FilterMagParamSource source, float magnitude);
TypedResult<MagFieldMagnitudeSource> readMagFieldMagnitudeSource(C::mip_interface& device, FilterMagParamSource* sourceOut, float* magnitudeOut);
TypedResult<MagFieldMagnitudeSource> saveMagFieldMagnitudeSource(C::mip_interface& device);
TypedResult<MagFieldMagnitudeSource> loadMagFieldMagnitudeSource(C::mip_interface& device);
TypedResult<MagFieldMagnitudeSource> defaultMagFieldMagnitudeSource(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_reference_position  (0x0D,0x26) Reference Position [CPP]
/// Set the Lat/Long/Alt reference position for the sensor.
/// 
/// This position is used by the sensor to calculate the WGS84 gravity and WMM2015 magnetic field parameters.
/// 
///
///@{

struct ReferencePosition
{
    FunctionSelector function = static_cast<FunctionSelector>(0);
    bool enable = 0; ///< enable/disable
    double latitude = 0; ///< [degrees]
    double longitude = 0; ///< [degrees]
    double altitude = 0; ///< [meters]
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_REFERENCE_POSITION;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "ReferencePosition";
    static constexpr const char* DOC_NAME = "Set Reference Position";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    static constexpr const uint32_t WRITE_PARAMS   = 0x800F;
    static constexpr const uint32_t READ_PARAMS    = 0x8000;
    static constexpr const uint32_t SAVE_PARAMS    = 0x8000;
    static constexpr const uint32_t LOAD_PARAMS    = 0x8000;
    static constexpr const uint32_t DEFAULT_PARAMS = 0x8000;
    static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(enable,latitude,longitude,altitude);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(enable),std::ref(latitude),std::ref(longitude),std::ref(altitude));
    }
    
    static ReferencePosition create_sld_all(::mip::FunctionSelector function)
    {
        ReferencePosition cmd;
        cmd.function = function;
        return cmd;
    }
    
    struct Response
    {
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_REFERENCE_POSITION;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "ReferencePosition::Response";
        static constexpr const char* DOC_NAME = "Set Reference Position Response";
        
        static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
        static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
        bool enable = 0; ///< enable/disable
        double latitude = 0; ///< [degrees]
        double longitude = 0; ///< [degrees]
        double altitude = 0; ///< [meters]
        
        
        auto as_tuple()
        {
            return std::make_tuple(std::ref(enable),std::ref(latitude),std::ref(longitude),std::ref(altitude));
        }
    };
};
void insert(Serializer& serializer, const ReferencePosition& self);
void extract(Serializer& serializer, ReferencePosition& self);

void insert(Serializer& serializer, const ReferencePosition::Response& self);
void extract(Serializer& serializer, ReferencePosition::Response& self);

TypedResult<ReferencePosition> writeReferencePosition(C::mip_interface& device, bool enable, double latitude, double longitude, double altitude);
TypedResult<ReferencePosition> readReferencePosition(C::mip_interface& device, bool* enableOut, double* latitudeOut, double* longitudeOut, double* altitudeOut);
TypedResult<ReferencePosition> saveReferencePosition(C::mip_interface& device);
TypedResult<ReferencePosition> loadReferencePosition(C::mip_interface& device);
TypedResult<ReferencePosition> defaultReferencePosition(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_accel_magnitude_error_adaptive_measurement  (0x0D,0x44) Accel Magnitude Error Adaptive Measurement [CPP]
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

struct AccelMagnitudeErrorAdaptiveMeasurement
{
    FunctionSelector function = static_cast<FunctionSelector>(0);
    FilterAdaptiveMeasurement adaptive_measurement = static_cast<FilterAdaptiveMeasurement>(0); ///< Adaptive measurement selector
    float frequency = 0; ///< Low-pass filter curoff frequency [hertz]
    float low_limit = 0; ///< [meters/second^2]
    float high_limit = 0; ///< [meters/second^2]
    float low_limit_uncertainty = 0; ///< 1-Sigma [meters/second^2]
    float high_limit_uncertainty = 0; ///< 1-Sigma [meters/second^2]
    float minimum_uncertainty = 0; ///< 1-Sigma [meters/second^2]
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_ACCEL_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "AccelMagnitudeErrorAdaptiveMeasurement";
    static constexpr const char* DOC_NAME = "Gravity Magnitude Error Adaptive Measurement";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    static constexpr const uint32_t WRITE_PARAMS   = 0x807F;
    static constexpr const uint32_t READ_PARAMS    = 0x8000;
    static constexpr const uint32_t SAVE_PARAMS    = 0x8000;
    static constexpr const uint32_t LOAD_PARAMS    = 0x8000;
    static constexpr const uint32_t DEFAULT_PARAMS = 0x8000;
    static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(adaptive_measurement,frequency,low_limit,high_limit,low_limit_uncertainty,high_limit_uncertainty,minimum_uncertainty);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(adaptive_measurement),std::ref(frequency),std::ref(low_limit),std::ref(high_limit),std::ref(low_limit_uncertainty),std::ref(high_limit_uncertainty),std::ref(minimum_uncertainty));
    }
    
    static AccelMagnitudeErrorAdaptiveMeasurement create_sld_all(::mip::FunctionSelector function)
    {
        AccelMagnitudeErrorAdaptiveMeasurement cmd;
        cmd.function = function;
        return cmd;
    }
    
    struct Response
    {
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_ACCEL_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "AccelMagnitudeErrorAdaptiveMeasurement::Response";
        static constexpr const char* DOC_NAME = "Gravity Magnitude Error Adaptive Measurement Response";
        
        static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
        static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
        FilterAdaptiveMeasurement adaptive_measurement = static_cast<FilterAdaptiveMeasurement>(0); ///< Adaptive measurement selector
        float frequency = 0; ///< Low-pass filter curoff frequency [hertz]
        float low_limit = 0; ///< [meters/second^2]
        float high_limit = 0; ///< [meters/second^2]
        float low_limit_uncertainty = 0; ///< 1-Sigma [meters/second^2]
        float high_limit_uncertainty = 0; ///< 1-Sigma [meters/second^2]
        float minimum_uncertainty = 0; ///< 1-Sigma [meters/second^2]
        
        
        auto as_tuple()
        {
            return std::make_tuple(std::ref(adaptive_measurement),std::ref(frequency),std::ref(low_limit),std::ref(high_limit),std::ref(low_limit_uncertainty),std::ref(high_limit_uncertainty),std::ref(minimum_uncertainty));
        }
    };
};
void insert(Serializer& serializer, const AccelMagnitudeErrorAdaptiveMeasurement& self);
void extract(Serializer& serializer, AccelMagnitudeErrorAdaptiveMeasurement& self);

void insert(Serializer& serializer, const AccelMagnitudeErrorAdaptiveMeasurement::Response& self);
void extract(Serializer& serializer, AccelMagnitudeErrorAdaptiveMeasurement::Response& self);

TypedResult<AccelMagnitudeErrorAdaptiveMeasurement> writeAccelMagnitudeErrorAdaptiveMeasurement(C::mip_interface& device, FilterAdaptiveMeasurement adaptiveMeasurement, float frequency, float lowLimit, float highLimit, float lowLimitUncertainty, float highLimitUncertainty, float minimumUncertainty);
TypedResult<AccelMagnitudeErrorAdaptiveMeasurement> readAccelMagnitudeErrorAdaptiveMeasurement(C::mip_interface& device, FilterAdaptiveMeasurement* adaptiveMeasurementOut, float* frequencyOut, float* lowLimitOut, float* highLimitOut, float* lowLimitUncertaintyOut, float* highLimitUncertaintyOut, float* minimumUncertaintyOut);
TypedResult<AccelMagnitudeErrorAdaptiveMeasurement> saveAccelMagnitudeErrorAdaptiveMeasurement(C::mip_interface& device);
TypedResult<AccelMagnitudeErrorAdaptiveMeasurement> loadAccelMagnitudeErrorAdaptiveMeasurement(C::mip_interface& device);
TypedResult<AccelMagnitudeErrorAdaptiveMeasurement> defaultAccelMagnitudeErrorAdaptiveMeasurement(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_mag_magnitude_error_adaptive_measurement  (0x0D,0x45) Mag Magnitude Error Adaptive Measurement [CPP]
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

struct MagMagnitudeErrorAdaptiveMeasurement
{
    FunctionSelector function = static_cast<FunctionSelector>(0);
    FilterAdaptiveMeasurement adaptive_measurement = static_cast<FilterAdaptiveMeasurement>(0); ///< Adaptive measurement selector
    float frequency = 0; ///< Low-pass filter curoff frequency [hertz]
    float low_limit = 0; ///< [meters/second^2]
    float high_limit = 0; ///< [meters/second^2]
    float low_limit_uncertainty = 0; ///< 1-Sigma [meters/second^2]
    float high_limit_uncertainty = 0; ///< 1-Sigma [meters/second^2]
    float minimum_uncertainty = 0; ///< 1-Sigma [meters/second^2]
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_MAG_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "MagMagnitudeErrorAdaptiveMeasurement";
    static constexpr const char* DOC_NAME = "Magnetometer Magnitude Error Adaptive Measurement";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    static constexpr const uint32_t WRITE_PARAMS   = 0x807F;
    static constexpr const uint32_t READ_PARAMS    = 0x8000;
    static constexpr const uint32_t SAVE_PARAMS    = 0x8000;
    static constexpr const uint32_t LOAD_PARAMS    = 0x8000;
    static constexpr const uint32_t DEFAULT_PARAMS = 0x8000;
    static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(adaptive_measurement,frequency,low_limit,high_limit,low_limit_uncertainty,high_limit_uncertainty,minimum_uncertainty);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(adaptive_measurement),std::ref(frequency),std::ref(low_limit),std::ref(high_limit),std::ref(low_limit_uncertainty),std::ref(high_limit_uncertainty),std::ref(minimum_uncertainty));
    }
    
    static MagMagnitudeErrorAdaptiveMeasurement create_sld_all(::mip::FunctionSelector function)
    {
        MagMagnitudeErrorAdaptiveMeasurement cmd;
        cmd.function = function;
        return cmd;
    }
    
    struct Response
    {
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_MAG_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "MagMagnitudeErrorAdaptiveMeasurement::Response";
        static constexpr const char* DOC_NAME = "Magnetometer Magnitude Error Adaptive Measurement Response";
        
        static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
        static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
        FilterAdaptiveMeasurement adaptive_measurement = static_cast<FilterAdaptiveMeasurement>(0); ///< Adaptive measurement selector
        float frequency = 0; ///< Low-pass filter curoff frequency [hertz]
        float low_limit = 0; ///< [meters/second^2]
        float high_limit = 0; ///< [meters/second^2]
        float low_limit_uncertainty = 0; ///< 1-Sigma [meters/second^2]
        float high_limit_uncertainty = 0; ///< 1-Sigma [meters/second^2]
        float minimum_uncertainty = 0; ///< 1-Sigma [meters/second^2]
        
        
        auto as_tuple()
        {
            return std::make_tuple(std::ref(adaptive_measurement),std::ref(frequency),std::ref(low_limit),std::ref(high_limit),std::ref(low_limit_uncertainty),std::ref(high_limit_uncertainty),std::ref(minimum_uncertainty));
        }
    };
};
void insert(Serializer& serializer, const MagMagnitudeErrorAdaptiveMeasurement& self);
void extract(Serializer& serializer, MagMagnitudeErrorAdaptiveMeasurement& self);

void insert(Serializer& serializer, const MagMagnitudeErrorAdaptiveMeasurement::Response& self);
void extract(Serializer& serializer, MagMagnitudeErrorAdaptiveMeasurement::Response& self);

TypedResult<MagMagnitudeErrorAdaptiveMeasurement> writeMagMagnitudeErrorAdaptiveMeasurement(C::mip_interface& device, FilterAdaptiveMeasurement adaptiveMeasurement, float frequency, float lowLimit, float highLimit, float lowLimitUncertainty, float highLimitUncertainty, float minimumUncertainty);
TypedResult<MagMagnitudeErrorAdaptiveMeasurement> readMagMagnitudeErrorAdaptiveMeasurement(C::mip_interface& device, FilterAdaptiveMeasurement* adaptiveMeasurementOut, float* frequencyOut, float* lowLimitOut, float* highLimitOut, float* lowLimitUncertaintyOut, float* highLimitUncertaintyOut, float* minimumUncertaintyOut);
TypedResult<MagMagnitudeErrorAdaptiveMeasurement> saveMagMagnitudeErrorAdaptiveMeasurement(C::mip_interface& device);
TypedResult<MagMagnitudeErrorAdaptiveMeasurement> loadMagMagnitudeErrorAdaptiveMeasurement(C::mip_interface& device);
TypedResult<MagMagnitudeErrorAdaptiveMeasurement> defaultMagMagnitudeErrorAdaptiveMeasurement(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_mag_dip_angle_error_adaptive_measurement  (0x0D,0x46) Mag Dip Angle Error Adaptive Measurement [CPP]
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

struct MagDipAngleErrorAdaptiveMeasurement
{
    FunctionSelector function = static_cast<FunctionSelector>(0);
    bool enable = 0; ///< Enable/Disable
    float frequency = 0; ///< Low-pass filter curoff frequency [hertz]
    float high_limit = 0; ///< [meters/second^2]
    float high_limit_uncertainty = 0; ///< 1-Sigma [meters/second^2]
    float minimum_uncertainty = 0; ///< 1-Sigma [meters/second^2]
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_MAG_DIP_ANGLE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "MagDipAngleErrorAdaptiveMeasurement";
    static constexpr const char* DOC_NAME = "Magnetometer Dig Angle Error Adaptive Measurement";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    static constexpr const uint32_t WRITE_PARAMS   = 0x801F;
    static constexpr const uint32_t READ_PARAMS    = 0x8000;
    static constexpr const uint32_t SAVE_PARAMS    = 0x8000;
    static constexpr const uint32_t LOAD_PARAMS    = 0x8000;
    static constexpr const uint32_t DEFAULT_PARAMS = 0x8000;
    static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(enable,frequency,high_limit,high_limit_uncertainty,minimum_uncertainty);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(enable),std::ref(frequency),std::ref(high_limit),std::ref(high_limit_uncertainty),std::ref(minimum_uncertainty));
    }
    
    static MagDipAngleErrorAdaptiveMeasurement create_sld_all(::mip::FunctionSelector function)
    {
        MagDipAngleErrorAdaptiveMeasurement cmd;
        cmd.function = function;
        return cmd;
    }
    
    struct Response
    {
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_MAG_DIP_ANGLE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "MagDipAngleErrorAdaptiveMeasurement::Response";
        static constexpr const char* DOC_NAME = "Magnetometer Dig Angle Error Adaptive Measurement Response";
        
        static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
        static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
        bool enable = 0; ///< Enable/Disable
        float frequency = 0; ///< Low-pass filter curoff frequency [hertz]
        float high_limit = 0; ///< [meters/second^2]
        float high_limit_uncertainty = 0; ///< 1-Sigma [meters/second^2]
        float minimum_uncertainty = 0; ///< 1-Sigma [meters/second^2]
        
        
        auto as_tuple()
        {
            return std::make_tuple(std::ref(enable),std::ref(frequency),std::ref(high_limit),std::ref(high_limit_uncertainty),std::ref(minimum_uncertainty));
        }
    };
};
void insert(Serializer& serializer, const MagDipAngleErrorAdaptiveMeasurement& self);
void extract(Serializer& serializer, MagDipAngleErrorAdaptiveMeasurement& self);

void insert(Serializer& serializer, const MagDipAngleErrorAdaptiveMeasurement::Response& self);
void extract(Serializer& serializer, MagDipAngleErrorAdaptiveMeasurement::Response& self);

TypedResult<MagDipAngleErrorAdaptiveMeasurement> writeMagDipAngleErrorAdaptiveMeasurement(C::mip_interface& device, bool enable, float frequency, float highLimit, float highLimitUncertainty, float minimumUncertainty);
TypedResult<MagDipAngleErrorAdaptiveMeasurement> readMagDipAngleErrorAdaptiveMeasurement(C::mip_interface& device, bool* enableOut, float* frequencyOut, float* highLimitOut, float* highLimitUncertaintyOut, float* minimumUncertaintyOut);
TypedResult<MagDipAngleErrorAdaptiveMeasurement> saveMagDipAngleErrorAdaptiveMeasurement(C::mip_interface& device);
TypedResult<MagDipAngleErrorAdaptiveMeasurement> loadMagDipAngleErrorAdaptiveMeasurement(C::mip_interface& device);
TypedResult<MagDipAngleErrorAdaptiveMeasurement> defaultMagDipAngleErrorAdaptiveMeasurement(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_aiding_measurement_enable  (0x0D,0x50) Aiding Measurement Enable [CPP]
/// Enables / disables the specified aiding measurement source.
/// 
/// 
///
///@{

struct AidingMeasurementEnable
{
    enum class AidingSource : uint16_t
    {
        GNSS_POS_VEL          = 0,  ///<  GNSS Position and Velocity
        GNSS_HEADING          = 1,  ///<  GNSS Heading (dual antenna)
        ALTIMETER             = 2,  ///<  Pressure altimeter (built-in sensor)
        SPEED                 = 3,  ///<  Speed sensor / Odometer
        MAGNETOMETER          = 4,  ///<  Magnetometer (built-in sensor)
        EXTERNAL_HEADING      = 5,  ///<  External heading input
        EXTERNAL_ALTIMETER    = 6,  ///<  External pressure altimeter input
        EXTERNAL_MAGNETOMETER = 7,  ///<  External magnetomer input
        VEHICLE_FRAME_VEL     = 8,  ///<  External vehicle frame velocity input
        ALL                   = 65535,  ///<  Save/load/reset all options
    };
    
    FunctionSelector function = static_cast<FunctionSelector>(0);
    AidingSource aiding_source = static_cast<AidingSource>(0); ///< Aiding measurement source
    bool enable = 0; ///< Controls the aiding source
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_AIDING_MEASUREMENT_ENABLE;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "AidingMeasurementEnable";
    static constexpr const char* DOC_NAME = "Aiding Measurement Control";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    static constexpr const uint32_t WRITE_PARAMS   = 0x8003;
    static constexpr const uint32_t READ_PARAMS    = 0x8001;
    static constexpr const uint32_t SAVE_PARAMS    = 0x8001;
    static constexpr const uint32_t LOAD_PARAMS    = 0x8001;
    static constexpr const uint32_t DEFAULT_PARAMS = 0x8001;
    static constexpr const uint32_t ECHOED_PARAMS  = 0x0001;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(aiding_source,enable);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(aiding_source),std::ref(enable));
    }
    
    static AidingMeasurementEnable create_sld_all(::mip::FunctionSelector function)
    {
        AidingMeasurementEnable cmd;
        cmd.function = function;
        cmd.aiding_source = ::mip::commands_filter::AidingMeasurementEnable::AidingSource::ALL;
        return cmd;
    }
    
    struct Response
    {
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_AIDING_MEASUREMENT_ENABLE;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "AidingMeasurementEnable::Response";
        static constexpr const char* DOC_NAME = "Aiding Measurement Control Response";
        
        static constexpr const uint32_t ECHOED_PARAMS  = 0x0001;
        static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
        AidingSource aiding_source = static_cast<AidingSource>(0); ///< Aiding measurement source
        bool enable = 0; ///< Controls the aiding source
        
        
        auto as_tuple()
        {
            return std::make_tuple(std::ref(aiding_source),std::ref(enable));
        }
    };
};
void insert(Serializer& serializer, const AidingMeasurementEnable& self);
void extract(Serializer& serializer, AidingMeasurementEnable& self);

void insert(Serializer& serializer, const AidingMeasurementEnable::Response& self);
void extract(Serializer& serializer, AidingMeasurementEnable::Response& self);

TypedResult<AidingMeasurementEnable> writeAidingMeasurementEnable(C::mip_interface& device, AidingMeasurementEnable::AidingSource aidingSource, bool enable);
TypedResult<AidingMeasurementEnable> readAidingMeasurementEnable(C::mip_interface& device, AidingMeasurementEnable::AidingSource aidingSource, bool* enableOut);
TypedResult<AidingMeasurementEnable> saveAidingMeasurementEnable(C::mip_interface& device, AidingMeasurementEnable::AidingSource aidingSource);
TypedResult<AidingMeasurementEnable> loadAidingMeasurementEnable(C::mip_interface& device, AidingMeasurementEnable::AidingSource aidingSource);
TypedResult<AidingMeasurementEnable> defaultAidingMeasurementEnable(C::mip_interface& device, AidingMeasurementEnable::AidingSource aidingSource);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_run  (0x0D,0x05) Run [CPP]
/// Manual run command.
/// 
/// If the initialization configuration has the "wait_for_run_command" option enabled, the filter will wait until it receives this command before commencing integration and enabling the Kalman filter. Prior to the receipt of this command, the filter will remain in the filter initialization mode.
///
///@{

struct Run
{
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_RUN;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "Run";
    static constexpr const char* DOC_NAME = "Run Navigation Filter";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple();
    }
    
    auto as_tuple()
    {
        return std::make_tuple();
    }
    typedef void Response;
};
void insert(Serializer& serializer, const Run& self);
void extract(Serializer& serializer, Run& self);

TypedResult<Run> run(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_kinematic_constraint  (0x0D,0x51) Kinematic Constraint [CPP]
/// Controls kinematic constraint model selection for the navigation filter.
/// 
/// See manual for explanation of how the kinematic constraints are applied.
///
///@{

struct KinematicConstraint
{
    FunctionSelector function = static_cast<FunctionSelector>(0);
    uint8_t acceleration_constraint_selection = 0; ///< Acceleration constraint: <br/> 0=None (default), <br/> 1=Zero-acceleration.
    uint8_t velocity_constraint_selection = 0; ///< 0=None (default), <br/> 1=Zero-velocity, <br/> 2=Wheeled-vehicle. <br/>
    uint8_t angular_constraint_selection = 0; ///< 0=None (default), 1=Zero-angular rate (ZUPT).
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_KINEMATIC_CONSTRAINT;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "KinematicConstraint";
    static constexpr const char* DOC_NAME = "Kinematic Constraint Control";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    static constexpr const uint32_t WRITE_PARAMS   = 0x8007;
    static constexpr const uint32_t READ_PARAMS    = 0x8000;
    static constexpr const uint32_t SAVE_PARAMS    = 0x8000;
    static constexpr const uint32_t LOAD_PARAMS    = 0x8000;
    static constexpr const uint32_t DEFAULT_PARAMS = 0x8000;
    static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(acceleration_constraint_selection,velocity_constraint_selection,angular_constraint_selection);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(acceleration_constraint_selection),std::ref(velocity_constraint_selection),std::ref(angular_constraint_selection));
    }
    
    static KinematicConstraint create_sld_all(::mip::FunctionSelector function)
    {
        KinematicConstraint cmd;
        cmd.function = function;
        return cmd;
    }
    
    struct Response
    {
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_KINEMATIC_CONSTRAINT;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "KinematicConstraint::Response";
        static constexpr const char* DOC_NAME = "Kinematic Constraint Control Response";
        
        static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
        static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
        uint8_t acceleration_constraint_selection = 0; ///< Acceleration constraint: <br/> 0=None (default), <br/> 1=Zero-acceleration.
        uint8_t velocity_constraint_selection = 0; ///< 0=None (default), <br/> 1=Zero-velocity, <br/> 2=Wheeled-vehicle. <br/>
        uint8_t angular_constraint_selection = 0; ///< 0=None (default), 1=Zero-angular rate (ZUPT).
        
        
        auto as_tuple()
        {
            return std::make_tuple(std::ref(acceleration_constraint_selection),std::ref(velocity_constraint_selection),std::ref(angular_constraint_selection));
        }
    };
};
void insert(Serializer& serializer, const KinematicConstraint& self);
void extract(Serializer& serializer, KinematicConstraint& self);

void insert(Serializer& serializer, const KinematicConstraint::Response& self);
void extract(Serializer& serializer, KinematicConstraint::Response& self);

TypedResult<KinematicConstraint> writeKinematicConstraint(C::mip_interface& device, uint8_t accelerationConstraintSelection, uint8_t velocityConstraintSelection, uint8_t angularConstraintSelection);
TypedResult<KinematicConstraint> readKinematicConstraint(C::mip_interface& device, uint8_t* accelerationConstraintSelectionOut, uint8_t* velocityConstraintSelectionOut, uint8_t* angularConstraintSelectionOut);
TypedResult<KinematicConstraint> saveKinematicConstraint(C::mip_interface& device);
TypedResult<KinematicConstraint> loadKinematicConstraint(C::mip_interface& device);
TypedResult<KinematicConstraint> defaultKinematicConstraint(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_initialization_configuration  (0x0D,0x52) Initialization Configuration [CPP]
/// Controls the source and values used for initial conditions of the navigation solution.
/// 
/// Notes: Initial conditions are the position, velocity, and attitude of the platform used when the filter starts running or is reset.
/// For the user specified position array, the units are meters if the ECEF frame is selected, and degrees latitude, degrees longitude, and meters above ellipsoid if the latitude/longitude/height frame is selected.
/// For the user specified velocity array, the units are meters per second, but the reference frame depends on the reference frame selector (ECEF or NED).
///
///@{

struct InitializationConfiguration
{
    struct AlignmentSelector : Bitfield<AlignmentSelector>
    {
        enum _enumType : uint8_t
        {
            NONE         = 0x00,
            DUAL_ANTENNA = 0x01,  ///<  Dual-antenna GNSS alignment
            KINEMATIC    = 0x02,  ///<  GNSS kinematic alignment (GNSS velocity determines initial heading)
            MAGNETOMETER = 0x04,  ///<  Magnetometer heading alignment (Internal magnetometer determines initial heading)
            EXTERNAL     = 0x08,  ///<  External heading alignment (External heading input determines heading)
            ALL          = 0x0F,
        };
        uint8_t value = NONE;
        
        AlignmentSelector() : value(NONE) {}
        AlignmentSelector(int val) : value((uint8_t)val) {}
        operator uint8_t() const { return value; }
        AlignmentSelector& operator=(uint8_t val) { value = val; return *this; }
        AlignmentSelector& operator=(int val) { value = uint8_t(val); return *this; }
        AlignmentSelector& operator|=(uint8_t val) { return *this = value | val; }
        AlignmentSelector& operator&=(uint8_t val) { return *this = value & val; }
        
        bool dualAntenna() const { return (value & DUAL_ANTENNA) > 0; }
        void dualAntenna(bool val) { if(val) value |= DUAL_ANTENNA; else value &= ~DUAL_ANTENNA; }
        bool kinematic() const { return (value & KINEMATIC) > 0; }
        void kinematic(bool val) { if(val) value |= KINEMATIC; else value &= ~KINEMATIC; }
        bool magnetometer() const { return (value & MAGNETOMETER) > 0; }
        void magnetometer(bool val) { if(val) value |= MAGNETOMETER; else value &= ~MAGNETOMETER; }
        bool external() const { return (value & EXTERNAL) > 0; }
        void external(bool val) { if(val) value |= EXTERNAL; else value &= ~EXTERNAL; }
        
        bool allSet() const { return value == ALL; }
        void setAll() { value |= ALL; }
    };
    
    enum class InitialConditionSource : uint8_t
    {
        AUTO_POS_VEL_ATT        = 0,  ///<  Automatic position, velocity and attitude
        AUTO_POS_VEL_PITCH_ROLL = 1,  ///<  Automatic position and velocity, automatic pitch and roll, and user-specified heading
        AUTO_POS_VEL            = 2,  ///<  Automatic position and velocity, with fully user-specified attitude
        MANUAL                  = 3,  ///<  User-specified position, velocity, and attitude.
    };
    
    FunctionSelector function = static_cast<FunctionSelector>(0);
    uint8_t wait_for_run_command = 0; ///< Initialize filter only after receiving "run" command
    InitialConditionSource initial_cond_src = static_cast<InitialConditionSource>(0); ///< Initial condition source:
    AlignmentSelector auto_heading_alignment_selector; ///< Bitfield specifying the allowed automatic heading alignment methods for automatic initial conditions. Bits are set to 1 to enable, and the correspond to the following: <br/>
    float initial_heading = 0; ///< User-specified initial platform heading (degrees).
    float initial_pitch = 0; ///< User-specified initial platform pitch (degrees)
    float initial_roll = 0; ///< User-specified initial platform roll (degrees)
    Vector3f initial_position; ///< User-specified initial platform position (units determined by reference frame selector, see note.)
    Vector3f initial_velocity; ///< User-specified initial platform velocity (units determined by reference frame selector, see note.)
    FilterReferenceFrame reference_frame_selector = static_cast<FilterReferenceFrame>(0); ///< User-specified initial position/velocity reference frames
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_INITIALIZATION_CONFIGURATION;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "InitializationConfiguration";
    static constexpr const char* DOC_NAME = "Navigation Filter Initialization";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    static constexpr const uint32_t WRITE_PARAMS   = 0x81FF;
    static constexpr const uint32_t READ_PARAMS    = 0x8000;
    static constexpr const uint32_t SAVE_PARAMS    = 0x8000;
    static constexpr const uint32_t LOAD_PARAMS    = 0x8000;
    static constexpr const uint32_t DEFAULT_PARAMS = 0x8000;
    static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(wait_for_run_command,initial_cond_src,auto_heading_alignment_selector,initial_heading,initial_pitch,initial_roll,initial_position,initial_velocity,reference_frame_selector);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(wait_for_run_command),std::ref(initial_cond_src),std::ref(auto_heading_alignment_selector),std::ref(initial_heading),std::ref(initial_pitch),std::ref(initial_roll),std::ref(initial_position),std::ref(initial_velocity),std::ref(reference_frame_selector));
    }
    
    static InitializationConfiguration create_sld_all(::mip::FunctionSelector function)
    {
        InitializationConfiguration cmd;
        cmd.function = function;
        return cmd;
    }
    
    struct Response
    {
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_INITIALIZATION_CONFIGURATION;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "InitializationConfiguration::Response";
        static constexpr const char* DOC_NAME = "Navigation Filter Initialization Response";
        
        static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
        static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
        uint8_t wait_for_run_command = 0; ///< Initialize filter only after receiving "run" command
        InitialConditionSource initial_cond_src = static_cast<InitialConditionSource>(0); ///< Initial condition source:
        AlignmentSelector auto_heading_alignment_selector; ///< Bitfield specifying the allowed automatic heading alignment methods for automatic initial conditions. Bits are set to 1 to enable, and the correspond to the following: <br/>
        float initial_heading = 0; ///< User-specified initial platform heading (degrees).
        float initial_pitch = 0; ///< User-specified initial platform pitch (degrees)
        float initial_roll = 0; ///< User-specified initial platform roll (degrees)
        Vector3f initial_position; ///< User-specified initial platform position (units determined by reference frame selector, see note.)
        Vector3f initial_velocity; ///< User-specified initial platform velocity (units determined by reference frame selector, see note.)
        FilterReferenceFrame reference_frame_selector = static_cast<FilterReferenceFrame>(0); ///< User-specified initial position/velocity reference frames
        
        
        auto as_tuple()
        {
            return std::make_tuple(std::ref(wait_for_run_command),std::ref(initial_cond_src),std::ref(auto_heading_alignment_selector),std::ref(initial_heading),std::ref(initial_pitch),std::ref(initial_roll),std::ref(initial_position),std::ref(initial_velocity),std::ref(reference_frame_selector));
        }
    };
};
void insert(Serializer& serializer, const InitializationConfiguration& self);
void extract(Serializer& serializer, InitializationConfiguration& self);

void insert(Serializer& serializer, const InitializationConfiguration::Response& self);
void extract(Serializer& serializer, InitializationConfiguration::Response& self);

TypedResult<InitializationConfiguration> writeInitializationConfiguration(C::mip_interface& device, uint8_t waitForRunCommand, InitializationConfiguration::InitialConditionSource initialCondSrc, InitializationConfiguration::AlignmentSelector autoHeadingAlignmentSelector, float initialHeading, float initialPitch, float initialRoll, const float* initialPosition, const float* initialVelocity, FilterReferenceFrame referenceFrameSelector);
TypedResult<InitializationConfiguration> readInitializationConfiguration(C::mip_interface& device, uint8_t* waitForRunCommandOut, InitializationConfiguration::InitialConditionSource* initialCondSrcOut, InitializationConfiguration::AlignmentSelector* autoHeadingAlignmentSelectorOut, float* initialHeadingOut, float* initialPitchOut, float* initialRollOut, float* initialPositionOut, float* initialVelocityOut, FilterReferenceFrame* referenceFrameSelectorOut);
TypedResult<InitializationConfiguration> saveInitializationConfiguration(C::mip_interface& device);
TypedResult<InitializationConfiguration> loadInitializationConfiguration(C::mip_interface& device);
TypedResult<InitializationConfiguration> defaultInitializationConfiguration(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_adaptive_filter_options  (0x0D,0x53) Adaptive Filter Options [CPP]
/// Configures the basic setup for auto-adaptive filtering. See product manual for a detailed description of this feature.
///
///@{

struct AdaptiveFilterOptions
{
    FunctionSelector function = static_cast<FunctionSelector>(0);
    uint8_t level = 0; ///< Auto-adaptive operating level: <br/> 0=Off, <br/> 1=Conservative, <br/> 2=Moderate (default), <br/> 3=Aggressive.
    uint16_t time_limit = 0; ///< Maximum duration of measurement rejection before entering recovery mode    (ms)
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_ADAPTIVE_FILTER_OPTIONS;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "AdaptiveFilterOptions";
    static constexpr const char* DOC_NAME = "Adaptive Filter Control";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    static constexpr const uint32_t WRITE_PARAMS   = 0x8003;
    static constexpr const uint32_t READ_PARAMS    = 0x8000;
    static constexpr const uint32_t SAVE_PARAMS    = 0x8000;
    static constexpr const uint32_t LOAD_PARAMS    = 0x8000;
    static constexpr const uint32_t DEFAULT_PARAMS = 0x8000;
    static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(level,time_limit);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(level),std::ref(time_limit));
    }
    
    static AdaptiveFilterOptions create_sld_all(::mip::FunctionSelector function)
    {
        AdaptiveFilterOptions cmd;
        cmd.function = function;
        return cmd;
    }
    
    struct Response
    {
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_ADAPTIVE_FILTER_OPTIONS;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "AdaptiveFilterOptions::Response";
        static constexpr const char* DOC_NAME = "Adaptive Filter Control Response";
        
        static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
        static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
        uint8_t level = 0; ///< Auto-adaptive operating level: <br/> 0=Off, <br/> 1=Conservative, <br/> 2=Moderate (default), <br/> 3=Aggressive.
        uint16_t time_limit = 0; ///< Maximum duration of measurement rejection before entering recovery mode    (ms)
        
        
        auto as_tuple()
        {
            return std::make_tuple(std::ref(level),std::ref(time_limit));
        }
    };
};
void insert(Serializer& serializer, const AdaptiveFilterOptions& self);
void extract(Serializer& serializer, AdaptiveFilterOptions& self);

void insert(Serializer& serializer, const AdaptiveFilterOptions::Response& self);
void extract(Serializer& serializer, AdaptiveFilterOptions::Response& self);

TypedResult<AdaptiveFilterOptions> writeAdaptiveFilterOptions(C::mip_interface& device, uint8_t level, uint16_t timeLimit);
TypedResult<AdaptiveFilterOptions> readAdaptiveFilterOptions(C::mip_interface& device, uint8_t* levelOut, uint16_t* timeLimitOut);
TypedResult<AdaptiveFilterOptions> saveAdaptiveFilterOptions(C::mip_interface& device);
TypedResult<AdaptiveFilterOptions> loadAdaptiveFilterOptions(C::mip_interface& device);
TypedResult<AdaptiveFilterOptions> defaultAdaptiveFilterOptions(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_multi_antenna_offset  (0x0D,0x54) Multi Antenna Offset [CPP]
/// Set the antenna lever arm.
/// 
/// This command works with devices that utilize multiple antennas.
/// <br/><br/><b>Offset Limit</b>: 10 m magnitude (default)
///
///@{

struct MultiAntennaOffset
{
    FunctionSelector function = static_cast<FunctionSelector>(0);
    uint8_t receiver_id = 0; ///< Receiver: 1, 2, etc...
    Vector3f antenna_offset; ///< Antenna lever arm offset vector in the vehicle frame (m)
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_MULTI_ANTENNA_OFFSET;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "MultiAntennaOffset";
    static constexpr const char* DOC_NAME = "GNSS Multi-Antenna Offset Control";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    static constexpr const uint32_t WRITE_PARAMS   = 0x8003;
    static constexpr const uint32_t READ_PARAMS    = 0x8001;
    static constexpr const uint32_t SAVE_PARAMS    = 0x8001;
    static constexpr const uint32_t LOAD_PARAMS    = 0x8001;
    static constexpr const uint32_t DEFAULT_PARAMS = 0x8001;
    static constexpr const uint32_t ECHOED_PARAMS  = 0x0001;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(receiver_id,antenna_offset);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(receiver_id),std::ref(antenna_offset));
    }
    
    static MultiAntennaOffset create_sld_all(::mip::FunctionSelector function)
    {
        MultiAntennaOffset cmd;
        cmd.function = function;
        cmd.receiver_id = 0;
        return cmd;
    }
    
    struct Response
    {
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_MULTI_ANTENNA_OFFSET;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "MultiAntennaOffset::Response";
        static constexpr const char* DOC_NAME = "GNSS Multi-Antenna Offset Control Response";
        
        static constexpr const uint32_t ECHOED_PARAMS  = 0x0001;
        static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
        uint8_t receiver_id = 0;
        Vector3f antenna_offset;
        
        
        auto as_tuple()
        {
            return std::make_tuple(std::ref(receiver_id),std::ref(antenna_offset));
        }
    };
};
void insert(Serializer& serializer, const MultiAntennaOffset& self);
void extract(Serializer& serializer, MultiAntennaOffset& self);

void insert(Serializer& serializer, const MultiAntennaOffset::Response& self);
void extract(Serializer& serializer, MultiAntennaOffset::Response& self);

TypedResult<MultiAntennaOffset> writeMultiAntennaOffset(C::mip_interface& device, uint8_t receiverId, const float* antennaOffset);
TypedResult<MultiAntennaOffset> readMultiAntennaOffset(C::mip_interface& device, uint8_t receiverId, float* antennaOffsetOut);
TypedResult<MultiAntennaOffset> saveMultiAntennaOffset(C::mip_interface& device, uint8_t receiverId);
TypedResult<MultiAntennaOffset> loadMultiAntennaOffset(C::mip_interface& device, uint8_t receiverId);
TypedResult<MultiAntennaOffset> defaultMultiAntennaOffset(C::mip_interface& device, uint8_t receiverId);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_rel_pos_configuration  (0x0D,0x55) Rel Pos Configuration [CPP]
/// Configure the reference location for filter relative positioning outputs
///
///@{

struct RelPosConfiguration
{
    FunctionSelector function = static_cast<FunctionSelector>(0);
    uint8_t source = 0; ///< 0 - auto (RTK base station), 1 - manual
    FilterReferenceFrame reference_frame_selector = static_cast<FilterReferenceFrame>(0); ///< ECEF or LLH
    Vector3d reference_coordinates; ///< reference coordinates, units determined by source selection
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_REL_POS_CONFIGURATION;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "RelPosConfiguration";
    static constexpr const char* DOC_NAME = "Relative Position Configuration";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    static constexpr const uint32_t WRITE_PARAMS   = 0x8007;
    static constexpr const uint32_t READ_PARAMS    = 0x8000;
    static constexpr const uint32_t SAVE_PARAMS    = 0x8000;
    static constexpr const uint32_t LOAD_PARAMS    = 0x8000;
    static constexpr const uint32_t DEFAULT_PARAMS = 0x8000;
    static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(source,reference_frame_selector,reference_coordinates);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(source),std::ref(reference_frame_selector),std::ref(reference_coordinates));
    }
    
    static RelPosConfiguration create_sld_all(::mip::FunctionSelector function)
    {
        RelPosConfiguration cmd;
        cmd.function = function;
        return cmd;
    }
    
    struct Response
    {
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_REL_POS_CONFIGURATION;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "RelPosConfiguration::Response";
        static constexpr const char* DOC_NAME = "Relative Position Configuration Response";
        
        static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
        static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
        uint8_t source = 0; ///< 0 - auto (RTK base station), 1 - manual
        FilterReferenceFrame reference_frame_selector = static_cast<FilterReferenceFrame>(0); ///< ECEF or LLH
        Vector3d reference_coordinates; ///< reference coordinates, units determined by source selection
        
        
        auto as_tuple()
        {
            return std::make_tuple(std::ref(source),std::ref(reference_frame_selector),std::ref(reference_coordinates));
        }
    };
};
void insert(Serializer& serializer, const RelPosConfiguration& self);
void extract(Serializer& serializer, RelPosConfiguration& self);

void insert(Serializer& serializer, const RelPosConfiguration::Response& self);
void extract(Serializer& serializer, RelPosConfiguration::Response& self);

TypedResult<RelPosConfiguration> writeRelPosConfiguration(C::mip_interface& device, uint8_t source, FilterReferenceFrame referenceFrameSelector, const double* referenceCoordinates);
TypedResult<RelPosConfiguration> readRelPosConfiguration(C::mip_interface& device, uint8_t* sourceOut, FilterReferenceFrame* referenceFrameSelectorOut, double* referenceCoordinatesOut);
TypedResult<RelPosConfiguration> saveRelPosConfiguration(C::mip_interface& device);
TypedResult<RelPosConfiguration> loadRelPosConfiguration(C::mip_interface& device);
TypedResult<RelPosConfiguration> defaultRelPosConfiguration(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_ref_point_lever_arm  (0x0D,0x56) Ref Point Lever Arm [CPP]
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

struct RefPointLeverArm
{
    enum class ReferencePointSelector : uint8_t
    {
        VEH = 1,  ///<  Defines the origin of the vehicle
    };
    
    FunctionSelector function = static_cast<FunctionSelector>(0);
    ReferencePointSelector ref_point_sel = static_cast<ReferencePointSelector>(0); ///< Reserved, must be 1
    Vector3f lever_arm_offset; ///< [m] Lever arm offset vector in the vehicle's reference frame.
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_REF_POINT_LEVER_ARM;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "RefPointLeverArm";
    static constexpr const char* DOC_NAME = "Reference point lever arm";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    static constexpr const uint32_t WRITE_PARAMS   = 0x8003;
    static constexpr const uint32_t READ_PARAMS    = 0x8000;
    static constexpr const uint32_t SAVE_PARAMS    = 0x8000;
    static constexpr const uint32_t LOAD_PARAMS    = 0x8000;
    static constexpr const uint32_t DEFAULT_PARAMS = 0x8000;
    static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(ref_point_sel,lever_arm_offset);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(ref_point_sel),std::ref(lever_arm_offset));
    }
    
    static RefPointLeverArm create_sld_all(::mip::FunctionSelector function)
    {
        RefPointLeverArm cmd;
        cmd.function = function;
        return cmd;
    }
    
    struct Response
    {
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_REF_POINT_LEVER_ARM;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "RefPointLeverArm::Response";
        static constexpr const char* DOC_NAME = "Reference point lever arm Response";
        
        static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
        static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
        ReferencePointSelector ref_point_sel = static_cast<ReferencePointSelector>(0); ///< Reserved, must be 1
        Vector3f lever_arm_offset; ///< [m] Lever arm offset vector in the vehicle's reference frame.
        
        
        auto as_tuple()
        {
            return std::make_tuple(std::ref(ref_point_sel),std::ref(lever_arm_offset));
        }
    };
};
void insert(Serializer& serializer, const RefPointLeverArm& self);
void extract(Serializer& serializer, RefPointLeverArm& self);

void insert(Serializer& serializer, const RefPointLeverArm::Response& self);
void extract(Serializer& serializer, RefPointLeverArm::Response& self);

TypedResult<RefPointLeverArm> writeRefPointLeverArm(C::mip_interface& device, RefPointLeverArm::ReferencePointSelector refPointSel, const float* leverArmOffset);
TypedResult<RefPointLeverArm> readRefPointLeverArm(C::mip_interface& device, RefPointLeverArm::ReferencePointSelector* refPointSelOut, float* leverArmOffsetOut);
TypedResult<RefPointLeverArm> saveRefPointLeverArm(C::mip_interface& device);
TypedResult<RefPointLeverArm> loadRefPointLeverArm(C::mip_interface& device);
TypedResult<RefPointLeverArm> defaultRefPointLeverArm(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_speed_measurement  (0x0D,0x60) Speed Measurement [CPP]
/// Speed aiding measurement, where speed is defined as rate of motion along the vehicle's x-axis direction.
/// Can be used by an external odometer/speedometer, for example.
/// This command cannot be used if the internal odometer is configured.
///
///@{

struct SpeedMeasurement
{
    uint8_t source = 0; ///< Reserved, must be 1.
    float time_of_week = 0; ///< GPS time of week when speed was sampled
    float speed = 0; ///< Estimated speed along vehicle's x-axis (may be positive or negative) [meters/second]
    float speed_uncertainty = 0; ///< Estimated uncertainty in the speed measurement (1-sigma value) [meters/second]
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_SPEED_MEASUREMENT;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "SpeedMeasurement";
    static constexpr const char* DOC_NAME = "Input speed measurement";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(source,time_of_week,speed,speed_uncertainty);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(source),std::ref(time_of_week),std::ref(speed),std::ref(speed_uncertainty));
    }
    typedef void Response;
};
void insert(Serializer& serializer, const SpeedMeasurement& self);
void extract(Serializer& serializer, SpeedMeasurement& self);

TypedResult<SpeedMeasurement> speedMeasurement(C::mip_interface& device, uint8_t source, float timeOfWeek, float speed, float speedUncertainty);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_speed_lever_arm  (0x0D,0x61) Speed Lever Arm [CPP]
/// Lever arm offset for speed measurements.
/// This is used to compensate for an off-center measurement point
/// having a different speed due to rotation of the vehicle.
/// The typical use case for this would be an odometer attached to a wheel
/// on a standard 4-wheeled vehicle. If the odometer is on the left wheel,
/// it will report higher speed on right turns and lower speed on left turns.
/// This is because the outside edge of the curve is longer than the inside edge.
///
///@{

struct SpeedLeverArm
{
    FunctionSelector function = static_cast<FunctionSelector>(0);
    uint8_t source = 0; ///< Reserved, must be 1.
    Vector3f lever_arm_offset; ///< [m] Lever arm offset vector in the vehicle's reference frame.
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_SPEED_LEVER_ARM;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "SpeedLeverArm";
    static constexpr const char* DOC_NAME = "Measurement speed lever arm";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    static constexpr const uint32_t WRITE_PARAMS   = 0x8003;
    static constexpr const uint32_t READ_PARAMS    = 0x8001;
    static constexpr const uint32_t SAVE_PARAMS    = 0x8001;
    static constexpr const uint32_t LOAD_PARAMS    = 0x8001;
    static constexpr const uint32_t DEFAULT_PARAMS = 0x8001;
    static constexpr const uint32_t ECHOED_PARAMS  = 0x0001;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(source,lever_arm_offset);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(source),std::ref(lever_arm_offset));
    }
    
    static SpeedLeverArm create_sld_all(::mip::FunctionSelector function)
    {
        SpeedLeverArm cmd;
        cmd.function = function;
        cmd.source = 0;
        return cmd;
    }
    
    struct Response
    {
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_SPEED_LEVER_ARM;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "SpeedLeverArm::Response";
        static constexpr const char* DOC_NAME = "Measurement speed lever arm Response";
        
        static constexpr const uint32_t ECHOED_PARAMS  = 0x0001;
        static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
        uint8_t source = 0; ///< Reserved, must be 1.
        Vector3f lever_arm_offset; ///< [m] Lever arm offset vector in the vehicle's reference frame.
        
        
        auto as_tuple()
        {
            return std::make_tuple(std::ref(source),std::ref(lever_arm_offset));
        }
    };
};
void insert(Serializer& serializer, const SpeedLeverArm& self);
void extract(Serializer& serializer, SpeedLeverArm& self);

void insert(Serializer& serializer, const SpeedLeverArm::Response& self);
void extract(Serializer& serializer, SpeedLeverArm::Response& self);

TypedResult<SpeedLeverArm> writeSpeedLeverArm(C::mip_interface& device, uint8_t source, const float* leverArmOffset);
TypedResult<SpeedLeverArm> readSpeedLeverArm(C::mip_interface& device, uint8_t source, float* leverArmOffsetOut);
TypedResult<SpeedLeverArm> saveSpeedLeverArm(C::mip_interface& device, uint8_t source);
TypedResult<SpeedLeverArm> loadSpeedLeverArm(C::mip_interface& device, uint8_t source);
TypedResult<SpeedLeverArm> defaultSpeedLeverArm(C::mip_interface& device, uint8_t source);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_wheeled_vehicle_constraint_control  (0x0D,0x63) Wheeled Vehicle Constraint Control [CPP]
/// Configure the wheeled vehicle kinematic constraint.
/// 
/// When enabled, the filter uses the assumption that velocity is constrained to the primary vehicle axis.
/// By convention, the primary vehicle axis is the vehicle X-axis (note: the sensor may be physically installed in
/// any orientation on the vehicle if the appropriate mounting transformation has been specified).
/// This constraint will typically improve heading estimates for vehicles where the assumption is valid, such
/// as an automobile, particularly when GNSS coverage is intermittent.
///
///@{

struct WheeledVehicleConstraintControl
{
    FunctionSelector function = static_cast<FunctionSelector>(0);
    uint8_t enable = 0; ///< 0 - Disable, 1 - Enable
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_VEHICLE_CONSTRAINT_CONTROL;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "WheeledVehicleConstraintControl";
    static constexpr const char* DOC_NAME = "Wheeled Vehicle Constraint Control";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    static constexpr const uint32_t WRITE_PARAMS   = 0x8001;
    static constexpr const uint32_t READ_PARAMS    = 0x8000;
    static constexpr const uint32_t SAVE_PARAMS    = 0x8000;
    static constexpr const uint32_t LOAD_PARAMS    = 0x8000;
    static constexpr const uint32_t DEFAULT_PARAMS = 0x8000;
    static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(enable);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(enable));
    }
    
    static WheeledVehicleConstraintControl create_sld_all(::mip::FunctionSelector function)
    {
        WheeledVehicleConstraintControl cmd;
        cmd.function = function;
        return cmd;
    }
    
    struct Response
    {
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_VEHICLE_CONSTRAINT_CONTROL;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "WheeledVehicleConstraintControl::Response";
        static constexpr const char* DOC_NAME = "Wheeled Vehicle Constraint Control Response";
        
        static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
        static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
        uint8_t enable = 0; ///< 0 - Disable, 1 - Enable
        
        
        auto as_tuple()
        {
            return std::make_tuple(std::ref(enable));
        }
    };
};
void insert(Serializer& serializer, const WheeledVehicleConstraintControl& self);
void extract(Serializer& serializer, WheeledVehicleConstraintControl& self);

void insert(Serializer& serializer, const WheeledVehicleConstraintControl::Response& self);
void extract(Serializer& serializer, WheeledVehicleConstraintControl::Response& self);

TypedResult<WheeledVehicleConstraintControl> writeWheeledVehicleConstraintControl(C::mip_interface& device, uint8_t enable);
TypedResult<WheeledVehicleConstraintControl> readWheeledVehicleConstraintControl(C::mip_interface& device, uint8_t* enableOut);
TypedResult<WheeledVehicleConstraintControl> saveWheeledVehicleConstraintControl(C::mip_interface& device);
TypedResult<WheeledVehicleConstraintControl> loadWheeledVehicleConstraintControl(C::mip_interface& device);
TypedResult<WheeledVehicleConstraintControl> defaultWheeledVehicleConstraintControl(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_vertical_gyro_constraint_control  (0x0D,0x62) Vertical Gyro Constraint Control [CPP]
/// Configure the vertical gyro kinematic constraint.
/// 
/// When enabled and no valid GNSS measurements are available, the filter uses the accelerometers to track pitch
/// and roll under the assumption that the sensor platform is not undergoing linear acceleration.
/// This constraint is useful to maintain accurate pitch and roll during GNSS signal outages.
///
///@{

struct VerticalGyroConstraintControl
{
    FunctionSelector function = static_cast<FunctionSelector>(0);
    uint8_t enable = 0; ///< 0 - Disable, 1 - Enable
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_GYRO_CONSTRAINT_CONTROL;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "VerticalGyroConstraintControl";
    static constexpr const char* DOC_NAME = "Vertical Gyro Constraint Control";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    static constexpr const uint32_t WRITE_PARAMS   = 0x8001;
    static constexpr const uint32_t READ_PARAMS    = 0x8000;
    static constexpr const uint32_t SAVE_PARAMS    = 0x8000;
    static constexpr const uint32_t LOAD_PARAMS    = 0x8000;
    static constexpr const uint32_t DEFAULT_PARAMS = 0x8000;
    static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(enable);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(enable));
    }
    
    static VerticalGyroConstraintControl create_sld_all(::mip::FunctionSelector function)
    {
        VerticalGyroConstraintControl cmd;
        cmd.function = function;
        return cmd;
    }
    
    struct Response
    {
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_GYRO_CONSTRAINT_CONTROL;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "VerticalGyroConstraintControl::Response";
        static constexpr const char* DOC_NAME = "Vertical Gyro Constraint Control Response";
        
        static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
        static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
        uint8_t enable = 0; ///< 0 - Disable, 1 - Enable
        
        
        auto as_tuple()
        {
            return std::make_tuple(std::ref(enable));
        }
    };
};
void insert(Serializer& serializer, const VerticalGyroConstraintControl& self);
void extract(Serializer& serializer, VerticalGyroConstraintControl& self);

void insert(Serializer& serializer, const VerticalGyroConstraintControl::Response& self);
void extract(Serializer& serializer, VerticalGyroConstraintControl::Response& self);

TypedResult<VerticalGyroConstraintControl> writeVerticalGyroConstraintControl(C::mip_interface& device, uint8_t enable);
TypedResult<VerticalGyroConstraintControl> readVerticalGyroConstraintControl(C::mip_interface& device, uint8_t* enableOut);
TypedResult<VerticalGyroConstraintControl> saveVerticalGyroConstraintControl(C::mip_interface& device);
TypedResult<VerticalGyroConstraintControl> loadVerticalGyroConstraintControl(C::mip_interface& device);
TypedResult<VerticalGyroConstraintControl> defaultVerticalGyroConstraintControl(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_gnss_antenna_cal_control  (0x0D,0x64) Gnss Antenna Cal Control [CPP]
/// Configure the GNSS antenna lever arm calibration.
/// 
/// When enabled, the filter will enable lever arm error tracking, up to the maximum offset specified.
///
///@{

struct GnssAntennaCalControl
{
    FunctionSelector function = static_cast<FunctionSelector>(0);
    uint8_t enable = 0; ///< 0 - Disable, 1 - Enable
    float max_offset = 0; ///< Maximum absolute value of lever arm offset error in the vehicle frame [meters]. See device user manual for the valid range of this parameter.
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_ANTENNA_CALIBRATION_CONTROL;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "GnssAntennaCalControl";
    static constexpr const char* DOC_NAME = "GNSS Antenna Offset Calibration Control";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    static constexpr const uint32_t WRITE_PARAMS   = 0x8003;
    static constexpr const uint32_t READ_PARAMS    = 0x8000;
    static constexpr const uint32_t SAVE_PARAMS    = 0x8000;
    static constexpr const uint32_t LOAD_PARAMS    = 0x8000;
    static constexpr const uint32_t DEFAULT_PARAMS = 0x8000;
    static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(enable,max_offset);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(enable),std::ref(max_offset));
    }
    
    static GnssAntennaCalControl create_sld_all(::mip::FunctionSelector function)
    {
        GnssAntennaCalControl cmd;
        cmd.function = function;
        return cmd;
    }
    
    struct Response
    {
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::REPLY_ANTENNA_CALIBRATION_CONTROL;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "GnssAntennaCalControl::Response";
        static constexpr const char* DOC_NAME = "GNSS Antenna Offset Calibration Control Response";
        
        static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
        static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
        uint8_t enable = 0; ///< 0 - Disable, 1 - Enable
        float max_offset = 0; ///< Maximum absolute value of lever arm offset error in the vehicle frame [meters]. See device user manual for the valid range of this parameter.
        
        
        auto as_tuple()
        {
            return std::make_tuple(std::ref(enable),std::ref(max_offset));
        }
    };
};
void insert(Serializer& serializer, const GnssAntennaCalControl& self);
void extract(Serializer& serializer, GnssAntennaCalControl& self);

void insert(Serializer& serializer, const GnssAntennaCalControl::Response& self);
void extract(Serializer& serializer, GnssAntennaCalControl::Response& self);

TypedResult<GnssAntennaCalControl> writeGnssAntennaCalControl(C::mip_interface& device, uint8_t enable, float maxOffset);
TypedResult<GnssAntennaCalControl> readGnssAntennaCalControl(C::mip_interface& device, uint8_t* enableOut, float* maxOffsetOut);
TypedResult<GnssAntennaCalControl> saveGnssAntennaCalControl(C::mip_interface& device);
TypedResult<GnssAntennaCalControl> loadGnssAntennaCalControl(C::mip_interface& device);
TypedResult<GnssAntennaCalControl> defaultGnssAntennaCalControl(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_filter_set_initial_heading  (0x0D,0x03) Set Initial Heading [CPP]
/// Set the initial heading angle.
/// 
/// The estimation filter will reset the heading estimate to provided value. If the product supports magnetometer aiding and this feature has been enabled, the heading
/// argument will be ignored and the filter will initialize using the inferred magnetic heading.
///
///@{

struct SetInitialHeading
{
    float heading = 0; ///< Initial heading in radians [-pi, pi]
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_filter::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_filter::CMD_SET_INITIAL_HEADING;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "SetInitialHeading";
    static constexpr const char* DOC_NAME = "Set Initial Heading Control";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(heading);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(heading));
    }
    typedef void Response;
};
void insert(Serializer& serializer, const SetInitialHeading& self);
void extract(Serializer& serializer, SetInitialHeading& self);

TypedResult<SetInitialHeading> setInitialHeading(C::mip_interface& device, float heading);

///@}
///

///@}
///@}
///
////////////////////////////////////////////////////////////////////////////////
} // namespace commands_filter
} // namespace mip


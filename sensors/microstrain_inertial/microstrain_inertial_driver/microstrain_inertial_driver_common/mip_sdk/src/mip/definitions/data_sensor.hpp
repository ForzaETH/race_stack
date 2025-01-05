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

namespace data_sensor {

////////////////////////////////////////////////////////////////////////////////
///@addtogroup MipData_cpp  MIP Data [CPP]
///@{
///@defgroup sensor_data_cpp  Sensor Data [CPP]
///
///@{

////////////////////////////////////////////////////////////////////////////////
// Descriptors
////////////////////////////////////////////////////////////////////////////////

enum 
{
    DESCRIPTOR_SET                      = 0x80,
    
    DATA_ACCEL_RAW                      = 0x01,
    DATA_GYRO_RAW                       = 0x02,
    DATA_MAG_RAW                        = 0x03,
    DATA_ACCEL_SCALED                   = 0x04,
    DATA_GYRO_SCALED                    = 0x05,
    DATA_MAG_SCALED                     = 0x06,
    DATA_DELTA_THETA                    = 0x07,
    DATA_DELTA_VELOCITY                 = 0x08,
    DATA_COMP_ORIENTATION_MATRIX        = 0x09,
    DATA_COMP_QUATERNION                = 0x0A,
    DATA_COMP_ORIENTATION_UPDATE_MATRIX = 0x0B,
    DATA_COMP_EULER_ANGLES              = 0x0C,
    DATA_TEMPERATURE_RAW                = 0x0D,
    DATA_TIME_STAMP_INTERNAL            = 0x0E,
    DATA_TIME_STAMP_PPS                 = 0x0F,
    DATA_STAB_MAG                       = 0x10,
    DATA_STAB_ACCEL                     = 0x11,
    DATA_TIME_STAMP_GPS                 = 0x12,
    DATA_TEMPERATURE_ABS                = 0x14,
    DATA_RAW_CLIP_DATA                  = 0x15,
    DATA_PRESSURE_RAW                   = 0x16,
    DATA_PRESSURE_SCALED                = 0x17,
    DATA_OVERRANGE_STATUS               = 0x18,
    DATA_ODOMETER                       = 0x40,
    
    MIP_DATA_DESC_ASPP                  = 0x81,
    MIP_DATA_DESC_GXSB                  = 0x82,
};

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_sensor_raw_accel  (0x80,0x01) Raw Accel [CPP]
/// Three element vector representing the sensed acceleration.
/// This quantity is temperature compensated and expressed in the sensor body frame.
///
///@{

struct RawAccel
{
    Vector3f raw_accel; ///< Native sensor counts
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_ACCEL_RAW;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "RawAccel";
    static constexpr const char* DOC_NAME = "RawAccel";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(raw_accel[0],raw_accel[1],raw_accel[2]);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(raw_accel[0]),std::ref(raw_accel[1]),std::ref(raw_accel[2]));
    }
};
void insert(Serializer& serializer, const RawAccel& self);
void extract(Serializer& serializer, RawAccel& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_sensor_raw_gyro  (0x80,0x02) Raw Gyro [CPP]
/// Three element vector representing the sensed angular rate.
/// This quantity is temperature compensated and expressed in the sensor body frame.
///
///@{

struct RawGyro
{
    Vector3f raw_gyro; ///< Native sensor counts
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_GYRO_RAW;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "RawGyro";
    static constexpr const char* DOC_NAME = "RawGyro";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(raw_gyro[0],raw_gyro[1],raw_gyro[2]);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(raw_gyro[0]),std::ref(raw_gyro[1]),std::ref(raw_gyro[2]));
    }
};
void insert(Serializer& serializer, const RawGyro& self);
void extract(Serializer& serializer, RawGyro& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_sensor_raw_mag  (0x80,0x03) Raw Mag [CPP]
/// Three element vector representing the sensed magnetic field.
/// This quantity is temperature compensated and expressed in the vehicle frame.
///
///@{

struct RawMag
{
    Vector3f raw_mag; ///< Native sensor counts
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_MAG_RAW;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "RawMag";
    static constexpr const char* DOC_NAME = "RawMag";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(raw_mag[0],raw_mag[1],raw_mag[2]);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(raw_mag[0]),std::ref(raw_mag[1]),std::ref(raw_mag[2]));
    }
};
void insert(Serializer& serializer, const RawMag& self);
void extract(Serializer& serializer, RawMag& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_sensor_raw_pressure  (0x80,0x16) Raw Pressure [CPP]
/// Scalar value representing the sensed ambient pressure.
/// This quantity is temperature compensated.
///
///@{

struct RawPressure
{
    float raw_pressure = 0; ///< Native sensor counts
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_PRESSURE_RAW;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "RawPressure";
    static constexpr const char* DOC_NAME = "RawPressure";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(raw_pressure);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(raw_pressure));
    }
};
void insert(Serializer& serializer, const RawPressure& self);
void extract(Serializer& serializer, RawPressure& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_sensor_scaled_accel  (0x80,0x04) Scaled Accel [CPP]
/// 3-element vector representing the sensed acceleration.
/// This quantity is temperature compensated and expressed in the vehicle frame.
///
///@{

struct ScaledAccel
{
    Vector3f scaled_accel; ///< (x, y, z)[g]
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_ACCEL_SCALED;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "ScaledAccel";
    static constexpr const char* DOC_NAME = "ScaledAccel";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(scaled_accel[0],scaled_accel[1],scaled_accel[2]);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(scaled_accel[0]),std::ref(scaled_accel[1]),std::ref(scaled_accel[2]));
    }
};
void insert(Serializer& serializer, const ScaledAccel& self);
void extract(Serializer& serializer, ScaledAccel& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_sensor_scaled_gyro  (0x80,0x05) Scaled Gyro [CPP]
/// 3-element vector representing the sensed angular rate.
/// This quantity is temperature compensated and expressed in the vehicle frame.
///
///@{

struct ScaledGyro
{
    Vector3f scaled_gyro; ///< (x, y, z) [radians/second]
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_GYRO_SCALED;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "ScaledGyro";
    static constexpr const char* DOC_NAME = "ScaledGyro";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(scaled_gyro[0],scaled_gyro[1],scaled_gyro[2]);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(scaled_gyro[0]),std::ref(scaled_gyro[1]),std::ref(scaled_gyro[2]));
    }
};
void insert(Serializer& serializer, const ScaledGyro& self);
void extract(Serializer& serializer, ScaledGyro& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_sensor_scaled_mag  (0x80,0x06) Scaled Mag [CPP]
/// 3-element vector representing the sensed magnetic field.
/// This quantity is temperature compensated and expressed in the vehicle frame.
///
///@{

struct ScaledMag
{
    Vector3f scaled_mag; ///< (x, y, z) [Gauss]
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_MAG_SCALED;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "ScaledMag";
    static constexpr const char* DOC_NAME = "ScaledMag";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(scaled_mag[0],scaled_mag[1],scaled_mag[2]);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(scaled_mag[0]),std::ref(scaled_mag[1]),std::ref(scaled_mag[2]));
    }
};
void insert(Serializer& serializer, const ScaledMag& self);
void extract(Serializer& serializer, ScaledMag& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_sensor_scaled_pressure  (0x80,0x17) Scaled Pressure [CPP]
/// Scalar value representing the sensed ambient pressure.
///
///@{

struct ScaledPressure
{
    float scaled_pressure = 0; ///< [mBar]
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_PRESSURE_SCALED;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "ScaledPressure";
    static constexpr const char* DOC_NAME = "ScaledPressure";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(scaled_pressure);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(scaled_pressure));
    }
};
void insert(Serializer& serializer, const ScaledPressure& self);
void extract(Serializer& serializer, ScaledPressure& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_sensor_delta_theta  (0x80,0x07) Delta Theta [CPP]
/// 3-element vector representing the time integral of angular rate.
/// This quantity is the integral of sensed angular rate over the period set by the IMU message format.  It is expressed in the vehicle frame.
///
///@{

struct DeltaTheta
{
    Vector3f delta_theta; ///< (x, y, z) [radians]
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_DELTA_THETA;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "DeltaTheta";
    static constexpr const char* DOC_NAME = "DeltaTheta";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(delta_theta[0],delta_theta[1],delta_theta[2]);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(delta_theta[0]),std::ref(delta_theta[1]),std::ref(delta_theta[2]));
    }
};
void insert(Serializer& serializer, const DeltaTheta& self);
void extract(Serializer& serializer, DeltaTheta& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_sensor_delta_velocity  (0x80,0x08) Delta Velocity [CPP]
/// 3-element vector representing the time integral of acceleration.
/// This quantity is the integral of sensed acceleration over the period set by the IMU message format.  It is expressed in the vehicle frame.
///
///@{

struct DeltaVelocity
{
    Vector3f delta_velocity; ///< (x, y, z) [g*sec]
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_DELTA_VELOCITY;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "DeltaVelocity";
    static constexpr const char* DOC_NAME = "DeltaVelocity";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(delta_velocity[0],delta_velocity[1],delta_velocity[2]);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(delta_velocity[0]),std::ref(delta_velocity[1]),std::ref(delta_velocity[2]));
    }
};
void insert(Serializer& serializer, const DeltaVelocity& self);
void extract(Serializer& serializer, DeltaVelocity& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_sensor_comp_orientation_matrix  (0x80,0x09) Comp Orientation Matrix [CPP]
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
/// The matrix elements are stored is row-major order: EQSTART M = \begin{bmatrix} M_{11}, M_{12}, M_{13}, M_{21}, M_{22}, M_{23}, M_{31}, M_{32}, M_{33} \end{bmatrix} EQEND
///
///@{

struct CompOrientationMatrix
{
    Matrix3f m; ///< Matrix elements in row-major order.
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_COMP_ORIENTATION_MATRIX;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "CompOrientationMatrix";
    static constexpr const char* DOC_NAME = "Complementary Filter Orientation Matrix";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(m[0],m[1],m[2],m[3],m[4],m[5],m[6],m[7],m[8]);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(m[0]),std::ref(m[1]),std::ref(m[2]),std::ref(m[3]),std::ref(m[4]),std::ref(m[5]),std::ref(m[6]),std::ref(m[7]),std::ref(m[8]));
    }
};
void insert(Serializer& serializer, const CompOrientationMatrix& self);
void extract(Serializer& serializer, CompOrientationMatrix& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_sensor_comp_quaternion  (0x80,0x0A) Comp Quaternion [CPP]
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

struct CompQuaternion
{
    Quatf q; ///< Quaternion elements EQSTART q = (q_w, q_x, q_y, q_z) EQEND
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_COMP_QUATERNION;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "CompQuaternion";
    static constexpr const char* DOC_NAME = "Complementary Filter Quaternion";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(q[0],q[1],q[2],q[3]);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(q[0]),std::ref(q[1]),std::ref(q[2]),std::ref(q[3]));
    }
};
void insert(Serializer& serializer, const CompQuaternion& self);
void extract(Serializer& serializer, CompQuaternion& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_sensor_comp_euler_angles  (0x80,0x0C) Comp Euler Angles [CPP]
/// Euler angles describing the orientation of the device with respect to the NED local-level frame.
/// The Euler angles are reported in 3-2-1 (Yaw-Pitch-Roll, AKA Aircraft) order.
///
///@{

struct CompEulerAngles
{
    float roll = 0; ///< [radians]
    float pitch = 0; ///< [radians]
    float yaw = 0; ///< [radians]
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_COMP_EULER_ANGLES;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "CompEulerAngles";
    static constexpr const char* DOC_NAME = "Complementary Filter Euler Angles";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(roll,pitch,yaw);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(roll),std::ref(pitch),std::ref(yaw));
    }
};
void insert(Serializer& serializer, const CompEulerAngles& self);
void extract(Serializer& serializer, CompEulerAngles& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_sensor_comp_orientation_update_matrix  (0x80,0x0B) Comp Orientation Update Matrix [CPP]
/// DEPRECATED!
///
///@{

struct CompOrientationUpdateMatrix
{
    Matrix3f m;
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_COMP_ORIENTATION_UPDATE_MATRIX;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "CompOrientationUpdateMatrix";
    static constexpr const char* DOC_NAME = "Complementary Filter Orientation Update Matrix";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(m[0],m[1],m[2],m[3],m[4],m[5],m[6],m[7],m[8]);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(m[0]),std::ref(m[1]),std::ref(m[2]),std::ref(m[3]),std::ref(m[4]),std::ref(m[5]),std::ref(m[6]),std::ref(m[7]),std::ref(m[8]));
    }
};
void insert(Serializer& serializer, const CompOrientationUpdateMatrix& self);
void extract(Serializer& serializer, CompOrientationUpdateMatrix& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_sensor_orientation_raw_temp  (0x80,0x0D) Orientation Raw Temp [CPP]
/// DEPRECATED!
///
///@{

struct OrientationRawTemp
{
    uint16_t raw_temp[4] = {0};
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_TEMPERATURE_RAW;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "OrientationRawTemp";
    static constexpr const char* DOC_NAME = "OrientationRawTemp";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(raw_temp);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(raw_temp));
    }
};
void insert(Serializer& serializer, const OrientationRawTemp& self);
void extract(Serializer& serializer, OrientationRawTemp& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_sensor_internal_timestamp  (0x80,0x0E) Internal Timestamp [CPP]
/// DEPRECATED!
///
///@{

struct InternalTimestamp
{
    uint32_t counts = 0;
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_TIME_STAMP_INTERNAL;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "InternalTimestamp";
    static constexpr const char* DOC_NAME = "InternalTimestamp";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(counts);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(counts));
    }
};
void insert(Serializer& serializer, const InternalTimestamp& self);
void extract(Serializer& serializer, InternalTimestamp& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_sensor_pps_timestamp  (0x80,0x0F) Pps Timestamp [CPP]
/// DEPRECATED!
///
///@{

struct PpsTimestamp
{
    uint32_t seconds = 0;
    uint32_t useconds = 0;
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_TIME_STAMP_PPS;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "PpsTimestamp";
    static constexpr const char* DOC_NAME = "PPS Timestamp";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(seconds,useconds);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(seconds),std::ref(useconds));
    }
};
void insert(Serializer& serializer, const PpsTimestamp& self);
void extract(Serializer& serializer, PpsTimestamp& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_sensor_gps_timestamp  (0x80,0x12) Gps Timestamp [CPP]
/// GPS timestamp of the SENSOR data
/// 
/// Should the PPS become unavailable, the device will revert to its internal clock, which will cause the reported time to drift from true GPS time.
/// Upon recovering from a PPS outage, the user should expect a jump in the reported GPS time due to the accumulation of internal clock error.
/// If synchronization to an external clock or onboard GNSS receiver (for products that have one) is disabled, this time is equivalent to internal system time.
/// 
/// Note: this data field may be deprecated in the future. The more flexible shared data field (0x80, 0xD3) should be used instead.
///
///@{

struct GpsTimestamp
{
    struct ValidFlags : Bitfield<ValidFlags>
    {
        enum _enumType : uint16_t
        {
            NONE              = 0x0000,
            PPS_VALID         = 0x0001,  ///<  True when the PPS signal is present.
            TIME_REFRESH      = 0x0002,  ///<  Toggles each time the time is updated via internal GPS or the GPS Time Update command (0x01, 0x72).
            TIME_INITIALIZED  = 0x0004,  ///<  True if the time has ever been set.
            TOW_VALID         = 0x0008,  ///<  True if the time of week is valid.
            WEEK_NUMBER_VALID = 0x0010,  ///<  True if the week number is valid.
            ALL               = 0x001F,
        };
        uint16_t value = NONE;
        
        ValidFlags() : value(NONE) {}
        ValidFlags(int val) : value((uint16_t)val) {}
        operator uint16_t() const { return value; }
        ValidFlags& operator=(uint16_t val) { value = val; return *this; }
        ValidFlags& operator=(int val) { value = uint16_t(val); return *this; }
        ValidFlags& operator|=(uint16_t val) { return *this = value | val; }
        ValidFlags& operator&=(uint16_t val) { return *this = value & val; }
        
        bool ppsValid() const { return (value & PPS_VALID) > 0; }
        void ppsValid(bool val) { if(val) value |= PPS_VALID; else value &= ~PPS_VALID; }
        bool timeRefresh() const { return (value & TIME_REFRESH) > 0; }
        void timeRefresh(bool val) { if(val) value |= TIME_REFRESH; else value &= ~TIME_REFRESH; }
        bool timeInitialized() const { return (value & TIME_INITIALIZED) > 0; }
        void timeInitialized(bool val) { if(val) value |= TIME_INITIALIZED; else value &= ~TIME_INITIALIZED; }
        bool towValid() const { return (value & TOW_VALID) > 0; }
        void towValid(bool val) { if(val) value |= TOW_VALID; else value &= ~TOW_VALID; }
        bool weekNumberValid() const { return (value & WEEK_NUMBER_VALID) > 0; }
        void weekNumberValid(bool val) { if(val) value |= WEEK_NUMBER_VALID; else value &= ~WEEK_NUMBER_VALID; }
        
        bool allSet() const { return value == ALL; }
        void setAll() { value |= ALL; }
    };
    
    double tow = 0; ///< GPS Time of Week [seconds]
    uint16_t week_number = 0; ///< GPS Week Number since 1980 [weeks]
    ValidFlags valid_flags;
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_TIME_STAMP_GPS;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "GpsTimestamp";
    static constexpr const char* DOC_NAME = "GpsTimestamp";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(tow,week_number,valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(tow),std::ref(week_number),std::ref(valid_flags));
    }
};
void insert(Serializer& serializer, const GpsTimestamp& self);
void extract(Serializer& serializer, GpsTimestamp& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_sensor_temperature_abs  (0x80,0x14) Temperature Abs [CPP]
/// SENSOR reported temperature statistics
/// 
/// Temperature may originate from the MEMS sensors, or be calculated in combination with board temperature sensors.
/// All quantities are calculated with respect to the last power on or reset, whichever is later.
/// 
///
///@{

struct TemperatureAbs
{
    float min_temp = 0; ///< [degC]
    float max_temp = 0; ///< [degC]
    float mean_temp = 0; ///< [degC]
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_TEMPERATURE_ABS;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "TemperatureAbs";
    static constexpr const char* DOC_NAME = "Temperature Statistics";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(min_temp,max_temp,mean_temp);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(min_temp),std::ref(max_temp),std::ref(mean_temp));
    }
};
void insert(Serializer& serializer, const TemperatureAbs& self);
void extract(Serializer& serializer, TemperatureAbs& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_sensor_up_vector  (0x80,0x11) Up Vector [CPP]
/// Gyro-stabilized 3-element vector representing the complementary filter's estimated vertical direction.
/// This quantity is expressed in the vehicle frame.
/// 
/// This quantity is sensitive to non-gravitational accelerations, which may cause notable deviations from the true vertical direction.
/// 
/// For legacy reasons, this vector is the inverse of the gravity vector.
/// 
///
///@{

struct UpVector
{
    Vector3f up; ///< [Gs]
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_STAB_ACCEL;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "UpVector";
    static constexpr const char* DOC_NAME = "UpVector";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(up[0],up[1],up[2]);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(up[0]),std::ref(up[1]),std::ref(up[2]));
    }
};
void insert(Serializer& serializer, const UpVector& self);
void extract(Serializer& serializer, UpVector& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_sensor_north_vector  (0x80,0x10) North Vector [CPP]
/// Gyro-stabilized 3-element vector representing the complementary filter's estimate of magnetic north.
/// This quantity is expressed in the vehicle frame.
/// 
/// This quantity is sensitive to local magnetic field perturbations, which may cause notable deviations from true magnetic north.
///
///@{

struct NorthVector
{
    Vector3f north; ///< [Gauss]
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_STAB_MAG;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "NorthVector";
    static constexpr const char* DOC_NAME = "NorthVector";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(north[0],north[1],north[2]);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(north[0]),std::ref(north[1]),std::ref(north[2]));
    }
};
void insert(Serializer& serializer, const NorthVector& self);
void extract(Serializer& serializer, NorthVector& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_sensor_overrange_status  (0x80,0x18) Overrange Status [CPP]
///
///@{

struct OverrangeStatus
{
    struct Status : Bitfield<Status>
    {
        enum _enumType : uint16_t
        {
            NONE    = 0x0000,
            ACCEL_X = 0x0001,  ///<  
            ACCEL_Y = 0x0002,  ///<  
            ACCEL_Z = 0x0004,  ///<  
            GYRO_X  = 0x0010,  ///<  
            GYRO_Y  = 0x0020,  ///<  
            GYRO_Z  = 0x0040,  ///<  
            MAG_X   = 0x0100,  ///<  
            MAG_Y   = 0x0200,  ///<  
            MAG_Z   = 0x0400,  ///<  
            PRESS   = 0x1000,  ///<  
            ALL     = 0x1777,
        };
        uint16_t value = NONE;
        
        Status() : value(NONE) {}
        Status(int val) : value((uint16_t)val) {}
        operator uint16_t() const { return value; }
        Status& operator=(uint16_t val) { value = val; return *this; }
        Status& operator=(int val) { value = uint16_t(val); return *this; }
        Status& operator|=(uint16_t val) { return *this = value | val; }
        Status& operator&=(uint16_t val) { return *this = value & val; }
        
        bool accelX() const { return (value & ACCEL_X) > 0; }
        void accelX(bool val) { if(val) value |= ACCEL_X; else value &= ~ACCEL_X; }
        bool accelY() const { return (value & ACCEL_Y) > 0; }
        void accelY(bool val) { if(val) value |= ACCEL_Y; else value &= ~ACCEL_Y; }
        bool accelZ() const { return (value & ACCEL_Z) > 0; }
        void accelZ(bool val) { if(val) value |= ACCEL_Z; else value &= ~ACCEL_Z; }
        bool gyroX() const { return (value & GYRO_X) > 0; }
        void gyroX(bool val) { if(val) value |= GYRO_X; else value &= ~GYRO_X; }
        bool gyroY() const { return (value & GYRO_Y) > 0; }
        void gyroY(bool val) { if(val) value |= GYRO_Y; else value &= ~GYRO_Y; }
        bool gyroZ() const { return (value & GYRO_Z) > 0; }
        void gyroZ(bool val) { if(val) value |= GYRO_Z; else value &= ~GYRO_Z; }
        bool magX() const { return (value & MAG_X) > 0; }
        void magX(bool val) { if(val) value |= MAG_X; else value &= ~MAG_X; }
        bool magY() const { return (value & MAG_Y) > 0; }
        void magY(bool val) { if(val) value |= MAG_Y; else value &= ~MAG_Y; }
        bool magZ() const { return (value & MAG_Z) > 0; }
        void magZ(bool val) { if(val) value |= MAG_Z; else value &= ~MAG_Z; }
        bool press() const { return (value & PRESS) > 0; }
        void press(bool val) { if(val) value |= PRESS; else value &= ~PRESS; }
        
        bool allSet() const { return value == ALL; }
        void setAll() { value |= ALL; }
    };
    
    Status status;
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_OVERRANGE_STATUS;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "OverrangeStatus";
    static constexpr const char* DOC_NAME = "OverrangeStatus";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(status);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(status));
    }
};
void insert(Serializer& serializer, const OverrangeStatus& self);
void extract(Serializer& serializer, OverrangeStatus& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_sensor_odometer_data  (0x80,0x40) Odometer Data [CPP]
///
///@{

struct OdometerData
{
    float speed = 0; ///< Average speed over the time interval [m/s]. Can be negative for quadrature encoders.
    float uncertainty = 0; ///< Uncertainty of velocity [m/s].
    uint16_t valid_flags = 0; ///< If odometer is configured, bit 0 will be set to 1.
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_sensor::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_sensor::DATA_ODOMETER;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "OdometerData";
    static constexpr const char* DOC_NAME = "OdometerData";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(speed,uncertainty,valid_flags);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(speed),std::ref(uncertainty),std::ref(valid_flags));
    }
};
void insert(Serializer& serializer, const OdometerData& self);
void extract(Serializer& serializer, OdometerData& self);


///@}
///

///@}
///@}
///
////////////////////////////////////////////////////////////////////////////////
} // namespace data_sensor
} // namespace mip


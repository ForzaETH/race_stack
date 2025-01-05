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

namespace commands_base {

////////////////////////////////////////////////////////////////////////////////
///@addtogroup MipCommands_cpp  MIP Commands [CPP]
///@{
///@defgroup base_commands_cpp  Base Commands [CPP]
///
///@{

////////////////////////////////////////////////////////////////////////////////
// Descriptors
////////////////////////////////////////////////////////////////////////////////

enum 
{
    DESCRIPTOR_SET                 = 0x01,
    
    CMD_PING                       = 0x01,
    CMD_SET_TO_IDLE                = 0x02,
    CMD_GET_DEVICE_INFO            = 0x03,
    CMD_GET_DEVICE_DESCRIPTORS     = 0x04,
    CMD_BUILT_IN_TEST              = 0x05,
    CMD_RESUME                     = 0x06,
    CMD_GET_EXTENDED_DESCRIPTORS   = 0x07,
    CMD_CONTINUOUS_BIT             = 0x08,
    CMD_COMM_SPEED                 = 0x09,
    CMD_GPS_TIME_UPDATE            = 0x72,
    CMD_SOFT_RESET                 = 0x7E,
    
    REPLY_DEVICE_INFO              = 0x81,
    REPLY_DEVICE_DESCRIPTORS       = 0x82,
    REPLY_BUILT_IN_TEST            = 0x83,
    REPLY_GPS_CORRELATION_WEEK     = 0x84,
    REPLY_GPS_CORRELATION_SECONDS  = 0x85,
    REPLY_GET_EXTENDED_DESCRIPTORS = 0x86,
    REPLY_CONTINUOUS_BIT           = 0x88,
    REPLY_COMM_SPEED               = 0x89,
};

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

struct BaseDeviceInfo
{
    uint16_t firmware_version = 0;
    char model_name[16] = {0};
    char model_number[16] = {0};
    char serial_number[16] = {0};
    char lot_number[16] = {0};
    char device_options[16] = {0};
    
};
void insert(Serializer& serializer, const BaseDeviceInfo& self);
void extract(Serializer& serializer, BaseDeviceInfo& self);

enum class TimeFormat : uint8_t
{
    GPS = 1,  ///<  GPS time, a = week number since 1980, b = time of week in milliseconds.
};

struct CommandedTestBitsGq7 : Bitfield<CommandedTestBitsGq7>
{
    enum _enumType : uint32_t
    {
        NONE                   = 0x00000000,
        GENERAL_HARDWARE_FAULT = 0x00000001,  ///<  
        GENERAL_FIRMWARE_FAULT = 0x00000002,  ///<  
        TIMING_OVERLOAD        = 0x00000004,  ///<  
        BUFFER_OVERRUN         = 0x00000008,  ///<  
        RESERVED               = 0x000000F0,  ///<  
        IPC_IMU_FAULT          = 0x00000100,  ///<  
        IPC_NAV_FAULT          = 0x00000200,  ///<  
        IPC_GNSS_FAULT         = 0x00000400,  ///<  
        COMMS_FAULT            = 0x00000800,  ///<  
        IMU_ACCEL_FAULT        = 0x00001000,  ///<  
        IMU_GYRO_FAULT         = 0x00002000,  ///<  
        IMU_MAG_FAULT          = 0x00004000,  ///<  
        IMU_PRESS_FAULT        = 0x00008000,  ///<  
        IMU_RESERVED           = 0x00030000,  ///<  
        IMU_CAL_ERROR          = 0x00040000,  ///<  
        IMU_GENERAL_FAULT      = 0x00080000,  ///<  
        FILT_RESERVED          = 0x00300000,  ///<  
        FILT_SOLUTION_FAULT    = 0x00400000,  ///<  
        FILT_GENERAL_FAULT     = 0x00800000,  ///<  
        GNSS_RECEIVER1_FAULT   = 0x01000000,  ///<  
        GNSS_ANTENNA1_FAULT    = 0x02000000,  ///<  
        GNSS_RECEIVER2_FAULT   = 0x04000000,  ///<  
        GNSS_ANTENNA2_FAULT    = 0x08000000,  ///<  
        GNSS_RTCM_FAILURE      = 0x10000000,  ///<  
        GNSS_RTK_FAULT         = 0x20000000,  ///<  
        GNSS_SOLUTION_FAULT    = 0x40000000,  ///<  
        GNSS_GENERAL_FAULT     = 0x80000000,  ///<  
        ALL                    = 0xFFFFFFFF,
    };
    uint32_t value = NONE;
    
    CommandedTestBitsGq7() : value(NONE) {}
    CommandedTestBitsGq7(int val) : value((uint32_t)val) {}
    operator uint32_t() const { return value; }
    CommandedTestBitsGq7& operator=(uint32_t val) { value = val; return *this; }
    CommandedTestBitsGq7& operator=(int val) { value = uint32_t(val); return *this; }
    CommandedTestBitsGq7& operator|=(uint32_t val) { return *this = value | val; }
    CommandedTestBitsGq7& operator&=(uint32_t val) { return *this = value & val; }
    
    bool generalHardwareFault() const { return (value & GENERAL_HARDWARE_FAULT) > 0; }
    void generalHardwareFault(bool val) { if(val) value |= GENERAL_HARDWARE_FAULT; else value &= ~GENERAL_HARDWARE_FAULT; }
    bool generalFirmwareFault() const { return (value & GENERAL_FIRMWARE_FAULT) > 0; }
    void generalFirmwareFault(bool val) { if(val) value |= GENERAL_FIRMWARE_FAULT; else value &= ~GENERAL_FIRMWARE_FAULT; }
    bool timingOverload() const { return (value & TIMING_OVERLOAD) > 0; }
    void timingOverload(bool val) { if(val) value |= TIMING_OVERLOAD; else value &= ~TIMING_OVERLOAD; }
    bool bufferOverrun() const { return (value & BUFFER_OVERRUN) > 0; }
    void bufferOverrun(bool val) { if(val) value |= BUFFER_OVERRUN; else value &= ~BUFFER_OVERRUN; }
    uint32_t reserved() const { return (value & RESERVED) >> 4; }
    void reserved(uint32_t val) { value = (value & ~RESERVED) | (val << 4); }
    bool ipcImuFault() const { return (value & IPC_IMU_FAULT) > 0; }
    void ipcImuFault(bool val) { if(val) value |= IPC_IMU_FAULT; else value &= ~IPC_IMU_FAULT; }
    bool ipcNavFault() const { return (value & IPC_NAV_FAULT) > 0; }
    void ipcNavFault(bool val) { if(val) value |= IPC_NAV_FAULT; else value &= ~IPC_NAV_FAULT; }
    bool ipcGnssFault() const { return (value & IPC_GNSS_FAULT) > 0; }
    void ipcGnssFault(bool val) { if(val) value |= IPC_GNSS_FAULT; else value &= ~IPC_GNSS_FAULT; }
    bool commsFault() const { return (value & COMMS_FAULT) > 0; }
    void commsFault(bool val) { if(val) value |= COMMS_FAULT; else value &= ~COMMS_FAULT; }
    bool imuAccelFault() const { return (value & IMU_ACCEL_FAULT) > 0; }
    void imuAccelFault(bool val) { if(val) value |= IMU_ACCEL_FAULT; else value &= ~IMU_ACCEL_FAULT; }
    bool imuGyroFault() const { return (value & IMU_GYRO_FAULT) > 0; }
    void imuGyroFault(bool val) { if(val) value |= IMU_GYRO_FAULT; else value &= ~IMU_GYRO_FAULT; }
    bool imuMagFault() const { return (value & IMU_MAG_FAULT) > 0; }
    void imuMagFault(bool val) { if(val) value |= IMU_MAG_FAULT; else value &= ~IMU_MAG_FAULT; }
    bool imuPressFault() const { return (value & IMU_PRESS_FAULT) > 0; }
    void imuPressFault(bool val) { if(val) value |= IMU_PRESS_FAULT; else value &= ~IMU_PRESS_FAULT; }
    uint32_t imuReserved() const { return (value & IMU_RESERVED) >> 16; }
    void imuReserved(uint32_t val) { value = (value & ~IMU_RESERVED) | (val << 16); }
    bool imuCalError() const { return (value & IMU_CAL_ERROR) > 0; }
    void imuCalError(bool val) { if(val) value |= IMU_CAL_ERROR; else value &= ~IMU_CAL_ERROR; }
    bool imuGeneralFault() const { return (value & IMU_GENERAL_FAULT) > 0; }
    void imuGeneralFault(bool val) { if(val) value |= IMU_GENERAL_FAULT; else value &= ~IMU_GENERAL_FAULT; }
    uint32_t filtReserved() const { return (value & FILT_RESERVED) >> 20; }
    void filtReserved(uint32_t val) { value = (value & ~FILT_RESERVED) | (val << 20); }
    bool filtSolutionFault() const { return (value & FILT_SOLUTION_FAULT) > 0; }
    void filtSolutionFault(bool val) { if(val) value |= FILT_SOLUTION_FAULT; else value &= ~FILT_SOLUTION_FAULT; }
    bool filtGeneralFault() const { return (value & FILT_GENERAL_FAULT) > 0; }
    void filtGeneralFault(bool val) { if(val) value |= FILT_GENERAL_FAULT; else value &= ~FILT_GENERAL_FAULT; }
    bool gnssReceiver1Fault() const { return (value & GNSS_RECEIVER1_FAULT) > 0; }
    void gnssReceiver1Fault(bool val) { if(val) value |= GNSS_RECEIVER1_FAULT; else value &= ~GNSS_RECEIVER1_FAULT; }
    bool gnssAntenna1Fault() const { return (value & GNSS_ANTENNA1_FAULT) > 0; }
    void gnssAntenna1Fault(bool val) { if(val) value |= GNSS_ANTENNA1_FAULT; else value &= ~GNSS_ANTENNA1_FAULT; }
    bool gnssReceiver2Fault() const { return (value & GNSS_RECEIVER2_FAULT) > 0; }
    void gnssReceiver2Fault(bool val) { if(val) value |= GNSS_RECEIVER2_FAULT; else value &= ~GNSS_RECEIVER2_FAULT; }
    bool gnssAntenna2Fault() const { return (value & GNSS_ANTENNA2_FAULT) > 0; }
    void gnssAntenna2Fault(bool val) { if(val) value |= GNSS_ANTENNA2_FAULT; else value &= ~GNSS_ANTENNA2_FAULT; }
    bool gnssRtcmFailure() const { return (value & GNSS_RTCM_FAILURE) > 0; }
    void gnssRtcmFailure(bool val) { if(val) value |= GNSS_RTCM_FAILURE; else value &= ~GNSS_RTCM_FAILURE; }
    bool gnssRtkFault() const { return (value & GNSS_RTK_FAULT) > 0; }
    void gnssRtkFault(bool val) { if(val) value |= GNSS_RTK_FAULT; else value &= ~GNSS_RTK_FAULT; }
    bool gnssSolutionFault() const { return (value & GNSS_SOLUTION_FAULT) > 0; }
    void gnssSolutionFault(bool val) { if(val) value |= GNSS_SOLUTION_FAULT; else value &= ~GNSS_SOLUTION_FAULT; }
    bool gnssGeneralFault() const { return (value & GNSS_GENERAL_FAULT) > 0; }
    void gnssGeneralFault(bool val) { if(val) value |= GNSS_GENERAL_FAULT; else value &= ~GNSS_GENERAL_FAULT; }
    
    bool allSet() const { return value == ALL; }
    void setAll() { value |= ALL; }
};


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_base_ping  (0x01,0x01) Ping [CPP]
/// Test Communications with a device.
/// 
/// The Device will respond with an ACK, if present and operating correctly.
/// 
/// If the device is not in a normal operating mode, it may NACK.
///
///@{

struct Ping
{
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_base::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_base::CMD_PING;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "Ping";
    static constexpr const char* DOC_NAME = "Ping";
    
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
void insert(Serializer& serializer, const Ping& self);
void extract(Serializer& serializer, Ping& self);

TypedResult<Ping> ping(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_base_set_idle  (0x01,0x02) Set Idle [CPP]
/// Turn off all device data streams.
/// 
/// The Device will respond with an ACK, if present and operating correctly.
/// This command will suspend streaming (if enabled) or wake the device from sleep (if sleeping) to allow it to respond to status and setup commands.
/// You may restore the device mode by issuing the Resume command.
///
///@{

struct SetIdle
{
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_base::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_base::CMD_SET_TO_IDLE;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "SetIdle";
    static constexpr const char* DOC_NAME = "Set to idle";
    
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
void insert(Serializer& serializer, const SetIdle& self);
void extract(Serializer& serializer, SetIdle& self);

TypedResult<SetIdle> setIdle(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_base_get_device_info  (0x01,0x03) Get Device Info [CPP]
/// Get the device ID strings and firmware version number.
///
///@{

struct GetDeviceInfo
{
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_base::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_base::CMD_GET_DEVICE_INFO;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "GetDeviceInfo";
    static constexpr const char* DOC_NAME = "Get device information";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple();
    }
    
    auto as_tuple()
    {
        return std::make_tuple();
    }
    struct Response
    {
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_base::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_base::REPLY_DEVICE_INFO;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "GetDeviceInfo::Response";
        static constexpr const char* DOC_NAME = "Get device information Response";
        
        static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
        static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
        BaseDeviceInfo device_info;
        
        
        auto as_tuple()
        {
            return std::make_tuple(std::ref(device_info));
        }
    };
};
void insert(Serializer& serializer, const GetDeviceInfo& self);
void extract(Serializer& serializer, GetDeviceInfo& self);

void insert(Serializer& serializer, const GetDeviceInfo::Response& self);
void extract(Serializer& serializer, GetDeviceInfo::Response& self);

TypedResult<GetDeviceInfo> getDeviceInfo(C::mip_interface& device, BaseDeviceInfo* deviceInfoOut);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_base_get_device_descriptors  (0x01,0x04) Get Device Descriptors [CPP]
/// Get the command and data descriptors supported by the device.
/// 
/// Reply has two fields: "ACK/NACK" and "Descriptors". The "Descriptors" field is an array of 16 bit values.
/// The MSB specifies the descriptor set and the LSB specifies the descriptor.
///
///@{

struct GetDeviceDescriptors
{
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_base::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_base::CMD_GET_DEVICE_DESCRIPTORS;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "GetDeviceDescriptors";
    static constexpr const char* DOC_NAME = "Get device descriptors";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple();
    }
    
    auto as_tuple()
    {
        return std::make_tuple();
    }
    struct Response
    {
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_base::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_base::REPLY_DEVICE_DESCRIPTORS;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "GetDeviceDescriptors::Response";
        static constexpr const char* DOC_NAME = "Get device descriptors Response";
        
        static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
        static constexpr const uint32_t COUNTER_PARAMS = 0x00000001;
        uint16_t descriptors[253] = {0};
        uint8_t descriptors_count = 0;
        
        
        auto as_tuple()
        {
            return std::make_tuple(std::ref(descriptors),std::ref(descriptors_count));
        }
    };
};
void insert(Serializer& serializer, const GetDeviceDescriptors& self);
void extract(Serializer& serializer, GetDeviceDescriptors& self);

void insert(Serializer& serializer, const GetDeviceDescriptors::Response& self);
void extract(Serializer& serializer, GetDeviceDescriptors::Response& self);

TypedResult<GetDeviceDescriptors> getDeviceDescriptors(C::mip_interface& device, uint16_t* descriptorsOut, size_t descriptorsOutMax, uint8_t* descriptorsOutCount);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_base_built_in_test  (0x01,0x05) Built In Test [CPP]
/// Run the device Built-In Test (BIT).
/// 
/// The Built-In Test command always returns a 32 bit value.
/// A value of 0 means that all tests passed.
/// A non-zero value indicates that not all tests passed.
/// Reference the device user manual to decode the result.
///
///@{

struct BuiltInTest
{
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_base::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_base::CMD_BUILT_IN_TEST;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "BuiltInTest";
    static constexpr const char* DOC_NAME = "Built in test";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple();
    }
    
    auto as_tuple()
    {
        return std::make_tuple();
    }
    struct Response
    {
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_base::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_base::REPLY_BUILT_IN_TEST;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "BuiltInTest::Response";
        static constexpr const char* DOC_NAME = "Built in test Response";
        
        static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
        static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
        uint32_t result = 0;
        
        
        auto as_tuple()
        {
            return std::make_tuple(std::ref(result));
        }
    };
};
void insert(Serializer& serializer, const BuiltInTest& self);
void extract(Serializer& serializer, BuiltInTest& self);

void insert(Serializer& serializer, const BuiltInTest::Response& self);
void extract(Serializer& serializer, BuiltInTest::Response& self);

TypedResult<BuiltInTest> builtInTest(C::mip_interface& device, uint32_t* resultOut);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_base_resume  (0x01,0x06) Resume [CPP]
/// Take the device out of idle mode.
/// 
/// The device responds with ACK upon success.
///
///@{

struct Resume
{
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_base::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_base::CMD_RESUME;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "Resume";
    static constexpr const char* DOC_NAME = "Resume";
    
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
void insert(Serializer& serializer, const Resume& self);
void extract(Serializer& serializer, Resume& self);

TypedResult<Resume> resume(C::mip_interface& device);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_base_get_extended_descriptors  (0x01,0x07) Get Extended Descriptors [CPP]
/// Get the command and data descriptors supported by the device.
/// 
/// Reply has two fields: "ACK/NACK" and "Descriptors". The "Descriptors" field is an array of 16 bit values.
/// The MSB specifies the descriptor set and the LSB specifies the descriptor.
///
///@{

struct GetExtendedDescriptors
{
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_base::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_base::CMD_GET_EXTENDED_DESCRIPTORS;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "GetExtendedDescriptors";
    static constexpr const char* DOC_NAME = "Get device descriptors (extended)";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple();
    }
    
    auto as_tuple()
    {
        return std::make_tuple();
    }
    struct Response
    {
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_base::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_base::REPLY_GET_EXTENDED_DESCRIPTORS;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "GetExtendedDescriptors::Response";
        static constexpr const char* DOC_NAME = "Get device descriptors (extended) Response";
        
        static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
        static constexpr const uint32_t COUNTER_PARAMS = 0x00000001;
        uint16_t descriptors[253] = {0};
        uint8_t descriptors_count = 0;
        
        
        auto as_tuple()
        {
            return std::make_tuple(std::ref(descriptors),std::ref(descriptors_count));
        }
    };
};
void insert(Serializer& serializer, const GetExtendedDescriptors& self);
void extract(Serializer& serializer, GetExtendedDescriptors& self);

void insert(Serializer& serializer, const GetExtendedDescriptors::Response& self);
void extract(Serializer& serializer, GetExtendedDescriptors::Response& self);

TypedResult<GetExtendedDescriptors> getExtendedDescriptors(C::mip_interface& device, uint16_t* descriptorsOut, size_t descriptorsOutMax, uint8_t* descriptorsOutCount);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_base_continuous_bit  (0x01,0x08) Continuous Bit [CPP]
/// Report result of continuous built-in test.
/// 
/// This test is non-disruptive but is not as thorough as the commanded BIT.
///
///@{

struct ContinuousBit
{
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_base::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_base::CMD_CONTINUOUS_BIT;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "ContinuousBit";
    static constexpr const char* DOC_NAME = "Continuous built-in test";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = false;
    static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple();
    }
    
    auto as_tuple()
    {
        return std::make_tuple();
    }
    struct Response
    {
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_base::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_base::REPLY_CONTINUOUS_BIT;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "ContinuousBit::Response";
        static constexpr const char* DOC_NAME = "Continuous built-in test Response";
        
        static constexpr const uint32_t ECHOED_PARAMS  = 0x0000;
        static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
        uint8_t result[16] = {0}; ///< Device-specific bitfield (128 bits). See device user manual. Bits are least-significant-byte first. For example, bit 0 is located at bit 0 of result[0], bit 1 is located at bit 1 of result[0], bit 8 is located at bit 0 of result[1], and bit 127 is located at bit 7 of result[15].
        
        
        auto as_tuple()
        {
            return std::make_tuple(std::ref(result));
        }
    };
};
void insert(Serializer& serializer, const ContinuousBit& self);
void extract(Serializer& serializer, ContinuousBit& self);

void insert(Serializer& serializer, const ContinuousBit::Response& self);
void extract(Serializer& serializer, ContinuousBit::Response& self);

TypedResult<ContinuousBit> continuousBit(C::mip_interface& device, uint8_t* resultOut);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_base_comm_speed  (0x01,0x09) Comm Speed [CPP]
/// Controls the baud rate of a specific port on the device.
/// 
/// Please see the device user manual for supported baud rates on each port.
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

struct CommSpeed
{
    static constexpr const uint32_t ALL_PORTS = 0;
    FunctionSelector function = static_cast<FunctionSelector>(0);
    uint8_t port = 0; ///< Port ID number, starting with 1. When function is SAVE, LOAD, or DEFAULT, this can be 0 to apply to all ports. See the device user manual for details.
    uint32_t baud = 0; ///< Port baud rate. Must be a supported rate.
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_base::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_base::CMD_COMM_SPEED;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "CommSpeed";
    static constexpr const char* DOC_NAME = "Comm Port Speed";
    
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
        return std::make_tuple(port,baud);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(port),std::ref(baud));
    }
    
    static CommSpeed create_sld_all(::mip::FunctionSelector function)
    {
        CommSpeed cmd;
        cmd.function = function;
        cmd.port = 0;
        return cmd;
    }
    
    struct Response
    {
        static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_base::DESCRIPTOR_SET;
        static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_base::REPLY_COMM_SPEED;
        static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
        static constexpr const char* NAME = "CommSpeed::Response";
        static constexpr const char* DOC_NAME = "Comm Port Speed Response";
        
        static constexpr const uint32_t ECHOED_PARAMS  = 0x0001;
        static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
        uint8_t port = 0; ///< Port ID number, starting with 1. When function is SAVE, LOAD, or DEFAULT, this can be 0 to apply to all ports. See the device user manual for details.
        uint32_t baud = 0; ///< Port baud rate. Must be a supported rate.
        
        
        auto as_tuple()
        {
            return std::make_tuple(std::ref(port),std::ref(baud));
        }
    };
};
void insert(Serializer& serializer, const CommSpeed& self);
void extract(Serializer& serializer, CommSpeed& self);

void insert(Serializer& serializer, const CommSpeed::Response& self);
void extract(Serializer& serializer, CommSpeed::Response& self);

TypedResult<CommSpeed> writeCommSpeed(C::mip_interface& device, uint8_t port, uint32_t baud);
TypedResult<CommSpeed> readCommSpeed(C::mip_interface& device, uint8_t port, uint32_t* baudOut);
TypedResult<CommSpeed> saveCommSpeed(C::mip_interface& device, uint8_t port);
TypedResult<CommSpeed> loadCommSpeed(C::mip_interface& device, uint8_t port);
TypedResult<CommSpeed> defaultCommSpeed(C::mip_interface& device, uint8_t port);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_base_gps_time_update  (0x01,0x72) Gps Time Update [CPP]
/// Set device internal GPS time
/// When combined with a PPS input signal applied to the I/O connector, this command enables complete synchronization of data outputs
/// with an external time base, such as GPS system time. Since the hardware PPS synchronization can only detect the fractional number of seconds when pulses arrive,
/// complete synchronization requires that the user provide the whole number of seconds via this command. After achieving PPS synchronization, this command should be sent twice: once to set the time-of-week and once to set the week number. PPS synchronization can be verified by monitoring the time sync status message (0xA0, 0x02) or the valid flags of any shared external timestamp (0x--, D7) data field.
///
///@{

struct GpsTimeUpdate
{
    enum class FieldId : uint8_t
    {
        WEEK_NUMBER  = 1,  ///<  Week number.
        TIME_OF_WEEK = 2,  ///<  Time of week in seconds.
    };
    
    FunctionSelector function = static_cast<FunctionSelector>(0);
    FieldId field_id = static_cast<FieldId>(0); ///< Determines how to interpret value.
    uint32_t value = 0; ///< Week number or time of week, depending on the field_id.
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_base::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_base::CMD_GPS_TIME_UPDATE;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "GpsTimeUpdate";
    static constexpr const char* DOC_NAME = "GPS Time Update Command";
    
    static constexpr const bool HAS_FUNCTION_SELECTOR = true;
    static constexpr const uint32_t WRITE_PARAMS   = 0x8003;
    static constexpr const uint32_t READ_PARAMS    = 0x0000;
    static constexpr const uint32_t SAVE_PARAMS    = 0x0000;
    static constexpr const uint32_t LOAD_PARAMS    = 0x0000;
    static constexpr const uint32_t DEFAULT_PARAMS = 0x0000;
    static constexpr const uint32_t COUNTER_PARAMS = 0x00000000;
    
    auto as_tuple() const
    {
        return std::make_tuple(field_id,value);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(field_id),std::ref(value));
    }
    
    static GpsTimeUpdate create_sld_all(::mip::FunctionSelector function)
    {
        GpsTimeUpdate cmd;
        cmd.function = function;
        return cmd;
    }
    
    typedef void Response;
};
void insert(Serializer& serializer, const GpsTimeUpdate& self);
void extract(Serializer& serializer, GpsTimeUpdate& self);

TypedResult<GpsTimeUpdate> writeGpsTimeUpdate(C::mip_interface& device, GpsTimeUpdate::FieldId fieldId, uint32_t value);

///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_base_soft_reset  (0x01,0x7E) Soft Reset [CPP]
/// Resets the device.
/// 
/// Device responds with ACK and immediately resets.
///
///@{

struct SoftReset
{
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::commands_base::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::commands_base::CMD_SOFT_RESET;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "SoftReset";
    static constexpr const char* DOC_NAME = "Reset device";
    
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
void insert(Serializer& serializer, const SoftReset& self);
void extract(Serializer& serializer, SoftReset& self);

TypedResult<SoftReset> softReset(C::mip_interface& device);

///@}
///

///@}
///@}
///
////////////////////////////////////////////////////////////////////////////////
} // namespace commands_base
} // namespace mip


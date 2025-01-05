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

namespace data_system {

////////////////////////////////////////////////////////////////////////////////
///@addtogroup MipData_cpp  MIP Data [CPP]
///@{
///@defgroup system_data_cpp  System Data [CPP]
///
///@{

////////////////////////////////////////////////////////////////////////////////
// Descriptors
////////////////////////////////////////////////////////////////////////////////

enum 
{
    DESCRIPTOR_SET         = 0xA0,
    
    DATA_BUILT_IN_TEST     = 0x01,
    DATA_TIME_SYNC_STATUS  = 0x02,
    DATA_GPIO_STATE        = 0x03,
    DATA_GPIO_ANALOG_VALUE = 0x04,
    
};

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_system_built_in_test  (0xA0,0x01) Built In Test [CPP]
/// Contains the continuous built-in-test (BIT) results.
/// 
/// Due to the large size of this field, it is recommended to stream it at
/// a low rate or poll it on demand.
/// 
/// These bits are "sticky" until the next output message. If a fault occurs
/// in between scheduled messages or while the device is idle, the next
/// packet with this field will have the corresponding flags set. The flag
/// is then cleared unless the fault persists.
/// 
/// Unlike the commanded BIT, some bits may be 1 in certain
/// non-fault situations, so simply checking if the result is all 0s is
/// not very useful. For example, on devices with a built-in GNSS receiver,
/// a "solution fault" bit may be set before the receiver has obtained
/// a position fix. Consult the device manual to determine which bits are
/// of interest for your application.
/// 
/// All unspecified bits are reserved for future use and must be ignored.
/// 
///
///@{

struct BuiltInTest
{
    uint8_t result[16] = {0}; ///< Device-specific bitfield (128 bits). See device user manual. Bits are least-significant-byte first. For example, bit 0 is located at bit 0 of result[0], bit 1 is located at bit 1 of result[0], bit 8 is located at bit 0 of result[1], and bit 127 is located at bit 7 of result[15].
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_system::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_system::DATA_BUILT_IN_TEST;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "BuiltInTest";
    static constexpr const char* DOC_NAME = "BuiltInTest";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(result);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(result));
    }
};
void insert(Serializer& serializer, const BuiltInTest& self);
void extract(Serializer& serializer, BuiltInTest& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_system_time_sync_status  (0xA0,0x02) Time Sync Status [CPP]
/// Indicates whether a sync has been achieved using the PPS signal.
///
///@{

struct TimeSyncStatus
{
    bool time_sync = 0; ///< True if sync with the PPS signal is currently valid. False if PPS feature is disabled or a PPS signal is not detected.
    uint8_t last_pps_rcvd = 0; ///< Elapsed time in seconds since last PPS was received, with a maximum value of 255.
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_system::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_system::DATA_TIME_SYNC_STATUS;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "TimeSyncStatus";
    static constexpr const char* DOC_NAME = "TimeSyncStatus";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(time_sync,last_pps_rcvd);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(time_sync),std::ref(last_pps_rcvd));
    }
};
void insert(Serializer& serializer, const TimeSyncStatus& self);
void extract(Serializer& serializer, TimeSyncStatus& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_system_gpio_state  (0xA0,0x03) Gpio State [CPP]
/// Indicates the state of all of the user GPIO pins.
/// 
/// This message can be used to correlate external signals
/// with the device time or other data quantities. It should
/// generally be used with slow GPIO signals as brief pulses
/// shorter than the scheduled data rate will be missed.
/// 
/// To synchronize with faster signals and pulses, or for more accurate timestamping,
/// utilize the event system and set the GPIO feature to TIMESTAMP in the 3DM GPIO
/// Configuration command (0x0C,0x41).
/// 
/// These GPIO states are sampled within one base period
/// of the system data descriptor set.
/// 
/// To obtain valid readings, the desired pin(s) must be configured to the GPIO feature
/// (either input or output behavior) using the 3DM GPIO Configuration command
/// (0x0C,0x41). Other gpio features may work on some devices but this is not guaranteed.
/// Consult the factory before producing a design relying on reading pins configured
/// to other feature types.
///
///@{

struct GpioState
{
    uint8_t states = 0; ///< Bitfield containing the states for each GPIO pin.<br/> Bit 0 (0x01): pin 1<br/> Bit 1 (0x02): pin 2<br/> Bit 2 (0x04): pin 3<br/> Bit 3 (0x08): pin 4<br/> Bits for pins that don't exist will read as 0.
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_system::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_system::DATA_GPIO_STATE;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "GpioState";
    static constexpr const char* DOC_NAME = "GpioState";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(states);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(states));
    }
};
void insert(Serializer& serializer, const GpioState& self);
void extract(Serializer& serializer, GpioState& self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup cpp_system_gpio_analog_value  (0xA0,0x04) Gpio Analog Value [CPP]
/// Indicates the analog value of the given user GPIO.
/// The pin must be configured for analog input.
///
///@{

struct GpioAnalogValue
{
    uint8_t gpio_id = 0; ///< GPIO pin number starting with 1.
    float value = 0; ///< Value of the GPIO line in scaled volts.
    
    static constexpr const uint8_t DESCRIPTOR_SET = ::mip::data_system::DESCRIPTOR_SET;
    static constexpr const uint8_t FIELD_DESCRIPTOR = ::mip::data_system::DATA_GPIO_ANALOG_VALUE;
    static constexpr const CompositeDescriptor DESCRIPTOR = {DESCRIPTOR_SET, FIELD_DESCRIPTOR};
    static constexpr const char* NAME = "GpioAnalogValue";
    static constexpr const char* DOC_NAME = "GpioAnalogValue";
    
    
    auto as_tuple() const
    {
        return std::make_tuple(gpio_id,value);
    }
    
    auto as_tuple()
    {
        return std::make_tuple(std::ref(gpio_id),std::ref(value));
    }
};
void insert(Serializer& serializer, const GpioAnalogValue& self);
void extract(Serializer& serializer, GpioAnalogValue& self);


///@}
///

///@}
///@}
///
////////////////////////////////////////////////////////////////////////////////
} // namespace data_system
} // namespace mip


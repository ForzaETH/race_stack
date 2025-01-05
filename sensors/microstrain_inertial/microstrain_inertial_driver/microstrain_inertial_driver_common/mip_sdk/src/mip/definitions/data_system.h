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
///@defgroup system_data_c  System Data [C]
///
///@{

////////////////////////////////////////////////////////////////////////////////
// Descriptors
////////////////////////////////////////////////////////////////////////////////

enum 
{
    MIP_SYSTEM_DATA_DESC_SET               = 0xA0,
    
    MIP_DATA_DESC_SYSTEM_BUILT_IN_TEST     = 0x01,
    MIP_DATA_DESC_SYSTEM_TIME_SYNC_STATUS  = 0x02,
    MIP_DATA_DESC_SYSTEM_GPIO_STATE        = 0x03,
    MIP_DATA_DESC_SYSTEM_GPIO_ANALOG_VALUE = 0x04,
    
};

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
///@defgroup c_system_built_in_test  (0xA0,0x01) Built In Test [C]
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

struct mip_system_built_in_test_data
{
    uint8_t result[16]; ///< Device-specific bitfield (128 bits). See device user manual. Bits are least-significant-byte first. For example, bit 0 is located at bit 0 of result[0], bit 1 is located at bit 1 of result[0], bit 8 is located at bit 0 of result[1], and bit 127 is located at bit 7 of result[15].
    
};
typedef struct mip_system_built_in_test_data mip_system_built_in_test_data;
void insert_mip_system_built_in_test_data(struct mip_serializer* serializer, const mip_system_built_in_test_data* self);
void extract_mip_system_built_in_test_data(struct mip_serializer* serializer, mip_system_built_in_test_data* self);
bool extract_mip_system_built_in_test_data_from_field(const struct mip_field* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_system_time_sync_status  (0xA0,0x02) Time Sync Status [C]
/// Indicates whether a sync has been achieved using the PPS signal.
///
///@{

struct mip_system_time_sync_status_data
{
    bool time_sync; ///< True if sync with the PPS signal is currently valid. False if PPS feature is disabled or a PPS signal is not detected.
    uint8_t last_pps_rcvd; ///< Elapsed time in seconds since last PPS was received, with a maximum value of 255.
    
};
typedef struct mip_system_time_sync_status_data mip_system_time_sync_status_data;
void insert_mip_system_time_sync_status_data(struct mip_serializer* serializer, const mip_system_time_sync_status_data* self);
void extract_mip_system_time_sync_status_data(struct mip_serializer* serializer, mip_system_time_sync_status_data* self);
bool extract_mip_system_time_sync_status_data_from_field(const struct mip_field* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_system_gpio_state  (0xA0,0x03) Gpio State [C]
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

struct mip_system_gpio_state_data
{
    uint8_t states; ///< Bitfield containing the states for each GPIO pin.<br/> Bit 0 (0x01): pin 1<br/> Bit 1 (0x02): pin 2<br/> Bit 2 (0x04): pin 3<br/> Bit 3 (0x08): pin 4<br/> Bits for pins that don't exist will read as 0.
    
};
typedef struct mip_system_gpio_state_data mip_system_gpio_state_data;
void insert_mip_system_gpio_state_data(struct mip_serializer* serializer, const mip_system_gpio_state_data* self);
void extract_mip_system_gpio_state_data(struct mip_serializer* serializer, mip_system_gpio_state_data* self);
bool extract_mip_system_gpio_state_data_from_field(const struct mip_field* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_system_gpio_analog_value  (0xA0,0x04) Gpio Analog Value [C]
/// Indicates the analog value of the given user GPIO.
/// The pin must be configured for analog input.
///
///@{

struct mip_system_gpio_analog_value_data
{
    uint8_t gpio_id; ///< GPIO pin number starting with 1.
    float value; ///< Value of the GPIO line in scaled volts.
    
};
typedef struct mip_system_gpio_analog_value_data mip_system_gpio_analog_value_data;
void insert_mip_system_gpio_analog_value_data(struct mip_serializer* serializer, const mip_system_gpio_analog_value_data* self);
void extract_mip_system_gpio_analog_value_data(struct mip_serializer* serializer, mip_system_gpio_analog_value_data* self);
bool extract_mip_system_gpio_analog_value_data_from_field(const struct mip_field* field, void* ptr);


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


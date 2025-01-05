#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "mip_packet.h"

#ifdef __cplusplus
namespace mip{
namespace C {
extern "C" {
#endif


////////////////////////////////////////////////////////////////////////////////
///@addtogroup mip_c
///@{

////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_field_c  Mip Fields [C]
///
///@brief Functions for processing received MIP fields.
///
///~~~
/// ---------------+------------+------------+-----/ /-----+------------+
///  ...   Header  |   Field    |   Field    |     ...     |  Checksum  |
/// ---------------+------------+------------+-----/ /-----+------------+
///              _/              \_
///            _/                  \_
///           /                      \.
///          +------+------+---/ /----+
///          | Len  | desc | payload  |        The MIP Field
///          +------+------+---/ /----+
///~~~
///
///@{
///

////////////////////////////////////////////////////////////////////////////////
///@brief A structure representing a MIP field.
///
/// Use to access fields from a received MIP packet.
///
/// This structure references the original packet data and does not contain a
/// copy of the field payload. Therefore, the data buffer must exist as long as
/// there are %mip_field instances which reference it (even if the field payload
/// itself is not used directly).
///
///@note This should be considered an "opaque" structure; its members should be
/// considered an internal implementation detail. Avoid accessing them directly
/// as they are subject to change in future versions of this software.
///
typedef struct mip_field
{
    const uint8_t* _payload;    ///<@private The field payload, excluding the header.
    uint8_t _payload_length;    ///<@private The length of the payload, excluding the header.
    uint8_t _field_descriptor;  ///<@private MIP field descriptor. Field not valid if set to 0x00.
    uint8_t _descriptor_set;    ///<@private MIP descriptor set (from the packet)
    uint8_t _remaining_length;  ///<@private Remaining space after this field.
} mip_field;


void mip_field_init(mip_field* field, uint8_t descriptor_set, uint8_t field_descriptor, const uint8_t* payload, uint8_t payload_length);

////////////////////////////////////////////////////////////////////////////////
///@defgroup FieldAccess  Field Accessors [C]
///
///@brief Functions for inspecting a MIP field.
///
///@{

uint8_t mip_field_descriptor_set(const mip_field* field);
uint8_t mip_field_field_descriptor(const mip_field* field);
uint8_t mip_field_payload_length(const mip_field* field);
const uint8_t* mip_field_payload(const mip_field* field);

bool mip_field_is_valid(const mip_field* field);

///@}
////////////////////////////////////////////////////////////////////////////////
///@defgroup FieldIteration_c  Field Iteration [C]
///
///@brief Functions for iterating over fields in a MIP packet.
///
/// Use these functions to iterate over the fields in a MIP packet.
///
/// Example:
///~~~{.c}
/// // Iterate over fields, starting with the first in the packet.
/// // Continue as long as a valid field is found.
/// for(mip_field field = mip_field_from_packet(packet); mip_field_is_valid(&field); mip_field_next(&field))
/// {
///     // Check the field descriptor for what kind of data it holds.
///     switch( mip_field_field_desriptor(&field) )
///     {
///     case MIP_DATA_DESC_SENSOR_SCALED_ACCEL:
///         break;
///     }
/// }
///~~~
///
///@{

void mip_field_init_empty(mip_field* field);

mip_field mip_field_from_header_ptr(const uint8_t* header, uint8_t total_length, uint8_t descriptor_set);

mip_field mip_field_first_from_packet(const mip_packet* packet);
mip_field mip_field_next_after(const mip_field* field);
bool mip_field_next(mip_field* field);

bool mip_field_next_in_packet(mip_field* field, const mip_packet* packet);

// bool mip_field_is_at_end(const struct mip_field* field);

///@}
///@}
///@}
////////////////////////////////////////////////////////////////////////////////
///

#ifdef __cplusplus
} // namespace mip
} // namespace C
} // extern "C"
#endif

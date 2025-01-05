
#include "mip_field.h"

#include "mip_packet.h"
#include "mip_offsets.h"
#include "definitions/descriptors.h"

#include <assert.h>


////////////////////////////////////////////////////////////////////////////////
///@brief Constructs a %mip_field given the parameters.
///
///@param field
///@param descriptor_set
///       The MIP descriptor set of the packet.
///@param field_descriptor
///       The MIP field descriptor.
///@param payload
///       A pointer to a buffer containing the field payload, not including the
///       field header. The data must exist while the field is in use. Can be
///       NULL if payload_length is 0.
///@param payload_length
///       The length of the payload. Cannot exceed MIP_FIELD_PAYLOAD_LENGTH_MAX.
///
///@returns A %mip_field initialized with the specified values.
///
void mip_field_init(mip_field* field, uint8_t descriptor_set, uint8_t field_descriptor, const uint8_t* payload, uint8_t payload_length)
{
    assert( payload_length <= MIP_FIELD_PAYLOAD_LENGTH_MAX );
    if( payload_length > MIP_FIELD_PAYLOAD_LENGTH_MAX )
        payload_length = MIP_FIELD_PAYLOAD_LENGTH_MAX;

    field->_payload          = payload;
    field->_payload_length   = payload_length;
    field->_field_descriptor = field_descriptor;
    field->_descriptor_set   = descriptor_set;
    field->_remaining_length = 0;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Returns the descriptor set of the packet containing this field._
///
uint8_t mip_field_descriptor_set(const mip_field* field)
{
    return field->_descriptor_set;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Returns the field descriptor.
///
uint8_t mip_field_field_descriptor(const mip_field* field)
{
    return field->_field_descriptor;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Returns the length of the payload.
///
uint8_t mip_field_payload_length(const mip_field* field)
{
    assert(field->_payload_length <= MIP_FIELD_PAYLOAD_LENGTH_MAX);

    return field->_payload_length;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Returns the payload pointer for the field data.
///
const uint8_t* mip_field_payload(const mip_field* field)
{
    return field->_payload;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Returns true if the field has a valid field descriptor.
///
bool mip_field_is_valid(const mip_field* field)
{
    return field->_field_descriptor != MIP_INVALID_FIELD_DESCRIPTOR;
}



////////////////////////////////////////////////////////////////////////////////
///@brief Initialize a mip_field struct to an invalid/empty state.
///
/// Intended to be used with mip_field_next_in_packet. The field will not be
/// valid (mip_field_is_valid will return false).
///
///@param field
///
void mip_field_init_empty(mip_field* field)
{
    mip_field_init(field, MIP_INVALID_DESCRIPTOR_SET, MIP_INVALID_FIELD_DESCRIPTOR, NULL, 0);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Constructs a %mip_field from a pointer to the heaader.
///
/// Generally you should use mip_field_from_packet() or mip_field_create() instead.
///
///@param header
///       A pointer to the header and payload. Usually inside of a MIP packet.
///@param total_length
///       The total length of either the field or packet payload, starting from
///       header_ptr and including the header bytes. If total_length is longer
///       than the field (i.e. if it's the packet payload length) then
///       mip_field_next() may be used to iterate fields.
///@param descriptor_set
///       The descriptor set for the packet containing this field. May be 0x00
///       if not used by any function handling the field.
///
///@returns a mip_field struct with the field data.
///
mip_field mip_field_from_header_ptr(const uint8_t* header, uint8_t total_length, uint8_t descriptor_set)
{
    mip_field field;

    // Default invalid values.
    field._payload          = NULL;
    field._descriptor_set   = descriptor_set;
    field._payload_length   = 0;
    field._field_descriptor = 0x00;  // This makes the field invalid.
    field._remaining_length = 0;

    if( total_length >= MIP_FIELD_HEADER_LENGTH )
    {
        // Field length is external input so it must be sanitized.
        uint8_t field_length = header[MIP_INDEX_FIELD_LEN];

        // Ensure field length does not exceed total_length.
        if( field_length > total_length )
            field_length = total_length;

        // Check for invalid field length.
        if( field_length >= MIP_FIELD_HEADER_LENGTH )
        {
            field._field_descriptor = header[MIP_INDEX_FIELD_DESC];
            field._payload_length   = field_length - MIP_FIELD_HEADER_LENGTH;
            field._payload          = header + MIP_INDEX_FIELD_PAYLOAD;
            field._remaining_length = total_length - field_length;
        }
    }

    return field;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Extracts the first field from a MIP packet.
///
/// Typically this would be used as the first step when iterating through all
/// fields in a received packet. To access each field after the first, call
/// mip_field_next(). You should call mip_field_is_at_end() or mip_field_is_valid()
/// after obtaining each field (including the first) to determine if the field
/// is actually valid.
///
///@param packet
///       The mip packet containing 0 or more fields. Assumed to be valid, and
///       the payload pointer and size must be correct.
///
///@returns A mip_field struct with the first field from the packet.
///
mip_field mip_field_first_from_packet(const mip_packet* packet)
{
    return mip_field_from_header_ptr( mip_packet_payload(packet), mip_packet_payload_length(packet), mip_packet_descriptor_set(packet) );
}

////////////////////////////////////////////////////////////////////////////////
///@brief Gets the next field after the specified field._
///
///@param field
///       An existing MIP field in a packet. Can be invalid, in which case the
///       result will also be invalid.
///
///@returns A mip_field struct referencing the next field after the input
///         field._ Check mip_field_is_valid() to see if the field exists.
///
mip_field mip_field_next_after(const mip_field* field)
{
    // Payload length must be zero if payload is NULL.
    assert(!(field->_payload == NULL) || (field->_payload_length == 0));

    const uint8_t* next_header = field->_payload + field->_payload_length;

    return mip_field_from_header_ptr(next_header, field->_remaining_length, field->_descriptor_set);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Updates the %mip_field to refer to the next field in a packet.
///
///@param field
///       This %mip_field struct will be updated to the next field._ Can be an
///       invalid field, in which case the result will be invalid as well.
///
///@returns true if the field exists and is valid.
///
bool mip_field_next(mip_field* field)
{
    *field = mip_field_next_after(field);

    return mip_field_is_valid(field);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Iterates over all fields in a packet.
///
///@param field
///       An initialized mip_field struct. This will be updated to the next
///       field in the packet, if one exists. Otherwise it will be invalid.
///@param packet
///       A valid MIP packet.
///
///@returns true if another field exists.
///@returns false if there are no more fields.
///
///Example usage:
///@code{.cpp}
/// struct mip_field field;
/// mip_field_init_empty(&field);
/// while( mip_field_next_in_packet(&field, packet) )
/// {
///   // Do something with the field.
/// }
///@endcode
///
bool mip_field_next_in_packet(mip_field* field, const mip_packet* packet)
{
    if( field->_descriptor_set != MIP_INVALID_DESCRIPTOR_SET )
        *field = mip_field_next_after(field);
    else
        *field = mip_field_first_from_packet(packet);

    return mip_field_is_valid(field);
}

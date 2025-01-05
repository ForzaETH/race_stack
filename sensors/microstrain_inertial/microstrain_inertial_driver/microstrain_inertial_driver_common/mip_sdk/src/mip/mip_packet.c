
#include "mip_packet.h"
#include "mip_offsets.h"

#include "definitions/descriptors.h"

#include <string.h>
#include <assert.h>


//
// Initialization
//

////////////////////////////////////////////////////////////////////////////////
///@brief Initializes a MIP packet from an existing buffer.
///
/// Use this when receiving or parsing MIP packets.
///
/// The data in the buffer should be a valid or suspected MIP packet.
///
///@param packet
///@param buffer
///       The data buffer containing the bytes for a MIP packet. Must be at
///       least MIP_PACKET_LENGTH_MIN bytes in size.
///@param length
///       The length of the data pointed to by buffer.
///
///@note The data does not need to be a valid MIP packet, for instance to use
///      the mip_packet_is_sane() or mip_packet_is_valid() functions. However, if
///      it is NOT a valid MIP packet, the result of calling any accessor
///      function is unpredictable. In particular, if length is less than
///      MIP_PACKET_LENGTH_MIN bytes, calling the accessor functions is undefined
///      behavior.
///
void mip_packet_from_buffer(mip_packet* packet, uint8_t* buffer, size_t length)
{
    assert(buffer != NULL);

    // Limit the length in case it's longer than a mip packer (or worse, longer than the buffer size field can hold)
    if( length > MIP_PACKET_LENGTH_MAX )
        length = MIP_PACKET_LENGTH_MAX;

    packet->_buffer        = buffer;
    packet->_buffer_length = (uint_least16_t)length;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Create a brand-new MIP packet in the given buffer.
///
/// Use this along with the packet building functions to create MIP packets.
///
///@param packet
///@param buffer
///       This is where the packet bytes will be stored. Must be at least
///       MIP_PACKET_LENGTH_MIN bytes in size.
///@param buffer_size
///       The size of buffer, in bytes.
///@param descriptor_set
///       The MIP descriptor set for the packet.
///
void mip_packet_create(mip_packet* packet, uint8_t* buffer, size_t buffer_size, uint8_t descriptor_set)
{
    mip_packet_from_buffer(packet, buffer, buffer_size);

    if( buffer_size < MIP_PACKET_LENGTH_MIN )
    {
        assert(false); // Buffer too small!
        return;
    }

    packet->_buffer[MIP_INDEX_SYNC1]   = MIP_SYNC1;
    packet->_buffer[MIP_INDEX_SYNC2]   = MIP_SYNC2;
    packet->_buffer[MIP_INDEX_DESCSET] = descriptor_set;
    packet->_buffer[MIP_INDEX_LENGTH]  = 0;
}



//
// Accessors
//

////////////////////////////////////////////////////////////////////////////////
///@brief Returns the MIP descriptor set for this packet.
///
uint8_t mip_packet_descriptor_set(const mip_packet* packet)
{
    return packet->_buffer[MIP_INDEX_DESCSET];
}

////////////////////////////////////////////////////////////////////////////////
///@brief Returns the length of the payload (MIP fields).
///
uint8_t mip_packet_payload_length(const mip_packet* packet)
{
    return packet->_buffer[MIP_INDEX_LENGTH];
}

////////////////////////////////////////////////////////////////////////////////
///@brief Returns the total length of the packet, in bytes.
///
///@returns The length of the packet. Always at least MIP_PACKET_LENGTH_MIN.
///
uint_least16_t mip_packet_total_length(const mip_packet* packet)
{
    return mip_packet_payload_length(packet) + MIP_PACKET_LENGTH_MIN;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Returns a writable pointer to the data buffer.
///
uint8_t* mip_packet_buffer(mip_packet* packet)
{
    return packet->_buffer;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Returns a pointer to the data buffer containing the packet.
///
const uint8_t* mip_packet_pointer(const mip_packet* packet)
{
    return packet->_buffer;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Returns a pointer to the packet's payload (the first field).
///
const uint8_t* mip_packet_payload(const mip_packet* packet)
{
    return packet->_buffer + MIP_INDEX_PAYLOAD;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Returns the value of the checksum as written in the packet.
///
/// This function does not compute the checksum. To do so, use
/// mip_packet_compute_checksum().
///
uint16_t mip_packet_checksum_value(const mip_packet* packet)
{
    const uint_least16_t index = mip_packet_total_length(packet) - MIP_CHECKSUM_LENGTH;

    return ((uint16_t)(packet->_buffer[index+0]) << 8) | (uint16_t)(packet->_buffer[index+1]);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Computes the checksum of the MIP packet.
///
///@returns The computed checksum value.
///
uint16_t mip_packet_compute_checksum(const mip_packet* packet)
{
    uint8_t a = 0;
    uint8_t b = 0;

    // mip_packet_total_length always returns at least MIP_PACKET_LENGTH_MIN so this
    // subtraction is guaranteed to be safe.
    const uint_least16_t length = mip_packet_total_length(packet) - MIP_CHECKSUM_LENGTH;

    for(uint_least16_t i=0; i<length; i++)
    {
        a += packet->_buffer[i];
        b += a;
    }

    return ((uint16_t)(a) << 8) | (uint16_t)(b);
}


////////////////////////////////////////////////////////////////////////////////
///@brief Returns true if the packet buffer is not NULL and is at least the
///       minimum size (MIP_PACKET_LENGTH_MIN).
///
/// If the packet is not 'sane', then none of the mip_packet_* functions may be
/// used to access it (to do so is undefined behavior). This should never occur
/// in normal circumstances.
///
bool mip_packet_is_sane(const mip_packet* packet)
{
    return packet->_buffer && (packet->_buffer_length >= MIP_PACKET_LENGTH_MIN) && (packet->_buffer_length >= mip_packet_payload_length(packet)+MIP_PACKET_LENGTH_MIN);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Returns true if the packet is valid.
///
/// A packet is valid if:
/// * mip_packet_is_sane() returns true,
/// * The descriptor set is not 0x00, and
/// * The checksum is valid.
///
bool mip_packet_is_valid(const mip_packet* packet)
{
    if( !mip_packet_is_sane(packet) || (mip_packet_descriptor_set(packet) == 0x00) )
        return false;

    const uint16_t listed_checksum = mip_packet_checksum_value(packet);
    const uint16_t computed_checksum = mip_packet_compute_checksum(packet);

    return listed_checksum == computed_checksum;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Returns true if the mip packet contains no payload.
///
///@param packet
///
///@returns true if the packet has a payload length of 0.
///
bool mip_packet_is_empty(const mip_packet* packet)
{
    if( !mip_packet_is_sane(packet) )
        return true;

    return mip_packet_payload_length(packet) == 0;
}


////////////////////////////////////////////////////////////////////////////////
///@brief Returns the size of the buffer backing the MIP packet.
///
///@note This is the BUFFER SIZE and not the packet length.
///
uint_least16_t mip_packet_buffer_size(const mip_packet* packet)
{
    return packet->_buffer_length;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Returns the remaining space available for more payload data.
///
/// This is equal to the buffer size less the total packet length.
///
///@warning The result may be negative if the packet length exceeds the actual
///         buffer capacity. Such packets are not 'sane' (mip_packet_is_sane)
///         and can only be produced by manipulating the buffered data directly.
///
int mip_packet_remaining_space(const mip_packet* packet)
{
    return mip_packet_buffer_size(packet) - mip_packet_total_length(packet);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Returns true if the packet is from a data descriptor set.
///
///@see is_data_descriptor_set
///
///@returns true if the packet contains data.
///@returns false if it contains commands or replies.
///
bool mip_packet_is_data(const mip_packet* packet)
{
    return mip_is_data_descriptor_set(mip_packet_descriptor_set(packet));
}

//
// Packet Building
//

////////////////////////////////////////////////////////////////////////////////
///@brief Adds a pre-constructed MIP field to the packet.
///
///~~~
///                                             +--------------------+
///                                             |  Payload Bytes     |
///                               Len  Desc     +--------------------+
///                                |    |        | copy
///  Packet Buffer                 V    V        V
/// ---------------+------------+-   -+-    -+--                  -+-
///  ...   Header  |   Field    | Len | Desc |  Payload            |
/// ---------------+------------+-----+------+---------------------+----------
///                             |                                  |
///                     End of last field   --------------->  End of new field
///~~~
///
///
///@param packet
///@param field_descriptor
///       The MIP field descriptor (e.g. command or data descriptor).
///@param payload
///       A pointer to the field payload data (without the header).
///       Can be NULL if payload_length is 0.
///@param payload_length
///       The length of the payload data. Must be less than or equal to
///       MIP_FIELD_PAYLOAD_LENGTH_MAX. Does not include the header.
///
///@returns true if the field was added, or false if there was not enough space.
///
bool mip_packet_add_field(mip_packet* packet, uint8_t field_descriptor, const uint8_t* payload, uint8_t payload_length)
{
    uint8_t* payload_buffer;
    int remaining = mip_packet_alloc_field(packet, field_descriptor, payload_length, &payload_buffer);
    if( remaining < 0 )
        return false;

    memcpy(payload_buffer, payload, payload_length);

    return true;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Allocate a MIP field within the packet and return the payload pointer.
///
///~~~
///                               Len   Desc  .---> Payload ptr out
///                                |     |    |
///  Packet Buffer                 V     V    |
/// ---------------+------------+-   -+-    -+---------------------+----------
///  ...   Header  |   Field    | Len | Desc | (unwritten payload) |
/// ---------------+------------+-----+------+---------------------+----------
///                             |                                  |
///                     End of last field   --------------->  End of new field
///~~~
///
///@param packet
///@param field_descriptor
///       The MIP field descriptor (e.g. command or data descriptor).
///@param payload_length
///       The requested length of the field payload (not including the header).
///       If the size is not known ahead of time, pass 0 and inspect the return
///       value to see how much payload data can be written. Then call
///       mip_packet_realloc_field() with the used size and same payload pointer.
///@param payload_ptr_out
///       A pointer to a pointer to the field payload. This will receive the
///       payload pointer into which data should be written.
///
///@returns The amount of space remaining after allocating this field. If this
///         is negative, the field could not be allocated and the payload must
///         not be written.
///
int mip_packet_alloc_field(mip_packet* packet, uint8_t field_descriptor, uint8_t payload_length, uint8_t** const payload_ptr_out)
{
    assert(payload_ptr_out != NULL);
    assert( payload_length <= MIP_FIELD_PAYLOAD_LENGTH_MAX );

    const int remaining = mip_packet_remaining_space(packet);

    const uint8_t field_length = MIP_FIELD_HEADER_LENGTH + payload_length;

    *payload_ptr_out = NULL;

    if( field_length <= remaining )
    {
        const uint_least16_t field_index = MIP_HEADER_LENGTH + mip_packet_payload_length(packet);

        packet->_buffer[MIP_INDEX_LENGTH] += field_length;

        packet->_buffer[field_index+MIP_INDEX_FIELD_LEN]  = field_length;
        packet->_buffer[field_index+MIP_INDEX_FIELD_DESC] = field_descriptor;

        *payload_ptr_out = &packet->_buffer[field_index + MIP_INDEX_FIELD_PAYLOAD];
    }

    return remaining - field_length;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Changes the size of the last field in the packet.
///
/// Use this in conjunction with mip_packet_alloc_field() when the size of the
/// field is not known in advance. Pass a payload size of 0 to alloc_field and
/// check that the returned available space is sufficient, then write the
/// payload and call this function with the actual space used.
///
///@param packet
///@param payload_ptr
///       Pointer to the field payload. This must be the same value returned
///       from alloc_field and must point to the last field.
///@param new_payload_length
///       Length of payload written. Generally it is an error for this to
///       exceed the actual amount of space available in the packet. In this
///       case, the packet is left with just the empty field and the return
///       value will be negative.
///
///@returns The space remaining in the packet after changing the field size.
///         This will be negative if the new length did not fit.
///
int mip_packet_realloc_last_field(mip_packet* packet, uint8_t* payload_ptr, uint8_t new_payload_length)
{
    assert(payload_ptr != NULL);
    assert( new_payload_length <= MIP_FIELD_PAYLOAD_LENGTH_MAX );

    uint8_t* field_ptr = payload_ptr - MIP_INDEX_FIELD_PAYLOAD;
    const uint8_t old_field_length = field_ptr[MIP_INDEX_FIELD_LEN];
    const uint8_t new_field_length = new_payload_length + MIP_FIELD_HEADER_LENGTH;

    const int delta_length = new_field_length - old_field_length;

    const int remaining = mip_packet_remaining_space(packet) - delta_length;

    if( remaining >= 0 )
    {
        field_ptr[MIP_INDEX_FIELD_LEN] = new_field_length;
        packet->_buffer[MIP_INDEX_LENGTH] += (int8_t)delta_length;
    }

    return remaining;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Removes the last field from the packet after having allocated it.
///
/// Use only after allocating a field with mip_packet_alloc_field to cancel it.
/// E.g. if it turns out that there isn't enough buffer space to write the
/// payload.
///
///@param packet
///@param payload_ptr
///       Pointer to the field payload. This must be the same value returned
///       from alloc_field and must point to the last field.
///
///@returns The remaining space in the packet after removing the field.
///
int mip_packet_cancel_last_field(mip_packet* packet, uint8_t* payload_ptr)
{
    assert(payload_ptr != NULL);

    uint8_t* field_ptr = payload_ptr - MIP_INDEX_FIELD_PAYLOAD;
    const uint8_t old_field_length = field_ptr[MIP_INDEX_FIELD_LEN];

    packet->_buffer[MIP_INDEX_LENGTH] -= old_field_length;

    return mip_packet_remaining_space(packet);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Prepares the packet for transmission by adding the checksum.
///
/// This does not increase the total packet length since the checksum is always
/// implicitly included.
///
///~~~
///                                                          Checksum
///                                                            VVVV
/// ---------------+------------+------------+-----//-----+--        --+----
///  ...   Header  |   Field    |   Field    |     ...    |  (empty)   |
/// ---------------+------------+------------+-----//-----+------------+----
///                                                       |            |
///                                            End of last field       |
///                                                               Total Length
///~~~
///
void mip_packet_finalize(mip_packet* packet)
{
    uint16_t checksum = mip_packet_compute_checksum(packet);
    uint_least16_t length = mip_packet_total_length(packet) - MIP_CHECKSUM_LENGTH;

    packet->_buffer[length+0] = checksum >> 8;
    packet->_buffer[length+1] = checksum & 0xFF;
}


////////////////////////////////////////////////////////////////////////////////
///@brief Reinitialize the packet with the given descriptor set.
///
/// This clears out all of the fields but keeps the same buffer.
///
///@param packet
///@param descriptor_set New descriptor set.
///
void mip_packet_reset(mip_packet* packet, uint8_t descriptor_set)
{
    mip_packet_create(packet, mip_packet_buffer(packet), mip_packet_buffer_size(packet), descriptor_set);
}


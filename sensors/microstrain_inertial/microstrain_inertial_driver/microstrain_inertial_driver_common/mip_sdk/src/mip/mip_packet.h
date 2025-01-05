#pragma once

#include "mip_types.h"

#ifdef __cplusplus
namespace mip{
namespace C {
extern "C" {
#endif


////////////////////////////////////////////////////////////////////////////////
///@addtogroup mip_c
///@{

////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_packet_c Mip Packet [C]
///
///@brief Functions for handling MIP packets.
///
/// A MIP Packet is represented by the mip_packet struct.
///
///~~~
/// +-------+-------+------+------+------------+-----/ /----+------------+----
/// | SYNC1 | SYNC2 | DESC | PLEN |   Field    |     ...    |  Checksum  |  remaining buffer space...
/// +-------+-------+------+------+------------+-----/ /----+------------+----
///~~~
///
///@{


////////////////////////////////////////////////////////////////////////////////
///@brief Structure representing a MIP Packet.
///
/// Use to inspect received packets or construct new ones.
///
///@note This should be considered an "opaque" structure; its members should be
/// considered an internal implementation detail. Avoid accessing them directly
/// as they are subject to change in future versions of this software.
///
typedef struct mip_packet
{
    uint8_t*       _buffer;         ///<@private Pointer to the packet data.
    uint_least16_t _buffer_length;  ///<@private Length of the buffer (NOT the packet length!).
} mip_packet;


////////////////////////////////////////////////////////////////////////////////
///@defgroup PacketBuilding  Packet Building [C]
///
///@brief Functions for building new MIP packets.
///
/// Use these functions to create a new packet, add fields, and write the
/// checksum.
///
///@{

void mip_packet_create(mip_packet* packet, uint8_t* buffer, size_t buffer_size, uint8_t descriptor_set);

bool mip_packet_add_field(mip_packet* packet, uint8_t field_descriptor, const uint8_t* payload, uint8_t payload_length);
int  mip_packet_alloc_field(mip_packet* packet, uint8_t field_descriptor, uint8_t payload_length, uint8_t** payload_ptr_out);
int  mip_packet_realloc_last_field(mip_packet* packet, uint8_t* payload_ptr, uint8_t new_payload_length);
int  mip_packet_cancel_last_field(mip_packet* packet, uint8_t* payload_ptr);

void mip_packet_finalize(mip_packet* packet);

void mip_packet_reset(mip_packet* packet, uint8_t descriptor_set);

///@}
////////////////////////////////////////////////////////////////////////////////
///@defgroup Accessors  Accessors - Functions for accessing information about an existing MIP packet.
///
/// Use these functions to get information about a MIP packet after it has been
/// parsed. Generally, first the descriptor set would be inspected followed by
/// iterating the fields using the MipFieldIteration functions.
///
/// With the exception of mip_packet_checksum_value() (and any function which
/// calls it, e.g. mip_packet_is_valid()), these functions may also be used on
/// packets which are under construction via the PacketBuilding functions.
///
///@{

void mip_packet_from_buffer(mip_packet* packet, uint8_t* buffer, size_t length);

uint8_t         mip_packet_descriptor_set(const mip_packet* packet);
uint_least16_t  mip_packet_total_length(const mip_packet* packet);
uint8_t         mip_packet_payload_length(const mip_packet* packet);
uint8_t*        mip_packet_buffer(mip_packet* packet);
const uint8_t*  mip_packet_pointer(const mip_packet* packet);
const uint8_t*  mip_packet_payload(const mip_packet* packet);
uint16_t        mip_packet_checksum_value(const mip_packet* packet);
uint16_t        mip_packet_compute_checksum(const mip_packet* packet);


bool            mip_packet_is_sane(const mip_packet* packet);
bool            mip_packet_is_valid(const mip_packet* packet);
bool            mip_packet_is_empty(const mip_packet* packet);

uint_least16_t  mip_packet_buffer_size(const mip_packet* packet);
int             mip_packet_remaining_space(const mip_packet* packet);

bool            mip_packet_is_data(const mip_packet* packet);

///@}
///@}
///@}
////////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
} // namespace mip
} // namespace C
} // extern "C"
#endif

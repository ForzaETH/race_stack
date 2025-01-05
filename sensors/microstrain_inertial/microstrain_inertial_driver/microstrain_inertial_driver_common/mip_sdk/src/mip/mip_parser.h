#pragma once

#include "mip_packet.h"
#include "mip_offsets.h"

#include "utils/byte_ring.h"
#include "mip_types.h"

#ifdef __cplusplus
namespace mip{
namespace C {
extern "C" {
#endif

////////////////////////////////////////////////////////////////////////////////
///@addtogroup mip_c  MIP C API
///
///@brief This module contains all of the C submodules.
///
///@{


////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_parser_c mip_parser [C]
///
///@brief Functions for parsing MIP packets
///
///@{
///
/// See @ref parsing_packets
///
/// Typical usage:
///@li Declare a mip_parser struct
///@li Declare a uint8_t buffer of some size which is at least 512 and a power of 2.
///@li Determine the packet timeout, e.g. with mip_timeout_from_baudrate().
///@li Call mip_parser_init(), passing the struct, buffer, buffer size, timeout, and callback function.
///@li Periodically call mip_parser_parse().
///


///@brief Callback function which receives parsed MIP packets.
///@param user A user-specified pointer which will be given the callback_object parameter which was previously passed to mip_parser_init.
///@param packet A pointer to the MIP packet. Do not store this pointer as it will be invalidated after the callback returns.
///@param timestamp The approximate time the packet was parsed.
typedef bool (*mip_packet_callback)(void* user, const mip_packet* packet, mip_timestamp timestamp);


////////////////////////////////////////////////////////////////////////////////
///@brief MIP Parser state.
///
///@note This should be considered an "opaque" structure; its members should be
/// considered an internal implementation detail. Avoid accessing them directly
/// as they are subject to change in future versions of this software.
///
typedef struct mip_parser
{
    mip_timestamp       _start_time;                           ///<@private The timestamp when the first byte was observed by the parser.
    mip_timestamp       _timeout;                              ///<@private Duration to wait for the rest of the data in a packet.
    uint8_t             _result_buffer[MIP_PACKET_LENGTH_MAX]; ///<@private Buffer used to output MIP packets to the callback.
    uint16_t            _expected_length;                      ///<@private Expected length of the packet currently being parsed. Keeps track of parser state. Always 1, MIP_HEADER_LENGTH, or at least MIP_PACKET_LENGTH_MAX.
    byte_ring_state     _ring;                                 ///<@private Ring buffer which holds data being parsed. User-specified backing buffer and size.
    mip_packet_callback _callback;                             ///<@private Callback called when a valid packet is parsed. Can be NULL.
    void*               _callback_object;                      ///<@private User-specified pointer passed to the callback function.

#ifdef MIP_ENABLE_DIAGNOSTICS
    uint32_t            _diag_bytes_read;                      ///<@private Counts bytes read from the user input buffer.
    uint32_t            _diag_bytes_skipped;                   ///<@private Counts bytes read from the user input buffer.
    uint32_t            _diag_packet_bytes;                    ///<@private Counts bytes parsed into valid packets.
    uint32_t            _diag_valid_packets;                   ///<@private Counts packets successfully parsed.
    uint32_t            _diag_invalid_packets;                 ///<@private Counts invalid packets encountered (bad checksums).
    uint32_t            _diag_timeouts;                        ///<@private Counts packet timeouts.
#endif // MIP_ENABLE_DIAGNOSTICS

} mip_parser;



#define MIPPARSER_UNLIMITED_PACKETS   0   ///< Specifies no limit when used as the max_packets argument to mip_parser_parse.
#define MIPPARSER_DEFAULT_TIMEOUT_MS 100  ///< Specifies the default timeout for a MIP parser, assuming timestamps are in milliseconds.


void mip_parser_init(mip_parser* parser, uint8_t* buffer, size_t buffer_size, mip_packet_callback callback, void* callback_object, mip_timestamp timeout);
bool mip_parser_parse_one_packet_from_ring(mip_parser* parser, mip_packet* packet_out, mip_timestamp timestamp);
size_t mip_parser_parse(mip_parser* parser, const uint8_t* input_buffer, size_t input_count, mip_timestamp timestamp, unsigned int max_packets);

void mip_parser_reset(mip_parser* parser);

size_t mip_parser_get_write_ptr(mip_parser* parser, uint8_t** ptr_out);
void mip_parser_process_written(mip_parser* parser, size_t count, mip_timestamp timestamp, unsigned int max_packets);

//
// Accessors
//

mip_timeout mip_parser_timeout(const mip_parser* parser);
void mip_parser_set_timeout(mip_parser* parser, mip_timeout timeout);

void mip_parser_set_callback(mip_parser* parser, mip_packet_callback callback, void* callback_object);
mip_packet_callback mip_parser_callback(const mip_parser* parser);
void* mip_parser_callback_object(const mip_parser* parser);

mip_timestamp mip_parser_last_packet_timestamp(const mip_parser* parser);


//
// Diagnostics
//

#ifdef MIP_ENABLE_DIAGNOSTICS

uint32_t mip_parser_diagnostic_bytes_read(const mip_parser* parser);
uint32_t mip_parser_diagnostic_bytes_skipped(const mip_parser* parser);
uint32_t mip_parser_diagnostic_packet_bytes(const mip_parser* parser);

uint32_t mip_parser_diagnostic_valid_packets(const mip_parser* parser);
uint32_t mip_parser_diagnostic_invalid_packets(const mip_parser* parser);
uint32_t mip_parser_diagnostic_timeouts(const mip_parser* parser);

#endif // MIP_ENABLE_DIAGNOSTICS


//
// Misc
//

mip_timeout mip_timeout_from_baudrate(uint32_t baudrate);

///@}
///@}
////////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
} // namespace mip
} // namespace C
} // extern "C"
#endif

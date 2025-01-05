
#include "mip_parser.h"

#include "mip_offsets.h"

#include <assert.h>


#define MIPPARSER_RESET_LENGTH 1

////////////////////////////////////////////////////////////////////////////////
///@brief Initializes the MIP parser.
///
///
///@param parser
///@param buffer
///       Scratch space for the parser to use internally; input data is consumed
///       and fed to this buffer. Cannot be NULL.
///@param buffer_size
///       Size of buffer, in bytes.
///@param callback
///       A function to be called when a valid packet is identified. It will be
///       passed an optional user-supplied parameter, a pointer to the packet,
///       and the time the first byte was parsed.
///@param callback_object
///       An optional user-specified pointer which is directly passed to
///       the callback as the first parameter.
///@param timeout
///       The timeout for receiving one packet. Depends on the serial baud rate
///       and is typically 100 milliseconds.
///
void mip_parser_init(mip_parser* parser, uint8_t* buffer, size_t buffer_size, mip_packet_callback callback, void* callback_object, mip_timestamp timeout)
{
    byte_ring_init(&parser->_ring, buffer, buffer_size);

    parser->_timeout = timeout;

    mip_parser_reset(parser);

    parser->_callback = callback;
    parser->_callback_object = callback_object;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Resets the MIP parser.
///
/// Clears the current packet and internal buffer. The parser will be restored
/// as if mip_parser_init had just been called.
///
///
///@param parser
///
void mip_parser_reset(mip_parser* parser)
{
    parser->_expected_length = MIPPARSER_RESET_LENGTH;
    parser->_result_buffer[0] = 0;
    parser->_start_time = 0;
    byte_ring_clear(&parser->_ring);

    MIP_DIAG_ZERO(parser->_diag_bytes_read);
    MIP_DIAG_ZERO(parser->_diag_bytes_skipped);
    MIP_DIAG_ZERO(parser->_diag_packet_bytes);
    MIP_DIAG_ZERO(parser->_diag_valid_packets);
    MIP_DIAG_ZERO(parser->_diag_invalid_packets);
    MIP_DIAG_ZERO(parser->_diag_timeouts);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Parses packets from the input data buffer.
///
/// For every valid MIP packet, the callback function will be called with the
/// packet and timestamp.
///
///
///@param parser
///@param input_buffer
///       Pointer to bytes received from the device or file. This buffer may
///       contain non-mip data (e.g. NMEA 0183), which will be ignored.
///       This buffer may be NULL if input_count is 0.
///@param input_count
///       The number of bytes in the input buffer.
///@param timestamp
///       The local time the data was received. This is used to check for
///       timeouts and is passed to the callback as the packet timestamp.
///@param max_packets
///       The maximum number of packets to process. Unprocessed data is left in
///       the internal buffer. If 0, processing runs until no complete packets
///       remain in the buffer.
///
///@returns The number of bytes left unprocessed from the input buffer.
///         If max_packets is 0, this will also be zero as all of the data will
///         be consumed. Data may still remain in the internal buffer.
///
///@note If max_packets is 0, then this function is guaranteed to consume all
///      of the input data and the buffer can be reused or discarded afterward.
///      However, if max_packets is nonzero (meaning the number of packets parsed
///      will be limited), then this is no longer guaranteed as the excess data
///      may fill up the internal bufffer. In this case, you must process packets
///      faster than they arrive on average. For bursty data (e.g. GNSS data),
///      use a large internal buffer (see mip_parser_init) to help average out
///      the packet processing load.
///
///@note The timestamp of parsed packets is based on the time the packet was
///      parsed. When max_packets==0, this is the same as the input timestamp.
///      When max_packets!=0, packets received during an earlier parse call
///      may be timestamped with the time from a later parse call. Therefore,
///      if packet timestamping is critical to your application, avoid using
///      max_packets > 0.
///
///@note The parser will do its best to ignore non-MIP data. However, it is
///      possible for some binary data to appear to be a MIP packet if it
///      conntains 0x75,0x65, has at least 6 bytes, and has a valid checksum. A
///      16-bit checksum has a 1 in 65,536 chance of appearing to be valid.
///
size_t mip_parser_parse(mip_parser* parser, const uint8_t* input_buffer, size_t input_count, mip_timestamp timestamp, unsigned int max_packets)
{
    // Reset the state if the timeout time has elapsed.
    if( parser->_expected_length != MIPPARSER_RESET_LENGTH && (timestamp - parser->_start_time) > parser->_timeout )
    {
        if( byte_ring_count(&parser->_ring) > 0 )
        {
            byte_ring_pop(&parser->_ring, 1);
            MIP_DIAG_INC(parser->_diag_bytes_skipped, 1);
        }

        parser->_expected_length = MIPPARSER_RESET_LENGTH;

        MIP_DIAG_INC(parser->_diag_timeouts, 1);
    }

    unsigned int num_packets = 0;
    do
    {
        // Copy as much data as will fit in the ring buffer.
        size_t count = byte_ring_copy_from_and_update(&parser->_ring, &input_buffer, &input_count);

        MIP_DIAG_INC(parser->_diag_bytes_read, count);

        mip_packet packet;
        while( mip_parser_parse_one_packet_from_ring(parser, &packet, timestamp) )
        {
            num_packets++;
            bool stop = (max_packets > 0) && (num_packets >= max_packets);

            if( parser->_callback )
                stop |= !parser->_callback(parser->_callback_object, &packet, parser->_start_time);

            if( stop )
            {
                // Pull more data from the input buffer if possible.
                count = byte_ring_copy_from_and_update(&parser->_ring, &input_buffer, &input_count);

                MIP_DIAG_INC(parser->_diag_bytes_read, count);

                return input_count;
            }
        }

        // Need more data to continue parsing.
        // This code assumes the ring buffer is large enough for any single
        // received mip packet, otherwise it will get stuck in an infinite loop.

    } while( input_count );

    return input_count;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Parses a single packet from the internal buffer.
///
///@internal
///
///@param parser
///@param packet_out
///       The mip packet to initialize with a valid packet, if found.
///@param timestamp
///       Time of the most recently received data.
///
///@returns true if a packet was found, false if more data is required. If false,
///         the packet is not initialized.
///
bool mip_parser_parse_one_packet_from_ring(mip_parser* parser, mip_packet* packet_out, mip_timestamp timestamp)
{
    // Parse packets while there is sufficient data in the ring buffer.
    while( byte_ring_count(&parser->_ring) >= parser->_expected_length )
    {
        if( parser->_expected_length == MIPPARSER_RESET_LENGTH )
        {
            if( byte_ring_at(&parser->_ring, MIP_INDEX_SYNC1) != MIP_SYNC1 )
            {
                byte_ring_pop(&parser->_ring, 1);

                MIP_DIAG_INC(parser->_diag_bytes_skipped, 1);
            }
            else
            {
                // Synchronized - set the start time and expect more data.
                parser->_start_time = timestamp;
                parser->_expected_length = MIP_HEADER_LENGTH;
            }
        }
        else if( parser->_expected_length == MIP_HEADER_LENGTH )
        {
            // Check the sync bytes and drop a single byte if not sync'd.
            if( byte_ring_at(&parser->_ring, MIP_INDEX_SYNC2) != MIP_SYNC2 )
            {
                byte_ring_pop(&parser->_ring, 1);
                MIP_DIAG_INC(parser->_diag_bytes_skipped, 1);
                parser->_expected_length = MIPPARSER_RESET_LENGTH;
            }
            else
            {
                // Read the payload length and add it and the checksum size to the complete packet size.
                parser->_expected_length += byte_ring_at(&parser->_ring, MIP_INDEX_LENGTH) + MIP_CHECKSUM_LENGTH;
            }
        }
        else // Just waiting on enough data
        {
            uint_least16_t packet_length = parser->_expected_length;
            parser->_expected_length = MIPPARSER_RESET_LENGTH;  // Reset parsing state

            byte_ring_copy_to(&parser->_ring, parser->_result_buffer, packet_length);

            mip_packet_from_buffer(packet_out, parser->_result_buffer, packet_length);

            if( !mip_packet_is_valid(packet_out) )
            {
                // Invalid packet, drop just the first sync byte and restart.
                byte_ring_pop(&parser->_ring, 1);
                MIP_DIAG_INC(parser->_diag_bytes_skipped, 1);
                MIP_DIAG_INC(parser->_diag_invalid_packets, 1);
            }
            else // Checksum is valid
            {
                // Discard the packet bytes from the ring buffer since a copy was made.
                byte_ring_pop(&parser->_ring, packet_length);

                MIP_DIAG_INC(parser->_diag_valid_packets, 1);
                MIP_DIAG_INC(parser->_diag_packet_bytes, packet_length);

                // Successfully parsed a packet.
                return true;
            }
        }
    }

    // Need more data to continue.

    return false;
}


////////////////////////////////////////////////////////////////////////////////
///@brief Returns the packet timeout of the parser.
///
///
mip_timestamp mip_parser_timeout(const mip_parser* parser)
{
    return parser->_timeout;
}


////////////////////////////////////////////////////////////////////////////////
///@brief Changes the timeout of the MIP parser.
///
///@param parser
///@param timeout
///
void mip_parser_set_timeout(mip_parser* parser, mip_timestamp timeout)
{
    parser->_timeout = timeout;
}

////////////////////////////////////////////////////////////////////////////////
///@brief mip_parser_set_callback
///
///@param parser
///@param callback
///@param callback_object
///
void mip_parser_set_callback(mip_parser* parser, mip_packet_callback callback, void* callback_object)
{
    parser->_callback_object = callback_object;
    parser->_callback        = callback;
}

////////////////////////////////////////////////////////////////////////////////
///@brief mip_parser_callback
///
///@param parser
///
///@returns the packet callback function.
///
mip_packet_callback mip_parser_callback(const mip_parser* parser)
{
    return parser->_callback;
}


////////////////////////////////////////////////////////////////////////////////
///@brief mip_parser_callback
///
///@param parser
///
///@returns the packet callback user data pointer.
///
void* mip_parser_callback_object(const mip_parser* parser)
{
    return parser->_callback_object;
}


////////////////////////////////////////////////////////////////////////////////
///@brief Gets the timestamp of the last parsed packet.
///
/// This is only valid after a valid packet has been parsed.
///
/// This function is provided to allow additional calls to mip_parser_parse()
/// with no input data (buffer=NULL and length=0) when max_packets > 0. The
/// additional calls can use the same timestamp because no new data will be
/// processed.
///
/// There are two possible situations after the last call to parse:
/// 1. Either max_packets was reached, meaning at least one packet was parsed, and
///    thus the timestamp is valid, or
/// 2. More data is required, in which case this time may not be valid, but it
///    won't matter because an additional call to parse won't produce a new
///    packet to be timestamped.
///
mip_timestamp mip_parser_last_packet_timestamp(const mip_parser* parser)
{
    return parser->_start_time;
}


////////////////////////////////////////////////////////////////////////////////
///@brief Obtain a pointer into which data may be read for processing.
///
/// Use this function when the source data stream (e.g. a file or serial port)
/// requires that you pass in a buffer when reading data. This avoids the need
/// for an intermediate buffer.
///
/// Call mip_parser_process_written() after the data has been read to update the
/// buffer count and process any packets.
///
///@code{.cpp}
/// uint8_t ptr;
/// size_t space = mip_parser_get_write_ptr(&parser, &ptr);
/// size_t used = fread(ptr, 1, space, file);
/// mip_parser_process_written(&parser, used);
///@endcode
///
///@param parser
///@param ptr_out
///       A pointer to a pointer which will be set to the buffer where data
///       should be written. Cannot be NULL.
///
///@returns How many bytes can be written to the buffer. Due to the use of a
///         cicular buffer, this may be less than the total available buffer
///         space. Do not write more data than specified.
///
size_t mip_parser_get_write_ptr(mip_parser* parser, uint8_t** const ptr_out)
{
    assert(ptr_out != NULL);

    return byte_ring_get_write_ptr(&parser->_ring, ptr_out);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Notify the parser that data has been written to the pointer previously
///       obtained via mip_parser_get_write_ptr().
///
/// The write pointer changes after calling this with count > 0. To write more
/// data, call mip_parser_get_write_ptr again.
///
///@param parser
///@param count
///@param timestamp
///@param max_packets
///
void mip_parser_process_written(mip_parser* parser, size_t count, mip_timestamp timestamp, unsigned int max_packets)
{
    MIP_DIAG_INC(parser->_diag_bytes_read, count);

    byte_ring_notify_written(&parser->_ring, count);
    mip_parser_parse(parser, NULL, 0, timestamp, max_packets);
}


#ifdef MIP_ENABLE_DIAGNOSTICS

////////////////////////////////////////////////////////////////////////////////
///@brief Gets the total number of bytes read from the user input buffer.
///
/// This includes data read into the internal ring buffer but not yet seen by
/// the parser. Ensure all packets have been processed by the parser before
/// comparing against the packet_bytes counter.
///
uint32_t mip_parser_diagnostic_bytes_read(const mip_parser* parser)
{
    return parser->_diag_bytes_read;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Gets total the number of bytes that have been parsed into valid
///       packets.
///
/// This is a summation of the total length of every valid mip packet emitted by
/// the parser.
///
uint32_t mip_parser_diagnostic_packet_bytes(const mip_parser* parser)
{
    return parser->_diag_packet_bytes;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Gets the total number of bytes which weren't part of a valid packet.
///
/// This is the difference between the "packet bytes" and "bytes read" counters.
///
uint32_t mip_parser_diagnostic_bytes_skipped(const mip_parser* parser)
{
    return parser->_diag_bytes_skipped;
}


////////////////////////////////////////////////////////////////////////////////
///@brief Gets the total number of valid packets emitted by the parser.
///
uint32_t mip_parser_diagnostic_valid_packets(const mip_parser* parser)
{
    return parser->_diag_valid_packets;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Gets the total number of packets that failed the checksum check.
///
/// These invalid packets are not emitted by the parser and are not included in
/// the "valid packets" or "packet bytes" counters.
///
uint32_t mip_parser_diagnostic_invalid_packets(const mip_parser* parser)
{
    return parser->_diag_invalid_packets;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Gets the total number of times a packet timed out waiting for more
///       data.
///
/// Packets may time out under the following conditions:
///@li The connection is interrupted
///@li The length byte is corrupted to make the packet look longer
///@li The connection bandwidth and/or latency is too low
///
uint32_t mip_parser_diagnostic_timeouts(const mip_parser* parser)
{
    return parser->_diag_timeouts;
}

#endif // MIP_ENABLE_DIAGNOSTICS


////////////////////////////////////////////////////////////////////////////////
///@brief Computes an appropriate packet timeout for a given serial baud rate.
///
///@note This function assumes a standard serial port with 10 symbols per byte:
/// 1 start bit, 8 data bits, and 1 stop bit.
///
///@param baudrate Serial baud rate in bits per second
///
///@return A timeout value in ms representing the time it would take to transmit
///        a single mip packet of maximum size at the given baud rate, plus some
///        tolerance.
///
mip_timeout mip_timeout_from_baudrate(uint32_t baudrate)
{
    // num_symbols [b] = (packet_length [B]) * (10 [b/B])
    unsigned int num_symbols = MIP_PACKET_LENGTH_MAX * 10;

    // packet_time [s] = (num_symbols [b]) / (baudrate [b/s])

    // timeout [ms] = (packet_time [s]) * (1000 [ms/s]) * tolerance
    return num_symbols * 1500 / baudrate;
}


#include <mip/mip_packet.h>
#include <mip/mip_parser.h>

#include <stdio.h>
#include <stdlib.h>


uint8_t parse_buffer[1024];
struct mip_parser parser;

unsigned int num_errors = 0;

unsigned int num_packets_parsed = 0;
size_t        parsed_packet_length    = 0;
mip_timestamp parsed_packet_timestamp = 0;


void print_packet(FILE* out, const struct mip_packet* packet)
{
    size_t size = mip_packet_total_length(packet);
    const uint8_t* ptr = mip_packet_pointer(packet);

    for(size_t i=0; i<size; i++)
    {
        fprintf(out, " %02X", ptr[i]);
    }
    fputc('\n', out);
}


bool handle_packet(void* p, const struct mip_packet* packet, mip_timestamp timestamp)
{
    (void)p;

    num_packets_parsed++;
    parsed_packet_length = mip_packet_total_length(packet);
    parsed_packet_timestamp = timestamp;

    return true;
}


int main(int argc, const char* argv[])
{
    mip_parser_init(&parser, parse_buffer, sizeof(parse_buffer), &handle_packet, NULL, MIPPARSER_DEFAULT_TIMEOUT_MS);

    srand(0);

    const unsigned int NUM_ITERATIONS = 100;

    unsigned int last_parsed = 0;
    for(unsigned int i=0; i<NUM_ITERATIONS; i++)
    {
        uint8_t desc_set = (rand() % 255) + 1;  // Random descriptor set.

        uint8_t buffer[MIP_PACKET_LENGTH_MAX];
        struct mip_packet packet;
        mip_packet_create(&packet, buffer, sizeof(buffer), desc_set);

        for(unsigned int f=0; ; f++)
        {
            const size_t max_field_len = mip_packet_remaining_space(&packet);
            if( max_field_len < MIP_FIELD_HEADER_LENGTH )
                break;

            const uint8_t max_payload = max_field_len - MIP_FIELD_HEADER_LENGTH;

            const uint8_t paylen = (rand() % (max_payload+1)) >> (rand() % 8);

            const uint8_t field_desc = (rand() % 255) + 1;  // Random field descriptor.

            uint8_t* payload;
            int available = mip_packet_alloc_field(&packet, field_desc, paylen, &payload);

            if( available < 0 )
            {
                num_errors++;
                fprintf(stderr, "Failed to create field of length %d\n", paylen+MIP_FIELD_HEADER_LENGTH);
                fprintf(stderr, "  max_len=%ld, available=%d\n", max_field_len, available);
                break;
            }

            // Random payload.
            for(unsigned int p=0; p<paylen; p++)
                payload[p] = rand() & 0xFF;

            // Random chance of not adding another field.
            if( rand() % 5 == 0 )
                break;
        }

        mip_packet_finalize(&packet);

        //
        // Send this packet to the parser in small chunks.
        //

        const size_t packet_size = mip_packet_total_length(&packet);


        // Keep track of offsets and timestamps for debug purposes.
        size_t        offsets[MIP_PACKET_PAYLOAD_LENGTH_MAX / MIP_FIELD_HEADER_LENGTH]    = {0};
        mip_timestamp timestamps[MIP_PACKET_PAYLOAD_LENGTH_MAX / MIP_FIELD_HEADER_LENGTH] = {0};
        unsigned int  c                                                                   = 0;

        const mip_timestamp start_time = rand();
        mip_timestamp       timestamp  = start_time;
        size_t              sent       = 0;

        // Send all but the last chunk.
        while( sent < (packet_size-MIP_PACKET_LENGTH_MIN) )
        {
            const size_t count = rand() % (packet_size - sent);

            mip_parser_parse(&parser, mip_packet_pointer(&packet)+sent, count, timestamp, MIPPARSER_UNLIMITED_PACKETS);

            sent += count;
            timestamps[c] = timestamp;
            offsets[c++] = sent;

            // Don't bump timestamp if no data sent to avoid screwing up the test code later.
            if( count > 0 )
                timestamp += (rand() % mip_parser_timeout(&parser));
        }

        // Final chunk
        const size_t extra = 0;

        const size_t count = packet_size - sent;

        mip_parser_parse(&parser, mip_packet_pointer(&packet)+sent, count, timestamp, MIPPARSER_UNLIMITED_PACKETS);

        sent += count;
        timestamps[c] = timestamp;
        offsets[c++] = sent;

        bool timedout = (timestamps[c-1] - start_time) > mip_parser_timeout(&parser);

        bool error = false;

        if( timedout )
        {
            if( num_packets_parsed != last_parsed )
            {
                num_errors++;
                error = true;
                fprintf(stderr, "Parser produced %d packet(s) but should have timed out.\n", num_packets_parsed-last_parsed);
            }
        }
        else if( num_packets_parsed != (last_parsed + 1) )
        {
            num_errors++;
            error = true;
            fprintf(stderr, "Parser produced %d packet(s) but expected exactly 1.\n", num_packets_parsed-last_parsed);
        }
        else if( parsed_packet_length != packet_size )
        {
            num_errors++;
            error = true;
            fprintf(stderr, "Parsed packet size is wrong (%ld bytes)\n", parsed_packet_length);
        }
        else if( parsed_packet_timestamp != start_time )
        {
            num_errors++;
            error = true;
            fprintf(stderr, "Parsed packet has wrong timestamp %ld\n", parsed_packet_timestamp);
        }
        last_parsed = num_packets_parsed;

        if( error )
        {
            fprintf(stderr, "  packet_size=%ld, last_count=%ld, extra=%ld, start_time=%ld\n", packet_size, count, extra, start_time);

            fprintf(stderr, "  Sent chunks:");
            for(unsigned int d=0; d<c; d++)
                fprintf(stderr, " %ld", offsets[d]);
            fputc('\n', stderr);

            fprintf(stderr, "  Sent timestamps:");
            for(unsigned int d=0; d<c; d++)
                fprintf(stderr, " %ld", timestamps[d]);
            fputc('\n', stderr);

            fprintf(stderr, "  Expected packet:");
            print_packet(stderr, &packet);

            fprintf(stderr, "  (packet %d / %d)\n\n", i+1, NUM_ITERATIONS);
        }
    }

    return num_errors;
}

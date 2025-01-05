
#include <mip/mip_parser.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>


uint8_t parse_buffer[1024];
uint8_t input_buffer[1024];
uint8_t check_buffer[MIP_PACKET_LENGTH_MAX];

struct mip_parser parser;

unsigned int num_errors = 0;
size_t bytesRead = 0;
size_t bytes_parsed = 0;

bool handle_packet(void* p, const struct mip_packet* packet, mip_timestamp t)
{
    (void)t;

    FILE* infile2 = (FILE*)p;

    size_t length = mip_packet_total_length(packet);
    if( length > MIP_PACKET_LENGTH_MAX )
    {
        num_errors++;
        fprintf(stderr, "Packet with length too long (%ld)\n", length);
        return false;
    }
    // size_t written = fwrite(mip_packet_buffer(packet), 1, length, outfile);
    // return written == length;

    bytes_parsed += length;

    size_t read = fread(check_buffer, 1, length, infile2);

    if( read != length )
    {
        num_errors++;
        fprintf(stderr, "Failed to read from input file (2).\n");
        return false;
    }

    const uint8_t* packet_buffer = mip_packet_pointer(packet);

    // printf("Packet: ");
    // for(size_t i=0; i<length; i++)
    //     printf(" %02X", packet_buffer[i]);
    // fputc('\n', stdout);

    bool good = memcmp(check_buffer, packet_buffer, length) == 0;

    if( !good )
    {
        fprintf(stderr, "Packet does not match next sequence in input file:\n");

        fputs("Packet:    ", stderr);
        for(size_t i=0; i<length; i++)
            fprintf(stderr, " %02X", packet_buffer[i]);

        fputs("Reference: ", stderr);
        for(size_t i=0; i<length; i++)
            fprintf(stderr, " %02X", check_buffer[i]);

        fputc('\n', stderr);
    }

    return good;
}


int main(int argc, const char* argv[])
{
    if( argc < 2 )
    {
        fprintf(stderr, "Usage: %s <input-file>\n", argv[0]);
        return 1;
    }

    const char* input_filename = argv[1];

    FILE* infile = fopen(input_filename, "rb");
    if( !infile )
    {
        fprintf(stderr, "Error: could not open input file '%s'.", input_filename);
        return 1;
    }

    FILE* infile2 = fopen(input_filename, "rb");
    if( !infile2 )
    {
        fclose(infile);
        fprintf(stderr, "Error: could not open input file '%s' (2).", input_filename);
        return 1;
    }

    srand(0);

    mip_parser_init(&parser, parse_buffer, sizeof(parse_buffer), &handle_packet, infile2, MIPPARSER_DEFAULT_TIMEOUT_MS);

    do
    {
        const size_t numToRead = rand() % sizeof(input_buffer);

        const size_t numRead = fread(input_buffer, 1, numToRead, infile);
        bytesRead += numRead;

        mip_parser_parse(&parser, input_buffer, numRead, 0, MIPPARSER_UNLIMITED_PACKETS);

        // End of file (or error)
        if( numRead != numToRead )
            break;

    } while(num_errors == 0);

    fclose(infile);

    if( bytes_parsed != bytesRead )
    {
        num_errors++;
        fprintf(stderr, "Read %ld bytes but only parsed %ld bytes (delta %ld).\n", bytesRead, bytes_parsed, bytesRead-bytes_parsed);
    }

    fclose(infile2);

    return num_errors;
}

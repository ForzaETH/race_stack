
#include <mip/mip_packet.h>
#include <mip/mip_offsets.h>

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#define EXTRA 1
uint8_t buffer[MIP_PACKET_LENGTH_MAX+EXTRA];

int num_errors = 0;

void print_buffer(FILE* file)
{
    for(unsigned int i=0; i<sizeof(buffer); i++)
    {
        fprintf(file, " %02X", buffer[i]);
    }

    fputc('\n', stderr);
}

bool check(bool condition, const char* fmt, ...)
{
    if( condition )
        return true;

    va_list argptr;
    va_start(argptr, fmt);
    vfprintf(stderr, fmt, argptr);
    va_end(argptr);

    fputc('\n', stderr);

    print_buffer(stderr);

    num_errors++;
    return false;
}

bool check_equal(int a, int b, const char* fmt, ...)
{
    if( a == b )
        return true;

    va_list argptr;
    va_start(argptr, fmt);
    vfprintf(stderr, fmt, argptr);
    va_end(argptr);

    fprintf(stderr, " (%d != %d)", a, b);

    fputc('\n', stderr);

    print_buffer(stderr);

    num_errors++;
    return false;
}


void test_init()
{
    struct mip_packet packet;
    mip_packet_from_buffer(&packet, buffer, sizeof(buffer));

    check(packet._buffer == buffer && packet._buffer_length == sizeof(buffer)-EXTRA, "mip_packet_init is broken");
}

void test_create()
{
    struct mip_packet packet;

    uint8_t descriptors[] = {0x80, 0x01, 0x0C};

    for(unsigned int i=0; i<sizeof(descriptors); i++)
    {
        mip_packet_create(&packet, buffer, sizeof(buffer), descriptors[i]);

        check(packet._buffer == buffer && packet._buffer_length == sizeof(buffer)-EXTRA, "mip_packet_create sets wrong buffer info");
        check(packet._buffer[MIP_INDEX_DESCSET] == descriptors[i], "mip_packet_create sets wrong descriptor set (%02X != %02X)", packet._buffer[MIP_INDEX_DESCSET], descriptors[i]);
    }
}

void test_add_fields()
{
    struct mip_packet packet;

    mip_packet_create(&packet, buffer, sizeof(buffer), 0x80);

    check(packet._buffer[MIP_INDEX_DESCSET] == 0x80, "Packet has wrong descriptor set (%02X != %02X)", packet._buffer[MIP_INDEX_DESCSET], 0x80);
    check_equal( mip_packet_total_length(&packet), MIP_PACKET_LENGTH_MIN, "Empty packet has wrong size" );
    check_equal( mip_packet_payload_length(&packet), 0, "Packet has nonzero empty payload");
    check(mip_packet_is_sane(&packet), "Packet is not sane");

    size_t payload_size = 0;

    check( mip_packet_add_field(&packet, 0x04, NULL, 0), "Could not add an empty field" );
    check_equal( mip_packet_total_length(&packet), MIP_PACKET_LENGTH_MIN + MIP_FIELD_HEADER_LENGTH, "Empty field - Total length is wrong" );
    check_equal( mip_packet_payload_length(&packet), MIP_FIELD_HEADER_LENGTH, "Empty field - Packet payload length is wrong" );
    check_equal( mip_packet_payload(&packet)[MIP_INDEX_FIELD_DESC], 0x04, "Empty field - Field descriptor is wrong" );
    check_equal( mip_packet_payload(&packet)[MIP_INDEX_FIELD_LEN], 2, "Empty field - Field length is wrong" );

    payload_size += 2;

    uint8_t payload1[] = { 1, 2, 3, 4, 5, 6 };

    check( mip_packet_add_field(&packet, 0x05, payload1, sizeof(payload1)), "Could not add payload of size %d", sizeof(payload1));
    // 7565 800A 0204 0805 010203040506 00...
    check_equal( mip_packet_total_length(&packet), MIP_PACKET_LENGTH_MIN + payload_size + MIP_FIELD_HEADER_LENGTH + sizeof(payload1), "Field 1 - Total length is wrong" );
    check_equal( mip_packet_payload_length(&packet), payload_size + MIP_FIELD_HEADER_LENGTH + sizeof(payload1), "Field 1 - Packet payload length is wrong" );
    check_equal( mip_packet_payload(&packet)[payload_size + MIP_INDEX_FIELD_DESC], 0x05, "Field 1 - Field descriptor is wrong" );
    check_equal( mip_packet_payload(&packet)[payload_size + MIP_INDEX_FIELD_LEN], 2+sizeof(payload1), "Field 1 - Field length is wrong" );
    for(unsigned int j=0; j<sizeof(payload1); j++)
    {
        check_equal( mip_packet_payload(&packet)[payload_size + MIP_INDEX_FIELD_PAYLOAD + j], payload1[j], "Field 1 - Field payload is wrong at index %d", j);
    }
    payload_size += 2+sizeof(payload1);

    const uint8_t payload2[] = { 0xAA, 0xBA, 0xAC, 0xDE, 0xFF, 0xFF, 0x99, 0x55 };
    uint8_t* p2;

    check_equal( mip_packet_remaining_space(&packet), 245, "Field 2 - Remaining count is wrong beforehand");
    check_equal( mip_packet_alloc_field(&packet, 0x06, sizeof(payload2), &p2), 245-2-sizeof(payload2), "Field 2 - Remaining count is wrong after allocation");
    const uint8_t* expected_p2 = &mip_packet_payload(&packet)[payload_size + MIP_INDEX_FIELD_PAYLOAD];
    check( p2 == expected_p2, "Field 2 - payload ptr is wrong (%p != %p)", p2, expected_p2 );
    memcpy(p2, payload2, sizeof(payload2));
    check_equal( mip_packet_payload(&packet)[payload_size + MIP_INDEX_FIELD_DESC], 0x06, "Field 2 - Field descriptor is wrong" );
    check_equal( mip_packet_payload(&packet)[payload_size + MIP_INDEX_FIELD_LEN], 2+sizeof(payload2), "Field 2 - Field length is wrong" );
    check_equal( mip_packet_total_length(&packet), 26, "Field 2 - Total length is wrong" );
    check_equal( mip_packet_payload_length(&packet), 20, "Field 2 - Packet payload length is wrong" );

    mip_packet_finalize(&packet);

    check_equal( mip_packet_checksum_value(&packet), 0x80F0, "Checksum is wrong" );

    check( buffer[MIP_PACKET_LENGTH_MAX] == 0x00, "Extra byte at end of buffer got clobbered" );
}

void test_short_buffer()
{
    struct mip_packet packet;

    mip_packet_create(&packet, buffer, MIP_PACKET_LENGTH_MIN+4, 0x80);

    uint8_t* p;
    check_equal( mip_packet_alloc_field(&packet, 0x04, 3, &p), -1, "Wrong remaining count after allocating 1 too many bytes" );
    check_equal( mip_packet_alloc_field(&packet, 0x04, MIP_FIELD_PAYLOAD_LENGTH_MAX, &p), 2-MIP_FIELD_PAYLOAD_LENGTH_MAX, "Wrong remaining count after allocating max payload" );
    check_equal( mip_packet_alloc_field(&packet, 0x04, 255, &p), -253, "Wrong remaining count after allocating excessive payload" );
    check_equal( mip_packet_alloc_field(&packet, 0x05, 1, &p), 1, "Wrong remaining size after allocating 3 bytes" );
    check_equal( mip_packet_alloc_field(&packet, 0x06, 1, &p), -2, "Wrong remaining size after allocating 3 more bytes" );
}

int main(int argc, const char* argv[])
{
    test_init();
    test_create();
    test_add_fields();
    test_short_buffer();

    return num_errors;
}

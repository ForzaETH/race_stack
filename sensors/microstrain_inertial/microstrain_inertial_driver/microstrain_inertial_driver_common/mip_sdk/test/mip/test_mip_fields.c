
#include <mip/mip_field.h>
#include <mip/mip_packet.h>
#include <mip/mip_offsets.h>

#include <stdio.h>
#include <stdlib.h>


uint8_t packet_buffer[MIP_PACKET_LENGTH_MAX];
struct mip_packet packet;

struct mip_field fields[MIP_PACKET_PAYLOAD_LENGTH_MAX / MIP_FIELD_LENGTH_MIN];
unsigned int num_fields = 0;


bool add_random_field()
{
    const uint8_t length = rand() % 10;
    const uint8_t field_desc = (rand() % 255) + 1;

    uint8_t* payload;
    if( !mip_packet_alloc_field(&packet, field_desc, length, &payload) )
        return false;

    for(unsigned int i=0; i<length; i++)
        payload[i] = rand() & 0xFF;

    mip_field_init(&fields[num_fields++], mip_packet_descriptor_set(&packet), field_desc, payload, length);

    return true;
}


int main(int argc, const char* argv[])
{
    srand(0);

    const unsigned int NUM_ITERATIONS = 100;

    unsigned int num_errors = 0;

    for(unsigned int i=0; i<NUM_ITERATIONS; i++)
    {
        // Create a packet with a random number of fields.
        const uint8_t desc_set = (rand() % 255) + 1;

        mip_packet_create(&packet, packet_buffer, sizeof(packet_buffer), desc_set);

        num_fields = 0;
        while( add_random_field() )
        {
            // 20% chance of not adding any more fields.
            if( (rand() % 5) == 0 )
                break;
        }

        mip_packet_finalize(&packet);

        bool error = false;

        // Now iterate the fields and verify they match.
        unsigned int scanned_fields = 0;
        for(struct mip_field field = mip_field_first_from_packet(&packet); mip_field_is_valid(&field); mip_field_next(&field))
        {
            const uint8_t test_field_desc      = mip_field_field_descriptor(&field);
            const uint8_t test_desc_set        = mip_field_descriptor_set(&field);
            const uint8_t test_paylen         = mip_field_payload_length(&field);
            const uint8_t* const test_payload = mip_field_payload(&field);

            const uint8_t ref_field_desc      = mip_field_field_descriptor(&fields[scanned_fields]);
            const uint8_t ref_desc_set        = mip_field_descriptor_set(&fields[scanned_fields]);
            const uint8_t ref_paylen         = mip_field_payload_length(&fields[scanned_fields]);
            const uint8_t* const ref_payload = mip_field_payload(&fields[scanned_fields]);

            if( test_field_desc != ref_field_desc )
            {
                error = true;
                fprintf(stderr, "Field descriptor %02X does not match reference %02X.\n", test_field_desc, ref_field_desc);
            }
            if( test_desc_set != ref_desc_set )
            {
                error = true;
                fprintf(stderr, "Descriptor set %02X does not match reference %02X.\n", test_desc_set, ref_desc_set);
            }
            if( test_paylen != ref_paylen )
            {
                error = true;
                fprintf(stderr, "Payload length %d does not match reference %d.\n", test_paylen, ref_paylen);
            }
            if( test_payload != ref_payload )
            {
                error = true;
                fprintf(stderr, "Payload %p does not match reference %p.\n", test_payload, ref_payload);
            }

            scanned_fields++;
        }

        if( scanned_fields != num_fields )
        {
            error = true;
            fprintf(stderr, "Found %d fields but expected %d.\n", scanned_fields, num_fields);
        }

        if( error )
        {
            num_errors++;

            fprintf(stderr, "Error(s) detected for field list (descriptor/length):");
            for(unsigned int f=0; f<num_fields; f++)
                fprintf(stderr, " %02X/%d", mip_field_field_descriptor(&fields[f]), mip_field_payload_length(&fields[f]));
            fputc('\n', stderr);
        }

        // Bail if too many errors.
        if( num_errors > 10 )
        {
            fprintf(stderr, "***\n_too many errors, aborting.\n");
            break;
        }
    }

    return num_errors;
}

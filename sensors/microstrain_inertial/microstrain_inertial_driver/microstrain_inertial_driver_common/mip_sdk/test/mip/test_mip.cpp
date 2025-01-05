
#include <mip/mip.hpp>

#include <cstring>
#include <algorithm>

#include <stdio.h>
#include <stdlib.h>


using namespace mip;
using namespace mip::C;

uint8_t packetBuffer[PACKET_LENGTH_MAX];
uint8_t parseBuffer[1024];

Field fields[MIP_PACKET_PAYLOAD_LENGTH_MAX / MIP_FIELD_LENGTH_MIN];
unsigned int numFields = 0;

unsigned int numErrors = 0;

int main(int argc, const char* argv[])
{
    srand(0);

    auto callback = [](void*, const PacketRef* parsedPacket, Timestamp timestamp)->bool
    {
        unsigned int parsedFields = 0;
        bool error = false;
        for(Field field : *parsedPacket)
        {
            if( field.descriptorSet() != fields[parsedFields].descriptorSet() )
            {
                error = true;
                fprintf(stderr, "Descriptor set does not match.\n");
            }
            if( field.fieldDescriptor() != fields[parsedFields].fieldDescriptor() )
            {
                error = true;
                fprintf(stderr, "Field descriptor does not match.\n");
            }
            if( field.payloadLength() != fields[parsedFields].payloadLength() )
            {
                error = true;
                fprintf(stderr, "Payload length does not match.\n");
            }
            if( std::memcmp(field.payload(), fields[parsedFields].payload(), std::min(field.payloadLength(),fields[parsedFields].payloadLength())) != 0 )
            {
                error = true;
                fprintf(stderr, "Payloads do not match.\n");
            }

            if( error )
            {
                numErrors++;
                fprintf(stderr, "  From field %d/%d\n", parsedFields, numFields);
                fprintf(stderr, "  Descriptor set: %02X/%02X\n", field.descriptorSet(), fields[parsedFields].descriptorSet());
                fprintf(stderr, "  Field Descriptor: %02X/%02X\n", field.fieldDescriptor(), fields[parsedFields].fieldDescriptor());
                fprintf(stderr, "  Payload Length: %02X/%02X\n", field.payloadLength(), fields[parsedFields].payloadLength());
                fputc('\n', stderr);
            }

            parsedFields++;
        }
        if( parsedFields != numFields )
        {
            numErrors++;
            fprintf(stderr, "Field count mismatch: %d != %d\n", parsedFields, numFields);
        }

        return true;
    };

    Parser parser(parseBuffer, sizeof(parseBuffer), callback, nullptr, MIPPARSER_DEFAULT_TIMEOUT_MS);


    const unsigned int NUM_ITERATIONS = 100;

    for(unsigned int i=0; i<NUM_ITERATIONS; i++)
    {
        PacketRef packet(packetBuffer, sizeof(packetBuffer), 0x80);

        for(numFields = 0; ; numFields++)
        {
            const uint8_t fieldDescriptor = (rand() % 255) + 1;
            const uint8_t payloadLength = (rand() % MIP_FIELD_PAYLOAD_LENGTH_MAX) + 1;

            uint8_t* payload;
            int rem = packet.allocField(fieldDescriptor, payloadLength, &payload);

            if( rem < 0 )
                break;

            for(unsigned int p=0; p<payloadLength; p++)
                payload[p] = rand() & 0xFF;

            fields[numFields] = Field(packet.descriptorSet(), fieldDescriptor, payload, payloadLength);
        }

        packet.finalize();

        size_t rem = parser.parse(packet.pointer(), packet.totalLength(), 0, MIPPARSER_UNLIMITED_PACKETS);

        if( rem != 0 )
        {
            numErrors++;
            fprintf(stderr, "Parser reports %zu unparsed bytes.\n", rem);
        }

        if( numErrors > 10 )
            break;
    }

    return numErrors;
}

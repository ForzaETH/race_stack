
#include "example_utils.hpp"

#include <mip/mip_result.h>
#include <mip/mip_dispatch.h>
#include <mip/utils/serialization.h>

#include <mip/definitions/commands_base.hpp>
#include <mip/definitions/commands_3dm.hpp>
#include <mip/definitions/data_sensor.hpp>
#include <mip/mip.hpp>

#include <stdexcept>
#include <thread>
#include <array>

#define __STDC_FORMAT_MACROS 1
#include <cinttypes>

mip::data_sensor::ScaledAccel scaled_accel;

void handlePacket(void*, const mip::PacketRef& packet, mip::Timestamp timestamp)
{
    // if(packet.descriptorSet() != mip::MIP_SENSOR_DATA_DESC_SET)
    //     return;

    printf("\nGot packet with descriptor set 0x%02X:", packet.descriptorSet());

    for(mip::Field field : packet)
        printf(" %02X", field.fieldDescriptor());

    printf("\n");
}

void handleAccel(void*, const mip::Field& field, mip::Timestamp timestamp)
{
    mip::data_sensor::ScaledAccel data;

    if( field.extract(data) )
    {
        // Compute delta from last packet (the extractor runs after this, so scaled_accel is one packet behind).
        float delta[3] = {
            data.scaled_accel[0] - scaled_accel.scaled_accel[0],
            data.scaled_accel[1] - scaled_accel.scaled_accel[1],
            data.scaled_accel[2] - scaled_accel.scaled_accel[2],
        };
        printf("Accel Data: %f, %f, %f (delta %f, %f, %f)\n", data.scaled_accel[0], data.scaled_accel[1], data.scaled_accel[2], delta[0], delta[1], delta[2]);
    }
}

void handleGyro(void*, const mip::data_sensor::ScaledGyro& data, mip::Timestamp timestamp)
{
    printf("Gyro Data:  %f, %f, %f\n", data.scaled_gyro[0], data.scaled_gyro[1], data.scaled_gyro[2]);
}

void handleMag(void*, const mip::data_sensor::ScaledMag& data, mip::Timestamp timestamp)
{
    printf("Mag Data:   %f, %f, %f\n", data.scaled_mag[0], data.scaled_mag[1], data.scaled_mag[2]);
}


int run(mip::DeviceInterface& device)
{
        mip::CmdResult result;

        // Get the base rate.

        uint16_t base_rate;
        result = mip::commands_3dm::getBaseRate(device, mip::data_sensor::DESCRIPTOR_SET, &base_rate);

        if( result != mip::CmdResult::ACK_OK )
            return fprintf(stderr, "Failed to get base rate: %s (%d)\n", result.name(), result.value), 1;

        // Set the message format to stream at 100 Hz.

        const uint16_t sample_rate = 100; // Hz
        const uint16_t decimation = base_rate / sample_rate;

        std::array<mip::DescriptorRate, 3> descriptors = {{
            { mip::data_sensor::DATA_ACCEL_SCALED, decimation },
            { mip::data_sensor::DATA_GYRO_SCALED,  decimation },
            { mip::data_sensor::DATA_MAG_SCALED,   decimation },
        }};

        result = mip::commands_3dm::writeMessageFormat(device, mip::data_sensor::DESCRIPTOR_SET, descriptors.size(), descriptors.data());

        if( result == mip::CmdResult::NACK_COMMAND_FAILED )
        {
            // Failed to set message format - maybe this device doesn't have a magnetometer.
            // Try again without the last descriptor (scaled mag).
            result = mip::commands_3dm::writeMessageFormat(device, mip::data_sensor::DESCRIPTOR_SET, descriptors.size()-1, descriptors.data());
        }
        if( result != mip::CmdResult::ACK_OK )
            return fprintf(stderr, "Failed to set message format: %s (%d)\n", result.name(), result.value), 1;

        // Register some callbacks.

        mip::DispatchHandler packetHandler;
        device.registerPacketCallback<&handlePacket>(packetHandler, mip::C::MIP_DISPATCH_ANY_DATA_SET, false);

        mip::DispatchHandler dataHandlers[4];
        device.registerFieldCallback<&handleAccel>(dataHandlers[0], mip::data_sensor::DESCRIPTOR_SET, mip::data_sensor::DATA_ACCEL_SCALED);
        device.registerDataCallback<mip::data_sensor::ScaledGyro, &handleGyro>(dataHandlers[1]);
        device.registerDataCallback<mip::data_sensor::ScaledMag,  &handleMag >(dataHandlers[2]);
        device.registerExtractor(dataHandlers[3], &scaled_accel);

        // Enable the data stream and resume the device.

        result = mip::commands_3dm::writeDatastreamControl(device, mip::data_sensor::DESCRIPTOR_SET, true);
        if( result != mip::CmdResult::ACK_OK )
            return fprintf(stderr, "Failed to enable datastream: %s (%d)\n", result.name(), result.value), 1;

        // Resume the device to ensure it's streaming.

        result = mip::commands_base::resume(device);
        if( result != mip::CmdResult::ACK_OK )
            return fprintf(stderr, "Failed to resume device: %s (%d)\n", result.name(), result.value), 1;

        // Process data for 3 seconds.
        const mip::Timestamp start_time = getCurrentTimestamp();
        do
        {
            device.update();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

        } while( getCurrentTimestamp() - start_time < 3000 );

        result = mip::commands_base::setIdle(device);
        if( result != mip::CmdResult::ACK_OK )
            return fprintf(stderr, "Failed to idle device: %s (%d)\n", result.name(), result.value), 1;

    return 0;
}

int main(int argc, const char* argv[])
{
    std::unique_ptr<ExampleUtils> utils;
    try
    {
        utils = handleCommonArgs(argc, argv);
    }
    catch(const std::underflow_error& ex)
    {
        return printCommonUsage(argv);
    }
    catch(const std::exception& ex)
    {
        fprintf(stderr, "Error: %s\n", ex.what());
        return 1;
    }

    const int result = run(*utils->device);

#ifdef MIP_ENABLE_DIAGNOSTICS
    printf(
        "\nDiagnostics:\n"
        "\n"
        "Commands:\n"
        "  Sent:     %" PRIu16 "\n"
        "  Acks:     %" PRIu16 "\n"
        "  Nacks:    %" PRIu16 "\n"
        "  Timeouts: %" PRIu16 "\n"
        "  Errors:   %" PRIu16 "\n"
        "\n"
        "Parser:\n"
        "  Valid packets:    %" PRIu32 "\n"
        "  Invalid packets:  %" PRIu32 "\n"
        "  Timeouts:         %" PRIu32 "\n"
        "\n"
        "  Bytes read:       %" PRIu32 "\n"
        "  Valid bytes:      %" PRIu32 "\n"
        "  Unparsed bytes:   %" PRIu32 "\n",
        mip_cmd_queue_diagnostic_cmds_queued(&utils->device->cmdQueue()),
        mip_cmd_queue_diagnostic_cmd_acks(&utils->device->cmdQueue()),
        mip_cmd_queue_diagnostic_cmd_nacks(&utils->device->cmdQueue()),
        mip_cmd_queue_diagnostic_cmd_timeouts(&utils->device->cmdQueue()),
        mip_cmd_queue_diagnostic_cmd_errors(&utils->device->cmdQueue()),

        mip_parser_diagnostic_valid_packets(&utils->device->parser()),
        mip_parser_diagnostic_invalid_packets(&utils->device->parser()),
        mip_parser_diagnostic_timeouts(&utils->device->parser()),
        mip_parser_diagnostic_bytes_read(&utils->device->parser()),
        mip_parser_diagnostic_packet_bytes(&utils->device->parser()),
        mip_parser_diagnostic_bytes_skipped(&utils->device->parser())
    );
#endif // MIP_ENABLE_DIAGNOSTICS

    return result;
}

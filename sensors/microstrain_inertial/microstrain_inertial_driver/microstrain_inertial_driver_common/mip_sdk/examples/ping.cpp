
#include "serial_mip_device.hpp"

#include <mip/definitions/commands_base.h>
#include <mip/mip.hpp>
#include <mip/mip_device.hpp>
#include <serial/serial.h>

#include <cstring>
#include <stdio.h>

namespace mip
{
    namespace C {
        CmdResult mipcmd_base_getDeviceInfo(struct C::MipInterfaceState* device, const struct MipCmd_Base_GetDeviceInfo* cmd, struct MipCmd_Base_GetDeviceInfo_Response* response);
        CmdResult mipcmd_base_getDeviceInfo(struct C::MipInterfaceState* device, struct MipBaseDeviceInfo* info);
    }
}

int usage(const char* argv[])
{
    fprintf(stderr, "Usage: %s <port> <baudrate>\n", argv[0]);
    return 1;
}


int main(int argc, const char* argv[])
{
    uint32_t baud = 0;

    if( argc == 1 )
    {
        printf("Available serial ports:\n");
        std::vector<serial::PortInfo> ports = serial::list_ports();

        for(const serial::PortInfo& port : ports)
        {
            printf("  %s %s %s\n", port.port.c_str(), port.description.c_str(), port.hardware_id.c_str());
        }
        return 0;
    }
    else if( argc == 3 )
    {
        baud = std::strtoul(argv[2], nullptr, 10);
        if( baud == 0 )
        {
            fprintf(stderr, "Error: invalid baud rate '%s'\n", argv[2]);
            return 1;
        }
    }
    else
    {
        return usage(argv);
    }

    try
    {
#define METHOD 8

#if METHOD == 1 || METHOD == 2 || METHOD == 3
        serial::Serial port(argv[1], baud, serial::Timeout::simpleTimeout(10));

        uint8_t buffer[PACKET_LENGTH_MAX];

        mip::Packet packet(buffer, sizeof(buffer), mip::MIP_BASE_COMMAND_DESC_SET);

        uint8_t* payload;
        mip::RemainingCount available = packet.allocField(mip::MIP_CMD_DESC_BASE_PING, 0, &payload);

    #if METHOD == 1
        mip::MipCmd_Base_GetDeviceInfo info;
        size_t used = mip::insert_MipCmd_Base_GetDeviceInfo(payload, available, 0, &info);
    #elif METHOD == 2
        mip::MipCmd_Base_GetDeviceInfo info;
        size_t used = mip::insert(payload, available, 0, info);
    #elif METHOD == 3
        size_t used = mip::insert_MipCmd_Base_GetDeviceInfo_args(payload, available, 0);
    #endif

        // Skip error checking as this field will always fit in this buffer.
        packet.reallocLastField(payload, used);

        packet.finalize();

        printf("Send bytes [");
        for(size_t i=0; i<packet.totalLength(); i++)
            printf("%02X", packet.pointer()[i]);
        printf("]\n");

        port.write(packet.pointer(), packet.totalLength());

        size_t read = port.read(buffer, sizeof(buffer));

        printf("Received bytes [");
        for(size_t i=0; i<read; i++)
            printf("%02X", buffer[i]);
        printf("]\n");

#elif METHOD == 4

        MipDevice device(argv[1], baud);

        uint8_t responseBuffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
        mip::C::MipPendingCmd cmd;
        mip::C::MipPendingCmd_initWithResponse(&cmd, mip::MIP_BASE_COMMAND_DESC_SET, mip::MIP_CMD_DESC_BASE_GET_DEVICE_INFO, mip::MIP_REPLY_DESC_BASE_DEVICE_INFO, responseBuffer, sizeof(responseBuffer));
        mip::C::MipCmdQueue_enqueue(device.cmdQueue(), &cmd);

        uint8_t packetBuffer[PACKET_LENGTH_MAX];

        mip::Packet packet(packetBuffer, sizeof(packetBuffer), mip::MIP_BASE_COMMAND_DESC_SET);

        packet.addField(mip::MIP_CMD_DESC_BASE_GET_DEVICE_INFO, NULL, 0);
        packet.finalize();

        device.sendToDevice(packet);

        mip::CmdResult result = device.waitForReply(cmd);

//         MipBase_DeviceInfo info;
//         CmdResult result = MipInterface_getDeviceInfo(device, &info);

        if( result == mip::MIP_ACK_OK )
        {
            const size_t responseSize = mip::C::MipPendingCmd_responseLength(&cmd);
            printf("Success: command completed with ACK: responseLength=%ld\n", responseSize);

            mip::MipCmd_Base_GetDeviceInfo_Response response;
            size_t used = mip::extract_MipCmd_Base_GetDeviceInfo_Response(mip::C::MipPendingCmd_response(&cmd), responseSize, 0, &response);
            if( used == responseSize )
            {
                auto print_info = [](const char* name, const char info[16])
                {
                    char msg[17] = {0};
                    std::strncpy(msg, info, 16);
                    printf("  %s%s\n", name, msg);
                };

                print_info("Model name:       ", response.device_info.model_name);
                print_info("Model number:     ", response.device_info.model_number);
                print_info("Serial Number:    ", response.device_info.serial_number);
                print_info("Device Options:   ", response.device_info.device_options);
                print_info("Lot Number:       ", response.device_info.lot_number);

                printf(  "  Firmware version:           %d.%d.%d\n\n",
                    (response.device_info.firmware_version / 1000),
                    (response.device_info.firmware_version / 100) % 10,
                    (response.device_info.firmware_version / 1)   % 100
                );
            }
            else
            {
                printf("Error: response size does not match expected (at least %ld bytes)\n", used);
            }
        }
        else
        {
            printf("Error: command completed with NACK: %s (%d)\n", mip::MipCmdResult_toString(result), result);
        }
#elif METHOD >= 5
    MipDevice device(argv[1], baud);

    mip::MipCmd_Base_GetDeviceInfo cmd;
    mip::MipCmd_Base_GetDeviceInfo_Response response;

    #if METHOD == 5
        mip::CmdResult result = exec_MipCmd_Base_GetDeviceInfo(&device, &cmd, &response);
    #elif METHOD == 6
        mip::CmdResult result = mipcmd_base_getDeviceInfo(&device, &response.device_info);
    #elif METHOD == 7
        mip::CmdResult result = mip::runCommand(&device, cmd, response);
    #elif METHOD == 8
        mip::DeviceInterface* device2 = &device;
        mip::C::MipInterfaceState* device3 = device2;
        mip::MipInterfaceState* device4 = device3;
        static_assert(std::is_same<mip::MipInterfaceState, mip::C::MipInterfaceState>::value, "Not the same");
        mip::CmdResult result = mip::get_device_information(device4, &response.device_info);
    #endif

        if( result == mip::MIP_ACK_OK )
        {
            printf("Success:\n");

            auto print_info = [](const char* name, const char info[16])
            {
                char msg[17] = {0};
                std::strncpy(msg, info, 16);
                printf("  %s%s\n", name, msg);
            };

            print_info("Model name:       ", response.device_info.model_name);
            print_info("Model number:     ", response.device_info.model_number);
            print_info("Serial Number:    ", response.device_info.serial_number);
            print_info("Device Options:   ", response.device_info.device_options);
            print_info("Lot Number:       ", response.device_info.lot_number);

            printf(  "  Firmware version:           %d.%d.%d\n\n",
                (response.device_info.firmware_version / 1000),
                (response.device_info.firmware_version / 100) % 10,
                (response.device_info.firmware_version / 1)   % 100
            );
        }
        else
        {
            printf("Error: command completed with NACK: %s (%d)\n", mip::MipCmdResult_toString(result), result);
        }

#endif
    }
    catch(const std::exception& ex)
    {
        fprintf(stderr, "Could not open serial port: %s", ex.what());
        return 1;
    }

    return 0;
}
//
// namespace mip
// {
// namespace C
// {
//
// CmdResult mipcmd_base_getDeviceInfo(struct C::MipInterfaceState* device, struct MipBaseDeviceInfo* info)
// {
//     uint8_t payload[MIP_FIELD_LENGTH_MAX];
//
//     size_t payloadLength = insert_MipCmd_Base_GetDeviceInfo(payload, sizeof(payload), 0, NULL);
//     assert(payloadLength <= sizeof(payload));
//
//     uint8_t responseLength;
//     CmdResult result = C::MipInterface_runCommandWithResponse(device, MIP_BASE_COMMAND_DESC_SET, MIP_CMD_DESC_BASE_GET_DEVICE_INFO, payload, payloadLength, MIP_REPLY_DESC_BASE_DEVICE_INFO, payload, &responseLength);
//     if( result == MIP_ACK_OK )
//     {
//         size_t used = extract_MipBaseDeviceInfo(payload, responseLength, 0, info);
//         if( used != responseLength )
//             result = MIP_STATUS_ERROR;
//     }
//
//     return result;
// }
//
// CmdResult mipcmd_base_getDeviceInfo(struct C::MipInterfaceState* device, const struct MipCmd_Base_GetDeviceInfo* cmd, struct MipCmd_Base_GetDeviceInfo_Response* response)
// {
//     uint8_t buffer[PACKET_LENGTH_MAX];
//
//     struct C::Packet packet;
//     MipPacket_create(&packet, buffer, sizeof(buffer), MIP_BASE_COMMAND_DESC_SET);
//
//     uint8_t* payload;
//     RemainingCount available = MipPacket_allocField(&packet, MIP_CMD_DESC_BASE_GET_DEVICE_INFO, 0, &payload);
//
//     size_t used = insert_MipCmd_Base_GetDeviceInfo(payload, available, 0, cmd);
//     assert( used <= available );
//     MipPacket_reallocLastField(&packet, payload, used);
//
//     MipPacket_finalize(&packet);
//
//
//     struct C::MipPendingCmd pending;
//     C::MipPendingCmd_initWithResponse(&pending, MIP_BASE_COMMAND_DESC_SET, MIP_CMD_DESC_BASE_GET_DEVICE_INFO, MIP_REPLY_DESC_BASE_DEVICE_INFO, buffer, MIP_FIELD_PAYLOAD_LENGTH_MAX);
//
//     C::MipCmdQueue_enqueue( C::MipInterface_cmdQueue(device), &pending );
//
//     if( !C::MipInterface_sendToDevice(device, MipPacket_pointer(&packet), MipPacket_totalLength(&packet)) )
//     {
//         C::MipCmdQueue_dequeue(C::MipInterface_cmdQueue(device), &pending);
//         return MIP_STATUS_ERROR;
//     }
//
//     CmdResult result = C::MipInterface_waitForReply(device, &pending);
//
//     if( result == MIP_ACK_OK )
//     {
//         size_t responseLength = C::MipPendingCmd_responseLength(&pending);
//
//         used = extract_MipCmd_Base_GetDeviceInfo_Response( C::MipPendingCmd_response(&pending), responseLength, 0, response);
//
//         if( used!= responseLength )
//             return MIP_STATUS_ERROR;
//     }
//
//     return result;
// }
//
// }
// }


#include "example_utils.hpp"

#include <mip/definitions/commands_base.hpp>
#include <mip/definitions/commands_filter.hpp>

#include <stdexcept>
#include <vector>
#include <cstring>
#include <stdio.h>


int main(int argc, const char* argv[])
{
    try
    {
        std::unique_ptr<ExampleUtils> utils = handleCommonArgs(argc, argv);
        std::unique_ptr<mip::DeviceInterface>& device = utils->device;

        mip::commands_base::BaseDeviceInfo device_info;

        mip::CmdResult result = mip::commands_base::getDeviceInfo(*device, &device_info);

        if( result == mip::CmdResult::ACK_OK)
        {
            printf("Success:\n");

            auto print_info = [](const char* name, const char info[16])
            {
                char msg[17] = {0};
                std::strncpy(msg, info, 16);
                printf("  %s%s\n", name, msg);
            };

            print_info("Model name:       ", device_info.model_name);
            print_info("Model number:     ", device_info.model_number);
            print_info("Serial Number:    ", device_info.serial_number);
            print_info("Device Options:   ", device_info.device_options);
            print_info("Lot Number:       ", device_info.lot_number);

            printf(  "  Firmware version:           %d.%d.%d\n\n",
                (device_info.firmware_version / 1000),
                (device_info.firmware_version / 100) % 10,
                (device_info.firmware_version / 1)   % 100
            );
        }
        else
        {
            printf("Error: command completed with NACK: %s (%d)\n", result.name(), result.value);
        }
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

    return 0;
}

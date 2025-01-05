
#include <serial/serial.h>

#include <stdio.h>


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
    else if( argc >= 2 )
    {
        if( argc == 3 )
        {
            baud = std::strtoul(argv[2], nullptr, 10);
            if( baud == 0 )
            {
                fprintf(stderr, "Error: invalid baud rate '%s'\n", argv[2]);
                return 1;
            }
        }
    }
    else
    {
        fprintf(stderr, "Usage: %s <port> <baud>\n", argv[0]);
        return 1;
    }

    serial::Serial port(argv[1], baud, serial::Timeout::simpleTimeout(10));

    return 0;
}

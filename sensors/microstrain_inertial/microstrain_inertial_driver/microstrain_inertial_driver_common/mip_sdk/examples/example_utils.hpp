
#pragma once

#ifdef MIP_USE_SERIAL
    #include "mip/platform/serial_connection.hpp"
#endif
#ifdef MIP_USE_TCP
    #include "mip/platform/tcp_connection.hpp"
#endif
#ifdef MIP_USE_EXTRAS
    #include "mip/extras/recording_connection.hpp"
#endif


#include <mip/mip_device.hpp>

#include <chrono>
#include <memory>
#include <fstream>
#include <iostream>
#include <exception>

struct ExampleUtils
{
    std::unique_ptr<mip::Connection> connection;
    std::unique_ptr<mip::DeviceInterface> device;
    std::unique_ptr<std::ofstream> recordedFile;
    uint8_t buffer[1024];
};

mip::Timestamp getCurrentTimestamp();

std::unique_ptr<ExampleUtils> openFromArgs(const std::string& port_or_hostname, const std::string& baud_or_port, const std::string& binary_file_path);

std::unique_ptr<ExampleUtils> handleCommonArgs(int argc, const char* argv[], int maxArgs=4);
int printCommonUsage(const char* argv[]);

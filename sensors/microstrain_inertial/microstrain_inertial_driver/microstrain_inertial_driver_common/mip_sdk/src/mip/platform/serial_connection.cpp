#include "serial_connection.hpp"

#include <stdexcept>
#include <chrono>
#include <cstdio>

namespace mip
{
namespace platform
{

///@brief Creates a SerialConnection that will communicate with a device over serial
///
///@param portName Path to the port to connect to. On Windows, this usually looks like "COM<N>", on linux, "/dev/tty<N>"
///@param baudrate Baud rate to open the device at. Note that the device needs to be configured to
SerialConnection::SerialConnection(const std::string& portName, uint32_t baudrate)
{
    mPortName = portName;
    mBaudrate = baudrate;
    mType     = TYPE;

    serial_port_init(&mPort);
}

///@brief Closes the underlying serial port
SerialConnection::~SerialConnection()
{
    SerialConnection::disconnect();
}

///@brief Check if the port is connected
bool SerialConnection::isConnected() const
{
    return serial_port_is_open(&mPort);
}

///@brief Connect to the port
bool SerialConnection::connect()
{
    if (serial_port_is_open(&mPort))
        return true;

   return serial_port_open(&mPort, mPortName.c_str(), mBaudrate);
}

///@brief Disconnect from the port
bool SerialConnection::disconnect()
{
   if (!serial_port_is_open(&mPort))
        return true;

   return serial_port_close(&mPort);
}



///@copydoc mip::Connection::recvFromDevice
bool SerialConnection::recvFromDevice(uint8_t* buffer, size_t max_length, Timeout wait_time, size_t* length_out, mip::Timestamp* timestamp)
{
    *timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();

    return serial_port_read(&mPort, buffer, max_length, wait_time, length_out);
}

///@copydoc mip::Connection::sendToDevice
bool SerialConnection::sendToDevice(const uint8_t* data, size_t length)
{
    size_t length_out;
    return serial_port_write(&mPort, data, length, &length_out);
}

}  // namespace platform
}  // namespace mip

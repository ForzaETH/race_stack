#pragma once

#include <mip/mip_device.hpp>
#include <mip/utils/serial_port.h>

#include <string>

namespace mip
{
namespace platform
{

////////////////////////////////////////////////////////////////////////////////
///@addtogroup mip_platform
///@{

////////////////////////////////////////////////////////////////////////////////
///@brief Can be used on Windows, OSX, or linux to communicate with a MIP device over serial
///
class SerialConnection : public mip::Connection
{
public:

    static constexpr auto TYPE = "Serial";

    SerialConnection(const std::string& portName, uint32_t baudrate);
    ~SerialConnection();

    bool recvFromDevice(uint8_t* buffer, size_t max_length, Timeout wait_time, size_t* length_out, mip::Timestamp* timestamp) final;
    bool sendToDevice(const uint8_t* data, size_t length) final;

    bool isConnected() const override;
    bool connect() override;
    bool disconnect() override;

    void connectionInfo(std::string &name, uint32_t &baudrate) const
    {
        name     = mPortName;
        baudrate = mBaudrate;
    };

private:
    serial_port mPort;
    std::string mPortName;
    uint32_t    mBaudrate;

public:
    const char* interfaceName() const override { return mPortName.c_str(); }
    uint32_t parameter() const override { return mBaudrate; }
};


////////////////////////////////////////////////////////////////////////////////
///@brief A serial connection but indicates that it's actually a USB connection.
class UsbSerialConnection : public SerialConnection
{
public:
    static constexpr auto TYPE = "USB";

    UsbSerialConnection(const std::string& portName, uint32_t baudrate) : SerialConnection(portName, baudrate) { mType = TYPE; }
};

///@}
////////////////////////////////////////////////////////////////////////////////

}  // namespace platform
}  // namespace mip
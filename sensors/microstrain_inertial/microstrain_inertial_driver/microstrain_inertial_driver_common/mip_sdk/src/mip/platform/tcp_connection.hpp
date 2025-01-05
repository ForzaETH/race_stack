#pragma once

#include <mip/mip_device.hpp>

#include <mip/utils/tcp_socket.h>

#include <string>

namespace mip
{
namespace platform
{

////////////////////////////////////////////////////////////////////////////////
///@addtogroup mip_platform
///@{

////////////////////////////////////////////////////////////////////////////////
///@brief Can be used on Windows, OSX, or linux to communicate with a MIP device over TCP
///
class TcpConnection : public mip::Connection
{
public:
    static constexpr auto TYPE = "TCP";

    TcpConnection() = default;
    TcpConnection(const std::string& hostname, uint16_t port);
    ~TcpConnection();

    bool recvFromDevice(uint8_t* buffer, size_t max_length, Timeout wait_time, size_t* length_out, mip::Timestamp* timestamp) final;
    bool sendToDevice(const uint8_t* data, size_t length) final;

    bool isConnected() const;
    bool connect();
    bool disconnect();

    void connectionInfo(std::string &host_name, uint32_t &port) const
    {
        host_name = mHostname;
        port      = mPort;
    };

private:
    tcp_socket mSocket;
    std::string mHostname;
    uint16_t mPort = 0;

public:
    const char* interfaceName() const override { return mHostname.c_str(); }
    uint32_t parameter() const override { return mPort; }
};

///@}
////////////////////////////////////////////////////////////////////////////////

};  // namespace platform
};  // namespace mip

#include "tcp_connection.hpp"

#include <stdexcept>
#include <chrono>
#include <cstdio>

namespace mip
{
namespace platform
{

///@brief Creates a TcpConnection that will communicate with a device over TCP
///
///@param hostName Host name or IP address to connect to
///@param port     Port on hostName to connect to
TcpConnection::TcpConnection(const std::string& hostname, uint16_t port)
{
    mHostname = hostname;
    mPort     = port;
    mType     = TYPE;

    tcp_socket_init(&mSocket);
}

///@brief Closes the underlying TCP socket
TcpConnection::~TcpConnection()
{
    if(isConnected())
      TcpConnection::disconnect();
}

///@brief Check if the socket is connected
bool TcpConnection::isConnected() const
{
  return tcp_socket_is_open(&mSocket);
}

///@brief Connect to the socket
bool TcpConnection::connect()
{
  if(isConnected())
    return true;

  return tcp_socket_open(&mSocket, mHostname.c_str(), mPort, 3000);
}


///@brief Disconnect from the socket
bool TcpConnection::disconnect()
{
   if(!isConnected())
     return true;

   return tcp_socket_close(&mSocket);
}

///@copydoc mip::Connection::sendToDevice
bool TcpConnection::recvFromDevice(uint8_t* buffer, size_t max_length, Timeout wait_time, size_t* length_out, mip::Timestamp* timestamp)
{
    (void)wait_time;  // Not used, timeout is always fixed

    *timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();

    return tcp_socket_recv(&mSocket, buffer, max_length, length_out);
}

///@copydoc mip::Connection::recvFromDevice
bool TcpConnection::sendToDevice(const uint8_t* data, size_t length)
{
    size_t length_out;
    return tcp_socket_send(&mSocket, data, length, &length_out);
}

}  // namespace platform
}  // namespace mip
#pragma once

#ifdef WIN32

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#include <winsock2.h>

#endif

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

////////////////////////////////////////////////////////////////////////////////
///@addtogroup mip_platform
///
///@{

////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_tcp  TCP Client
///
///@brief Simple implementation for reading and writing to a tcp client socket
///
///@{

typedef struct tcp_socket
{
#ifdef WIN32
    SOCKET handle;
#else
    int handle;
#endif
} tcp_socket;



void tcp_socket_init(tcp_socket* socket_ptr);
bool tcp_socket_is_open(const tcp_socket* socket_ptr);
bool tcp_socket_open(tcp_socket* socket_ptr, const char* hostname, uint16_t port, unsigned int timeout_ms);
bool tcp_socket_close(tcp_socket* socket_ptr);
bool tcp_socket_send(tcp_socket* socket_ptr, const void* buffer, size_t num_bytes, size_t* bytes_written);
bool tcp_socket_recv(tcp_socket* socket_ptr, void* buffer, size_t num_bytes, size_t* bytes_read);

///@}
///@}
////////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
}
#endif

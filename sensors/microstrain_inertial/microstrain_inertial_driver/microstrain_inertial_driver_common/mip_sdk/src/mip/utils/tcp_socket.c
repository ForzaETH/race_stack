
#include "tcp_socket.h"

#include "mip/mip_logging.h"

#ifdef _WIN32

#include <winsock2.h>
#include <ws2tcpip.h>

static const int SEND_FLAGS = 0;

#ifdef _MSC_VER
typedef int ssize_t;
#endif

#else

#include <errno.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/ip.h>
#include <netdb.h>
#include <string.h>

static const int INVALID_SOCKET = -1;
static const int SEND_FLAGS = MSG_NOSIGNAL;

#endif

#include <stdio.h>

void tcp_socket_init(tcp_socket* socket_ptr)
{
    socket_ptr->handle = INVALID_SOCKET;
}

bool tcp_socket_is_open(const tcp_socket* socket_ptr)
{
    return socket_ptr->handle != INVALID_SOCKET;
}

static bool tcp_socket_open_common(tcp_socket* socket_ptr, const char* hostname, uint16_t port, unsigned int timeout_ms)
{
    //assert(socket_ptr->handle == INVALID_SOCKET);

    // https://man7.org/linux/man-pages/man3/getaddrinfo.3.html
    struct addrinfo hints, *info;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family   = AF_INET; // AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;
    hints.ai_flags    = 0;

    char port_str[6];  // Maximum 5 digits
    sprintf(port_str, "%d", port);

    int result = getaddrinfo(hostname, port_str, &hints, &info);
    if( result != 0 )
        return false;

    for(struct addrinfo* addr=info; addr!=NULL; addr=addr->ai_next)
    {
        socket_ptr->handle = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP); //socket(addr->ai_family, addr->ai_socktype, addr->ai_protocol);
        if( socket_ptr->handle == INVALID_SOCKET )
            continue;

        if( connect(socket_ptr->handle, addr->ai_addr, addr->ai_addrlen) == 0 )
            break;

#ifdef WIN32
        closesocket(socket_ptr->handle);
#else
        close(socket_ptr->handle);
#endif
        socket_ptr->handle = INVALID_SOCKET;
    }

    freeaddrinfo(info);

    if( socket_ptr->handle == INVALID_SOCKET )
        return false;

#ifdef WIN32
    if( setsockopt(socket_ptr->handle, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout_ms, sizeof(timeout_ms)) != 0 )
        return false;

    if( setsockopt(socket_ptr->handle, SOL_SOCKET, SO_SNDTIMEO, (char*)&timeout_ms, sizeof(timeout_ms)) != 0 )
        return false;
#else
    struct timeval timeout_option;
    timeout_option.tv_sec  = timeout_ms / 1000;
    timeout_option.tv_usec = (timeout_ms % 1000) * 1000;

    if( setsockopt(socket_ptr->handle, SOL_SOCKET, SO_RCVTIMEO, &timeout_option, sizeof(timeout_option)) != 0 )
        return false;

    if( setsockopt(socket_ptr->handle, SOL_SOCKET, SO_SNDTIMEO, &timeout_option, sizeof(timeout_option)) != 0 )
        return false;
#endif

    return true;
}

bool tcp_socket_open(tcp_socket* socket_ptr, const char* hostname, uint16_t port, unsigned int timeout_ms)
{
#ifdef WIN32

    // Initialize winsock for each connection since there's no global init function.
    // This is safe to do multiple times, as long as it's shutdown the same number of times.
    struct WSAData wsaData;
    int result = WSAStartup(MAKEWORD(2,2), &wsaData);
    if(result != 0)
    {
        MIP_LOG_ERROR("WSAStartup() failed: %d\n", result);
        return false;
    }

#endif

    return tcp_socket_open_common(socket_ptr, hostname, port ,timeout_ms);
}

bool tcp_socket_close(tcp_socket* socket_ptr)
{
    if( socket_ptr->handle == INVALID_SOCKET )
        return false;

#ifdef WIN32
    closesocket(socket_ptr->handle);
    WSACleanup(); // See tcp_socket_open
#else
    close(socket_ptr->handle);
#endif

    socket_ptr->handle = INVALID_SOCKET;
    return true;
}

bool tcp_socket_send(tcp_socket* socket_ptr, const void* buffer, size_t num_bytes, size_t* bytes_written)
{
    for(*bytes_written = 0; *bytes_written < num_bytes; )
    {
        ssize_t sent = send(socket_ptr->handle, buffer, num_bytes, SEND_FLAGS);
        if(sent < 0)
            return false;

        *bytes_written += sent;
    }
    return true;
}

bool tcp_socket_recv(tcp_socket* socket_ptr, void* buffer, size_t num_bytes, size_t* bytes_read)
{
    ssize_t local_bytes_read = recv(socket_ptr->handle, buffer, num_bytes, SEND_FLAGS);

    if( local_bytes_read < 0 )
    {
#ifdef WIN32
        return false;
#else
        if(errno != EAGAIN && errno != EWOULDBLOCK)
            return false;
        else
            return true;
#endif
    }
    // Throw an error if the connection has been closed by the other side.
    else if( local_bytes_read == 0 )
        return false;

    *bytes_read = local_bytes_read;
    return true;
}

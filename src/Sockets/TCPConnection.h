#ifndef __TCPCONNECTION_H__
#define __TCPCONNECTION_H__

#include <sock.h>
#include <string>

class TCPConnection {
private:
    uint32_t port;
    std::string address;
    int sock_local_fd = -1;
    int sock_remote_fd = -1;
    sockaddr_in sock_local = NULL;
    sockaddr_in sock_remote = NULL;
public:
    TCPConnection(uint32_t port, std::string address);
    void accept_inbound_connection();    
    void is_connected();
}

#endif // __TCPCONNECTION_H__
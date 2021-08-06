#include "TCPConnection.h"

TCPConnection::TCPConnection(uint32_t port, std::string address) :
    port(port), address(address)
{}

void TCPConnection::accept_inbound_connection() {
    
}
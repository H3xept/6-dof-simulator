#include "TCPConnection.h"
#include "TCPAcceptor.h"

TCPAcceptor::TCPAcceptor(boost::asio::io_service& service, int port) :
    _acceptor(service, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), port)),
    service(service) {
        this->start();
    }

TCPAcceptor::~TCPAcceptor() {}

void TCPAcceptor::start() 
{
    TCPConnection::tcp_connection_ptr conn = TCPConnection::create(this->service, *this);
    conn->start_reading();
    this->_acceptor.async_accept(
        conn->get_socket(),
        boost::bind(
            &TCPAcceptor::handle_accept,
            this,
            conn,
            boost::asio::placeholders::error
        )
    ); 
}

void TCPAcceptor::handle_accept(TCPConnection::tcp_connection_ptr conn, const boost::system::error_code& err) {
    printf("Accepted \n");
}

void TCPAcceptor::receive_data(const char* buff, size_t len) {
    for (auto* r : this->broadcast_recv) {
        r->receive_data(buff, len);
    }
}

void TCPAcceptor::add_data_receiver(DataReceiver* r) {
    this->broadcast_recv.push_back(r);
}

void TCPAcceptor::send_data(const char* buff, size_t len) {}
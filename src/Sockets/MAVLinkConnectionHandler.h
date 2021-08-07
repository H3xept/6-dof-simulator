#ifndef __MAVLINKCONNECTIONHANDLER_H__
#define __MAVLINKCONNECTIONHANDLER_H__

#include "TCPAcceptor.h"
#include "../Interfaces/MAVLinkMessageRelay.h"
#include <boost/asio.hpp>

#define MAX_MAVLINK_PACKET_LEN 512

using namespace boost::asio;

enum ConnectionTarget {
    PX4 = 4560
};

class MAVLinkConnectionHandler : public MAVLinkMessageRelay, public DataReceiver {
private:
    char mavlink_msg_buffer[MAX_MAVLINK_PACKET_LEN];
    uint buffer_read_head = 0;
    uint buffer_parse_head = 0;
    TCPAcceptor tcp_acceptor;
    bool parse_mavlink_message(const char* buff, size_t len, mavlink_message_t& msg, mavlink_status_t& status);
public:
    MAVLinkConnectionHandler(io_service& service, ConnectionTarget target);    
    ~MAVLinkConnectionHandler();
    void receive_data(const char* buff, size_t len) override;
    bool received_message(mavlink_message_t m) override;
    bool send_message(mavlink_message_t& m) override;
};

#endif // __MAVLINKCONNECTIONHANDLER_H__
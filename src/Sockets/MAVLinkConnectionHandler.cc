#include "MAVLinkConnectionHandler.h"

MAVLinkConnectionHandler::MAVLinkConnectionHandler(io_service& service, ConnectionTarget target) : 
    tcp_acceptor(service, (int)target) {
        this->tcp_acceptor.add_data_receiver(this);
}

MAVLinkConnectionHandler::~MAVLinkConnectionHandler() {}

bool MAVLinkConnectionHandler::parse_mavlink_message(const char* buff, size_t len, mavlink_message_t& msg, mavlink_status_t& status) {
    for (int i = 0; i < len; i++) {
        if (mavlink_parse_char(MAVLINK_COMM_0,buff[i], &msg, &status)) return true;
    }
    return false;
}

void MAVLinkConnectionHandler::receive_data(const char* buff, size_t len) {
    mavlink_message_t msg;
    mavlink_status_t status;
    if (this->parse_mavlink_message(buff, len, msg, status)) {
        this->received_message(msg);
    }
}

bool MAVLinkConnectionHandler::received_message(mavlink_message_t m) {
    printf("Received mavlink message! (%d)\n", m.msgid);
}

bool MAVLinkConnectionHandler::send_message(mavlink_message_t& m) {}

#include "MAVLinkConnectionHandler.h"
#include "../Logging/ConsoleLogger.h"

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
    if (this->message_handler != NULL) 
        this->message_handler->handle_mavlink_message(m);
}

bool MAVLinkConnectionHandler::send_message(const mavlink_message_t& m) {
#define MAX_MAVLINK_MESSAGE_SIZE 300

    uint8_t buf[MAX_MAVLINK_MESSAGE_SIZE];
    u_int16_t len = mavlink_msg_to_send_buffer(buf, &m);
    int bytes_sent = this->tcp_acceptor.send_data(&buf, len);
    if (bytes_sent > 0) fprintf(stdout, "Sent mavlink message (%d bytes)\n", bytes_sent);
    else {
        fprintf(stderr, "Error in sending MAVLink message.\n");
        return false;
    }
    return true;
}

bool MAVLinkConnectionHandler::connection_open() {
    return this->tcp_acceptor.connected();
}

void MAVLinkConnectionHandler::set_message_handler(MAVLinkMessageHandler* h) {
    this->message_handler = h;
}
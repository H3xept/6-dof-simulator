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

void MAVLinkConnectionHandler::handle_mavlink_message(mavlink_message_t m) {
    ConsoleLogger* logger = ConsoleLogger::shared_instance();
    switch(m.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:
            logger->log("MSG: HEARTBEAT");
            break;
        case MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS:
            logger->log("MSG: HIL_ACTUATOR_CONTROLS");
            break;
        case MAVLINK_MSG_ID_HIL_CONTROLS:
            logger->log("MSG: HIL_CONTROLS");
            break;
        case MAVLINK_MSG_ID_COMMAND_LONG:
            logger->log("MSG: COMMAND_LONG");
            break;
        default:
            logger->log("Unknown message!");
    }
}

bool MAVLinkConnectionHandler::received_message(mavlink_message_t m) {
    printf("Received mavlink message! (%d)\n", m.msgid);
    this->handle_mavlink_message(m);
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
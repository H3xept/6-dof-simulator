#ifndef __MAVLINKMESSAGEHANDLER_H__
#define __MAVLINKMESSAGEHANDLER_H__

#include <mavlink.h>

class MAVLinkMessageHandler {
public:
    ~MAVLinkMessageHandler() {};
    virtual void handle_heartbeat(mavlink_message_t m) = 0;
};

#endif // __MAVLINKMESSAGEHANDLER_H__
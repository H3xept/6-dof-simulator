#ifndef __MAVLINKMESSAGERELAY_H__
#define __MAVLINKMESSAGERELAY_H__

#include <mavlink.h>

class MAVLinkMessageRelay {
public:
    virtual bool received_message(mavlink_message_t m) = 0;
    virtual bool send_message(const mavlink_message_t& m) = 0;
    virtual bool connection_open() = 0;
};

#endif // __MAVLINKMESSAGERELAY_H__
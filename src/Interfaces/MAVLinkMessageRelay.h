#ifndef __MAVLINKMESSAGERELAY_H__
#define __MAVLINKMESSAGERELAY_H__

#include <mavlink.h>

class MAVLinkMessageRelay {
    virtual bool received_message(mavlink_message_t m) = 0;
    virtual bool send_message(mavlink_message_t& m) = 0;
};

#endif // __MAVLINKMESSAGERELAY_H__
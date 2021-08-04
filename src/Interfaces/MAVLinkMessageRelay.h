#ifndef __MAVLINKMESSAGERELAY_H__
#define __MAVLINKMESSAGERELAY_H__

#include <mavlink.h>
#include "MAVLinkConnection.h"

class MAVLinkMessageRelay {
    virtual bool received_message(mavlink_message_t m);
    virtual bool send_message(MAVLinkConnection c, mavlink_message_t& m);
};

#endif // __MAVLINKMESSAGERELAY_H__
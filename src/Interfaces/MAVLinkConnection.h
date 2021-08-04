#ifndef __MAVLINKCONNECTION_H__
#define __MAVLINKCONNECTION_H__

#include <exception>
#include "MAVLinkTarget.h"

class MAVLinkConnection {
    virtual bool connect();
    virtual bool disconnect();
    virtual void connection_enstabilished();
    virtual bool connection_error(std::exception e);
public: 
    MAVLinkTarget source;
    MAVLinkTarget target;
};

#endif // __MAVLINKCONNECTION_H__
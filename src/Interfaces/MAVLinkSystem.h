#ifndef __MAVLINKSYSTEM_H__
#define __MAVLINKSYSTEM_H__

#include <mavlink.h>
#include <boost/asio.hpp>
#include "TimeHandler.h"
#include "MAVLinkMessageRelay.h"
#include "Alive.h"

using namespace boost::asio;

class MAVLinkSystem : public Alive, public TimeHandler {
private:
    // Milliseconds
    double heartbeat_interval = 1000;
    std::chrono::steady_clock::time_point last_heartbeat = std::chrono::steady_clock::now();
public:
    uint8_t system_id;
    uint8_t component_id;

    virtual MAVLinkMessageRelay& get_mavlink_message_relay() = 0;

    MAVLinkSystem(uint8_t system_id, uint8_t component_id) : 
        system_id(system_id), 
        component_id(component_id) {}
    
    virtual double get_heartbeat_interval() { return this->heartbeat_interval; }
    virtual void set_heartbeat_interval(double interval) { this->heartbeat_interval = interval; }

    void update(double dt) override {
        auto now = std::chrono::steady_clock::now();
        double elapsed_time = std::chrono::duration_cast<chrono::milliseconds>(now - this->last_heartbeat).count();
        if (elapsed_time >= this->heartbeat_interval) {
            this->send_heartbeat();
            this->last_heartbeat = now;
        }
    }

    void send_heartbeat() override {
        MAVLinkMessageRelay& relay = this->get_mavlink_message_relay();
        if (!relay.connection_open()) { return; }
        mavlink_message_t hb;
        mavlink_msg_heartbeat_pack(this->system_id, this->component_id, &hb, MAV_TYPE_HELICOPTER, MAV_AUTOPILOT_PX4, MAV_MODE_GUIDED_ARMED, 0, MAV_STATE_ACTIVE);
        relay.send_message(hb);
    }

    void received_heartbeat() override {
        printf("Hearbeat received from PX4\n");
    }
    
};

#endif // __MAVLINKSYSTEM_H__
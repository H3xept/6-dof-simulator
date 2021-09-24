#ifndef __GODOT_ROUTER_H__
#define __GODOT_ROUTER_H__

#include "../Interfaces/DroneStateProcessor.h"
#include <Eigen/Eigen>
#include "../Helpers/rotationMatrix.h"
#include "../Helpers/rotation_utils.h"
#include "../Sockets/UDPSender.h"
#include <boost/asio.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/optional.hpp>
#include <iostream>
#include <sstream>
#include <cstdlib>
#include "../Helpers/json.hh"

class GodotRouter : public DroneStateProcessor {
private:    
    UDPSender sender;
public:

    GodotRouter(boost::asio::io_service& service) : sender(service) {}

    std::stringstream state_to_json(Eigen::VectorXd xyz, Eigen::VectorXd rpy) {
        nlohmann::json data;
        boost::property_tree::ptree pt;
        data["position"] = {xyz[0], -xyz[2], xyz[1]};
        data["attitude"] = {rpy[1], -rpy[3], rpy[2], rpy[0]};
        std::stringstream ss;
        ss << data.dump();
        return ss;
    }

    void new_drone_state(Eigen::VectorXd state, Eigen::VectorXd dx_state) {
        DroneStateProcessor::new_drone_state(state, dx_state);
        Eigen::VectorXd gyro = state.segment(9, 3);
        Eigen::VectorXd xyz = state.segment(0, 3);
        Eigen::VectorXd rpy = euler_angles_to_quaternions(state.segment(6, 3));
        sender.send_data(state_to_json(xyz, rpy).str());
    }
};

#endif // __GODOT_ROUTER_H__
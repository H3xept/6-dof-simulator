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

class GodotRouter : public DroneStateProcessor {
private:    
    UDPSender sender;
public:

    GodotRouter(boost::asio::io_service& service) : sender(service) {}

    std::stringstream state_to_json(Eigen::VectorXd xyz, Eigen::VectorXd rpy) {
        boost::property_tree::ptree pt;
        std::string xyz_string = "[" + std::to_string(xyz[0]) + "," + std::to_string(xyz[1]) + "," + std::to_string(xyz[2]) + "]";
        std::string rpy_string = "[" + std::to_string(rpy[0]) + "," + std::to_string(rpy[1]) + "," + std::to_string(rpy[2]) + "," + std::to_string(rpy[3]) + "]";
        pt.put("position", xyz_string);
        pt.put("attitude", rpy_string);
        std::stringstream ss;
        boost::property_tree::json_parser::write_json(ss, pt);
        return ss;
    }

    void new_drone_state(Eigen::VectorXd state, Eigen::VectorXd dx_state) {
        DroneStateProcessor::new_drone_state(state, dx_state);
        Eigen::VectorXd gyro = state.segment(9, 3);
        Eigen::VectorXd xyz = state.segment(0, 3);
        Eigen::VectorXd rpy = euler_angles_to_quaternions(state.segment(6, 3));
        // remote_endpoint = boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string("127.0.0.1"), 1234);
        // boost::system::error_code err;
        // socket.send_to(this->state_to_json(xyz, rpy).str().c_str(), remote_endpoint, 0, err);
        sender.send_data(state_to_json(xyz, rpy).str());
        printf("Sending\n");
    }
};

#endif // __GODOT_ROUTER_H__
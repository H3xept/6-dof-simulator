#ifndef __DRONE_H__
#define __DRONE_H__

#include <mavlink.h>
#include <Eigen/Eigen>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <chrono>
#include <boost/numeric/odeint.hpp>
#include <boost/lockfree/queue.hpp>
#include <EquationsOfMotion/FixedWingEOM.h>
#include "Containers/DroneConfig.h"
#include "Interfaces/DynamicObject.h"
#include "Interfaces/MAVLinkSystem.h"
#include "Interfaces/Alive.h"
#include "ClassExtensions/FixedWingEOM_Extension.h"
#include "Interfaces/MAVLinkMessageRelay.h"
#include "Interfaces/MAVLinkMessageHandler.h"
#include "Interfaces/DroneStateEncoder.h"
#include "Propellers.h"
#include "Ailerons.h"

#define STATE_SIZE 12

typedef boost::numeric::odeint::runge_kutta_dopri5<Eigen::VectorXd,double,Eigen::VectorXd,double,boost::numeric::odeint::vector_space_algebra> ODESolver;

class Drone : public DynamicObject,
              public MAVLinkSystem,
              public MAVLinkMessageHandler,
              public DroneStateEncoder {
private:
    
    double drone_time = .0;
    bool armed = false;

    Propellers propellers;
    Ailerons ailerons;

    DroneConfig config;

    Eigen::VectorXd state;
    Eigen::VectorXd dx_state;
    FixedWingEOM dynamics;
    ODESolver dynamics_solver;

    MAVLinkMessageRelay& connection;
    boost::lockfree::queue<mavlink_message_t, boost::lockfree::capacity<50>> message_queue;
    
    std::chrono::steady_clock::time_point last_autopilot_telemetry = std::chrono::steady_clock::now();
    uint16_t hil_state_quaternion_message_frequency = 1000; // Default frequency of 1s

    void _setup_drone();

    void _process_mavlink_message(mavlink_message_t m);
    void _process_mavlink_messages();
    void _process_command_long_message(mavlink_message_t m);
    void _process_hil_actuator_controls(mavlink_message_t m);

    void _publish_hil_gps();
    void _publish_hil_state_quaternion();
    void _publish_state();

    void _step_dynamics(double dt);
public:

    Drone(char* config_file, MAVLinkMessageRelay& connection);
    ~Drone() {};

    Eigen::VectorXd& get_state() override { return this->state; }
    DroneConfig get_config() { return this->config; }
    bool is_armed() { return this->armed; }

    void update(double dt) override;
    MAVLinkMessageRelay& get_mavlink_message_relay() override;
    // Receives mavlink message from non-main thread
    // Should store messages in queue and process them within the update loop.
    void handle_mavlink_message(mavlink_message_t m) override;

    uint64_t get_sim_time() override;
    Eigen::VectorXd& get_vector_state() override;
    Eigen::VectorXd& get_vector_dx_state() override;
};

#endif // __DRONE_H__
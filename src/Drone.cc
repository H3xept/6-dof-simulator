#include "Drone.h"
#include "Logging/ConsoleLogger.h"

DroneConfig config_from_file_path(char* path) {
    DroneConfig conf;
    std::ifstream fin(path);
    fin >> conf;
    return conf;
}

Drone::Drone(char* config_file, MAVLinkMessageRelay& connection) : 
    MAVLinkSystem::MAVLinkSystem(1, 200),
    state(STATE_SIZE),
    dx_state(STATE_SIZE),
    config(config_from_file_path(config_file)),
    dynamics(config),
    dynamics_solver(),
    connection(connection)
    {
        this->connection.set_message_handler(this);
    }

void Drone::update(double dt) {
    MAVLinkSystem::update(dt);
    this->_process_mavlink_messages();
    this->_publish_state();
}

MAVLinkMessageRelay& Drone::get_mavlink_message_relay() {
    return this->connection;
}

void Drone::_publish_hil_gps() {
    ConsoleLogger* logger = ConsoleLogger::shared_instance();
    logger->debug_log("Publishing HIL_GPS message");
}

void Drone::_publish_hil_state_quaternion() {
    
    ConsoleLogger* logger = ConsoleLogger::shared_instance();
    mavlink_message_t msg = this->hil_state_quaternion_msg(this->system_id, this->component_id);
    logger->debug_log("Publishing HIL_STATE_QUATERNION message");
    this->connection.send_message(msg);
}

void Drone::_publish_state() {
    if (!this->connection.connection_open()) return;

    auto now = std::chrono::steady_clock::now();
    bool should_send_autopilot_telemetry = 
        std::chrono::duration_cast<std::chrono::milliseconds>
        (now - this->last_autopilot_telemetry).count()
        > this->hil_state_quaternion_message_frequency;

    if (!should_send_autopilot_telemetry) return;

    this->last_autopilot_telemetry = now;
    this->_publish_hil_gps();
    this->_publish_hil_state_quaternion();
}

/**
 * Process a MAVLink command.
 * Correct command receival must be ACK'ed.
 * 
 */
void Drone::_process_command_long_message(mavlink_message_t m) {
    mavlink_command_long_t command;
    mavlink_message_t command_ack_msg;
    mavlink_msg_command_long_decode(&m, &command);
    uint16_t command_id = command.command;
    
    switch(command_id) {
        case MAV_CMD_SET_MESSAGE_INTERVAL:
            printf("Simulator -> PX4 message interval now set to %f (us?)\n", command.param2);
            this->hil_state_quaternion_message_frequency = command.param1;
            break;
        default:
            fprintf(stdout, "Unknown command id from command long (%d)", command_id);
    }
    
    mavlink_msg_command_ack_pack(this->system_id, this->component_id, &command_ack_msg, command_id, 0, 0, 0, command.target_system, command.target_component);
    this->connection.send_message(command_ack_msg);
}

void Drone::_process_hil_actuator_controls(mavlink_message_t m) {
    mavlink_hil_actuator_controls_t controls;
    mavlink_msg_hil_actuator_controls_decode(&m, &controls);
    this->armed = (controls.flags & MAV_MODE_FLAG_SAFETY_ARMED) > 0;
    for (int i = 0; i < 16; i++) {
        printf("Control %d -> %f\n", i, controls.controls[i]);
    }
}

void Drone::_process_mavlink_message(mavlink_message_t m) {
    ConsoleLogger* logger = ConsoleLogger::shared_instance();
    switch(m.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:
            logger->debug_log("MSG: HEARTBEAT");
            break;
        case MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS:
            logger->debug_log("MSG: HIL_ACTUATOR_CONTROLS");
            break;
        case MAVLINK_MSG_ID_HIL_CONTROLS:
            logger->debug_log("MSG: HIL_CONTROLS");
            this->_process_hil_actuator_controls(m);
            break;
        case MAVLINK_MSG_ID_COMMAND_LONG:
            logger->debug_log("MSG: COMMAND_LONG");
            this->_process_command_long_message(m);
            break;
        default:
            logger->debug_log("Unknown message!");
    }
}

void Drone::_process_mavlink_messages() {
    this->message_queue.consume_all([this](mavlink_message_t m){
        this->_process_mavlink_message(m);
    });
}

void Drone::handle_mavlink_message(mavlink_message_t m) {
    this->message_queue.push(m);
}

#pragma mark DroneStateEncoder

uint64_t Drone::get_sim_time() {
    return 0;
}

Eigen::VectorXd& Drone::get_vector_state() {
    return this->state;
}

Eigen::VectorXd& Drone::get_vector_dx_state() {
    return this->dx_state;
}

void Drone::get_lat_lon_alt(float* lat_lon_alt) {
    return;
}

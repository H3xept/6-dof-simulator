#include "IrisQuadController.h"


Eigen::VectorXd IrisQuadController::none_controller(DroneConfig conf, boost::chrono::microseconds t) {
    return Eigen::VectorXd::Zero(4);
}

Eigen::VectorXd IrisQuadController::hold_controller(DroneConfig conf, boost::chrono::microseconds t) {
    Eigen::VectorXd control{4};
    for (auto i = 0; i < control.size(); i++) control[i] = 1;
    return control;
}

Eigen::VectorXd IrisQuadController::climb_controller(DroneConfig conf, boost::chrono::microseconds t) {
    return none_controller(conf, t);
}

Eigen::VectorXd IrisQuadController::roll_controller(DroneConfig conf, boost::chrono::microseconds t) {
    return none_controller(conf, t);
}

Eigen::VectorXd IrisQuadController::pitch_controller(DroneConfig conf, boost::chrono::microseconds t) {
    return none_controller(conf, t);
}

Eigen::VectorXd IrisQuadController::yaw_controller(DroneConfig conf, boost::chrono::microseconds t) {
    return none_controller(conf, t);
}

void IrisQuadController::update(boost::chrono::microseconds us) {
    if (!this->executing_manoeuvre) {
        printf("[WARNING] Called QuadController update without specifying a manoeuvre plan.\n");
        return;
    }
    uint8_t sections_n = this->plan.sections_n;
    boost::chrono::microseconds current_section_length = this->plan.section_length[this->plan_cursor];
    if (this->manoeuvre_timer_us > current_section_length) {
        this->transition_to_next_manouvre();
    }
    this->manoeuvre_timer_us += us;
    this->total_timer_us += us;
}
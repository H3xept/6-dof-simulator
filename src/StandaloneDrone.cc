#include "StandaloneDrone.h"

void StandaloneDrone::update(boost::chrono::microseconds us) {
    DynamicObject::update(us);
    pp_state(this->state);
}

void StandaloneDrone::fake_ground_transform(boost::chrono::microseconds us) {
    double dt = (us.count() / 1000.0) / 1000.0;
    Eigen::Vector3d position = this->sensors.get_earth_frame_position(); // NED
    Eigen::Vector3d velocity = this->sensors.get_earth_frame_velocity(); // NED
    Eigen::Vector3d acceleration = caelus_fdm::body2earth(this->state) * this->sensors.get_body_frame_acceleration();

    if (position[2] >= this->ground_height && velocity[2] + acceleration[2] * dt >= 0.0) {
        this->state[2] = 0;
        // Body frame velocity
        this->state.segment(3, 3) = Eigen::VectorXd::Zero(3);
        // // Body frame acc
        this->dx_state.segment(3, 3) = Eigen::VectorXd::Zero(3);
        this->dx_state[5] = G_FORCE;
        // Rotation rate
        this->state.segment(9, 3) = Eigen::VectorXd::Zero(3);
        // Orientation
        this->state.segment(6, 3) = Eigen::VectorXd::Zero(3);
    }
}

void StandaloneDrone::_setup_drone() {
    // Inject controllers into dynamics model
    this->setControllerThrust([this] (double dt) -> Eigen::VectorXd
        { return this->thrust_propeller.control(dt); });
    this->setControllerAero([this] (double dt) -> Eigen::VectorXd
        { return this->elevons.control(dt); });
    this->setControllerVTOL([this] (double dt) -> Eigen::VectorXd
        { return this->vtol_propellers.control(dt); });
}

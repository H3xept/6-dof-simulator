#ifndef __DRONESTATELOGGER_H__
#define __DRONESTATELOGGER_H__

#include "../Interfaces/DroneStateProcessor.h"
#include <Eigen/Eigen>
#include "../Helpers/rotationMatrix.h"

class DroneStateLogger : public DroneStateProcessor {

    void simulation_complete() {
        Eigen::VectorXd state = this->get_last_drone_state();
        Eigen::VectorXd dx_state = this->get_last_drone_dx_state();
        DroneStateProcessor::new_drone_state(state, dx_state);
        Eigen::VectorXd gyro = state.segment(9, 3);
        Eigen::VectorXd rpy = state.segment(6, 3);
        printf("Final p q r: %f %f %f\n", gyro[0], gyro[1], gyro[2]);
        printf("Final phi theta psi: %f %f %f\n", rpy[0], rpy[1], rpy[2]);
    }
};

#endif // __DRONESTATELOGGER_H__
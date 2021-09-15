#ifndef __DRONESTATELOGGER_H__
#define __DRONESTATELOGGER_H__

#include "../Interfaces/DroneStateProcessor.h"

class DroneStateLogger : public DroneStateProcessor {
    void new_drone_state(Eigen::VectorXd state, Eigen::VectorXd dx_state) override {
        printf("State xyz: %f %f %f\n", state[0], state[1], state[2]);
        printf("State rpy: %f %f %f\n", state[6], state[7], state[8]);
    }
};

#endif // __DRONESTATELOGGER_H__
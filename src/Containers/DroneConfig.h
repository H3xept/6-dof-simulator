#ifndef __DRONECONFIG_H__
#define __DRONECONFIG_H__

#include <iostream>
#include <istream>
#include <fstream>
#include <Eigen/Eigen>
#include "../Interfaces/PrettyPrintable.h"
#include "../ClassExtensions/MatrixXd_Extension.h"
#include "../ClassExtensions/APM_Extension.h"

struct DroneConfig : public PrettyPrintable {
    double mass; // kg
    double b_prop; // thrust coefficient
    double d; // drag factor
    double c; // ? 
    double S; // ?
    double b_aero; // ?
    APM drone_aero_config;
    MatrixXd J; // Inertial matrix
    
    friend std::istream &operator>>(std::istream &i, DroneConfig& dc) {
        i >> dc.mass >> dc.b_prop >> dc.d >> dc.c >> dc.S >> dc.b_aero;
        // Aero Data
        i >> dc.drone_aero_config;
        // Inertial Matrix
        i >> dc.J;
        return i;
    }

    std::string str() override {
        return std::string(
            "<DroneConfig: m(" +
            std::to_string(this->mass) +
            ") >"
        );
    }
};

#endif // __DRONECONFIG_H__
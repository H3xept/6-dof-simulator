#ifndef __IRISQUADCONTROLLER_H__
#define __IRISQUADCONTROLLER_H__

#include "Controllers/DroneController.h"
#include "Containers/DroneConfig.h"
#include "Interfaces/TimeHandler.h"
#include <Eigen/Eigen>
#include <boost/chrono.hpp>
#include <stdio.h>

enum Manoeuvre { NONE, CLIMB, HOLD, ROLL, PITCH, YAW };

struct ManoeuvrePlan {
    boost::chrono::microseconds* section_length;
    Manoeuvre* manoeuvre;
    size_t sections_n;
};

static char* manouvre_name(Manoeuvre m) {
    switch(m) {
        case Manoeuvre::NONE:
            return "Shut down";
        case Manoeuvre::CLIMB:
            return "Climb";
        case Manoeuvre::HOLD:
            return "Hold";
        case Manoeuvre::ROLL:
            return "Hold";
        case Manoeuvre::PITCH:
            return "Pitch";
        case Manoeuvre::YAW:
            return "Yaw";
        default:
            return "Unknown Manouvre";
    }
}

class IrisQuadController : public DroneController {
protected: 
    DroneConfig config;

    ManoeuvrePlan plan{NULL, NULL, 0};
    bool executing_manoeuvre = false;
    uint8_t plan_cursor = 0;

    boost::chrono::microseconds manoeuvre_timer_us{0};
    boost::chrono::microseconds total_timer_us{0};

    static Eigen::VectorXd none_controller(DroneConfig conf, boost::chrono::microseconds t);
    static Eigen::VectorXd hold_controller(DroneConfig conf, boost::chrono::microseconds t);
    static Eigen::VectorXd climb_controller(DroneConfig conf, boost::chrono::microseconds t);
    static Eigen::VectorXd roll_controller(DroneConfig conf, boost::chrono::microseconds t);
    static Eigen::VectorXd pitch_controller(DroneConfig conf, boost::chrono::microseconds t);
    static Eigen::VectorXd yaw_controller(DroneConfig conf, boost::chrono::microseconds t);

    void transition_to_next_manouvre() {

        if (++this->plan_cursor > this->plan.sections_n) {
            fprintf(stdout, "[END] Manouvre plan complete.\n");
            this->manoeuvre_timer_us = boost::chrono::microseconds{0};
            this->total_timer_us = boost::chrono::microseconds{0};
            this->executing_manoeuvre = false;
            return;
        }

        Manoeuvre next_manouvre = this->plan.manoeuvre[this->plan_cursor];
        boost::chrono::microseconds section_length = this->plan.section_length[this->plan_cursor];
        fprintf(stdout, "[TRANSITION] Starting %s (%lld us)\n", manouvre_name(next_manouvre), section_length.count());
        this->manoeuvre_timer_us = boost::chrono::microseconds{0};
    }

public:
    IrisQuadController(DroneConfig config) : config(config) {};

    Manoeuvre get_current_manoeuvre() { return this->plan.manoeuvre[this->plan_cursor]; }
    boost::chrono::microseconds get_current_manoeuvre_duration() { return this->plan.section_length[this->plan_cursor]; }
    
    boost::chrono::microseconds get_total_plan_duration_us() {
        boost::chrono::microseconds duration{0};
        uint8_t manoeuvres_n = this->plan.sections_n;
        for (auto i = 0; i < manoeuvres_n; i++) duration += this->plan.section_length[i];
        return duration;
    }

    auto controller_for_manoeuvre() -> Eigen::VectorXd(*)(DroneConfig, boost::chrono::microseconds) {
        switch(this->get_current_manoeuvre()) {
            case Manoeuvre::NONE:
                return &IrisQuadController::none_controller;
            case Manoeuvre::CLIMB:
                return &IrisQuadController::climb_controller;
            case Manoeuvre::HOLD:
                return &IrisQuadController::hold_controller;
            case Manoeuvre::ROLL:
                return &IrisQuadController::roll_controller;
            case Manoeuvre::PITCH:
                return &IrisQuadController::pitch_controller;
            case Manoeuvre::YAW:
                return &IrisQuadController::yaw_controller;
            default:
                fprintf(stdout, "Unknown manoeuvre! (%d)\n", this->get_current_manoeuvre());
                std::exit(-1);
        }
    }

    void set_plan(ManoeuvrePlan plan) {
        uint8_t manoeuvres_n = plan.sections_n;
        this->manoeuvre_timer_us = boost::chrono::microseconds{0};
        this->total_timer_us = boost::chrono::microseconds{0};
        this->executing_manoeuvre = true;
        this->plan = plan;
        this->plan_cursor = 0;
        boost::chrono::microseconds duration = this->get_current_manoeuvre_duration();
        fprintf(stdout, "[NEW PLAN] Manoeuvres n: %d | total duration (us): %lld\n", manoeuvres_n, duration.count());
        fprintf(stdout, "[INITIAL_MANOEUVRE] Starting -> %s (%lld us)\n", manouvre_name(this->get_current_manoeuvre()), this->get_current_manoeuvre_duration().count());
    }

    Eigen::VectorXd get_current_pwm_control() {
        auto func = this->controller_for_manoeuvre();
        return func(this->config, this->manoeuvre_timer_us);
    }
    
    virtual Eigen::VectorXd control(double dt) override {
        return this->get_current_pwm_control();
    }

    virtual void set_control(Eigen::VectorXd c) override {};
    void update(boost::chrono::microseconds us) override;
};

#endif // __IRISQUADCONTROLLER_H__
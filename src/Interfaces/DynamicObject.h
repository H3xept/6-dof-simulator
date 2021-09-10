#ifndef __DYNAMICOBJECT_H__
#define __DYNAMICOBJECT_H__

#include <array>
#include <Eigen/Eigen>
#include <boost/numeric/odeint.hpp>
#include <random>

#include "EnvironmentObject.h"
#include "../ClassExtensions/Aerodynamics_Extension.h"
#include "../ClassExtensions/Weight_Extension.h"
#include "../ClassExtensions/ThrustQuadrotor_Extension.h"
#include "../ClassExtensions/ThrustFixedWing_Extension.h"
#include "../Helpers/constants.h"

typedef boost::numeric::odeint::runge_kutta_dopri5<Eigen::VectorXd,double,Eigen::VectorXd,double,boost::numeric::odeint::vector_space_algebra> ODESolver;

#define DYNAMIC_OBJECT_STATE_SIZE 12
#define ZEROVEC(x) Eigen::VectorXd{x}

struct DynamicObject : public EnvironmentObject {
protected:

    Weight weight_force_m;
    ThrustFixedWing fixed_wing_thrust_m;
    ThrustQuadrotor quadrotor_thrust_m;
    Aerodynamics hor_flight_aero_force_m;

    ODESolver dynamics_solver;
    
    bool compute_quadrotor_dynamics = true;
    bool compute_fixed_wing_dynamics = false;
    bool compute_aero_dynamics = false;
    bool compute_weight_dynamics = true;

    /**
     * Drone state as populated by the CAELUS_FDM package.
     * <
     *  x , y , z    [0:3]  body-frame origin with respect to earth-frame (NED m)
     *  ẋ , ẏ , ż    [3:6]  body-frame velocity (m/s)
     *  ɸ , θ , ѱ    [6:9]  (roll, pitch, yaw) body-frame orientation with respect to earth-frame (rad)
     *  ɸ. , θ. , ѱ. [9:12] (roll., pitch., yaw.) body-frame orientation velocity (rad/s)
     * >
     */
    Eigen::VectorXd state{DYNAMIC_OBJECT_STATE_SIZE};

    /**
     *  (FixedWingEOM.h:evaluate)
     *  Drone state derivative as populated by the CAELUS_FDM package.
     *  ẋ , ẏ , ż       [0:3]  earth-frame velocity (What unit?)
     *  ẍ , ÿ , z̈       [3:6]  body-frame acceleration (m/s**2)
     *  ? , ? , ?       [6:9]  earth-frame angle rates (Euler rates?)
     *  ɸ.. , θ.. , ѱ.. [9:12] body-frame angular acceleration (What unit?)
     */
    Eigen::VectorXd dx_state{DYNAMIC_OBJECT_STATE_SIZE};

    void step_dynamics(
        boost::chrono::microseconds us,
        const Eigen::VectorXd& state,
        Eigen::VectorXd& dx_state) {

        Eigen::Vector3d total_forces = this->get_forces();
        Eigen::Vector3d total_momenta = this->get_momenta();
    
        Eigen::Vector3d Vb = state.segment(3,3);
        Eigen::Vector3d wb = state.segment(9,3);

        dx_state = Eigen::VectorXd::Zero(12);

        return;
        
        dx_state.segment(0,3) = caelus_fdm::body2earth(state)*Vb;
        dx_state.segment(3,3)  = total_forces/this->weight_force_m.get_mass(); // external forces
        dx_state.segment(3,3) -= wb.cross(Vb.eval()); // account for frame dependent acc
        // earth-frame angle rates
        dx_state.segment(6,3) = caelus_fdm::angularVelocity2eulerRate(state)*wb;
        // body-frame angular acceleration
        dx_state.segment(9,3)  = total_momenta; // external torques
        // dx_state.segment(9,3) -= wb.cross( m_I*wb.eval() ); // account for frame dependent ang acc
        // dx_state.segment(9,3)  = m_I.colPivHouseholderQr().solve(dx_state.segment(9,3).eval()); // inertia matrix into account
    }

    void integration_step(boost::chrono::microseconds us) {

        // Integrate using ODESolver
        double dt = us.count() / 1000000.0; // Microseconds to seconds
        this->dynamics_solver.do_step(
            [this, dt, us] (const Eigen::VectorXd & x, Eigen::VectorXd &dx, const double t) -> void
            {
                this->step_dynamics(us, x, dx);
                this->dx_state = dx;
            },
            this->state,
            0, // Simulation time here in milliseconds?
            this->state,
            dt
        );
    }

    void initialise_state() {
        this->state = Eigen::VectorXd::Zero(DYNAMIC_OBJECT_STATE_SIZE);
    }

    void initialise_dx_state() {
        this->dx_state = Eigen::VectorXd::Zero(DYNAMIC_OBJECT_STATE_SIZE);
    }

public:

    DynamicObject(DroneConfig config) : 
        weight_force_m(Weight{config, G_FORCE}),
        fixed_wing_thrust_m(ThrustFixedWing{config}),
        quadrotor_thrust_m(ThrustQuadrotor{config}),
        hor_flight_aero_force_m(Aerodynamics{config}) {
            this->initialise_state();
            this->initialise_dx_state();
        };

    ~DynamicObject() {};

    Eigen::VectorXd& get_vector_state() { return this->state; }
    Eigen::VectorXd& get_vector_dx_state() { return this->dx_state; }
    
    Eigen::Vector3d get_forces() {
        Eigen::Vector3d fixed_wing_force = this->compute_fixed_wing_dynamics ? this->fixed_wing_thrust_m.getF() : ZEROVEC(3);
        Eigen::Vector3d quadrotor_force = this->compute_quadrotor_dynamics ? this->quadrotor_thrust_m.getF() : ZEROVEC(3);
        Eigen::Vector3d aero_force = this->compute_aero_dynamics ? this->hor_flight_aero_force_m.getF() : ZEROVEC(3);
        Eigen::Vector3d weight_force = this->compute_weight_dynamics ? this->weight_force_m.getF() : ZEROVEC(3);
        return fixed_wing_force + quadrotor_force + aero_force + weight_force;
    }

    Eigen::Vector3d get_momenta() {
        Eigen::Vector3d fixed_wing_momentum = this->compute_fixed_wing_dynamics ? this->fixed_wing_thrust_m.getM() : ZEROVEC(3);
        Eigen::Vector3d quadrotor_momentum = this->compute_quadrotor_dynamics ? this->quadrotor_thrust_m.getM() : ZEROVEC(3);
        Eigen::Vector3d aero_momentum = this->compute_aero_dynamics ? this->hor_flight_aero_force_m.getM() : ZEROVEC(3);
        Eigen::Vector3d weight_momentum = this->compute_weight_dynamics ? this->weight_force_m.getM() : ZEROVEC(3);
        return fixed_wing_momentum + quadrotor_momentum + aero_momentum + weight_momentum;
    }

    void update(boost::chrono::microseconds us) override {
        Eigen::VectorXd state = this->get_vector_state();
        if (this->compute_fixed_wing_dynamics)
            this->fixed_wing_thrust_m.updateParamsImpl(0,state);
        if (this->compute_quadrotor_dynamics)
            this->quadrotor_thrust_m.updateParamsImpl(0,state);
        if (this->compute_aero_dynamics)
            this->hor_flight_aero_force_m.updateParamsImpl(0,state);
        if (this->compute_weight_dynamics)
            this->weight_force_m.updateParamsImpl(0,state);
        this->integration_step(us);
    }

    void setControllerVTOL(std::function<Eigen::VectorXd(double)> controller) {
        this->quadrotor_thrust_m.setController(controller);
    }

    void setControllerThrust(std::function<Eigen::VectorXd(double)> controller) {
        this->fixed_wing_thrust_m.setController(controller);
    }

    void setControllerAero(std::function<Eigen::VectorXd(double)> controller) {
        this->hor_flight_aero_force_m.setController(controller);
    }
};

#endif // __DYNAMICOBJECT_H__
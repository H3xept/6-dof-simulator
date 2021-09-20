/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
/*
------ Copyright (C) 2021 University of Strathclyde and Authors ------
-------------------- e-mail: c.greco@strath.ac.uk --------------------
----------------------- Author: Cristian Greco -----------------------
*/

#ifndef CAELUS_FDM_THRUSTQUADROTOR_H
#define CAELUS_FDM_THRUSTQUADROTOR_H

#include "BaseFM.h"

namespace caelus_fdm {

    using namespace std;

    class ThrustQuadrotor : public BaseFM {

    protected:

        // Attributes
        function<Eigen::VectorXd(double)> m_controller;
        double m_b; // thrust coefficient
        double m_d; // drug coefficient
        double m_l; // the distance between the center of the quadrotor and the center of a propeller

        Eigen::VectorXd m_Omega;

    public:

        ThrustQuadrotor(double b, double d, double l, function<Eigen::VectorXd(double)> controller) :
                BaseFM(), m_l(l), m_controller(move(controller)), m_b(b), m_d(d)
        {
        }

        virtual ~ThrustQuadrotor() = default;

        /**
         * Rotation Matrix Wind to Body Frame [Gryte, “Aerodynamic  modeling  of  the  Skywalker  X8  Fixed-Wing  UnmannedAerial  Vehicle”]
         * @param x : state as [ x y z ...                    body-frame origin wrt earth-frame
         *                      x_dot y_dot z_dot ...         body-frame velocity
         *                      phi (roll) theta (pitch) psi (yaw) ...             body-frame orientation wrt earth-frame
         *                      phi_dot theta_dot psi_dot ... body-frame orientation velocity
         *                      ]
         * @param R: Rotation Matrix Wind to Body frame
         * @return
         */

        int computeF(const double &t, const State &x) override
        {
            // set force
            m_F.resize(3);
            m_F[0] = 0.;
            m_F[1] = 0.;
            m_F[2] = -m_b*(pow(m_Omega[0],2.)+pow(m_Omega[1],2.)+pow(m_Omega[2],2.)+pow(m_Omega[3],2.));
            return 0;
        }
        int computeM(const double &t, const State &x) override
        {
            // set moment (need to check if the orientations are right)
            m_M.resize(3);
            double m_l_rotated = 0.707106 * m_l;
            m_M[0] = m_b * m_l_rotated * (pow(m_Omega[3], 2) + pow(m_Omega[2], 2) - pow(m_Omega[0], 2) - pow(m_Omega[1], 2));
            m_M[1] = m_b * m_l_rotated * (pow(m_Omega[3], 2) + pow(m_Omega[0], 2) - pow(m_Omega[2], 2) - pow(m_Omega[1], 2));
            m_M[2] = m_d*(+pow(m_Omega[0],2.)-pow(m_Omega[1],2.)+pow(m_Omega[2],2.)-pow(m_Omega[3],2.));
            return 0;
        }

        int updateParamsImpl(const double &t, const State &x) override {
            m_Omega = m_controller(t);
            auto state_F = computeF(t,x);
            auto state_M = computeM(t,x);
            return 0;
        } 

        int setController(function<Eigen::VectorXd(double)> controller) {
            this->m_controller = controller;
            return 0;
        }
    };


    class ThrustQuadrotorCCPID : public ThrustQuadrotor {

    protected:

        // Attributes
        function<Eigen::VectorXd(double,const State &x)> m_controllerCCPID;
        Eigen::VectorXd m_Ttaus = Eigen::VectorXd::Zero(4); // current control

    public:

        ThrustQuadrotorCCPID(double b, double d, double l, function<Eigen::VectorXd(double,const State &x)> controller) :
                ThrustQuadrotor(b,d,l,nullptr), m_controllerCCPID(move(controller))
        {
        }

        virtual ~ThrustQuadrotorCCPID() = default;

        int computeF(const double &t, const State &x) final {
            // set force (ORIENTATION AS IN PAPER Rubi et Al. A Survey of Path Following Control Strategies for UAVs Focused on Quadrotors)
            m_F.resize(3);
            m_F[0] = 0.;
            m_F[1] = 0.;
            m_F[2] = +m_Ttaus[0];
            return 0;
        }
        int computeM(const double &t, const State &x) final {
            // set momentum (ORIENTATION AS IN PAPER Rubi et Al. A Survey of Path Following Control Strategies for UAVs Focused on Quadrotors)
            m_M.resize(3);
            m_M[0] = +m_Ttaus[1];
            m_M[1] = +m_Ttaus[2];
            m_M[2] = +m_Ttaus[3];
            return 0;
        }

        int updateParamsImpl(const double &t, const State &x) final {
            m_Ttaus = m_controllerCCPID(t,x);
            auto state_F = computeF(t,x);
            auto state_M = computeM(t,x);
            return 0;
        }

        const function<Eigen::VectorXd(double, const State &)> &get_controller_ccpid() const {
            return m_controllerCCPID;
        }

        void set_controller_ccpid(const function<Eigen::VectorXd(double, const State &)> &controller_ccpid) {
            m_controllerCCPID = controller_ccpid;
        }

    };



}



#endif //CAELUS_FDM_THRUSTQUADROTOR_H

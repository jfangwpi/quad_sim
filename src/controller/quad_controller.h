/*
* quad_controller.h
* 
* Create on : March 2, 2018
* Description:
*
* Copyright (c) 2018 Jie Fang (jfang)
*/

#ifndef QUAD_CONTROLLER_H_
#define QUAD_CONTROLLER_H_

#include <iostream>
#include "quad_states.h"
#include "quad_client.h"

namespace srcl_quad{

struct PhysicalParamer{
    double mass;
    double g = 9.8;
    double arm_length;
    /* 0 -- plus type, 1 -- X type */
    bool type;
}

struct ControlGain{
    double kp_phi;
    double kd_p;

    double kp_theta;
    double kd_q;

    double kp_psi;
    double kd_r;
}


class Quadrotor{
    public:
        /* Physical parameter */
        PhysicalParamer phy_parameter_;
        /* Desired Euler Angle */
        EulerAngle ang_d_;
        /* Desired Angular Velocity */
        AngularVelocity ang_vel_d_;
        
        /* Actual position */
        Position pos_;
        /* Actual Euler Angle */
        EulerAngle ang_;

        Quaternion quat_;

        /* Velocity */
        Velocity vel_;
        /* Angular velocity */
        AngularVelocity ang_vel_;

        ControlGain K_;

        /* Desired speed */
        std::vector<double> motor_speed_;

    public:
        Quadrotor(double m, double length, bool type, double phi_d, double theta_d, double psi_d);

        ~Quadrotor(){};

        void AttitudeControlEuler();
        void TransitionMatrix();
}


}

#endif /* QUAD_CONTROLLER_H_ */

                      
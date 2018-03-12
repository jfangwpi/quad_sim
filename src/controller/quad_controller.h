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
#include <eigen3/Eigen/Core>
#include <vector>

#include "quad_states.h"
#include "quad_client.h"

namespace srcl_quad{

struct ControlGain{
    double kp_phi;
    double kd_p;

    double kp_theta;
    double kd_q;

    double kp_psi;
    double kd_r;
};


class AttitudeControlEuler{
    public:
        /* Quadrotor states */
        QuadrotorState qs_;

        /* Desired Euler Angle */
        EulerAngle ang_d_;
        /* Desired Angular Velocity */
        AngularVelocity ang_vel_d_;

        ControlGain K_;

        /* Desired speed */
        std::vector<double> motor_speed_;

    public:
        AttitudeControlEuler(const QuadrotorState& _qs, const EulerAngle& angle_d);

        ~AttitudeControlEuler(){};

        void AttitudeControlLoop();
        
};


}

#endif /* QUAD_CONTROLLER_H_ */

                      
/*
* quad_controller.cpp
*
* Create on: March 2, 2018
*       Author: jfang
*/

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Core>

#include "quad_controller.h"
#include "quad_states.h"


using namespace srcl_quad;

QuadControl::QuadControl(const QuadrotorState& _qs):
                        qs_(_qs)
{
    
    /* Initialize control gain */
    K_.kp_phi = 0.0;
    K_.kd_p = 0.0;
    K_.kp_theta = 0.0;
    K_.kd_q = 0.0;
    K_.kp_psi = 0.0;
    K_.kd_r = 0.0;

    /* Initialize the motor speed */
    std::vector<double> motor_speed_(4,0.0);

};

void QuadControl::AttitudeControlEuler(){
    /* Update the error of euler angle */
    std::vector<double> error_euler_angle (3, 0.0);
    error_euler_angle[0] = ang_d_.roll - qs_.ang_.roll;
    error_euler_angle[1] = ang_d_.pitch - qs_.ang_.pitch;
    error_euler_angle[2] = ang_d_.yaw - qs_.ang_.yaw;

    /* Update the error of angular velocity */
    std::vector<double> error_angular_velocity (3,0.0);
    error_angular_velocity[0] = ang_vel_d_.d_roll - qs_.ang_vel_.d_roll;
    error_angular_velocity[1] = ang_vel_d_.d_pitch - qs_.ang_vel_.d_pitch;
    error_angular_velocity[2] = ang_vel_d_.d_yaw - qs_.ang_vel_.d_yaw;

    /* Update the delta omega */
    std::vector<double> delta_omega (3,0.0);
    delta_omega[0] = K_.kp_phi * error_euler_angle[0] + K_.kd_p * error_angular_velocity[0];
    delta_omega[1] = K_.kp_theta * error_euler_angle[1] + K_.kd_q * error_angular_velocity[1];
    delta_omega[2] = K_.kp_psi * error_euler_angle[2] + K_.kd_r * error_angular_velocity[2];


    double d = qs_.arm_length;
    double c = qs_.kM_/qs_.kF_;
    /* Calculate the transition matrix */
    if (qs_.type == 1){
        std::cout << "X type" << std::endl;    
        qs_.trans_r_ << 1, 1, 1, 1,
                    sqrt(2)*d/2, -sqrt(2)*d/2, -sqrt(2)*d/2, sqrt(2)*d/2,
                    -sqrt(2)*d/2, -sqrt(2)*d/2, sqrt(2)*d/2, sqrt(2)*d/2,
                    c, -c, c, -c;

        std::cout << qs_.trans_r_ << std::endl;
    }
    else 
        std::cout << "+ type" << std::endl; 

}


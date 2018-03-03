/*
* quad_controller.cpp
*
* Create on: March 2, 2018
*       Author: jfang
*/

#include <iostream>
#include <math>
#include <vector>

#include "quad_control.h"

using namespace srcl_quad;

Quadrotor::Quadrotor(double m, double length, bool type, double phi_d, double theta_d, double psi_d, double p_d, double q_d, double r_d){
    /* Physical parameter */
    phy_parameter_.mass = m;
    phy_parameter_.arm_length = length;

    phy_parameter_.type = 1;

    /* Desired euler angle */
    ang_d_.roll = phi_d;
    ang_d_.pitch; = theta_d;
    ang_d_.yaw = psi_d;

    /* Desired angular velocity */
    ang_vel_d_.d_roll = p_d;
    ang_vel_d_.d_pitch = q_d;
    ang_vel_d_.d_yaw = r_d;

    /* Initialize control gain */
    K_.kp_phi = 0.0;
    K_.kd_p = 0.0;
    K_.kp_theta = 0.0;
    K_.kd_q = 0.0;
    K_.kp_psi = 0.0;
    K_.kd_r = 0.0;

    /* Initialize the motor speed */
    std::vector<double> motor_speed_(4,0,0);

}

void Quadrotor::AttitudeControlEuler(){
    /* Update the error of euler angle */
    std::vector<double> error_euler_angle (3, 0.0);
    error_euler_angle[0] = ang_d_.roll - ang_.roll;
    error_euler_angle[1] = ang_d_.pitch - ang_.pitch;
    error_euler_angle[2] = ang_d_.yaw - ang_.yaw;

    /* Update the error of angular velocity */
    std::vector<double> error_angular_velocity (3,0.0);
    error_angular_velocity[0] = ang_vel_d_.d_roll - ang_vel_.d_roll;
    error_angular_velocity[1] = ang_vel_d_.d_pitch - ang_vel_.d_pitch;
    error_angular_velocity[2] = ang_vel_d_.d_yaw - ang_vel_.d_yaw;

    /* Update the delta omega */
    std::vector<double> delta_omega (3,0.0);
    delta_omega[0] = K_.kp_phi * error_euler_angle[0] + K_.kd_p * error_angular_velocity[0];
    delta_omega[1] = K_.kp_theta * error_euler_angle[1] + K_.kd_q * error_angular_velocity[1];
    delat_omega[2] = K_.kp_psi * error_euler_angle[2] + K_.kd_r * error_angular_velocity[2];

    /* Calculate the transition matrix */
    if (phy_parameter_.type == 1){
        std::cout << "X type" << std::endl;

    }
    else 
        std::cout << "+ type" << std::endl; 

}

void Quadrotor::TransitionMatrix(){
    
}

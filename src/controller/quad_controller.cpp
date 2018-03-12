/*
* quad_controller.cpp
*
* Create on: March 2, 2018
*       Author: jfang
*/

#include <iostream>
#include <vector>
#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include "quad_controller.h"
#include "quad_states.h"


using namespace srcl_quad;

AttitudeControlEuler::AttitudeControlEuler(const QuadrotorState& _qs, const EulerAngle& angle_d):
                        qs_(_qs),
                        ang_d_(angle_d)
{
    /* Desried angular velocity */
    ang_vel_d_.p = 0.0;
    ang_vel_d_.q = 0.0;
    ang_vel_d_.r = 0.0;

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

void AttitudeControlEuler::AttitudeControlLoop(){
    /* Update the error of euler angle */
    std::vector<double> error_euler_angle (3, 0.0);
    error_euler_angle[0] = ang_d_.roll - qs_.ang_.roll;
    error_euler_angle[1] = ang_d_.pitch - qs_.ang_.pitch;
    error_euler_angle[2] = ang_d_.yaw - qs_.ang_.yaw;

    /* Update the error of angular velocity */
    std::vector<double> error_angular_velocity (3,0.0);
    error_angular_velocity[0] = ang_vel_d_.p - qs_.ang_vel_.p;
    error_angular_velocity[1] = ang_vel_d_.q - qs_.ang_vel_.q;
    error_angular_velocity[2] = ang_vel_d_.r - qs_.ang_vel_.r;

    /* Update the delta omega */
    Eigen::Matrix <double, 4, 1> delta_omega;
    delta_omega(0) = qs_.w_h_ + qs_.delta_w_F_;
    delta_omega(1) = K_.kp_phi * error_euler_angle[0] + K_.kd_p * error_angular_velocity[0];
    delta_omega(2) = K_.kp_theta * error_euler_angle[1] + K_.kd_q * error_angular_velocity[1];
    delta_omega(3) = K_.kp_psi * error_euler_angle[2] + K_.kd_r * error_angular_velocity[2];

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
        qs_.trans_r_inv_ = qs_.trans_r_.inverse();
        std::cout << qs_.trans_r_inv_ << std::endl;
    }
    else{
        std::cout << "+ type" << std::endl; 
        qs_.trans_r_ << 1, 1, 1, 1,
                        0, -d, 0, d,
                        -d, 0, d, 0,
                        c, -c, c, -c;
        std::cout << qs_.trans_r_ << std::endl;
        qs_.trans_r_inv_ = qs_.trans_r_.inverse();
    }

    /* Calculate the ideal force */
    Eigen::Matrix <double, 4, 1> force_desired;
    force_desired(0) = qs_.trans_r_inv_(0,0) * delta_omega(0) + qs_.trans_r_inv_(0,1) * delta_omega(1) + qs_.trans_r_inv_(0,2) * delta_omega(2) + qs_.trans_r_inv_(0,3) * delta_omega(3);
    force_desired(1) = qs_.trans_r_inv_(1,0) * delta_omega(0) + qs_.trans_r_inv_(1,1) * delta_omega(1) + qs_.trans_r_inv_(1,2) * delta_omega(2) + qs_.trans_r_inv_(1,3) * delta_omega(3);
    force_desired(2) = qs_.trans_r_inv_(2,0) * delta_omega(0) + qs_.trans_r_inv_(2,1) * delta_omega(1) + qs_.trans_r_inv_(2,2) * delta_omega(2) + qs_.trans_r_inv_(2,3) * delta_omega(3);
    force_desired(3) = qs_.trans_r_inv_(3,0) * delta_omega(0) + qs_.trans_r_inv_(3,1) * delta_omega(1) + qs_.trans_r_inv_(3,2) * delta_omega(2) + qs_.trans_r_inv_(3,3) * delta_omega(3);
    
    /* Update the motor speed */
    motor_speed_[0] = sqrt(force_desired(0)/qs_.kF_);
    motor_speed_[1] = sqrt(force_desired(1)/qs_.kF_);
    motor_speed_[2] = sqrt(force_desired(2)/qs_.kF_);
    motor_speed_[3] = sqrt(force_desired(3)/qs_.kF_);

}


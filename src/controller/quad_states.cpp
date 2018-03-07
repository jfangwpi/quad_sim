/*
* quad_controller.h
* 
* Create on : March 2, 2018
* Description:
*
* Copyright (c) 2018 Jie Fang (jfang)
*/

#include <iostream>
#include <vector>
#include <cmath>

#include "quad_states.h"

using namespace srcl_quad;

QuadrotorState::QuadrotorState():
    mass(2.0),
    g(9.8),
    arm_length(0.216),
    type(1),
    kM_(1.5e-9),
    kF_(6.11e-8)
{
    w_h_ = sqrt(mass * g /4/kF_);

    pos_.x = 0.0;
    pos_.y = 0.0;
    pos_.z = 0.0;

    ang_.roll = 0.0;
    ang_.pitch = 0.0;
    ang_.yaw = 0.0;

    quat_.q0 = 0.0;
    quat_.q1 = 0.0;
    quat_.q2 = 0.0;
    quat_.q3 = 0.0;

    ang_vel_.d_roll = 0.0;
    ang_vel_.d_pitch = 0.0;
    ang_vel_.d_yaw = 0.0;

}
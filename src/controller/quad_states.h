/*
* quad_controller.h
* 
* Create on : March 2, 2018
* Description:
*
* Copyright (c) 2018 Jie Fang (jfang)
*/

#ifndef QUAD_STATES_H_
#define QUAD_STATES_H_

#include <iostream>
#include <vector>

namespace srcl_quad{

struct Position
{
    float x;
    float y;
    float z;
};

struct EulerAngle
{
    float roll;
    float pitch;
    float yaw;
};

struct Quaternion
{
    float q0;
    float q1;
    float q2;
    float q3;
};

struct Velocity
{
    float dx;
    float dy;
    float dz;
};

struct AngularVelocity
{
    float d_roll;
    float d_pitch;
    float d_yaw;
};


}

#endif /* QUAD_STATES_H_ */
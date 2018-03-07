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
#include <eigen3/Eigen/Core>

namespace srcl_quad{

struct Position
{
    double x;
    double y;
    double z;
};

struct EulerAngle
{
    double roll;
    double pitch;
    double yaw;
};

struct Quaternion
{
    double q0;
    double q1;
    double q2;
    double q3;
};

struct Velocity
{
    double dx;
    double dy;
    double dz;
};

struct AngularVelocity
{
    double d_roll;
    double d_pitch;
    double d_yaw;
};


class QuadrotorState
{
    public: 
        QuadrotorState();
        ~QuadrotorState(){};

    public:
        /* Physical parameter */
        const double mass;
        const double g;
        const double arm_length;
        /* 0 -- plus type, 1 -- X type */
        const bool type;

        /* Motor information */
        const double kM_;
        const double kF_;

        double w_h_;

        Eigen::Matrix<double, 4, 4> trans_r_;

        /* Actual position */
        Position pos_;
        /* Actual Euler Angle */
        EulerAngle ang_;

        Quaternion quat_;

        /* Velocity */
        //Velocity vel_;
        /* Angular velocity */
        AngularVelocity ang_vel_;
};

}

#endif /* QUAD_STATES_H_ */
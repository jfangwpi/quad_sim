/*
* communicate_test.cpp
*
* Create on: March 4, 2018
*       Author: jfang
*/

#include <iostream>
#include <vector>

#include "controller/quad_controller.h"

using namespace srcl_quad;

int main(int argc, char* argv[])
{
    /* Physical parameter */
    double mass = 2;
    double length = 0.216;
    bool type = 1;

    /* Desired Euler Angle */
    double phi_d = 0.3;
    double theta_d = 0.2;
    double psi_d = 0.5;

    /* Desired angular velocity */
    double p_d = 0.0;
    double q_d = 0.0;
    double r_d = 0.0;

    Quadrotor quad(mass, length, type, phi_d, theta_d, psi_d, p_d, q_d, r_d);
    quad.AttitudeControlEuler();


    return 0;
}
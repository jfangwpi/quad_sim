/*
* communicate_test.cpp
*
* Create on: March 4, 2018
*       Author: jfang
*/

#include <iostream>
#include <vector>

#include "controller/quad_controller.h"
#include "controller/quad_states.h"

using namespace srcl_quad;

extern "C" {
    #include "extApi.h"
    //#include "extApiCustom.h" // custom remote API functions */
}


int main(int argc, char* argv[])
{
    /* Start the communication with vrep */
    std::string serverIP = "127.0.0.1";
    int serverPort = 19999;

    /* start a connection with the server */
    simxInt clientID = simxStart((simxChar*)"127.0.0.1",serverPort,true,true,2000,5);

    /* Initialize the quadrotor state */
    QuadrotorState qs;

    /* Desired Euler Angle */
    EulerAngle Desired_Euler;
    Desired_Euler.roll = 0.3;
    Desired_Euler.pitch = 0.2;
    Desired_Euler.yaw = 0.5;

    AttitudeControlEuler quad_att_control(qs,Desired_Euler);
    quad_att_control.AttitudeControlLoop();

    //quad.AttitudeControlEuler();
    // if (clientID != -1){
    //     std::cout << "INFO: connected to server." << std::endl;

    //     while (simxGetConnectionId(clientID) != -1){
    //         std::cout << "Enter the loop" << std::endl;
    //     }
    //     std::cout << "INFO: Exit control loop" << std::endl;
    //     simxFinish(clientID);
    // }
    // else{
    //     std::cout << "ERROR: Failed to connect to server" << std::endl;
    // }


    return 0;
}
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

class Quadrotor{
    public:
        Position pos_;
        EulerAngle ang_;

        Quaternion quat_;

        Velocity vel_;
        AngularVelocity ang_vel_;

        

        


}
}

#endif /* QUAD_CONTROLLER_H_ */

                      
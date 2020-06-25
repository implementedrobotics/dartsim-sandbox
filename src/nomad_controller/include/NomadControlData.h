/*
 * NomadControlData.h
 *
 *  Created on: June 21, 2020
 *      Author: Quincy Jones
 *
 * Copyright (c) <2020> <Quincy Jones - quincy@implementedrobotics.com/>
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software
 * is furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef NOMAD_CONTROLDATA_H_
#define NOMAD_CONTROLDATA_H_

// C System Files

// C++ System Files

// Third Party Includes
#include <Eigen/Dense>

// Project Include Files
#include <NomadLegController.h>


typedef enum {
  IDLE = 0,
  PASSIVE  = 1,
  STAND = 2,
  BALANCE= 3,
  LOCOMOTION = 4,
  JUMP = 5,
  ESTOP = 6
} CONTROL_MODE;

//const char *CONTROL_MODE_NAME2[] = {"IDLE", "PASSIVE", "STAND", "BALANCE", "LOCOMOTION", "JUMP", "ESTOP"};

// Struct to hold relevant control data 
struct NomadControlData
{
  CONTROL_MODE control_mode_;

  // 0 = FL, 1 = FR, 2 = RL, 3 = RR
  std::shared_ptr<NomadLegController> leg_controllers_[4];
  // Desired State Trajectory
  // State Data
  // Leg Force Outputs
   //Eigen::Vector3d leg_forces[4];
   //Eigen::Vector3d leg_torques[4];
   //Eigen::Vector3d leg_
};

#endif // NOMAD_CONTROLDATA_H_
/*
 * StandState.h
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

#ifndef NOMAD_STAND_STATE_H_
#define NOMAD_STAND_STATE_H_

// C System Files

// C++ System Files
#include <memory>
#include <string>

// Third Party Includes
#include <Eigen/Dense>

// Project Include Files
#include <NomadState.h>
#include <CubicPolynomialTrajectory.h>


class StandState : public NomadState
{

public:
    StandState();

    // Called upon a state change and we enter this state
    // current_time = current robot/controller time
    void Enter(double current_time);

    // // current_time = current robot/controller time
    // // Called upon a state change and we are exiting this state
    // void Exit(double current_time);

    // Logic to run each iteration of the state machine run
    // dt = time step for this iteration
    void Run(double dt) ;

protected:
  CubicPolynomialTrajectory stand_traj_[4];
  Eigen::Vector3d start_pos_[4];
  double current_time_;

};

#endif // NOMAD_STAND_STATE_H_
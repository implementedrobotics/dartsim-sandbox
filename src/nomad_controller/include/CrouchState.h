/*
 * CrouchState.h
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

#ifndef NOMAD_CROUCH_STATE_H_
#define NOMAD_CROUCH_STATE_H_

// C System Files

// C++ System Files
#include <memory>
#include <string>

// Third Party Includes
#include <Eigen/Dense>

// Project Include Files
#include <State.h>
#include <CubicPolynomialTrajectory.h>

class CrouchState : public FiniteStateMachine::State
{

public:
  CrouchState();

  void Enter();                     // Default Do Nothing
  void Exit();                      // Default Do Nothing

  virtual bool Transition(std::shared_ptr<FiniteStateMachine::State> pNextState) = 0; // Force Transition
  void Run(); // Override for state execution logic

protected:
    CubicPolynomialTrajectory crouch_traj_;
    double start_time_;
    Eigen::Vector3d start_pos_;
};

#endif // NOMAD_CROUCH_STATE_H_
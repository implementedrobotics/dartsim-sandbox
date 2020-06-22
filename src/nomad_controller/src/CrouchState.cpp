/*
 * State.cpp
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

// C System Files

// C++ System Files

// Third Party Includes

// Project Include Files
#include <CrouchState.h>
#include <StateTypes.h>

CrouchState::CrouchState() : FiniteStateMachine::State("CROUCH", ControllerState::STATE_CROUCH)
{
}
void CrouchState::Run()
{
    //   double time_now = g_world->getTime();
    //   double h_t = crouch_traj_.Position(time_now - start_time_);

    //   //std::cout << "H: " << h_t << "to: " << g_Controller->GetFootPosition()[2] << std::endl;
    //   Eigen::Vector3d foot_pos_desired = start_pos_;
    //   foot_pos_desired[2] = h_t;

    //   g_Controller->SetCartesianPD(Eigen::Vector3d(3000,3000,3000), Eigen::Vector3d(200,200,200));
    //   g_Controller->SetFootStateDesired(foot_pos_desired, Eigen::Vector3d::Zero());
}
void CrouchState::Enter()
{
    // next_state_ = this;

    // // Cache Current Position
    // start_pos_ = g_Controller->GetFootPosition();

    // std::cout << "POS!: "<< start_pos_ << std::endl;
    // // Compute Trajectory from Initial Foot to Stand Height
    // double crouch_height = 0.1;
    // crouch_traj_.Generate(start_pos_[2], -crouch_height, 0.0, 0.0, 0.0, 0.5);
    // start_time_ = g_world->getTime();
}
bool CrouchState::Transition(std::shared_ptr<FiniteStateMachine::State> pNextState)
{
    // switch (pNextState->Id())
    // {
    // case ControllerState::STATE_CROUCH:
    //   std::cout << "Transition from Crouch to Crouch VALID" << std::endl;
    //   break;
    // case ControllerState::STATE_STAND:
    //   std::cout << "Transition from Crouch to Stand VALID" << std::endl;
    //   next_state_ = pNextState;
    //   break;
    // default:
    //   std::cout << "Invalid State Transition." << std::endl;
    // }
}

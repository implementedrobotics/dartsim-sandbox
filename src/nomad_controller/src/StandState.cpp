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
#include <iostream>

// Third Party Includes

// Project Include Files
#include <StandState.h>
#include <StateTypes.h>

StandState::StandState() : FiniteStateMachine::State("STAND", ControllerState::STATE_STAND)
{
}
void StandState::Run()
{
    // double time_now = g_world->getTime();
    // double h_t = stand_traj_.Position(time_now - start_time_);
    // double a_t = stand_traj_.Acceleration(time_now - start_time_);

    // //std::cout << "H: " << h_t << "to: " << g_Controller->GetFootPosition()[2] << std::endl;
    // Eigen::Vector3d foot_pos_desired = start_pos_;
    // foot_pos_desired[2] = h_t;

    // g_Controller->SetCartesianPD(Eigen::Vector3d(125, 125, 125), Eigen::Vector3d(50, 50, 50));
    // g_Controller->SetFootStateDesired(foot_pos_desired, Eigen::Vector3d::Zero());

    // // F = ma
    // Eigen::Vector3d force_ff = g_Controller->Skeleton()->getMass() * Eigen::Vector3d(0, 0, -9.81);
    // if (time_now - start_time_ <= 1.0)
    // {
    //     //std::cout << g_Controller->Skeleton()->getMass() << " : " << std::endl;
    //     force_ff += g_Controller->Skeleton()->getMass() * Eigen::Vector3d(0, 0, -a_t);
    // }
    // g_Controller->SetForceFeedForward(force_ff);
}
void StandState::Enter()
{
    // next_state_ = this; 
    // // Cache Current Position
    // start_pos_ = g_Controller->GetFootPosition();
    // // Compute Trajectory from Initial Foot to Stand Height
    // double stand_height = .41;
    // stand_traj_.Generate(start_pos_[2], -stand_height, 0.0, 0.0, 0.0, 1.0);
    // start_time_ = g_world->getTime();
}
bool StandState::Transition(std::shared_ptr<FiniteStateMachine::State> pNextState)
{
    // switch (pNextState->Id())
    // {
    // case ControllerState::STATE_STAND:
    //   std::cout << "Transition from Stand to Stand VALID" << std::endl;
    //   break;
    // case ControllerState::STATE_CROUCH:
    //   std::cout << "Transition from Stand to Crouch VALID" << std::endl;
    //   next_state_ = pNextState;
    //   break;
    // default:
    //   std::cout << "Invalid State Transition." << std::endl;
    // }
}

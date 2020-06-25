/*
 * StandState.cpp
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
#include <CubicPolynomialTrajectory.h>

StandState::StandState() : NomadState("STAND", 1)
{
}
void StandState::Run(double dt)
{
    current_time_ += dt;

    //double time_now = g_world->getTime();
    for (int i = 0; i < 4; i++)
    {

        double h_t = stand_traj_[i].Position(current_time_ - start_time_);
        double a_t = stand_traj_[i].Acceleration(current_time_ - start_time_);

        //std::cout << "H: " << h_t << "to: " << g_Controller->GetFootPosition()[2] << std::endl;
        Eigen::Vector3d foot_pos_desired = start_pos_[i];
        foot_pos_desired[2] = h_t;

        control_DATA_->leg_controllers_[i]->SetCartesianPD(Eigen::Vector3d(500, 500, 500), Eigen::Vector3d(50, 50, 50));
        control_DATA_->leg_controllers_[i]->SetFootStateDesired(foot_pos_desired, Eigen::Vector3d::Zero());

        // F = ma
         Eigen::Vector3d force_ff = control_DATA_->leg_controllers_[i]->Skeleton()->getMass()/4 * Eigen::Vector3d(0, 0, -9.81);
         if (current_time_ - start_time_ <= 1.0)
         {
             //std::cout << g_Controller->Skeleton()->getMass() << " : " << std::endl;
             force_ff += control_DATA_->leg_controllers_[i]->Skeleton()->getMass()/4 * Eigen::Vector3d(0, 0, -a_t);
         }
         control_DATA_->leg_controllers_[i]->SetForceFeedForward(force_ff);
    }
}
void StandState::Enter(double current_time)
{
    // Call default enter
    State::Enter(current_time);

    current_time_ = current_time;

    // Cache Current Position
    for (int i = 0; i < 4; i++)
    {
        start_pos_[i] = control_DATA_->leg_controllers_[i]->GetFootPosition();

        // Compute Trajectory from Initial Foot to Stand Height
        double stand_height = .35;
        stand_traj_[i].Generate(start_pos_[i][2], -stand_height, 0.0, 0.0, 0.0, 1.0);
    }

    std::cout << "Trajectory Generated" << std::endl;
}

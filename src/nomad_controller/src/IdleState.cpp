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
#include <IdleState.h>
#include <StateTypes.h>

IdleState::IdleState() : FiniteStateMachine::State("IDLE", ControllerState::STATE_IDLE)
{
}
void IdleState::Run()
{
}
void IdleState::Enter()
{
std::cout << "In Idle State" << std::endl;
}
bool IdleState::Transition(std::shared_ptr<FiniteStateMachine::State> pNextState)
{
    // switch (pNextState->Id())
    // {
    // // case ControllerState::STATE_CROUCH:
    // //   std::cout << "Transition from Idle to Crouch VALID" << std::endl;
    // //   break;
    // case ControllerState::STATE_STAND:
    //   std::cout << "Transition from Idle to Stand VALID" << std::endl;
    //   next_state_ = pNextState;
    //   break;
    // default:
    //   std::cout << "Invalid State Transition." << std::endl;
    // }
}
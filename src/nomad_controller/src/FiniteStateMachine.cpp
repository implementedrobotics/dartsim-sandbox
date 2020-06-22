/*
 * FiniteStateMachine.cpp
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
#include <FiniteStateMachine.h>

namespace FiniteStateMachine
{
    FiniteStateMachine::FiniteStateMachine(const std::string &name) : name_(name), current_state_(nullptr)
    {
    }

    bool FiniteStateMachine::AddState(std::shared_ptr<State> state)
    {
        //state->parent_ = this;
        state_list_.emplace(state->Id(), state);
    }

    bool FiniteStateMachine::SetDefaultState(std::shared_ptr<State> state)
    {
        default_state_ = current_state_ = state;
    }

    bool FiniteStateMachine::SetDefaultState(int stateId)
    {
        default_state_ = current_state_ = FindState(stateId);
    }

    bool FiniteStateMachine::Reset()
    {
        if (current_state_ != nullptr)
            current_state_->Exit(); // Cleanup old state

        return Start();
    }

    bool FiniteStateMachine::Start()
    {
        current_state_ = default_state_;
        current_state_->Enter();
    }

    bool FiniteStateMachine::Run()
    {
        if (current_state_ == nullptr)
        {
            std::cout << "[ERROR]: Current state is NULL";
        }

        if (current_state_->InTransition()) // In Transition, run code
        {
            std::cout << "In state transition next state" << std::endl;
            // TODO: Call state transition callbackk
            //current_state_->RunTransition()
        }
        else if (current_state_ != current_state_->NextState()) // Transitioned, cleanup
        {
            // Transitioned
            current_state_->Exit(); // Run State Exit Code
            current_state_ = current_state_->NextState();
            current_state_->Enter(); // Run State Entry Code
            std::cout << "Successfully Transitioned to State: " << current_state_->Name() << std::endl;
        }
        else // Execute Normally
        {
            current_state_->Run();
        }
    }
    void FiniteStateMachine::TransitionTo(int state)
    {
        current_state_->Transition(FindState(state));
    }

    std::shared_ptr<State> FiniteStateMachine::FindState(int eStateId)
    {
        // TODO: Check for valid key
        return state_list_[eStateId];
    }

} // namespace FiniteStateMachine

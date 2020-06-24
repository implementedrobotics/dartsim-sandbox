/*
 * NomadPrimaryControlFSM.cpp
 *
 *  Created on: June 23, 2020
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
#include <NomadControlData.h>
#include <NomadPrimaryControlFSM.h>
#include <TransitionEvent.h>

// Transition Events For States
NomadTransitionEvent::NomadTransitionEvent(const std::string &name, std::shared_ptr<NomadControlData> control_data)
    : TransitionEvent(name), control_DATA_(control_data)
{
    control_DATA_ = control_data;
}

// States
class IdleState : public State
{

public:
    IdleState() : State("Idle", 0) {}

    void Enter()
    {
        std::cout << "Entering State IDLE" << std::endl;
    }
    void Exit()
    {
        std::cout << "Exiting State IDLE" << std::endl;
    }

    void Run()
    {
        // std::cout << "State Machine RUN" << std::endl;
    }
};

class StandState : public State
{

public:
    StandState() : State("Stand", 1) {}

    void Enter()
    {
        std::cout << "Entering State STAND" << std::endl;
    }
    void Exit()
    {
        std::cout << "Exiting State STAND" << std::endl;
    }

    void Run()
    {
        // std::cout << "State Machine RUN" << std::endl;
    }
};

// Transitions
class CommandRequestEvent : public NomadTransitionEvent
{
public:
    // Base Class Transition Event
    // name = Transition Event name
    CommandRequestEvent(const std::string &name, CONTROL_MODE mode, std::shared_ptr<NomadControlData> control_data) : NomadTransitionEvent(name, control_data), req_mode_(mode)
    {
    }

    // Stop state machine and cleans up
    bool Triggered()
    {
        if (control_DATA_->control_mode_ == req_mode_)
        {
            std::cout << "Event ID: " << name_ << " is SET!" << std::endl;
            return true;
        }
        return false;
    };

protected:
    CONTROL_MODE req_mode_;
};

NomadPrimaryControlFSM::NomadPrimaryControlFSM(std::shared_ptr<NomadControlData> control_DATA) : FiniteStateMachine("Nomad Primary Control FSM"), control_DATA_(control_DATA)
{
    _CreateFSM();
}

void NomadPrimaryControlFSM::_CreateFSM()
{
    //nomad_control_FSM_ = std::make_unique<NomadPrimaryControlFSM>();
    std::cout << "Creating FSM in NPCFSM" << std::endl;

    ///////////////////////// Define Our States
    // Idle
    std::shared_ptr<IdleState> idle = std::make_shared<IdleState>();
    // Stand
    std::shared_ptr<StandState> stand = std::make_shared<StandState>();

    std::shared_ptr<CommandRequestEvent> transitionStand = std::make_shared<CommandRequestEvent>("STAND TRANSITION", CONTROL_MODE::STAND, control_DATA_);
    std::shared_ptr<CommandRequestEvent> transitionIdle = std::make_shared<CommandRequestEvent>("IDLE TRANSITION", CONTROL_MODE::IDLE, control_DATA_);

    // Setup Transitions
    idle->AddTransitionEvent(transitionStand, stand);
    stand->AddTransitionEvent(transitionIdle, idle);

    // Add the state to the FSM
    AddState(idle);
    AddState(stand);

    // Set Initials State
    SetInitialState(idle);

    // Start the state machine
    Start();
}

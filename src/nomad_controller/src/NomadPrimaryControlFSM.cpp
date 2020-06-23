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
#include <dart/gui/osg/osg.hpp>

// Project Include Files
#include <NomadControlData.h>
#include <NomadPrimaryControlFSM.h>


int key_event2;

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
        std::cout << "Entering State EXIT" << std::endl;
    }

    void Run()
    {
        std::cout << "State Machine RUN" << std::endl;
    }
};

class KeyboardEvent : public TransitionEvent
{
public:
    // Base Class Transition Event
    // name = Transition Event name
    KeyboardEvent(int *key_event) : TransitionEvent("STAND EVENT"), key_event_(key_event)
    {
    }

    // Stop state machine and cleans up
    bool Triggered()
    {
        std::cout << "Event: " << *key_event_ << std::endl;
        if (*key_event_ == ::osgGA::GUIEventAdapter::KEY_S)
            return true;
        return false;
    };

protected:
    int *key_event_;
};

NomadPrimaryControlFSM::NomadPrimaryControlFSM() : FiniteStateMachine("Nomad Primary Control FSM")
{
    _CreateFSM();
}

void NomadPrimaryControlFSM::_CreateFSM()
{
    //nomad_control_FSM_ = std::make_unique<NomadPrimaryControlFSM>();
    std::cout << "Creating FSM in NPCFSM" << std::endl;

    std::shared_ptr<IdleState> idle = std::make_shared<IdleState>();


    std::shared_ptr<KeyboardEvent> transition = std::make_shared<KeyboardEvent>(&key_event2);
    idle->AddTransitionEvent(transition, idle);

    AddState(idle);
    SetInitialState(idle);

    Start();
}

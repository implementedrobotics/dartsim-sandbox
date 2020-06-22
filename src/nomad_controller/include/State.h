/*
 * State.h
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

#ifndef NOMAD_STATE_H_
#define NOMAD_STATE_H_

// C System Files

// C++ System Files
#include <memory>
#include <string>

// Third Party Includes


// Project Include Files

namespace FiniteStateMachine
{
class State
{
  //friend class FiniteStateMachine;

public:
  State(const std::string &name, int id) : name_(name), id_(id), in_transition_(false), next_state_(this)
  {
  }
  const std::string &Name() const { return name_; } // State Name
  const int &Id() const { return id_; }       // State ID
  virtual void Setup(){};                     // Default Do Nothing
  virtual void Enter(){};                     // Default Do Nothing
  virtual void Exit(){};                      // Default Do Nothing

  virtual bool Transition(std::shared_ptr<State> pNextState) = 0; // Force Transition
  const bool InTransition() const { return in_transition_; }
  virtual void Run() = 0; // Override for state execution logic

  std::shared_ptr<State> NextState() { return next_state_; } // Next State to transition to
  // TODO: Valid state transition?

protected:
  std::string name_;           // State Name
  int id_;         // State ID
  bool in_transition_;         // In Transition
  std::shared_ptr<State>  next_state_;  // Next State
  //std::unique_ptr<FiniteStateMachine> parent_; // Parent FSM that state is in
};
}

#endif // NOMAD_STATE_H_
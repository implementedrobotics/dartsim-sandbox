/*
 * FiniteStateMachine.h
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

#ifndef NOMAD_FINITESTATEMACHINE_H_
#define NOMAD_FINITESTATEMACHINE_H_

// C System Files

// C++ System Files
#include <memory>
#include <string>
#include <map>

// Third Party Includes


// Project Include Files
#include <State.h>

namespace FiniteStateMachine
{
class FiniteStateMachine
{
public:

  FiniteStateMachine(const std::string &name);

  bool AddState(std::shared_ptr<State> state);

  bool SetDefaultState(std::shared_ptr<State> state);

  bool SetDefaultState(int stateId);

  bool Reset();

  bool Start();

  bool Run();

  void TransitionTo(int state);

  std::shared_ptr<State> FindState(int eStateId);

  // Nomad Data Pointer

protected:
  std::shared_ptr<State> current_state_; // Set Current State
  std::shared_ptr<State> default_state_; // State to reset
  std::string name_;

  std::map<int, std::shared_ptr<State> > state_list_;
};
}

#endif // NOMAD_FINITESTATEMACHINE_H_
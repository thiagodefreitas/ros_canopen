/****************************************************************
 *
 * Copyright (c) 2014
 *
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: canopen_402
 * ROS stack name: canopen_402
 * ROS package name: canopen_402
 * Description: This class implements the CANopen device profile for
 * drives and motion control
 * CiA (r) 402
 * Standardized in IEC 61800-7-201 and IEC 61800-7-301
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Thiago de Freitas, email:tdf@ipa.fhg.de
 * Supervised by: Thiago de Freitas, email:tdf@ipa.fhg.de
 *
 * Date of creation: July 2014
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

#include <canopen_402/canopen_402.h>

using canopen::Node_402;

void Node_402::pending(LayerStatus &status)
{
  processSW(status);
  processCW();
}

bool Node_402::enterModeAndWait(const OperationMode &op_mode_var)
{
  boost::mutex::scoped_lock lock(mode_mutex_, boost::try_to_lock);
  if(!lock) return false;

  motorEvent(highLevelSM::enterStandBy());
  valid_mode_state_ = false;

  canopen::time_point abs_time = canopen::get_abs_time(boost::chrono::seconds(1));
  canopen::time_point actual_point;

  if (isModeSupported(op_mode_var) || op_mode_var == OperationMode(No_Mode))
  {

    bool transition_success = motorEvent(highLevelSM::checkModeSwitch(op_mode_var));
    motorEvent(highLevelSM::enterStandBy());

    while(transition_success == boost::msm::back::HANDLED_FALSE)
    {
      actual_point = boost::chrono::high_resolution_clock::now();
      if(op_mode_var != OperationMode(Homing))
        if(boost::chrono::duration_cast<boost::chrono::milliseconds>(actual_point - abs_time).count() >= 0 )
        {
          return false;
        }
      transition_success = motorEvent(highLevelSM::checkModeSwitch(op_mode_var));
      motorEvent(highLevelSM::enterStandBy());
    }
    valid_mode_state_ = true;
    return true;
  }
  else
  {
    return false;
  }
}


bool Node_402::isModeSupported(const OperationMode &op_mode)
{
  bool transition_success = motorEvent(highLevelSM::checkModeSupport(op_mode));
  return transition_success;
}

void Node_402::processSW(LayerStatus &status)
{
  boost::mutex::scoped_lock lock(word_mutex_, boost::try_to_lock);
  if(!lock) return;

  SwCwSM.process_event(StatusandControl::newStatusWord());
}


void Node_402::handleRead(LayerStatus &status, const LayerState &current_state)
{
  if(current_state == Init || current_state == Recover)
  {
    pending(status);
  }
  processSW(status);
}

void Node_402::handleWrite(LayerStatus &status, const LayerState &current_state)
{
  if(current_state == Init || current_state == Recover)
  {
    return;
  }
  if(motor_feedback_->state == Fault)
  {
    bool transition_success;
    transition_success =  motorEvent(highLevelSM::runMotorSM(FaultEnable));
    motorEvent(highLevelSM::enterStandBy());
  }
  move(status);
  processCW();
}

uint32_t Node_402::getModeMask(const OperationMode &op_mode)
{
  switch(op_mode)
  {
  case Profiled_Position:
  case Velocity:
  case Profiled_Velocity:
  case Profiled_Torque:
  case Interpolated_Position:
  case Cyclic_Synchronous_Position:
  case Cyclic_Synchronous_Velocity:
  case Cyclic_Synchronous_Torque:
  case Homing:
    return (1<<(op_mode-1));
  case No_Mode:
    return 0;
  }
  return 0;
}

void Node_402::processCW()
{
  boost::mutex::scoped_lock lock(word_mutex_, boost::try_to_lock);
  if(!lock) return;

  SwCwSM.process_event(StatusandControl::newControlWord());
}

void Node_402::move(LayerStatus &status)
{
  if((motor_feedback_->state == Operation_Enable) && (valid_mode_state_ == true))
  {
    bool transition_success = motorEvent(highLevelSM::enableMove(target_values_->target_pos, target_values_->target_vel));
  }
}

void Node_402::handleShutdown(LayerStatus &status)
{
  turnOff(status);
}

void Node_402::handleDiag(LayerReport &report)
{
}

void Node_402::handleHalt(LayerStatus &status)
{
  bool transition_success;

  transition_success = motorEvent(highLevelSM::runMotorSM(QuickStop));
}


void Node_402::handleRecover(LayerStatus &status)
{
  bool recover_success;

  OperationMode previous_mode = getMode();

  motorEvent(highLevelSM::stopMachine());

  recover_success = turnOn(status);

  if(recover_success)
  {
    bool transition_success;

    if(isModeSupported(previous_mode))
    {
      transition_success = enterModeAndWait(previous_mode);
      if(!transition_success)
      {
        status.error("Failed to re-enter mode in Recover");
      }
    }
  }
  else
    status.error("Failed to Recover the modules");
}

const double Node_402::getTargetPos()
{
  return target_values_->target_pos;
}
const double Node_402::getTargetVel()
{
  return target_values_->target_vel;
}


const OperationMode Node_402::getMode()
{
  if(motor_feedback_->current_mode == Homing) return No_Mode; // TODO: remove after mode switch is handled properly in init
  return motor_feedback_->current_mode;
}

const double Node_402::getActualVel()
{
  return motor_feedback_->actual_vel;
}

const double Node_402::getActualEff()
{
  return motor_feedback_->actual_eff;
}

const double Node_402::getActualPos()
{
  return motor_feedback_->actual_pos;
}

void Node_402::setTargetVel(const double &target_vel)
{
  if (motor_feedback_->state == Operation_Enable && valid_mode_state_)
  {
    target_values_->target_vel = target_vel;
  }
  else
    target_values_->target_vel = 0;
}

void Node_402::setTargetPos(const double &target_pos)
{
  if (motor_feedback_->state == Operation_Enable && valid_mode_state_)
  {
    target_values_->target_pos = target_pos;
  }
  else
    target_values_->target_pos = motor_feedback_->actual_pos;
}

bool Node_402::turnOn(LayerStatus &s)
{
  boost::mutex::scoped_lock cond_lock(cond_mutex_);
  if(!cond_lock)
    return false;

  canopen::time_point abs_time = canopen::get_abs_time(boost::chrono::seconds(2));
  canopen::time_point actual_point;

  bool transition_success;

  motorEvent(highLevelSM::startMachine());

  while(motor_feedback_->state == Start)
  {
    boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
  }

  if(motor_feedback_->state == Fault)
  {
    transition_success =  motorEvent(highLevelSM::runMotorSM(FaultEnable)); //this is the timeout in milliseconds
    if(!transition_success)
    {
      s.error("Could not properly set the device to a fault state");
      return false;
    }

    transition_success =  motorEvent(highLevelSM::runMotorSM(FaultReset)); //this is the timeout in milliseconds

    while(!transition_success)
    {
      actual_point = boost::chrono::high_resolution_clock::now();
      if(boost::chrono::duration_cast<boost::chrono::milliseconds>(actual_point - abs_time).count() >= 0 )
      {
        s.error("Could not reset fault");
        return false;
      }

      transition_success =  motorEvent(highLevelSM::runMotorSM(FaultReset)); //this is the timeout in milliseconds
    }
  }


  if(motor_feedback_->state==Quick_Stop_Active)
  {
    transition_success = motorEvent(highLevelSM::runMotorSM(DisableQuickStop));

    while(!transition_success)
    {
      actual_point = boost::chrono::high_resolution_clock::now();
      if( boost::chrono::duration_cast<boost::chrono::milliseconds>(actual_point - abs_time).count() >= 0 )
      {
        s.error("Could not disable the quick stop");
        return false;
      }
      transition_success = motorEvent(highLevelSM::runMotorSM(DisableQuickStop));
    }
  }

  transition_success = motorEvent(highLevelSM::runMotorSM(ShutdownMotor));

  while(!transition_success)
  {
    actual_point = boost::chrono::high_resolution_clock::now();
    if( boost::chrono::duration_cast<boost::chrono::milliseconds>(actual_point - abs_time).count() >= 0 )
    {
      s.error("Could not prepare the device");
      return false;
    }
    transition_success = motorEvent(highLevelSM::runMotorSM(ShutdownMotor));
  }

  transition_success = motorEvent(highLevelSM::runMotorSM(SwitchOn));


  while(!transition_success)
  {
    actual_point = boost::chrono::high_resolution_clock::now();
    if(boost::chrono::duration_cast<boost::chrono::milliseconds>(actual_point - abs_time).count() >= 0 )
    {
      s.error("Could not switch on");
      return false;
    }
    transition_success = motorEvent(highLevelSM::runMotorSM(SwitchOn));
  }

  transition_success = motorEvent(highLevelSM::runMotorSM(EnableOp));

  while(!transition_success)
  {
    actual_point = boost::chrono::high_resolution_clock::now();
    if(boost::chrono::duration_cast<boost::chrono::milliseconds>(actual_point - abs_time).count() >= 0 )
    {
      s.error("Could not enable op");
      return false;
    }
    transition_success = motorEvent(highLevelSM::runMotorSM(EnableOp));
  }

  return true;
}

template<class Event>
bool Node_402::motorEvent(Event const& evt)
{
  boost::mutex::scoped_lock lock(motor_mutex_, boost::try_to_lock);
  if(!lock) return false;

  bool transition_success;

  transition_success = motorAbstraction.process_event(evt);

  return transition_success;
}

bool Node_402::turnOff(LayerStatus &s)
{
  motorEvent(highLevelSM::runMotorSM(ShutdownMotor));

  motorEvent(highLevelSM::stopMachine());


  return true;
}


void Node_402::handleInit(LayerStatus &s)
{
  bool turn_on = Node_402::turnOn(s);

  if (turn_on)
  {
    bool transition_success;

    if(isModeSupported(Homing))
    {
      transition_success = enterModeAndWait(Homing);
      if(!transition_success)
      {
        s.error("Failed to do the homing procedure");
      }
    }
  }
  else
    s.error("Could not properly initialize the module");
}

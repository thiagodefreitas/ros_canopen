/****************************************************************
 *
 * Copyright (c) 2015
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
 * Author: Thiago de Freitas, email:thiagodefreitas@gmail.com
 * Supervised by: Thiago de Freitas, email:thiagodefreitas@gmail.com
 *
 * Date of creation: March 2015
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

#ifndef STATUS_AND_CONTROL_H
#define STATUS_AND_CONTROL_H

#include <string>
#include <vector>
#include <canopen_402/enums_402.h>
#include <boost/msm/back/state_machine.hpp>
#include <boost/msm/front/state_machine_def.hpp>
#include <boost/msm/front/functor_row.hpp>
#include <canopen_master/canopen.h>

namespace msm = boost::msm;
namespace mpl = boost::mpl;

using namespace boost::msm::front;
///////
/// \brief m
///
///
///
// the status and control state machine
namespace canopen
{
typedef std::bitset<16> word_bitset;

class StatusandControl_ : public msm::front::state_machine_def<StatusandControl_>
{
public:
  struct motorFeedback
  {
    double actual_pos;
    double actual_vel;
    double actual_eff;
    enums402::InternalState state;
    enums402::OperationMode current_mode;
    bool target_reached;
    motorFeedback() : actual_pos(0), actual_vel(0), actual_eff(0), state(enums402::Start), current_mode(enums402::No_Mode), target_reached(false){}
  };

  struct wordBitset
  {
    word_bitset status_word;
    word_bitset control_word;

    wordBitset() : status_word(), control_word() {}
  };

  struct commandTargets
  {
    double target_pos;
    double target_vel;
    enums402::InternalState target_internal_state;
    enums402::OperationMode target_mode;
    commandTargets() : target_pos(0), target_vel(0), target_internal_state(enums402::InternalState(0)), target_mode(enums402::OperationMode(0)) {}
    commandTargets(double pos) : target_pos(pos), target_vel(0), target_internal_state(enums402::InternalState(0)), target_mode(enums402::OperationMode(0)) {}
    commandTargets(double pos, double vel) : target_pos(pos), target_vel(vel), target_internal_state(enums402::InternalState(0)), target_mode(enums402::OperationMode(0)) {}
  };


  StatusandControl_(){
  }
  StatusandControl_(boost::shared_ptr<ObjectStorage> storage) :  storage_(storage), state_change_needed_(false), mode_change_needed_(false) {

    word_bitset_ = boost::make_shared<wordBitset>();

    motor_commands_ = boost::make_shared<commandTargets>();
    motor_feedback_ = boost::make_shared<motorFeedback>();
    cond_state_change_ = boost::make_shared<boost::condition_variable>();
    cond_mode_change_ = boost::make_shared<boost::condition_variable>();

    storage_->entry(status_word_entry_, 0x6041);
    storage_->entry(control_word_entry_, 0x6040);
    storage_->entry(op_mode_display, 0x6061);

    storage_->entry(actual_vel, 0x606C);
    storage_->entry(actual_pos, 0x6064);

    status_word_mask_.set(enums402::SW_Ready_To_Switch_On);
    status_word_mask_.set(enums402::SW_Switched_On);
    status_word_mask_.set(enums402::SW_Operation_enabled);
    status_word_mask_.set(enums402::SW_Fault);
    status_word_mask_.reset(enums402::SW_Voltage_enabled);
    status_word_mask_.set(enums402::SW_Quick_stop);
    status_word_mask_.set(enums402::Switch_On_Disabled);
  }


  struct newStatusWord {};
  struct newControlWord {};

  template <class Event,class FSM>
  void on_entry(Event const&,FSM& ) {/*std::cout << "entering: StatusandControl" << std::endl;*/}
  template <class Event,class FSM>
  void on_exit(Event const&,FSM& ) {/*std::cout << "leaving: StatusandControl" << std::endl;*/}

  // The list of FSM states
  struct writeControl : public msm::front::state<>
  {
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& )
    {
      // std::cout << "starting: writeControl" << std::endl;
    }
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& )
    {
      //std::cout << "finishing: writeControl" << std::endl;
    }

  };
  struct readStatus : public msm::front::state<>
  {
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& )
    {
      //std::cout << "starting: readStatus" << std::endl;
    }
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& )
    {
      //std::cout << "finishing: readStatus" << std::endl;
    }
  };

  struct stateCheck : public msm::front::state<>
  {
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& )
    {
      //std::cout << "starting: readStatus" << std::endl;
    }
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& )
    {
      //std::cout << "finishing: readStatus" << std::endl;
    }
  };

  // the initial state. Must be defined
  typedef writeControl initial_state;
  // transition actions
  void write_control(newControlWord const&)
  {
    int16_t cw_set = static_cast<int>((word_bitset_->control_word).to_ulong());

    control_word_entry_.set(cw_set);
  }

  void read_status(newStatusWord const&)
  {
    std::bitset<16> sw_new(status_word_entry_.get());

    word_bitset_->status_word = sw_new;
    switch (((word_bitset_->status_word) & status_word_mask_).to_ulong())
    {
    case 0b0000000:
    case 0b0100000:
      motor_feedback_->state = enums402::Not_Ready_To_Switch_On;
      break;
    case 0b1000000:
    case 0b1100000:
      motor_feedback_->state =  enums402::Switch_On_Disabled;
      break;
    case 0b0100001:
      motor_feedback_->state =  enums402::Ready_To_Switch_On;
      break;
    case 0b0100011:
      motor_feedback_->state =  enums402::Switched_On;
      break;
    case 0b0100111:
      motor_feedback_->state =  enums402::Operation_Enable;
      break;
    case 0b0000111:
      motor_feedback_->state =  enums402::Quick_Stop_Active;
      break;
    case 0b0001111:
    case 0b0101111:
      motor_feedback_->state =  enums402::Fault_Reaction_Active;
      break;
    case 0b0001000:
    case 0b0101000:
      motor_feedback_->state =  enums402::Fault;
      break;
    default:
      LOG("Motor currently in an unknown state");
    }

    if(state_change_needed_ || mode_change_needed_)
    {
      if (motor_feedback_->state == motor_commands_->target_internal_state)
      {
        cond_state_change_->notify_all();
        state_change_needed_ = false;
      }

      if (motor_feedback_->current_mode == motor_commands_->target_mode)
      {
        cond_mode_change_->notify_all();
        mode_change_needed_ = false;
      }
    }

    motor_feedback_->current_mode = (enums402::OperationMode) op_mode_display.get();
    motor_feedback_->actual_vel = actual_vel.get();
    motor_feedback_->actual_pos = actual_pos.get();
    motor_feedback_->actual_eff = 0; //Currently,no effort value is directly obtained from the HW

    motor_feedback_->target_reached = word_bitset_->status_word.test(enums402::SW_Target_reached); //Currently,no effort value is directly obtained from the HW
  }
  // guard conditions

  typedef StatusandControl_ sc; // makes transition table cleaner
  // Transition table for StatusandControl
  struct transition_table : mpl::vector<
      //      Start     Event         Next      Action               Guard
      //    +---------+-------------+---------+---------------------+----------------------+
      a_row < writeControl   , newStatusWord    , readStatus   , &sc::read_status                       >,
      a_row < writeControl   , newControlWord    , writeControl   , &sc::write_control                       >,
      a_row < readStatus   , newControlWord, writeControl   , &sc::write_control                       >,
      a_row < readStatus   , newStatusWord, readStatus   , &sc::read_status                       >
      > {};
  // Replaces the default no-transition response.
  template <class FSM,class Event>
  void no_transition(Event const& e, FSM&,int state)
  {
    //    std::cout << "no transition from state " << state
    //              << " on event " << typeid(e).name() << std::endl;
  }

  template <class FSM,class Event>
  void exception_caught (Event const&,FSM& fsm,std::exception& ){}


  boost::shared_ptr<motorFeedback> getFeedback()
  {
    return motor_feedback_;
  }

  boost::shared_ptr<commandTargets> updateCommands()
  {
    return motor_commands_;
  }

  boost::shared_ptr<wordBitset> getWords()
  {
    return word_bitset_;
  }

  void setStateChangeNeeded()
  {
    state_change_needed_ = true;
  }

  void setModeChangeNeeded()
  {
    mode_change_needed_ = true;
  }

  boost::shared_ptr<boost::condition_variable> getStateChangeCondition()
  {
    return cond_state_change_;
  }

  boost::shared_ptr<boost::condition_variable> getModeChangeCondition()
  {
    return cond_mode_change_;
  }


private:
  boost::shared_ptr<wordBitset> word_bitset_;
  std::bitset<16> status_word_mask_;
  boost::shared_ptr<motorFeedback> motor_feedback_;
  boost::shared_ptr<commandTargets> motor_commands_;
  bool state_change_needed_;
  bool mode_change_needed_;

  boost::shared_ptr<boost::condition_variable> cond_state_change_;
  boost::shared_ptr<boost::condition_variable> cond_mode_change_;

  canopen::ObjectStorage::Entry<canopen::ObjectStorage::DataType<0x006>::type >  status_word_entry_;
  canopen::ObjectStorage::Entry<canopen::ObjectStorage::DataType<0x006>::type >  control_word_entry_;

  boost::shared_ptr<ObjectStorage> storage_;

  canopen::ObjectStorage::Entry<int32_t> actual_vel;
  canopen::ObjectStorage::Entry<int32_t> actual_pos;
  canopen::ObjectStorage::Entry<int32_t> actual_internal_pos;

  canopen::ObjectStorage::Entry<int8_t>  op_mode_display;
};
// back-end
typedef msm::back::state_machine<StatusandControl_> StatusandControl;
};
/// */
///
#endif // STATUS_AND_CONTROL_H

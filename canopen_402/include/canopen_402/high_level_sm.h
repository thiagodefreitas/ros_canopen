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

#ifndef HIGH_LEVEL_SM_H
#define HIGH_LEVEL_SM_H
///////
/// \brief m
///
///
///
// the high level state machine
#include <boost/msm/back/state_machine.hpp>
#include <boost/msm/front/state_machine_def.hpp>
#include <boost/msm/front/functor_row.hpp>
#include <canopen_402/enums_402.h>
#include <canopen_402/internal_sm.h>
#include <canopen_402/status_and_control.h>
#include <canopen_402/mode_switch.h>
#include <boost/thread/thread.hpp>
#include <canopen_master/canopen.h>

namespace msm = boost::msm;
namespace mpl = boost::mpl;


using namespace boost::msm::front;

namespace canopen
{
class highLevelSM_ : public msm::front::state_machine_def<highLevelSM_>
{
public:
  highLevelSM_(){}
  highLevelSM_(boost::shared_ptr<ObjectStorage> &storage, boost::shared_ptr<StatusandControl> &statusandControlMachine)
    : previous_mode_(enums402::No_Mode), storage_(storage), statusandControlMachine_(statusandControlMachine), old_pos_(0)
  {
    ///////////////////*
    ///
    ///
    ///
    ///
    ///
    state_change_mutex_ = boost::make_shared<boost::mutex>();
    mode_change_mutex_ = boost::make_shared<boost::mutex>();

    storage_->entry(op_mode, 0x6060);
    storage_->entry(supported_drive_modes, 0x6502);

    storage_->entry(homing_method, 0x6098);

    supported_modes_ = supported_drive_modes.get_cached();

    motorStateMachine = motorSM(statusandControlMachine_);
    motorStateMachine.start();
    motorStateMachine.process_event(motorSM::boot());

    if(isSupported(enums402::Homing))
    {
      homingModeMachine_ = boost::make_shared<HomingSM>(HomingSM(statusandControlMachine_));
      homingModeMachine_->start();
    }

    if(isSupported(enums402::Velocity))
    {
      velModeMachine_ = boost::make_shared<velModeSM>(velModeSM(statusandControlMachine_, storage_));
      velModeMachine_->start();
    }

    if(isSupported(enums402::Interpolated_Position))
    {
      ipModeMachine_ = boost::make_shared<IPModeSM>(IPModeSM(statusandControlMachine_, storage_));
      ipModeMachine_->start();
    }

    if(isSupported(enums402::Profiled_Position))
    {
      ppModeMachine_ = boost::make_shared<ppModeSM>(ppModeSM(statusandControlMachine_, storage_));
      ppModeMachine_->start();
    }

    if(isSupported(enums402::Profiled_Velocity))
    {
      pvModeMachine_ = boost::make_shared<pvModeSM>(pvModeSM(statusandControlMachine_, storage_));
      pvModeMachine_->start();
    }

    //
    //
    //
    modeSwitchMachine = ModeSwitchSM(ipModeMachine_, velModeMachine_, homingModeMachine_, pvModeMachine_, ppModeMachine_);
    modeSwitchMachine.start();
  }
  struct startMachine {};
  struct stopMachine {};

  struct enterStandBy {};

  struct updateSwitch {};
  struct updateMotor {};

  struct checkModeSwitch
  {
    enums402::OperationMode op_mode;
    canopen::time_point  timeout;

    checkModeSwitch( enums402::OperationMode mode, canopen::time_point timeOut) : op_mode(mode), timeout(timeOut) {}
  };
  struct enableMove
  {
    double pos;
    double vel;

    enableMove() : pos(0), vel(0) {}
    enableMove(double pos) : pos(pos), vel(0) {}
    enableMove( double pos, double vel) : pos(pos), vel(vel) {}
  };

  struct checkModeSupport
  {
    enums402::OperationMode op_mode;

    checkModeSupport( enums402::OperationMode mode) : op_mode(mode) {}
  };

  struct runMotorSM
  {
    enums402::InternalActions action;
    canopen::time_point timeout;

    runMotorSM(enums402::InternalActions actionToTake, canopen::time_point timeOut) : action(actionToTake), timeout(timeOut) {}
  };

  motorSM motorStateMachine;
  ModeSwitchSM modeSwitchMachine;

  // The list of FSM states
  struct StartUp : public msm::front::state<>
  {
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& ) {/*std::cout << "starting: StartUp" << std::endl;*/}
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& ) {/*std::cout << "finishing: StartUp" << std::endl;*/}

  };
  // The list of FSM states
  struct Standby : public msm::front::state<>
  {
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& ) {/*std::cout << "starting: Standby" << std::endl;*/}
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& ) {/*std::cout << "finishing: Standby" << std::endl;*/}

  };

  // The list of FSM states
  struct updateMotorSM : public msm::front::state<>
  {
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& ) {/*std::cout << "starting: updateMotorSM" << std::endl;*/}
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& ) {/*std::cout << "finishing: updateMotorSM" << std::endl;*/}

  };

  struct ModeSwitch : public msm::front::state<>
  {
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& ) {/*std::cout << "starting: ModeSwitch" << std::endl;*/}
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& ) {/*std::cout << "finishing: ModeSwitch" << std::endl;*/}
  };

  struct Move : public msm::front::state<>
  {
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& ) {/*std::cout << "starting: Move" << std::endl;*/}
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& )
    {/*std::cout << "finishing: Move" << std::endl;*/}
  };

  void enter_standby(enterStandBy const&)
  {
    statusandControlMachine_->updateCommands()->target_pos = statusandControlMachine_->getFeedback()->actual_pos;
    statusandControlMachine_->updateCommands()->target_vel = 0;
  }

  template <class runMotorSM> void run_motor_sm(runMotorSM const& evt)
  {
    bool transition_success;

    switch(evt.action)
    {
    case enums402::QuickStop:
      modeSwitchMachine.process_event(ModeSwitchSM::deactivateMode(previous_mode_));
      motorStateMachine.process_event(motorSM::quick_stop());
      transition_success = check_state_change(enums402::Quick_Stop_Active, evt.timeout);
      if(!transition_success)
        BOOST_THROW_EXCEPTION(std::invalid_argument("The transition was not successful"));
      break;

    case enums402::FaultReset:
      motorStateMachine.process_event(motorSM::fault_reset());
      transition_success = check_state_change(enums402::Fault, evt.timeout, true);
      if(!transition_success)
        BOOST_THROW_EXCEPTION(std::invalid_argument("The transition was not successful"));
      break;

    case enums402::ShutdownMotor:
      motorStateMachine.process_event(motorSM::shutdown());
      transition_success = check_state_change(enums402::Ready_To_Switch_On, evt.timeout);
      if(!transition_success)
        BOOST_THROW_EXCEPTION(std::invalid_argument("The transition was not successful"));
      break;

    case enums402::SwitchOn:
      motorStateMachine.process_event(motorSM::switch_on());
      transition_success = check_state_change(enums402::Switched_On, evt.timeout);
      if(!transition_success)
        BOOST_THROW_EXCEPTION(std::invalid_argument("The transition was not successful"));
      break;

    case enums402::EnableOp:
      motorStateMachine.process_event(motorSM::enable_op());
      transition_success = check_state_change(enums402::Operation_Enable, evt.timeout);
      if(!transition_success)
        BOOST_THROW_EXCEPTION(std::invalid_argument("The transition was not successful"));
      break;

    case enums402::FaultEnable:
      modeSwitchMachine.process_event(ModeSwitchSM::deactivateMode(previous_mode_));
      motorStateMachine.process_event(motorSM::fault());
      transition_success = check_state_change(enums402::Fault, evt.timeout);
      if(!transition_success)
        BOOST_THROW_EXCEPTION(std::invalid_argument("The transition was not successful"));
      break;

    case enums402::DisableQuickStop:
      motorStateMachine.process_event(motorSM::disable_voltage());
      if(statusandControlMachine_->getFeedback()->state != enums402::Not_Ready_To_Switch_On && statusandControlMachine_->getFeedback()->state != enums402::Switch_On_Disabled)
        BOOST_THROW_EXCEPTION(std::invalid_argument("The transition was not successful"));
      break;

    default:
      BOOST_THROW_EXCEPTION(std::invalid_argument("Action not specified"));
    }
  }

  template <class checkModeSwitch> void switch_mode(checkModeSwitch const& evt)
  {
    bool transition_sucess;

    op_mode.set_cached(evt.op_mode);

    transition_sucess = check_mode_change(evt.op_mode, evt.timeout);

    if(!transition_sucess && evt.op_mode!=enums402::No_Mode)
    {
      previous_mode_ = statusandControlMachine_->getFeedback()->current_mode;

      modeSwitchMachine.process_event(ModeSwitchSM::deactivateMode(previous_mode_));
      BOOST_THROW_EXCEPTION(std::invalid_argument("This operation mode can not be used"));
    }

    switch(evt.op_mode)
    {
    case enums402::No_Mode:
      modeSwitchMachine.process_event(ModeSwitchSM::deactivateMode(previous_mode_));
      break;

    case enums402::Interpolated_Position:
      modeSwitchMachine.process_event(ModeSwitchSM::deactivateMode(previous_mode_));
      modeSwitchMachine.process_event(ModeSwitchSM::selectIP());
      break;

    case enums402::Velocity:
      modeSwitchMachine.process_event(ModeSwitchSM::deactivateMode(previous_mode_));
      modeSwitchMachine.process_event(ModeSwitchSM::selectVel());
      break;

    case enums402::Profiled_Velocity:
      modeSwitchMachine.process_event(ModeSwitchSM::deactivateMode(previous_mode_));
      modeSwitchMachine.process_event(ModeSwitchSM::selectPV());
      break;

    case enums402::Profiled_Position:
      modeSwitchMachine.process_event(ModeSwitchSM::deactivateMode(previous_mode_));
      modeSwitchMachine.process_event(ModeSwitchSM::selectPP());
      break;

    case enums402::Homing:
      updateHoming();
      break;

    default:
      BOOST_THROW_EXCEPTION(std::invalid_argument("Mode not supported"));
      break;
    }
  }

  bool updateHoming()
  {
    bool transition_success;
    canopen::time_point abs_time = canopen::get_abs_time(boost::chrono::seconds(60));
    canopen::time_point actual_point;

    if (!homing_method.valid() && homing_method.get() == 0)
    {
      BOOST_THROW_EXCEPTION(std::invalid_argument("Homing invalid"));
    }
    modeSwitchMachine.process_event(ModeSwitchSM::deactivateMode(previous_mode_));
    modeSwitchMachine.process_event(ModeSwitchSM::selectHoming());
    transition_success = homingModeMachine_->process_event(HomingSM::runHomingCheck());
    while(transition_success == boost::msm::back::HANDLED_FALSE)
    {
      actual_point = boost::chrono::high_resolution_clock::now();
      if(boost::chrono::duration_cast<boost::chrono::milliseconds>(actual_point - abs_time).count() >= 0 )
      {
        BOOST_THROW_EXCEPTION(std::invalid_argument("Homing still not completed"));
      }
      transition_success = homingModeMachine_->process_event(HomingSM::runHomingCheck());
    }
  }

  bool check_mode_change(enums402::OperationMode target_mode, canopen::time_point t0)
  {
    boost::mutex::scoped_lock cond_lock(*mode_change_mutex_);
    if(!cond_lock)
      return false;

    statusandControlMachine_->updateCommands()->target_mode = target_mode;

    statusandControlMachine_->setModeChangeNeeded();

    while(statusandControlMachine_->getFeedback()->current_mode != target_mode)
    {
      if(statusandControlMachine_->getModeChangeCondition()->wait_until(cond_lock,t0)  == boost::cv_status::timeout)
      {
        return false;
      }
    }

    return true;
  }

  bool check_state_change(enums402::InternalState target_state, canopen::time_point t0, bool fault_reset=false)
  {
    boost::mutex::scoped_lock cond_lock(*state_change_mutex_);
    if(!cond_lock)
      return false;

    if(!fault_reset)
    {
      statusandControlMachine_->updateCommands()->target_internal_state = target_state;

      statusandControlMachine_->setStateChangeNeeded();
    }
    while(statusandControlMachine_->getFeedback()->state != target_state || fault_reset)
    {
      if(statusandControlMachine_->getStateChangeCondition()->wait_until(cond_lock,t0)  == boost::cv_status::timeout)
      {
        if(fault_reset)
          if(statusandControlMachine_->getFeedback()->state != target_state)
          {
            return true;
          }
        return false;
      }
    }

    return true;
  }

  bool isSupported(const enums402::OperationMode &op_mode)
  {
    uint32_t masked_mode;
    bool supported;

    switch(op_mode)
    {
    case enums402::Profiled_Position:
    case enums402::Velocity:
    case enums402::Profiled_Velocity:
    case enums402::Profiled_Torque:
    case enums402::Interpolated_Position:
    case enums402::Cyclic_Synchronous_Position:
    case enums402::Cyclic_Synchronous_Velocity:
    case enums402::Cyclic_Synchronous_Torque:
    case enums402::Homing:
      masked_mode = (1<<(op_mode-1));
      break;
    case enums402::No_Mode:
    default:
      masked_mode = 0;
      break;
    }

    supported = supported_modes_ & masked_mode;

    return supported;
  }

  template <class checkModeSupport> void check_support(checkModeSupport const& evt)
  {
    bool supported = isSupported(evt.op_mode);

    if(!supported)
      BOOST_THROW_EXCEPTION(std::invalid_argument("Mode not supported"));
  }

  template <class enableMove> void move(enableMove const& evt)
  {
    switch(statusandControlMachine_->getFeedback()->current_mode)
    {
    case enums402::Interpolated_Position:
      ipModeMachine_->process_event(IPModeSM::selectMode());
      ipModeMachine_->process_event(IPModeSM::enable());
      ipModeMachine_->process_event(IPModeSM::setTarget(evt.pos, evt.vel));
      break;

    case enums402::Velocity:
      velModeMachine_->process_event(velModeSM::selectMode());
      velModeMachine_->process_event(velModeSM::enable());
      velModeMachine_->process_event(velModeSM::setTarget(evt.vel));
      break;

    case enums402::Homing:
      homingModeMachine_->process_event(HomingSM::enable());
      homingModeMachine_->current_state();
      break;

    case enums402::Profiled_Position:
      ppModeMachine_->process_event(ppModeSM::selectMode());
      if(evt.pos != old_pos_)
      {
        ppModeMachine_->process_event(ppModeSM::setTarget(evt.pos));
        ppModeMachine_->process_event(ppModeSM::enable());
      }
      else
        ppModeMachine_->process_event(ppModeSM::disable());
      old_pos_ = evt.pos;
      break;

    case enums402::Profiled_Velocity:
      pvModeMachine_->process_event(pvModeSM::selectMode());
      pvModeMachine_->process_event(pvModeSM::enable());
      pvModeMachine_->process_event(pvModeSM::setTarget(evt.vel));
      break;

    default:
      BOOST_THROW_EXCEPTION(std::invalid_argument("Mode not supported"));
    }
  }

  template <class Event,class FSM>
  void on_entry(Event const&,FSM& ) {/*std::cout << "entering: highLevelSm" << std::endl;*/}
  template <class Event,class FSM>
  void on_exit(Event const&,FSM& ) {/*std::cout << "leaving: highLevelSm" << std::endl;*/}

  // The list of FSM states
  struct machineStopped : public msm::front::state<>
  {
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& ) {/*std::cout << "starting: Off" << std::endl;*/}
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& ) {/*std::cout << "finishing: Off" << std::endl;*/}

  };
  struct machineRunning : public msm::front::state<>
  {
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& ) {/*std::cout << "starting: On" << std::endl;*/}
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& ) {/*std::cout << "finishing: On" << std::endl;*/}
  };

  struct isModeSupported : public msm::front::state<>
  {
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& ) {/*std::cout << "starting: On" << std::endl;*/}
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& ) {/*std::cout << "finishing: On" << std::endl;*/}
  };

  // the initial state. Must be defined
  typedef mpl::vector<machineStopped,isModeSupported>  initial_state;
  // transition actions
  void start_machine(startMachine const&)
  {
  }

  void update_motor(updateMotor const&)
  {
    //    std::cout << "highLevelSm::turn_on\n";
  }

  void update_switch(updateSwitch const&)
  {
    //    std::cout << "highLevelSm::turn_on\n";
  }


  void stop_machine(stopMachine const&)
  {
    modeSwitchMachine.process_event(ModeSwitchSM::deactivateMode(previous_mode_));
  }
  // guard conditions

  typedef highLevelSM_ hl; // makes transition table cleaner
  // Transition table for highLevelSm
  struct transition_table : mpl::vector<
      //      Start     Event         Next      Action               Guard
      //    +---------+-------------+---------+---------------------+----------------------+
      a_row < machineStopped   , startMachine    , machineRunning   , &hl::start_machine                       >,

      Row < machineRunning   , none    , Standby   , none, none                       >,

      a_row < Standby   , runMotorSM    , updateMotorSM   , &hl::run_motor_sm                       >,
      a_row < Standby   , checkModeSwitch    , ModeSwitch   , &hl::switch_mode                      >,
      a_row < Standby   , enableMove, Move   , &hl::move                     >,
      a_row < Standby   , stopMachine, machineStopped   , &hl::stop_machine                      >,

      a_row < ModeSwitch   , runMotorSM, updateMotorSM   , &hl::run_motor_sm                     >,
      a_row < ModeSwitch   , enterStandBy, Standby   , &hl::enter_standby                      >,
      a_row < ModeSwitch   , stopMachine, machineStopped   , &hl::stop_machine                      >,
      a_row < ModeSwitch   , checkModeSwitch, ModeSwitch   , &hl::switch_mode                      >,
      a_row < ModeSwitch   , enableMove, Move   , &hl::move                     >,

      a_row < updateMotorSM   , checkModeSwitch    , ModeSwitch   , &hl::switch_mode                       >,
      a_row < updateMotorSM   , enterStandBy, Standby   , &hl::enter_standby                      >,
      a_row < updateMotorSM   , enableMove, Move   , &hl::move                     >,
      a_row < updateMotorSM   , stopMachine, machineStopped   , &hl::stop_machine                      >,
      a_row < updateMotorSM   , runMotorSM, updateMotorSM   , &hl::run_motor_sm                      >,

      a_row < Move   , enterStandBy, Standby   , &hl::enter_standby                      >,
      a_row < Move   , stopMachine, machineStopped   , &hl::stop_machine                      >,
      a_row < Move   , enableMove, Move   , &hl::move                      >,
      //    +---------+-------------+---------+---------------------+----------------------+
      a_row < isModeSupported   , checkModeSupport    , isModeSupported   , &hl::check_support                       >
      //    +---------+-------------+---------+---------------------+----------------------+
      > {};
  // Replaces the default no-transition response.
  template <class FSM,class Event>
  void no_transition(Event const& e, FSM&,int state){}

  template <class FSM,class Event>
  void exception_caught (Event const&,FSM& fsm,std::exception& ){}

private:
  boost::shared_ptr<boost::mutex> state_change_mutex_;
  boost::shared_ptr<boost::mutex> mode_change_mutex_;

  boost::shared_ptr<double> target_pos_;
  boost::shared_ptr<double> target_vel_;
  double old_pos_;

  enums402::OperationMode previous_mode_;

  boost::shared_ptr<IPModeSM> ipModeMachine_;
  boost::shared_ptr<HomingSM> homingModeMachine_;
  boost::shared_ptr<velModeSM> velModeMachine_;
  boost::shared_ptr<ppModeSM> ppModeMachine_;
  boost::shared_ptr<pvModeSM> pvModeMachine_;
  boost::shared_ptr<StatusandControl> statusandControlMachine_;

  boost::shared_ptr<ObjectStorage> storage_;

  canopen::ObjectStorage::Entry<int8_t>  op_mode;

  canopen::ObjectStorage::Entry<uint32_t>  supported_drive_modes;

  canopen::ObjectStorage::Entry<int8_t>  homing_method;

  uint32_t supported_modes_;
};
// back-end
typedef msm::back::state_machine<highLevelSM_> highLevelSM;
};

#endif // HIGH_LEVEL_SM_H

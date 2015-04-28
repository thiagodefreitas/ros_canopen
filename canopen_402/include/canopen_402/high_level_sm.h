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
  highLevelSM_(boost::shared_ptr<StatusandControl::wordBitset> &words,  boost::shared_ptr<StatusandControl::commandTargets> target,boost::shared_ptr<StatusandControl::motorFeedback> feedback, boost::shared_ptr<ObjectStorage> &storage)
    : words_(words), targets_(target), motor_feedback_(feedback), previous_mode_(No_Mode), storage_(storage)
  {
    ///////////////////*
    ///
    ///
    ///
    storage_->entry(op_mode, 0x6060);
    storage_->entry(supported_drive_modes, 0x6502);

    storage_->entry(homing_method, 0x6098);

    supported_modes_ = supported_drive_modes.get_cached();

    std::cout << "Supported Modes" << supported_modes_ << std::endl;

    motorStateMachine = motorSM(words_, motor_feedback_);
    motorStateMachine.start();
    motorStateMachine.process_event(motorSM::boot());

    if(isSupported(Homing))
    {
      std::cout << "homing is supported" << std::endl;
      homingModeMachine_ = boost::make_shared<HomingSM>(HomingSM(words_));
      homingModeMachine_.get()->start();
    }

    if(isSupported(Velocity))
    {
      std::cout << "velocity is supported" << std::endl;
      velModeMachine_ = boost::make_shared<velModeSM>(velModeSM(words_, storage_));
      velModeMachine_.get()->start();
    }

    if(isSupported(Interpolated_Position))
    {
      std::cout << "ip is supported" << std::endl;
      ipModeMachine_ = boost::make_shared<IPModeSM>(IPModeSM(words_, storage_));
      ipModeMachine_.get()->start();
    }

    if(isSupported(Profiled_Position))
    {
      std::cout << "pp is supported" << std::endl;
      ppModeMachine_ = boost::make_shared<ppModeSM>(ppModeSM(words_, storage_));
      ppModeMachine_.get()->start();
    }

    if(isSupported(Profiled_Velocity))
    {
      std::cout << "pv is supported" << std::endl;
      pvModeMachine_ = boost::make_shared<pvModeSM>(pvModeSM(words_, storage_));
      pvModeMachine_.get()->start();
    }

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
    OperationMode op_mode;
    int timeout;

    checkModeSwitch() : op_mode(), timeout(0) {}
    checkModeSwitch( OperationMode mode) : op_mode(mode), timeout(0) {}
    checkModeSwitch( OperationMode mode, int timeOut) : op_mode(mode), timeout(timeOut) {}
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
    OperationMode op_mode;

    checkModeSupport( OperationMode mode) : op_mode(mode) {}
  };

  struct runMotorSM
  {
    InternalActions action;
    int timeout;

    runMotorSM() : action(), timeout(0) {}
    runMotorSM(InternalActions actionToTake) : action(actionToTake), timeout(0) {}
    runMotorSM(InternalActions actionToTake, int timeOut) : action(actionToTake), timeout(timeOut) {}
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

  // the initial state. Must be defined
  // typedef StartUp initial_state;
  // transition actions
  void standby(enterStandBy const&)
  {
    (*targets_).target_pos = (*motor_feedback_).actual_pos;
    (*targets_).target_vel = 0;
  }


  template <class runMotorSM> void motor_sm(runMotorSM const& evt)
  {
    switch(evt.action)
    {
    case QuickStop:
      modeSwitchMachine.process_event(ModeSwitchSM::deactivateMode(previous_mode_));
      motorStateMachine.process_event(motorSM::quick_stop());
      if((*motor_feedback_).state != Quick_Stop_Active)
        BOOST_THROW_EXCEPTION(std::invalid_argument("The transition was not successful"));
      break;

    case FaultReset:
      motorStateMachine.process_event(motorSM::fault_reset());
      if((*motor_feedback_).state == Fault)
        BOOST_THROW_EXCEPTION(std::invalid_argument("The transition was not successful"));
      break;

    case ShutdownMotor:
      motorStateMachine.process_event(motorSM::shutdown());
      if((*motor_feedback_).state != Ready_To_Switch_On)
        BOOST_THROW_EXCEPTION(std::invalid_argument("The transition was not successful"));
      break;

    case SwitchOn:
      motorStateMachine.process_event(motorSM::switch_on());
      if((*motor_feedback_).state != Switched_On)
        BOOST_THROW_EXCEPTION(std::invalid_argument("The transition was not successful"));
      break;

    case EnableOp:
      motorStateMachine.process_event(motorSM::enable_op());
      if((*motor_feedback_).state != Operation_Enable)
        BOOST_THROW_EXCEPTION(std::invalid_argument("The transition was not successful"));
      break;

    case FaultEnable:
      modeSwitchMachine.process_event(ModeSwitchSM::deactivateMode(previous_mode_));
      motorStateMachine.process_event(motorSM::fault());
      if((*motor_feedback_).state != Fault)
        BOOST_THROW_EXCEPTION(std::invalid_argument("The transition was not successful"));
      break;

    case DisableQuickStop:
      motorStateMachine.process_event(motorSM::disable_voltage());
      if((*motor_feedback_).state != Not_Ready_To_Switch_On && (*motor_feedback_).state != Switch_On_Disabled)
        BOOST_THROW_EXCEPTION(std::invalid_argument("The transition was not successful"));
      break;

    default:
      BOOST_THROW_EXCEPTION(std::invalid_argument("Action not specified"));
    }
  }

  template <class checkModeSwitch> void mode_switch(checkModeSwitch const& evt)
  {
    if((*motor_feedback_).current_mode != evt.op_mode && evt.op_mode != No_Mode)
    {
      op_mode.set_cached(evt.op_mode);
      modeSwitchMachine.process_event(ModeSwitchSM::deactivateMode(previous_mode_));
      BOOST_THROW_EXCEPTION(std::invalid_argument("This operation mode can not be used"));
    }

    switch(evt.op_mode)
    {
    bool transition_sucess;
    case No_Mode:
      modeSwitchMachine.process_event(ModeSwitchSM::deactivateMode(previous_mode_));
      break;

    case Interpolated_Position:
      modeSwitchMachine.process_event(ModeSwitchSM::deactivateMode(previous_mode_));
      modeSwitchMachine.process_event(ModeSwitchSM::selectIP());
      break;

    case Velocity:
      modeSwitchMachine.process_event(ModeSwitchSM::deactivateMode(previous_mode_));
      modeSwitchMachine.process_event(ModeSwitchSM::selectVel());
      break;

    case Profiled_Velocity:
      modeSwitchMachine.process_event(ModeSwitchSM::deactivateMode(previous_mode_));
      modeSwitchMachine.process_event(ModeSwitchSM::selectPV());
      break;

    case Profiled_Position:
      modeSwitchMachine.process_event(ModeSwitchSM::deactivateMode(previous_mode_));
      modeSwitchMachine.process_event(ModeSwitchSM::selectPP());
      break;

    case Homing:
      if (!homing_method.valid() && homing_method.get() == 0)
      {
        BOOST_THROW_EXCEPTION(std::invalid_argument("Homing invalid"));
      }
      modeSwitchMachine.process_event(ModeSwitchSM::deactivateMode(previous_mode_));
      modeSwitchMachine.process_event(ModeSwitchSM::selectHoming());
      transition_sucess = homingModeMachine_.get()->process_event(HomingSM::runHomingCheck());
      if(!transition_sucess)
      {
        BOOST_THROW_EXCEPTION(std::invalid_argument("Homing still not completed"));
      }
      break;

    default:
      BOOST_THROW_EXCEPTION(std::invalid_argument("Mode not supported"));
      break;
    }

    previous_mode_ = (*motor_feedback_).current_mode;
  }

  bool isSupported(const OperationMode &op_mode)
  {
    uint32_t masked_mode;
    bool supported;

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
      masked_mode = (1<<(op_mode-1));
      break;
    case No_Mode:
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
    switch((*motor_feedback_).current_mode)
    {
    case Interpolated_Position:
      ipModeMachine_.get()->process_event(IPModeSM::enableIP());
      ipModeMachine_.get()->process_event(IPModeSM::setTarget(evt.pos, evt.vel));
      break;

    case Velocity:
      velModeMachine_.get()->process_event(velModeSM::enableVel());
      velModeMachine_.get()->process_event(velModeSM::setTarget(evt.vel));
      break;

    case Homing:
      homingModeMachine_.get()->process_event(HomingSM::enableHoming());
      break;

    case Profiled_Position:
      ppModeMachine_.get()->process_event(ppModeSM::enablePP());
      ppModeMachine_.get()->process_event(ppModeSM::setTarget(evt.pos));
      break;

    case Profiled_Velocity:
      pvModeMachine_.get()->process_event(pvModeSM::enablePV());
      pvModeMachine_.get()->process_event(pvModeSM::setTarget(evt.vel));
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
    //    std::cout << "highLevelSm::turn_on\n";
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

      a_row < Standby   , runMotorSM    , updateMotorSM   , &hl::motor_sm                       >,
      a_row < Standby   , checkModeSwitch    , ModeSwitch   , &hl::mode_switch                      >,
      a_row < Standby   , enableMove, Move   , &hl::move                     >,
      a_row < Standby   , stopMachine, machineStopped   , &hl::stop_machine                      >,

      a_row < ModeSwitch   , runMotorSM, updateMotorSM   , &hl::motor_sm                     >,
      a_row < ModeSwitch   , enterStandBy, Standby   , &hl::standby                      >,
      a_row < ModeSwitch   , stopMachine, machineStopped   , &hl::stop_machine                      >,
      a_row < ModeSwitch   , checkModeSwitch, ModeSwitch   , &hl::mode_switch                      >,

      a_row < updateMotorSM   , checkModeSwitch    , ModeSwitch   , &hl::mode_switch                       >,
      a_row < updateMotorSM   , enterStandBy, Standby   , &hl::standby                      >,
      a_row < updateMotorSM   , enableMove, Move   , &hl::move                     >,
      a_row < updateMotorSM   , stopMachine, machineStopped   , &hl::stop_machine                      >,
      a_row < updateMotorSM   , runMotorSM, updateMotorSM   , &hl::motor_sm                      >,

      a_row < Move   , enterStandBy, Standby   , &hl::standby                      >,
      a_row < Move   , stopMachine, machineStopped   , &hl::stop_machine                      >,
      a_row < Move   , enableMove, Move   , &hl::move                      >,
      //    +---------+-------------+---------+---------------------+----------------------+
      a_row < isModeSupported   , checkModeSupport    , isModeSupported   , &hl::check_support                       >
      //    +---------+-------------+---------+---------------------+----------------------+
      > {};
  // Replaces the default no-transition response.
  template <class FSM,class Event>
  void no_transition(Event const& e, FSM&,int state)
  {
    //std::cout << "no transition from state " << state
    //          << " on event " << typeid(e).name() << std::endl;
  }

  template <class FSM,class Event>
  void exception_caught (Event const&,FSM& fsm,std::exception& )
  {

  }

private:
  boost::shared_ptr<StatusandControl::wordBitset> words_;

  boost::shared_ptr<double> target_pos_;
  boost::shared_ptr<double> target_vel_;

  boost::shared_ptr<StatusandControl::commandTargets> targets_;
  boost::shared_ptr<StatusandControl::motorFeedback> motor_feedback_;
  OperationMode previous_mode_;

  boost::shared_ptr<IPModeSM> ipModeMachine_;
  boost::shared_ptr<HomingSM> homingModeMachine_;
  boost::shared_ptr<velModeSM> velModeMachine_;
  boost::shared_ptr<ppModeSM> ppModeMachine_;
  boost::shared_ptr<pvModeSM> pvModeMachine_;

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

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

#ifndef MODE_SWITCH_H
#define MODE_SWITCH_H
#include <canopen_402/status_and_control.h>
#include <canopen_402/ip_mode.h>
#include <canopen_402/vel_mode.h>
#include <canopen_402/homing_mode.h>
#include <canopen_402/pv_mode.h>
#include <canopen_402/pp_mode.h>
///////
/// \brief m
///
///
///
// the ip mode state machine
namespace msm = boost::msm;
namespace mpl = boost::mpl;


using namespace boost::msm::front;


namespace canopen
{
class ModeSwitchSM_ : public msm::front::state_machine_def<ModeSwitchSM_>
{
public:
  ModeSwitchSM_(){}
  ModeSwitchSM_(const boost::shared_ptr<IPModeSM> &ip_machine, const boost::shared_ptr<velModeSM> &vel_machine,
                const boost::shared_ptr<HomingSM> &homing_machine , const boost::shared_ptr<pvModeSM> &pv_machine, const boost::shared_ptr<ppModeSM> &pp_machine)
    : ip_machine_(ip_machine), vel_machine_(vel_machine), homing_machine_(homing_machine), pv_machine_(pv_machine), pp_machine_(pp_machine){}

  struct selectIP {};
  struct selectPP {};
  struct selectPV {};
  struct selectVel {};
  struct selectHoming {};

  struct deactivateMode
  {
    OperationMode op_mode;

    deactivateMode() : op_mode(No_Mode) {}
    deactivateMode( OperationMode mode) : op_mode(mode){}
  };

  template <class Event,class FSM>
  void on_entry(Event const&,FSM& ) {/*std::cout << "entering: MODESWITCH" << std::endl;*/}
  template <class Event,class FSM>
  void on_exit(Event const&,FSM& ) {/*std::cout << "leaving: MODESWITCH" << std::endl;*/}

  // The list of FSM states
  struct PVMode : public msm::front::state<>
  {
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& ) {/*std::cout << "starting: IPInactive" << std::endl;*/}
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& ) {/*std::cout << "finishing: IPInactive" << std::endl;*/}

  };
  struct IPMode : public msm::front::state<>
  {
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& ) {/*std::cout << "starting: IPActive" << std::endl;*/}
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& ) {/*std::cout << "finishing: IPActive" << std::endl;*/}
  };

  // The list of FSM states
  struct noActiveMode : public msm::front::state<>
  {
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& ) {/*std::cout << "starting: modeDeselected" << std::endl;*/}
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& ) {/*std::cout << "finishing: modeDeselected" << std::endl;*/}

  };
  struct PPMode : public msm::front::state<>
  {
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& ) {/*std::cout << "starting: modeSelected" << std::endl;*/}
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& ) {/*std::cout << "finishing: modeSelected" << std::endl;*/}
  };

  struct velMode : public msm::front::state<>
  {
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& ) {/*std::cout << "starting: modeSelected" << std::endl;*/}
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& ) {/*std::cout << "finishing: modeSelected" << std::endl;*/}
  };

  struct homingMode : public msm::front::state<>
  {
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& ) {/*std::cout << "starting: modeSelected" << std::endl;*/}
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& ) {/*std::cout << "finishing: modeSelected" << std::endl;*/}
  };

  // the initial state. Must be defined
  typedef noActiveMode initial_state;
  // transition actionss
  void select_pp(selectPP const&)
  {
    std::cout << "ModeSwitch::selectPP\n";
    pp_machine_->process_event(ppModeSM::selectMode());
  }

  void select_pv(selectPV const&)
  {
    std::cout << "ModeSwitch::selectPV\n";
    pv_machine_->process_event(pvModeSM::selectMode());
  }

  void select_vel(selectVel const&)
  {
    std::cout << "ModeSwitch::selectvel\n";
    vel_machine_->process_event(velModeSM::selectMode());
  }

  void select_ip(selectIP const&)
  {
    std::cout << "ModeSwitch::selectip\n";
    ip_machine_->process_event(IPModeSM::selectMode());
  }


  void select_homing(selectHoming const&)
  {
    homing_machine_->process_event(HomingSM::selectMode());
  }

  void deselect_mode(deactivateMode const& evt)
  {
    switch(evt.op_mode)
    {
    case Interpolated_Position:
      ip_machine_->process_event(IPModeSM::disable());
      ip_machine_->process_event(IPModeSM::deselectMode());
      break;

    case Velocity:
      vel_machine_->process_event(velModeSM::disable());
      vel_machine_->process_event(velModeSM::deselectMode());
      break;

    case Homing:
      homing_machine_->process_event(HomingSM::disable());
      homing_machine_->process_event(HomingSM::deselectMode());
      break;

    case Profiled_Velocity:
      pv_machine_->process_event(pvModeSM::disable());
      pv_machine_->process_event(pvModeSM::deselectMode());
      break;

    case Profiled_Position:
      pp_machine_->process_event(ppModeSM::disable());
      pp_machine_->process_event(ppModeSM::deselectMode());
      break;
    }
  }
  // guard conditions

  typedef ModeSwitchSM_ ms; // makes transition table cleaner
  // Transition table for Mode Switching
  struct transition_table : mpl::vector<
      //      Start     Event         Next      Action               Guard
      //    +---------+-------------+---------+---------------------+----------------------+
      a_row < noActiveMode   , selectPP    , PPMode   , &ms::select_pp                       >,
      a_row < noActiveMode   , selectIP    , IPMode   , &ms::select_ip                       >,
      a_row < noActiveMode   , selectPV    , PVMode   , &ms::select_pv                       >,
      a_row < noActiveMode   , selectVel    , velMode   , &ms::select_vel                       >,
      a_row < noActiveMode   , selectHoming    , homingMode   , &ms::select_homing                       >,

      a_row < PPMode   , deactivateMode    , noActiveMode   , &ms::deselect_mode                       >,
      a_row < IPMode   , deactivateMode    , noActiveMode   , &ms::deselect_mode                       >,
      a_row < PVMode   , deactivateMode    , noActiveMode   , &ms::deselect_mode                       >,
      a_row < noActiveMode   , deactivateMode    , noActiveMode   , &ms::deselect_mode                       >,
      a_row < velMode   , deactivateMode    , noActiveMode   , &ms::deselect_mode                       >,
      a_row < homingMode   , deactivateMode    , noActiveMode   , &ms::deselect_mode                       >
      //    +---------+-------------+---------+---------------------+----------------------+
      > {};
  // Replaces the default no-transition response.
  template <class FSM,class Event>
  void no_transition(Event const& e, FSM&,int state)
  {
    //    std::cout << "no transition from state " << state
    //              << " on event " << typeid(e).name() << std::endl;
  }
private:
  boost::shared_ptr<IPModeSM> ip_machine_;
  boost::shared_ptr<velModeSM> vel_machine_;
  boost::shared_ptr<HomingSM> homing_machine_;
  boost::shared_ptr<pvModeSM> pv_machine_;
  boost::shared_ptr<ppModeSM> pp_machine_;

};
// back-end
typedef msm::back::state_machine<ModeSwitchSM_> ModeSwitchSM;
};
/// */
///
///
#endif // MODE_SWITCH_H

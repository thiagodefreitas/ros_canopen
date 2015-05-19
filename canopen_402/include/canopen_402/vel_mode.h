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

#ifndef VEL_MODE_H
#define VEL_MODE_H

#include <canopen_402/status_and_control.h>
///////
/// \brief m
///
///
///
// the vel mode state machine
namespace msm = boost::msm;
namespace mpl = boost::mpl;


using namespace boost::msm::front;


namespace canopen
{
class velModeSM_ : public msm::front::state_machine_def<velModeSM_>
{
public:
  velModeSM_(){}
  velModeSM_(const boost::shared_ptr<StatusandControl> &statusandControlMachine, const boost::shared_ptr<ObjectStorage> &storage) : statusandControlMachine_(statusandControlMachine), storage_(storage)
  {
    storage_->entry(target_velocity, 0x6042);
  }
  struct enable {};
  struct disable {};
  struct selectMode {};
  struct deselectMode {};

  struct setTarget
  {
    double target_vel;

    setTarget() : target_vel(0) {}
    setTarget(double vel) : target_vel(vel) {}
  };

  template <class Event,class FSM>
  void on_entry(Event const&,FSM& ) {/*std::cout << "entering: PPMode" << std::endl;*/}
  template <class Event,class FSM>
  void on_exit(Event const&,FSM& ) {/*std::cout << "leaving: PPMode" << std::endl;*/}

  // The list of FSM states
  struct Inactive : public msm::front::state<>
  {
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& ) {/*std::cout << "starting: PPInactive" << std::endl;*/}
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& ) {/*std::cout << "finishing: PPInactive" << std::endl;*/}

  };
  struct updateTarget : public msm::front::state<>
  {
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& ) {/*std::cout << "starting: IPInactive" << std::endl;*/}
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& ) {/*std::cout << "finishing: IPInactive" << std::endl;*/}

  };
  struct Active : public msm::front::state<>
  {
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& ) {/*std::cout << "starting: PPActive" << std::endl;*/}
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& ) {/*std::cout << "finishing: PPActive" << std::endl;*/}
  };

  // The list of FSM states
  struct modeDeselected : public msm::front::state<>
  {
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& ) {/*std::cout << "starting: modeDeselected" << std::endl;*/}
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& ) {/*std::cout << "finishing: modeDeselected" << std::endl;*/}

  };
  struct modeSelected : public msm::front::state<>
  {
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& ) {/*std::cout << "starting: modeSelected" << std::endl;*/}
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& ) {/*std::cout << "finishing: modeSelected" << std::endl;*/}
  };

  // the initial state. Must be defined
  typedef mpl::vector<modeDeselected,updateTarget> initial_state;
  // transition actions
  void enable_mode(enable const&)
  {
    statusandControlMachine_->getWords()->control_word.reset(enums402::CW_Halt);

    statusandControlMachine_->getWords()->control_word.set(enums402::CW_Operation_mode_specific0);
    statusandControlMachine_->getWords()->control_word.set(enums402::CW_Operation_mode_specific1);
    statusandControlMachine_->getWords()->control_word.set(enums402::CW_Operation_mode_specific2);
  }
  void disable_mode(disable const&)
  {
    statusandControlMachine_->getWords()->control_word.set(enums402::CW_Halt);

    statusandControlMachine_->getWords()->control_word.reset(enums402::CW_Operation_mode_specific0);
    statusandControlMachine_->getWords()->control_word.reset(enums402::CW_Operation_mode_specific1);
    statusandControlMachine_->getWords()->control_word.reset(enums402::CW_Operation_mode_specific2);
  }

  void select_mode(selectMode const&)
  {
    statusandControlMachine_->getWords()->control_word.set(enums402::CW_Halt);
  }
  void deselect_mode(deselectMode const&)
  {
    statusandControlMachine_->getWords()->control_word.set(enums402::CW_Halt);
    target_velocity.set(0.0);

    statusandControlMachine_->getWords()->control_word.reset(enums402::CW_Operation_mode_specific0);
    statusandControlMachine_->getWords()->control_word.reset(enums402::CW_Operation_mode_specific1);
    statusandControlMachine_->getWords()->control_word.reset(enums402::CW_Operation_mode_specific2);
  }

  template <class setTarget> void set_target(setTarget const& evt)
  {
    target_velocity.set(evt.target_vel);
  }

  typedef velModeSM_ vel; // makes transition table cleaner
  // Transition table for PPMode
  struct transition_table : mpl::vector<
      //      Start     Event         Next      Action               Guard
      //    +---------+-------------+---------+---------------------+----------------------+
      a_row < modeDeselected   , selectMode    , modeSelected   , &vel::select_mode                       >,

      Row < modeSelected   , none    , Inactive   , none, none                       >,
      a_row < modeSelected   , deselectMode    , modeDeselected   , &vel::deselect_mode                       >,

      a_row < Active   , disable, Inactive   , &vel::disable_mode                      >,
      a_row < Active   , deselectMode    , modeDeselected   , &vel::deselect_mode                       >,

      a_row < Inactive   , enable    , Active   , &vel::enable_mode                       >,
      a_row < Inactive   , deselectMode    , modeDeselected   , &vel::deselect_mode                       >,
      //    +---------+-------------+---------+---------------------+----------------------+
      //    +---------+-------------+---------+---------------------+----------------------+
      a_row < updateTarget   , setTarget    , updateTarget   , &vel::set_target                       >
      //    +---------+-------------+---------+---------------------+----------------------+
      > {};
  // Replaces the default no-transition response.
  template <class FSM,class Event>
  void no_transition(Event const& e, FSM&,int state)
  {
  }
private:
  boost::shared_ptr<StatusandControl> statusandControlMachine_;
  boost::shared_ptr<ObjectStorage> storage_;

  canopen::ObjectStorage::Entry<int16_t> target_velocity;

};
// back-end
typedef msm::back::state_machine<velModeSM_> velModeSM;
};
/// */
///
///
///
#endif // VEL_MODE_H

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
// the ip mode state machine
namespace msm = boost::msm;
namespace mpl = boost::mpl;


using namespace boost::msm::front;


namespace canopen
{
class velModeSM_ : public msm::front::state_machine_def<velModeSM_>
{
public:
  velModeSM_(){}
  velModeSM_(const boost::shared_ptr<cw_word> &control_word) : control_word_(control_word){}
  struct enableVel {};
  struct disableVel {};
  struct selectMode {};
  struct deselectMode {};

  template <class Event,class FSM>
  void on_entry(Event const&,FSM& ) {/*std::cout << "entering: PPMode" << std::endl;*/}
  template <class Event,class FSM>
  void on_exit(Event const&,FSM& ) {/*std::cout << "leaving: PPMode" << std::endl;*/}

  // The list of FSM states
  struct velInactive : public msm::front::state<>
  {
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& ) {/*std::cout << "starting: PPInactive" << std::endl;*/}
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& ) {/*std::cout << "finishing: PPInactive" << std::endl;*/}

  };
  struct velActive : public msm::front::state<>
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
  typedef modeDeselected initial_state;
  // transition actions
  void enable_vel(enableVel const&)
  {
    control_word_->set(CW_Operation_mode_specific0);
    control_word_->set(CW_Operation_mode_specific1);
    control_word_->set(CW_Operation_mode_specific2);
    std::cout << "VelMode::enable_vel\n";
  }
  void disable_vel(disableVel const&)
  {
    control_word_->reset(CW_Operation_mode_specific0);
    control_word_->reset(CW_Operation_mode_specific1);
    control_word_->reset(CW_Operation_mode_specific2);
  }

  void select_mode(selectMode const&)
  {
    //    std::cout << "PPMode::selectMode\n";
  }
  void deselect_mode(deselectMode const&)
  {
    control_word_->reset(CW_Operation_mode_specific0);
    control_word_->reset(CW_Operation_mode_specific1);
    control_word_->reset(CW_Operation_mode_specific2);
  }
  // guard conditions

  typedef velModeSM_ vel; // makes transition table cleaner
  // Transition table for PPMode
  struct transition_table : mpl::vector<
      //      Start     Event         Next      Action               Guard
      //    +---------+-------------+---------+---------------------+----------------------+
      a_row < modeDeselected   , selectMode    , modeSelected   , &vel::select_mode                       >,

      Row < modeSelected   , none    , velInactive   , none, none                       >,
      a_row < modeSelected   , deselectMode    , modeDeselected   , &vel::deselect_mode                       >,

      a_row < velActive   , disableVel, velInactive   , &vel::disable_vel                      >,
      a_row < velActive   , deselectMode    , modeDeselected   , &vel::deselect_mode                       >,

      a_row < velInactive   , enableVel    , velActive   , &vel::enable_vel                       >,
      a_row < velInactive   , deselectMode    , modeDeselected   , &vel::deselect_mode                       >
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
  boost::shared_ptr<cw_word> control_word_;

};
// back-end
typedef msm::back::state_machine<velModeSM_> velModeSM;
};
/// */
///
///
///
#endif // VEL_MODE_H

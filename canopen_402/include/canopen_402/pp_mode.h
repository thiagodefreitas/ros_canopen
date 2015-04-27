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

#ifndef pp_MODE_H
#define pp_MODE_H

#include <canopen_402/status_and_control.h>
///////
/// \brief m
///
///
///
// the pp mode state machine
namespace msm = boost::msm;
namespace mpl = boost::mpl;


using namespace boost::msm::front;


namespace canopen
{
class ppModeSM_ : public msm::front::state_machine_def<ppModeSM_>
{
public:
  ppModeSM_(){}
  ppModeSM_(const boost::shared_ptr<StatusandControl::wordBitset> &words) : words_(words){}
  struct enablePP {};
  struct disablePP {};
  struct selectMode {};
  struct deselectMode {};

  template <class Event,class FSM>
  void on_entry(Event const&,FSM& ) {/*std::cout << "entering: ppMode" << std::endl;*/}
  template <class Event,class FSM>
  void on_exit(Event const&,FSM& ) {/*std::cout << "leaving: ppMode" << std::endl;*/}

  // The list of FSM states
  struct ppInactive : public msm::front::state<>
  {
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& ) {/*std::cout << "starting: ppInactive" << std::endl;*/}
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& ) {/*std::cout << "finishing: ppInactive" << std::endl;*/}

  };
  struct ppActive : public msm::front::state<>
  {
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& ) {/*std::cout << "starting: ppInactive" << std::endl;*/}
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& ) {/*std::cout << "finishing: ppInactive" << std::endl;*/}
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
  void enable_pp(enablePP const&)
  {
    (*words_).control_word.set(CW_Operation_mode_specific0);
    (*words_).control_word.set(CW_Operation_mode_specific1);
    (*words_).control_word.set(CW_Operation_mode_specific2);
    std::cout << "ppMode::enable_pp\n";
  }
  void disable_pp(disablePP const&)
  {
    (*words_).control_word.reset(CW_Operation_mode_specific0);
    (*words_).control_word.reset(CW_Operation_mode_specific1);
    (*words_).control_word.reset(CW_Operation_mode_specific2);
  }

  void select_mode(selectMode const&)
  {
    //    std::cout << "ppMode::selectMode\n";
  }
  void deselect_mode(deselectMode const&)
  {
    (*words_).control_word.reset(CW_Operation_mode_specific0);
    (*words_).control_word.reset(CW_Operation_mode_specific1);
    (*words_).control_word.reset(CW_Operation_mode_specific2);
  }
  // guard conditions

  typedef ppModeSM_ pp; // makes transition table cleaner
  // Transition table for ppMode
  struct transition_table : mpl::vector<
      //      Start     Event         Next      Action               Guard
      //    +---------+-------------+---------+---------------------+----------------------+
      a_row < modeDeselected   , selectMode    , modeSelected   , &pp::select_mode                       >,

      Row < modeSelected   , none    , ppInactive   , none, none                       >,
      a_row < modeSelected   , deselectMode    , modeDeselected   , &pp::deselect_mode                       >,

      a_row < ppActive   , disablePP, ppInactive   , &pp::disable_pp                      >,
      a_row < ppActive   , deselectMode    , modeDeselected   , &pp::deselect_mode                       >,

      a_row < ppInactive   , enablePP    , ppActive   , &pp::enable_pp                       >,
      a_row < ppInactive   , deselectMode    , modeDeselected   , &pp::deselect_mode                       >
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
  boost::shared_ptr<StatusandControl::wordBitset> words_;

};
// back-end
typedef msm::back::state_machine<ppModeSM_> ppModeSM;
};
/// */
///
///
///
#endif // pp_MODE_H

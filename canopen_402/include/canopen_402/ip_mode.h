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

#ifndef IP_MODE_H
#define IP_MODE_H
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
class IPModeSM_ : public msm::front::state_machine_def<IPModeSM_>
{
public:
  IPModeSM_(){}
  IPModeSM_(const boost::shared_ptr<StatusandControl::wordBitset> &words, const boost::shared_ptr<ObjectStorage> &storage) : words_(words), storage_(storage)
  {
    storage_->entry(ip_mode_sub_mode, 0x60C0);
    storage_->entry(target_interpolated_position, 0x60C1, 0x01);

    if (ip_mode_sub_mode.get_cached() == -1)
      storage_->entry(target_interpolated_velocity, 0x60C1, 0x02);
  }
  struct enableIP {};
  struct disableIP {};
  struct selectMode {};
  struct deselectMode {};
  struct setTarget
  {
    double target_pos;
    double target_vel;

    setTarget() : target_pos(0), target_vel(0) {}
    setTarget(double pos) : target_pos(pos), target_vel(0) {}
    setTarget(double pos, double vel) : target_pos(pos), target_vel(vel) {}
  };

  template <class Event,class FSM>
  void on_entry(Event const&,FSM& ) {/*std::cout << "entering: IPMode" << std::endl;*/}
  template <class Event,class FSM>
  void on_exit(Event const&,FSM& ) {/*std::cout << "leaving: IPMode" << std::endl;*/}

  // The list of FSM states
  struct IPInactive : public msm::front::state<>
  {
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& ) {/*std::cout << "starting: IPInactive" << std::endl;*/}
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& ) {/*std::cout << "finishing: IPInactive" << std::endl;*/}

  };

  struct updateTarget : public msm::front::state<>
  {
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& ) {/*std::cout << "starting: IPInactive" << std::endl;*/}
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& ) {/*std::cout << "finishing: IPInactive" << std::endl;*/}

  };
  struct IPActive : public msm::front::state<>
  {
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& ) {/*std::cout << "starting: IPActive" << std::endl;*/}
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& ) {/*std::cout << "finishing: IPActive" << std::endl;*/}
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
  void enable_ip(enableIP const&)
  {
    (*words_).control_word.set(CW_Operation_mode_specific0);
    (*words_).control_word.reset(CW_Operation_mode_specific1);
    (*words_).control_word.reset(CW_Operation_mode_specific2);
    std::cout << "IPMode::enable_ip\n";
  }

  template <class setTarget> void set_target(setTarget const& evt)
  {
    std::cout << "Setting IP target" << evt.target_pos << "," << evt.target_vel << std::endl;

    target_interpolated_position.set(evt.target_pos);
    if (ip_mode_sub_mode.get_cached() == -1)
      target_interpolated_velocity.set(evt.target_vel);
  }
  void disable_ip(disableIP const&)
  {
    (*words_).control_word.reset(CW_Operation_mode_specific0);
    (*words_).control_word.reset(CW_Operation_mode_specific1);
    (*words_).control_word.reset(CW_Operation_mode_specific2);
    std::cout << "IPMode::disable_ip\n";
  }

  void select_mode(selectMode const&)
  {
    (*words_).control_word.reset(CW_Operation_mode_specific0);
    (*words_).control_word.reset(CW_Operation_mode_specific1);
    (*words_).control_word.reset(CW_Operation_mode_specific2);
    std::cout << "IPMode::selectMode\n";
  }
  void deselect_mode(deselectMode const&)
  {
    std::cout << "IPMode::deselectMode\n";
    (*words_).control_word.reset(CW_Operation_mode_specific0);
    (*words_).control_word.reset(CW_Operation_mode_specific1);
    (*words_).control_word.reset(CW_Operation_mode_specific2);
  }
  // guard conditions

  typedef IPModeSM_ ip; // makes transition table cleaner
  // Transition table for IPMode
  struct transition_table : mpl::vector<
      //      Start     Event         Next      Action               Guard
      //    +---------+-------------+---------+---------------------+----------------------+
      a_row < modeDeselected   , selectMode    , modeSelected   , &ip::select_mode                       >,

      Row < modeSelected   , none    , IPInactive   , none, none                       >,
      a_row < modeSelected   , deselectMode    , modeDeselected   , &ip::deselect_mode                       >,

      a_row < IPActive   , disableIP, IPInactive   , &ip::disable_ip                      >,
      a_row < IPActive   , enableIP    , IPActive   , &ip::enable_ip                       >,
      a_row < IPActive   , deselectMode    , modeDeselected   , &ip::deselect_mode                       >,

      a_row < IPInactive   , enableIP    , IPActive   , &ip::enable_ip                       >,
      a_row < IPInactive   , deselectMode    , modeDeselected   , &ip::deselect_mode                       >,
      //    +---------+-------------+---------+---------------------+----------------------+
      //    +---------+-------------+---------+---------------------+----------------------+
      a_row < updateTarget   , setTarget    , updateTarget   , &ip::set_target                       >
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
  boost::shared_ptr<ObjectStorage> storage_;

  canopen::ObjectStorage::Entry<int16_t>  ip_mode_sub_mode;

  canopen::ObjectStorage::Entry<int32_t> target_interpolated_position;
  canopen::ObjectStorage::Entry<int32_t> target_interpolated_velocity;

};
// back-end
typedef msm::back::state_machine<IPModeSM_> IPModeSM;
};
/// */
///
///
#endif // IP_MODE_H

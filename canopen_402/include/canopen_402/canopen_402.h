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

#ifndef CANOPEN_402_CANOPEN_402_H
#define CANOPEN_402_CANOPEN_402_H

#include <canopen_402/status_and_control.h>
#include <canopen_402/high_level_sm.h>
#include <string>
#include <vector>

namespace canopen
{
class Node_402 : public canopen::Layer
{
public:
  Node_402(boost::shared_ptr <canopen::Node> n, const std::string &name) : Layer(name), n_(n), storage_(n_->getStorage()), valid_mode_state_(true)
  {
    words_ = boost::make_shared<StatusandControl::wordBitset>();

    target_values_ = boost::make_shared<StatusandControl::commandTargets>();
    motor_feedback_ = boost::make_shared<StatusandControl::motorFeedback>();

    SwCwSM = boost::make_shared<StatusandControl>(words_, motor_feedback_, storage_);
    motorAbstraction = highLevelSM(words_, target_values_, motor_feedback_, storage_, SwCwSM);
    SwCwSM->start();
    motorAbstraction.start();
    SwCwSM->process_event(StatusandControl::readStatus());
  }

  const  enums402::OperationMode getMode();

  bool enterModeAndWait(const enums402::OperationMode &op_mode);
  bool isModeSupported(const enums402::OperationMode &op_mode);
  static uint32_t getModeMask(const enums402::OperationMode &op_mode);


  const double getActualPos();

  const double getActualVel();
  const double getActualEff();

  void setTargetPos(const double &target_pos);
  void setTargetVel(const double &target_vel);
  void setTargetEff(const double &v) {}

  const double getTargetPos();
  const double getTargetVel();
  const double getTargetEff()
  {
    return 0;
  }

private:

  template<typename T> int wait_for(const bool &condition, const T &timeout);

  boost::shared_ptr <canopen::Node> n_;
  boost::shared_ptr <ObjectStorage> storage_;

  volatile bool running;

  boost::mutex motor_mutex_;
  boost::mutex word_mutex_;
  boost::mutex cond_mutex_;
  boost::mutex mode_mutex_;

  boost::condition_variable cond_mode_;
  boost::condition_variable cond_event_;

  bool valid_mode_state_;

  boost::shared_ptr<double> target_vel_;
  boost::shared_ptr<double> target_pos_;

  enums402::OperationMode default_operation_mode_;

  boost::shared_ptr<StatusandControl::wordBitset> words_;

  boost::shared_ptr<StatusandControl> SwCwSM;
  highLevelSM motorAbstraction;

  boost::shared_ptr<StatusandControl::commandTargets> target_values_;
  boost::shared_ptr<StatusandControl::motorFeedback> motor_feedback_;

  template <class Event>
  bool motorEvent(Event const&);

  virtual void move(LayerStatus &status);

  virtual void processSW(LayerStatus &status);


  virtual void processCW();

  virtual void pending(LayerStatus &status);

  bool turnOn(LayerStatus &status);
  bool turnOff(LayerStatus &status);

  bool transition_success_;

protected:

  virtual void handleRead(LayerStatus &status, const LayerState &current_state);
  virtual void handleWrite(LayerStatus &status, const LayerState &current_state);
  virtual void handleDiag(LayerReport &report);
  virtual void handleInit(LayerStatus &status);
  virtual void handleShutdown(LayerStatus &status);
  virtual void handleHalt(LayerStatus &status);
  virtual void handleRecover(LayerStatus &status);

};
}  //  namespace canopen
#endif  // CANOPEN_402_CANOPEN_402_H

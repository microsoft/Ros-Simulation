/* Copyright (c) Microsoft Corporation. All rights reserved.
   Licensed under the MIT License. */

/*
 * AUTHOR: Xinyi Joffre <xiou@microsoft.com>
 *
 * Publishes the ARGoS Simulation Clock to ROS Clock. The rosparam /use_sim_time should be
 * true if this plugin is being used.
 */

#ifndef ARGOS_ROS_CLOCK_H
#define ARGOS_ROS_CLOCK_H

#include <argos3/core/simulator/loop_functions.h>

#include <ros/ros.h>
#include <time.h>

using namespace argos;

class CArgosRosClock : public CLoopFunctions {

public:

  virtual ~CArgosRosClock() {};

  virtual void Init(TConfigurationNode& t_tree);

  virtual void PreStep();

  virtual void PostStep();

  Real TVTimeToHumanReadable(timeval& t_time);

private:

  UInt32 unTicksPerSec;
  UInt32 unRealTimeFactorFrequency;
  const UInt32 unDefaultRealTimeFactorFrequency = 10;
  UInt32 unRealTimeFactorIndex;
  float fRealTimeFactor_min;
  float fRealTimeFactor_max;
  float fRealTimeFactor_avg;

  ros::Time simTime;

  ros::Publisher clockPub;

private:
  /** The length of a clock tick */
  ::timeval m_tStepDuration;

  /** The starting time of a clock tick */
  ::timeval m_tStepStartTime;

  /** The end time of a clock tick */
  ::timeval m_tStepEndTime;

  /** The elapsed time between start and end of a clock tick */
  ::timeval m_tStepElapsedTime;

  void resetRealTimeFactorStatus();
  void updateRealTimeFactorStatus(float);

public:

  static ros::NodeHandle* nodeHandle;

};

#endif

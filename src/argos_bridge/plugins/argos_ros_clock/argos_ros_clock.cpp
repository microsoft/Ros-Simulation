#include "argos_ros_clock.h"

/* Include ros clock message definition */
#include "rosgraph_msgs/Clock.h"

#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/utility/configuration/argos_exception.h>

#include <float.h>

using namespace rosgraph_msgs;

/* Initialize ros node (this should reference the same node that is called in robot plugin as well) */
ros::NodeHandle* initROS() {
  int argc = 0;
  char *argv = (char *) "";
  ros::init(argc, &argv, "argos_bridge");
  return new ros::NodeHandle();
}

ros::NodeHandle* CArgosRosClock::nodeHandle = initROS();

void CArgosRosClock::Init(TConfigurationNode& t_tree) {
  simTime = ros::Time(0);
  clockPub = nodeHandle->advertise<Clock>("/clock", 1);

  TConfigurationNode root_tree = GetSimulator().GetConfigurationRoot();
  TConfigurationNode tFramework = GetNode(root_tree, "framework");
  TConfigurationNode tExperiment = GetNode(tFramework, "experiment");
  GetNodeAttribute(tExperiment, "ticks_per_second", unTicksPerSec);

  try
  {
    GetNodeAttribute(tExperiment, "rtf_frequency", unRealTimeFactorFrequency);
  }
  catch(CARGoSException e)
  {
    unRealTimeFactorFrequency = unDefaultRealTimeFactorFrequency;
  }

  resetRealTimeFactorStatus();

  /* Get the duration of a simulation step in microseconds */
  timerclear(&m_tStepDuration);
  m_tStepDuration.tv_usec = 1e6 * CPhysicsEngine::GetSimulationClockTick();

  /* Initialize the start time */
  timerclear(&m_tStepStartTime);
}

void CArgosRosClock::PreStep() {
  simTime = ros::Time(1.0 * GetSpace().GetSimulationClock() / unTicksPerSec);
  Clock currentSimTime;
  currentSimTime.clock = simTime;
  clockPub.publish(currentSimTime);
}

void CArgosRosClock::PostStep() {
  /* Calculate realtime factor if we've done at least one step */
  if(timerisset(&m_tStepStartTime)){
    /* Calculate the elapsed time */
    ::gettimeofday(&m_tStepEndTime, NULL);
    timersub(&m_tStepEndTime, &m_tStepStartTime, &m_tStepElapsedTime);
    /* Calculate the realtime factor */
    float fRealtimeFactor = TVTimeToHumanReadable(m_tStepDuration)
                          / TVTimeToHumanReadable(m_tStepElapsedTime);

    updateRealTimeFactorStatus(fRealtimeFactor);

    if(unRealTimeFactorIndex == unRealTimeFactorFrequency)
    {
      /* Log the result */
      LOG << "[INFO] Realtime Factor: avg " << fRealTimeFactor_avg / unRealTimeFactorFrequency
          << " min " << fRealTimeFactor_min << " max " << fRealTimeFactor_max << std::endl;

      LOG.Flush();

      resetRealTimeFactorStatus();
    }
  }
  /* Set the start time */
  ::gettimeofday(&m_tStepStartTime, NULL);
}

void CArgosRosClock::resetRealTimeFactorStatus()
{
  unRealTimeFactorIndex = 0;
  fRealTimeFactor_min = FLT_MAX;
  fRealTimeFactor_max = 0.0f;
  fRealTimeFactor_avg = 0.0f;
}

void CArgosRosClock::updateRealTimeFactorStatus(float fRealtimeFactor)
{
  unRealTimeFactorIndex++;
    
  fRealTimeFactor_min = fRealTimeFactor_min < fRealtimeFactor ? fRealTimeFactor_min : fRealtimeFactor;

  fRealTimeFactor_max = fRealTimeFactor_max > fRealtimeFactor ? fRealTimeFactor_max : fRealtimeFactor;

  fRealTimeFactor_avg += fRealtimeFactor;
}

Real CArgosRosClock::TVTimeToHumanReadable(::timeval& t_time) {
  return
    static_cast<Real>(t_time.tv_sec) +
    static_cast<Real>(t_time.tv_usec * 1e-6);
}

REGISTER_LOOP_FUNCTIONS(CArgosRosClock, "argos_ros_clock")

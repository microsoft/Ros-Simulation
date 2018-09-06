/*
 * AUTHOR: Andrew Vardy <av@mun.ca>
 * MODIFIED BY: Xinyi Joffre <xiou@microsoft.com>
 *
 * Connects an ARGoS robot with a particular configuration to ROS by publishing
 * sensor values and subscribing to a desired wheel speeds topic.
 */

#ifndef ARGOS_ROS_BOT_H
#define ARGOS_ROS_BOT_H

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>

#include <robots/generic/control_interface/ci_robot_ground_truth_sensor.h>
#include <robots/generic/control_interface/ci_robot_lidar_sensor.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <string>
#include <std_msgs/Bool.h>

using namespace argos;

class CArgosRosBot : public CCI_Controller {

public:

  CArgosRosBot();
  virtual ~CArgosRosBot() {}

  /*
   * This function initializes the controller.
   * The 't_node' variable points to the <parameters> section in the XML
   * file in the <controllers><footbot_ccw_wander_controller> section.
   */
  virtual void Init(TConfigurationNode& t_node);

  /*
   * This function is called once every time step.
   * The length of the time step is set in the XML file.
   */
  virtual void ControlStep();

  /*
   * This function resets the controller to its state right after the
   * Init().
   * It is called when you press the reset button in the GUI.
   * In this example controller there is no need for resetting anything,
   * so the function could have been omitted. It's here just for
   * completeness.
   */
  virtual void Reset() {}

  /*
   * Called to cleanup what done by Init() when the experiment finishes.
   * In this example controller there is no need for clean anything up,
   * so the function could have been omitted. It's here just for
   * completeness.
   */
  virtual void Destroy() {}

  /*
   * The callback method for getting new commanded speed on the cmd_vel topic.
   */
  void cmdVelCallback(const geometry_msgs::Twist& twist);

private:

  CCI_DifferentialSteeringActuator* m_pcWheels;
  CCI_RobotLIDARSensor* m_pcLidar;
  CCI_RobotGroundTruthSensor* m_pcGroundTruth;

  /*
   * The following variables are used as parameters for the
   * algorithm. You can set their value in the <parameters> section
   * of the XML configuration file, under the
   * <controllers><argos_ros_bot_controller> section.
   */

  // The number of time steps from the time step of the last callback
  // after which leftSpeed and rightSpeed will be set to zero.  Useful to
  // shutdown the robot after the controlling code on the ROS side has quit.
  int stopWithoutSubscriberCount;

  // The number of time steps since the last callback.
  int stepsSinceCallback;

  // The timeout for waiting for cmd_vel message for callback
  float callbackTimeoutInSeconds;

  // Most recent twist message
  geometry_msgs::Twist currentTwist;

  // Most recent left and right wheel speeds, converted from the ROS twist
  // message.
  Real leftSpeed, rightSpeed;

  // LaserScan sensor publisher
  ros::Publisher laserScanPub;

  // Odometry publisher
  ros::Publisher odomPub;

  // Base Ground Truth publisher
  ros::Publisher baseTruthPub;

  // Subscriber for cmd_vel (Twist message) topic.
  ros::Subscriber cmdVelSub;

  // Odom transform publisher
  ros::Publisher odomTfPub;

protected:

  void publishOdomTf();

  void publishOdom();

  void publishLaserScans();

public:
  // We need only a single ROS node, although there are individual publishers
  // and subscribers for each instance of the class.
  static ros::NodeHandle* nodeHandle;
};

#endif

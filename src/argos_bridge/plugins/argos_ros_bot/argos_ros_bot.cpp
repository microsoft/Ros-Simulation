/* ROS libraries */
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include <geometry_msgs/TransformStamped.h>
#include <tf2_msgs/TFMessage.h>

/* Include the controller definition */
#include "argos_ros_bot.h"

/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>

/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>

/* Turtlebot measurements */
#include <turtlebot/simulator/turtlebot_measures.h>

#include <iostream>
#include <sstream>

#include <ros/callback_queue.h>

using namespace std;
using namespace nav_msgs;
using namespace sensor_msgs;

// Initialize ROS node.  There will be only one ROS node no matter how many robots are created in
// ARGoS.  However, we will have one instance of the CArgosRosBot class for each ARGoS robot.
ros::NodeHandle* initROS() {
  int argc = 0;
  char *argv = (char *) "";
  ros::init(argc, &argv, "argos_bridge");
  return new ros::NodeHandle();
}

ros::NodeHandle* CArgosRosBot::nodeHandle = initROS();

/****************************************/
/****************************************/

CArgosRosBot::CArgosRosBot() :
  m_pcWheels(NULL),
  m_pcGroundTruth(NULL),
  m_pcLidar(NULL),
  stopWithoutSubscriberCount(10),
  stepsSinceCallback(0),
  callbackTimeoutInSeconds(0.1),
  leftSpeed(0),
  rightSpeed(0)
{
}

/****************************************/
/****************************************/

void CArgosRosBot::Init(TConfigurationNode& t_node) {

  // Create empty twist
  currentTwist = geometry_msgs::Twist();

  // Create the topics to publish
  stringstream odomTopic, baseTruthTopic, laserScanTopic, odomTfTopic;
  odomTopic << "/" << GetId() << "/odom";
  baseTruthTopic << "/" << GetId() << "/base_pose_ground_truth";
  laserScanTopic << "/" << GetId() << "/scan";
  odomTfTopic << "/" << GetId() << "/odom_tf";
  odomPub = nodeHandle->advertise<Odometry>(odomTopic.str(), 10);
  baseTruthPub = nodeHandle->advertise<Odometry>(baseTruthTopic.str(), 10);
  odomTfPub = nodeHandle->advertise<tf2_msgs::TFMessage>(odomTfTopic.str(), 10);
  laserScanPub = nodeHandle->advertise<LaserScan>(laserScanTopic.str(), 10);

  // Create the subscribers
  stringstream cmdVelTopic;
  cmdVelTopic << "/" << GetId() << "/cmd_vel";
  cmdVelSub = nodeHandle->subscribe(cmdVelTopic.str(), 1, &CArgosRosBot::cmdVelCallback, this);

  // Get sensor/actuator handles
  m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
  m_pcGroundTruth = GetSensor<CCI_RobotGroundTruthSensor>("robot_ground_truth");
  m_pcLidar = GetSensor<CCI_RobotLIDARSensor>("robot_lidar");

  /*
   * Parse the configuration file
   *
   * The user defines this part. Here, the algorithm accepts three
   * parameters and it's nice to put them in the config file so we don't
   * have to recompile if we want to try other settings.
   */
  GetNodeAttributeOrDefault(t_node, "stopWithoutSubscriberCount", stopWithoutSubscriberCount, stopWithoutSubscriberCount);

  GetNodeAttributeOrDefault(t_node, "callbackTimeoutInSeconds", callbackTimeoutInSeconds, callbackTimeoutInSeconds);

}

/****************************************/
/****************************************/

void CArgosRosBot::ControlStep() {

  /* Publish updated positions and sensor data */
  publishOdomTf();
  publishOdom();
  publishLaserScans();

  // Wait for any callbacks to be called.
  ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(callbackTimeoutInSeconds));

  // If we haven't heard from the subscriber in a while, set the speed to zero.
  if (stepsSinceCallback > stopWithoutSubscriberCount) {
    leftSpeed = 0;
    rightSpeed = 0;
    currentTwist = geometry_msgs::Twist();
  } else {
    stepsSinceCallback++;
  }

  m_pcWheels->SetLinearVelocity(leftSpeed, rightSpeed);
}

/****************************************/
/****************************************/

void CArgosRosBot::publishOdomTf() {

  ros::Time current_time = ros::Time::now();

  /* Publish odom tf update */
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";
  odom_trans.transform.translation.x = m_pcGroundTruth->GetReading().Position.GetX();
  odom_trans.transform.translation.y = m_pcGroundTruth->GetReading().Position.GetY();
  odom_trans.transform.translation.z = m_pcGroundTruth->GetReading().Position.GetZ();
  odom_trans.transform.rotation.x = m_pcGroundTruth->GetReading().Orientation.GetX();
  odom_trans.transform.rotation.y = m_pcGroundTruth->GetReading().Orientation.GetY();
  odom_trans.transform.rotation.z = m_pcGroundTruth->GetReading().Orientation.GetZ();
  odom_trans.transform.rotation.w = m_pcGroundTruth->GetReading().Orientation.GetW();

  /* Convert TransformStamped message to TFMessage to publish */
  tf2_msgs::TFMessage tfMessage;
  tfMessage.transforms.push_back(odom_trans);

  odomTfPub.publish(tfMessage);
}

/****************************************/
/****************************************/

void CArgosRosBot::publishOdom() {

  Odometry odom;

  /* Publish odom */
  odom.header.stamp = ros::Time::now();
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";

  odom.pose.pose.position.x = m_pcGroundTruth->GetReading().Position.GetX();
  odom.pose.pose.position.y = m_pcGroundTruth->GetReading().Position.GetY();
  odom.pose.pose.position.z = m_pcGroundTruth->GetReading().Position.GetZ();

  odom.pose.pose.orientation.x = m_pcGroundTruth->GetReading().Orientation.GetX();
  odom.pose.pose.orientation.y = m_pcGroundTruth->GetReading().Orientation.GetY();
  odom.pose.pose.orientation.z = m_pcGroundTruth->GetReading().Orientation.GetZ();
  odom.pose.pose.orientation.w = m_pcGroundTruth->GetReading().Orientation.GetW();

  //TO-DO: Still need to figure out how to calculate velocity of entity from differential steering sensor
  //Current approach may be inaccurate if simulator and ros are not synced
  odom.twist.twist = currentTwist;

  odomPub.publish(odom);
  baseTruthPub.publish(odom);
}

/****************************************/
/****************************************/

void CArgosRosBot::publishLaserScans() {

  ros::Time current_time = ros::Time::now();

  /* Publish laser scan */
  unsigned int num_readings = m_pcLidar->GetNumReadings();
  double laser_frequency = 1;
  double ranges[num_readings];
  double intensities [num_readings];

  for(unsigned int i=0; i < num_readings; ++i){
    ranges[i] = m_pcLidar->GetReading(i);;
    intensities[i] = 0.0;
  }

  LaserScan scan;
  scan.header.stamp = current_time;
  scan.header.frame_id = "laser_frame";
  scan.angle_min = -TURTLEBOT_LIDAR_ANGLE_SPAN.GetValue() * 0.5;
  scan.angle_max = TURTLEBOT_LIDAR_ANGLE_SPAN.GetValue() * 0.5;
  scan.angle_increment = TURTLEBOT_LIDAR_ANGLE_SPAN.GetValue() / num_readings;
  scan.time_increment = (1 / laser_frequency) / (num_readings);
  scan.range_min = TURTLEBOT_LIDAR_RADIUS + TURTLEBOT_LIDAR_SENSORS_RING_RANGE.GetMin();
  scan.range_max = TURTLEBOT_LIDAR_RADIUS + TURTLEBOT_LIDAR_SENSORS_RING_RANGE.GetMax();

  scan.ranges.resize(num_readings);
  scan.intensities.resize(num_readings);

  for(unsigned int i=0; i < num_readings; ++i) {
    scan.ranges[i] = ranges[i];
    scan.intensities[i] = intensities[i];
  }

  laserScanPub.publish(scan);
}

/****************************************/
/****************************************/

void CArgosRosBot::cmdVelCallback(const geometry_msgs::Twist& twist) {

  Real v = twist.linear.x;  // Forward speed
  Real w = twist.angular.z; // Rotational speed

  currentTwist = twist;

  // Use the kinematics of a differential-drive robot to derive the left
  // and right wheel speeds.
  leftSpeed = (v - TURTLEBOT_HALF_WHEEL_DISTANCE * w) / TURTLEBOT_WHEEL_RADIUS;
  rightSpeed = (v + TURTLEBOT_HALF_WHEEL_DISTANCE * w) / TURTLEBOT_WHEEL_RADIUS;

  stepsSinceCallback = 0;
}

/*
* This statement notifies ARGoS of the existence of the controller.
* It binds the class passed as first argument to the string passed as
* second argument.
* The string is then usable in the configuration file to refer to this
* controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CArgosRosBot, "argos_ros_bot_controller")

/**
 * @author Xinyi Joffre - <xiou@microsoft.com>
 * @author Yi Gu - <guyi@microsoft.com>
 */

#include "turtlebot_measures.h"

/** References measures from:
 * https://www.roscomponents.com/en/mobile-robots/215-turtlebot-3-waffle.html
 *https://github.com/ROBOTIS-GIT/turtlebot3/blob/master/turtlebot3_description/urdf/turtlebot3_waffle_pi.urdf.xacro
 *https://github.com/ROBOTIS-GIT/turtlebot3/blob/master/turtlebot3_description/urdf/turtlebot3_waffle_pi.gazebo.xacro
 **/

const Real TURTLEBOT_MAX_FORCE      = 15.0f; // missing
const Real TURTLEBOT_MAX_TORQUE     = 150.0f; // wheel torque
const Real TURTLEBOT_MAX_PAYLOAD    = 30.0f; // kilograms 
const Real TURTLEBOT_MASS           = 1.8f; // kilograms
const Real TURTLEBOT_MAX_TRANSLATIONAL_VEL = 0.26f; // m/s 
const Real TURTLEBOT_MAX_ROTATIONAL_VEL = 1.82f; //rad/s

/*
* If the turtle turns around
* the inner radius is TURTLEBOT_BASE_RADIUS, 
* and the outter radius is TURTLEBOT_WIDE_RADIUS
*/
const Real TURTLEBOT_WIDE_RADIUS    = 0.220f;
const Real TURTLEBOT_BASE_RADIUS    = 0.1435f; 

/* Wheel related measures */
const Real TURTLEBOT_WHEEL_WIDTH         = 0.0190f;
const Real TURTLEBOT_WHEEL_RADIUS        = 0.033f;
const Real TURTLEBOT_WHEEL_DISTANCE      = 0.287f; 
const Real TURTLEBOT_HALF_WHEEL_DISTANCE = TURTLEBOT_WHEEL_DISTANCE * 0.5;
const Real TURTLEBOT_AXLE_TO_FRONT       = TURTLEBOT_BASE_LENGTH * 0.5 - 0.064f;
const Real TURTLEBOT_AXLE_TO_REAR        = TURTLEBOT_BASE_LENGTH * 0.5 + 0.064f;

const Real TURTLEBOT_BASE_LENGTH    = 0.281f;
const Real TURTLEBOT_BASE_WIDTH     = 0.306f - (2 * TURTLEBOT_WHEEL_WIDTH);
const Real TURTLEBOT_BASE_HEIGHT    = 0.1015f;

const Real TURTLEBOT_BASE_ELEVATION = 0.010f;
const Real TURTLEBOT_BASE_TOP       = TURTLEBOT_BASE_ELEVATION + TURTLEBOT_BASE_HEIGHT;

const Real TURTLEBOT_OVERALL_HEIGHT = 0.141f;

/* LIDAR related measures */
const Real TURTLEBOT_LIDAR_HEIGHT           = 0.0395f;
const Real TURTLEBOT_LIDAR_ELEVATION        = TURTLEBOT_BASE_TOP;
const Real TURTLEBOT_LIDAR_FORWARD_OFFSET   = 0.0f;
const Real TURTLEBOT_LIDAR_LEFT_OFFSET      = 0.0f;
const Real TURTLEBOT_LIDAR_RADIUS           = 0.03475f;

const CRadians TURTLEBOT_LIDAR_ANGLE_SPAN(6.28319);
const CRange<Real> TURTLEBOT_LIDAR_SENSORS_RING_RANGE(0.12, 3.5);

/* Camera related measures -- currently not used */
const Real TURTLEBOT_CAMERA_HORIZONTAL_FOV = 1.085595;
const UInt32 TURTLEBOT_CAMERA_WIDTH = 640;
const UInt32 TURTLEBOT_CAMERA_HEIGHT = 480;
const std::string TURTLEBOT_CAMERA_FORMAT = "R8G8B8";
const Real TURTLEBOT_CAMERA_HORIZONTAL_NEAR = 0.03f;
const Real TURTLEBOT_CAMERA_HORIZONTAL_FAR = 100.0f;
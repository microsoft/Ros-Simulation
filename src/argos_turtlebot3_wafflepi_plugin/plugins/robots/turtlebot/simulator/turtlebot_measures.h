/**
 * @author Xinyi Joffre - <xiou@microsoft.com>
 * @author Yi Gu - <guyi@microsoft.com>
 */

#ifndef TURTLEBOT_MEASURES_H
#define TURTLEBOT_MEASURES_H

#include <argos3/core/utility/datatypes/datatypes.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/vector3.h>

using namespace argos;

extern const Real TURTLEBOT_MAX_FORCE; 
extern const Real TURTLEBOT_MAX_TORQUE;
extern const Real TURTLEBOT_MAX_PAYLOAD; 
extern const Real TURTLEBOT_MASS;
extern const Real TURTLEBOT_MAX_TRANSLATIONAL_VEL; 
extern const Real TURTLEBOT_MAX_ROTATIONAL_VEL;

extern const Real TURTLEBOT_WHEEL_WIDTH;
extern const Real TURTLEBOT_WHEEL_RADIUS;
extern const Real TURTLEBOT_WHEEL_DISTANCE;
extern const Real TURTLEBOT_HALF_WHEEL_DISTANCE;
extern const Real TURTLEBOT_AXLE_TO_FRONT;
extern const Real TURTLEBOT_AXLE_TO_REAR;

extern const Real TURTLEBOT_WIDE_RADIUS;
extern const Real TURTLEBOT_BASE_RADIUS;

extern const Real TURTLEBOT_BASE_ELEVATION;
extern const Real TURTLEBOT_BASE_TOP;

extern const Real TURTLEBOT_BASE_HEIGHT;
extern const Real TURTLEBOT_BASE_WIDTH;
extern const Real TURTLEBOT_BASE_LENGTH;

extern const Real TURTLEBOT_OVERALL_HEIGHT;

extern const Real TURTLEBOT_LIDAR_HEIGHT;
extern const Real TURTLEBOT_LIDAR_ELEVATION;
extern const Real TURTLEBOT_LIDAR_RADIUS;
extern const Real TURTLEBOT_LIDAR_FORWARD_OFFSET;
extern const Real TURTLEBOT_LIDAR_LEFT_OFFSET;
extern const CRadians TURTLEBOT_LIDAR_ANGLE_SPAN;
extern const CRange<Real> TURTLEBOT_LIDAR_SENSORS_RING_RANGE;

extern const Real TURTLEBOT_CAMERA_HORIZONTAL_FOV;
extern const UInt32 TURTLEBOT_CAMERA_WIDTH;
extern const UInt32 TURTLEBOT_CAMERA_HEIGHT;
extern const std::string TURTLEBOT_CAMERA_FORMAT;
extern const Real TURTLEBOT_CAMERA_HORIZONTAL_NEAR;
extern const Real TURTLEBOT_CAMERA_HORIZONTAL_FAR;

#endif

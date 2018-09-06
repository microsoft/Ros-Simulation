/**
 *
 * @brief This file provides the definition of the RoBot Ground Truth sensor.
 *
 * This file provides the definition of the RoBot Ground Truth sensor.
 *
 * @author Carlo Pinciroli <ilpincy@gmail.com>
 * @author Xinyi Joffre <xiou@microsoft.com>
 * @author Yi Gu <guyi@microsoft.com>
 */

#ifndef CCI_ROBOT_GROUND_TRUTH_SENSOR_H
#define CCI_ROBOT_GROUND_TRUTH_SENSOR_H

namespace argos {
   class CCI_RobotGroundTruthSensor;
}

#include <argos3/core/control_interface/ci_sensor.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/angles.h>

namespace argos {

   class CCI_RobotGroundTruthSensor : public CCI_Sensor {

   public:
      struct SReading {
         CVector3 Position;
         CQuaternion Orientation;
      };

   public:

      /**
       * Class constructor
       */
      CCI_RobotGroundTruthSensor();

      /**
       * Class destructor
       */
      virtual ~CCI_RobotGroundTruthSensor() {}

      /**
       * Returns the readings of this sensor
       */
      const SReading& GetReading() const;

   protected:
      SReading m_sReading;

#ifdef ARGOS_WITH_LUA
      virtual void CreateLuaState(lua_State* pt_lua_state);

      virtual void ReadingsToLuaState(lua_State* pt_lua_state);
#endif

   };

}

#endif

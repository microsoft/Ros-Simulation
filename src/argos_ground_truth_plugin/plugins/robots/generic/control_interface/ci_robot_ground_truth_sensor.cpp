/**
 *
 * @author Carlo Pinciroli <ilpincy@gmail.com>
 * @author Xinyi Joffre <xiou@microsoft.com>
 * @author Yi Gu <guyi@microsoft.com>
 */

#include "ci_robot_ground_truth_sensor.h"
#include <argos3/core/utility/math/angles.h>

#ifdef ARGOS_WITH_LUA
#include <argos3/core/wrappers/lua/lua_utility.h>
#endif

namespace argos {

   /****************************************/
   /****************************************/

   CCI_RobotGroundTruthSensor::CCI_RobotGroundTruthSensor() {
   }


   const CCI_RobotGroundTruthSensor::SReading& CCI_RobotGroundTruthSensor::GetReading() const {
     return m_sReading;
   }

   /****************************************/
   /****************************************/

#ifdef ARGOS_WITH_LUA
   void CCI_RobotGroundTruthSensor::CreateLuaState(lua_State* pt_lua_state) {

   }
#endif

   /****************************************/
   /****************************************/

#ifdef ARGOS_WITH_LUA
   void CCI_RobotGroundTruthSensor::ReadingsToLuaState(lua_State* pt_lua_state) {

   }
#endif


   /****************************************/
   /****************************************/

}

/**
 *
 * @author Carlo Pinciroli <ilpincy@gmail.com>
 * @author Xinyi Joffre <xiou@microsoft.com>
 * @author Yi Gu <guyi@microsoft.com>
 */

#include "ci_robot_lidar_sensor.h"
#include <argos3/core/utility/math/angles.h>

#ifdef ARGOS_WITH_LUA
#include <argos3/core/wrappers/lua/lua_utility.h>
#endif

namespace argos {

   /****************************************/
   /****************************************/

   CCI_RobotLIDARSensor::CCI_RobotLIDARSensor() {
   }

   /****************************************/
   /****************************************/

#ifdef ARGOS_WITH_LUA
   void CCI_RobotLIDARSensor::CreateLuaState(lua_State* pt_lua_state) {
       CLuaUtility::OpenRobotStateTable(pt_lua_state, "lidar");
       for(size_t i = 0; i < GetNumReadings(); ++i) {
          CLuaUtility::StartTable(pt_lua_state, i+1);
          CLuaUtility::AddToTable(pt_lua_state, "value", GetReading(i));
          CLuaUtility::EndTable  (pt_lua_state);
       }
       CLuaUtility::CloseRobotStateTable(pt_lua_state);
   }
#endif

   /****************************************/
   /****************************************/

#ifdef ARGOS_WITH_LUA
   void CCI_RobotLIDARSensor::ReadingsToLuaState(lua_State* pt_lua_state) {
      lua_getfield(pt_lua_state, -1, "lidar");
      for(size_t i = 0; i < GetNumReadings(); ++i) {
          lua_pushnumber(pt_lua_state, i+1                 );
          lua_gettable  (pt_lua_state, -2                  );
          lua_pushnumber(pt_lua_state, GetReading(i));
          lua_setfield  (pt_lua_state, -2, "value"         );
          lua_pop(pt_lua_state, 1);
       }
       lua_pop(pt_lua_state, 1);
   }
#endif


   /****************************************/
   /****************************************/

}

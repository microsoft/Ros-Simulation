/**
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 * 
 * Modified by Xinyi Joffre - <xiou@microsoft.com>
 * 
 * Modified by Yi Gu <guyi@microsoft.com>
 * 
 */

#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/core/simulator/simulator.h>

#include <argos3/core/utility/datatypes/datatypes.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/vector3.h>

#include "robot_lidar_default_sensor.h"
//#include <Robot/simulator/Robot_measures.h>

#include <limits>
#include <cstring>

namespace argos {

   /****************************************/
   /****************************************/

   static UInt8 ROBOT_POWERON_LASERON   = 3;

   /****************************************/
   /****************************************/

   CRobotLIDARDefaultSensor::CRobotLIDARDefaultSensor() :
      m_pnReadings(NULL),
      m_unNumReadings(360),
      m_unPowerLaserState(ROBOT_POWERON_LASERON),
      m_pcEmbodiedEntity(NULL),
      m_bShowRays(false),
      m_pcRNG(NULL),
      m_bAddNoise(false),
      m_cSpace(CSimulator::GetInstance().GetSpace()) {}

   /****************************************/
   /****************************************/

   CRobotLIDARDefaultSensor::~CRobotLIDARDefaultSensor() {
   }

   /****************************************/
   /****************************************/

   void CRobotLIDARDefaultSensor::SetRobot(CComposableEntity& c_entity) {
      try {
         m_pcEmbodiedEntity = &(c_entity.GetComponent<CEmbodiedEntity>("body"));
         m_pcControllableEntity = &(c_entity.GetComponent<CControllableEntity>("controller"));
         m_pcLIDARSensorEquippedEntity = &(c_entity.GetComponent<CLIDARSensorEquippedEntity>("lidar_sensors[lidar]"));
         m_pcLIDARSensorEquippedEntity->Enable();
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Can't set robot for the Robot LIDAR default sensor", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CRobotLIDARDefaultSensor::Init(TConfigurationNode& t_tree) {
      try {

         CCI_RobotLIDARSensor::Init(t_tree);

         /* How many readings? */
         GetNodeAttributeOrDefault(t_tree, "num_readings", m_unNumReadings, m_unNumReadings);
         m_pnReadings = new Real[m_unNumReadings];

         /* Show rays? */
         GetNodeAttributeOrDefault(t_tree, "show_rays", m_bShowRays, m_bShowRays);
         
         /* Parse noise level */
         Real fNoiseLevel = 0.0f;
         GetNodeAttributeOrDefault(t_tree, "noise_level", fNoiseLevel, fNoiseLevel);
         if(fNoiseLevel < 0.0f) {
            THROW_ARGOSEXCEPTION("Can't specify a negative value for the noise level of the lidar sensor");
         }
         else if(fNoiseLevel > 0.0f) {
            m_bAddNoise = true;
            m_cNoiseRange.Set(-fNoiseLevel, fNoiseLevel);
            m_pcRNG = CRandom::CreateRNG("argos");
         }
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Initialization error in default lidar sensor", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CRobotLIDARDefaultSensor::Update() {
      /* Nothing to do if sensor is deactivated */
      if(m_unPowerLaserState != ROBOT_POWERON_LASERON)
         return;
      /* Ray used for scanning the environment for obstacles */
      CRay3 cScanningRay;
      CVector3 cRayStart, cRayEnd, cOffset, cPosition, cDirection;
      CQuaternion cOrientation;

      /* Buffers to contain data about the intersection */
      SEmbodiedEntityIntersectionItem sIntersection;
      /* Go through the sensors */
      for(UInt32 i = 0; i < m_unNumReadings; ++i) {

         CLIDARSensorEquippedEntity::SSensor sensor = m_pcLIDARSensorEquippedEntity->GetSensor(i);

         /* Compute ray for sensor i */
         cOffset = sensor.Offset;
         cPosition = sensor.Anchor.Position;
         cDirection = sensor.Direction;
         cOrientation = sensor.Anchor.Orientation;

         cRayStart = cOffset;
         cRayStart.Rotate(cOrientation);
         cRayStart += cPosition;
         cRayEnd = cOffset;
         cRayEnd += cDirection;
         cRayEnd.Rotate(cOrientation);
         cRayEnd += cPosition;
         cScanningRay.Set(cRayStart,cRayEnd);

         /* Compute reading */
         /* Get the closest intersection */
         if(GetClosestEmbodiedEntityIntersectedByRay(sIntersection,
                                                     cScanningRay,
                                                     *m_pcEmbodiedEntity)) {
            /* There is an intersection */
            if(m_bShowRays) {
               m_pcControllableEntity->AddIntersectionPoint(cScanningRay,
                                                            sIntersection.TOnRay);
               m_pcControllableEntity->AddCheckedRay(true, cScanningRay);
            }
            /* The actual reading is in meters */
            m_pnReadings[i] = cScanningRay.GetDistance(sIntersection.TOnRay);

         }
         else {
            /* No intersection */
            m_pnReadings[i] = std::numeric_limits<Real>::infinity();
            if(m_bShowRays) {
               m_pcControllableEntity->AddCheckedRay(false, cScanningRay);
            }
         }
         /* Apply noise to the sensor */
         if(m_bAddNoise) {
            m_pnReadings[i] += m_pcRNG->Uniform(m_cNoiseRange);
         }
      }
   }

   /****************************************/
   /****************************************/

   void CRobotLIDARDefaultSensor::Reset() {
      memset(m_pnReadings, 0, m_unNumReadings * sizeof(Real));
   }

   /****************************************/
   /****************************************/

   void CRobotLIDARDefaultSensor::Destroy() {
      delete[] m_pnReadings;
   }

   /****************************************/
   /****************************************/

   Real CRobotLIDARDefaultSensor::GetReading(UInt32 un_idx) const {
      return m_pnReadings[un_idx];
   }

   /****************************************/
   /****************************************/

   size_t CRobotLIDARDefaultSensor::GetNumReadings() const {
      return m_unNumReadings;
   }

   /****************************************/
   /****************************************/

   void CRobotLIDARDefaultSensor::PowerOn() {
      m_unPowerLaserState = m_unPowerLaserState | 0x1;
      m_pcLIDARSensorEquippedEntity->SetEnabled(m_unPowerLaserState == ROBOT_POWERON_LASERON);
   }

   /****************************************/
   /****************************************/

   void CRobotLIDARDefaultSensor::PowerOff() {
      m_unPowerLaserState = m_unPowerLaserState & 0xFE;
      m_pcLIDARSensorEquippedEntity->SetEnabled(m_unPowerLaserState == ROBOT_POWERON_LASERON);
   }

   /****************************************/
   /****************************************/

   void CRobotLIDARDefaultSensor::LaserOn() {
      m_unPowerLaserState = m_unPowerLaserState | 0x2;
      m_pcLIDARSensorEquippedEntity->SetEnabled(m_unPowerLaserState == ROBOT_POWERON_LASERON);
   }

   /****************************************/
   /****************************************/

   void CRobotLIDARDefaultSensor::LaserOff() {
      m_unPowerLaserState = m_unPowerLaserState & 0xFD;
      m_pcLIDARSensorEquippedEntity->SetEnabled(m_unPowerLaserState == ROBOT_POWERON_LASERON);
   }

   /****************************************/
   /****************************************/

   REGISTER_SENSOR(CRobotLIDARDefaultSensor,
                   "robot_lidar", "default",
                   "Xinyi Joffre [xiou@microsoft.com]",
                   "1.0",
                   "The Robot LIDAR sensor.",
                   "This sensor accesses the Robot LIDAR sensor. The sensors return the\n"
                   "distance to nearby objects in meters. In controllers, you must include the\n"
                   "ci_robot_lidar_sensor.h header.\n\n"
                   "REQUIRED XML CONFIGURATION\n\n"
                   "  <controllers>\n"
                   "    ...\n"
                   "    <my_controller ...>\n"
                   "      ...\n"
                   "      <sensors>\n"
                   "        ...\n"
                   "        <lidar implementation=\"default\" />\n"
                   "        ...\n"
                   "      </sensors>\n"
                   "      ...\n"
                   "    </my_controller>\n"
                   "    ...\n"
                   "  </controllers>\n\n"
                   "OPTIONAL XML CONFIGURATION\n\n"
                   "It is possible to draw the rays shot by the LIDAR sensor in the OpenGL\n"
                   "visualization. This can be useful for sensor debugging but also to understand\n"
                   "what's wrong in your controller. In OpenGL, the rays are drawn in cyan when\n"
                   "they are not obstructed and in purple when they are. In case a ray is\n"
                   "obstructed, a black dot is drawn where the intersection occurred.\n"
                   "To turn this functionality on, add the attribute \"show_rays\" as in this\n"
                   "example:\n\n"
                   "  <controllers>\n"
                   "    ...\n"
                   "    <my_controller ...>\n"
                   "      ...\n"
                   "      <sensors>\n"
                   "        ...\n"
                   "        <lidar implementation=\"default\"\n"
                   "               show_rays=\"true\" />\n"
                   "        ...\n"
                   "      </sensors>\n"
                   "      ...\n"
                   "    </my_controller>\n"
                   "    ...\n"
                   "  </controllers>\n\n"
                   "It is possible to change the default number of readings to make computation\n"
                   "faster. The default number of readings is 682, but using the 'num_readings'\n"
                   "attribute you can change it to a different value:\n\n"
                   "  <controllers>\n"
                   "    ...\n"
                   "    <my_controller ...>\n"
                   "      ...\n"
                   "      <sensors>\n"
                   "        ...\n"
                   "        <lidar implementation=\"default\"\n"
                   "               num_readings=\"100\" />\n"
                   "        ...\n"
                   "      </sensors>\n"
                   "      ...\n"
                   "    </my_controller>\n"
                   "    ...\n"
                   "  </controllers>\n\n"
                   "It is possible to add uniform noise to the sensors, thus matching the\n"
                   "characteristics of a real robot better. This can be done with the attribute\n"
                   "\"noise_level\", whose allowed range is in [-1,1] and is added to the calculated\n"
                   "reading. The final sensor reading is always normalized in the [0-1] range.\n\n"
                   "  <controllers>\n"
                   "    ...\n"
                   "    <my_controller ...>\n"
                   "      ...\n"
                   "      <sensors>\n"
                   "        ...\n"
                   "        <lidar implementation=\"default\"\n"
                   "               noise_level=\"0.1\" />\n"
                   "        ...\n"
                   "      </sensors>\n"
                   "      ...\n"
                   "    </my_controller>\n"
                   "    ...\n"
                   "  </controllers>\n\n",
                   "Usable"
		  );

}

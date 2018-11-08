/**
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 * @author Xinyi Joffre - <xiou@microsoft.com>
 * @author Yi Gu <guyi@microsoft.com>
 */

#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/core/simulator/simulator.h>

#include "robot_ground_truth_default_sensor.h"

#include <cstring>

namespace argos {

   /****************************************/
   /****************************************/

   CRobotGroundTruthDefaultSensor::CRobotGroundTruthDefaultSensor() :
      m_pcEmbodiedEntity(NULL),
      m_pcRNG(NULL),
      m_bAddNoise(false),
      m_cSpace(CSimulator::GetInstance().GetSpace()) {}

   /****************************************/
   /****************************************/

   CRobotGroundTruthDefaultSensor::~CRobotGroundTruthDefaultSensor() {
   }

   /****************************************/
   /****************************************/

   void CRobotGroundTruthDefaultSensor::SetRobot(CComposableEntity& c_entity) {
      try {
         m_pcEmbodiedEntity = &(c_entity.GetComponent<CEmbodiedEntity>("body"));
         m_pcControllableEntity = &(c_entity.GetComponent<CControllableEntity>("controller"));
         m_pcGroundTruthEntity = &(c_entity.GetComponent<CGroundTruthEquippedEntity>("ground_truth"));
         m_pcGroundTruthEntity->Enable();

         m_sReading.Position = m_pcEmbodiedEntity->GetOriginAnchor().Position;
         m_sReading.Orientation = m_pcEmbodiedEntity->GetOriginAnchor().Orientation;
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Can't set robot for the Robot ground truth default sensor", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CRobotGroundTruthDefaultSensor::Init(TConfigurationNode& t_tree) {
      try {

         CCI_RobotGroundTruthSensor::Init(t_tree);

         m_pcGroundTruthEntity->AddSensor(
            CVector2(0.0, 0.0),
            m_pcEmbodiedEntity->GetOriginAnchor());
        
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

   void CRobotGroundTruthDefaultSensor::Update() {

       m_sReading.Position = m_pcEmbodiedEntity->GetOriginAnchor().Position;
       m_sReading.Orientation = m_pcEmbodiedEntity->GetOriginAnchor().Orientation;

   }

   /****************************************/
   /****************************************/

   void CRobotGroundTruthDefaultSensor::Reset() {
      m_sReading.Position = m_pcEmbodiedEntity->GetOriginAnchor().Position;
      m_sReading.Orientation = m_pcEmbodiedEntity->GetOriginAnchor().Orientation;
    }

   /****************************************/
   /****************************************/

   REGISTER_SENSOR(CRobotGroundTruthDefaultSensor,
                   "robot_ground_truth", "default",
                   "Carlo Pinciroli [ilpincy@gmail.com]",
                   "1.0",
                   "",
                   "",
                   "Usable"
		  );

}

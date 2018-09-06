/**
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 * @author Xinyi Joffre - <xiou@microsoft.com>
 * @author Yi Gu - <guyi@microsoft.com>
 */

#ifndef TURTLEBOT_ENTITY_H
#define TURTLEBOT_ENTITY_H

namespace argos {
   class CControllableEntity;
   class CEmbodiedEntity;
   class CLIDARSensorEquippedEntity;
   class CGroundTruthEquippedEntity;
}

#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/plugins/simulator/entities/wheeled_entity.h>

#include <simulator/entities/ground_truth_equipped_entity.h>
#include <simulator/entities/lidar_sensor_equipped_entity.h>

namespace argos {
   class CTurtleBotEntity : public CComposableEntity {

   public:

      ENABLE_VTABLE();

   public:

      CTurtleBotEntity();

      CTurtleBotEntity(const std::string& str_id,
                     const std::string& str_controller_id,
                     const CVector3& c_position = CVector3(),
                     const CQuaternion& c_orientation = CQuaternion());

      virtual void Init(TConfigurationNode& t_tree);
      virtual void Reset();
      virtual void UpdateComponents();
      
      inline CControllableEntity& GetControllableEntity() {
         return *m_pcControllableEntity;
      }

      inline CEmbodiedEntity& GetEmbodiedEntity() {
         return *m_pcEmbodiedEntity;
      }

      inline CLIDARSensorEquippedEntity& GetLIDARSensorEquippedEntity() {
         return *m_pcLIDARSensorEquippedEntity;
      }

      inline CWheeledEntity& GetWheeledEntity() {
         return *m_pcWheeledEntity;
      }

      virtual std::string GetTypeDescription() const {
         return "turtlebot";
      }

   private:

      CControllableEntity*                   m_pcControllableEntity;
      CEmbodiedEntity*                       m_pcEmbodiedEntity;
      CLIDARSensorEquippedEntity*            m_pcLIDARSensorEquippedEntity;
      CGroundTruthEquippedEntity*            m_pcGroundTruthSensorEquippedEntity;
      CWheeledEntity*                        m_pcWheeledEntity;
    };

}

#endif

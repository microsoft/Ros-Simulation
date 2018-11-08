/**
 * @file <argos3/plugins/simulator/entities/ground_sensor_equipped_entity.h>
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 * @author Xinyi Joffre - <xiou@microsoft.com>
 */

#ifndef GROUND_TRUTH_EQUIPPED_ENTITY_H
#define GROUND_TRUTH_EQUIPPED_ENTITY_H

namespace argos {
   class CGroundTruthEquippedEntity;
}

#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/simulator/entity/entity.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <map>

namespace argos {

   class CGroundTruthEquippedEntity : public CEntity {

   public:

      ENABLE_VTABLE();

      struct SSensor {
         typedef std::vector<SSensor*> TList;

         CVector2 Offset;
         SAnchor& Anchor;
 
         SSensor(const CVector2& c_position,
                 SAnchor& s_anchor) :
            Offset(c_position),
            Anchor(s_anchor) {}
      };

   public:

      CGroundTruthEquippedEntity(CComposableEntity* pc_parent);

      CGroundTruthEquippedEntity(CComposableEntity* pc_parent,
                                  const std::string& str_id);

      virtual ~CGroundTruthEquippedEntity();

      virtual void Init(TConfigurationNode& t_tree);

      virtual std::string GetTypeDescription() const {
         return "ground_truth";
      }

      virtual void Enable();

      virtual void Disable();

      inline size_t GetNumSensors() const {
         return m_tSensors.size();
      }

      inline SSensor& GetSensor(size_t un_idx) {
         return *m_tSensors[un_idx];
      }

      inline SSensor::TList& GetSensors() {
         return m_tSensors;
      }

      void AddSensor(const CVector2& c_offset,
                     SAnchor& s_anchor);

   protected:

      /** The list of sensors */
      SSensor::TList m_tSensors;

   };

}

#endif
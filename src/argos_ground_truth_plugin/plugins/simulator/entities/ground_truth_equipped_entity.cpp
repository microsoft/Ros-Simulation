
/**
 * @file <argos3/plugins/simulator/entities/ground_sensor_equipped_entity.cpp>
 *
 * @author Carlo Pinciroli <ilpincy@gmail.com>
 * @author Xinyi Joffre <xiou@microsoft.com>
 */
#include "ground_truth_equipped_entity.h"
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/entity/composable_entity.h>

namespace argos {

   /****************************************/
   /****************************************/

   CGroundTruthEquippedEntity::CGroundTruthEquippedEntity(CComposableEntity* pc_parent) :
      CEntity(pc_parent) {
      Disable();
   }
   
   /****************************************/
   /****************************************/

   CGroundTruthEquippedEntity::CGroundTruthEquippedEntity(CComposableEntity* pc_parent,
                                                                  const std::string& str_id) :
      CEntity(pc_parent, str_id) {
      Disable();
   }

   /****************************************/
   /****************************************/

   CGroundTruthEquippedEntity::~CGroundTruthEquippedEntity() {
      while(! m_tSensors.empty()) {
         delete m_tSensors.back();
         m_tSensors.pop_back();
      }
   }

   /****************************************/
   /****************************************/

   void CGroundTruthEquippedEntity::Init(TConfigurationNode& t_tree) {
      try {
         /*
          * Parse basic entity stuff
          */
         CEntity::Init(t_tree);
         /*
          * Parse ground truth sensors
          */
         /* Not adding any sensor is a fatal error */
         if(t_tree.NoChildren()) {
            THROW_ARGOSEXCEPTION("No sensors defined");
         }
         /* Go through children */
         TConfigurationNodeIterator it;
         for(it = it.begin(&t_tree); it != it.end(); ++it) {
            std::string strAnchorId;
            GetNodeAttribute(*it, "anchor", strAnchorId);
            /*
             * NOTE: here we get a reference to the embodied entity
             * This line works under the assumption that:
             * 1. the GroundTruthEquippedEntity has a parent;
             * 2. the parent has a child whose id is "body"
             * 3. the "body" is an embodied entity
             * If any of the above is false, this line will bomb out.
             */
            CEmbodiedEntity& cBody = GetParent().GetComponent<CEmbodiedEntity>("body");
            if(it->Value() == "sensor") {
               CVector2 cOffset;
               GetNodeAttribute(*it, "offset", cOffset);
               AddSensor(cOffset, cBody.GetAnchor(strAnchorId));
            }
            else {
               THROW_ARGOSEXCEPTION("Unrecognized tag \"" << it->Value() << "\"");
            }
         }
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Initialization error in ground sensor equipped entity", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CGroundTruthEquippedEntity::Enable() {
      CEntity::Enable();
      for(size_t i = 0; i < m_tSensors.size(); ++i) {
         m_tSensors[i]->Anchor.Enable();
      }
   }

   /****************************************/
   /****************************************/

   void CGroundTruthEquippedEntity::Disable() {
      CEntity::Disable();
      for(size_t i = 0; i < m_tSensors.size(); ++i) {
         m_tSensors[i]->Anchor.Disable();
      }
   }

   /****************************************/
   /****************************************/

   void CGroundTruthEquippedEntity::AddSensor(const CVector2& c_offset,
                                               SAnchor& s_anchor) {
      m_tSensors.push_back(new SSensor(c_offset, s_anchor));
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_SPACE_OPERATIONS_ON_ENTITY(CGroundTruthEquippedEntity);
   
   /****************************************/
   /****************************************/

}
/**
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 * @author Yi Gu - <guyi@microsoft.com>
 * @author Xinyi Joffre - <xiou@microsoft.com>
 */

#ifndef DYNAMICS2D_TURTLEBOT_MODEL_H
#define DYNAMICS2D_TURTLEBOT_MODEL_H

namespace argos {
   class CDynamics2DDifferentialSteeringControl;
   class CDynamics2DGrippable;
}

#include <turtlebot/simulator/entities/turtlebot_entity.h>
#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_multi_body_object_model.h>
#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_differentialsteering_control.h>

namespace argos {

   class CDynamics2DTurtleBotModel : public CDynamics2DMultiBodyObjectModel {

   public:

      CDynamics2DTurtleBotModel(CDynamics2DEngine& c_engine,
                              CTurtleBotEntity& c_entity);

      virtual ~CDynamics2DTurtleBotModel();
      
      virtual void MoveTo(const CVector3& c_position,
                          const CQuaternion& c_orientation);

      virtual void Reset();

      virtual void CalculateBoundingBox();

      virtual void UpdateFromEntityStatus();

      void UpdateOriginAnchor(SAnchor& s_anchor);

      inline cpBody* GetActualBaseBody() {
         return m_ptActualBaseBody;
      }

      inline const cpBody* GetActualBaseBody() const {
         return m_ptActualBaseBody;
      }

   private:

      CTurtleBotEntity&         m_cTurtleBotEntity;
      CWheeledEntity&           m_cWheeledEntity;

      CDynamics2DDifferentialSteeringControl m_cDiffSteering;
      
      cpFloat  m_fMass;
      cpShape* m_ptBaseShape;
      cpBody*  m_ptActualBaseBody;

      const Real* m_fCurrentWheelVelocity;
   };

}

#endif

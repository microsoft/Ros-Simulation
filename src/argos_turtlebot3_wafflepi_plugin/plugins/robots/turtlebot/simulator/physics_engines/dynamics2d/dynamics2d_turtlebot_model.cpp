/**
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 * @author Yi Gu - <guyi@microsoft.com>
 * @author Xinyi Joffre - <xiou@microsoft.com>
 */

#include "dynamics2d_turtlebot_model.h"
#include <turtlebot/simulator/turtlebot_measures.h>
#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_gripping.h>
#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_engine.h>

namespace argos {

   /****************************************/
   /****************************************/

   enum TURTLEBOT_WHEELS {
      TURTLEBOT_LEFT_WHEEL = 0,
      TURTLEBOT_RIGHT_WHEEL = 1
   };

   /****************************************/
   /****************************************/

   CDynamics2DTurtleBotModel::CDynamics2DTurtleBotModel(CDynamics2DEngine& c_engine,
                                                    CTurtleBotEntity& c_entity) :
      CDynamics2DMultiBodyObjectModel(c_engine, c_entity),
      m_cTurtleBotEntity(c_entity),
      m_cWheeledEntity(m_cTurtleBotEntity.GetWheeledEntity()),
      m_cDiffSteering(c_engine,
                      TURTLEBOT_MAX_FORCE,
                      TURTLEBOT_MAX_TORQUE,
                      TURTLEBOT_WHEEL_DISTANCE,
                      c_entity.GetConfigurationNode()),
      m_fMass(TURTLEBOT_MASS),
      m_fCurrentWheelVelocity(m_cWheeledEntity.GetWheelVelocities()) 
      {
            RegisterAnchorMethod<CDynamics2DTurtleBotModel>(
            GetEmbodiedEntity().GetOriginAnchor(),
            &CDynamics2DTurtleBotModel::UpdateOriginAnchor);
            /* Create the actual body with initial position and orientation */
            m_ptActualBaseBody =
            cpSpaceAddBody(GetDynamics2DEngine().GetPhysicsSpace(),
                              cpBodyNew(m_fMass,
                                    cpMomentForBox(m_fMass,TURTLEBOT_BASE_LENGTH, TURTLEBOT_BASE_WIDTH)));
            const CVector3& cPosition = GetEmbodiedEntity().GetOriginAnchor().Position;
            m_ptActualBaseBody->p = cpv(cPosition.GetX(), cPosition.GetY());
      
            CRadians cXAngle, cYAngle, cZAngle;
            GetEmbodiedEntity().GetOriginAnchor().Orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);
            cpBodySetAngle(m_ptActualBaseBody, cZAngle.GetValue());
            /* Create the actual body shape */
            m_ptBaseShape =
            cpSpaceAddShape(GetDynamics2DEngine().GetPhysicsSpace(),
                              cpBoxShapeNew(m_ptActualBaseBody,
                                                TURTLEBOT_BASE_LENGTH,
                                                TURTLEBOT_BASE_WIDTH));
            m_ptBaseShape->e = 0.0; // No elasticity
            m_ptBaseShape->u = 0.7; // Lots of friction
            /* Constrain the actual base body to follow the diff steering control */
            m_cDiffSteering.AttachTo(m_ptActualBaseBody);

            /* Add the body so that the default methods work as expected */
            AddBody(m_ptActualBaseBody, cpvzero, 0, TURTLEBOT_OVERALL_HEIGHT);
   }

   /****************************************/
   /****************************************/

   CDynamics2DTurtleBotModel::~CDynamics2DTurtleBotModel() {
      m_cDiffSteering.Detach();
   }

   /****************************************/
   /****************************************/

   void CDynamics2DTurtleBotModel::MoveTo(const CVector3& c_position,
                                        const CQuaternion& c_orientation) {
      /* Move robot */
      CDynamics2DMultiBodyObjectModel::MoveTo(c_position,
                                              c_orientation);
   }

   /****************************************/
   /****************************************/

   void CDynamics2DTurtleBotModel::Reset() {
      /* Zero speed and applied forces of base control body */
      m_cDiffSteering.Reset();
      /* Reset the rest */
      CDynamics2DMultiBodyObjectModel::Reset();
   }

   /****************************************/
   /****************************************/

   void CDynamics2DTurtleBotModel::CalculateBoundingBox() {
      GetBoundingBox().MinCorner.SetX(m_ptBaseShape->bb.l);
      GetBoundingBox().MinCorner.SetY(m_ptBaseShape->bb.b);
      GetBoundingBox().MinCorner.SetZ(GetDynamics2DEngine().GetElevation());
      GetBoundingBox().MaxCorner.SetX(m_ptBaseShape->bb.r);
      GetBoundingBox().MaxCorner.SetY(m_ptBaseShape->bb.t);
      GetBoundingBox().MaxCorner.SetZ(GetDynamics2DEngine().GetElevation() + TURTLEBOT_OVERALL_HEIGHT);
   }

   /****************************************/
   /****************************************/

   void CDynamics2DTurtleBotModel::UpdateFromEntityStatus() {
      
      /* Do we want to move? */
      if((m_fCurrentWheelVelocity[TURTLEBOT_LEFT_WHEEL] != 0.0f) ||
         (m_fCurrentWheelVelocity[TURTLEBOT_RIGHT_WHEEL] != 0.0f)) {
         m_cDiffSteering.SetWheelVelocity(m_fCurrentWheelVelocity[TURTLEBOT_LEFT_WHEEL],
                                          m_fCurrentWheelVelocity[TURTLEBOT_RIGHT_WHEEL]);
      }
      else {
         /* No, we don't want to move - zero all speeds */
         m_cDiffSteering.Reset();
      }
   }

   /****************************************/
   /****************************************/

   void CDynamics2DTurtleBotModel::UpdateOriginAnchor(SAnchor& s_anchor) {
      s_anchor.Position.SetX(m_ptActualBaseBody->p.x);
      s_anchor.Position.SetY(m_ptActualBaseBody->p.y);
      s_anchor.Orientation.FromAngleAxis(CRadians(m_ptActualBaseBody->a), CVector3::Z);
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_DYNAMICS2D_OPERATIONS_ON_ENTITY(CTurtleBotEntity, CDynamics2DTurtleBotModel);

   /****************************************/
   /****************************************/

}

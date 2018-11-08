/**
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 * @author Yi Gu - <guyi@microsoft.com>
 * @author Xinyi Joffre - <xiou@microsoft.com>
 */

#include "turtlebot_entity.h"

#include <argos3/core/utility/math/matrix/rotationmatrix3.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/entity/controllable_entity.h>
#include <argos3/core/simulator/entity/embodied_entity.h>

#include <robots/generic/simulator/sensors/robot_lidar_default_sensor.h>
#include <turtlebot/simulator/turtlebot_measures.h>

namespace argos
{

/****************************************/
/****************************************/

CTurtleBotEntity::CTurtleBotEntity() : CComposableEntity(NULL),
                                       m_pcControllableEntity(NULL),
                                       m_pcEmbodiedEntity(NULL),
                                       m_pcLIDARSensorEquippedEntity(NULL),
                                       m_pcGroundTruthSensorEquippedEntity(NULL),
                                       m_pcWheeledEntity(NULL)
{
}

/****************************************/
/****************************************/

CTurtleBotEntity::CTurtleBotEntity(const std::string &str_id,
                                   const std::string &str_controller_id,
                                   const CVector3 &c_position,
                                   const CQuaternion &c_orientation) : CComposableEntity(NULL, str_id),
                                                                       m_pcControllableEntity(NULL),
                                                                       m_pcEmbodiedEntity(NULL),
                                                                       m_pcLIDARSensorEquippedEntity(NULL),
                                                                       m_pcGroundTruthSensorEquippedEntity(NULL),
                                                                       m_pcWheeledEntity(NULL)
{
    try
    {
        /*
          * Create and init components
          */
        m_pcEmbodiedEntity = new CEmbodiedEntity(this, "body_0", c_position, c_orientation);
        AddComponent(*m_pcEmbodiedEntity);

        /* Wheeled entity and wheel positions (left, right) */
        m_pcWheeledEntity = new CWheeledEntity(this, "wheels_0", 2);
        AddComponent(*m_pcWheeledEntity);
        m_pcWheeledEntity->SetWheel(0, CVector3(0.0f, TURTLEBOT_HALF_WHEEL_DISTANCE, 0.0f), TURTLEBOT_WHEEL_RADIUS);
        m_pcWheeledEntity->SetWheel(1, CVector3(0.0f, -TURTLEBOT_HALF_WHEEL_DISTANCE, 0.0f), TURTLEBOT_WHEEL_RADIUS);

        /* Ground Truth equipped entity */
        m_pcGroundTruthSensorEquippedEntity =
            new CGroundTruthEquippedEntity(this, "ground_truth");
        AddComponent(*m_pcGroundTruthSensorEquippedEntity);

        /* LIDAR sensor equipped entity */
        m_pcLIDARSensorEquippedEntity =
            new CLIDARSensorEquippedEntity(this,
                                           "lidar");
        AddComponent(*m_pcLIDARSensorEquippedEntity);

        m_pcLIDARSensorEquippedEntity->AddSensorFan(
            CVector3(TURTLEBOT_LIDAR_FORWARD_OFFSET, TURTLEBOT_LIDAR_LEFT_OFFSET, TURTLEBOT_LIDAR_ELEVATION),
            0,                                                                                  //radius (set to 0, since tf only understands mount at single point)
            -TURTLEBOT_LIDAR_ANGLE_SPAN * 0.5,                                                  //start angle
            TURTLEBOT_LIDAR_ANGLE_SPAN * 0.5,                                                   //end angle
            TURTLEBOT_LIDAR_RADIUS + TURTLEBOT_LIDAR_SENSORS_RING_RANGE.GetMax(),               //range
            360,                                                                                //num sensors
            m_pcEmbodiedEntity->GetOriginAnchor()                                               //anchor
        );

        /* Controllable entity
            It must be the last one, for actuators/sensors to link to composing entities correctly */
        m_pcControllableEntity = new CControllableEntity(this, "controller_0");
        AddComponent(*m_pcControllableEntity);
        m_pcControllableEntity->SetController(str_controller_id);
        /* Update components */
        UpdateComponents();
    }
    catch (CARGoSException &ex)
    {
        THROW_ARGOSEXCEPTION_NESTED("Failed to initialize entity \"" << GetId() << "\".", ex);
    }
}

/****************************************/
/****************************************/

void CTurtleBotEntity::Init(TConfigurationNode &t_tree)
{
    try
    {
        /*
          * Init parent
          */
        CComposableEntity::Init(t_tree);
        /*
          * Create and init components
          */
        m_pcEmbodiedEntity = new CEmbodiedEntity(this);
        AddComponent(*m_pcEmbodiedEntity);
        m_pcEmbodiedEntity->Init(GetNode(t_tree, "body"));

        /* Wheeled entity and wheel positions (left, right) */
        m_pcWheeledEntity = new CWheeledEntity(this, "wheels_0", 2);
        AddComponent(*m_pcWheeledEntity);
        m_pcWheeledEntity->SetWheel(0, CVector3(0.0f, TURTLEBOT_HALF_WHEEL_DISTANCE, 0.0f), TURTLEBOT_WHEEL_RADIUS);
        m_pcWheeledEntity->SetWheel(1, CVector3(0.0f, -TURTLEBOT_HALF_WHEEL_DISTANCE, 0.0f), TURTLEBOT_WHEEL_RADIUS);

        /* Ground Truth equipped entity */
        m_pcGroundTruthSensorEquippedEntity =
            new CGroundTruthEquippedEntity(this, "ground_truth");
        AddComponent(*m_pcGroundTruthSensorEquippedEntity);

        /* LIDAR sensor equipped entity */
        m_pcLIDARSensorEquippedEntity =
            new CLIDARSensorEquippedEntity(this,
                                           "lidar");
        AddComponent(*m_pcLIDARSensorEquippedEntity);
        
        m_pcLIDARSensorEquippedEntity->AddSensorFan(
            CVector3(TURTLEBOT_LIDAR_FORWARD_OFFSET, TURTLEBOT_LIDAR_LEFT_OFFSET, TURTLEBOT_LIDAR_ELEVATION),
            0,                                                                                  //radius (set to 0, since tf only understands mount at single point)
            -TURTLEBOT_LIDAR_ANGLE_SPAN * 0.5,                                                  //start angle
            TURTLEBOT_LIDAR_ANGLE_SPAN * 0.5,                                                   //end angle
            TURTLEBOT_LIDAR_RADIUS + TURTLEBOT_LIDAR_SENSORS_RING_RANGE.GetMax(),               //range
            360,                                                                                //num sensors
            m_pcEmbodiedEntity->GetOriginAnchor()                                               //anchor
        );

        /* Controllable entity
            It must be the last one, for actuators/sensors to link to composing entities correctly */
        m_pcControllableEntity = new CControllableEntity(this);
        AddComponent(*m_pcControllableEntity);
        m_pcControllableEntity->Init(GetNode(t_tree, "controller"));
        /* Update components */
        UpdateComponents();
    }
    catch (CARGoSException &ex)
    {
        THROW_ARGOSEXCEPTION_NESTED("Failed to initialize entity \"" << GetId() << "\".", ex);
    }
}

/****************************************/
/****************************************/

void CTurtleBotEntity::Reset()
{
    /* Reset all components */
    CComposableEntity::Reset();
    /* Update components */
    UpdateComponents();
}

/****************************************/
/****************************************/

#define UPDATE(COMPONENT)       \
    if (COMPONENT->IsEnabled()) \
        COMPONENT->Update();

void CTurtleBotEntity::UpdateComponents()
{
    /* Update only the components that might change */
}

/****************************************/
/****************************************/

REGISTER_ENTITY(CTurtleBotEntity,
                "turtlebot",
                "Xinyi Joffre [xiou@microsoft.com]",
                "1.0",
                "The turtle-bot 3 waffle robot.",
                "The turtle-bot 3 waffle is a wheeled robot. It is a\n"
                "modular robot with a rich set of sensors and actuators. For more information,\n"
                "refer to the dedicated web page\n"
                "(https://www.roscomponents.com/en/mobile-robots/215-turtlebot-3-waffle.html).\n\n"
                "REQUIRED XML CONFIGURATION\n\n"
                "  <arena ...>\n"
                "    ...\n"
                "    <turtlebot id=\"fb0\">\n"
                "      <body position=\"0.4,2.3,0.25\" orientation=\"45,0,0\" />\n"
                "      <controller config=\"mycntrl\" />\n"
                "    </turtlebot>\n"
                "    ...\n"
                "  </arena>\n\n",
                "Under development");

/****************************************/
/****************************************/

REGISTER_STANDARD_SPACE_OPERATIONS_ON_COMPOSABLE(CTurtleBotEntity);

/****************************************/
/****************************************/

} // namespace argos

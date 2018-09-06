/**
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 * @author Xinyi Joffre - <xiou@microsoft.com>
 * @author Yi Gu <guyi@microsoft.com>
 */

#ifndef ROBOT_GROUND_TRUTH_DEFAULT_SENSOR_H
#define ROBOT_GROUND_TRUTH_DEFAULT_SENSOR_H

#include <string>
#include <map>

namespace argos {
   class CRobotGroundTruthDefaultSensor;
}

#include <argos3/core/simulator/sensor.h>
#include <robots/generic/control_interface/ci_robot_ground_truth_sensor.h>
#include <simulator/entities/ground_truth_equipped_entity.h>

namespace argos {

   class CRobotGroundTruthDefaultSensor : public CCI_RobotGroundTruthSensor,
                                        public CSimulatedSensor {

   public:

      CRobotGroundTruthDefaultSensor();

      virtual ~CRobotGroundTruthDefaultSensor();

      virtual void SetRobot(CComposableEntity& c_entity);

      virtual void Init(TConfigurationNode& t_tree);

      virtual void Update();

      virtual void Reset();

   private:

      /** Reference to embodied entity associated to this sensor */
      CEmbodiedEntity* m_pcEmbodiedEntity;

      /** Reference to ground truth sensor equipped entity associated to this sensor */
      CGroundTruthEquippedEntity* m_pcGroundTruthEntity;

      /** Reference to controllable entity associated to this sensor */
      CControllableEntity* m_pcControllableEntity;

      /** Random number generator */
      CRandom::CRNG* m_pcRNG;

      /** Whether to add noise or not */
      bool m_bAddNoise;

      /** Noise range */
      CRange<Real> m_cNoiseRange;

      /** Reference to the space */
      CSpace& m_cSpace;
   };

}

#endif

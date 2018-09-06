/**
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 * @author Xinyi Joffre - <xiou@microsoft.com>
 * @author Yi Gu <guyi@microsoft.com>
 */

#ifndef ROBOT_LIDAR_DEFAULT_SENSOR_H
#define ROBOT_LIDAR_DEFAULT_SENSOR_H

#include <string>
#include <map>

#include <limits.h>

namespace argos {
   class CRobotLIDARDefaultSensor;
}

#include <argos3/core/utility/math/rng.h>
#include <argos3/core/simulator/sensor.h>
#include <robots/generic/control_interface/ci_robot_lidar_sensor.h>
#include <simulator/entities/lidar_sensor_equipped_entity.h>

namespace argos {

   class CRobotLIDARDefaultSensor : public CCI_RobotLIDARSensor,
                                        public CSimulatedSensor {

   public:

      CRobotLIDARDefaultSensor();

      virtual ~CRobotLIDARDefaultSensor();

      virtual void SetRobot(CComposableEntity& c_entity);

      virtual void Init(TConfigurationNode& t_tree);

      virtual void Update();

      virtual void Reset();

      virtual void Destroy();

      virtual Real GetReading(UInt32 un_idx) const;

      virtual size_t GetNumReadings() const;

      virtual void PowerOn();

      virtual void PowerOff();

      virtual void LaserOn();

      virtual void LaserOff();

   private:

      /** Readings of the LIDAR sensor */
      //long int* m_pnReadings;
      Real* m_pnReadings;

      /** Number of readings of the LIDAR sensor */
      size_t m_unNumReadings;

      /** Power and Laser states */
      UInt8 m_unPowerLaserState;

      /** Reference to embodied entity associated to this sensor */
      CEmbodiedEntity* m_pcEmbodiedEntity;

      /** Reference to lidar sensor equipped entity associated to this sensor */
      CLIDARSensorEquippedEntity* m_pcLIDARSensorEquippedEntity;

      /** Reference to controllable entity associated to this sensor */
      CControllableEntity* m_pcControllableEntity;

      /** Flag to show rays in the simulator */
      bool m_bShowRays;

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

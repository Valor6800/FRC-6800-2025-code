/*                                 Valor 6800                                 */
/* Copyright (c) 2025 Company Name. All Rights Reserved.                      */

#pragma once

#include <grpl/LaserCan.h>

#include <frc/TimedRobot.h>
#include <units/length.h>

#include "valkyrie/sensors/LaserProximitySensor.h"

namespace valor {

/**
 * @brief Specific implementation of the Lidar Sensor for Grapple Robotics
 * LidarCAN device
 */
class GrappleSensor : public LaserProximitySensor<units::millimeter_t> {
public:
  GrappleSensor(frc::TimedRobot *_robot, const char *name, int canId);

private:
  grpl::LaserCan *device;
};
} // namespace valor

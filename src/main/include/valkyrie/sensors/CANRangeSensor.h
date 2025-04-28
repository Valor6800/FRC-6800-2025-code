/*                                 Valor 6800                                 */
/* Copyright (c) 2025 Company Name. All Rights Reserved.                      */

#pragma once

#include <string>

#include <ctre/phoenix6/CANrange.hpp>
#include <frc/TimedRobot.h>
#include <units/length.h>

#include "Constants.h"
#include "LaserProximitySensor.h"

namespace valor {

class CANrangeSensor : public LaserProximitySensor<units::millimeter_t> {
public:
  CANrangeSensor(frc::TimedRobot *_robot, const char *name, int deviceId,
                 std::string canbus, units::millimeter_t defaultDistance);
  bool isConnected();
  units::millimeter_t getDefaultDistance();

private:
  ctre::phoenix6::hardware::CANrange *device;
  bool isFaulting();
  units::millimeter_t defaultDistance;
};

} // namespace valor

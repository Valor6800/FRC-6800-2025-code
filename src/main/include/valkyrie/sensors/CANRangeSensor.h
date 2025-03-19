#pragma once

#include <frc/TimedRobot.h>
#include <units/length.h>
#include <ctre/phoenix6/CANrange.hpp>
#include <string>
#include "LaserProximitySensor.h"
#include "Constants.h"

namespace valor {

class CANrangeSensor : public LaserProximitySensor<units::millimeter_t>
{
public:
    CANrangeSensor(frc::TimedRobot *_robot, const char *name, int deviceId, std::string canbus);

private:
    ctre::phoenix6::hardware::CANrange *device;
    bool isFaulting();
};

}
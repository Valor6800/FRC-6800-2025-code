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
    CANrangeSensor(frc::TimedRobot *_robot, const char *name, int deviceId, std::string canbus, units::millimeter_t defaultDistance);
    bool isConnected();
    units::millimeter_t getDefaultDistance();

private:
    ctre::phoenix6::hardware::CANrange *device;
    bool isFaulting();
    units::millimeter_t defaultDistance;

};

}
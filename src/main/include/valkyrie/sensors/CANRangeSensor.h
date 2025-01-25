#pragma once

#include "valkyrie/sensors/BaseSensor.h"
#include <frc/TimedRobot.h>
#include <units/length.h>
#include <ctre/phoenix6/CANrange.hpp>
#include <string>

namespace valor {

class CANrangeSensor : public BaseSensor<units::length::meter_t>
{
public:
    CANrangeSensor(frc::TimedRobot *_robot, const char *name, int deviceId, std::string canbus);

    void reset();

    units::length::meter_t getCANrangeDistance();

    void InitSendable(wpi::SendableBuilder &builder) override;

private:
    ctre::phoenix6::hardware::CANrange *device;

    void calculate();

    bool isFaulting();
};

}
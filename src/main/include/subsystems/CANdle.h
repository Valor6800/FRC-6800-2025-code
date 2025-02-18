#pragma once
#include "valkyrie/BaseSubsystem.h"
#include "valkyrie/sensors/CANdleSensor.h"
#include <ctre/phoenix6/CANcoder.hpp>

class CANdle : valor::BaseSubsystem {
public:
    CANdle(frc::TimedRobot*);

    void analyzeDashboard() override;
    void assignOutputs() override;
    void InitSendable(wpi::SendableBuilder&);

    std::vector<ctre::phoenix6::hardware::CANcoder*> cancoders;

private:
    std::vector<int64_t> colors{8};

    valor::CANdleSensor candle;
};
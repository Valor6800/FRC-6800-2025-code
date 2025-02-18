#include "subsystems/CANdle.h"
#include "Constants.h"
using namespace ctre::phoenix6::signals;

#define LED_COUNT 86
#define SEGMENTS 2

CANdle::CANdle(frc::TimedRobot *robot) :
    valor::BaseSubsystem{robot, "CANdle"},
    candle{robot, LED_COUNT, SEGMENTS, CANIDs::CANDLE} {}

void CANdle::analyzeDashboard() {}

void CANdle::assignOutputs() {
    for (size_t i = 0; i < getters.size(); i++)
        if (getters[i]) candle.setLED(i, getters[i]());
}

void CANdle::InitSendable(wpi::SendableBuilder& builder) {
    builder.SetSmartDashboardType("Subsystem");
}

int CANdle::cancoderMagnetHealthGetter(ctre::phoenix6::hardware::CANcoder& cancoder) {
    using namespace ctre::phoenix6::signals;

    auto value = cancoder.GetMagnetHealth().GetValue();
    if (value == MagnetHealthValue::Magnet_Green) return valor::CANdleSensor::GREEN;
    else if (value == MagnetHealthValue::Magnet_Orange) return valor::CANdleSensor::ORANGE;
    else if (value == MagnetHealthValue::Magnet_Red) return valor::CANdleSensor::RED;
    return valor::CANdleSensor::WHITE;
}
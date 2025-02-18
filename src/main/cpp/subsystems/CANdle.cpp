#include "subsystems/CANdle.h"
#include "Constants.h"
using namespace ctre::phoenix6::signals;

#define LED_COUNT 86
#define SEGMENTS 2

CANdle::CANdle(frc::TimedRobot *robot) :
    valor::BaseSubsystem{robot, "CANdle"},
    candle{robot, LED_COUNT, SEGMENTS, CANIDs::CANDLE} {}

void CANdle::analyzeDashboard() {
    unsigned int i = 0;
    for (; i < cancoders.size(); i++) {
        int magnetHealth = cancoders[i]->GetMagnetHealth().GetValue().value;
        if (magnetHealth == MagnetHealthValue::Magnet_Green) colors[i] = valor::CANdleSensor::GREEN;
        else if (magnetHealth == MagnetHealthValue::Magnet_Orange) colors[i] = valor::CANdleSensor::ORANGE;
        else if (magnetHealth == MagnetHealthValue::Magnet_Red) colors[i] = valor::CANdleSensor::RED;
        else colors[i] = valor::CANdleSensor::WHITE;
    }

    for (; i < 8; i++) colors[i] = valor::CANdleSensor::WHITE;
}

void CANdle::assignOutputs() {
    for (int i = 0; i < 8; i++) candle.setLED(i, colors[i]);
}

void CANdle::InitSendable(wpi::SendableBuilder& builder) {
    builder.SetSmartDashboardType("Subsystem");
    builder.AddIntegerArrayProperty(
        "LED Colors",
        [this] { return colors; },
        nullptr
    );
}
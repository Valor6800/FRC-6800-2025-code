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

    // Some common and helpful getters
    static int cancoderMagnetHealthGetter(ctre::phoenix6::hardware::CANcoder&);

    valor::CANdleSensor::RGBColor toRGB(int color) {
        return candle.toRGB(color);
    }

    void setColor(int segment, valor::CANdleSensor::RGBColor rgb) {
        return candle.setColor(segment, rgb);
    }

    void setColor(int segment, int color) {
        return candle.setColor(segment, toRGB(color));
    }

    wpi::array<std::function<int()>, 8> getters{wpi::empty_array};

private:
    valor::CANdleSensor candle;
};
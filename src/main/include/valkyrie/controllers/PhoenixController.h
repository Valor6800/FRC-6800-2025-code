#pragma once

#include "valkyrie/controllers/BaseController.h"

#include <ctre/phoenix6/TalonFX.hpp>
#include <string>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/CANBus.hpp>

const units::revolutions_per_minute_t FREE_SPD_KRAKEN(5800);
const units::revolutions_per_minute_t FREE_SPD_KRAKEN_FOC(6000);
const units::revolutions_per_minute_t FREE_SPD_FALCON(6380);

namespace valor {

enum PhoenixControllerType {
    KRAKEN_FOC,
    KRAKEN,
    FALCON_FOC,
    FALCON
}; 

class PhoenixController : public BaseController<ctre::phoenix6::hardware::TalonFX>
{
public:
    PhoenixController(valor::PhoenixControllerType, int _canID, valor::NeutralMode _mode, bool _inverted, std::string _canbus = "");
    PhoenixController(valor::PhoenixControllerType, int _canID, valor::NeutralMode _mode, bool _inverted, double rotorToSensor, double sensorToMech, valor::PIDF pidf, std::string _canbus = "");

    static units::revolutions_per_minute_t getPhoenixControllerMotorSpeed(PhoenixControllerType controllerType)
    {
        switch (controllerType) {
            case KRAKEN_FOC:
                return FREE_SPD_KRAKEN;
            case KRAKEN:
                return FREE_SPD_KRAKEN_FOC;
            default:
                return FREE_SPD_FALCON;
        }
    }

    void init(double rotorToSensor, double sensorToMech, valor::PIDF pidf);
    void init() override;

    void enableFOC(bool enableFOC);
    void applyConfig() override;

    void reset() override;
    void setNeutralMode(ctre::phoenix6::configs::MotorOutputConfigs& config, valor::NeutralMode mode);
    void setNeutralMode(valor::NeutralMode mode) override;

    units::turn_t getPosition() override;
    units::turns_per_second_t getSpeed() override;
    units::ampere_t getCurrent() override;

    void setPositionUpdateFrequency(units::hertz_t);
    void setSpeedUpdateFrequency(units::hertz_t);

    void setEncoderPosition(units::turn_t position) override;
    
    void setPosition(units::turn_t) override;
    void setSpeed(units::turns_per_second_t) override;
    void setPower(units::volt_t) override;

    void setupFollower(int, bool = false) override;
    
    void setPIDF(valor::PIDF pidf, int slot) override;
    void setPIDF(ctre::phoenix6::configs::Slot0Configs&, ctre::phoenix6::configs::MotionMagicConfigs&, valor::PIDF pidf);

    void setForwardLimit(units::turn_t forward) override;
    void setReverseLimit(units::turn_t reverse) override;
    
    void setGearRatios(double, double) override;
    void setGearRatios(ctre::phoenix6::configs::FeedbackConfigs&, double, double);

    void setProfile(int slot) override;

    units::turn_t getAbsEncoderPosition();

    void setupCANCoder(int deviceId, units::turn_t offset, bool clockwise, std::string canbus = "", ctre::phoenix6::signals::AbsoluteSensorRangeValue absoluteRange=ctre::phoenix6::signals::AbsoluteSensorRangeValue::Unsigned_0To1) override;
    units::turn_t getCANCoder() override;

    float getRevBusUtil();
    float getCANivoreBusUtil();
    ctre::phoenix6::signals::MagnetHealthValue getMagnetHealth();
    
    void setOpenLoopRamp(units::second_t time) override;

    void InitSendable(wpi::SendableBuilder& builder) override;

private:
    valor::PIDF pidf;
    int currentProfile;

    ctre::phoenix6::controls::MotionMagicVoltage req_position;
    ctre::phoenix6::controls::VelocityVoltage req_velocity;
    ctre::phoenix6::controls::VoltageOut req_voltage;

    ctre::phoenix6::hardware::CANcoder *cancoder;

    ctre::phoenix6::StatusSignal<units::turn_t>& res_position;
    ctre::phoenix6::StatusSignal<units::turns_per_second_t>& res_velocity;
};
}

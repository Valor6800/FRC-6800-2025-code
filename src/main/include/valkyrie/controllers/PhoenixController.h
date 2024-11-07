#pragma once

#include "valkyrie/controllers/BaseController.h"
#include <ctre/phoenix6/TalonFX.hpp>
#include <string>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/CANBus.hpp>

const units::revolutions_per_minute_t FREE_SPD_KRAKEN_X60(6000);
const units::revolutions_per_minute_t FREE_SPD_KRAKEN_X60_FOC(5800);
const units::revolutions_per_minute_t FREE_SPD_KRAKEN_X44(7530);
const units::revolutions_per_minute_t FREE_SPD_KRAKEN_X44_FOC(7530);
const units::revolutions_per_minute_t FREE_SPD_FALCON(6380);
const units::revolutions_per_minute_t FREE_SPD_FALCON_FOC(6080);

namespace valor {

enum PhoenixControllerType {
    KRAKEN_X60_FOC,
    KRAKEN_X60,
    KRAKEN_X44_FOC,
    KRAKEN_X44,
    FALCON_FOC,
    FALCON
}; 

class PhoenixController : public BaseController<ctre::phoenix6::hardware::TalonFX>
{
public:
    PhoenixController(valor::PhoenixControllerType, int _canID, valor::NeutralMode _mode, bool _inverted, std::string _canbus = "");

    static units::revolutions_per_minute_t getPhoenixControllerMotorSpeed(PhoenixControllerType controllerType)
    {
        switch (controllerType) {
            case KRAKEN_X60_FOC:
                return FREE_SPD_KRAKEN_X60_FOC;
            case KRAKEN_X60:
                return FREE_SPD_KRAKEN_X60;
            case KRAKEN_X44_FOC:
                return FREE_SPD_KRAKEN_X44_FOC;
            case KRAKEN_X44:
                return FREE_SPD_KRAKEN_X44;
            case FALCON_FOC:
                return FREE_SPD_FALCON_FOC;
            case FALCON:
                return FREE_SPD_FALCON;
            default:
                return FREE_SPD_KRAKEN_X60;
        }
    }

    void init() override;

    void enableFOC(bool enableFOC);
    void applyConfig() override;

    void reset() override;
    void setNeutralMode(valor::NeutralMode mode, bool saveImmediately = false) override;
    
    void setCurrentLimits(units::ampere_t statorCurrentLimit, units::ampere_t supplyCurrentLimit, units::ampere_t supplyCurrentThreshold, units::second_t supplyTimeThreshold, bool saveImmediately = false) override;

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
    
    void setPIDF(valor::PIDF pidf, int slot, bool saveImmediately = false) override;

    void setForwardLimit(units::turn_t forward, bool saveImmediately = false) override;
    void setReverseLimit(units::turn_t reverse, bool saveImmediately = false) override;
    
    void setGearRatios(double, double, bool saveImmediately = false) override;

    void setProfile(int slot) override;

    units::turn_t getAbsEncoderPosition();

    void setupCANCoder(int deviceId, units::turn_t offset, bool clockwise, std::string canbus = "", ctre::phoenix6::signals::AbsoluteSensorRangeValue absoluteRange=ctre::phoenix6::signals::AbsoluteSensorRangeValue::Unsigned_0To1, bool saveImmediately = false) override;
    units::turn_t getCANCoder() override;

    float getBusUtil(const char* canBusName);
    ctre::phoenix6::signals::MagnetHealthValue getMagnetHealth();
    
    void setOpenLoopRamp(units::second_t time, bool saveImmediately = false) override;

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

    ctre::phoenix6::configs::TalonFXConfiguration config;
};
}

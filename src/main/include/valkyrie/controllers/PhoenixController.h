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

template <class RawOutput = ctre::phoenix6::controls::VoltageOut, class VelocityOutput = ctre::phoenix6::controls::VelocityVoltage, class PositionOutput = ctre::phoenix6::controls::PositionVoltage>
class PhoenixController : public BaseController<ctre::phoenix6::hardware::TalonFX>
{
    static_assert(std::is_base_of_v<ctre::phoenix6::controls::ControlRequest, RawOutput>);
    static_assert(std::is_base_of_v<ctre::phoenix6::controls::ControlRequest, VelocityOutput>);
    static_assert(std::is_base_of_v<ctre::phoenix6::controls::ControlRequest, PositionOutput>);

public:
    typedef decltype(RawOutput::Output) RawOutputUnit;

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
    void setContinuousWrap(bool, bool saveImmediately = false);
    
    void setPosition(units::turn_t) override;
    void setSpeed(units::turns_per_second_t) override;
    void setPower(RawOutputUnit);

    void setupFollower(int, bool = false) override;
    
    void setPIDF(valor::PIDF pidf, int slot, bool saveImmediately = false) override;

    void setupReverseHardwareLimit(int canID, ctre::phoenix6::signals::ReverseLimitTypeValue type, units::turn_t autosetPosition = 0_tr, bool saveImmediately = false);
    void setupForwardHardwareLimit(int canID, ctre::phoenix6::signals::ForwardLimitTypeValue type, units::turn_t autosetPosition = 0_tr, bool saveImmediately = false);

    void setForwardLimit(units::turn_t forward, bool saveImmediately = false) override;
    void setReverseLimit(units::turn_t reverse, bool saveImmediately = false) override;
    
    void setGearRatios(double, double, bool saveImmediately = false) override;

    void setProfile(int slot) override;

    units::turn_t getAbsEncoderPosition();

    void setupCANCoder(int deviceId, units::turn_t offset, bool clockwise, std::string canbus = "", units::turn_t absoluteRange=1_tr, bool saveImmediately = false) override;
    units::turn_t getCANCoder() override;

    float getBusUtil(const char* canBusName);

    enum MagnetHealth
    {
        RED,
        ORANGE,
        GREEN
    };

    struct x
    {
        MagnetHealth magnetHealth;

    } state;
    
    MagnetHealth getMagnetHealth();

    void setOpenLoopRamp(units::second_t time, bool saveImmediately = false) override;

    units::frequency::hertz_t getPositionUpdateFrequency();
    units::frequency::hertz_t getSpeedUpdateFrequency();

    void InitSendable(wpi::SendableBuilder& builder) override;

private:
    valor::PIDF pidf;
    int currentProfile;

    RawOutput req_raw_out;
    VelocityOutput req_vel_out;
    PositionOutput req_pos_out;

    ctre::phoenix6::hardware::CANcoder *cancoder;

    ctre::phoenix6::StatusSignal<units::turn_t>& res_position;
    ctre::phoenix6::StatusSignal<units::turns_per_second_t>& res_velocity;

    ctre::phoenix6::configs::TalonFXConfiguration config;
};
}

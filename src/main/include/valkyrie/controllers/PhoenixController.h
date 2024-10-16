#pragma once

#include "valkyrie/controllers/BaseController.h"

#include <ctre/phoenix6/TalonFX.hpp>
#include <string>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/CANBus.hpp>

namespace valor {

class PhoenixController : public BaseController<ctre::phoenix6::hardware::TalonFX>
{
public:
    PhoenixController(int _canID, valor::NeutralMode _mode, bool _inverted, std::string _canbus = "");
    PhoenixController(int _canID, valor::NeutralMode _mode, bool _inverted, double rotorToSensor, double sensorToMech, valor::PIDF pidf, double voltageComp, bool isKraken = false, std::string _canbus = "");

    void init(double rotorToSensor, double sensorToMech, valor::PIDF pidf);
    void init();

    void reset();
    void setNeutralMode(ctre::phoenix6::configs::MotorOutputConfigs& config, valor::NeutralMode mode);
    void setNeutralMode(valor::NeutralMode mode);

    double getPosition();
    double getSpeed();
    double getCurrent();

    void setPositionUpdateFrequency(units::frequency::hertz_t);
    void setSpeedUpdateFrequency(units::frequency::hertz_t);

    void setEncoderPosition(double position);
    void setVoltageCompensation(double volts) override;
    
    void setPosition(double);
    void enableFOC(bool enableFOC);
    void setSpeed(double);
    void setPower(double);

    void setupFollower(int, bool = false);
    
    void setPIDF(valor::PIDF pidf, int slot);
    void setPIDF(ctre::phoenix6::configs::Slot0Configs&, ctre::phoenix6::configs::MotionMagicConfigs&, valor::PIDF pidf);

    void setForwardLimit(double forward);
    void setReverseLimit(double reverse);
    void setRange(int slot, double min, double max);
    
    void setConversion(double, double);
    void setConversion(ctre::phoenix6::configs::FeedbackConfigs&, double, double);
    void setProfile(int slot);
    double getAbsEncoderPosition();
    void setupCANCoder(int deviceId, double offset, bool clockwise, std::string canbus = "", ctre::phoenix6::signals::AbsoluteSensorRangeValue absoluteRange=ctre::phoenix6::signals::AbsoluteSensorRangeValue::Unsigned_0To1) override;
    double getCANCoder() override;
    
    void setMotorInversion(bool);
    bool getMotorInversion();

    void setVoltage(units::volt_t);
    units::volt_t getVoltage();


    float getRevBusUtil();
    float getCANivoreBusUtil();
    ctre::phoenix6::signals::MagnetHealthValue getMagnetHealth();
    
    void setOpenLoopRamp(double time);

    void InitSendable(wpi::SendableBuilder& builder);
private:
    valor::PIDF pidf;
    int currentProfile;
        
    ctre::phoenix::StatusCode status;
    ctre::phoenix6::controls::MotionMagicVoltage req_position;
    ctre::phoenix6::controls::VelocityVoltage req_velocity;
    ctre::phoenix6::controls::VoltageOut req_voltage;

    double voltageCompenstation;
    ctre::phoenix6::hardware::CANcoder *cancoder;

    ctre::phoenix6::StatusSignal<units::turn_t>& res_position;
    ctre::phoenix6::StatusSignal<units::turns_per_second_t>& res_velocity;
};
}

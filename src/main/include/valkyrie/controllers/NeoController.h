#pragma once

#include "valkyrie/controllers/BaseController.h"

#include <iostream>

#include <rev/CANSparkMax.h>
#include <rev/CANEncoder.h>
#include <rev/SparkAbsoluteEncoder.h>
#include <string>

namespace valor {

class NeoController : public BaseController<rev::CANSparkMax>
{
public:
    NeoController(int, valor::NeutralMode, bool, std::string canbus = "");

    void init();
    void reset();

    double getCurrent();
    double getPosition();
    double getSpeed();

    int getProfile();

    void setEncoderPosition(double position);
    void setVoltageCompensation(double volts) override;
    void setMaxCurrent(double amps);

    void setPosition(double);
    void setSpeed(double);
    void setPower(double);
    void setVoltage(double);
    
    void setupFollower(int, bool = false);
    
    void setPIDF(valor::PIDF pidf, int slot);
    void setForwardLimit(double forward);
    void setReverseLimit(double reverse);
    void setRange(int slot, double min, double max);

    void setConversion(double, double);

    void setMotorInversion(bool);
    bool getMotorInversion();

    void setProfile(int slot);
    void setNeutralMode(valor::NeutralMode nmode);

    void setOpenLoopRamp(double time);

    double getAbsEncoderPosition() override;
    void setupCANCoder(int deviceId, double offset, bool clockwise, std::string canbus = "", ctre::phoenix6::signals::AbsoluteSensorRangeValue absoluteRange=ctre::phoenix6::signals::AbsoluteSensorRangeValue::Unsigned_0To1) override;
    double getCANCoder() override;
    
    void InitSendable(wpi::SendableBuilder& builder) override;
    
private:
    rev::SparkPIDController pidController;
    rev::SparkRelativeEncoder encoder;
    rev::SparkAbsoluteEncoder extEncoder;

    rev::REVLibError lastError;

    int currentPidSlot;
};
}

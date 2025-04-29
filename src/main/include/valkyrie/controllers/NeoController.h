#pragma once

#include "valkyrie/controllers/BaseController.h"
#include <string>
#include <rev/SparkMax.h>
#include <rev/config/ClosedLoopConfig.h>

const units::revolutions_per_minute_t FREE_SPD_NEO_550(11000);
const units::revolutions_per_minute_t FREE_SPD_NEO(5676);

namespace valor {

enum NeoControllerType {
    NEO,
    NEO_550
}; 

class NeoController : public BaseController<rev::spark::SparkMax>
{
public:
    NeoController(valor::NeoControllerType, int _canID, valor::NeutralMode _mode, bool _inverted, std::string _canbus = "");

    static units::revolutions_per_minute_t getNeoControllerMotorSpeed(NeoControllerType controllerType)
    {
        switch (controllerType) {
            case NEO_550:
                return FREE_SPD_NEO_550;
            default:
                return FREE_SPD_NEO;
        }
    }

    void init() override;

    void applyConfig() override;

    void reset() override;
    void setNeutralMode(valor::NeutralMode mode, bool saveImmediately = false) override;
    
    void setCurrentLimits(units::ampere_t statorCurrentLimit, units::ampere_t supplyCurrentLimit, units::ampere_t supplyCurrentThreshold, units::second_t supplyTimeThreshold, bool saveImmediately = false) override;

    units::turn_t getPosition() override;
    units::turns_per_second_t getSpeed() override;
    units::ampere_t getCurrent() override;

    void setEncoderPosition(units::turn_t position) override;
    
    void setPosition(units::turn_t, int) override;
    void setSpeed(units::turns_per_second_t, int) override;
    void setPower(units::volt_t);

    void setupFollower(int, bool = false) override;
    
    void setPIDF(valor::PIDF pidf, int slot, bool saveImmediately = false) override;

    void setForwardLimit(units::turn_t forward, bool saveImmediately = false) override;
    void setReverseLimit(units::turn_t reverse, bool saveImmediately = false) override;
    
    void setGearRatios(double, double, bool saveImmediately = false) override;

    void setupCANCoder(int deviceId, units::turn_t offset, bool clockwise, std::string canbus = "", units::turn_t absoluteRange=1_tr, bool saveImmediately = false) override;
    ctre::phoenix6::hardware::CANcoder *getCANCoder() override;
    
    void setOpenLoopRamp(units::second_t time, bool saveImmediately = false) override;

    void InitSendable(wpi::SendableBuilder& builder) override;

private:
    valor::PIDF pidf;
    int currentProfile;
    ctre::phoenix6::hardware::CANcoder *cancoder;
    rev::spark::SparkClosedLoopController pidController;
};
}

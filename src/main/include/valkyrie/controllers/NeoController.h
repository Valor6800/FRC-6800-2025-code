#pragma once

#include "valkyrie/controllers/BaseController.h"
#include <string>
#include <rev/SparkMax.h>
#include <rev/AbsoluteEncoder.h>
#include <rev/RelativeEncoder.h>
#include <rev/config/ClosedLoopConfig.h>
#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableHelper.h>
#include <wpi/sendable/SendableBuilder.h>

const units::revolutions_per_minute_t FREE_SPD_NEO_550(11000);
const units::revolutions_per_minute_t FREE_SPD_NEO(5676);

namespace valor {

enum NeoControllerType {
    NEO,
    NEO_550
}; 

class NeoController : public BaseController, rev::spark::SparkMax, public wpi::Sendable, public wpi::SendableHelper<NeoController> // SparkMax does not inherit Sendable/SendableHelper, so we do it ourselves
{
public:
    NeoController(valor::NeoControllerType, int _canID, valor::NeutralMode _mode, bool _inverted, double rotorToSensor, double sensorToMech, std::string _canbus = "");

    static units::revolutions_per_minute_t getNeoControllerMotorSpeed(NeoControllerType controllerType)
    {
        switch (controllerType) {
            case NEO_550:
                return FREE_SPD_NEO_550;
            default:
                return FREE_SPD_NEO;
        }
    }

    void reset() override;

    units::turn_t getPosition() override;
    units::turns_per_second_t getSpeed() override;

    void setEncoderPosition(units::turn_t position) override;
    
    void setPosition(units::turn_t, int) override;
    void setSpeed(units::turns_per_second_t, int) override;


    void setPower(units::volt_t) override;
    [[deprecated("NeoController doesn't support current based power")]]
    void setPower(units::ampere_t) override { assert(false); }
    void setPower(units::scalar_t) override;

    units::volt_t getVoltage() override;
    units::ampere_t getCurrent() override;
    units::scalar_t getDutyCycle() override;

    void InitSendable(wpi::SendableBuilder& builder) override;

private:
    valor::PIDF pidf;
    int currentProfile;
    ctre::phoenix6::hardware::CANcoder *cancoder;
    rev::spark::SparkClosedLoopController pidController;
};
}

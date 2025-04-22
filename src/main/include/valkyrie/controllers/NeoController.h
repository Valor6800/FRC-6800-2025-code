#pragma once
#include <valkyrie/controllers/BaseController.h>
#include <rev/SparkMax.h>
#include <units/angular_velocity.h>

const units::revolutions_per_minute_t FREE_SPD_NEO_550(11000);
const units::revolutions_per_minute_t FREE_SPD_NEO(5676);

namespace valor {

enum NeoControllerType {
    NEO,
    NEO_550
}; 

class NeoController : public BaseController, public wpi::Sendable, public wpi::SendableHelper<NeoController>, rev::spark::SparkMax
{
public:
    NeoController(
        valor::NeoControllerType,
        int canID,
        valor::NeutralMode mode,
        bool inverted,
        double rotorToSensor,
        double sensorToMech
    );

    static units::revolutions_per_minute_t getNeoControllerMotorSpeed(NeoControllerType controllerType)
    {
        switch (controllerType) {
            case NEO_550:
                return FREE_SPD_NEO_550;
            default:
                return FREE_SPD_NEO;
        }
    }

    void applyConfig();

    void reset();

    void setPosition(units::turn_t position, int slot = 0) override;
    void setSpeed(units::turns_per_second_t speed, int slot = 0) override;
    void setPower(units::volt_t voltage) override { SetVoltage(voltage); }
    void setPower(units::scalar_t dutyCycle) override { Set(dutyCycle); }
    [[deprecated("NeoController doesn't support current based power")]]
    void setPower(units::ampere_t) override { assert(false); }

    units::turn_t getPosition() override;
    units::turns_per_second_t getSpeed() override;
    units::ampere_t getCurrent() override { return units::ampere_t{GetOutputCurrent()}; }
    units::volt_t getVoltage() override { return units::volt_t{GetAppliedOutput()}; }
    units::scalar_t getDutyCycle() override { return Get(); }

    void setEncoderPosition(units::turn_t) override;

    void setupFollower(int canID, bool followerInverted = false);
    void setPIDF(valor::PIDF pidf, int slot = 0, bool saveImmediately = false);
    void InitSendable(wpi::SendableBuilder&) override;

    rev::spark::SparkBaseConfig config;

private:
    valor::PIDF pidf;
    std::unique_ptr<rev::spark::SparkMax> followerMotor;
};
}

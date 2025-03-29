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

class NeoController : public BaseController<rev::spark::SparkMax>
{
public:
    NeoController(valor::NeoControllerType, int _canID, valor::NeutralMode _mode, bool _inverted);

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

    void init();

    void reset();

    units::turn_t getPosition();

    units::ampere_t getCurrent();

    units::turns_per_second_t getSpeed();

    void setEncoderPosition(units::turn_t);

    void setPosition(units::turn_t position, int slot = 0);

    void setSpeed(units::turns_per_second_t speed, int slot = 0);

    void setupFollower(int canID, bool followerInverted = false);

    void setPIDF(valor::PIDF pidf, int slot = 0, bool saveImmediately = false);

    void setForwardLimit(units::turn_t, bool saveImmediately = false);

    void setReverseLimit(units::turn_t, bool saveImmediately = false);

    void setGearRatios(double rotorToSensor, double sensorToMech, bool saveImmediately = false);

    void setCurrentLimits(units::ampere_t statorCurrentLimit, units::ampere_t supplyCurrentLimit, units::ampere_t supplyCurrentThreshold, units::second_t supplyTimeThreshold, bool saveImmediately = false);

    void setNeutralMode(valor::NeutralMode mode, bool saveImmediately = false);

    void setOpenLoopRamp(units::second_t time, bool saveImmediately = false);

    void InitSendable(wpi::SendableBuilder&);

    units::turn_t getAbsEncoderPosition();

    void setupCANCoder(int, units::turn_t, bool, std::string = "", units::turn_t = 1_tr, bool = false);
    ctre::phoenix6::hardware::CANcoder *getCANCoder() { return nullptr; }

    void setContinuousWrap(bool continuousWrap, bool saveImmediately = false);

    void setPower(units::volt_t);

private:
    rev::spark::SparkBaseConfig config;
    valor::PIDF pidf;
};
}

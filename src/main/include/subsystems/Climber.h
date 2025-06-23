#pragma once

#include "units/angular_velocity.h"
#include "valkyrie/BaseSubsystem.h"
#include "valkyrie/controllers/PhoenixController.h"
#include "Constants.h"
#include "valkyrie/controllers/PIDF.h"
#include "valkyrie/sensors/CurrentSensor.h"
#include "valkyrie/sensors/CANdleSensor.h"

#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "valkyrie/Gamepad.h"
#include <frc/Alert.h>

class Climber : public valor::BaseSubsystem
{
public:

    Climber(frc::TimedRobot *robot, valor::CANdleSensor* leds);

    ~Climber();

    void init();
    void resetState();
    void assessInputs();
    void analyzeDashboard();
    void assignOutputs();

    frc2::CommandPtr pitSequence();

    void setDegrees(units::degree_t deg);

    void InitSendable(wpi::SendableBuilder& builder);

    enum CLIMB_STATE
    {
        MANUAL,
        STOW,
        DEPLOYED,
        RETRACTED
    };

    enum STABBY_STATE
    {
        NO_CRAB,
        CRABBING
    };

    struct x
    {
        CLIMB_STATE climbState;
        // STABBY_STATE stabState;

        bool hasClimbed;
        double spikeCurrent;
        double cacheSize;
        units::volt_t manualSpeed;
        bool lockout;

    } state;

private:
    frc2::CommandPtr pitSequenceStage(CLIMB_STATE);

    // valor::PhoenixController<> *stabbyMotor;
    valor::PhoenixController<> *climbMotors;
    valor::CurrentSensor currentSensor;
    ctre::phoenix6::hardware::CANcoder* climbCancoder;
    valor::CANdleSensor *leds;

    frc::Alert climberPosSuccess{"Climber position is within tolerance", frc::Alert::AlertType::kInfo};
    frc::Alert climberPosFail{"Climber position not within tolerance", frc::Alert::AlertType::kError};
    // frc::Alert stabbySpeedSuccess{"Stabby speed is within tolerance", frc::Alert::AlertType::kInfo};
    // frc::Alert stabbySpeedFail{"Stabby speed is not within tolerance", frc::Alert::AlertType::kError};
};


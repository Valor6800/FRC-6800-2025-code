#pragma once

#include "units/angular_velocity.h"
#include "valkyrie/BaseSubsystem.h"
#include "valkyrie/controllers/PhoenixController.h"
#include "Constants.h"
#include "valkyrie/controllers/PIDF.h"
#include "valkyrie/sensors/CurrentSensor.h"

#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "valkyrie/Gamepad.h"


class Climber : public valor::BaseSubsystem
{
public:

    Climber(frc::TimedRobot *robot);

    ~Climber();

    void init();
    void resetState();
    void assessInputs();
    void analyzeDashboard();
    void assignOutputs();

    // void setCrabPID();

    void InitSendable(wpi::SendableBuilder& builder);

    enum CLIMB_STATE
    {
        MANUAL,
        STOW,
        DEPLOYED,
        RETRACTED
    };

    enum CRAB_STATE
    {
        NO_CRAB,
        CRABBING,
        CRABBED
    };

    struct x
    {
        CLIMB_STATE climbState;
        CRAB_STATE crabState;

        bool climbed;
        double spikeCurrent;
        double cacheSize;
        units::volt_t manualSpeed;

    }state;

private:
    valor::PhoenixController *climbMotors;
    valor::PhoenixController *crabMotor;

    valor::CurrentSensor currentSensor;
};


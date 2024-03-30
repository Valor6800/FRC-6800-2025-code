#pragma once

#include "units/angular_velocity.h"
#include "valkyrie/BaseSubsystem.h"
#include "valkyrie/controllers/PhoenixController.h"
#include "Constants.h"
#include "valkyrie/controllers/PIDF.h"
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/Command.h>
#include <frc2/command/Commands.h>

#include <frc/PWM.h>
#include <unordered_map>
#include "valkyrie/Gamepad.h"

#include "Drivetrain.h"


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

    frc2::SequentialCommandGroup UpClimb();
    frc2::SequentialCommandGroup DownClimb();

    void InitSendable(wpi::SendableBuilder& builder);

    enum CLIMB_STATE
    {
        MANUAL,
        DISABLE,
        EXTEND,
        RETRACT
    };

    enum LATCH_STATE
    {
        UNLATCH,
        LATCH
    };

    struct x
    {
        CLIMB_STATE climbState;
        LATCH_STATE latchState;

    }state;


private:
    valor::PhoenixController climbMotors;
    frc::PWM *servo;
    frc2::CommandPtr climbUp = frc2::cmd::Sequence();
    frc2::CommandPtr climbDown = frc2::cmd::Sequence();
    Drivetrain *drive;
};
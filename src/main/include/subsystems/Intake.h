#pragma once

#include "units/angular_velocity.h"
#include "valkyrie/BaseSubsystem.h"
#include "valkyrie/controllers/PhoenixController.h"
#include "Constants.h"
#include "valkyrie/BaseSubsystem.h"
#include "valkyrie/controllers/PIDF.h"
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/Command.h>
#include <frc2/command/Commands.h>

#include <unordered_map>
#include "valkyrie/Gamepad.h"

#include "Drivetrain.h"


class Intake : public valor::BaseSubsystem
{
public:

    Intake(frc::TimedRobot *robot, valor::CANdleSensor *_leds);

    ~Intake();

    void init();
    void resetState();
    void assessInputs();
    // void analyzeDashboard();
    void assignOutputs();

    void InitSendable(wpi::SendableBuilder& builder);

    enum PIVOT_STATE
    {
        DEPLOYED,
        STOW
    };

    enum WHEEL_STATE
    {
        INTAKE,
        OUTTAKE,
        NONE
    };

    struct x
    {
        PIVOT_STATE pivotState;
        WHEEL_STATE wheelState;

    }state;


private:
    valor::PhoenixController *pivotMotor;
    valor::PhoenixController *wheelMotor;

    Drivetrain *drive;
    valor::CANdleSensor *leds;
};

#include "subsystems/Intake.h"
#include <iostream>
#include <math.h>

#include "valkyrie/controllers/NeutralMode.h"
#include "Constants.h"

#include <pathplanner/lib/auto/NamedCommands.h>
#include <frc/DriverStation.h>

#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>

#define STOW_POS 1.0f
#define DEPLOYED_POS 65.0f

#define INTAKE_POWER 2.0f
#define OUTTAKE_POWER -2.0f



Intake::Intake(frc::TimedRobot *_robot, valor::CANdleSensor *_leds) : valor::BaseSubsystem(_robot, "Intake"),
    pivotMotor(nullptr),
    wheelMotor(nullptr),
    leds(_leds)
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

Intake::~Intake()
{

}

void Intake::resetState()
{
    state.pivotState = STOW;
    state.wheelState = NONE;
}

void Intake::init()
{
    valor::PIDF wheelPID;

    wheelPID.aFFType = valor::FeedForwardType::LINEAR;
    bool wheelI = false;

    pivotMotor = new valor::PhoenixController(
        valor::PhoenixControllerType::FALCON,
        CANIDs::INTAKE_PIVOT,
        valor::NeutralMode::Brake,
        true,
        "baseCAN"
    );

    pivotMotor->enableFOC(true);

    wheelMotor = new valor::PhoenixController(
        valor::PhoenixControllerType::FALCON,
        CANIDs::INTAKE_WHEEL,
        valor::NeutralMode::Brake,
        true,
        "baseCAN"
    );

    wheelMotor->enableFOC(true);

    resetState();

}

void Intake::assessInputs()
{
    if (driverGamepad == nullptr || !driverGamepad->IsConnected()) return;

    if (driverGamepad->DPadUp()){
        state.pivotState = DEPLOYED;
        state.wheelState = INTAKE;

    } else if (driverGamepad->DPadDown()){
        state.pivotState = DEPLOYED;
        state.wheelState = OUTTAKE;

    } else {
        state.pivotState = STOW;
        state.wheelState = NONE;

    }
}

// void Intake::analyzeDashboard()
// {
//     return;
// }

void Intake::assignOutputs()
{
    if (state.pivotState == PIVOT_STATE::DEPLOYED){
        pivotMotor->setPosition(units::turn_t{DEPLOYED_POS});

    } else {
        pivotMotor->setPosition(units::turn_t{STOW_POS});

    }


    if (state.wheelState == WHEEL_STATE::INTAKE){
        wheelMotor->setPower(units::volt_t{INTAKE_POWER});

    } else if (state.wheelState == WHEEL_STATE::OUTTAKE){
        wheelMotor->setPower(units::volt_t{OUTTAKE_POWER});

    } else {
        wheelMotor->setPower(units::volt_t{0});

    }
}

void Intake::InitSendable(wpi::SendableBuilder& builder)
{
    builder.SetSmartDashboardType("Subsystem");

    builder.AddIntegerProperty(
        "Wheel State",
        [this] {return state.pivotState;},
        nullptr
    );
    builder.AddIntegerProperty(
        "Pivot State",
        [this] {return state.wheelState;},
        nullptr
    );
    builder.AddDoubleProperty(
        "Wheel Position",
        [this] {return wheelMotor->getSpeed().to<double>();},
        nullptr
    );
    builder.AddDoubleProperty(
        "Pivot Position",
        [this] {return pivotMotor->getPosition().to<double>();},
        nullptr
    );
}


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
#include <frc/PWM.h>

#define FORWARD_LIMIT 68.0f
#define REVERSE_LIMIT 2.0f
#define EXTENDED_POS 65.0f
#define DEADBAND 63.0f
#define RETRACTED_POS 1.5f
#define RESTING_POS 1.5f

#define INTAKE_MAX_SPEED 50.0f
#define INTAKE_MAX_ACCEL 120.0f
#define INTAKE_K_P 4.0f
#define INTAKE_K_ERROR 0.00f
#define INTAKE_K_AFF 0.00f
#define CONVERSION 14.72f

#define UNLATCH_PULSE_TIME 2500_us
#define LATCH_PULSE_TIME 1900_us

#define LATCH_POS 0.55f
#define UNLATCH_POS 1.0f



Intake::Intake(frc::TimedRobot *_robot, valor::CANdleSensor *_leds) : valor::BaseSubsystem(_robot, "Intake"),
    pivotMotors(nullptr),
    intakeMotors(nullptr),
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
    valor::PIDF intakePID;
    // climbPID.maxVelocity = CLIMB_MAX_SPEED;
    // climbPID.maxAcceleration = CLIMB_MAX_ACCEL;
    // climbPID.P = CLIMB_K_P;
    // climbPID.error = CLIMB_K_ERROR;
    // climbPID.aFF = CLIMB_K_AFF;

    intakePID.aFFType = valor::FeedForwardType::LINEAR;
    bool intakeI = false;
    intakeMotors = new valor::PhoenixController(
        CANIDs:: INTAKE,
        valor::NeutralMode::Brake,
        intakeI,
        CONVERSION,
        1,
        intakePID,
        12.0,
        true,
        "baseCAN"
    );
    intakeMotors->setupFollower(CANIDs::INTAKE_FOLLOW, intakeI);
    // intakeMotors->setRange(0, REVERSE_LIMIT, FORWARD_LIMIT);
    intakeMotors->enableFOC(true);

    resetState();

}

// void Climber::assessInputs()
// {
//     if (driverGamepad == nullptr || !driverGamepad->IsConnected()) return;
//     if (operatorGamepad == nullptr || !operatorGamepad->IsConnected()) return;

//     if (operatorGamepad->rightStickYActive() && climbUp.IsScheduled()){
//         climbUp.Cancel();
//     }

//     // if ((driverGamepad->DPadUp() || operatorGamepad->DPadUp()) && !climbUp.IsScheduled()){
//     //     climbUp.Schedule();
//     // }

//     if (operatorGamepad->DPadRight()){
//         state.latchState = UNLATCH;
//     } else if (operatorGamepad->DPadDown()){
//         state.latchState = LATCH;
//     }
//     if (operatorGamepad->rightStickYActive()){
//         state.climbState = MANUAL;
//     }

//     if (driverGamepad->DPadLeft()) {
//         state.latchState = LATCH;
//     } else if (driverGamepad->DPadRight()) {
//         state.latchState = UNLATCH;
//     }

//     if (driverGamepad->DPadUp()) {
//         state.climbState = EXTEND_PWR;
//     } else if (driverGamepad->DPadDown()) {
//         state.climbState = RETRACT_PWR;
//     }
// }

// void Climber::analyzeDashboard()
// {
//     if (climbMotors->getPosition() > 1.0) {
//         table->PutBoolean("Climber overriding leds", true);
//         leds->setColor(0, valor::CANdleSensor::VALOR_PURPLE);
//         leds->setColor(1, valor::CANdleSensor::VALOR_PURPLE);
//     } else {
//         table->PutBoolean("Climber overriding leds", false);
//     }
// }

// void Climber::assignOutputs()
// {
//     // TODO: Set position with state.targetPos when valor::Servo gets implemented
//     if (state.latchState == LATCH_STATE::UNLATCH){
//         servo->SetPulseTime(UNLATCH_PULSE_TIME);
//     } else if(state.latchState == LATCH_STATE::LATCH){
//         servo->SetPulseTime(LATCH_PULSE_TIME);
//     }
//     if (state.climbState == CLIMB_STATE::EXTEND){
//         climbMotors->setPosition(EXTENDED_POS);
//     }
//     if(state.climbState == CLIMB_STATE::DISABLE){
//         climbMotors->setPower(0);
//     }

//     if (state.climbState == EXTEND_PWR && state.latchState == UNLATCH) {
//         climbMotors->setPower(.75);
//     } else if (state.climbState == RETRACT_PWR && state.latchState == LATCH) {
//         climbMotors->setPower(-.75);
//     }

//     if (state.climbState == MANUAL){
//         double opIn = operatorGamepad->rightStickY(2);
//         if (state.latchState == UNLATCH || opIn < 0) {
//             climbMotors->setPower(opIn);
//         }
//     }
//     if (table->GetBoolean("Climber tuning", false)) { 
//         double st = table->GetNumber("Servo Pos", 500) / 1000000;
//         if (servo != nullptr)
//             servo->SetPulseTime(units::second_t{st}); // Can't cast straight to microseconds
//     }
// }


// frc2::CommandPtr Climber::upSequence()
// {
//     return frc2::SequentialCommandGroup(
//             frc2::InstantCommand(
//                 [this]() {
//                     state.latchState = Climber::LATCH_STATE::UNLATCH;
//                 }
//             ),
//             frc2::WaitCommand(0.5_s),
//             frc2::FunctionalCommand(
//                 [this]() { //onInit
//                     state.climbState = Climber::CLIMB_STATE::EXTEND;
//                 },
//                 [this](){}, //onExecute
//                 [this](bool){ //onEnd
//                     state.climbState = Climber::CLIMB_STATE::DISABLE;
//                 },
//                 [this](){ //isFinished
//                     return climbMotors->getPosition() >= DEADBAND;
//                 },
//                 {}
//             ),
//             frc2::InstantCommand(
//                 [this]() {
//                     state.latchState = Climber::LATCH_STATE::LATCH;
//                 }
//             )
//     ).ToPtr();
// }

// void Climber::InitSendable(wpi::SendableBuilder& builder)
// {
//     builder.SetSmartDashboardType("Subsystem");

//     builder.AddDoubleProperty(
//         "Servo Position",
//         [this] {return servo->GetPosition();},
//         nullptr
//     );
//     builder.AddDoubleProperty(
//         "Servo Pulse Time",
//         [this] {return servo->GetPulseTime().to<double>();},
//         nullptr
//     );
//     builder.AddIntegerProperty(
//         "Climber State",
//         [this] {return state.climbState;},
//         nullptr
//     );
//     builder.AddIntegerProperty(
//         "Latch State",
//         [this] {return state.latchState;},
//         nullptr
//     );
//     builder.AddDoubleProperty(
//         "Operator right stick",
//         [this] {return operatorGamepad->rightStickY(2);},
//         nullptr
//     );
// }


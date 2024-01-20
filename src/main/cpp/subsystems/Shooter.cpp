#include "subsystems/Shooter.h"
#include <iostream>
#include <math.h>
#include "valkyrie/controllers/NeutralMode.h"

#define PIVOT_ROTATE_K_VEL 90.0_rpm
#define PIVOT_ROTATE_K_ACC_MUL 0.5f
#define PIVOT_ROTATE_K_F 0.0f
#define PIVOT_ROTATE_K_P 0.0f
#define PIVOT_ROTATE_K_I 0.0f
#define PIVOT_ROTATE_K_D 0.0f
#define PIVOT_ROTATE_K_ERROR 0.0f
#define PIVOT_ROTATE_K_AFF 0.0f
#define PIVOT_ROTATE_K_AFF_POS 0.0f

#define SUBWOOFER_ANG 30.0_deg
#define PODIUM_ANG 45.0_deg
#define STARTING_LINE_ANG 60.0_deg

#define SHOOT_LEFT_POWER -1.0f
#define SPOOLED_LEFT_POWER -0.8f
#define SHOOT_RIGHT_POWER 1.0f
#define SPOOLED_RIGHT_POWER 0.8f
#define OFF_LEFT_POWER 0.0f
#define OFF_RIGHT_POWER 0.0f

#define SHOOTER_ROTATE_GEAR_RATIO 1.0f
#define SHOOTER_ROTATE_FORWARD_LIMIT 90.0_deg
#define SHOOTER_ROTATE_REVERSE_LIMIT 0.0_deg

Shooter::Shooter(frc::TimedRobot *_robot, frc::DigitalInput* _beamBreak) :
    valor::BaseSubsystem(_robot, "Shooter"),
    //pivotMotors(CANIDs::ANGLE_CONTROLLER, valor::NeutralMode::Brake, false),
    LeftflywheelMotors(CANIDs::LEFT_SHOOTER_WHEEL_CONTROLLER, valor::NeutralMode::Brake, false),
    RightflywheelMotors(CANIDs::RIGHT_SHOOTER_WHEEL_CONTROLLER, valor::NeutralMode::Brake, false),
    beamBreak(_beamBreak),
    leftShootPwr(SHOOT_LEFT_POWER),
    rightShootPwr(SHOOT_RIGHT_POWER),
    leftSpooledPwr(SPOOLED_LEFT_POWER),
    rightSpooledPwr(SPOOLED_RIGHT_POWER)

{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

Shooter::~Shooter()
{
}

void Shooter::resetState()
{
    state.flywheel = FlywheelState::NOT_SHOOTING;
    state.pivot = PivotState::SUBWOOFER;
}

void Shooter::init()
{
    pivotPID.velocity = PIVOT_ROTATE_K_VEL.to<double>();
    pivotPID.acceleration = PIVOT_ROTATE_K_ACC_MUL;
    pivotPID.F = PIVOT_ROTATE_K_F;
    pivotPID.P = PIVOT_ROTATE_K_P;
    pivotPID.I = PIVOT_ROTATE_K_I;
    pivotPID.D = PIVOT_ROTATE_K_D;
    pivotPID.error = PIVOT_ROTATE_K_ERROR;
    pivotPID.aFF = PIVOT_ROTATE_K_AFF;
    pivotPID.aFFTarget = PIVOT_ROTATE_K_AFF_POS;

    // pivotMotors.setConversion(1.0 / SHOOTER_ROTATE_GEAR_RATIO * 360);
    // pivotMotors.setForwardLimit(SHOOTER_ROTATE_FORWARD_LIMIT.to<double>());
    // pivotMotors.setReverseLimit(SHOOTER_ROTATE_REVERSE_LIMIT.to<double>());
    // pivotMotors.setPIDF(pivotPID, 0);
    table->PutNumber("Left shoot power", leftShootPwr);
    table->PutNumber("Right shoot power", rightShootPwr);
    table->PutNumber("Left spooled power", leftSpooledPwr);
    table->PutNumber("Right spooled power", rightSpooledPwr);


    resetState();

}

void Shooter::assessInputs()
{
    //SHOOT LOGIC
    if (driverGamepad->rightTriggerActive()) {
        state.flywheel = FlywheelState::SHOOTING;
    }
    else if (beamBreak->Get()) {
        state.flywheel = FlywheelState::SPOOLED;
    }
    else {
        state.flywheel = FlywheelState::NOT_SHOOTING;
    } 

    //PIVOT LOGIC
    if (operatorGamepad->rightTriggerActive()) {
        state.pivot = PivotState::SUBWOOFER;
    }
    else if (operatorGamepad->GetRightBumperPressed()) {
        state.pivot = PivotState::PODIUM;
    }
    else if (operatorGamepad->GetLeftBumperPressed()) { 
        state.pivot = PivotState::STARTING_LINE;
    }
    else if (operatorGamepad->leftTriggerActive()) {
        state.pivot = PivotState::TRACKING;
    }
}

void Shooter::analyzeDashboard()
{
    calculatingPivotingAngle = 0.0_deg;
    leftShootPwr = table->GetNumber("Left shoot power", leftShootPwr);
    rightShootPwr = table->GetNumber("Right shoot power", rightShootPwr);
    leftSpooledPwr = table->GetNumber("Left spooled power", leftSpooledPwr);
    rightSpooledPwr = table->GetNumber("Right spooled power", rightSpooledPwr);}

void Shooter::assignOutputs()
{

    //NEED PIVOT MOTOR
    // if(state.pivot == PivotState::SUBWOOFER){
    //     pivotMotors.setPosition(SUBWOOFER_ANG.to<double>());
    // }
    // else if(state.pivot == PivotState::PODIUM){
    //     pivotMotors.setPosition(PODIUM_ANG.to<double>());
    // }
    // else if(state.pivot == PivotState::STARTING_LINE){
    //     pivotMotors.setPosition(STARTING_LINE_ANG.to<double>());
    // }
    // else if(state.pivot == PivotState::TRACKING){
    //     pivotMotors.setPosition(calculatingPivotingAngle.to<double>());
    // }

    //SHOOTER
    if(state.flywheel == FlywheelState::SHOOTING){
        LeftflywheelMotors.setPower(leftShootPwr);
        RightflywheelMotors.setPower(rightShootPwr);
    }
    else if(state.flywheel == FlywheelState::SPOOLED){
        LeftflywheelMotors.setPower(leftSpooledPwr);
        RightflywheelMotors.setPower(rightSpooledPwr);
    }
    else{
        RightflywheelMotors.setPower(OFF_LEFT_POWER);
        LeftflywheelMotors.setPower(OFF_RIGHT_POWER);
    }
}

void Shooter::InitSendable(wpi::SendableBuilder& builder)
{
    builder.SetSmartDashboardType("Shooter");

    builder.AddIntegerProperty(
        "flywheel state",
        [this] {return state.flywheel;},
        nullptr
    );

    builder.AddIntegerProperty(
        "pivot state",
        [this] {return state.pivot;},
        nullptr
    );

    builder.AddDoubleProperty(
        "Left shoot power", 
        [this] { return leftShootPwr; },
        nullptr
    );

    builder.AddDoubleProperty(
        "Right shoot power", 
        [this] { return rightShootPwr; },
        nullptr
    );

    builder.AddDoubleProperty(
        "Left spooled power", 
        [this] { return leftSpooledPwr; },
        nullptr
    );

    builder.AddDoubleProperty(
        "Right spooled power", 
        [this] { return rightSpooledPwr; },
        nullptr
    );




}
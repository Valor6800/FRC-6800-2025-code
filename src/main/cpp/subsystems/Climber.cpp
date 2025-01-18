#include "subsystems/Climber.h"
#include <iostream>
#include <math.h>

#include "valkyrie/controllers/NeutralMode.h"
#include "Constants.h"

#include <pathplanner/lib/auto/NamedCommands.h>
#include <frc/DriverStation.h>

#define FORWARD_LIMIT 

#define CLIMB_MAX_SPEED units::angular_velocity::turns_per_second_t (0)
#define CLIMB_MAX_ACCEL units::angular_acceleration::turns_per_second_squared_t (0)
#define CLIMB_K_P 0
#define CLIMB_K_ERROR units::angle::turn_t (0)
#define CLIMB_K_AFF 0

#define CRAB_MAX_SPEED units::angular_velocity::turns_per_second_t (0)
#define CRAB_MAX_ACCEL units::angular_acceleration::turns_per_second_squared_t (0)
#define CRAB_K_P 0
#define CRAB_K_ERROR units::angle::turn_t (0)
#define CRAB_K_AFF 0

#define CLIMB_GEAR_RATIO 0
#define CRAB_GEAR_RATIO 0

#define CLIMBED_POS units::angle::turn_t (0)
#define DEPLOY_POS units::angle::turn_t (0)
#define STOW_POS units::angle::turn_t (0)

#define CRAB_SPEED units::voltage::volt_t (0)

using namespace valor;

Climber::Climber(frc::TimedRobot *_robot) : valor::BaseSubsystem(_robot, "Climber"),
    climbMotor(nullptr)
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
    table->PutBoolean("Climbed?", state.climbed);
}

Climber::~Climber()
{

}

void Climber::resetState()
{
    state.climbState = CLIMB_STATE::STOW;
    state.crabState = CRAB_STATE::NO_CRAB;
}

void Climber::init()
{
    valor::PIDF climbPID;
    climbPID.maxVelocity = CLIMB_MAX_SPEED;
    climbPID.maxAcceleration = CLIMB_MAX_ACCEL;
    climbPID.P = CLIMB_K_P;
    climbPID.error = CLIMB_K_ERROR;
    climbPID.aFF = CLIMB_K_AFF;

    valor::PIDF crabPID;
    crabPID.maxVelocity = CRAB_MAX_SPEED;
    crabPID.maxAcceleration = CRAB_MAX_ACCEL;
    crabPID.P = CRAB_K_P;
    crabPID.error = CRAB_K_ERROR;
    crabPID.aFF = CRAB_K_AFF;

    climbPID.aFFType = valor::FeedForwardType::LINEAR;
    bool climberInversion = false;
    bool crabInversion = false;
    climbMotor = new valor::PhoenixController(
        valor::PhoenixControllerType::FALCON,
        CANIDs::CLIMBER,
        valor::NeutralMode::Brake,
        climberInversion,
        "baseCAN"
    );

    climbMotor->setGearRatios(CLIMB_GEAR_RATIO, 1.0);
    climbMotor->setPIDF(climbPID, 0);
    climbMotor->setContinuousWrap(true);
    climbMotor->applyConfig();

    crabMotor = new valor::PhoenixController(
        valor::PhoenixControllerType::FALCON,
        CANIDs::CRABB,
        valor::NeutralMode::Brake,
        crabInversion,
        "baseCAN"
    );

    crabMotor->setGearRatios(CRAB_GEAR_RATIO, 1.0);
    crabMotor->setPIDF(crabPID, 0);
    crabMotor->setContinuousWrap(true);
    crabMotor->applyConfig();
}

void Climber::assessInputs()
{
    if (driverGamepad == nullptr || !driverGamepad->IsConnected()) return;
    if (operatorGamepad == nullptr || !operatorGamepad->IsConnected()) return;

    if (operatorGamepad->rightTriggerActive()) {
        state.crabState = CRAB_STATE::CRABBING;
        state.climbState = CLIMB_STATE::DEPLOY;
    } else {
        state.crabState = CRAB_STATE::NO_CRAB;
    }
}

void Climber::analyzeDashboard()
{
    //crab sensor
    /*if (crabSensor) {
        state.climbState = CLIMB_STATE::CLIMB;
    }*/

    if(climbMotor->getPosition() == CLIMBED_POS) {
        state.climbed = true;
    } else {
        state.climbed = false;
    }

}

void Climber::assignOutputs()
{
    if (state.crabState == CRAB_STATE::CRABBING) {
        crabMotor->setPower(CRAB_SPEED);
    } else{
        crabMotor->setPower(units::voltage::volt_t (0));
    }

    if (state.climbState == CLIMB_STATE::DEPLOY) {
        climbMotor->setPosition(DEPLOY_POS);
    } else if (state.climbState == CLIMB_STATE::CLIMB) {
        climbMotor->setPosition(CLIMBED_POS);
    } else{
        climbMotor->setPosition(STOW_POS);
    }
}


void Climber::InitSendable(wpi::SendableBuilder& builder)
{
    builder.SetSmartDashboardType("Subsystem");
    builder.AddBooleanProperty(
        "Climbed?",
        [this]{return state.climbed;},
        nullptr
    );
}



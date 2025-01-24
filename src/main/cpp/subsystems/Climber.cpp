#include "subsystems/Climber.h"
#include <iostream>
#include <math.h>

#include "valkyrie/controllers/NeutralMode.h"
#include "Constants.h"

#include <pathplanner/lib/auto/NamedCommands.h>
#include <frc/DriverStation.h>

#define FORWARD_LIMIT 

#define CLIMB_MANUAL_SPEED units::angular_velocity::turns_per_second_t (0)
#define CLIMB_MAX_SPEED units::angular_velocity::turns_per_second_t (0.5503)
#define CLIMB_MAX_ACCEL units::angular_acceleration::turns_per_second_squared_t (0.5503)
#define CLIMB_K_P 0
#define CLIMB_K_D 0
#define CLIMB_K_ERROR units::angle::turn_t (0)
#define CLIMB_K_JERK units::angular_jerk::turns_per_second_cubed_t (0)
#define CLIMB_K_AFF 0
#define CLIMB_K_S 0

#define CRAB_MAX_SPEED units::angular_velocity::turns_per_second_t (0)
#define CRAB_MAX_ACCEL units::angular_acceleration::turns_per_second_squared_t (0)
#define CRAB_K_P 0
#define CRAB_K_ERROR units::angle::turn_t (0)
#define CRAB_K_AFF 0

#define CLIMB_GEAR_RATIO 181.7
#define CRAB_GEAR_RATIO 0

#define CLIMBED_POS units::angle::turn_t (0)
#define DEPLOY_POS units::angle::turn_t (0)
#define STOW_POS units::angle::turn_t (0)

#define CRAB_SPEED units::voltage::volt_t (0)
#define SPIKE_CURRENT 0
#define CACHE_SIZE 0

using namespace valor;

Climber::Climber(frc::TimedRobot *_robot) : valor::BaseSubsystem(_robot, "Climber"),
    climbMotor(nullptr),
    crabMotor(nullptr),
    currentSensor(_robot, "Climber")
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    table->PutBoolean("Climbed?", state.climbed);
    init();
}

Climber::~Climber()
{

}

void Climber::resetState()
{
    state.climbState = CLIMB_STATE::STOW;
    state.crabState = CRAB_STATE::NO_CRAB;
    state.climbed = false;
    currentSensor.reset();
}

void Climber::init()
{
    setClimberPID();
    setCrabPID();

    state.spikeCurrent = SPIKE_CURRENT;
    state.cacheSize = CACHE_SIZE;

    currentSensor.setSpikeSetpoint(state.spikeCurrent);
    currentSensor.setGetter([this]() { return crabMotor->getCurrent().to<double>(); });
    currentSensor.setSpikeCallback([this]() {state.crabState == CRAB_STATE::CRABBED;});
    currentSensor.setCacheSize(state.cacheSize);
    
    table->PutNumber("Spike Current", state.spikeCurrent);
    table->PutNumber("Cache Size", state.cacheSize);

    resetState();
}

void Climber::assessInputs()
{
    if (driverGamepad == nullptr || !driverGamepad->IsConnected()) return;
    if (operatorGamepad == nullptr || !operatorGamepad->IsConnected()) return;

    if (operatorGamepad->rightStickYActive()) {
        state.climbState = CLIMB_STATE::MANUAL;
    } else if (operatorGamepad->rightTriggerActive()) {
        state.crabState = CRAB_STATE::CRABBING;
        state.climbState = CLIMB_STATE::DEPLOY;
    } else {
        state.crabState = CRAB_STATE::NO_CRAB;
    }
}

void Climber::analyzeDashboard()
{
    state.spikeCurrent = table->GetNumber("Spike Current", SPIKE_CURRENT);
    state.cacheSize = table->GetNumber("Cache Size", CACHE_SIZE);

    currentSensor.setSpikeSetpoint(state.spikeCurrent);
    currentSensor.setCacheSize(state.cacheSize);

    //crab sensor
    if (driverGamepad->GetAButton()) { //state.crabState == CRAB_STATE::CRABBED
        state.climbState = CLIMB_STATE::CLIMB;
    }

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

    if (state.climbState == CLIMB_STATE::MANUAL) {
        manualSpeed = units::angular_velocity::turns_per_second_t {operatorGamepad->leftStickY(2) * CLIMB_MANUAL_SPEED};
        climbMotor->setSpeed(manualSpeed);
    } else if (state.climbState == CLIMB_STATE::DEPLOY) {
        climbMotor->setPosition(DEPLOY_POS);
    } else if (state.climbState == CLIMB_STATE::CLIMB) {
        climbMotor->setPosition(CLIMBED_POS);
    } else{
        climbMotor->setPosition(STOW_POS);
    }
}

void Climber::setClimberPID() {

    valor::PIDF climbPID;

    climbPID.maxVelocity = CLIMB_MAX_SPEED;
    climbPID.maxAcceleration = CLIMB_MAX_ACCEL;
    climbPID.P = CLIMB_K_P;
    climbPID.D = CLIMB_K_D;
    climbPID.error = CLIMB_K_ERROR;
    climbPID.aFF = CLIMB_K_AFF;
    climbPID.maxJerk = CLIMB_K_JERK;
    climbPID.aFFType = valor::FeedForwardType::CIRCULAR;
    climbPID.S = CLIMB_K_S;

    bool climberInversion = false;

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
}

void Climber::setCrabPID() {

    valor::PIDF crabPID;

    crabPID.maxVelocity = CRAB_MAX_SPEED;
    crabPID.maxAcceleration = CRAB_MAX_ACCEL;
    crabPID.P = CRAB_K_P;

    bool crabInversion = false;

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


void Climber::InitSendable(wpi::SendableBuilder& builder)
{
    builder.SetSmartDashboardType("Subsystem");
    builder.AddBooleanProperty(
        "Climbed?",
        [this]{return state.climbed;},
        nullptr
    );
    builder.AddDoubleProperty(
        "Spike Current",
        [this] {return state.spikeCurrent;},
        nullptr
    );
    builder.AddDoubleProperty(
        "Cache Size",
        [this] {return state.cacheSize;},
        nullptr
    );
}



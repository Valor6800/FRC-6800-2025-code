#include "subsystems/Climber.h"
#include <iostream>
#include <math.h>

#include "valkyrie/controllers/NeutralMode.h"
#include "Constants.h"

#include <pathplanner/lib/auto/NamedCommands.h>
#include <frc/DriverStation.h>

#define CLIMB_MANUAL_SPEED units::angular_velocity::turns_per_second_t (0)

#define CRAB_MAX_SPEED units::angular_velocity::turns_per_second_t (0)
#define CRAB_MAX_ACCEL units::angular_acceleration::turns_per_second_squared_t (0)
#define CRAB_K_P 0
#define CRAB_K_ERROR units::angle::turn_t (0)
#define CRAB_K_AFF 0
#define CRABB_ROTOR_TO_SENSOR 1.6
#define STABBY_K_P 0.6
#define STABBY_K_S 0.5

#define CLIMB_GEAR_RATIO 145.36
#define CRAB_GEAR_RATIO 0

#define DEPLOYED_POS units::angle::turn_t (0.1)
#define RETRACTED_POS units::angle::turn_t (-0.1)
#define STOW_POS units::angle::turn_t (0)

#define CRAB_SPEED units::voltage::volt_t (0)
#define SPIKE_CURRENT 0
#define CACHE_SIZE 0

using namespace valor;

Climber::Climber(frc::TimedRobot *_robot) : valor::BaseSubsystem(_robot, "Climber"),
    climbMotors(nullptr),
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
    state.climbState = CLIMB_STATE::MANUAL;
    state.crabState = CRAB_STATE::NO_CRAB;
    state.climbed = false;
    currentSensor.reset();
}

void Climber::init()
{
    valor::PIDF climbPID = Constants::Climber::getClimberPIDF();
    valor::PIDF stabbyPID;

    climbMotors = new valor::PhoenixController(
        valor::PhoenixControllerType::FALCON_FOC,
        CANIDs::CLIMBER_LEAD,
        valor::NeutralMode::Brake,
        Constants::Climber::climbMotorInverted(),
        "baseCAN"
    );

    stabbyMotor = new valor::PhoenixController(
        valor::PhoenixControllerType::KRAKEN_X44_FOC, 
        CANIDs::CRABB, 
        valor::NeutralMode::Coast, 
        false, 
        "baseCAN"     
    );
    
    climbMotors->setupCANCoder(CANIDs::CLIMBER_CAN, 0.0_tr, true, "baseCAN", 1_tr); //0.5022

    climbMotors->setGearRatios(CLIMB_GEAR_RATIO, 1.0);
    stabbyMotor->setGearRatios(1.0, CRABB_ROTOR_TO_SENSOR);
    stabbyPID.P = STABBY_K_P;

    stabbyMotor->setPIDF(stabbyPID, 0);
    stabbyMotor->applyConfig();
    stabbyMotor->enableFOC(true);

    climbPID.maxVelocity = climbMotors->getMaxMechSpeed();
    climbPID.maxAcceleration = climbMotors->getMaxMechSpeed() / Constants::Climber::maxVelocityRampTime();

    climbMotors->setForwardLimit(Constants::Climber::getForwardLimit());
    climbMotors->setReverseLimit(Constants::Climber::getReverseLimit());
    climbMotors->setupFollower(CANIDs::CLIMBER_FOLLOW, Constants::Climber::climbMotorInverted());
    climbMotors->setPIDF(climbPID);
    climbMotors->setContinuousWrap(true);
    climbMotors->enableFOC(true);
    climbMotors->applyConfig();
    
    table->PutNumber("Spike Current", state.spikeCurrent);
    table->PutNumber("Cache Size", state.cacheSize);

    resetState();
}

void Climber::assessInputs()
{
    if (operatorGamepad == nullptr || !operatorGamepad->IsConnected()) return;

    if (operatorGamepad->rightStickYActive()) {
        state.climbState = CLIMB_STATE::MANUAL;
        state.manualSpeed = operatorGamepad->rightStickY(2) * 12_V;
    }

    if (operatorGamepad->DPadLeft()) {
        stabbyMotor->setSpeed(10_tps);
    }else{
        stabbyMotor->setSpeed(0_tps);
    }

}

void Climber::analyzeDashboard()
{

}

void Climber::assignOutputs()
{

     if (state.climbState == CLIMB_STATE::MANUAL) {
         climbMotors->setPower(state.manualSpeed);
     } else if (state.climbState == CLIMB_STATE::DEPLOYED) {
         climbMotors->setPosition(DEPLOYED_POS);
     } else if (state.climbState == CLIMB_STATE::RETRACTED) {
         climbMotors->setPosition(RETRACTED_POS);
     } else{
         climbMotors->setPosition(STOW_POS);
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



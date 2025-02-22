#include "subsystems/Climber.h"
#include <iostream>
#include <math.h>

#include "valkyrie/controllers/NeutralMode.h"
#include "Constants.h"

#include <pathplanner/lib/auto/NamedCommands.h>
#include <frc/DriverStation.h>

#define STABBY_ROTOR_TO_SENSOR 1.6
#define STABBY_K_P 0.6
#define STABBY_K_S 0.5

#define CLIMB_GEAR_RATIO 145.36
#define CRAB_GEAR_RATIO 0

#define DEPLOYED_POS units::angle::turn_t (0.75)
#define RETRACTED_POS units::angle::turn_t (0.353)
#define LOCK_OUT_POS units::angle::turn_t (0.369)
#define STOW_POS units::angle::turn_t (0.5)

#define STABBY_SPEED 40_tps
#define SPIKE_CURRENT 0
#define CACHE_SIZE 0

using namespace valor;

Climber::Climber(frc::TimedRobot *_robot) : valor::BaseSubsystem(_robot, "Climber"),
    climbMotors(nullptr),
    currentSensor(_robot, "Climber")
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    table->PutBoolean("Climbed?", state.hasClimbed);
    init();
}

Climber::~Climber()
{

}

void Climber::resetState()
{
    state.climbState = CLIMB_STATE::STOW;
    state.stabState = STABBY_STATE::NO_CRAB;
    state.hasClimbed = false;
    currentSensor.reset();
}

void Climber::init()
{
    valor::PIDF climbPID = Constants::Climber::getClimberPIDF();
    valor::PIDF climbRetractPID = Constants::Climber::getClimberRetractPIDF();
    valor::PIDF stabbyPID;

    climbMotors = new valor::PhoenixController(
        valor::PhoenixControllerType::KRAKEN_X60_FOC,
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
    stabbyMotor->setGearRatios(1.0, STABBY_ROTOR_TO_SENSOR);
    stabbyPID.P = STABBY_K_P;

    stabbyMotor->setPIDF(stabbyPID, 0);
    stabbyMotor->applyConfig();
    stabbyMotor->enableFOC(true);

    climbPID.maxVelocity = climbMotors->getMaxMechSpeed();
    climbPID.maxAcceleration = climbMotors->getMaxMechSpeed() / Constants::Climber::maxVelocityRampTime();

    climbRetractPID.maxVelocity = climbMotors->getMaxMechSpeed();
    climbRetractPID.maxAcceleration = climbMotors->getMaxMechSpeed() / Constants::Climber::maxVelocityRampTime();

    climbMotors->setForwardLimit(Constants::Climber::getForwardLimit());
    climbMotors->setReverseLimit(Constants::Climber::getReverseLimit());
    climbMotors->setupFollower(CANIDs::CLIMBER_FOLLOW, Constants::Climber::climbMotorInverted());
    climbMotors->setPIDF(climbRetractPID, 1);
    climbMotors->setPIDF(climbPID, 0);
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

    if(driverGamepad->GetYButton()){
        state.climbState = CLIMB_STATE::DEPLOYED;
    } else if(driverGamepad->GetXButton()){
        state.climbState = CLIMB_STATE::STOW;
    } else if(driverGamepad->GetAButton() && operatorGamepad->DPadDown()){
        state.climbState = CLIMB_STATE::RETRACTED;
    }

}

void Climber::analyzeDashboard()
{

}

void Climber::assignOutputs()
{

    if(climbMotors->getPosition() < LOCK_OUT_POS || state.hasClimbed){
        climbMotors->setPower(0_V);
        state.hasClimbed = true;
    } else{
        if (state.climbState == CLIMB_STATE::MANUAL) {
            climbMotors->setPower(state.manualSpeed);
        } else if (state.climbState == CLIMB_STATE::DEPLOYED) {
            climbMotors->setPosition(DEPLOYED_POS, 0);
        } else if (state.climbState == CLIMB_STATE::RETRACTED) {
            climbMotors->setPosition(RETRACTED_POS, 1);
        } else if(state.climbState == CLIMB_STATE::STOW){
            climbMotors->setPosition(STOW_POS, 0);
        }
    }

     if (state.climbState == CLIMB_STATE::DEPLOYED || state.stabState == STABBY_STATE::CRABBING) {
        stabbyMotor->setSpeed(STABBY_SPEED);
     } else{
        stabbyMotor->setPower(0_V);
     }
}

void Climber::setDegrees(units::degree_t deg)
{
    climbMotors->setPosition((deg / 360_deg) * 1_tr);
}


void Climber::InitSendable(wpi::SendableBuilder& builder)
{
    builder.SetSmartDashboardType("Subsystem");
    builder.AddIntegerProperty(
        "Climb State",
        [this]{return state.climbState;},
        nullptr
    );
    builder.AddIntegerProperty(
        "Stabby State",
        [this]{return state.stabState;},
        nullptr
    );
}



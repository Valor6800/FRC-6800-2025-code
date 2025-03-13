#include "subsystems/Climber.h"
#include <iostream>
#include <math.h>
#include <mutex>

#include "units/voltage.h"
#include "valkyrie/controllers/NeutralMode.h"
#include "Constants.h"

#include <pathplanner/lib/auto/NamedCommands.h>
#include <frc/DriverStation.h>

#define STABBY_ROTOR_TO_SENSOR 1.6
#define STABBY_K_P 0.6
#define STABBY_K_S 0.5

#define CLIMB_GEAR_RATIO 58.1432
#define CRAB_GEAR_RATIO 0

#define DEPLOYED_POS units::angle::turn_t (1.5) // (1.6
#define RETRACTED_POS units::angle::turn_t (3.646)// (0.36
#define LOCK_OUT_POS units::angle::turn_t (0.3) // (2.7)
#define STOW_POS units::angle::turn_t (0.5)
//enocder climb value at .3

// soft limit of 3

#define STABBY_SPEED 40_tps
#define SPIKE_CURRENT 0
#define CACHE_SIZE 0

using namespace valor;

Climber::Climber(frc::TimedRobot *_robot, valor::CANdleSensor* _leds) : valor::BaseSubsystem(_robot, "Climber"),
    climbMotors(nullptr),
    currentSensor(_robot, "Climber"),
    climbCancoder(new ctre::phoenix6::hardware::CANcoder(CANIDs::CLIMBER_CAN, "baseCAN")),
    leds(_leds)
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

Climber::~Climber()
{

}

void Climber::resetState()
{
    state.climbState = CLIMB_STATE::STOW;
    // state.stabState = STABBY_STATE::NO_CRAB;
    currentSensor.reset();
    state.hasClimbed = false;
}

void Climber::init()
{
    valor::PIDF climbPID = Constants::Climber::getClimberPIDF();
    valor::PIDF climbRetractPID = Constants::Climber::getClimberRetractPIDF();
    valor::PIDF stabbyPID;

    climbMotors = new valor::PhoenixController(
        valor::PhoenixControllerType::KRAKEN_X44_FOC,
        CANIDs::CLIMBER_LEAD,
        valor::NeutralMode::Brake,
        Constants::Climber::climbMotorInverted(),
        "baseCAN"
    );

    // stabbyMotor = new valor::PhoenixController(
    //     valor::PhoenixControllerType::KRAKEN_X44_FOC, 
    //     CANIDs::CRABB, 
    //     valor::NeutralMode::Coast, 
    //     false, 
    //     "baseCAN"     
    // );
    
    // climbMotors->setupCANCoder(CANIDs::CLIMBER_CAN, Constants::Climber::magnetOffset(), true, "baseCAN", 1_tr); //0.5022
    ctre::phoenix6::configs::MagnetSensorConfigs cancoderConfig;
    cancoderConfig.AbsoluteSensorDiscontinuityPoint = 1_tr;
    cancoderConfig.SensorDirection = ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive;
    cancoderConfig.MagnetOffset = -Constants::Climber::magnetOffset();
    climbCancoder->GetConfigurator().Apply(cancoderConfig);

    climbMotors->setCurrentLimits(
        units::ampere_t{100},
        units::ampere_t{60},
        units::ampere_t{45},
        units::second_t{0.5},
        true
    );

    climbMotors->setGearRatios(1.0, CLIMB_GEAR_RATIO);
    // stabbyMotor->setGearRatios(1.0, STABBY_ROTOR_TO_SENSOR);
    // stabbyPID.P = STABBY_K_P;

    // stabbyMotor->setPIDF(stabbyPID, 0);
    // stabbyMotor->applyConfig();
    // stabbyMotor->enableFOC(true);

    climbPID.maxVelocity = climbMotors->getMaxMechSpeed() / 2.0;
    climbPID.maxAcceleration = climbMotors->getMaxMechSpeed() / Constants::Climber::maxVelocityRampTime();

    climbRetractPID.maxVelocity = climbMotors->getMaxMechSpeed();
    climbRetractPID.maxAcceleration = climbMotors->getMaxMechSpeed() / Constants::Climber::maxVelocityRampTime();

    climbMotors->setForwardLimit(Constants::Climber::getForwardLimit());
    climbMotors->setReverseLimit(Constants::Climber::getReverseLimit());
    // climbMotors->setupFollower(CANIDs::CLIMBER_FOLLOW, Constants::Climber::climbMotorInverted());
    climbMotors->setPIDF(climbRetractPID, 1);
    climbMotors->setPIDF(climbPID, 0);
    climbMotors->setContinuousWrap(false);
    climbMotors->enableFOC(true);
    climbMotors->setVoltageLimits(
        units::volt_t(0.0),
        units::volt_t(12.0)
    );
    climbMotors->applyConfig();
    
    table->PutNumber("Spike Current", state.spikeCurrent);
    table->PutNumber("Cache Size", state.cacheSize);

    resetState();

    climbMotors->setEncoderPosition(climbCancoder->GetAbsolutePosition().GetValue());
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
    } else if(operatorGamepad->DPadUp()) {
        state.hasClimbed = true;
    }

}

void Climber::analyzeDashboard()
{
    leds->setLED(LEDConstants::LED_POS_CLIMBER, valor::CANdleSensor::cancoderMagnetHealthGetter(climbCancoder));
}

void Climber::assignOutputs()
{
    std::unique_lock<std::mutex> ledLock(ledMutex, std::defer_lock);
    if (state.hasClimbed) {
        if (!ledLock.owns_lock()) {
            ledLock.lock();
            leds->setColorAll(valor::CANdleSensor::GREEN);
        }
    } else {
        if (ledLock.owns_lock()) {
            ledLock.unlock();
        }
    }
    
    if(climbCancoder->GetAbsolutePosition().GetValue()<= LOCK_OUT_POS || state.hasClimbed){
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

    //  if (state.climbState == CLIMB_STATE::DEPLOYED || state.stabState == STABBY_STATE::CRABBING) {
    //     stabbyMotor->setSpeed(STABBY_SPEED);
    //  } else{
    //     stabbyMotor->setPower(0_V);
    //  }
}

void Climber::setDegrees(units::degree_t deg)
{
    climbMotors->setPosition((deg / 360_deg) * 1_tr);
}

frc2::CommandPtr Climber::pitSequenceStage(CLIMB_STATE climbState) {
    units::turn_t climbPos = STOW_POS;
    if (climbState == RETRACTED) climbPos = RETRACTED_POS;
    else if (climbState == DEPLOYED) climbPos = DEPLOYED_POS;
    return frc2::cmd::Parallel(
        frc2::cmd::Wait(5_s),
        frc2::FunctionalCommand{
            [this, climbState] {
                climberPosSuccess.Set(false);
                climberPosFail.Set(false);
                state.climbState = climbState;
            },
            [this, climbPos] {
            },
            [](bool) {},
            [] { return false; },
            { this }
        }.ToPtr()
    );
}

frc2::CommandPtr Climber::pitSequence() {
    return frc2::cmd::Sequence(
        pitSequenceStage(STOW),
        pitSequenceStage(DEPLOYED),
        pitSequenceStage(RETRACTED)
    );
}

void Climber::InitSendable(wpi::SendableBuilder& builder)
{
    builder.SetSmartDashboardType("Subsystem");
    builder.AddIntegerProperty(
        "Climb State",
        [this]{return state.climbState;},
        nullptr
    );
    // builder.AddIntegerProperty(
    //     "Stabby State",
    //     [this]{return state.stabState;},
    //     nullptr
    // );
}



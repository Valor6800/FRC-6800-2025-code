#include "subsystems/Climber.h"
#include <iostream>
#include <math.h>

#include "valkyrie/controllers/NeutralMode.h"
#include "Constants.h"

#include <pathplanner/lib/auto/NamedCommands.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>
#include <frc/DriverStation.h>

#define FORWARD_LIMIT 0.25_tr
#define REVERSE_LIMIT -0.20_tr

#define CLIMB_K_P 100
#define CLIMB_K_D 0
#define CLIMB_K_ERROR units::angle::turn_t (0)
#define CLIMB_K_JERK units::angular_jerk::turns_per_second_cubed_t (0)
#define CLIMB_K_AFF 0
#define CLIMB_K_S 0

#define CRAB_MAX_ACCEL units::angular_acceleration::turns_per_second_squared_t (0)
#define CRAB_K_P 0
#define CRAB_K_ERROR units::angle::turn_t (0)
#define CRAB_K_AFF 0

#define CLIMB_GEAR_RATIO 227.12
#define CRAB_GEAR_RATIO 0

#define DEPLOYED_POS units::angle::turn_t (0.1)
#define RETRACTED_POS units::angle::turn_t (-0.1)
#define STOW_POS units::angle::turn_t (0)

#define CRAB_SPEED units::voltage::volt_t (0)

using namespace valor;

Climber::Climber(frc::TimedRobot *_robot) : valor::BaseSubsystem(_robot, "Climber"),
    climbMotors(nullptr),
    currentSensor(_robot, "Climber")
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
    pathplanner::NamedCommands::registerCommand("Stow climber", std::move(
        frc2::SequentialCommandGroup(
            frc2::InstantCommand(
                [this]() {
                    
                    state.climbState = CLIMB_STATE::STOW;
                }
            )
        )
    ).ToPtr());
    pathplanner::NamedCommands::registerCommand("Deploy climber", std::move(
        frc2::SequentialCommandGroup(
            frc2::InstantCommand(
                [this]() {
                    
                    state.climbState = CLIMB_STATE::DEPLOYED;
                }
            )
        )
    ).ToPtr());
    pathplanner::NamedCommands::registerCommand("Retract climber", std::move(
        frc2::SequentialCommandGroup(
            frc2::InstantCommand(
                [this]() {
                    
                    state.climbState = CLIMB_STATE::RETRACTED;
                }
            )
        )
    ).ToPtr());
    pathplanner::NamedCommands::registerCommand("Enable crab", std::move(
        frc2::SequentialCommandGroup(
            frc2::InstantCommand(
                [this]() {
                    
                    state.crabState = CRAB_STATE::CRABBING;
                }
            )
        )
    ).ToPtr());
    pathplanner::NamedCommands::registerCommand("Disable crab", std::move(
        frc2::SequentialCommandGroup(
            frc2::InstantCommand(
                [this]() {
                    
                    state.crabState = CRAB_STATE::NO_CRAB;
                }
            )
        )
    ).ToPtr());
}

Climber::~Climber()
{

}

void Climber::resetState()
{
    state.climbState = CLIMB_STATE::MANUAL;
    state.crabState = CRAB_STATE::NO_CRAB;
    currentSensor.reset();
}

void Climber::init()
{

    valor::PIDF climbPID;

    climbMotors = new valor::PhoenixController(
        valor::PhoenixControllerType::FALCON_FOC,
        CANIDs::CLIMBER_LEAD,
        valor::NeutralMode::Brake,
        false,
        "baseCAN"
    );

    climbMotors->setGearRatios(1.0, CLIMB_GEAR_RATIO);

    climbPID.maxVelocity = climbMotors->getMaxMechSpeed();
    climbPID.maxAcceleration = climbMotors->getMaxMechSpeed() / (1.0_s / 3);
    climbPID.P = CLIMB_K_P;
    climbPID.D = CLIMB_K_D;
    climbPID.error = CLIMB_K_ERROR;
    climbPID.aFF = CLIMB_K_AFF;
    climbPID.maxJerk = CLIMB_K_JERK;
    climbPID.aFFType = valor::FeedForwardType::CIRCULAR;
    climbPID.S = CLIMB_K_S;

    climbMotors->setForwardLimit(FORWARD_LIMIT);
    climbMotors->setReverseLimit(REVERSE_LIMIT);
    climbMotors->setupFollower(CANIDs::CLIMBER_FOLLOW, false);
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

frc2::CommandPtr Climber::climberPitSequence() 
{
    return frc2::SequentialCommandGroup(
        frc2::InstantCommand(
            [this]() {
                state.climbState = Climber::CLIMB_STATE::STOW;
            }
        ),
        frc2::WaitCommand(2_s),
        frc2::InstantCommand(
            [this]() {
                state.climbState = Climber::CLIMB_STATE::DEPLOYED;
            }
        ),
        frc2::WaitCommand(2_s),
        frc2::InstantCommand(
            [this]() {
                state.climbState = Climber::CLIMB_STATE::RETRACTED;
            }
        )
    ).ToPtr();

}

void Climber::InitSendable(wpi::SendableBuilder& builder)
{
    builder.SetSmartDashboardType("Subsystem");
    builder.AddBooleanProperty(
        "Climb state",
        [this]{return state.climbState;},
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



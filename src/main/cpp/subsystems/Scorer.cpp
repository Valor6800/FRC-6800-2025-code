#include "Constants.h"
#include "subsystems/Scorer.h"
#include "units/current.h"
#include "units/time.h"
#include "valkyrie/controllers/NeutralMode.h"
#include "valkyrie/controllers/PIDF.h"
#include <iostream>

#include <pathplanner/lib/auto/NamedCommands.h>

#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>

#include <frc/DriverStation.h>

#define ELEV_K_ERROR units::angle::turn_t (0)
#define ELEVATOR_SENSOR_TO_MECH 1.0f

#define CORAL_INTAKE_SPEED 20_tps //5
#define ALGEE_INTAKE_SPEED 35_tps
#define SCORE_SPEED 20_tps
#define ALGEE_SCORE_SPEED -40_tps
#define ALGEE_HOLD_SPD 0.25_tps

#define ELEVATOR_FORWARD_LIMIT 6_tr
#define ELEVATOR_OFFSET 3_in
#define ELEVATOR_MAGNET_OFFSET 0.321_tr
#define ELEVATOR_JERK 40_tr_per_s_cu

#define ELEVATOR_MOTOR_TO_SENSOR 8.02f
#define PULLEY_CIRCUMFERENCE 1.432_in

#define ALGAE_CACHE_SIZE 1000

using namespace Constants::Scorer;

Scorer::Scorer(frc::TimedRobot *_robot) :
    valor::BaseSubsystem(_robot, "Scorer"),
    hallEffectDebounceSensor(_robot, "HallEffectDebounce"),
    candi(CANIDs::HALL_EFFECT, "baseCAN"),
    elevatorMotor(new valor::PhoenixController(valor::PhoenixControllerType::KRAKEN_X60, CANIDs::ELEV_WHEEL, valor::NeutralMode::Brake, elevatorMotorInverted(), "baseCAN")),
    scorerMotor(new valor::PhoenixController(Constants::getScorerMotorType(), CANIDs::SCORER_WHEEL, valor::NeutralMode::Brake, scorerMotorInverted(), "baseCAN")),
    frontRangeSensor(_robot, "Front Lidar Sensor", CANIDs::FRONT_LIDAR_SENSOR),
    scorerStagingSensor(_robot, "Scorer Staging Sensor", CANIDs::STAGING_LIDAR_SENSOR, "baseCAN"),
    currentSensor(_robot, "Algae Current Sensor"),
    positionMap{std::move(getPositionMap())},
    scoringSpeedMap{std::move(getScoringSpeedMap())}
{

    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);

    pathplanner::NamedCommands::registerCommand("OUTTAKE, INTAKE, HOLD", std::move(
        frc2::SequentialCommandGroup(
            frc2::InstantCommand(
                [this]() {
                    state.scoringState = Scorer::SCORE_STATE::SCORING;
                }
            ),
            frc2::WaitCommand(5_s),
            frc2::InstantCommand(
                [this]() {
                    state.scoringState = Scorer::SCORE_STATE::INTAKING;
                }
            ),
            frc2::WaitCommand(5_s),
            frc2::InstantCommand(
                [this]() {
                    state.scoringState = Scorer::SCORE_STATE::HOLD;
                }
            )
        )
    ).ToPtr());
    pathplanner::NamedCommands::registerCommand("Intaking", std::move(
        frc2::SequentialCommandGroup(
            frc2::InstantCommand(
                [this]() {
                    
                    state.scoringState = Scorer::SCORE_STATE::INTAKING;
                }
            )
        )
    ).ToPtr());
    pathplanner::NamedCommands::registerCommand("Enable Scorer", std::move(
        frc2::SequentialCommandGroup(
            frc2::InstantCommand(
                [this]() {
                    
                    state.scoringState = Scorer::SCORE_STATE::SCORING;
                }
            )
        )
    ).ToPtr());
     pathplanner::NamedCommands::registerCommand("DISABLE Scorer", std::move(
        frc2::SequentialCommandGroup(
            frc2::InstantCommand(
                [this]() {
                    state.scoringState = Scorer::SCORE_STATE::HOLD;
                }
            )
        )
    ).ToPtr());
    pathplanner::NamedCommands::registerCommand("TROUGH POSITION", std::move(
        frc2::SequentialCommandGroup(
            frc2::InstantCommand(
                [this]() {
                    state.scopedState = SCOPED_STATE::SCOPED;
                    state.elevState = ELEVATOR_STATE::ONE;
                    state.gamePiece = GAME_PIECE::CORAL;
                }
            )
        )
    ).ToPtr());
    pathplanner::NamedCommands::registerCommand("LEVEL TWO POSITION", std::move(
        frc2::SequentialCommandGroup(
            frc2::InstantCommand(
                [this]() {
                    state.scopedState = SCOPED_STATE::SCOPED;
                    state.elevState = ELEVATOR_STATE::TWO;
                    state.gamePiece = GAME_PIECE::CORAL;
                }
            )
        )
    ).ToPtr());
    pathplanner::NamedCommands::registerCommand("LEVEL THREE POSITION", std::move(
        frc2::SequentialCommandGroup(
            frc2::InstantCommand(
                [this]() {
                    state.scopedState = SCOPED_STATE::SCOPED;
                    state.elevState = ELEVATOR_STATE::THREE;
                    state.gamePiece = GAME_PIECE::CORAL;
                }
            )
        )
    ).ToPtr());
    pathplanner::NamedCommands::registerCommand("LEVEL FOUR POSITION", std::move(
        frc2::SequentialCommandGroup(
            frc2::InstantCommand(
                [this]() {
                    state.scopedState = SCOPED_STATE::SCOPED;
                    state.elevState = ELEVATOR_STATE::FOUR;
                    state.gamePiece = GAME_PIECE::CORAL;
                }
            )
        )
    ).ToPtr());
    pathplanner::NamedCommands::registerCommand("STOWED POSITION", std::move(
        frc2::SequentialCommandGroup(
            frc2::InstantCommand(
                [this]() {
                    state.scopedState = SCOPED_STATE::SCOPED;
                    state.elevState = ELEVATOR_STATE::STOWED;
                    state.gamePiece = GAME_PIECE::CORAL;
                }
            )
        )
    ).ToPtr());
    pathplanner::NamedCommands::registerCommand("HP", std::move(
        frc2::SequentialCommandGroup(
            frc2::InstantCommand(
                [this]() {
                    state.scopedState = SCOPED_STATE::SCOPED;
                    state.elevState = ELEVATOR_STATE::HP;
                    state.gamePiece = GAME_PIECE::CORAL;
                }
            )
        )
    ).ToPtr());

    init();
}

frc2::CommandPtr Scorer::createScoringSequence() {
    return frc2::SequentialCommandGroup(
        frc2::InstantCommand([this]() { state.scoringState = Scorer::SCORE_STATE::SCORING; }),
        frc2::WaitCommand(5_s),
        frc2::InstantCommand([this]() { state.scoringState = Scorer::SCORE_STATE::INTAKING; }),
        frc2::WaitCommand(5_s),
        frc2::InstantCommand([this]() { state.scoringState = Scorer::SCORE_STATE::HOLD; })
    ).ToPtr();
}

frc2::CommandPtr Scorer::createElevatorSequence() {
    return frc2::SequentialCommandGroup(
        frc2::InstantCommand([this]() { 
            state.scopedState = SCOPED_STATE::SCOPED;
            state.gamePiece = GAME_PIECE::CORAL;
        }),
        frc2::InstantCommand([this]() { state.elevState = ELEVATOR_STATE::STOWED; }),
        frc2::WaitCommand(1_s),
        frc2::InstantCommand([this]() { state.elevState = ELEVATOR_STATE::ONE; }),
        frc2::WaitCommand(1_s),
        frc2::InstantCommand([this]() { state.elevState = ELEVATOR_STATE::STOWED; }),
        frc2::WaitCommand(1_s),
        frc2::InstantCommand([this]() { state.elevState = ELEVATOR_STATE::TWO; }),
        frc2::WaitCommand(1_s),
        frc2::InstantCommand([this]() { state.elevState = ELEVATOR_STATE::STOWED; }),
        frc2::WaitCommand(1_s),
        frc2::InstantCommand([this]() { state.elevState = ELEVATOR_STATE::THREE; }),
        frc2::WaitCommand(1_s),
        frc2::InstantCommand([this]() { state.elevState = ELEVATOR_STATE::STOWED; }),
        frc2::WaitCommand(1_s),
        frc2::InstantCommand([this]() { state.elevState = ELEVATOR_STATE::FOUR; }),
        frc2::WaitCommand(1_s),
        frc2::InstantCommand([this]() { state.elevState = ELEVATOR_STATE::STOWED; })        
    ).ToPtr();
}

void Scorer::resetState()
{
    state.scoringState = HOLD;
    state.elevState = ELEVATOR_STATE::STOWED;
    state.gamePiece = CORAL;
    state.scopedState = UNSCOPED;
    state.tuning = false;
    state.manualSpeed = 0_V;
    state.algaeSpikeCurrent = 30;
    currentSensor.reset();
    state.hasAlgae = false;
}

bool Scorer::hallEffectSensorActive()
{
    return candi.GetS1Closed().GetValue();
}

void Scorer::init()
{
    table->PutBoolean("Scope Button", false);

    scorerStagingSensor.setMaxDistance(12_in);
    scorerStagingSensor.setThresholdDistance(7.7_cm);
    
    // CANdi init sequence (for reverse hard limit sensor of elevator)
    ctre::phoenix6::configs::CANdiConfiguration candiConfig;
    candiConfig.DigitalInputs.S1CloseState = ctre::phoenix6::signals::S1CloseStateValue::CloseWhenLow;
    candiConfig.DigitalInputs.S1FloatState = ctre::phoenix6::signals::S1FloatStateValue::FloatDetect;
    // @todo LED flash if float detected - map to CANDle LED states like CANCoders
    candi.GetConfigurator().Apply(candiConfig.DigitalInputs);

    // Elevator init sequence
    elevatorMotor->setGearRatios(ELEVATOR_MOTOR_TO_SENSOR, ELEVATOR_SENSOR_TO_MECH);
    elevatorMotor->enableFOC(true);
    elevatorMotor->setForwardLimit(ELEVATOR_FORWARD_LIMIT);
    elevatorMotor->setupReverseHardwareLimit(CANIDs::HALL_EFFECT, ctre::phoenix6::signals::ReverseLimitTypeValue::NormallyOpen);

    valor::PIDF elevatorPID = Constants::Scorer::getElevatorPIDF();
    elevatorPID.maxVelocity = elevatorMotor->getMaxMechSpeed();
    elevatorPID.maxAcceleration = elevatorMotor->getMaxMechSpeed() / Constants::Scorer::getElevMaxVelRampTime();
    elevatorMotor->setPIDF(elevatorPID);
    elevatorMotor->setupCANCoder(
        CANIDs::ELEVATOR_CAN,
        Constants::getElevatorMagnetOffset(), 
        Constants::Scorer::isElevatorClockwise(),
        "baseCAN",
        Constants::Scorer::getElevatorAbsoluteRange()
    );
    elevatorMotor->applyConfig();

    // Scorer init sequence
    scorerMotor->setGearRatios(1, Constants::Scorer::getScorerSensorToMech());
    scorerMotor->enableFOC(true);
    scorerMotor->setCurrentLimits(
        units::ampere_t{60},
        units::ampere_t{60},
        units::ampere_t{45},
        units::second_t{0.5},
        true
    );

    valor::PIDF scorerPID = Constants::Scorer::getScorerPIDF();
    scorerPID.maxVelocity = scorerMotor->getMaxMechSpeed();
    scorerPID.maxAcceleration = scorerMotor->getMaxMechSpeed() / Constants::Scorer::getScorerMaxVelRampTime();
    
    scorerMotor->setPIDF(scorerPID);

    scorerMotor->applyConfig();

    // Zeroing debounce sensor (utilizes CANdi configured hall effect sensor)
    hallEffectDebounceSensor.setGetter([this] { return hallEffectSensorActive();});
    hallEffectDebounceSensor.setRisingEdgeCallback([this] {
        state.hasZeroed = true;
    });

    // Beambreak debounce sensor (on scoring mechanism)
    scorerStagingSensor.setRisingEdgeCallback([this] {
        if (state.gamePiece == GAME_PIECE::CORAL) {
            state.scoringState = HOLD;
            scorerMotor->setSpeed(0_tps);
            scorerMotor->setEncoderPosition(0_tr);
        }
    });

    currentSensor.setSpikeSetpoint(45);
    currentSensor.setGetter([this]() {return scorerMotor->getCurrent().to<double>(); });
    currentSensor.setSpikeCallback([this]() {state.hasAlgae = true;});
    currentSensor.setCacheSize(ALGAE_CACHE_SIZE);
    
    resetState();

    // Must be at the end of init() because the CANdi has to be setup before reading
    state.hasZeroed = hallEffectSensorActive();
}

void Scorer::assessInputs()
{
    // Operator controller section
    if (operatorGamepad == nullptr || !operatorGamepad->IsConnected())
        return;

    if(operatorGamepad->leftTriggerActive()) {
        state.gamePiece = ALGEE;
    } else if(operatorGamepad->rightTriggerActive()) {
        state.gamePiece = CORAL;
    }

    if (operatorGamepad->leftStickYActive()) {
        state.elevState = MANUAL;
        state.manualSpeed = operatorGamepad->leftStickY(2) * 12_V;
    } else if (operatorGamepad->GetYButton()) {
         state.elevState = ELEVATOR_STATE::FOUR;
    } else if (operatorGamepad->GetBButton()) {
        state.elevState = ELEVATOR_STATE::THREE;
    } else if (operatorGamepad->GetAButton()) {
        state.elevState = ELEVATOR_STATE::TWO;
    } else if (operatorGamepad->GetXButton()) {
        state.elevState = ELEVATOR_STATE::ONE;
    } else if (operatorGamepad->DPadRight()) {
        state.elevState = ELEVATOR_STATE::HP;
    }

    // Driver controller section
    if (driverGamepad == nullptr || !driverGamepad->IsConnected())
        return;
    
    if (driverGamepad->leftTriggerActive()) {
        state.scopedState = SCOPED;
    } else {
        state.scopedState = UNSCOPED;
    }

    if (driverGamepad->rightTriggerActive()) {
        state.scoringState = SCORE_STATE::SCORING;
        state.hasAlgae = false;
    } else if (driverGamepad->GetRightBumperButton()) {
        state.scoringState = SCORE_STATE::INTAKING;
    } else {
        state.scoringState = SCORE_STATE::HOLD;
    }
} 

void Scorer::analyzeDashboard()
{
    state.tuning = table->GetBoolean("Scope Button", false);
    state.algaeSpikeCurrent = table->GetNumber("Algae Spike Setpoint", 30);
}

units::meter_t Scorer::convertToMechSpace(units::turn_t turns) 
{
    return units::meter_t{turns * units::meter_t {PULLEY_CIRCUMFERENCE * M_PI}/1_tr} + ELEVATOR_OFFSET;
}

units::turn_t Scorer::convertToMotorSpace(units::meter_t meters)     
{
    return (meters - ELEVATOR_OFFSET) / units::meter_t {PULLEY_CIRCUMFERENCE * M_PI} * 1_tr;
}

void Scorer::assignOutputs()
{
    // if (!state.hasZeroed) {
    //     elevatorMotor->setPower(-3.0_V);
    //     return;
    // }

    //Elevator State Machine
    if (state.elevState == ELEVATOR_STATE::MANUAL) {
        elevatorMotor->setPower(state.manualSpeed + units::volt_t{Constants::getElevKAFF()});
    } else {
        if(state.scopedState == SCOPED || state.tuning){
            state.targetHeight = positionMap.at(state.gamePiece).at(state.elevState);
        } else{
            state.targetHeight = positionMap.at(state.gamePiece).at(HP);
        }
        units::turn_t targetRotations = convertToMotorSpace(state.targetHeight);
        elevatorMotor->setPosition(targetRotations);
    }

    // Scorer State Machine
    if (state.scoringState == SCORE_STATE::SCORING) {
        if (state.gamePiece == GAME_PIECE::ALGEE) {
            scorerMotor->setSpeed(ALGEE_SCORE_SPEED);  
        } else {
            auto it = scoringSpeedMap.find(state.elevState);
            if (it != scoringSpeedMap.end()) {
                scorerMotor->setSpeed(it->second);
            } else {
            // Fallback to the default SCORE_SPEED 
                scorerMotor->setSpeed(SCORE_SPEED);
            }
        }
    }else if (state.hasAlgae && state.gamePiece == GAME_PIECE::ALGEE) {
        scorerMotor->setSpeed(ALGEE_HOLD_SPD);
    } else if (state.scoringState == SCORE_STATE::INTAKING || !scorerStagingSensor.isTriggered()) {
        if(state.gamePiece == GAME_PIECE::ALGEE){
            scorerMotor->setSpeed(ALGEE_INTAKE_SPEED);
        } else{
            scorerMotor->setSpeed(CORAL_INTAKE_SPEED);
        }
    } else{
        // HOLD the coral at a specific position
        // @todo check inversion
        scorerMotor->setPosition(getIntakeTurns());
    }
}


void Scorer::InitSendable(wpi::SendableBuilder& builder)
{

    builder.SetSmartDashboardType("Subsystem");
    builder.AddDoubleProperty(
        "Desired Position: Elevator (in)",
        [this] { return units::inch_t{state.targetHeight}.value(); },
        nullptr
    );
    builder.AddDoubleProperty(
        "Actual Position: Elevator (in)",
        [this] { return units::inch_t{convertToMechSpace(elevatorMotor->getPosition())}.value(); },
        nullptr
    );
    builder.AddDoubleProperty(
        "State: Scoring",
        [this] { return state.scoringState; },
        nullptr
    );
    builder.AddIntegerProperty(
        "State: Gamepiece",
        [this] { return state.gamePiece; },
        nullptr
    );
    builder.AddIntegerProperty(
        "State: Elevator Level",
        [this] { return state.elevState; },
        nullptr
    );
    builder.AddBooleanProperty(
        "State: Scoped",
        [this] { return state.scopedState; },
        nullptr
    );
    builder.AddBooleanProperty(
        "State: Tuning",
        [this] { return state.tuning; },
        nullptr
    );
    builder.AddDoubleProperty(
        "Desired Speed: Scorer",
        [this] {
            auto speed = scoringSpeedMap.find(state.elevState);
            if (speed == scoringSpeedMap.end()) return 0.0;
            return speed->second.to<double>();
        },
        nullptr
    );

    builder.AddDoubleProperty(
        "MANUAL SPEED ELEVATOR",
        [this] { return state.manualSpeed.to<double>(); },
        nullptr
    );
    builder.AddBooleanProperty(
        "Has Algae",
        [this] {return state.hasAlgae;},
        nullptr
    );
    
}



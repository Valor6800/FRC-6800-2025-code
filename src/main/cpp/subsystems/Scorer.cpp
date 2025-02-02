#include "Constants.h"
#include "subsystems/Scorer.h"
#include "valkyrie/controllers/NeutralMode.h"
#include "valkyrie/controllers/PIDF.h"
#include <iostream>

#include <pathplanner/lib/auto/NamedCommands.h>

#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>

#include <frc/DriverStation.h>

#define ELEV_K_P 10
#define ELEV_K_ERROR units::angle::turn_t (0)
#define ELEV_K_AFF 0.5
#define ELEVATOR_SENSOR_TO_MECH 8.02f

#define SCORER_K_P 0.5
#define SCORER_K_S 0.45
#define INTAKE_SPEED -5_tps
#define SCORE_SPEED -11_tps

#define ELEVATOR_FORWARD_LIMIT 5.75_tr
#define ELEVATOR_OFFSET 3_in

#define ELEVATOR_MOTOR_TO_SENSOR 1.0f
#define SCORER_SENSOR_TO_MECH 1.66666f
#define PULLEY_CIRCUMFERENCE 1.432_in

Scorer::Scorer(frc::TimedRobot *_robot) :
    valor::BaseSubsystem(_robot, "Scorer"),
    scorerDebounceSensor(_robot, "ScorerDebounce"),
    hallEffectDebounceSensor(_robot, "HallEffectDebounce"),
    candi(CANIDs::HALL_EFFECT, "baseCAN"),
    elevatorMotor(new valor::PhoenixController(valor::PhoenixControllerType::KRAKEN_X60, CANIDs::ELEV_WHEEL, valor::NeutralMode::Brake, true, "baseCAN")),
    scorerMotor(new valor::PhoenixController(valor::PhoenixControllerType::FALCON_FOC, CANIDs::SCORER_WHEEL, valor::NeutralMode::Brake, false, "baseCAN")),
    frontRangeSensor(_robot, "Front Lidar Sensor", CANIDs::FRONT_LIDAR_SENSOR),
    scorerStagingSensor(_robot, "Scorer Staging Sensor", CANIDs::STAGING_LIDAR_SENSOR, "baseCAN")
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);

    pathplanner::NamedCommands::registerCommand("OUTTAKE, INTAKE, HOLD", std::move(
        frc2::SequentialCommandGroup(
            frc2::InstantCommand(
                [this]() {
                    state.scoringState = Scorer::SCORING_SPEED::SCORING;
                }
            ),
            frc2::WaitCommand(5_s),
            frc2::InstantCommand(
                [this]() {
                    state.scoringState = Scorer::SCORING_SPEED::INTAKING;
                }
            ),
            frc2::WaitCommand(5_s),
            frc2::InstantCommand(
                [this]() {
                    state.scoringState = Scorer::SCORING_SPEED::HOLD;
                }
            )
        )
    ).ToPtr());
    pathplanner::NamedCommands::registerCommand("Intaking", std::move(
        frc2::SequentialCommandGroup(
            frc2::InstantCommand(
                [this]() {
                    
                    state.scoringState = Scorer::SCORING_SPEED::INTAKING;
                }
            )
        )
    ).ToPtr());
    
    pathplanner::NamedCommands::registerCommand("Enable Scorer", std::move(
        frc2::SequentialCommandGroup(
            frc2::InstantCommand(
                [this]() {
                    
                    state.scoringState = Scorer::SCORING_SPEED::SCORING;
                }
            )
        )
    ).ToPtr());
     pathplanner::NamedCommands::registerCommand("DISABLE Scorer", std::move(
        frc2::SequentialCommandGroup(
            frc2::InstantCommand(
                [this]() {
                    state.scoringState = Scorer::SCORING_SPEED::HOLD;
                }
            )
        )
    ).ToPtr());
    pathplanner::NamedCommands::registerCommand("TROUGH POSITION", std::move(
        frc2::SequentialCommandGroup(
            frc2::InstantCommand(
                [this]() {
                    state.scopedState = Scorer::SCOPED_STATE::SCOPED;
                    state.elevState = Scorer::ELEV_LVL::ONE;
                    state.gamePiece = Scorer::GAME_PIECE::CORAL;
                }
            )
        )
    ).ToPtr());
    pathplanner::NamedCommands::registerCommand("LEVEL TWO POSITION", std::move(
        frc2::SequentialCommandGroup(
            frc2::InstantCommand(
                [this]() {
                    state.scopedState = Scorer::SCOPED_STATE::SCOPED;
                    state.elevState = Scorer::ELEV_LVL::TWO;
                    state.gamePiece = Scorer::GAME_PIECE::CORAL;
                }
            )
        )
    ).ToPtr());
    pathplanner::NamedCommands::registerCommand("LEVEL THREE POSITION", std::move(
        frc2::SequentialCommandGroup(
            frc2::InstantCommand(
                [this]() {
                    state.scopedState = Scorer::SCOPED_STATE::SCOPED;
                    state.elevState = Scorer::ELEV_LVL::THREE;
                    state.gamePiece = Scorer::GAME_PIECE::CORAL;
                }
            )
        )
    ).ToPtr());
    pathplanner::NamedCommands::registerCommand("LEVEL FOUR POSITION", std::move(
        frc2::SequentialCommandGroup(
            frc2::InstantCommand(
                [this]() {
                    state.scopedState = Scorer::SCOPED_STATE::SCOPED;
                    state.elevState = Scorer::ELEV_LVL::FOUR;
                    state.gamePiece = Scorer::GAME_PIECE::CORAL;
                }
            )
        )
    ).ToPtr());
    pathplanner::NamedCommands::registerCommand("STOWED POSITION", std::move(
        frc2::SequentialCommandGroup(
            frc2::InstantCommand(
                [this]() {
                    state.scopedState = Scorer::SCOPED_STATE::SCOPED;
                    state.elevState = Scorer::ELEV_LVL::STOWED;
                    state.gamePiece = Scorer::GAME_PIECE::CORAL;
                }
            )
        )
    ).ToPtr());
    pathplanner::NamedCommands::registerCommand("HP", std::move(
        frc2::SequentialCommandGroup(
            frc2::InstantCommand(
                [this]() {
                    state.scopedState = Scorer::SCOPED_STATE::SCOPED;
                    state.elevState = Scorer::ELEV_LVL::HP;
                    state.gamePiece = Scorer::GAME_PIECE::CORAL;
                }
            )
        )
    ).ToPtr());

    init();
}

frc2::CommandPtr Scorer::createScoringSequence() {
    return frc2::SequentialCommandGroup(
        frc2::InstantCommand([this]() { state.scoringState = Scorer::SCORING_SPEED::SCORING; }),
        frc2::WaitCommand(5_s),
        frc2::InstantCommand([this]() { state.scoringState = Scorer::SCORING_SPEED::INTAKING; }),
        frc2::WaitCommand(5_s),
        frc2::InstantCommand([this]() { state.scoringState = Scorer::SCORING_SPEED::HOLD; })
    ).ToPtr();
}

frc2::CommandPtr Scorer::createElevatorSequence() {
    return frc2::SequentialCommandGroup(
        frc2::InstantCommand([this]() { 
            state.scopedState = Scorer::SCOPED_STATE::SCOPED;
            state.gamePiece = Scorer::GAME_PIECE::CORAL;
        }),
        frc2::InstantCommand([this]() { state.elevState = Scorer::ELEV_LVL::STOWED; }),
        frc2::WaitCommand(1_s),
        frc2::InstantCommand([this]() { state.elevState = Scorer::ELEV_LVL::ONE; }),
        frc2::WaitCommand(1_s),
        frc2::InstantCommand([this]() { state.elevState = Scorer::ELEV_LVL::STOWED; }),
        frc2::WaitCommand(1_s),
        frc2::InstantCommand([this]() { state.elevState = Scorer::ELEV_LVL::TWO; }),
        frc2::WaitCommand(1_s),
        frc2::InstantCommand([this]() { state.elevState = Scorer::ELEV_LVL::STOWED; }),
        frc2::WaitCommand(1_s),
        frc2::InstantCommand([this]() { state.elevState = Scorer::ELEV_LVL::THREE; }),
        frc2::WaitCommand(1_s),
        frc2::InstantCommand([this]() { state.elevState = Scorer::ELEV_LVL::STOWED; }),
        frc2::WaitCommand(1_s),
        frc2::InstantCommand([this]() { state.elevState = Scorer::ELEV_LVL::FOUR; }),
        frc2::WaitCommand(1_s),
        frc2::InstantCommand([this]() { state.elevState = Scorer::ELEV_LVL::STOWED; })        
    ).ToPtr();
}

void Scorer::resetState()
{
    state.scoringState = SCORING_SPEED::HOLD;
    state.elevState = ELEV_LVL::MANUAL;
    state.gamePiece = CORAL;
    state.scopedState = UNSCOPED;
    state.tuning = false;
    state.manualSpeed = 0_V;
}

bool Scorer::hallEffectSensorActive()
{
    return candi.GetS1Closed().GetValue();
}

void Scorer::init()
{
    table->PutBoolean("Scope Button", false);
    
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

    valor::PIDF elevatorPID;
    elevatorPID.maxVelocity = elevatorMotor->getMaxMechSpeed();
    elevatorPID.maxAcceleration = elevatorMotor->getMaxMechSpeed()/(1.0_s/5);
    elevatorPID.P =ELEV_K_P ;
    elevatorPID.error = ELEV_K_ERROR;
    elevatorPID.aFF = ELEV_K_AFF;
    elevatorPID.aFFType = valor::FeedForwardType::LINEAR;
    elevatorMotor->setPIDF(elevatorPID, 0);
    elevatorMotor->applyConfig();

    // Scorer init sequence
    scorerMotor->setGearRatios(1, SCORER_SENSOR_TO_MECH);
    scorerMotor->enableFOC(true);

    valor::PIDF scorerPID;
    scorerPID.maxVelocity = scorerMotor->getMaxMechSpeed();
    scorerPID.maxAcceleration = scorerMotor->getMaxMechSpeed()/(1.0_s/2);
    scorerPID.P = SCORER_K_P;
    scorerPID.S = SCORER_K_S;
    
    scorerMotor->setPIDF(scorerPID, 0);
    scorerMotor->applyConfig();

    // Zeroing debounce sensor (utilizes CANdi configured hall effect sensor)
    hallEffectDebounceSensor.setGetter([this] { return hallEffectSensorActive();});
    hallEffectDebounceSensor.setRisingEdgeCallback([this] {
        state.hasZeroed = true;
    });

    posMap[GAME_PIECE::CORAL][ELEV_LVL::STOWED] = units::meter_t(3_in);
    posMap[GAME_PIECE::CORAL][ELEV_LVL::HP] = units::meter_t(5.069_in);
    posMap[GAME_PIECE::CORAL][ELEV_LVL::ONE] = units::meter_t(13.57_in);
    posMap[GAME_PIECE::CORAL][ELEV_LVL::TWO] = units::meter_t(17.0_in);
    posMap[GAME_PIECE::CORAL][ELEV_LVL::THREE] = units::meter_t(25.05_in);
    posMap[GAME_PIECE::CORAL][ELEV_LVL::FOUR] = units::meter_t(5_in);

    posMap[GAME_PIECE::ALGEE][ELEV_LVL::ONE] = units::meter_t(5_in);
    posMap[GAME_PIECE::ALGEE][ELEV_LVL::TWO] = units::meter_t(5_in);
    posMap[GAME_PIECE::ALGEE][ELEV_LVL::THREE] = units::meter_t(5_in);
    posMap[GAME_PIECE::ALGEE][ELEV_LVL::FOUR] = units::meter_t(5_in);
    
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
         state.elevState = ELEV_LVL::FOUR;
    } else if (operatorGamepad->GetBButton()) {
        state.elevState = ELEV_LVL::THREE;
    } else if (operatorGamepad->GetAButton()) {
        state.elevState = ELEV_LVL::TWO;
    } else if (operatorGamepad->GetXButton()) {
        state.elevState = ELEV_LVL::ONE;
    } else if (operatorGamepad->DPadRight()) {
        state.elevState = ELEV_LVL::HP;
    }

    // Driver controller section
    if (driverGamepad == nullptr || !driverGamepad->IsConnected())
        return;
    
    if (driverGamepad->leftTriggerActive()) {
        state.scopedState = SCOPED;
    } else {
        state.scopedState = UNSCOPED;
    }

    if (driverGamepad->GetRightBumperButton()) {
        state.scoringState = SCORING_SPEED::INTAKING;
    } else if (driverGamepad->rightTriggerActive()) {
        state.scoringState = SCORING_SPEED::SCORING;
    } else {
        state.scoringState = SCORING_SPEED::HOLD;
    }
} 

    void Scorer::stageCoral(){
        scorerMotor->setSpeed(-1_tps); 
        if(scorerMotor->getCurrent()>18_A){
            scorerMotor->setPosition(units::angle::turn_t{-0.5} + scorerMotor->getPosition());
            state.staged= STAGING_SEQ::DONE_STAGE;
        }
             
    }


void Scorer::analyzeDashboard()
{
    state.tuning = table->GetBoolean("Scope Button", false);
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
        if (state.hasZeroed) {
            if (state.elevState == ELEV_LVL::MANUAL) {
                elevatorMotor->setPower(state.manualSpeed);
            } else {
                if(state.scopedState == SCOPED || state.tuning){
                    state.targetHeight = posMap[state.gamePiece][state.elevState];
                } else{
                    if(state.elevState == HP){
                        state.targetHeight = posMap[CORAL][HP];
                    }else{
                        state.targetHeight = posMap[CORAL][STOWED];
                    }
                }
                units::turn_t targetRotations = convertToMotorSpace(state.targetHeight);
                elevatorMotor->setPosition(targetRotations);
                
            }
        } else {
            elevatorMotor->setPower(-3.0_V);
        }


      
        if (state.scoringState == SCORING_SPEED::INTAKING) {
    scorerMotor->setSpeed(INTAKE_SPEED);
} 
    else if (state.scoringState == SCORING_SPEED::SCORING) {
        auto it = scoringSpeedMap.find(state.elevState);
        if (it != scoringSpeedMap.end()) {
            scorerMotor->setSpeed(it->second);
        } else {
            // Fallback to the default SCORE_SPEED
            scorerMotor->setSpeed(SCORE_SPEED);
        }
    } 
    else if (state.staged == STAGING_SEQ::STAGING) {
        std::cout << "Hello World\n\n\n";
        stageCoral();
    } 
    else if (state.staged == STAGING_SEQ::DONE_STAGE) {} 
    else if (state.elevState == ELEV_LVL::HP) {
        scorerMotor->setSpeed(INTAKE_SPEED);
    } 
    else {
        scorerMotor->setSpeed(0_tps);
        std::cout << "zero" << std::endl;
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
        "State: Hall Effect Active",
        [this] { return hallEffectSensorActive(); },
        nullptr
    );
    builder.AddBooleanProperty(
        "State: Tuning",
        [this] { return state.tuning; },
        nullptr
    );
    builder.AddDoubleProperty(
        "Desired Speed: Scorer",
        [this] { return scoringSpeedMap.find(state.elevState)->second.to<double>(); },
        nullptr
    );
    builder.AddBooleanProperty(
        "isDetectingCANRANGE",
        [this] { return scorerStagingSensor.isDetected(); },
        nullptr
    );
    builder.AddIntegerProperty(
            "Staging Seq",
            [this] {return state.staged;},
            nullptr
        );
}



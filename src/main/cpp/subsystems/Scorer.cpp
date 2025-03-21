#include "Constants.h"
#include "subsystems/Scorer.h"
#include "units/current.h"
#include "units/time.h"
#include "units/length.h"
#include "units/math.h"
#include "Drivetrain.h"
#include "valkyrie/controllers/NeutralMode.h"
#include "valkyrie/controllers/PIDF.h"
#include <iostream>

#include <pathplanner/lib/auto/NamedCommands.h>

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
#define ELEVATOR_HEALTH_LIMIT 0.05_tr

#define ELEVATOR_MOTOR_TO_SENSOR 8.02f
#define PULLEY_CIRCUMFERENCE 1.432_in

#define ALGAE_CACHE_SIZE 1000

#define VIABLE_ELEVATOR_THRESHOLD 0.02_m
#define VIABLE_DUNK_DISTANCE 0.28_m
#define VIABLE_ELEVATOR_DISTANCE 0.8_m //consider the offset of the canrange from the front of robot which is 8 inches


using namespace Constants::Scorer;

Scorer::Scorer(frc::TimedRobot *_robot, Drivetrain *_drivetrain, valor::CANdleSensor* _leds) :
    valor::BaseSubsystem(_robot, "Scorer"),
    hallEffectDebounceSensor(_robot, "HallEffectDebounce"),
    candi(CANIDs::HALL_EFFECT, "baseCAN"),
    elevatorMotor(new valor::PhoenixController(valor::PhoenixControllerType::KRAKEN_X60, CANIDs::ELEV_WHEEL, valor::NeutralMode::Brake, elevatorMotorInverted(), "baseCAN")),
    scorerMotor(new valor::PhoenixController(Constants::getScorerMotorType(), CANIDs::SCORER_WHEEL, valor::NeutralMode::Brake, scorerMotorInverted(), "baseCAN")),
    scorerStagingSensor(_robot, "Scorer Staging Sensor", CANIDs::STAGING_LIDAR_SENSOR, "baseCAN"),
    currentSensor(_robot, "Algae Current Sensor"),
    positionMap{std::move(getPositionMap())},
    scoringSpeedMap{std::move(getScoringSpeedMap())},
    drivetrain(_drivetrain),
    leds(_leds)
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
    pathplanner::NamedCommands::registerCommand("Algae1", std::move(
        frc2::SequentialCommandGroup(
            frc2::InstantCommand(
                [this](){
                    state.scopedState = SCOPED_STATE::SCOPED;
                    state.gamePiece = GAME_PIECE::ALGEE;
                    state.elevState = ELEVATOR_STATE::TWO;
                    state.scoringState = SCORE_STATE::INTAKING;
                }
            )
        )
    ).ToPtr());
    pathplanner::NamedCommands::registerCommand("Barge", std::move(
        frc2::SequentialCommandGroup(
            frc2::InstantCommand(
                [this](){
                    state.gamePiece = GAME_PIECE::ALGEE;
                    state.elevState = ELEVATOR_STATE::FOUR;
                }
            )
        )
    ).ToPtr());

    pathplanner::NamedCommands::registerCommand("Align", std::move(
        frc2::SequentialCommandGroup(
            frc2::FunctionalCommand(
                [&]{ // on begin
                    drivetrain->state.startTimestamp = frc::Timer::GetFPGATimestamp();
                    state.gamePiece = GAME_PIECE::CORAL;
                    state.scopedState = SCOPED_STATE::SCOPED;
                    state.elevState = ELEVATOR_STATE::TWO;
                    drivetrain->state.dir = RIGHT;
                },
                [&]{ // on execute
                    drivetrain->state.alignToTarget = true;
                    drivetrain->xAlign = true;
                    // frc::ChassisSpeeds speeds = frc::ChassisSpeeds{0.5_mps, 0.0_mps, 0.0_rad_per_s};
                    // drive(0.5_mps, 0.0_mps, 0.0_rad_per_s, true);
                },
                [&](bool){ // on end
                    drivetrain->state.alignToTarget = false;
                    drivetrain->xAlign = false;
                },
                [&]{ // is Finished
                    return state.scoringState == SCORE_STATE::SCORING;
                },
                {}
            )
        )
    ).ToPtr());


        table->PutNumber("Viable Dunk Distance (m)", VIABLE_DUNK_DISTANCE.value());
        table->PutNumber("Viable Elevator Distance (m)", VIABLE_ELEVATOR_DISTANCE.value());

    init();

    visualizerStage1 = nt::NetworkTableInstance::GetDefault().GetStructTopic<frc::Pose3d>("LiveWindow/BaseSubsystem/Scorer/Stage1Height").Publish();
    visualizerStage2 = nt::NetworkTableInstance::GetDefault().GetStructTopic<frc::Pose3d>("LiveWindow/BaseSubsystem/Scorer/Stage2Height").Publish();
    visualizerStage3 = nt::NetworkTableInstance::GetDefault().GetStructTopic<frc::Pose3d>("LiveWindow/BaseSubsystem/Scorer/Stage3Height").Publish();
}

frc2::CommandPtr Scorer::scorerPitSequenceStage(GAME_PIECE gamePiece, ELEVATOR_STATE elevState) {
    return frc2::cmd::Deadline(
        frc2::cmd::Wait(3_s),
        frc2::FunctionalCommand{
            [this, elevState, gamePiece] {
                state.elevState = elevState;
                state.gamePiece = gamePiece;
                std::string msg = "Elevator going to ";
                if (gamePiece == ALGEE) msg += "algae ";
                else if (gamePiece == CORAL) msg += "coral ";
                if (elevState == STOWED) msg += "stowed";
                else if (elevState == HP) msg += "human player";
                else if (elevState == ONE) msg += "L1";
                else if (elevState == TWO) msg += "L2";
                else if (elevState == THREE) msg += "L3";
                else if (elevState == FOUR) msg += "L4";
                elevatorStage.SetText(msg);
                elevatorStage.Set(true);
                elevatorBButtonWait.Set(false);
            },
            [this] {
                units::inch_t currentPos = convertToMechSpace(elevatorMotor->getPosition());
                units::inch_t targetPos = positionMap[state.gamePiece][state.elevState];
                elevatorPositionSuccess.Set(units::math::abs(targetPos - currentPos) < 0.25_in);
                elevatorPositionFail.Set(!elevatorPositionSuccess.Get());
            },
            [this](bool) {
                elevatorPositionFail.Set(false);
                elevatorPositionSuccess.Set(false);
            },
            [this] { return false; }
        }.ToPtr()
    );
}

frc2::CommandPtr Scorer::scorerPitSequence() {
    return frc2::cmd::Sequence(
        frc2::cmd::RunOnce([this] { state.scopedState = SCOPED; }),
        scorerPitSequenceStage(GAME_PIECE::CORAL, ELEVATOR_STATE::HP),
        scorerPitSequenceStage(GAME_PIECE::CORAL, ELEVATOR_STATE::ONE),
        frc2::cmd::RunOnce([this] { elevatorBButtonWait.Set(true); }),
        frc2::cmd::WaitUntil([this] { return driverGamepad->GetBButton(); }),
        frc2::cmd::RunOnce([this] { state.scoringState = SCORING; }),
        frc2::cmd::RunOnce([this] { elevatorBButtonWait.Set(true); }),
        frc2::cmd::WaitUntil([this] { return driverGamepad->GetBButton(); }),
        scorerPitSequenceStage(GAME_PIECE::CORAL, ELEVATOR_STATE::TWO),
        scorerPitSequenceStage(GAME_PIECE::CORAL, ELEVATOR_STATE::THREE),
        scorerPitSequenceStage(GAME_PIECE::CORAL, ELEVATOR_STATE::FOUR),
        scorerPitSequenceStage(GAME_PIECE::CORAL, ELEVATOR_STATE::STOWED),
        frc2::cmd::RunOnce([this] {
            state.scoringState = HOLD;
            elevatorBButtonWait.Set(true);
        }),
        frc2::cmd::WaitUntil([this] { return driverGamepad->GetBButton(); }),
        scorerPitSequenceStage(GAME_PIECE::ALGEE, ELEVATOR_STATE::STOWED),
        frc2::cmd::RunOnce([this] { state.scoringState = SCORING; }),
        frc2::cmd::Wait(3_s),
        scorerPitSequenceStage(GAME_PIECE::ALGEE, ELEVATOR_STATE::FOUR),
        frc2::cmd::RunOnce([this] {
            resetState();
            elevatorStage.Set(false);
        })
    );
}

void Scorer::resetState()
{
    state.scoringState = HOLD;
    state.elevState = ELEVATOR_STATE::HP;
    state.gamePiece = CORAL;
    state.scopedState = UNSCOPED;
    state.manualSpeed = 0_V;
    state.algaeSpikeCurrent = 30;
    currentSensor.reset();
    state.hasAlgae = false;
    state.protectChin = false;
    state.autoDunkEnabled = true;
}

bool Scorer::hallEffectSensorActive()
{
    return candi.GetS1Closed().GetValue();
}

bool Scorer::cancoderSensorBad()
{
    return (elevatorMotor->GetFault_BadMagnet() || (elevatorMotor->getMagnetHealth() == ctre::phoenix6::signals::MagnetHealthValue::Magnet_Red) || elevatorMotor->getMagnetHealth() == ctre::phoenix6::signals::MagnetHealthValue::Magnet_Invalid);
       
}


void Scorer::init()
{
    table->PutBoolean("Scope Button", false);
     table->PutBoolean("Auto Dunk Enabled", true);

    scorerStagingSensor.setMaxDistance(12_in);
    scorerStagingSensor.setThresholdDistance(7.7_cm);
    
    table->PutNumber("Elevator Threshold (m)", VIABLE_ELEVATOR_THRESHOLD.value());
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
    elevatorMotor->setCurrentLimits(
        units::ampere_t{100},
        units::ampere_t{60},
        units::ampere_t{45},
        units::second_t{0.5},
        true
    );

    // Scorer init sequence
    scorerMotor->setGearRatios(1, Constants::Scorer::getScorerSensorToMech());
    scorerMotor->enableFOC(true);
    scorerMotor->setCurrentLimits(
        units::ampere_t{100},
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
        if (cancoderSensorBad())
        {
            elevatorMotor->setEncoderPosition(0_tr);
        }
        state.hasZeroed = true;
        absSensorCorrect = elevatorMotor->getAbsEncoderPosition();

        std::cout << "\nZeored absEncoderPosition: " << absSensorCorrect.value() << std::endl;
        std::cout << "\nZeroed elevator position(in): " << convertToMechSpace(elevatorMotor->getPosition()).value() << std::endl;
    });

    // Beambreak debounce sensor (on scoring mechanism)
    scorerStagingSensor.setRisingEdgeCallback([this] {
        if (state.gamePiece == GAME_PIECE::CORAL) {
            state.scoringState = HOLD;
            scorerMotor->setSpeed(0_tps);
            scorerMotor->setEncoderPosition(0_tr);
        }
    });

     scorerStagingSensor.setFallingEdgeCallback([this] {
        if (state.gamePiece == GAME_PIECE::CORAL) {
            state.protectChin = true;
           
        }
    });

    currentSensor.setSpikeSetpoint(45);
    currentSensor.setGetter([this]() {return scorerMotor->getCurrent().to<double>(); });
    currentSensor.setSpikeCallback([this]() {state.hasAlgae = true;});
    currentSensor.setCacheSize(ALGAE_CACHE_SIZE);

    resetState();
    table->PutBoolean("Auto Dunk Disabled", false);
    
    // Must be at the end of init() because the CANdi has to be setup before reading   
    if (cancoderSensorBad()){
        state.hasZeroed = hallEffectSensorActive();
    } else{
        state.hasZeroed = true;
    }
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
    
    if (driverGamepad->leftTriggerActive() || driverGamepad->rightTriggerActive() || driverGamepad->GetLeftBumperButton()) {
        state.scopedState = SCOPED;
    } else {
        state.scopedState = UNSCOPED;
    }


    if (driverGamepad->GetRightBumperButton()) {
        state.scoringState = SCORE_STATE::SCORING;
        state.hasAlgae = false;
    } else {
        state.scoringState = SCORE_STATE::HOLD;
    }
} 

void Scorer::analyzeDashboard()
{

    auto cancoder = elevatorMotor->getCANCoder();
    leds->setLED(LEDConstants::LED_POS_ELEVATOR, valor::CANdleSensor::cancoderMagnetHealthGetter(cancoder));
    int elevatorHealthLED = std::abs(absSensorCorrect.value()) < ELEVATOR_HEALTH_LIMIT.value() ? valor::CANdleSensor::GREEN : valor::CANdleSensor::RED;
    leds->setLED(LEDConstants::LED_POS_ELEVATOR_NOT_ZERO, elevatorHealthLED);

    if (candi.GetS1State().GetValue() == ctre::phoenix6::signals::S1StateValue::Floating) {
        leds->setLED(LEDConstants::LED_POS_CANDI, valor::CANdleSensor::RED);
    } else {
        leds->setLED(LEDConstants::LED_POS_CANDI, valor::CANdleSensor::GREEN);
    }

    bool disableAutoDunk = table->GetBoolean("Auto Dunk Disabled", false);

    if (state.scoringState != SCORE_STATE::SCORING || (state.elevState == ELEVATOR_STATE::ONE && state.gamePiece == GAME_PIECE::CORAL)){
        state.protectChin = false;
    }

    units::meter_t elevatorError = units::math::fabs(convertToMechSpace(elevatorMotor->getPosition()) - positionMap[state.gamePiece][state.elevState]);
    elevatorWithinThreshold = elevatorError.value() < table->GetNumber("Elevator Threshold (m)", VIABLE_ELEVATOR_THRESHOLD.value());
    
    if (
        (state.autoDunkEnabled && !disableAutoDunk) &&
        drivetrain->isSpeedBelowThreshold() &&
        drivetrain->withinXRange((units::meter_t) table->GetNumber("Viable Dunk Distance (m)", VIABLE_DUNK_DISTANCE.value())) &&
        drivetrain->withinYRange() &&
        state.scopedState == SCOPED_STATE::SCOPED &&
        elevatorWithinThreshold &&
        state.gamePiece == GAME_PIECE::CORAL &&
        (state.elevState == TWO || state.elevState == THREE || state.elevState == FOUR)
    ) {
        state.scoringState = SCORE_STATE::SCORING;
    }

    int botColor = state.gamePiece == GAME_PIECE::CORAL ? valor::CANdleSensor::VALOR_GOLD : valor::CANdleSensor::VALOR_PURPLE;
    int midColor = scorerStagingSensor.isTriggered() && state.gamePiece == GAME_PIECE::CORAL ? valor::CANdleSensor::GREEN : valor::CANdleSensor::RED;
    int topColor = state.scoringState == SCORE_STATE::SCORING ? valor::CANdleSensor::RED :
        (state.scopedState == SCOPED_STATE::SCOPED && elevatorWithinThreshold ? valor::CANdleSensor::GREEN : valor::CANdleSensor::VALOR_GOLD);
    leds->setColor(0, botColor);
    leds->setColor(1, midColor);
    leds->setColor(2, topColor);
    leds->setColor(3, topColor);
    leds->setColor(4, midColor);
    leds->setColor(5, botColor);

    state.algaeSpikeCurrent = table->GetNumber("Algae Spike Setpoint", 30);
    drivetrain->setGamePieceInRobot(state.gamePiece);
    drivetrain->state.elevState = state.elevState;

    units::turn_t height = elevatorMotor->getPosition();
    visualizerStage1.Set(frc::Pose3d{0_m, 0_m, convertToMechSpace(height), frc::Rotation3d()});
    visualizerStage2.Set(frc::Pose3d{0_m, 0_m, convertToMechSpace(height * 2), frc::Rotation3d()});
    visualizerStage3.Set(frc::Pose3d{0_m, 0_m, convertToMechSpace(height * 3), frc::Rotation3d()});
}

void Scorer::setScopedState(SCOPED_STATE st){
    state.scopedState = st;
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
    if (!state.hasZeroed) {
        elevatorMotor->setPower(-3.0_V);
        return;
    }

    //Elevator State Machine
    if (state.elevState == ELEVATOR_STATE::MANUAL) {
        elevatorMotor->setPower(state.manualSpeed + units::volt_t{Constants::getElevKAFF()});
    } else {
        if (state.scopedState == SCOPED &&
            (
                state.gamePiece == GAME_PIECE::ALGEE ||
                (state.gamePiece == GAME_PIECE::CORAL && drivetrain->withinXRange((units::meter_t)table->GetNumber("Viable Elevator Distance (m)", VIABLE_ELEVATOR_DISTANCE.value())))
            )
        ) {
            state.targetHeight = positionMap.at(state.gamePiece).at(state.elevState);
            if (state.elevState == ELEVATOR_STATE::ONE && state.gamePiece == GAME_PIECE::CORAL && state.scoringState == SCORE_STATE::SCORING) {
                state.targetHeight += 4_in;
            }
        } else{
            if (scorerStagingSensor.isTriggered()) {
                state.targetHeight = positionMap.at(state.gamePiece).at(STOWED);
            } else {
                state.targetHeight = positionMap.at(state.gamePiece).at(HP);
            }
        }
        units::turn_t targetRotations = convertToMotorSpace(state.targetHeight);
        elevatorMotor->setPosition(targetRotations);
    }

  // Scorer State Machine
if (state.scoringState == SCORE_STATE::SCORING) {
    if (state.gamePiece == GAME_PIECE::ALGEE) {
        scorerMotor->setSpeed(ALGEE_SCORE_SPEED);
    } else if (!state.protectChin) {
        auto it = scoringSpeedMap.find(state.elevState);
        if (it != scoringSpeedMap.end()) {
            scorerMotor->setSpeed(it->second);
        } else {
            scorerMotor->setSpeed(SCORE_SPEED);
        }
    } else {
        scorerMotor->setSpeed(0_tps);
    }
    } else if (state.hasAlgae && state.gamePiece == GAME_PIECE::ALGEE) {
        scorerMotor->setSpeed(ALGEE_HOLD_SPD);
    } else if (!scorerStagingSensor.isTriggered()) {
        if (state.gamePiece == GAME_PIECE::ALGEE) {
            scorerMotor->setSpeed(ALGEE_INTAKE_SPEED);
        } else {
            scorerMotor->setSpeed(CORAL_INTAKE_SPEED);
        }
    } else {
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
    builder.AddBooleanProperty(
        "CORAL",
        [this] {return state.gamePiece == GAME_PIECE::CORAL;},
        nullptr
    );
    builder.AddBooleanProperty(
        "ALGAE",
        [this] {return state.gamePiece == GAME_PIECE::ALGEE;},
        nullptr
    );

    builder.AddBooleanProperty(
        "CHIN PROTECTED?",
        [this] {return state.protectChin;},
        nullptr
    );

    builder.AddBooleanProperty(
        "ZEROED?",
        [this] {return state.hasZeroed;},
        nullptr
    );

    builder.AddBooleanProperty(
        "ABSOLUTE POSITION NO OFFSET?",
        [this] {return absSensorCorrect.value();},
        nullptr
    );
}



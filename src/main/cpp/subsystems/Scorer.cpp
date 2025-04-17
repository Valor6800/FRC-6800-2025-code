#include "Constants.h"
#include "subsystems/Scorer.h"
#include "units/angular_velocity.h"
#include "units/current.h"
#include "units/time.h"
#include "units/length.h"
#include "units/math.h"
#include "Drivetrain.h"
#include "units/velocity.h"
#include "subsystems/Climber.h"
#include "valkyrie/controllers/NeutralMode.h"
#include "valkyrie/controllers/PIDF.h"
#include <bitset>
#include <iostream>

#include <pathplanner/lib/auto/NamedCommands.h>

#include <frc/DriverStation.h>

#define ELEV_K_ERROR units::angle::turn_t (0)
#define ELEVATOR_SENSOR_TO_MECH 1.0f

#define CORAL_INTAKE_SPEED 30_tps //5
#define CORAL_GROUND_INTAKE_SPEED -20_tps //5
#define ALGEE_INTAKE_SPEED -16_tps
#define SCORE_SPEED 20_tps
#define ALGEE_SCORE_SPEED 11.5_tps
#define ALGEE_HOLD_SPD -1.25_tps
#define CORAL_HOLD_SPD -0.25_tps

#define ELEVATOR_FORWARD_LIMIT 6_tr
#define ELEVATOR_OFFSET 3_in
#define ELEVATOR_MAGNET_OFFSET 0.321_tr
#define ELEVATOR_JERK 40_tr_per_s_cu
#define ELEVATOR_HEALTH_LIMIT 0.04_tr

#define ELEVATOR_MOTOR_TO_SENSOR 8.02f
#define PULLEY_CIRCUMFERENCE 1.432_in

#define PIVOT_MOTOR_TO_SENSOR 54.0f
#define PIVOT_SENSOR_TO_MECH 1.0f
#define SECOND_SCORER_FORWARD_LIMIT 0.65_tr
#define SECOND_SCORER_REVERSE_LIMIT 0.15_tr
#define PIVOT_CANCODER_RATIO 9.0f/21.0f

#define MINIMUM_PIVOT_SPEED 0.25_tps

//1-50 motor - sensor
//21-9 gear to encoder

#define ALGAE_CACHE_SIZE 2000
#define CORAL_CACHE_SIZE 500

#define VIABLE_ELEVATOR_THRESHOLD 0.02_m
#define VIABLE_ELEVATOR_DISTANCE 1.8_m //consider the offset of the canrange from the front of robot which is 8 inches
#define TELEOP_VIABLE_DUNK_SPEED_L4 0.25_mps

#define MIN_DUNK_DISTANCE_OVER_CORAL 11_in
#define MAX_DUNK_DISTANCE_OVER_CORAL 14.5_in
#define DUNK_OFFSET_OVER_CORAL 2.5_in

#define ELEVATOR_THRESHOLD_FOR_FUNNEL 0.25_in
#define PIVOT_THRESHOLD_FOR_FUNNEL 0.05_tr

using namespace Constants::Scorer;

Scorer::Scorer(frc::TimedRobot *_robot, Drivetrain *_drivetrain, Climber *_climber, valor::CANdleSensor* _leds) :
    valor::BaseSubsystem(_robot, "Scorer"),
    hallEffectDebounceSensor(_robot, "HallEffectDebounce"),
    candi(CANIDs::HALL_EFFECT, "baseCAN"),
    elevatorMotor(new valor::PhoenixController(valor::PhoenixControllerType::KRAKEN_X60, CANIDs::ELEV_WHEEL, valor::NeutralMode::Brake, elevatorMotorInverted(), "baseCAN")),
    scorerMotor(new valor::PhoenixController(Constants::getScorerMotorType(), CANIDs::SCORER_WHEEL, valor::NeutralMode::Brake, scorerMotorInverted(), "baseCAN")),
    scorerPivotMotor(new valor::PhoenixController(Constants::getScorerPivotMotorType(), CANIDs::SCORER_PIVOT_MOTOR, valor::NeutralMode::Brake, scorerPivotInverted(), "baseCAN")),
    funnelMotor(new valor::PhoenixController(Constants::getFunnelMotorType(), CANIDs::FUNNEL, valor::NeutralMode::Coast, funnelInverted(), "baseCAN")),
    scorerStagingSensor(_robot, "Scorer Staging Sensor", CANIDs::STAGING_LIDAR_SENSOR, "baseCAN", -1_mm),
    currentSensor(_robot, "Algae Current Sensor"),
    coralCurrentSensor(_robot, "Coral Current Sensor"),
    positionMap{std::move(getPositionMap())},
    scoringSpeedMap{std::move(getScoringSpeedMap())},
    drivetrain(_drivetrain),
    climber(_climber),
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
                    state.scopedState = SCOPED_STATE::UNSCOPED;
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
                    state.scopedState = SCOPED_STATE::UNSCOPED;
                    state.elevState = ELEVATOR_STATE::HP;
                    state.gamePiece = GAME_PIECE::CORAL;
                }
            )
        )
    ).ToPtr());

        pathplanner::NamedCommands::registerCommand("HPAlgae", std::move(
        frc2::SequentialCommandGroup(
            frc2::InstantCommand(
                [this]() {
                    state.scopedState = SCOPED_STATE::UNSCOPED;
                    state.elevState = ELEVATOR_STATE::HP;
                    state.gamePiece = GAME_PIECE::ALGEE;
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
                    state.scopedState = SCOPED_STATE::MANUAL_SCOPE;
                }
            )
        )
    ).ToPtr());

    pathplanner::NamedCommands::registerCommand("Scope", std::move(
        frc2::SequentialCommandGroup(
            frc2::InstantCommand(
                [this](){
                    state.scopedState = SCOPED_STATE::SCOPED;
                }
            )
        )
    ).ToPtr());

    pathplanner::NamedCommands::registerCommand("Unscope", std::move(
        frc2::SequentialCommandGroup(
            frc2::InstantCommand(
                [this](){
                    state.scopedState = SCOPED_STATE::UNSCOPED;
                }
            )
        )
    ).ToPtr());

    pathplanner::NamedCommands::registerCommand("NoAlgae", std::move(
        frc2::SequentialCommandGroup(
            frc2::InstantCommand(
                [this](){
                    state.hasAlgae = false;
                }
            )
        )
    ).ToPtr());


    pathplanner::NamedCommands::registerCommand("AlignLeft", std::move(
        frc2::SequentialCommandGroup(
            frc2::FunctionalCommand(
                [&]{ // on begin
                    drivetrain->state.startTimestamp = frc::Timer::GetFPGATimestamp();
                    drivetrain->state.reefTag.first = -1;

                    drivetrain->state.dir = LEFT;
                    // drivetrain->resetAlignControllers();
                    drivetrain->hasYReset = true;

                    state.gamePiece = GAME_PIECE::CORAL;
                    state.scopedState = SCOPED_STATE::SCOPED;
                    state.elevState = ELEVATOR_STATE::FOUR;
                    
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
                    state.scopedState = SCOPED_STATE::UNSCOPED;

                    drivetrain->state.reefTag.first = -1;
                    drivetrain->hasYReset = false;
                },
                [&]{ // is Finished
                    // return state.scoringState == SCORE_STATE::SCORING;
                    return (!scorerStagingSensor.isTriggered())
                    && drivetrain->withinXRange((units::meter_t)table->GetNumber("Viable Elevator Distance (m)", VIABLE_ELEVATOR_DISTANCE.value()));
                },
                {}
            )
        )
    ).ToPtr());

    pathplanner::NamedCommands::registerCommand("AlignRight", std::move(
        frc2::SequentialCommandGroup(
            frc2::FunctionalCommand(
                [&]{ // on begin
                    drivetrain->state.startTimestamp = frc::Timer::GetFPGATimestamp();
                    drivetrain->state.reefTag.first = -1;
                    
                    drivetrain->state.dir = RIGHT;
                    drivetrain->hasYReset = true;
                    // drivetrain->resetAlignControllers();

                    state.gamePiece = GAME_PIECE::CORAL;
                    state.scopedState = SCOPED_STATE::SCOPED;
                    state.elevState = ELEVATOR_STATE::FOUR;
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
                    state.scopedState = SCOPED_STATE::UNSCOPED;

                    drivetrain->state.reefTag.first = -1;
                    drivetrain->hasYReset = false;
                },
                [&]{ // is Finished
                    // return state.scoringState == SCORE_STATE::SCORING;
                    return (!scorerStagingSensor.isTriggered())
                        && drivetrain->withinXRange((units::meter_t)table->GetNumber("Viable Elevator Distance (m)", VIABLE_ELEVATOR_DISTANCE.value()));
                },
                {}
            )
        )
    ).ToPtr());

    pathplanner::NamedCommands::registerCommand("AlignAlgaeL2", std::move(
        frc2::SequentialCommandGroup(
            frc2::FunctionalCommand(
                [&]{ // on begin
                    drivetrain->state.startTimestamp = frc::Timer::GetFPGATimestamp();
                    drivetrain->state.reefTag.first = -1;
                    
                    state.gamePiece = GAME_PIECE::ALGEE;
                    drivetrain->state.dir = NONE;
                    drivetrain->hasYReset = true;
                    // drivetrain->resetAlignControllers();

                    state.scopedState = SCOPED_STATE::SCOPED;
                    state.elevState = ELEVATOR_STATE::TWO;
                    // state.scoringState = SCORE_STATE::SCORING;
                    state.scoringState = SCORE_STATE::INTAKING;
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
                    // state.scopedState = SCOPED_STATE::UNSCOPED;

                    drivetrain->state.reefTag.first = -1;
                    drivetrain->hasYReset = false;
                },
                [&]{ // is Finished
                    // return state.scoringState == SCORE_STATE::SCORING;
                    return state.hasAlgae;
                },
                {}
            )
        )
    ).ToPtr());

        pathplanner::NamedCommands::registerCommand("AlignAlgaeL3", std::move(
        frc2::SequentialCommandGroup(
            frc2::FunctionalCommand(
                [&]{ // on begin
                    drivetrain->state.startTimestamp = frc::Timer::GetFPGATimestamp();
                    drivetrain->state.reefTag.first = -1;
                    
                    state.gamePiece = GAME_PIECE::ALGEE;
                    drivetrain->state.dir = NONE;
                    drivetrain->hasYReset = true;
                    // drivetrain->resetAlignControllers();

                    state.scopedState = SCOPED_STATE::SCOPED;
                    state.elevState = ELEVATOR_STATE::THREE;
                    // state.scoringState = SCORE_STATE::SCORING;
                    state.scoringState = SCORE_STATE::INTAKING;
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
                    // state.scopedState = SCOPED_STATE::UNSCOPED;

                    drivetrain->state.reefTag.first = -1;
                    drivetrain->hasYReset = false;
                },
                [&]{ // is Finished
                    // return state.scoringState == SCORE_STATE::SCORING;
                    return state.hasAlgae;
                },
                {}
            )
        )
    ).ToPtr());

    /*pathplanner::NamedCommands::registerCommand("SetReefTag", std::move(

    ).ToPtr());*/


    
    table->PutNumber("Viable Elevator Distance (m)", VIABLE_ELEVATOR_DISTANCE.value());

    init();

    visualizerStage1 = nt::NetworkTableInstance::GetDefault().GetStructTopic<frc::Pose3d>("LiveWindow/BaseSubsystem/Scorer/Stage1Height").Publish();
    visualizerStage2 = nt::NetworkTableInstance::GetDefault().GetStructTopic<frc::Pose3d>("LiveWindow/BaseSubsystem/Scorer/Stage2Height").Publish();
    visualizerStage3 = nt::NetworkTableInstance::GetDefault().GetStructTopic<frc::Pose3d>("LiveWindow/BaseSubsystem/Scorer/Stage3Height").Publish();
}

frc2::CommandPtr Scorer::scorerPitSequenceStage(GAME_PIECE gamePiece, ELEVATOR_STATE elevState, int offset, int numLeds) {
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
            [this, offset, numLeds](bool) {
                int color = elevatorPositionSuccess.Get() ? valor::CANdleSensor::GREEN : valor::CANdleSensor::RED;
                for (int i = 0; i < numLeds; i++) {
                    leds->setLED(8 + offset + i, color);
                    leds->setLED(LEDConstants::LED_COUNT - offset - i, color);
                }
                elevatorPositionFail.Set(false);
                elevatorPositionSuccess.Set(false);
            },
            [this] { return false; }
        }.ToPtr()
    );
}

frc2::CommandPtr Scorer::scorerPitSequence() {
    return frc2::cmd::Sequence(
        frc2::cmd::Sequence(
            frc2::cmd::RunOnce([this] {
                state.gamePiece = GAME_PIECE::ALGEE;
                state.intaking = true;
            }),
            frc2::cmd::WaitUntil([this] { return state.hasAlgae; })
        ),
        frc2::cmd::WaitUntil([this] { return driverGamepad->GetBButton(); }),
        frc2::cmd::Sequence(
            frc2::cmd::RunOnce([this] {
                state.elevState = ELEVATOR_STATE::ONE;
                state.scopedState = SCOPED_STATE::MANUAL_SCOPE;
            }),
            // frc2::cmd::WaitUntil([this] {  })
            frc2::cmd::Wait(1_s),
            frc2::cmd::RunOnce([this] {
                state.scoringState = SCORE_STATE::SCORING;
                state.hasAlgae = false;
            }),
            frc2::cmd::Wait(2_s),
            frc2::cmd::RunOnce([this] {
                state.scoringState = SCORE_STATE::HOLD;
                state.scopedState = SCOPED_STATE::UNSCOPED;
            })
        ),
        frc2::cmd::WaitUntil([this] { return driverGamepad->GetBButton(); }),
        frc2::cmd::Sequence(
            frc2::cmd::RunOnce([this] {
                state.gamePiece = GAME_PIECE::ALGEE;
                state.elevState = ELEVATOR_STATE::FOUR;
                state.scopedState = SCOPED_STATE::MANUAL_SCOPE;
            }),
            frc2::cmd::Wait(4_s)
        ),
        frc2::cmd::WaitUntil([this] { return driverGamepad->GetBButton(); }),
        // frc2::cmd::RunOnce([this] { state.scopedState = SCOPED; }),
        // scorerPitSequenceStage(GAME_PIECE::CORAL, ELEVATOR_STATE::STOWED, 8, 2),
        // scorerPitSequenceStage(GAME_PIECE::CORAL, ELEVATOR_STATE::ONE, 10, 2),
        // frc2::cmd::RunOnce([this] { elevatorBButtonWait.Set(true); }),
        // frc2::cmd::WaitUntil([this] { return driverGamepad->GetBButton(); }),
        // frc2::cmd::RunOnce([this] { state.scoringState = SCORING; }),
        // frc2::cmd::RunOnce([this] { elevatorBButtonWait.Set(true); }),
        // frc2::cmd::WaitUntil([this] { return driverGamepad->GetBButton(); }),
        // scorerPitSequenceStage(GAME_PIECE::CORAL, ELEVATOR_STATE::TWO, 12, 2),
        // scorerPitSequenceStage(GAME_PIECE::CORAL, ELEVATOR_STATE::THREE, 14, 2),
        // scorerPitSequenceStage(GAME_PIECE::CORAL, ELEVATOR_STATE::FOUR, 16, 2),
        // scorerPitSequenceStage(GAME_PIECE::CORAL, ELEVATOR_STATE::STOWED, 18, 3),
        // frc2::cmd::RunOnce([this] {
        //     state.scoringState = HOLD;
        //     elevatorBButtonWait.Set(true);
        // }),
        // frc2::cmd::WaitUntil([this] { return driverGamepad->GetBButton(); }),
        // scorerPitSequenceStage(GAME_PIECE::ALGEE, ELEVATOR_STATE::STOWED, 21, 3),
        // frc2::cmd::RunOnce([this] { state.scoringState = SCORING; }),
        // frc2::cmd::Wait(3_s),
        // scorerPitSequenceStage(GAME_PIECE::ALGEE, ELEVATOR_STATE::FOUR, 24, 3),
        frc2::cmd::RunOnce([this] {
            resetState();
            // elevatorStage.Set(false);
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
    coralCurrentSensor.reset();
    state.intaking = false;
    state.hasAlgae = false;
    state.hasCoral = false;
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

    scorerStagingSensor.setMaxDistance(0.175_m);
    scorerStagingSensor.setThresholdDistance(8_cm);
    
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
    elevatorMotor->setCurrentLimits(
        units::ampere_t{120},
        units::ampere_t{60},
        units::ampere_t{45},
        units::second_t{0.5},
        true
    );
    elevatorMotor->applyConfig();

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

    scorerPivotMotor->setGearRatios(PIVOT_MOTOR_TO_SENSOR, PIVOT_SENSOR_TO_MECH);
    scorerPivotMotor->enableFOC(true);
    scorerPivotMotor->setForwardLimit(SECOND_SCORER_FORWARD_LIMIT);
    scorerPivotMotor->setReverseLimit(SECOND_SCORER_REVERSE_LIMIT);

    valor::PIDF pivotPID = Constants::Scorer::getScorerPivotPIDF();
    pivotPID.maxVelocity = scorerPivotMotor->getMaxMechSpeed();
    pivotPID.maxAcceleration = units::turns_per_second_squared_t(3.45); // scorerPivotMotor->getMaxMechSpeed() / 1_s;
    scorerPivotMotor->setPIDF(pivotPID);
    scorerPivotMotor->setupCANCoder(CANIDs::SCORER_PIVOT_CAN, scorerPivotMagnetOffset(), ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive, "baseCAN");
    scorerPivotMotor->applyConfig();

    funnelMotor->setGearRatios(1, Constants::Scorer::getFunnelSensorToMech());
    funnelMotor->enableFOC(true);
    funnelMotor->setCurrentLimits(
        units::ampere_t{100},
        units::ampere_t{60},
        units::ampere_t{45},
        units::second_t{0.5},
        true
    );

    valor::PIDF funnelPID = Constants::Scorer::getFunnelPIDF();
    
    funnelMotor->setPIDF(funnelPID);
    funnelMotor->applyConfig();

    // Zeroing debounce sensor (utilizes CANdi configured hall effect sensor)
    hallEffectDebounceSensor.setGetter([this] { return hallEffectSensorActive();});
    hallEffectDebounceSensor.setRisingEdgeCallback([this] {
        if (cancoderSensorBad())
        {
            elevatorMotor->setEncoderPosition(0_tr);
        }
        state.hasZeroed = true;
        absSensorCorrect = elevatorMotor->getAbsEncoderPosition();

        if (frc::DriverStation::IsTest()){
            std::cout << "\nNew Magnet Offset: " << -(Constants::getElevatorMagnetOffset() - elevatorMotor->getCANCoder()->GetAbsolutePosition().GetValue()).value() << std::endl;
        }
    });

    // Beambreak debounce sensor (on scoring mechanism)
    scorerStagingSensor.setRisingEdgeCallback([this] {
        if (state.gamePiece == GAME_PIECE::CORAL) {
            state.scoringState = HOLD;
            scorerMotor->setSpeed(0_tps);
            scorerMotor->setEncoderPosition(0_tr);
        }
    });

    currentSensor.setSpikeSetpoint(25);
    currentSensor.setGetter([this]() {return scorerMotor->getCurrent().to<double>(); });
    currentSensor.setSpikeCallback([this]() {
        if (scorerPivotMotor->getSpeed() < MINIMUM_PIVOT_SPEED && state.gamePiece == GAME_PIECE::ALGEE) {
            state.hasAlgae = true;
            currentSensor.reset();
        }
    });
    currentSensor.setCacheSize(ALGAE_CACHE_SIZE);

    coralCurrentSensor.setSpikeSetpoint(25);
    coralCurrentSensor.setGetter([this]() {return scorerMotor->getCurrent().to<double>(); });
    coralCurrentSensor.setSpikeCallback([this]() {
        if (scorerPivotMotor->getSpeed() < MINIMUM_PIVOT_SPEED && state.gamePiece == GAME_PIECE::CORAL && state.intaking) {
            state.hasCoral = true;
            coralCurrentSensor.reset();
        }
    });
    coralCurrentSensor.setCacheSize(CORAL_CACHE_SIZE);

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

    // Driver controller section
    if (driverGamepad == nullptr || !driverGamepad->IsConnected())
        return;

    if(operatorGamepad->leftTriggerActive() || driverGamepad->GetXButton()) {
        state.gamePiece = ALGEE;
        state.elevState = ELEVATOR_STATE::TWO;
    } else if(operatorGamepad->rightTriggerActive() || driverGamepad->GetBButton()) {
        state.gamePiece = CORAL;
        state.elevState = ELEVATOR_STATE::TWO;
    }

    if (operatorGamepad->leftStickYActive()) {
        state.elevState = MANUAL;
        state.manualSpeed = operatorGamepad->leftStickY(3) * 12_V * 0.75;
    } else if (operatorGamepad->GetYButton() || driverGamepad->DPadUp()) {
         state.elevState = ELEVATOR_STATE::FOUR;
    } else if (operatorGamepad->GetBButton() || driverGamepad->DPadRight()) {
        state.elevState = ELEVATOR_STATE::THREE;
    } else if (operatorGamepad->GetAButton() || driverGamepad->DPadDown()) {
        state.elevState = ELEVATOR_STATE::TWO;
    } else if (operatorGamepad->GetXButton() || driverGamepad->DPadLeft()) {
        state.elevState = ELEVATOR_STATE::ONE;
    } else if (operatorGamepad->DPadRight()) {
        state.elevState = ELEVATOR_STATE::HP;
    }

    if (state.elevState == ELEVATOR_STATE::ONE && driverGamepad->leftTriggerActive()) {
        state.intaking = true;
        state.scopedState = UNSCOPED;
        drivetrain->state.intaking = true;
    } else {
        state.intaking = false;
        drivetrain->state.intaking = false;
        if (driverGamepad->GetLeftBumperButton()) {
            state.scopedState = MANUAL_SCOPE;
        } else if (driverGamepad->leftTriggerActive() || driverGamepad->rightTriggerActive()) {
            state.scopedState = SCOPED;
        } else {
            state.scopedState = UNSCOPED;
        }
    }

    if (driverGamepad->GetRightBumperButton()) {
        state.scoringState = SCORE_STATE::SCORING;
        state.hasAlgae = false;
        state.hasCoral = false;
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

    if (state.elevState == ELEVATOR_STATE::FOUR && drivetrain->getXConstraints().maxVelocity != drivetrain->x_constraints_l4.maxVelocity) {
        drivetrain->setXConstraints(drivetrain->x_constraints_l4);
    } else if (
        (state.elevState == ELEVATOR_STATE::ONE || state.elevState == ELEVATOR_STATE::TWO || state.elevState == ELEVATOR_STATE::THREE) && 
        drivetrain->getXConstraints().maxVelocity != drivetrain->x_constraints.maxVelocity
    ) {
        drivetrain->setXConstraints(drivetrain->x_constraints);
    }

    if ((state.hasAlgae || state.hasCoral) && state.intaking){
        driverGamepad->setRumble(true);
    } else {
        driverGamepad->setRumble(false);
    }

    units::meter_t elevatorSetpoint = positionMap[state.gamePiece][state.elevState];

    bool isStopped = drivetrain->isSpeedStopped();
    units::meter_t distanceFromReef = drivetrain->lidarDistance();
    state.shootOverCoral = false; // isStopped && (MIN_DUNK_DISTANCE_OVER_CORAL<= distanceFromReef && distanceFromReef <= MAX_DUNK_DISTANCE_OVER_CORAL);
    if (state.shootOverCoral) {
        elevatorSetpoint += DUNK_OFFSET_OVER_CORAL;
    }

    units::meter_t elevatorError = units::math::fabs(convertToMechSpace(elevatorMotor->getPosition()) - elevatorSetpoint);
    elevatorWithinThreshold = elevatorError.value() < table->GetNumber("Elevator Threshold (m)", VIABLE_ELEVATOR_THRESHOLD.value());
    
    if (
        (state.autoDunkEnabled && !disableAutoDunk) &&
        state.scopedState == SCOPED_STATE::SCOPED &&
        elevatorWithinThreshold &&
        state.gamePiece == GAME_PIECE::CORAL &&
        (state.elevState == TWO || state.elevState == THREE || state.elevState == FOUR) &&
        drivetrain->getAutoDunkAcceptance().all()
    ) {
        state.scoringState = SCORE_STATE::SCORING;
    }

    int botColor = state.gamePiece == GAME_PIECE::CORAL ? valor::CANdleSensor::VALOR_GOLD : valor::CANdleSensor::VALOR_PURPLE;
    int midTopColor = state.scoringState == SCORE_STATE::SCORING ? valor::CANdleSensor::BLUE :
        (drivetrain->state.alignToTarget ? valor::CANdleSensor::RED : 
        (scorerStagingSensor.isTriggered() || state.hasAlgae || state.hasCoral ? valor::CANdleSensor::GREEN : valor::CANdleSensor::OFF));

    if (!frc::DriverStation::IsTestEnabled()) {
        leds->setColor(0, botColor, valor::CANdleSensor::Priority::PRIORITY_SCORER);
        leds->setColor(1, midTopColor, valor::CANdleSensor::Priority::PRIORITY_SCORER);
        leds->setColor(2, midTopColor, valor::CANdleSensor::Priority::PRIORITY_SCORER);
        leds->setColor(3, midTopColor, valor::CANdleSensor::Priority::PRIORITY_SCORER);
        leds->setColor(4, midTopColor, valor::CANdleSensor::Priority::PRIORITY_SCORER);
        leds->setColor(5, botColor, valor::CANdleSensor::Priority::PRIORITY_SCORER);
    }

    int tagID = drivetrain->state.reefTag.first;

    if ((state.elevState == TWO || state.elevState == THREE) && state.gamePiece == GAME_PIECE::ALGEE && !state.intaking && (state.scopedState == SCOPED || state.scopedState == MANUAL_SCOPE)) {
        if (tagID >= 6 && tagID <= 11) {
            state.elevState = (tagID % 2 == 1) ? ELEVATOR_STATE::THREE : ELEVATOR_STATE::TWO;
        } else if (tagID >= 17 && tagID <= 22) {
            state.elevState = (tagID % 2 == 0) ? ELEVATOR_STATE::THREE : ELEVATOR_STATE::TWO;
        }
    }
   
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

    // Pivot State Machine
    if(climber->state.climbState == Climber::CLIMB_STATE::STOW) {
        if (state.gamePiece == GAME_PIECE::ALGEE) {
            if (state.intaking) {
                if (state.hasAlgae && !state.intaking) {
                    scorerPivotMotor->setPosition(getPivotPositionMap()[PIVOT_STATE::CARRY]);
                } else {
                    scorerPivotMotor->setPosition(getPivotPositionMap()[PIVOT_STATE::GROUND]);
                }  
            } else if (state.elevState == ELEVATOR_STATE::FOUR && state.gamePiece == GAME_PIECE::ALGEE) {
                if (units::math::fabs(convertToMechSpace(elevatorMotor->getPosition()) - positionMap[state.gamePiece][state.elevState]) < 0.1_m){
                    scorerPivotMotor->setPosition(getPivotPositionMap()[PIVOT_STATE::PICK]);
                } else {
                    scorerPivotMotor->setPosition(getPivotPositionMap()[PIVOT_STATE::CARRY]);
                }   
            } else if (state.scopedState == SCOPED_STATE::SCOPED || state.scopedState == SCOPED_STATE::MANUAL_SCOPE)  {
                if (state.elevState == ELEVATOR_STATE::TWO || state.elevState == ELEVATOR_STATE::THREE || state.elevState == ELEVATOR_STATE::FOUR) {
                    scorerPivotMotor->setPosition(getPivotPositionMap()[PIVOT_STATE::PICK]);
                } else {
                    scorerPivotMotor->setPosition(getPivotPositionMap()[PIVOT_STATE::PRESCORE]);
                }
            } else if (state.hasAlgae) {
                scorerPivotMotor->setPosition(getPivotPositionMap()[PIVOT_STATE::CARRY]);
            } else {
                scorerPivotMotor->setPosition(getPivotPositionMap()[PIVOT_STATE::CARRY]);
            }
        } else {
            if (state.intaking) {
                scorerPivotMotor->setPosition(getPivotPositionMap()[PIVOT_STATE::CORAL_GROUND]);
            } else {
                if (state.elevState == ELEVATOR_STATE::ONE) {
                    scorerPivotMotor->setPosition(getPivotPositionMap()[PIVOT_STATE::PICK]);
                } else {
                    scorerPivotMotor->setPosition(getPivotPositionMap()[PIVOT_STATE::CORAL_STOW]);
                }
            }
        }
    } else{
        scorerPivotMotor->setPosition(getPivotPositionMap()[PIVOT_STATE::CARRY]);
    }

    //Elevator State Machine
    if (state.elevState == ELEVATOR_STATE::MANUAL) {
        elevatorMotor->setPower(state.manualSpeed + units::volt_t{Constants::Scorer::getElevatorPIDF().aFF});
    } else {
        if (state.intaking) {
            if (state.hasAlgae) {
                state.targetHeight = positionMap.at(state.gamePiece).at(ELEVATOR_STATE::STOWED);
            } else {
                state.targetHeight = positionMap.at(state.gamePiece).at(ELEVATOR_STATE::FLOOR);
            }
        } else if ((state.scopedState == SCOPED &&
            (
                state.gamePiece == GAME_PIECE::ALGEE ||
                (state.gamePiece == GAME_PIECE::CORAL && drivetrain->withinXRange((units::meter_t)table->GetNumber("Viable Elevator Distance (m)", VIABLE_ELEVATOR_DISTANCE.value())))
            )) || state.scopedState == MANUAL_SCOPE
        ) {
            state.targetHeight = positionMap.at(state.gamePiece).at(state.elevState);
            if (state.shootOverCoral) {
                state.targetHeight += DUNK_OFFSET_OVER_CORAL;
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
    if (climber->state.climbState == Climber::CLIMB_STATE::STOW){
        if (state.scoringState == SCORE_STATE::SCORING) {
            currentSensor.reset();
            coralCurrentSensor.reset();
            if (state.gamePiece == GAME_PIECE::ALGEE) {
                scorerMotor->setSpeed(ALGEE_SCORE_SPEED);
            } else {
                auto it = scoringSpeedMap.find(state.elevState);
                if (it != scoringSpeedMap.end()) {
                    scorerMotor->setSpeed(it->second);
                } else {
                    scorerMotor->setSpeed(SCORE_SPEED);
                }
            }
        } else if (state.hasAlgae && state.gamePiece == GAME_PIECE::ALGEE) {
            scorerMotor->setSpeed(ALGEE_HOLD_SPD);
        } else if (state.hasCoral && state.gamePiece == GAME_PIECE::CORAL && state.elevState == ELEVATOR_STATE::ONE) {
            scorerMotor->setSpeed(CORAL_HOLD_SPD);
        } else if (state.intaking) {
            if (state.gamePiece == GAME_PIECE::ALGEE) {
                scorerMotor->setSpeed(ALGEE_INTAKE_SPEED);
            } else {
                scorerMotor->setSpeed(CORAL_GROUND_INTAKE_SPEED);
            }
        } else if (!scorerStagingSensor.isTriggered()) {
            if (state.gamePiece == GAME_PIECE::ALGEE) {
                scorerMotor->setSpeed(ALGEE_INTAKE_SPEED);
            } else {
                scorerMotor->setSpeed(CORAL_INTAKE_SPEED);
            }
        } else {
            scorerMotor->setPosition(getIntakeTurns());
        }

    } else {
        scorerMotor->setPower(0_V);
    }

    // Funnel State Machine
    if(!scorerStagingSensor.isTriggered() && state.gamePiece == GAME_PIECE::CORAL && !state.intaking 
        && (convertToMechSpace(elevatorMotor->getPosition()) < positionMap.at(CORAL).at(HP) + ELEVATOR_THRESHOLD_FOR_FUNNEL || hallEffectSensorActive())
        && state.elevState != ELEVATOR_STATE::ONE
        && scorerPivotMotor->getPosition() < getPivotPositionMap().at(PIVOT_STATE::CORAL_STOW) + PIVOT_THRESHOLD_FOR_FUNNEL
        && climber->state.climbState == Climber::CLIMB_STATE::STOW){
        funnelMotor->setSpeed(40_tps);
    } else{
        funnelMotor->setPower(0_V);
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
        "Has Coral",
        [this] {return state.hasCoral;},
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
        "ZEROED?",
        [this] {return state.hasZeroed;},
        nullptr
    );

    builder.AddBooleanProperty(
        "ABSOLUTE POSITION NO OFFSET?",
        [this] {return absSensorCorrect.value();},
        nullptr
    );
    builder.AddBooleanArrayProperty(
        "Auto Dunk Acceptance",
        [this] {
            std::bitset<5> drivetrainAcceptances = drivetrain->getAutoDunkAcceptance();
            std::vector<int> gates = {
                state.autoDunkEnabled && !table->GetBoolean("Auto Dunk Disabled", false), // 0
                drivetrainAcceptances[0],
                state.scopedState == SCOPED_STATE::SCOPED, // 2
                elevatorWithinThreshold, // 3
                state.gamePiece == GAME_PIECE::CORAL, // 4
                state.elevState == TWO || state.elevState == THREE || state.elevState == FOUR, // 5
                drivetrainAcceptances[1], // 6
                drivetrainAcceptances[2], // 7
                drivetrainAcceptances[3], // 8
                drivetrainAcceptances[4] // 9
            };
            return gates;
        },
        nullptr
    );
}



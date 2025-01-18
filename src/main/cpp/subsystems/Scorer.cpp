#include "Constants.h"
#include "subystems/Scorer.h"
#include "valkyrie/controllers/NeutralMode.h"
#include "valkyrie/controllers/PIDF.h"

#include <pathplanner/lib/auto/NamedCommands.h>

#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>

#include <frc/DriverStation.h>

#define ELEV_MAX_SPEED units::angular_velocity::turns_per_second_t (0)
#define ELEV_MAX_ACCEL units::angular_acceleration::turns_per_second_squared_t (0)
#define ELEV_K_P 0
#define ELEV_K_ERROR units::angle::turn_t (0)
#define ELEV_K_AFF 0

#define SCORER_MAX_SPEED units::angular_velocity::turns_per_second_t (0)
#define SCORER_MAX_ACCEL units::angular_acceleration::turns_per_second_squared_t (0)
#define SCORER_K_P 0


#define CORAL_LEVEL_2_HEIGHT 1.5f  // Example height for Coral Level 2
#define CORAL_LEVEL_3_HEIGHT 3.5f  // Example height for Coral Level 3
#define CORAL_LEVEL_4_HEIGHT 5.5f  // Example height for Coral Level 4
#define TROUGH_HEIGHT 0.0f        // Trough position (ground level)
#define MOTOR_TO_SENSOR 1.0f
#define SENSOR_TO_MECH 1.0f

Scorer::Scorer(frc::TimedRobot *_robot, Drivetrain *_drive) :
    valor::BaseSubsystem(_robot, "Scorer"),
    drivetrain(_drive),
    scorerDebounceSensor(_robot, "FeederBanner"),
    elevatorMotor(new valor::PhoenixController(valor::PhoenixControllerType::KRAKEN_X60, CANIDs::ELEV_WHEEL, valor::NeutralMode::Coast, true)),
    scorerMotor(new valor::PhoenixController(valor::PhoenixControllerType::KRAKEN_X60, CANIDs::SCORER_WHEEL, valor::NeutralMode::Coast, true)),
    lidarSensor(_robot, "Front Lidar Sensor", CANIDs::FRONT_LIDAR_SENSOR)
    {
        frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
    pathplanner::NamedCommands::registerCommand("Score sequence Scorer", std::move(
        frc2::SequentialCommandGroup(
            frc2::InstantCommand(
                [this]() {
                    // shooter->state.isShooting = true;
                    state.scoringState = Scorer::SCORING_SPEED::SCORING;
                }
            ),
            frc2::WaitCommand(1_s),
            frc2::InstantCommand(
                [this]() {
                    // shooter->state.isShooting = false;
                    state.scoringState = Scorer::SCORING_SPEED::HOLD;
                }
            )
        )
    ).ToPtr());

    pathplanner::NamedCommands::registerCommand("Intaking", std::move(
        frc2::SequentialCommandGroup(
            frc2::InstantCommand(
                [this]() {
                    // shooter->state.isShooting = true;
                    state.scoringState = Scorer::SCORING_SPEED::INTAKING;
                }
            )
        )
    ).ToPtr());
    pathplanner::NamedCommands::registerCommand("Enable Scorer", std::move(
        frc2::SequentialCommandGroup(
            frc2::InstantCommand(
                [this]() {
                    // shooter->state.isShooting = true;
                    state.scoringState = Scorer::SCORING_SPEED::SCORING;
                }
            )
        )
    ).ToPtr());
     pathplanner::NamedCommands::registerCommand("DISABLE Scorer", std::move(
        frc2::SequentialCommandGroup(
            frc2::InstantCommand(
                [this]() {
                    // shooter->state.isShooting = true;
                    state.scoringState = Scorer::SCORING_SPEED::HOLD;
                }
            )
        )
    ).ToPtr());
        scorerMotor->setGearRatios(MOTOR_TO_SENSOR, SENSOR_TO_MECH);
        init();

    }

    void Scorer::resetState()
    {
        state.scoringState = SCORING_SPEED::HOLD;
        state.coralLevel = ELEV_LVL::TROUGH;

    }


    void Scorer::init()
    {

        valor::PIDF elevatorPID;
        elevatorPID.maxVelocity = ELEV_MAX_SPEED;
        elevatorPID.maxAcceleration = ELEV_MAX_ACCEL;
        elevatorPID.P =ELEV_K_P ;
        elevatorPID.error = ELEV_K_ERROR;
        elevatorPID.aFF = ELEV_K_AFF;
        elevatorPID.aFFType = valor::FeedForwardType::LINEAR;

        valor::PIDF scorerPID;
        scorerPID.maxVelocity = SCORER_MAX_SPEED;
        scorerPID.maxAcceleration = SCORER_MAX_ACCEL;
        scorerPID.P = SCORER_K_P;
        
        scorerMotor->setPIDF(scorerPID, 0);

    //     scorerDebounceSensor.setGetter([this] { return (!->GetInWindow()); });
    //     scorerDebounceSensor.setRisingEdgeCallback([this] {

    //         //TODO: RISING SENSOR 2 EDGE
      
        
    // });

        resetState();
    }

    void Scorer::assessInputs()
    {
        if (driverGamepad == nullptr || !driverGamepad->IsConnected())
        return;

        if (operatorGamepad == nullptr || !operatorGamepad->IsConnected())
        return;

        if (driverGamepad->GetAButton()) {
            state.coralLevel = ELEV_LVL::TWO;
        } else if (driverGamepad->GetBButton() ) {
            state.coralLevel = ELEV_LVL::THREE;
        } else if (driverGamepad->GetXButton()){
            state.coralLevel = ELEV_LVL::FOUR;
        } else if (driverGamepad->GetYButton()){
            state.coralLevel = ELEV_LVL::TROUGH;
        } else{
        state.coralLevel = ELEV_LVL::HP;
        }

    }

    void Scorer::analyzeDashboard()
    {
    //     if (aligned && properlyElevated && !state.sensor2Broken) {
    //         state.scoringState = SCORING_SPEED::SCORING;  // Move to placing state
    // } 
    //     else if (state.sensor2Broken){
    //     state.scoringState = SCORING_SPEED::HOLD;
    // }
    //     else if (state.senssor1Broken){
    //     state.scoringState = SCORING_SPEED::INTAKING;
    // }

    }

    void Scorer::assignOutputs()
    {
        units::length::meter_t targetHeightInMeters = coralHMap[state.coralLevel];
        double wheelDiameter = 0.1;
        // TODO: check whether chain or pulley system
        double targetRotations = targetHeightInMeters.value() / (wheelDiameter * M_PI);  // Convert meters to rotations (turns)
        scorerMotor->setPosition(units::turn_t(targetRotations));
    }


    void Scorer::InitSendable(wpi::SendableBuilder& builder)
    {

        builder.SetSmartDashboardType("Subsystem");
        builder.AddDoubleProperty(
            "State Speed",
            [this] { return state.scoringState; },
            nullptr
        );
        builder.AddDoubleProperty(
            "Scorer Vel",
            [this] {return scorerMotor->getSpeed().to<double>();},
            nullptr
        );
        builder.AddDoubleProperty(
            "Scorer current",
            [this] {return scorerMotor->getCurrent().to<double>();},
            nullptr
        );






    }


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
// 
#define ELEV_K_P 0
#define ELEV_K_ERROR units::angle::turn_t (0)
#define ELEV_K_AFF 0
#define MOTOR_TO_MECH_IN 6000/SENSOR_TO_MECH*

#define SCORER_K_P 0

#define ELEVATOR_FORWARD_LIMIT 4_tr
#define ELEVATOR_REVERSE_LIMIT 0.5_tr

#define SEGMENTS 0.0f
#define CIRCUM 1.432* M_Pi
#define MOTOR_TO_SENSOR 1.0f
#define SENSOR_TO_MECH 8.02f
#define ELEVATOR_TOLERANCE 0.5f
#define GEAR_CIRCUMFERENCE units::meter_t{1.432_in} * M_PI // meters
#define CONVERSION_FACTOR units::turn_t{1} / GEAR_CIRCUMFERENCE

Scorer::Scorer(frc::TimedRobot *_robot, Drivetrain *_drive) :
    valor::BaseSubsystem(_robot, "Scorer"),
    drivetrain(_drive),
    scorerDebounceSensor(_robot, "ScorerDebounce"),
    hallEffectDebounceSensor(_robot, "HallEffectDebounce"),
    hallEffectDigitalSensor(DIOPorts::HALL_EFFECT),
    elevatorMotor(new valor::PhoenixController(valor::PhoenixControllerType::KRAKEN_X60, CANIDs::ELEV_WHEEL, valor::NeutralMode::Brake, true, "baseCAN")),
    // scorerMotor(new valor::PhoenixController(valor::PhoenixControllerType::KRAKEN_X60, CANIDs::SCORER_WHEEL, valor::NeutralMode::Brake, false, "baseCAN")),
    lidarSensor(_robot, "Front Lidar Sensor", CANIDs::FRONT_LIDAR_SENSOR)
    {

        frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
    pathplanner::NamedCommands::registerCommand("Score sequence Scorer", std::move(
        frc2::SequentialCommandGroup(
            frc2::InstantCommand(
                [this]() {
                    
                    state.scoringState = Scorer::SCORING_SPEED::SCORING;
                }
            ),
            frc2::WaitCommand(1_s),
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

        init();

    }

    void Scorer::resetState()
    {
        state.scoringState = SCORING_SPEED::HOLD;
        state.coralState = ELEV_LVL::MANUAL;

    }


    void Scorer::init()
    {
        elevatorMotor->setGearRatios(MOTOR_TO_SENSOR, SENSOR_TO_MECH);
        // scorerMotor->setGearRatios(1, 1);

        valor::PIDF elevatorPID;
        elevatorPID.maxVelocity = elevatorMotor->getMaxMechSpeed();
        elevatorPID.maxAcceleration = elevatorMotor->getMaxMechSpeed()/(1.0_s/2);
        elevatorPID.P =ELEV_K_P ;
        elevatorPID.error = ELEV_K_ERROR;
        elevatorPID.aFF = ELEV_K_AFF;
        elevatorPID.aFFType = valor::FeedForwardType::LINEAR;

        valor::PIDF scorerPID;
        // scorerPID.maxVelocity = scorerMotor->getMaxMechSpeed();
        // scorerPID.maxAcceleration = scorerMotor->getMaxMechSpeed()/(1.0_s/2);
        scorerPID.P = SCORER_K_P;
        
        // scorerMotor->setPIDF(scorerPID, 0);
        elevatorMotor->setPIDF(elevatorPID, 0);

        elevatorMotor->setForwardLimit(ELEVATOR_FORWARD_LIMIT);
        elevatorMotor->setReverseLimit(ELEVATOR_REVERSE_LIMIT);
        elevatorMotor->applyConfig();


       hallEffectDebounceSensor.setGetter([this] { return !hallEffectDigitalSensor.Get();});
        hallEffectDebounceSensor.setRisingEdgeCallback([this] {
            std::cout << "HALLEFFECT RESET FOR ELEVATOR" << std::endl;
            elevatorMotor->setEncoderPosition(0_tr);
         });
        resetState();

        // scorerDebounceSensor.setGetter([this] { return this->isBeamBroken();});
        // scorerDebounceSensor.setRisingEdgeCallback([this] {
        //  state.sensorTwoTripped = true;
        //  });
        // resetState();


    }

   void Scorer::assessInputs()
{
    if (operatorGamepad == nullptr || !operatorGamepad->IsConnected())
        return;

    if (operatorGamepad->rightStickYActive()) {
        state.coralState = MANUAL;
        state.manualSpeed = operatorGamepad->rightStickY(2) * 12_V;
    } else if (operatorGamepad->GetYButtonPressed()){
        state.coralState = ELEV_LVL::HP;
    } else if (operatorGamepad->GetYButtonPressed()){
        state.coralState = ELEV_LVL::TROUGH;
    }
    // } else if (driverGamepad->GetYButtonPressed()) {
    //     state.coralState = drivetrain->state.getTag ? ELEV_LVL::HP : ELEV_LVL::STOWED;
    // } else if (driverGamepad->GetAButtonPressed()) {
    //     state.coralState = drivetrain->state.getTag ? ELEV_LVL::TROUGH: ELEV_LVL::STOWED;
    // }



}

    void Scorer::analyzeDashboard()
    {

    }



    void Scorer::assignOutputs()
    {
        if (state.coralState == ELEV_LVL::MANUAL){

            elevatorMotor->setPower(state.manualSpeed);
        } else{
            state.targetHeight = coralHMap[state.coralState];
            units::turn_t targetRotations = (state.targetHeight * (1/GEAR_CIRCUMFERENCE));
            elevatorMotor->setPosition(targetRotations);
        }
    
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
            "Target position",
            [this] {return state.targetHeight.value();},
            nullptr
        );

        builder.AddDoubleProperty(
            "Current position (in)",
            [this] {return (elevatorMotor->getPosition() * (GEAR_CIRCUMFERENCE)).value();},
            nullptr
        );

         builder.AddDoubleProperty(
            "Current position (inches version 2)",
            [this] { return (elevatorMotor->getPosition().value() * 1.432 * M_PI); },
            nullptr
        );

    }


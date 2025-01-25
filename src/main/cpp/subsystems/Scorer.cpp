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

#define SCORER_MAX_SPEED units::angular_velocity::turns_per_second_t (12.469)
#define SCORER_MAX_ACCEL units::angular_acceleration::turns_per_second_squared_t (0)
#define SCORER_K_P 0


#define CORAL_LEVEL_2_HEIGHT 1.5f  // Example height for Coral Level 2
#define CORAL_LEVEL_3_HEIGHT 3.5f  // Example height for Coral Level 3
#define CORAL_LEVEL_4_HEIGHT 5.5f  // Example height for Coral Level 4
#define TROUGH_HEIGHT 0.0f        // Trough position (ground level)
#define MOTOR_TO_SENSOR 8.02f
#define SENSOR_TO_MECH 1.0f
#define ELEVATOR_TOLERANCE 0.5f
#define GEAR_CIRCUMFERENCE units::meter_t{1.432_in} * M_PI // meters
#define CONVERSION_FACTOR units::turn_t{1} / GEAR_CIRCUMFERENCE // turns/meter

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
        scorerMotor->setGearRatios(MOTOR_TO_SENSOR, SENSOR_TO_MECH);
        init();

    }

    void Scorer::resetState()
    {
        state.scoringState = SCORING_SPEED::HOLD;
        state.coralState = ELEV_LVL::HP;

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

        scorerDebounceSensor.setGetter([this] { return this->isBeamBroken();});
        scorerDebounceSensor.setRisingEdgeCallback([this] {
         state.sensorTwoTripped = true;
         });
        resetState();
    }

   void Scorer::assessInputs()
{
    if (driverGamepad == nullptr || !driverGamepad->IsConnected() ||
        operatorGamepad == nullptr || !operatorGamepad->IsConnected())
        return;

    if (operatorGamepad->rightStickYActive()) {
        state.coralState = MANUAL;
    } else {
        if (state.sensorTwoTripped) {
            if (driverGamepad->GetAButton()) {
                state.coralState = drivetrain->state.getTag ? ELEV_LVL::FOUR : ELEV_LVL::STOWED;
            } 
            else if (driverGamepad->GetBButton()) {
                state.coralState = drivetrain->state.getTag ? ELEV_LVL::THREE : ELEV_LVL::STOWED;
            }
            else if (driverGamepad->GetXButton()) {
                state.coralState = drivetrain->state.getTag ? ELEV_LVL::TWO : ELEV_LVL::STOWED;
            }
            else if (driverGamepad->GetYButton()) {
                state.coralState = drivetrain->state.getTag ? ELEV_LVL::TROUGH : ELEV_LVL::STOWED;
            }
            else {
                state.coralState = ELEV_LVL::HP;
            }
        } else {
            state.coralState = ELEV_LVL::HP;
        }
    }

     //units::turn_t currentPosition = scorerMotor->getPosition();

        //if ((currentPosition.value() - state.targetRotations) <= ELEVATOR_TOLERANCE) {
}


    void Scorer::analyzeDashboard()
    {

    }

     bool Scorer::isBeamBroken() 
     {
    //     const units::millimeter_t beamBreakThreshold{100};  
    //     units::millimeter_t currentDistance = lidarSensor.getLidarData();
    //     return currentDistance < beamBreakThreshold;
    }

    


    void Scorer::assignOutputs()
    {
        if (state.coralState == ELEV_LVL::MANUAL){
            joystickInput = operatorGamepad->leftStickY(2);
            motorVoltage = joystickInput * 12.0;
            scorerMotor->setPower(units::volt_t(motorVoltage));
        } else{
            state.targetHeight = coralHMap[state.coralState];
            targetRotations = (state.targetHeight * CONVERSION_FACTOR);
            scorerMotor->setPosition(targetRotations);
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
            "Scorer Vel",
            [this] {return scorerMotor->getSpeed().to<double>();},
            nullptr
        );
        builder.AddDoubleProperty(
            "Scorer current",
            [this] {return scorerMotor->getCurrent().to<double>();},
            nullptr
        );

        builder.AddDoubleProperty(
            "Target position",
            [this] {return state.targetHeight.value();},
            nullptr
        );

        builder.AddDoubleProperty(
            "Current position",
            [this] {return (scorerMotor->getPosition() * (1/CONVERSION_FACTOR)).value();},
            nullptr
        );

    }


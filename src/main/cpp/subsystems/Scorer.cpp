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
#define MOTOR_TO_MECH_IN 6000/SENSOR_TO_MECH*

#define SCORER_K_P 0
#define INTAKE_SPEED 4_V
#define SCORE_SPEED -4_V

#define ELEVATOR_FORWARD_LIMIT 5.75_tr
#define ELEVATOR_REVERSE_LIMIT 0.0_tr
#define ELEVATOR_OFFSET 3_in

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
    scorerMotor(new valor::PhoenixController(valor::PhoenixControllerType::KRAKEN_X60, CANIDs::SCORER_WHEEL, valor::NeutralMode::Coast, false, "baseCAN")),
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
        state.elevState = ELEV_LVL::MANUAL;
        state.gamePiece = CORAL;

    }

    bool Scorer::hallEffectSensorActive(){
        return !hallEffectDigitalSensor.Get();
    }


    void Scorer::init()
    {
        state.hasZeroed = hallEffectSensorActive();
        elevatorMotor->setGearRatios(MOTOR_TO_SENSOR, SENSOR_TO_MECH);
        elevatorMotor->enableFOC(true);
        scorerMotor->setGearRatios(1, 1);

        valor::PIDF elevatorPID;
        elevatorPID.maxVelocity = elevatorMotor->getMaxMechSpeed();
        elevatorPID.maxAcceleration = elevatorMotor->getMaxMechSpeed()/(1.0_s/5);
        elevatorPID.P =ELEV_K_P ;
        elevatorPID.error = ELEV_K_ERROR;
        elevatorPID.aFF = ELEV_K_AFF;
        elevatorPID.aFFType = valor::FeedForwardType::LINEAR;

        valor::PIDF scorerPID;
        scorerPID.maxVelocity = scorerMotor->getMaxMechSpeed();
        scorerPID.maxAcceleration = scorerMotor->getMaxMechSpeed()/(1.0_s/2);
        scorerPID.P = SCORER_K_P;
        
        // scorerMotor->setPIDF(scorerPID, 0);
        elevatorMotor->setPIDF(elevatorPID, 0);

        elevatorMotor->setForwardLimit(ELEVATOR_FORWARD_LIMIT);
        elevatorMotor->setReverseLimit(ELEVATOR_REVERSE_LIMIT);
        elevatorMotor->applyConfig();


       hallEffectDebounceSensor.setGetter([this] { return hallEffectSensorActive();});
        hallEffectDebounceSensor.setRisingEdgeCallback([this] {
            state.hasZeroed = true;
            std::cout << "HALLEFFECT RESET FOR ELEVATOR" << std::endl;
            elevatorMotor->setEncoderPosition(0_tr);
         });



        posMap[GAME_PIECE::CORAL][ELEV_LVL::STOWED] = units::meter_t(5_in);
        posMap[GAME_PIECE::CORAL][ELEV_LVL::HP] = units::meter_t(5_in);
        posMap[GAME_PIECE::CORAL][ELEV_LVL::TROUGH] = units::meter_t(13.57_in);
        posMap[GAME_PIECE::CORAL][ELEV_LVL::TWO] = units::meter_t(17.0_in);
        posMap[GAME_PIECE::CORAL][ELEV_LVL::THREE] = units::meter_t(25.05_in);
        posMap[GAME_PIECE::CORAL][ELEV_LVL::FOUR] = units::meter_t(5_in);

        posMap[GAME_PIECE::ALGEE][ELEV_LVL::TROUGH] = units::meter_t(5_in); //Processor
        posMap[GAME_PIECE::ALGEE][ELEV_LVL::TWO] = units::meter_t(5_in);
        posMap[GAME_PIECE::ALGEE][ELEV_LVL::THREE] = units::meter_t(5_in);
        posMap[GAME_PIECE::ALGEE][ELEV_LVL::FOUR] = units::meter_t(5_in); //Net
        
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

    if (driverGamepad == nullptr || !driverGamepad->IsConnected())
        return;

    // if(operatorGamepad->rightTriggerActive())
    // {
    //     state.gamePiece = CORAL;
    // }

    if(operatorGamepad->leftTriggerActive())
    {
        state.gamePiece = ALGEE;
    } else if(operatorGamepad->rightTriggerActive()){
        state.gamePiece = CORAL;
    }
      
    if (operatorGamepad->leftStickYActive()) {
        state.elevState = MANUAL;
        state.manualSpeed = operatorGamepad->leftStickY(2) * 12_V;
    } else if (operatorGamepad->GetYButton()){
        if (driverGamepad->leftTriggerActive()) {state.elevState = ELEV_LVL::FOUR;} 
        else {state.elevState = ELEV_LVL::STOWED;}
    } else if (operatorGamepad->GetBButton()) {
        if (driverGamepad->leftTriggerActive()) {state.elevState = ELEV_LVL::THREE;} 
        else {state.elevState = ELEV_LVL::STOWED;} 
    } else if (operatorGamepad->GetAButton()) {
        if (driverGamepad->leftTriggerActive()) {state.elevState = ELEV_LVL::TWO;} 
        else {state.elevState = ELEV_LVL::STOWED;}
    } else if (operatorGamepad->GetXButton()){
        if (driverGamepad->leftTriggerActive()) {state.elevState = ELEV_LVL::TROUGH;} 
        else {state.elevState = ELEV_LVL::STOWED;}
    } else if(operatorGamepad->DPadRight()){
        state.elevState = ELEV_LVL::STOWED;
    }


    if (driverGamepad->GetRightBumperButton()) {
        state.scoringState =  SCORING_SPEED::SCORING;
    }
    else if (driverGamepad->GetLeftBumperButton()) {
        state.scoringState = SCORING_SPEED::INTAKING;
    } else{
        state.scoringState = SCORING_SPEED::HOLD;
    }


// if(driverGamepad->GetRightBumperButton()){
//     state.scoringState = SCORING_SPEED::INTAKING;
// } else if (driverGamepad->GetRightTrigge

//     units::turn_t currentPosition = elevatorMotor->getPosition() * CONVERSION_FACTOR;
//         if (drivetrain->state.getTag) {
//             if (abs((currentPosition - state.targetHeight).to<double>()) <= ELEVATOR_TOLERANCE) {
//                 state.scoringState = SCORING_SPEED::SCORING;
//             } else {
//                 state.scoringState = SCORING_SPEED::HOLD;
//             }
//         } else {
//             state.scoringState = SCORING_SPEED::HOLD;
//         }
//     } else {
//         state.scoringState = SCORING_SPEED::HOLD;
//     }  

} 


    void Scorer::analyzeDashboard()
    {

    }

    units::meter_t Scorer::convertToMechSpace(units::turn_t turns) 
    {
        return units::meter_t{turns * units::meter_t {1.432_in * M_PI}/1_tr} + ELEVATOR_OFFSET;
    }

    units::turn_t Scorer::convertToMotorSpace(units::meter_t meters)     
    {
        return (meters - ELEVATOR_OFFSET) / units::meter_t {1.432_in * M_PI} * 1_tr;
    }


    void Scorer::assignOutputs()
    {
        if (state.hasZeroed) {
            if (state.elevState == ELEV_LVL::MANUAL) {
                elevatorMotor->setPower(state.manualSpeed);
            } else {
                state.targetHeight = posMap[state.gamePiece][state.elevState];
                units::turn_t targetRotations = convertToMotorSpace(state.targetHeight);
                elevatorMotor->setPosition(targetRotations);
            }
        } else {
            elevatorMotor->setPower(-2.0_V);
        }

        if (state.scoringState == SCORING_SPEED::INTAKING){
            scorerMotor->setPower(INTAKE_SPEED);
        }
        else if(state.scoringState == SCORING_SPEED::SCORING){
            scorerMotor->setPower(SCORE_SPEED);
    
        }else{
            scorerMotor->setPower(units::volt_t(0));
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
            "State elevator level",
            [this] { return state.elevState; },
            nullptr
        );

        builder.AddDoubleProperty(
            "Target position",
            [this] {return state.targetHeight.value();},
            nullptr
        );
         builder.AddDoubleProperty(
            "Current position (in)",
            [this] { return units::inch_t{convertToMechSpace(elevatorMotor->getPosition())}.value();},
            nullptr
        );
        builder.AddBooleanProperty(
            "zeroed?",
            [this] { return hallEffectSensorActive();},
            nullptr
        );

    }



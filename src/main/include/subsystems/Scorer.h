# pragma once 
#include "Drivetrain.h"
#include "valkyrie/BaseSubsystem.h"
#include "valkyrie/controllers/PhoenixController.h"
#include "Constants.h"
#include <vector>
#include "valkyrie/sensors/GrappleSensor.h"
#include "valkyrie/controllers/PIDF.h"
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/DigitalInput.h>
#include "valkyrie/sensors/DebounceSensor.h"
#include <frc2/command/FunctionalCommand.h>
#include <unordered_map>
#include "valkyrie/Gamepad.h"
#include <ctre/phoenix6/core/CoreCANdi.hpp>


class Scorer : public valor::BaseSubsystem
{
public:

    Scorer(frc::TimedRobot *robot, Drivetrain *drivetrain);
    
    void resetState();
     
    void init();
    units::meter_t convertToMechSpace(units::turn_t turns);
    units::turn_t convertToMotorSpace(units::meter_t meters);
    void assessInputs();
    void analyzeDashboard();
    void assignOutputs();
    void InitSendable(wpi::SendableBuilder& builder);
    frc2::CommandPtr createScoringSequence();
    frc2::CommandPtr  elevatorSequence();

    enum SCORING_SPEED
    {
        HOLD,
        INTAKING,
        SCORING,
    };

     enum ELEV_LVL
    {
        MANUAL,
        STOWED,
        HP,
        ONE,
        TWO,
        THREE,
        FOUR,
    };

     enum GAME_PIECE
    {
        CORAL,
        ALGEE,
    };

    enum SCOPED_STATE
    {
        UNSCOPED,
        SCOPED,
    };

    struct x
    {
        SCORING_SPEED scoringState;
        ELEV_LVL elevState;
        bool sensorTwoTripped;
        units::meter_t targetHeight;
        units::volt_t manualSpeed;
        bool hasZeroed;
        GAME_PIECE gamePiece;
        SCOPED_STATE scopedState;

    } state;

    std::unordered_map<std::string, ELEV_LVL> elevMap = {
        {"MANUAL", ELEV_LVL::MANUAL},
        {"STOW", ELEV_LVL::STOWED},
        {"HP", ELEV_LVL::HP},      
        {"ONE", ELEV_LVL::ONE},
        {"TWO", ELEV_LVL::TWO},
        {"THREE", ELEV_LVL::THREE},
        {"FOUR", ELEV_LVL::FOUR}
    };
 
    std::unordered_map<std::string, GAME_PIECE> gamePieceHMap = {
        {"CORAL", GAME_PIECE::CORAL},
        {"ALGEE", GAME_PIECE::ALGEE}
    };

private:
    

    Drivetrain *drivetrain;
    bool hallEffectSensorActive();
    valor::DebounceSensor scorerDebounceSensor;
    valor::DebounceSensor hallEffectDebounceSensor;
    ctre::phoenix6::hardware::core::CoreCANdi candi;
    valor::PhoenixController *elevatorMotor;
    valor::PhoenixController *scorerMotor;
    valor::GrappleSensor lidarSensor;
    std::map<GAME_PIECE, std::map<ELEV_LVL, units::meter_t>> posMap;
};

# pragma once 
#include "Drivetrain.h"
#include "valkyrie/BaseSubsystem.h"
#include "valkyrie/controllers/PhoenixController.h"
#include "Constants.h"
#include <vector>
#include "valkyrie/sensors/GrappleSensor.h"
#include "valkyrie/sensors/CANRangeSensor.h"
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
#include "valkyrie/sensors/CurrentSensor.h"


class Scorer : public valor::BaseSubsystem
{
public:

    Scorer(frc::TimedRobot *robot);
    
    void resetState();
     
    void init();
    void assessInputs();
    void analyzeDashboard();
    void assignOutputs();
    void InitSendable(wpi::SendableBuilder& builder);

    frc2::CommandPtr createScoringSequence();
    frc2::CommandPtr createElevatorSequence();
    void stageCoral();

    enum STAGING_SEQ
    {
        NOT_STAGE,
        STAGING,
        DONE_STAGE
    };

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
        GAME_PIECE gamePiece;
        SCOPED_STATE scopedState;

        units::meter_t targetHeight;
        units::volt_t manualSpeed;
    
        bool hasZeroed;
        bool tuning;
        STAGING_SEQ staged;

    } state;

    
    std::map<ELEV_LVL, units::turns_per_second_t> scoringSpeedMap = {
        {ELEV_LVL::ONE, -12_tps},
        {ELEV_LVL::TWO, -15_tps},
        {ELEV_LVL::THREE, -15_tps},
        {ELEV_LVL::FOUR, -15_tps}
    };

private:
    
    units::meter_t convertToMechSpace(units::turn_t turns);
    units::turn_t convertToMotorSpace(units::meter_t meters);

    bool hallEffectSensorActive();

    valor::DebounceSensor scorerDebounceSensor;
    valor::DebounceSensor hallEffectDebounceSensor;

    ctre::phoenix6::hardware::core::CoreCANdi candi;

    valor::PhoenixController *elevatorMotor;
    valor::PhoenixController *scorerMotor;

    valor::GrappleSensor frontRangeSensor;
    valor::CANrangeSensor scorerStagingSensor;

    std::map<GAME_PIECE, std::map<ELEV_LVL, units::meter_t>> posMap;
};

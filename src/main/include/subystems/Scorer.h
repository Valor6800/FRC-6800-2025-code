# pragma once 
#include "Drivetrain.h"
#include "valkyrie/BaseSubsystem.h"
#include "valkyrie/controllers/NeoController.h"
#include "valkyrie/controllers/PhoenixController.h"
#include "Constants.h"
#include <vector>
#include "valkyrie/sensors/GrappleLidarSensor.h"
#include "valkyrie/controllers/PIDF.h"
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/DigitalOutput.h>
#include "valkyrie/sensors/DebounceSensor.h"
#include <frc2/command/FunctionalCommand.h>
#include <unordered_map>
#include "valkyrie/Gamepad.h"



class Scorer : public valor::BaseSubsystem
{
public:

    Scorer(frc::TimedRobot *robot, Drivetrain *drivetrain);
    
    void resetState();
    void init();
    void assessInputs();
    void analyzeDashboard();
    void assignOutputs();
    bool isBeamBroken();
    void InitSendable(wpi::SendableBuilder& builder);

    enum SCORING_SPEED
    {
        INTAKING,
        SCORING,
        HOLD,
    };

     enum ELEV_LVL
    {
        HP,
        STOWED,
        TROUGH,
        TWO,
        THREE,
        FOUR,
        BARGE,
        PROCESSOR,
        ALGEE2,
        ALGEE3,
    };

    struct x
    {
        SCORING_SPEED scoringState;
        ELEV_LVL coralLevel;
        bool sensorTwoTripped;
        double targetRotations;
    } state;

    std::unordered_map<ELEV_LVL, units::meter_t> coralHMap = {
        {ELEV_LVL::HP, 1.3_m},
        {ELEV_LVL::TROUGH, 1.4_m},
        {ELEV_LVL::TWO, 1.5_m},
        {ELEV_LVL::THREE, 1.6_m},
        {ELEV_LVL::FOUR, 1.7_m},
        {ELEV_LVL::BARGE, 1.8_m},
        {ELEV_LVL::PROCESSOR, 1.9_m},
        {ELEV_LVL::ALGEE2, 2.0_m},
        {ELEV_LVL::ALGEE3, 2.1_m}
    };

private:
    Drivetrain *drivetrain;
    valor::DebounceSensor scorerDebounceSensor;
    valor::PhoenixController *elevatorMotor;
    valor::PhoenixController *scorerMotor;
    valor::GrappleLidarSensor lidarSensor;
    double wheelDiameter = 0.1;
    units::turn_t currentPosition;
};

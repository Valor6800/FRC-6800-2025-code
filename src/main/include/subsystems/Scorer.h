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

    enum SCORING_SPEED
    {
        INTAKING,
        SCORING,
        HOLD,
    };

     enum ELEV_LVL
    {
        MANUAL,
        HP,
        STOWED,
        TROUGH,
        TWO,
        THREE,
        FOUR,
        BARGE,
        PROCESSOR,
    };

     enum ALGEE_OR_CORAL
    {
        ALGEE,
        CORAL,
    };

    enum ALGEE_LVL
    {
        ALGEE2,
        ALGEE3,
    };



    struct x
    {
        SCORING_SPEED scoringState;
        ELEV_LVL coralState;
        bool sensorTwoTripped;
        units::meter_t targetHeight;
        units::volt_t manualSpeed;
        bool hasZeroed;
        ALGEE_OR_CORAL algeeOrCoral;
        ALGEE_LVL algeeLevel;
    } state;



    std::unordered_map<ELEV_LVL, units::meter_t> coralHMap = {
        {ELEV_LVL::HP, 5_in},
        {ELEV_LVL::TROUGH, 13.57_in},
        {ELEV_LVL::TWO, 17.0_in},
        {ELEV_LVL::THREE, 25.05_in},
        {ELEV_LVL::FOUR, 0.5_m},
        {ELEV_LVL::BARGE, 1.8_m},
        {ELEV_LVL::PROCESSOR, 1.9_m}
    };

    std::unordered_map<ALGEE_LVL, units::meter_t> algeeHmap = {
    {ALGEE_LVL::ALGEE2, 2.0_m},
    {ALGEE_LVL::ALGEE3, 2.1_m}
    };

private:

    Drivetrain *drivetrain;
    bool hallEffectSensorActive();
    valor::DebounceSensor scorerDebounceSensor;
    valor::DebounceSensor hallEffectDebounceSensor;
    frc::DigitalInput hallEffectDigitalSensor;  
    valor::PhoenixController *elevatorMotor;
    valor::PhoenixController *scorerMotor;
    valor::GrappleSensor lidarSensor;
    bool hallEffectSensorActive();

};

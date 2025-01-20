#pragma once
#include "valkyrie/drivetrain/Swerve.h"
#include "valkyrie/controllers/PhoenixController.h"
#include "valkyrie/sensors/AprilTagsSensor.h"
#include <frc2/command/FunctionalCommand.h>

#define SWERVE_COUNT 4



/**
 * @brief Quick way to select the drive motor controller
 * To change what motor controller runs the drive motor, change this to either:
 * * valor::PhoenixController
 * * valor::NeoController
 */
typedef valor::PhoenixController SwerveDriveMotor;

/**
 * @brief Quick way to select the azimuth motor controller
 * To change what motor controller runs the azimuth motor, change this to either:
 * * valor::PhoenixController
 * * valor::NeoController
 */
typedef valor::PhoenixController SwerveAzimuthMotor;

/**
 * @brief Subsystem - Drivetrain
 * 
 * Subsystem responsible for driving the robot chassis, and housing all the logic to control the
 * 4 swerve modules on the robot.
 */
class Drivetrain : public valor::Swerve<SwerveAzimuthMotor, SwerveDriveMotor>
{
public:

     /**
      * @brief Construct a new Drivetrain object
      * 
      * @param robot Top level robot object to parse out smart dashboard and table information
      */
     Drivetrain(frc::TimedRobot *robot);

     /**
      * @brief Destroy the Drivetrain object
      * 
      * Drivetrain objects have member objects on the heap - need a destructor to take care of memory on destruction
      */
     ~Drivetrain();

     /**
      * @brief Initialize the drivetrain
      * 
      * Includes:
      * * Calibrating the pigeon
      * * Configuring each swerve module (including controllers for azimuth and drive motors)
      * * Setting the PID values for the Azimuth controller
      * * Resetting the drivetrain state
      */
     void init();

     void assessInputs();
     void analyzeDashboard();
     void assignOutputs();
     void resetState();

     void InitSendable(wpi::SendableBuilder& builder);

     struct x
     {
          // Use in auto to toggle between vision pose and normal pose
          bool useCalculatedEstimator;

          units::second_t startTimestamp; // generic
          
          struct { units::acceleration::meters_per_second_squared_t x,y,z; } accel;
     } state;

     frc2::FunctionalCommand* getResetOdom();

     units::meters_per_second_t getRobotSpeeds();

     // void setDriveMotorNeutralMode(valor::NeutralMode mode);
     double teleopStart;

     double doubtX, doubtY;

private:

     std::vector<std::pair<SwerveAzimuthMotor*, SwerveDriveMotor*>> generateModules();

     valor::PIDF xPIDF;
     valor::PIDF thetaPIDF;
     
     std::vector<valor::AprilTagsSensor*> aprilTagSensors;
     // valor::GrappleLidarSensor lidarSensor;

     units::meter_t visionAcceptanceRadius;

    ctre::phoenix6::swerve::requests::ApplyRobotSpeeds autoRobotSpeedsRequest;
};

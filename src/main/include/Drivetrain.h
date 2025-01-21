#pragma once
#include "valkyrie/drivetrain/Swerve.h"
#include "valkyrie/controllers/PhoenixController.h"
#include "valkyrie/sensors/AprilTagsSensor.h"
#include <frc2/command/FunctionalCommand.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/smartdashboard/Field2d.h>

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
     std::vector<units::ampere_t> getCurrents();

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

     const units::radians_per_second_t MAX_ROTATION_VEL = 16_rad_per_s;
     const units::radians_per_second_squared_t MAX_ROTATION_ACCEL = 12_rad_per_s_sq;
     const units::meters_per_second_t MAX_TRANSLATION_VEL = 5.5_mps;
     const units::meters_per_second_squared_t MAX_TRANSLATION_ACCEL = 1_mps_sq;
     // frc::TrapezoidProfile<units::radian_t>::Constraints rot_constraints{MAX_ROTATION_VEL, MAX_ROTATION_ACCEL};
     // frc::ProfiledPIDController<units::radian_t> rot_controller{3, 0, 0.5};
     frc::TrapezoidProfile<units::meter>::Constraints trans_constraints{MAX_TRANSLATION_VEL, MAX_TRANSLATION_ACCEL};
     frc::ProfiledPIDController<units::meter> trans_controller{1, 0, 0, trans_constraints};

     valor::PIDF xPIDF;
     valor::PIDF thetaPIDF;

     bool alignToTarget;
     units::meter_t horizontalDistance;

     frc::Pose3d testAprilTag{
          frc::Translation3d{
               209.49_in,
               158.5_in,
               12.13_in
          },
          frc::Rotation3d{
               0_deg,
               0_deg,
               0_deg
          }
     };

     nt::StructPublisher<frc::Pose3d> aprilTagPosePublisher;
     nt::StructPublisher<frc::Pose2d> ppTargetPosePublisher;
     nt::StructArrayPublisher<frc::Pose2d> ppActivePathPublisher;
     
     std::vector<valor::AprilTagsSensor*> aprilTagSensors;
     // valor::GrappleLidarSensor lidarSensor;

     units::meter_t visionAcceptanceRadius;

    ctre::phoenix6::swerve::requests::ApplyRobotSpeeds autoRobotSpeedsRequest;
};

#pragma once
#include "valkyrie/drivetrain/Swerve.h"
#include "valkyrie/controllers/PhoenixController.h"
#include "valkyrie/sensors/AprilTagsSensor.h"
#include <frc2/command/FunctionalCommand.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/smartdashboard/Field2d.h>

#define SWERVE_COUNT 4

class Drivetrain : public valor::Swerve
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
     units::degree_t getTagAngle(int id);

     const units::meters_per_second_t MAX_TRANSLATION_VEL = 5.5_mps;
     const units::meters_per_second_squared_t MAX_TRANSLATION_ACCEL = 1_mps_sq;
     frc::TrapezoidProfile<units::meter>::Constraints trans_constraints{MAX_TRANSLATION_VEL, MAX_TRANSLATION_ACCEL};
     frc::ProfiledPIDController<units::meter> trans_controller{1, 0, 0, trans_constraints};

     valor::PIDF xPIDF;
     valor::PIDF thetaPIDF;

     bool alignToTarget;
     units::meter_t horizontalDistance;

     double xSpeed, ySpeed, rotSpeed;
     units::meters_per_second_t xSpeedMPS, ySpeedMPS;
     units::turns_per_second_t rotSpeedTPS;
     std::optional<int> reefTag;

     nt::StructPublisher<frc::Pose3d> aprilTagPosePublisher;
     nt::StructPublisher<frc::Pose2d> ppTargetPosePublisher;
     nt::StructArrayPublisher<frc::Pose2d> ppActivePathPublisher;

     std::vector<valor::AprilTagsSensor*> aprilTagSensors;
     // valor::GrappleLidarSensor lidarSensor;

     units::meter_t visionAcceptanceRadius;

    ctre::phoenix6::swerve::requests::ApplyRobotSpeeds autoRobotSpeedsRequest;
};

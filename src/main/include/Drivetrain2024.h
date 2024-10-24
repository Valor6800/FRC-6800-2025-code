#pragma once

#include "frc/geometry/Pose3d.h"
#include "units/acceleration.h"
#include "units/length.h"
#include "valkyrie/sensors/AprilTagsSensor.h"
#include "valkyrie/BaseSubsystem.h"
#include "Constants.h"
#include "valkyrie/SwerveModule.h"
#include <map>
#include <string>
#include <vector>
#include "valkyrie/controllers/PhoenixController.h"
#include "valkyrie/controllers/NeoController.h"
#include "valkyrie/controllers/PIDF.h"
#include "valkyrie/sensors/CANdleSensor.h"

#include "PoseTracker.h"

#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/Timer.h>

#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>
#include <frc/TimedRobot.h>
#include <frc2/command/FunctionalCommand.h>

#include <rev/CANSparkMax.h>

#define SWERVE_COUNT 4

/**
 * @brief Subsystem - Drivetrain
 * 
 * Subsystem responsible for driving the robot chassis, and housing all the logic to control the
 * 4 swerve modules on the robot.
 */
class Drivetrain2024 : public valor::BaseSubsystem
{
public:

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
      * @brief Construct a new Drivetrain object
      * 
      * @param robot Top level robot object to parse out smart dashboard and table information
      */
     Drivetrain2024(frc::TimedRobot *robot, valor::CANdleSensor *_leds);

     /**
      * @brief Destroy the Drivetrain object
      * 
      * Drivetrain objects have member objects on the heap - need a destructor to take care of memory on destruction
      */
     ~Drivetrain2024();

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

     enum Alignment{
          SOURCE,
          TRAP,
          AMP,
          LOCK
     };

     struct x
     {
          double xSpeed;
          double ySpeed;
          double rot;

          units::radian_t targetAngle;
          units::angular_velocity::radians_per_second_t angleRPS;
  
          bool adas;
          bool lock;
          
          bool xPose;

          bool isLeveled;
          bool abovePitchThreshold;

          bool isAlign;
          bool ampHeadingTrack;
          bool isHeadingTrack;
          bool thetaLock;
          bool sourceAlign;
          bool trapAlign;
          bool ampAlign;

          bool pitMode;

          int stage;

          int trackingID;
          double visionOdomDiff;

          double matchStart;

          units::meter_t distanceFromSpeaker;
          units::meter_t distanceFromAmp;

          frc::Pose2d visionPose;
          frc::Pose2d prevVisionPose;

          units::velocity::meters_per_second_t xSpeedMPS;
          units::velocity::meters_per_second_t ySpeedMPS;
          units::angular_velocity::radians_per_second_t rotRPS;

          frc::Pose2d prevPose;

          units::second_t startTimestamp; // generic
          
          bool useCalculatedEstimator; // only during auto

          bool manualFlag;
          
          struct { units::acceleration::meters_per_second_squared_t x,y,z; } accel;
          int pitSequenceCommandIndex;

          double prevError;

          bool backshot;
     } state;
     
     
     /**
      * Drive the robot with given x, y and rotational velocities using open loop velocity control
      * @param vx_mps the desired x velocity component in meters per second
      * @param vy_mps the desired y velocity component in meters per second
      * @param omega_radps the desired rotational velocity component in radians per second
      * @param isFOC true if driving field oriented
      */
     void drive(units::velocity::meters_per_second_t vx_mps, units::velocity::meters_per_second_t vy_mps, units::angular_velocity::radians_per_second_t omega_radps, bool isFOC);
     void driveRobotRelative(frc::ChassisSpeeds speeds);

     /**
      * Reset the drive encoders to currently read a position of 0
      */
     void resetDriveEncoders();


     units::degree_t getGlobalPitch();

     /**
      * Get the configured swerve modules
      * @return vector of swerve modules
      */
     std::vector<valor::SwerveModule<SwerveAzimuthMotor, SwerveDriveMotor> *> getSwerveModules();


     /**
      * Returns the current gyro heading of the robot
      * This will be affected by any gyro drift that may have accumulated since last recalibration
      * The angle is continuous from 360 to 361 degrees
      * This allows algorithms that wouldn't want to see a discontinuity in the gyro as it sweeps 
      *   past from 360 to 0 on the second time around.
      * The angle is expected to increase as the gyro turns clockwise
      */
     frc::Rotation2d getHeading(bool);

     //returns angle within the range [-180, 180]
     double angleWrap(double degrees);

     /**
      * Returns the position of the robot on the field in meters
      * @return the pose of the robot (x and y are in meters)
      */
     frc::Pose2d getPose_m();
     frc::Pose2d getCalculatedPose_m();

     /**
      * Returns the kinematics object in use by the swerve drive
      * @return kinematics object
      */
     frc::SwerveDriveKinematics<SWERVE_COUNT>* getKinematics();

     void angleLock();

     frc2::FunctionalCommand* getResetOdom();

     units::meters_per_second_t getRobotSpeeds();

     void setAlignmentAngle(Drivetrain2024::Alignment align);
     void getSpeakerLockAngleRPS();
     void getAmpLockAngle();
     units::radian_t getAngleError();
     double clampAngleRadianRange(units::radian_t angle, double max);
     units::meter_t getDistanceFromSpeaker();
     units::meter_t getDistanceFromAmp();
     frc::Pose2d getPoseFromSpeaker();

     void setXMode();
     bool isWheelSlip(int i);

     frc2::InstantCommand* getSetXMode();

     void setDriveMotorNeutralMode(valor::NeutralMode mode);
     double teleopStart;

     double doubtX, doubtY;
     

private:
     
     double driveMaxSpeed;
     double rotMaxSpeed;

     void setSwerveDesiredState(wpi::array<frc::SwerveModuleState, SWERVE_COUNT> desiredStates, bool isDriveOpenLoop);
     
     wpi::array<frc::SwerveModuleState, SWERVE_COUNT> getModuleStates(units::velocity::meters_per_second_t,
                                                           units::velocity::meters_per_second_t,
                                                           units::angular_velocity::radians_per_second_t,
                                                           bool);
     wpi::array<frc::SwerveModuleState, SWERVE_COUNT> getModuleStates(frc::ChassisSpeeds chassisSpeeds);
     frc::ChassisSpeeds getRobotRelativeSpeeds();

     void configSwerveModule(int);
     void calculateCarpetPose();

     std::vector<valor::SwerveModule<SwerveAzimuthMotor, SwerveDriveMotor> *> swerveModules;
     std::vector<SwerveAzimuthMotor *> azimuthControllers;
     std::vector<SwerveDriveMotor *> driveControllers;

     wpi::array<frc::Translation2d, SWERVE_COUNT> motorLocations;

     wpi::array<frc::SwerveModulePosition, SWERVE_COUNT> getModuleStates();

     frc::SwerveDriveKinematics<SWERVE_COUNT> * kinematics;
     frc::SwerveDrivePoseEstimator<SWERVE_COUNT> * estimator;
     frc::SwerveDrivePoseEstimator<SWERVE_COUNT> * calculatedEstimator;
     frc::Pose2d previousPose;

     valor::PIDF xPIDF;
     valor::PIDF thetaPIDF;
     
     std::vector<valor::AprilTagsSensor*> aprilTagSensors;

     units::meter_t visionAcceptanceRadius;

     PoseTracker currentPoseTracker;
     PoseTracker targetPoseTracker;
     PoseTracker unfilteredPoseTracker;

     bool swerveNoError;
     valor::CANdleSensor *leds;
};

#pragma once

#include "frc/geometry/Pose3d.h" 
#include "units/acceleration.h"
#include "units/length.h"
#include "valkyrie/sensors/AprilTagsSensor.h"
#include "valkyrie/sensors/GrappleSensor.h"
#include "valkyrie/BaseSubsystem.h"
#include "Constants.h"
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

#include "valkyrie/controllers/PhoenixController.h"
#include "valkyrie/controllers/PIDF.h"
#include "valkyrie/drivetrain/Swerve.h"

#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/Timer.h>

#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Transform2d.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "subsystems/CANdle.h"

#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>
#include <frc/TimedRobot.h>
#include <frc2/command/FunctionalCommand.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StructTopic.h>

#include <frc/filter/LinearFilter.h>

#include <ctre/phoenix6/Pigeon2.hpp>
#include <frc/Alert.h>

#define SWERVE_COUNT 4



/**
 * @brief Quick way to select the drive motor controller
 * To change what motor controller runs the drive motor, change this to either:
 * * valor::PhoenixController
 * * valor::NeoController
 */
typedef valor::PhoenixController<> SwerveDriveMotor;

/**
 * @brief Quick way to select the azimuth motor controller
 * To change what motor controller runs the azimuth motor, change this to either:
 * * valor::PhoenixController
 * * valor::NeoController
 */
typedef valor::PhoenixController<> SwerveAzimuthMotor;

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
     Drivetrain(frc::TimedRobot *robot, CANdle& candle);

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

          bool lockingToReef;

          int reefTag;
          bool getTag;
          Direction dir;
          bool aligned;
          double yEstimate;

          bool alignToTarget;
          bool right;
          bool left;

          Constants::Scorer::GAME_PIECE gamePiece;
          Constants::Scorer::ELEVATOR_STATE elevState;
          bool climberAlign;
     } state;

     frc2::FunctionalCommand* getResetOdom();

     frc2::CommandPtr pitSequence();

     units::meters_per_second_t getRobotSpeeds();

     // void setDriveMotorNeutralMode(valor::NeutralMode mode);

     // alings the robot using tags
     void alignAngleTags();

     // set angle based on tag
     void setAngleBasedOnTag(int tagID);

     // align the robot using the position on the field
     void alignAngleZoning();

     void choosePoleDirection(Direction dir, Constants::AprilTag tag);

     void setGamePieceInRobot(Constants::Scorer::GAME_PIECE);

     void setTeleopStartTime();

     double teleopStart;

     double doubtX, doubtY;

private:
     frc2::CommandPtr pitSequenceCommand(const frc::ChassisSpeeds&);
     bool aprilTagOnReef(int id);

     bool hasReset;

     std::vector<std::pair<SwerveAzimuthMotor*, SwerveDriveMotor*>> generateModules(CANdle&);
     std::vector<SwerveAzimuthMotor *> azimuthControllers;

     valor::PIDF xPIDF;
     valor::PIDF thetaPIDF;
     
     std::vector<valor::AprilTagsSensor*> aprilTagSensors;

     units::meter_t visionAcceptanceRadius;
     nt::StructSubscriber<frc::Pose2d> currentPosePathPlanner;
     nt::StructSubscriber<frc::Pose2d> targetPosePathPlanner;
     frc::Transform2d poseErrorPP;
     frc::LinearFilter<double> filter = frc::LinearFilter<double>::MovingAverage(10);
     double unfilteredYDistance;

     valor::CANdleSensor *leds;
     
     nt::StructPublisher<frc::Transform2d> poseErrorPPTopic;
};

#define _USE_MATH_DEFINES
#include "valkyrie/drivetrain/Swerve.h"
#include "valkyrie/controllers/PhoenixController.h"

#include <ctre/phoenix6/swerve/SwerveDrivetrainConstants.hpp>
#include <ctre/phoenix6/swerve/SwerveModuleConstants.hpp>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/geometry/Translation2d.h>
#include <frc/DriverStation.h>
#include <networktables/StructTopic.h>
#include "Constants.h"

#define MODULE_DIFF_XS {1, 1, -1, -1}
#define MODULE_DIFF_YS {1, -1, -1, 1}
 
const units::hertz_t KP_ROTATE(-90);
const units::hertz_t KD_ROTATE(-30);

using namespace valor;

Swerve::Swerve(frc::TimedRobot *_robot, std::string _name) :
    valor::BaseSubsystem(_robot, _name),
    posePublisher{table->GetStructTopic<frc::Pose2d>("Drivetrain Pose").Publish()},
    chassisSpeedsPublisher{table->GetStructTopic<frc::ChassisSpeeds>("Chassis Speeds").Publish()},
    currentModuleStatesPublisher{table->GetStructArrayTopic<frc::SwerveModuleState>("Current Module States").Publish()},
    targetModuleStatesPublisher{table->GetStructArrayTopic<frc::SwerveModuleState>("Target Module States").Publish()}
{}

Swerve::~Swerve()
{}

void Swerve::init()
{
}

void Swerve::assessInputs()
{}

void Swerve::analyzeDashboard()
{
    if (frc::RobotBase::IsSimulation())
        drivetrain.UpdateSimState(20_ms, frc::RobotController::GetBatteryVoltage());
    posePublisher.Set(drivetrain.GetState().Pose);
    chassisSpeedsPublisher.Set(drivetrain.GetState().Speeds);
    currentModuleStatesPublisher.Set(drivetrain.GetState().ModuleStates);
    targetModuleStatesPublisher.Set(drivetrain.GetState().ModuleTargets);
}


void Swerve::assignOutputs()
{}

void Swerve::resetState()
{
    drivetrain.TareEverything();
    rotTest = false;
    strLineTest = false;
}

void Swerve::enableCarpetGrain(double _grainMultiplier, bool _roughTowardsRed)
{
    carpetGrainMultiplier = _grainMultiplier;
    roughTowardsRed = _roughTowardsRed;
    useCarpetGrain = true;
}

void  Swerve::calculateCarpetPose()
{
    static frc::Pose2d previousPose;
    frc::Pose2d newPose = drivetrain.GetState().Pose;
    units::meter_t deltaX = newPose.X() - previousPose.X();
    double factor = 1.0;
    if (roughTowardsRed) {
        factor = deltaX < 0_m ? carpetGrainMultiplier : 1;
    } else {
        factor = deltaX > 0_m ? carpetGrainMultiplier : 1;
    }
    
    drivetrain.AddVisionMeasurement(
        frc::Pose2d{
            factor * deltaX + previousPose.X(),
            newPose.Y(),
            newPose.Rotation()
        },
        frc::Timer::GetFPGATimestamp(),
        {0.1, 0.1, 0.1}
    );
    previousPose = drivetrain.GetState().Pose;
}

void Swerve::InitSendable(wpi::SendableBuilder& builder)
{
    builder.SetSmartDashboardType("Subsystem");
    // builder.AddDoubleProperty(
    //     "Commanded X Speed",
    //     [this] { return xSpeed; },
    //     nullptr
    // );
    // builder.AddDoubleProperty(
    //     "Commanded Y Speed",
    //     [this] { return ySpeed; },
    //     nullptr
    // );
    // builder.AddDoubleProperty(
    //     "Commanded Rot Speed",
    //     [this] { return rotSpeed; },
    //     nullptr
    // );
    // builder.AddDoubleProperty(
    //     "Commanded X Speed MPS",
    //     [this] { return xSpeedMPS.value(); },
    //     nullptr
    // );
    // builder.AddDoubleProperty(
    //     "Commanded Y Speed MPS",
    //     [this] { return ySpeedMPS.value(); },
    //     nullptr
    // );
    // builder.AddDoubleProperty(
    //     "Commanded Rot Speed MPS",
    //     [this] { return rotSpeedRPS.value(); },
    //     nullptr
    // );
    // builder.AddDoubleProperty(
    //     "Actual Raw Pose X",
    //     [this] { return getRawPose().X().template to<double>(); },
    //     nullptr
    // );
    // builder.AddDoubleProperty(
    //     "Actual Raw Pose Y",
    //     [this] { return getRawPose().Y().template to<double>(); },
    //     nullptr
    // );
    // builder.AddDoubleProperty(
    //     "Actual Raw Pose Theta",
    //     [this] { return getRawPose().Rotation().Degrees().template to<double>(); },
    //     nullptr
    // );
    // builder.AddDoubleArrayProperty(
    //     "Actual Raw Pose",
    //     [this] 
    //     { 
    //         std::vector<double> pose;
    //         pose.push_back(getRawPose().X().template to<double>());
    //         pose.push_back(getRawPose().Y().template to<double>());
    //         pose.push_back(getRawPose().Rotation().Radians().template to<double>());
    //         return pose;
    //     },
    //     nullptr
    // );
    // builder.AddDoubleProperty(
    //     "Actual Calculated Pose X",
    //     [this] { return getCalculatedPose().X().template to<double>(); },
    //     nullptr
    // );
    // builder.AddDoubleProperty(
    //     "Actual Calculated Pose Y",
    //     [this] { return getCalculatedPose().Y().template to<double>(); },
    //     nullptr
    // );
    // builder.AddDoubleProperty(
    //     "Actual Calculated Pose Theta",
    //     [this] { return getCalculatedPose().Rotation().Degrees().template to<double>(); },
    //     nullptr
    // );
    // builder.AddDoubleArrayProperty(
    //     "Actual Calculated Pose",
    //     [this] 
    //     { 
    //         std::vector<double> pose;
    //         pose.push_back(getCalculatedPose().X().template to<double>());
    //         pose.push_back(getCalculatedPose().Y().template to<double>());
    //         pose.push_back(getCalculatedPose().Rotation().Radians().template to<double>());
    //         return pose;
    //     },
    //     nullptr
    // );
    // builder.AddDoubleProperty(
    //     "Gyro Pitch",
    //     [this]
    //     {
    //         return pigeon->GetPitch().GetValueAsDouble();
    //     },
    //     nullptr
    // );
    // builder.AddDoubleProperty(
    //     "Gyro Yaw",
    //     [this]
    //     {
    //         return pigeon->GetYaw().GetValueAsDouble();
    //     },
    //     nullptr
    // );
    // builder.AddDoubleProperty(
    //     "Gyro Roll",
    //     [this]
    //     {
    //         return pigeon->GetRoll().GetValueAsDouble();
    //     },
    //     nullptr
    // );
    // builder.AddBooleanProperty(
    //     "Tilted",
    //     [this]
    //     {
    //         double yaw = pigeon->GetYaw().GetValueAsDouble(), roll = pigeon->GetRoll().GetValueAsDouble(); // in degrees
    //         double elevation = std::fabs(yaw) + std::fabs(roll); // not at all but close enough
    //         return elevation > 5.0;
    //     },
    //     nullptr
    // );
    // builder.AddDoubleArrayProperty(
    //     "swerve states",
    //     [this] 
    //     { 
    //         std::vector<double> states;
    //         states.push_back(swerveModules[0]->getState().angle.Degrees().template to<double>());
    //         states.push_back(swerveModules[0]->getState().speed.template to<double>());
    //         states.push_back(swerveModules[1]->getState().angle.Degrees().template to<double>());
    //         states.push_back(swerveModules[1]->getState().speed.template to<double>());
    //         states.push_back(swerveModules[2]->getState().angle.Degrees().template to<double>());
    //         states.push_back(swerveModules[2]->getState().speed.template to<double>());
    //         states.push_back(swerveModules[3]->getState().angle.Degrees().template to<double>());
    //         states.push_back(swerveModules[3]->getState().speed.template to<double>());
    //         return states;
    //     },
    //     nullptr
    // );
    // builder.AddDoubleArrayProperty(
    //     "Robot Velocities",
    //     [this]
    //     {
    //         std::vector<double> states;
    //         states.push_back(getRobotRelativeSpeeds().vx.template to<double>());
    //         states.push_back(getRobotRelativeSpeeds().vy.template to<double>());
    //         return states;
    //     },
    //     nullptr
    // );
    // builder.AddDoubleProperty(
    //     "Max Drive Speed MPS",
    //     [this] {return maxDriveSpeed.value();},
    //     nullptr
    // );
}

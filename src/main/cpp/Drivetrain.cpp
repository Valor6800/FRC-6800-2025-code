#define _USE_MATH_DEFINES
#include "Drivetrain.h"
#include "Constants.h"
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <pathplanner/lib/util/PathPlannerLogging.h>
#include <frc/DriverStation.h>
#include <wpi/print.h>

using namespace pathplanner;

#define TXRANGE  30.0f
#define KPIGEON 2.0f
#define KLIMELIGHT -29.8f
#define KP_LIMELIGHT 0.7f

#define KPX 7.0f // 30
#define KIX 0.0f //0
#define KDX 0.1f //.1

#define KPT 8.0f //15
#define KIT 0.0f
#define KDT 0.0f

const units::meter_t WHEEL_DIAMETER(0.0973_m);
#define AZIMUTH_GEAR_RATIO 13.37f
#define SLIP_FACTOR 1.25f

#define AUTO_VISION_THRESHOLD 4.0f //meters

#define X_TIME 214.85f

#define PIGEON_CAN_BUS "baseCAN" //phoenix

#define VISION_ACCEPTANCE 3.5_m // meters

#define TIME_TELEOP_VERT 105.0f

#define MT2_POSE true

Drivetrain::Drivetrain(frc::TimedRobot *_robot) : 
    valor::Swerve<SwerveAzimuthMotor, SwerveDriveMotor>(_robot, "Drivetrain"),
    teleopStart(999999999999),
    aprilTagPosePublisher{table->GetStructTopic<frc::Pose3d>("April Tag Pose").Publish()},
    ppTargetPosePublisher{table->GetStructTopic<frc::Pose2d>("PP Target Pose").Publish()},
    ppActivePathPublisher{table->GetStructArrayTopic<frc::Pose2d>("PP Active Path").Publish()}
    // lidarSensor(_robot, "Front Lidar Sensor", CANIDs::FRONT_LIDAR_SENSOR)
{
    frc::SmartDashboard::PutData("Drivetrain", this);
    fieldCentricFacingAngleRequest.HeadingController.SetPID(1, 0, 0);
    aprilTagPosePublisher.Set(testAprilTag);
    xPIDF.P = KPX;
    xPIDF.I = KIX;
    xPIDF.D = KDX;

    thetaPIDF.P = KPT;
    thetaPIDF.I = KIT;
    thetaPIDF.D = KDT;

    AutoBuilder::configure(
        [this]() { return drivetrain.GetState().Pose; },
        [this](frc::Pose2d pose) { drivetrain.ResetPose(pose); }, // Method to reset odometry (will be called if your auto has a starting pose)
        [this](){ return drivetrain.GetState().Speeds; }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        [this](frc::ChassisSpeeds speeds) { drivetrain.SetControl(autoRobotSpeedsRequest.WithSpeeds(speeds)); }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        std::make_shared<PPHolonomicDriveController>(
            PIDConstants(xPIDF.P, xPIDF.I, xPIDF.D), // Translation PID constants
            PIDConstants(thetaPIDF.P, thetaPIDF.I, thetaPIDF.D) // Rotation PID constants
        ),
        RobotConfig::fromGUISettings(),
        []() {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            auto alliance = frc::DriverStation::GetAlliance();
            if (alliance) {
                return alliance.value() == frc::DriverStation::Alliance::kRed;
            }
            return false;
        },
        this // Reference to this subsystem to set requirements
    );

    // Current pose is already logged, no need for setLogCurrentPoseCallback
    PathPlannerLogging::setLogTargetPoseCallback([this](const frc::Pose2d& pose) {
        ppTargetPosePublisher.Set(pose);
    });
    PathPlannerLogging::setLogActivePathCallback([this](const std::vector<frc::Pose2d>& path) {
        wpi::println("Setting active path");
        ppActivePathPublisher.Set(path);
    });
    resetState();
    // init();
}

Drivetrain::~Drivetrain(){}

std::vector<std::pair<SwerveAzimuthMotor*, SwerveDriveMotor*>> Drivetrain::generateModules()
{
    std::vector<std::pair<SwerveAzimuthMotor*, SwerveDriveMotor*>> modules;

    valor::PIDF azimuthPID;
    azimuthPID.maxVelocity = Constants::azimuthKVel();
    azimuthPID.maxAcceleration = Constants::azimuthKAcc();
    azimuthPID.P = Constants::azimuthKP();
    azimuthPID.error = 0.0027_tr;

    valor::PIDF drivePID;
    drivePID.setMaxVelocity(Constants::driveKVel(), WHEEL_DIAMETER);
    drivePID.setMaxAcceleration(Constants::driveKAcc(), WHEEL_DIAMETER);
    drivePID.P = Constants::driveKP();
    drivePID.error = 0.0027_tr;

    for (size_t i = 0; i < 4; i++) {
        SwerveAzimuthMotor* azimuthMotor = new SwerveAzimuthMotor(
            valor::PhoenixControllerType::FALCON_FOC,
            CANIDs::AZIMUTH_CANS[i],
            valor::NeutralMode::Brake,
            Constants::swerveAzimuthsReversals()[i],
            PIGEON_CAN_BUS
        );
        azimuthMotor->setGearRatios(Constants::azimuthGearRatio(), 1.0);
        azimuthMotor->setPIDF(azimuthPID, 0);
        azimuthMotor->enableFOC(true);
        azimuthMotor->setupCANCoder(CANIDs::CANCODER_CANS[i], Constants::swerveZeros()[i], false, PIGEON_CAN_BUS);
        azimuthMotor->setContinuousWrap(true);
        azimuthMotor->applyConfig();

        SwerveAzimuthMotor* driveMotor = new SwerveDriveMotor(
            valor::PhoenixControllerType::KRAKEN_X60_FOC,
            CANIDs::DRIVE_CANS[i],
            valor::NeutralMode::Coast,
            Constants::swerveDrivesReversals()[i],
            PIGEON_CAN_BUS
        );
        driveMotor->setGearRatios(1.0, Constants::driveGearRatio());
        driveMotor->setPIDF(drivePID, 0);
        driveMotor->enableFOC(true);
        driveMotor->setOpenLoopRamp(1_s);
        driveMotor->applyConfig();

        modules.push_back(std::make_pair(azimuthMotor, driveMotor));
    }
    return modules;
}

void Drivetrain::resetState()
{
    Swerve::resetState();
    // Roughly facing the center AprilTag
    // drivetrain.ResetTranslation(frc::Translation2d{7_m, 7_m});
    drivetrain.ResetPose(
        frc::Pose2d{
            frc::Translation2d{7_m, 7_m},
            45_deg
        }
    );

    // resetEncoders();
    // resetOdometry(frc::Pose2d{0_m, 0_m, 0_rad});
}

void Drivetrain::init()
{
    // Swerve::init();
}

#include <wpi/print.h>
void Drivetrain::assessInputs()
{
    Swerve::assessInputs();
    frc::Pose2d aprilTagPose = testAprilTag.ToPose2d();
    frc::Pose2d botRelativeToTag = drivetrain.GetState().Pose.RelativeTo(aprilTagPose);
    horizontalDistance = -botRelativeToTag.Y();
    if (driverGamepad->GetAButtonPressed()) {
        alignToTarget = true;
        trans_controller.Reset(horizontalDistance);
        trans_controller.SetGoal(0_m);
    }
}

void Drivetrain::analyzeDashboard()
{
    Swerve::analyzeDashboard();

    // visionAcceptanceRadius = (units::meter_t) table->GetNumber("Vision Acceptance", VISION_ACCEPTANCE.to<double>());

    // for (valor::AprilTagsSensor* aprilLime : aprilTagSensors) {
    //     aprilLime->applyVisionMeasurement(
    //         calcEstimator.get(),
    //         getRobotSpeeds(),
    //         table->GetBoolean("Accepting Vision Measurements", true),
    //         doubtX,
    //         doubtY
    //     );
    // }

    // if (!driverGamepad || !driverGamepad->IsConnected() || !operatorGamepad || !operatorGamepad->IsConnected())
    //     return;

    // if (frc::Timer::GetFPGATimestamp().to<double>() - teleopStart > TIME_TELEOP_VERT && frc::Timer::GetFPGATimestamp().to<double>() - teleopStart < TIME_TELEOP_VERT + 3) {
    //     operatorGamepad->setRumble(true);
    // } else {
    //     operatorGamepad->setRumble(false);
    // }
}

std::vector<units::ampere_t> Drivetrain::getCurrents() {
    std::vector<units::ampere_t> currents;
    for (const auto& mod : drivetrain.GetModules()) {
        currents.insert(
            currents.end(),
            {
                mod->GetDriveMotor().GetSupplyCurrent().GetValue(),
                mod->GetSteerMotor().GetSupplyCurrent().GetValue(),
            }
        );
    }

    return currents;
}

void Drivetrain::assignOutputs()
{
    Swerve::assignOutputs();

    if (alignToTarget) {
        units::meters_per_second_t transSpeed = units::meters_per_second_t{trans_controller.Calculate(horizontalDistance)} + trans_controller.GetSetpoint().velocity;
        frc::Rotation2d transRotation = testAprilTag.Rotation().ToRotation2d() - 90_deg;
        Eigen::Vector2d transVector{transRotation.Cos(), transRotation.Sin()};
        transVector *= transSpeed.value();

        fieldCentricFacingAngleRequest
            .WithVelocityX(units::meters_per_second_t{transVector[0]})
            .WithVelocityY(units::meters_per_second_t{transVector[1]})
            .WithTargetDirection(testAprilTag.Rotation().ToRotation2d().RotateBy(180_deg));
        drivetrain.SetControl(fieldCentricFacingAngleRequest);
    }
}

units::meters_per_second_t Drivetrain::getRobotSpeeds(){
    return units::meters_per_second_t{sqrtf(powf(getRobotRelativeSpeeds().vx.to<double>(), 2) + powf(getRobotRelativeSpeeds().vy.to<double>(), 2))};
}

frc2::FunctionalCommand* Drivetrain::getResetOdom() {
    return new frc2::FunctionalCommand(
        [&]{ // onBegin
            for (valor::AprilTagsSensor* aprilLime : aprilTagSensors) {
                aprilLime->setPipe(valor::VisionSensor::PIPELINE_0);
            }

            state.startTimestamp = frc::Timer::GetFPGATimestamp();
        },
        [&]{ // continuously running
            table->PutNumber("resetting maybe", true);

            for (valor::AprilTagsSensor* aprilLime : aprilTagSensors) {
                if (aprilLime->hasTarget() && (aprilLime->getSensor().ToPose2d().X() > 0_m && aprilLime->getSensor().ToPose2d().Y() > 0_m)) {
                    table->PutNumber("resetting odom", table->GetNumber("resetting odom", 0) + 1);
                    //aprilLime->applyVisionMeasurement(estimator);
                    table->PutBoolean("resetting", true);
                    break;
                } else {
                    table->PutBoolean("resetting", false);
                }
            }
        },
        [&](bool){ // onEnd
                
        },
        [&]{ // isFinished
            return (frc::Timer::GetFPGATimestamp() - state.startTimestamp) > 1.0_s;
        },
        {}
    );
}

// void Drivetrain::setDriveMotorNeutralMode(valor::NeutralMode mode) {
//     for (int i = 0; i < SWERVE_COUNT; i++)
//     {
//         driveControllers[i]->setNeutralMode(mode);
//     }
// }

void Drivetrain::InitSendable(wpi::SendableBuilder& builder)
{
    Swerve::InitSendable(builder);

    builder.SetSmartDashboardType("Subsystem");
}

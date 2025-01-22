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
#define MAKE_VECTOR(angle) Eigen::Vector2d{units::math::cos(angle).value(), units::math::sin(angle).value()}

constexpr frc::Translation2d REEF_POS{176.745_in, 158.5_in};

constexpr int SIMULATION_APRILTAG_ID = 7;
constexpr frc::Pose3d SIMULATION_APRILTAG{
    frc::Translation3d{
        144.0_in,
        158.50_in,
        12.13_in
    },
    frc::Rotation3d{
        0_deg,
        0_deg,
        180_deg
    }
};
constexpr frc::Pose2d SIMULATION_STARTPOSE{
    frc::Translation2d{1_m, 4_m},
    0_deg
};

Drivetrain::Drivetrain(frc::TimedRobot *_robot) : 
    valor::Swerve<SwerveAzimuthMotor, SwerveDriveMotor>(_robot, "Drivetrain"),
    teleopStart(999999999999),
    aprilTagPosePublisher{table->GetStructTopic<frc::Pose3d>("April Tag Pose").Publish()},
    ppTargetPosePublisher{table->GetStructTopic<frc::Pose2d>("PP Target Pose").Publish()},
    ppActivePathPublisher{table->GetStructArrayTopic<frc::Pose2d>("PP Active Path").Publish()}
    // lidarSensor(_robot, "Front Lidar Sensor", CANIDs::FRONT_LIDAR_SENSOR)
{
    frc::SmartDashboard::PutData("Drivetrain", this);
    fieldCentricFacingAngleRequest.HeadingController.SetPID(3, 0, 0.5);
    aprilTagPosePublisher.Set(SIMULATION_APRILTAG);
    xPIDF.P = KPX;
    xPIDF.I = KIX;
    xPIDF.D = KDX;

    thetaPIDF.P = KPT;
    thetaPIDF.I = KIT;
    thetaPIDF.D = KDT;

    for (const auto& [name, pos] : Constants::aprilCameras()) {
        aprilTagSensors.push_back(new valor::AprilTagsSensor{robot, name, pos});
        aprilTagSensors.back()->setPipe(valor::VisionSensor::PIPELINE_0);
    }
    aprilTagSensors[4]->setPipe(valor::VisionSensor::PIPELINE_1);
    aprilTagSensors[4]->setCameraPose(Constants::aprilCameras()[4].second);

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
        ppActivePathPublisher.Set(path);
    });
    resetState();

    trans_controller.SetTolerance(40_mm, .01_mps);
    trans_controller.SetGoal(0_m);
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
    drivetrain.ResetPose(SIMULATION_STARTPOSE);
}

void Drivetrain::init()
{
    // Swerve::init();
}

void Drivetrain::assessInputs()
{
    Swerve::assessInputs();

    if (!driverGamepad || !driverGamepad->IsConnected())
        return;

    if (driverGamepad->GetBackButtonPressed())
        drivetrain.ResetRotation(frc::Rotation2d{0_deg});

    xSpeed = driverGamepad->leftStickY(2);
    ySpeed = driverGamepad->leftStickX(2);
    rotSpeed = driverGamepad->rightStickX(3);

    if (driverGamepad->GetAButtonPressed()) {
        alignToTarget = true;
        // To prevent drivetrain from immediately trying to go to 0 degrees or previous setting
        drivetrain.SetControl(fieldCentricFacingAngleRequest.WithTargetDirection(drivetrain.GetRotation3d().ToRotation2d()));
        trans_controller.Reset(horizontalDistance);
    } else if (driverGamepad->GetAButtonReleased()) {
        alignToTarget = false;
        reefTag.reset();
        drivetrain.SetControl(fieldCentricRequest);
    }
}

void Drivetrain::analyzeDashboard()
{
    Swerve::analyzeDashboard();
    if (alignToTarget) {
        if (frc::RobotBase::IsSimulation()) {
            // reefTag = SIMULATION_APRILTAG_ID;
            fieldCentricFacingAngleRequest.TargetDirection = getTagAngle(SIMULATION_APRILTAG_ID);
            horizontalDistance = drivetrain.GetState().Pose.RelativeTo(SIMULATION_APRILTAG.ToPose2d()).Y();
        } else {
            for (valor::AprilTagsSensor *aprilLime : aprilTagSensors) {
                if (!aprilLime->hasTarget()) continue;
                int id = aprilLime->getTagID();
                if (!reefTag) {
                    // Check if tag belongs on the reef
                    if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed) {
                        if (id >= 6 && id <= 11) reefTag = id;
                    } else if (id >= 17 && id <= 22) reefTag = id;
                    if (reefTag) fieldCentricFacingAngleRequest.TargetDirection = getTagAngle(id);
                } else if (id == reefTag.value()) {
                    // TODO: Only update horizontal distance if new depth is closer than old (if the measurement is more likely to be more accurate)
                    horizontalDistance = aprilLime->get_botpose_targetspace().X();
                }
            }
        }
        if (!reefTag) {
            // Align to center of reef
            fieldCentricFacingAngleRequest.TargetDirection = units::math::atan2(REEF_POS.Y() - drivetrain.GetState().Pose.Y(), REEF_POS.X() - drivetrain.GetState().Pose.X());
            wpi::println("Target direction: {}", fieldCentricFacingAngleRequest.TargetDirection.Degrees());
            xSpeedMPS = 10_mps * xSpeed;
            ySpeedMPS = 10_mps * ySpeed;
        } else {
            // Once center of reef implemented, this check is not necessary
            units::degree_t reefAngle = getTagAngle(reefTag.value());
            Eigen::Vector2d joystickVector{xSpeed, ySpeed};
            Eigen::Vector2d reefVector = MAKE_VECTOR(reefAngle);
            joystickVector = joystickVector.dot(reefVector) * reefVector;

            Eigen::Vector2d pidVector = MAKE_VECTOR(reefAngle - 90_deg) * (units::meters_per_second_t{trans_controller.Calculate(horizontalDistance)} + trans_controller.GetSetpoint().velocity).value();
            Eigen::Vector2d powerVector = pidVector + joystickVector;
            xSpeedMPS = units::meters_per_second_t{powerVector.x()};
            ySpeedMPS = units::meters_per_second_t{powerVector.y()};
        }
    } else {
        xSpeedMPS = 10_mps * xSpeed;
        ySpeedMPS = 10_mps * ySpeed;
        rotSpeedTPS = 10_tps * rotSpeed;
    }

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
        // TargetDirection is already set in analyzeDashboard
        fieldCentricFacingAngleRequest
            .WithVelocityX(xSpeedMPS)
            .WithVelocityY(ySpeedMPS);
    } else {
        fieldCentricRequest
            .WithVelocityX(xSpeedMPS)
            .WithVelocityY(ySpeedMPS)
            .WithRotationalRate(rotSpeedTPS);
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

units::degree_t Drivetrain::getTagAngle(int id) {
    if      (id == 6) return -60_deg;
    else if (id == 7) return 0_deg;
    else if (id == 8) return 60_deg;
    else if (id == 9) return 120_deg;
    else if (id == 10) return 180_deg;
    else if (id == 11) return -120_deg;
    else if (id == 17) return 120_deg;
    else if (id == 18) return 180_deg;
    else if (id == 19) return -120_deg;
    else if (id == 20) return -30_deg;
    else if (id == 21) return 0_deg;
    else if (id == 22) return 30_deg;

    // FIXME: Panic some other way
    return 0_deg;
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

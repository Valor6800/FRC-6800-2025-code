#include "Drivetrain.h"
#include <cmath> 
#include <cstddef>
#include <frc/DriverStation.h>
#include <iostream>
#include <math.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <string>
#include "Constants.h"
#include "Eigen/Core"
#include "frc2/command/Commands.h"
#include "frc2/command/FunctionalCommand.h"
#include "frc2/command/SequentialCommandGroup.h"
#include "units/acceleration.h"
#include "units/base.h"
#include "units/length.h"
#include "units/math.h"
#include "units/velocity.h"
#include "valkyrie/sensors/AprilTagsSensor.h"
#include "units/length.h"
#include "valkyrie/sensors/VisionSensor.h"
#include <frc2/command/InstantCommand.h> 
#include <pathplanner/lib/auto/NamedCommands.h>
#include <unordered_map>
#include <utility>
#include "frc/geometry/Pose3d.h"
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include "frc/geometry/Rotation3d.h"
#include "units/angle.h"
#include <ctre/phoenix6/TalonFX.hpp>

using namespace pathplanner;

#define TXRANGE  30.0f
#define KPIGEON 2.0f
#define KLIMELIGHT -29.8f
#define KP_LIMELIGHT 0.7f

#define KPX 4.0f
#define KIX 0.0f
#define KDX 0.1f

#define KPT 5.0f
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

#define Y_ALIGN_KP 1
#define Y_ALIGN_KD 0.025

// fix these
#define BLUE_REEF_17_ANGLE 120_deg
#define BLUE_REEF_18_ANGLE 180_deg
#define BLUE_REEF_19_ANGLE -120_deg
#define BLUE_REEF_20_ANGLE -30_deg
#define BLUE_REEF_21_ANGLE 0_deg
#define BLUE_REEF_22_ANGLE  30_deg

// these are correct
#define RED_REEF_6_ANGLE -60_deg + 180_deg
#define RED_REEF_7_ANGLE 0_deg + 180_deg
#define RED_REEF_8_ANGLE 60_deg - 180_deg
#define RED_REEF_9_ANGLE 120_deg - 180_deg
#define RED_REEF_10_ANGLE 180_deg - 180_deg
#define RED_REEF_11_ANGLE -120_deg + 180_deg

#define POLE_OFFSET 6.5_in

Drivetrain::Drivetrain(frc::TimedRobot *_robot) : 
    valor::Swerve<SwerveAzimuthMotor, SwerveDriveMotor>(
        _robot,
        "SwerveDrive",
        generateModules(),
        Constants::moduleDiff(),
        WHEEL_DIAMETER
    ),
    teleopStart(999999999999)
{
    xPIDF.P = KPX;
    xPIDF.I = KIX;
    xPIDF.D = KDX;

    thetaPIDF.P = KPT;
    thetaPIDF.I = KIT;
    thetaPIDF.D = KDT;

    table->PutNumber("Rot_Pos_Tol", Swerve::rotPosTolerance.to<double>());
    table->PutNumber("Rot_Vel_Tol", Swerve::rotVelTolerance.to<double>());

    table->PutNumber("Y_Pos_Tol", Swerve::yPosTolerance.to<double>());
    table->PutNumber("Y_Vel_Tol", Swerve::yVelTolerance.to<double>());

    Swerve::Y_KP = Y_ALIGN_KP;
    Swerve::Y_KD = Y_ALIGN_KD;

    table->PutNumber("Y_KP", Swerve::Y_KP);
    table->PutNumber("Y_KD", Swerve::Y_KD);

    table->PutNumber("Rot_KP", Swerve::ROT_KP);
    table->PutNumber("Rot_KD", Swerve::ROT_KD);

    table->PutNumber("Pole Offset", POLE_OFFSET.to<double>());

    table->PutNumber("Vision Std", 3.0);
    table->PutNumber("Vision Acceptance", VISION_ACCEPTANCE.to<double>() );
    table->PutNumber("KPLIMELIGHT", KP_LIMELIGHT);
    table->PutBoolean("Accepting Vision Measurements", true);

    setRotAlignOffset(90_deg);

    for (std::pair<const char*, frc::Pose3d> aprilCam : Constants::aprilCameras) {
        aprilTagSensors.push_back(new valor::AprilTagsSensor(robot, aprilCam.first, aprilCam.second));  
        aprilTagSensors.back()->setPipe(valor::VisionSensor::PIPELINE_0);
    }

    aprilTagSensors[4]->setPipe(valor::VisionSensor::PIPELINE_1);
    aprilTagSensors[4]->setCameraPose(Constants::aprilCameras[4].second);

    state.dir = NONE;

    setupGyro(
        CANIDs::PIGEON_CAN,
        PIGEON_CAN_BUS,
        Constants::pigeonMountRoll(),
        Constants::pigeonMountPitch(),
        Constants::pigeonMountYaw()
    );
    
    /*
     * 3.8m/s, 5m/s^2, ~125lbs Apr. 2
     */
    AutoBuilder::configure(
        [this](){ 
            if (state.useCalculatedEstimator) {
                return getCalculatedPose();
            }
            return getRawPose();
        }, // Robot pose supplier
        [this](frc::Pose2d pose){ resetOdometry(pose); }, // Method to reset odometry (will be called if your auto has a starting pose)
        [this](){ return getRobotRelativeSpeeds(); }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        [this](frc::ChassisSpeeds speeds, auto _){ driveRobotRelative(speeds); }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        std::shared_ptr<PPHolonomicDriveController>(new PPHolonomicDriveController(
            PIDConstants(xPIDF.P, xPIDF.I, xPIDF.D), // Translation PID constants
            PIDConstants(thetaPIDF.P, thetaPIDF.I, thetaPIDF.D) // Rotation PID constants
        )),
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
    resetState();
    init();
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

    resetEncoders();
    resetOdometry(frc::Pose2d{0_m, 0_m, 0_rad});
}

void Drivetrain::init()
{
    Swerve::init();
    state.reefTag = -1;
    currentPosePathPlanner = nt::NetworkTableInstance::GetDefault().GetStructTopic<frc::Pose2d>("/PathPlanner/currentPose").Subscribe(frc::Pose2d{});
    targetPosePathPlanner = nt::NetworkTableInstance::GetDefault().GetStructTopic<frc::Pose2d>("/PathPlanner/targetPose").Subscribe(frc::Pose2d{});
}

void Drivetrain::assessInputs()
{
    if (!driverGamepad || !driverGamepad->IsConnected() || !operatorGamepad || !operatorGamepad->IsConnected())
        return;

    if(operatorGamepad->GetRightBumperButton()){
        state.dir = RIGHT;
    } else if(operatorGamepad->GetLeftBumperButton()){
        state.dir = LEFT;
    } else if(operatorGamepad->leftTriggerActive()){
        state.dir = NONE;
    }
    // state.lockingToReef = driverGamepad->GetAButtonPressed();
    state.getTag = false;
    if (driverGamepad->leftTriggerActive() && state.reefTag == -1) {
        state.getTag = true;
        
    } else if (!driverGamepad->leftTriggerActive()) {
        state.reefTag = -1;
        hasReset = false;
    }

    for(valor::AprilTagsSensor* aprilLime : aprilTagSensors) {
        if (aprilLime->hasTarget()) {
            if(
                (frc::DriverStation::GetAlliance() == frc::DriverStation::kRed && 
                aprilLime->getTagID() >= 6 &&
                aprilLime->getTagID() <= 11) || 
                (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue &&
                aprilLime->getTagID() >= 17 &&
                aprilLime->getTagID() <= 22)
            ){ 

                units::radian_t currSkew = aprilLime->getTargetToBotPose().Rotation().ToRotation2d().RotateBy(frc::Rotation2d(90_deg)).Radians();
                if (state.getTag) {
                    std::cout << aprilLime->getName() << " : " << aprilLime->getTagID() << " : " << currSkew.convert<units::degrees>().value() << "\n";
                }
                if (state.getTag && leastSkew > currSkew) {
                    state.reefTag = aprilLime->getTagID();
                    leastSkew = aprilLime->getTargetToBotPose().Rotation().Y();
                }
                if (state.reefTag == aprilLime->getTagID()) {
                    Swerve::yDistance = aprilLime->get_botpose_targetspace().X();
                }
            }
        } 
    }

    if (state.getTag) std::cout << "\n\n";


    Swerve::alignToTarget = driverGamepad->leftTriggerActive();
    if (driverGamepad->leftTriggerActive() && !hasReset) {
        Swerve::resetAlignControllers();
        hasReset = true;
    }

    Swerve::assessInputs();
}

void Drivetrain::analyzeDashboard()
{
    poseErrorPP = currentPosePathPlanner.Get() - targetPosePathPlanner.Get();
    Swerve::ROT_KP = table->GetNumber("Rot_KP", Swerve::ROT_KP);
    Swerve::ROT_KD = table->GetNumber("Rot_KD", Swerve::ROT_KD);

    Swerve::Y_KP = table->GetNumber("Y_KP", Swerve::Y_KP);
    Swerve::Y_KD = table->GetNumber("Y_KD", Swerve::Y_KD);

    Swerve::rotPosTolerance = table->GetNumber("Rot_Pos_Tol", Swerve::rotPosTolerance.to<double>()) * 1_deg;
    Swerve::rotVelTolerance = table->GetNumber("Rot_Vel_Tol", Swerve::rotVelTolerance.to<double>()) * 1_deg_per_s;

    Swerve::yPosTolerance = table->GetNumber("Y_Pos_Tol", Swerve::yPosTolerance.to<double>()) * 1_mm;
    Swerve::yVelTolerance = table->GetNumber("Y_Vel_Tol", Swerve::yVelTolerance.to<double>()) * 1_mps;

    // Swerve::goalAlign = units::meter_t{table->GetNumber("Pole Offset", Swerve::goalAlign.to<double>())};
    choosePoleDirection(state.dir);
    if (state.reefTag != -1){
        state.aligned = (units::math::abs(Swerve::yDistance - Swerve::goalAlign) <= yPosTolerance);
    } else {
        state.aligned = false;
    }

    alignAngleTags(); 

    Swerve::analyzeDashboard();

    visionAcceptanceRadius = (units::meter_t) table->GetNumber("Vision Acceptance", VISION_ACCEPTANCE.to<double>());

    for (valor::AprilTagsSensor* aprilLime : aprilTagSensors) {
        aprilLime->applyVisionMeasurement(
            calcEstimator.get(),
            getRobotSpeeds(),
            table->GetBoolean("Accepting Vision Measurements", true),
            doubtX,
            doubtY
        );
    }

    if (!driverGamepad || !driverGamepad->IsConnected() || !operatorGamepad || !operatorGamepad->IsConnected())
        return;

    if (frc::Timer::GetFPGATimestamp().to<double>() - teleopStart > TIME_TELEOP_VERT && frc::Timer::GetFPGATimestamp().to<double>() - teleopStart < TIME_TELEOP_VERT + 3) {
        operatorGamepad->setRumble(true);
    } else {
        operatorGamepad->setRumble(false);
    }
}

void Drivetrain::assignOutputs()
{
    Swerve::assignOutputs();
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

void Drivetrain::alignAngleTags()
{
    // set each id angle to the correct angle
    // aprilTagSensors[4]->get_botpose_targetspace();
    setAngleBasedOnTag(state.reefTag);
}

void Drivetrain::setAngleBasedOnTag(int tagID)
{
    switch(tagID){
        //red tags
        case 6:
            Swerve::targetAngle = RED_REEF_6_ANGLE; // 6
            break;
        case 7:
            Swerve::targetAngle = RED_REEF_7_ANGLE;
            break;
        case 8:
            Swerve::targetAngle = RED_REEF_8_ANGLE; // 8
            break;
        case 9:
            Swerve::targetAngle = RED_REEF_9_ANGLE;
            break;
        case 10:
            Swerve::targetAngle = RED_REEF_10_ANGLE; // 10
            break;
        case 11:
            Swerve::targetAngle = RED_REEF_11_ANGLE; // 11
            break;
        //blue tags
        case 17:
            Swerve::targetAngle = BLUE_REEF_17_ANGLE; // 17
            break;
        case 18:
            Swerve::targetAngle = BLUE_REEF_18_ANGLE; // 18
            break;
        case 19:
            Swerve::targetAngle = BLUE_REEF_19_ANGLE; // 19
            break;
        case 20:
            Swerve::targetAngle = BLUE_REEF_20_ANGLE; // 20
            break;
        case 21:
            Swerve::targetAngle = BLUE_REEF_21_ANGLE; // 21
            break;
        case 22:
            Swerve::targetAngle = BLUE_REEF_22_ANGLE; // 22
            break;
        default:
            ;
    }
}

void Drivetrain::alignAngleZoning()
{
    auto robotX = getCalculatedPose().X();
    auto robotY = getCalculatedPose().Y();
    if(frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed){
        if(robotX >= 13_m){
            if(robotY >= 4.5_m) Swerve::targetAngle = RED_REEF_8_ANGLE;
            if(robotY <= 3.5_m) Swerve::targetAngle = RED_REEF_6_ANGLE;
            else Swerve::targetAngle = RED_REEF_7_ANGLE;
        }
        else{
            if(robotY >= 4.5_m) Swerve::targetAngle = RED_REEF_9_ANGLE;
            if(robotY <= 3.5_m) Swerve::targetAngle = RED_REEF_11_ANGLE;
            else Swerve::targetAngle = RED_REEF_10_ANGLE;
        }
    }
    // blue alliance TODO
    else{

    }
}

void Drivetrain::choosePoleDirection(Drivetrain::Direction dir){
    switch (dir) {
        case LEFT:
            Swerve::goalAlign = -units::math::abs(POLE_OFFSET);
            break;
        case RIGHT:
            Swerve::goalAlign = units::math::abs(POLE_OFFSET);
            break;
        default:
            Swerve::goalAlign = 0_m;
            break;
    }
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

        builder.AddDoubleArrayProperty(
            "Acceleration",
            [this] {
                std::vector<double> acceleration;
                acceleration.push_back(state.accel.x.to<double>());
                acceleration.push_back(state.accel.y.to<double>());
                acceleration.push_back(state.accel.z.to<double>());
                return acceleration;
            },
            nullptr
        );
        builder.AddBooleanProperty(
            "Bonk!",
            [this] {
                return units::acceleration::meters_per_second_squared_t{
                sqrtf(powf(state.accel.x.to<double>(), 2) + powf(state.accel.y.to<double>(), 2))
            } > 20.0_mps_sq; // ~60 kg bot -> 600 N, 5 measurements * 20ms = .1s, 
                                                                                     // impulse = .1 * 600 = 60 Joules
            },
            nullptr
        );
        builder.AddDoubleProperty(
            "Locking Tag ID",
            [this] {return state.reefTag;},
            nullptr
        );

        builder.AddDoubleProperty(
            "Direction",
            [this] {return state.dir;},
            nullptr
        );
        
        builder.AddBooleanProperty(
            "Aligned",
            [this] {return state.aligned;},
            nullptr
        );
        builder.AddDoubleArrayProperty(
            "Current Pose PP",
            [this] {
                std::vector<double> pose;
                pose.push_back(currentPosePathPlanner.Get().X().to<double>());
                pose.push_back(currentPosePathPlanner.Get().Y().to<double>());
                pose.push_back(currentPosePathPlanner.Get().Rotation().Radians().to<double>());
                return pose;
            },
            nullptr
        );
        builder.AddDoubleArrayProperty(
            "Target Pose PP",
            [this] {
                std::vector<double> pose;
                pose.push_back(targetPosePathPlanner.Get().X().to<double>());
                pose.push_back(targetPosePathPlanner.Get().Y().to<double>());
                pose.push_back(targetPosePathPlanner.Get().Rotation().Radians().to<double>());
                return pose;
            },
            nullptr
        );
        builder.AddDoubleArrayProperty(
            "Pose Error PP",
            [this] {
                std::vector<double> pose;
                pose.push_back(poseErrorPP.X().to<double>());
                pose.push_back(poseErrorPP.Y().to<double>());
                pose.push_back(poseErrorPP.Rotation().Radians().to<double>());
                return pose;
            },
            nullptr
        );
    }

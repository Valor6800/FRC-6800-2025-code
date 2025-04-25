#include "Drivetrain.h"
#include <bitset>
#include <cmath> 
#include <cstddef>
#include <frc/DriverStation.h>
#include <iterator>
#include <limits>
#include <math.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <string>
#include "Constants.h"
#include "frc/geometry/Pose2d.h"
#include "frc2/command/FunctionalCommand.h"
#include "units/acceleration.h"
#include "units/angular_velocity.h"
#include "units/base.h"
#include "units/length.h"
#include "units/math.h"
#include "units/velocity.h"
#include "valkyrie/controllers/PIDF.h"
#include "valkyrie/sensors/AprilTagsSensor.h"
#include "units/length.h"
#include "valkyrie/sensors/VisionSensor.h"
#include <frc2/command/InstantCommand.h> 
#include <pathplanner/lib/auto/NamedCommands.h>
#include <utility>
#include "frc/geometry/Pose3d.h"
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include "frc/geometry/Rotation3d.h"
#include "units/angle.h"
#include "wpi/detail/value_t.h"
#include <ctre/phoenix6/TalonFX.hpp>
#include <units/math.h>
#include <frc/Alert.h>
#include "AprilTagPositions.h"

using namespace pathplanner;

#define LOOP_TIME 0.02

#define TXRANGE  30.0f
#define KPIGEON 2.0f
#define KLIMELIGHT -29.8f
#define KP_LIMELIGHT 0.7f

#define KPX 4.5f // 4
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

// 135 - TELEOP_MAX_TIME - TIME_TO_RUMBLE = Time at which rumble starts
#define TIME_TO_RUMBLE 20_s
#define TELEOP_MAX_TIME 135_s

#define MT2_POSE true

#define Y_FILTER_CONST 0.95 // .99
#define Y_ALIGN_KP 25
#define Y_ALIGN_KI 10
#define Y_ALIGN_KD 0.1

#define Y_WORLD_ALIGN_KP KPX
#define Y_WORLD_ALIGN_KI KIX
#define Y_WORLD_ALIGN_KD KDX

#define X_FILTER_CONST 0.95 // .99
#define X_ALIGN_KP 1
#define X_ALIGN_KI 0
#define X_ALIGN_KD 0

#define X_WORLD_ALIGN_KP KPX
#define X_WORLD_ALIGN_KI KIX
#define X_WORLD_ALIGN_KD KDX
// fix these
#define BLUE_REEF_17_ANGLE 60_deg // -120_deg + 180_deg
#define BLUE_REEF_18_ANGLE 0_deg // 180_deg + 180_deg
#define BLUE_REEF_19_ANGLE -60_deg // 120_deg + 180_deg
#define BLUE_REEF_20_ANGLE -120_deg // -30_deg - 90_deg
#define BLUE_REEF_21_ANGLE 180_deg // 0_deg + 180_deg
#define BLUE_REEF_22_ANGLE  120_deg // 30_deg + 90_deg

// these are correct
#define RED_REEF_6_ANGLE -60_deg + 180_deg
#define RED_REEF_7_ANGLE 0_deg + 180_deg
#define RED_REEF_8_ANGLE 60_deg - 180_deg
#define RED_REEF_9_ANGLE 120_deg - 180_deg
#define RED_REEF_10_ANGLE 180_deg - 180_deg
#define RED_REEF_11_ANGLE -120_deg + 180_deg

#define POLE_OFFSET 6.758_in
#define RIGHT_TO_ROBOT 0.5_in
#define LEFT_TO_ROBOT -1.0_in

#define AA_LEFT_OFFSET 0.0_in // 0.5_in
#define AA_RIGHT_OFFSET 0.0_in // 1.5_in


#define VIABLE_DUNK_SPEED 1.1_mps
#define Y_CONTROLLER_SPEED_LIMIT .15_mps
#define ROT_CONTROLLER_SPEED_LIMIT units::degrees_per_second_t{9}
#define VIABLE_DUNK_DISTANCE 0.28_m
#define AUTO_VIABLE_DUNK_SPEED_L4 1.1_mps

#define Y_ACTIVATION_THRESHOLD 30.0_deg
#define X_SHUTOFF_Y_DISTANCE 4_in

#define ROBOT_STOPPED_THRESHOLD 0.2_mps

#define FIELD_LENGTH 17.5482504_m
#define FIELD_WIDTH 8.0519016_m 

#define VIABLE_ELEVATOR_DISTANCE 1.2_m

Drivetrain::Drivetrain(frc::TimedRobot *_robot, valor::CANdleSensor* _leds) : 
    valor::Swerve<SwerveAzimuthMotor, SwerveDriveMotor>(
        _robot,
        "SwerveDrive",
        generateModules(),
        Constants::moduleDiff(),
        WHEEL_DIAMETER
    ),
    teleopStart(999999999999),
    leftDistanceSensor(_robot, "LEFT Front Distance Sensor", CANIDs::LEFT_CAN_RANGE_DRIVETRAIN_SENSOR, "", VIABLE_ELEVATOR_DISTANCE - 1_mm ),
    rightdistanceSensor(_robot, "RIGHT FRONT Distance Sensor", CANIDs::RIGHT_CAN_RANGE_DRIVETRAIN_SENSOR, "", VIABLE_ELEVATOR_DISTANCE - 1_mm),
    leds(_leds)
{
    xPIDF.P = KPX;
    xPIDF.I = KIX;
    xPIDF.D = KDX;

    thetaPIDF.P = KPT;
    thetaPIDF.I = KIT;
    thetaPIDF.D = KDT;

    // table->PutNumber("Rot_Pos_Tol", Swerve::rotPosTolerance.to<double>());
    // table->PutNumber("Rot_Vel_Tol", Swerve::rotVelTolerance.to<double>());

    table->PutNumber("Y_Pos_Tol", Swerve::yPosTolerance.to<double>());
    // table->PutNumber("Y_Vel_Tol", Swerve::yVelTolerance.to<double>());
    //
    table->PutNumber("Left Align Offset", AA_LEFT_OFFSET.value());
    table->PutNumber("Right Align Offset", AA_RIGHT_OFFSET.value());
    table->PutNumber("X SHUTOFF Y DISTANCE", X_SHUTOFF_Y_DISTANCE.value());

    leftDistanceSensor.setMaxDistance(2_m);
    rightdistanceSensor.setMaxDistance(2_m);

    table->PutNumber("LEFT ALIGN OFFSET (in)", LEFT_TO_ROBOT.value());
    table->PutNumber("RIGHT ALIGN OFFSET (in)", RIGHT_TO_ROBOT.value());
    
    valor::PIDF tempY;
    tempY.P = Y_ALIGN_KP;
    tempY.I = Y_ALIGN_KI;
    tempY.D = Y_ALIGN_KD;

    Swerve::setPIDy(tempY);

    valor::PIDF tempX;
    tempX.P = X_ALIGN_KP;
    tempX.I = X_ALIGN_KI;
    tempX.D = X_ALIGN_KD;

    Swerve::setPIDx(tempX);

    worldAlignpidx.P = X_WORLD_ALIGN_KP;
    worldAlignpidx.I = X_WORLD_ALIGN_KI;
    worldAlignpidx.D = X_WORLD_ALIGN_KD;

    worldAlignpidy.P = Y_WORLD_ALIGN_KP;
    worldAlignpidy.I = Y_WORLD_ALIGN_KI;
    worldAlignpidy.D = Y_WORLD_ALIGN_KD;

    table->PutNumber("Y_KP", Swerve::Y_KP);
    table->PutNumber("Y_KI", Swerve::Y_KI);
    table->PutNumber("Y_KD", Swerve::Y_KD);

    table->PutNumber("X_KP", Swerve::X_KP);
    table->PutNumber("X_KI", Swerve::X_KI);
    table->PutNumber("X_KD", Swerve::X_KD);

    table->PutNumber("Rot_KP", Swerve::ROT_KP);
    table->PutNumber("Rot_KD", Swerve::ROT_KD);

    table->PutNumber("Pole Offset", POLE_OFFSET.to<double>());

    table->PutNumber("Vision Std", 3.0);
    table->PutNumber("Vision Acceptance", VISION_ACCEPTANCE.to<double>() );
    table->PutNumber("KPLIMELIGHT", KP_LIMELIGHT);
    table->PutBoolean("Accepting Vision Measurements", true);
    table->PutBoolean("USE AUTO-ALIGN", true);

    table->PutBoolean("Align Right", false);
    table->PutBoolean("Align Left", false);
    table->PutBoolean("Use xController", true);
    table->PutNumber("Viable Dunk Distance (m)", VIABLE_DUNK_DISTANCE.value());

    setRotAlignOffset(00_deg);

    for (std::pair<const char*, frc::Pose3d> aprilCam : Constants::getAprilCameras()) {
        aprilTagSensors.push_back(new valor::AprilTagsSensor(robot, aprilCam.first, aprilCam.second));  
        aprilTagSensors.back()->setPipe(valor::VisionSensor::PIPELINE_0);
    }

    state.dir = NONE;

    setupGyro(
        CANIDs::PIGEON_CAN,
        PIGEON_CAN_BUS,
        Constants::pigeonMountRoll(),
        Constants::pigeonMountPitch(),
        Constants::pigeonMountYaw()
    );

    table->PutNumber("Viable Dunk speed", VIABLE_DUNK_SPEED.value());
    /*
     * 3.8m/s, 5m/s^2, ~125lbs Apr. 2
     */
    AutoBuilder::configure(
        [this](){ 
            if (state.useCalculatedEstimator) {
                return getCalculatedPose();
            }
            return getCalculatedPose();
            //return getRawPose();
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

    poseErrorPPTopic = nt::NetworkTableInstance::GetDefault().GetStructTopic<frc::Transform2d>("LiveWindow/BaseSubsystem/SwerveDrive/Pose Error PP").Publish();
    table->PutNumber("Y Controller Activation Degree Threshold", Y_ACTIVATION_THRESHOLD.value());
    reefPublisher = nt::NetworkTableInstance::GetDefault().GetStructTopic<frc::Pose2d>("Reef Pose").Publish();
    robotPredictedPublisher = nt::NetworkTableInstance::GetDefault().GetStructTopic<frc::Pose2d>("Robot Predicted Pose").Publish();
    doubtRot = 1;
    robotPublisher = nt::NetworkTableInstance::GetDefault().GetStructTopic<frc::Pose2d>("Robot Pose").Publish();
    robotInTagSpacePublisher = nt::NetworkTableInstance::GetDefault().GetStructTopic<frc::Transform2d>("Robot In Tag Space Transform").Publish();

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
    drivePID.D = Constants::driveKD();
    drivePID.kV = 0.90;
    drivePID.error = 0.0027_tr;

    for (size_t i = 0; i < 4; i++) {
        SwerveAzimuthMotor* azimuthMotor = new SwerveAzimuthMotor(
            Constants::getAzimuthMotorType(),
            CANIDs::AZIMUTH_CANS[i],
            valor::NeutralMode::Brake,
            Constants::swerveAzimuthsReversals()[i],
            Constants::azimuthGearRatio(),
            1.0,
            PIGEON_CAN_BUS
            
        );
        azimuthMotor->setPIDF(azimuthPID, 0);
        azimuthMotor->enableFOC();
        azimuthMotor->setupCANCoder(CANIDs::CANCODER_CANS[i], Constants::swerveZeros()[i], false, PIGEON_CAN_BUS);
        azimuthMotor->config.ClosedLoopGeneral.ContinuousWrap = true;
        azimuthMotor->applyConfig();

        SwerveAzimuthMotor* driveMotor = new SwerveDriveMotor(
            valor::PhoenixControllerType::KRAKEN_X60_FOC,
            CANIDs::DRIVE_CANS[i],
            valor::NeutralMode::Coast,
            Constants::swerveDrivesReversals()[i],
            1.0,
            Constants::driveGearRatio(),
            PIGEON_CAN_BUS
        );
        driveMotor->setPIDF(drivePID, 0);
        driveMotor->enableFOC();
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
    state.reefTag = {-1, frc::Pose2d()};
    currentPosePathPlanner = nt::NetworkTableInstance::GetDefault().GetStructTopic<frc::Pose2d>("/PathPlanner/currentPose").Subscribe(frc::Pose2d{});
    targetPosePathPlanner = nt::NetworkTableInstance::GetDefault().GetStructTopic<frc::Pose2d>("/PathPlanner/targetPose").Subscribe(frc::Pose2d{});
}

void Drivetrain::assessInputs()
{
    if (!driverGamepad || !driverGamepad->IsConnected() || !operatorGamepad || !operatorGamepad->IsConnected())
        return;

    if(driverGamepad->rightTriggerActive()){
        state.dir = RIGHT;
    } else if(driverGamepad->leftTriggerActive()){
        state.dir = LEFT;
    }

    state.alignToTarget = (driverGamepad->leftTriggerActive() || driverGamepad->rightTriggerActive()) && !state.intaking;

    state.climberAlign = driverGamepad->GetYButton();

    Swerve::assessInputs();
}

void Drivetrain::analyzeDashboard()
{
    for (size_t i = 0; i < swerveModules.size(); i++) {
        auto cancoder = swerveModules.at(i)->azimuthMotor->cancoder;
        leds->setLED(i, valor::CANdleSensor::cancoderMagnetHealthGetter(cancoder));
    }

    if(robot->IsDisabled()){
        state.right = table->GetBoolean("Align Right", false);
        state.left = table->GetBoolean("Align Left", false);
        if (state.right) {
            state.dir = RIGHT;
        } else if(state.left) {
            state.dir = LEFT;
        } else{
            state.dir = NONE;
        }
    }


    poseErrorPP = currentPosePathPlanner.Get() - targetPosePathPlanner.Get();

    poseErrorPPTopic.Set(poseErrorPP);

    frc::Pose2d predictedPose = getCalculatedPose().Exp(getRobotRelativeSpeeds().ToTwist2d(.25_s));
    robotPredictedPublisher.Set(predictedPose);

    frc::Pose2d robotToCenter{
        predictedPose.Translation() + frc::Translation2d(-FIELD_LENGTH / 2.0, -FIELD_WIDTH / 2.0),
        predictedPose.Rotation()
    };


    std::pair<int, frc::Pose3d> temp = valor::getNearestTag(
        robotToCenter,
        frc::DriverStation::kRed == frc::DriverStation::GetAlliance() ? valor::redReefAprilTagPoses : valor::blueReefAprilTagPoses
    );

    frc::Pose2d reefTagPose {
        temp.second.ToPose2d().Translation() + frc::Translation2d(FIELD_LENGTH / 2.0, FIELD_WIDTH / 2.0),
        temp.second.ToPose2d().Rotation()
    };

    if (!state.alignToTarget){
        state.reefTag = {
            temp.first,
            reefTagPose
        };
    }
    

    reefPublisher.Set(state.reefTag.second);

    alignAngleTags(); 
    Swerve::transformControllerSpeeds();

    choosePoleDirection(
        state.gamePiece == Constants::Scorer::ALGEE ? Direction::NONE : state.dir,
        state.reefTag.first
    );

    if (state.climberAlign){
        Swerve::targetAngle = frc::DriverStation::GetAlliance() == frc::DriverStation::kRed ? 90_deg : -90_deg;
        Swerve::yAlign = false;
        Swerve::xAlign = false;
        Swerve::rotAlign = true;
    } else if (
        state.alignToTarget &&
        state.elevState == Constants::Scorer::ELEVATOR_STATE::ONE &&
        state.gamePiece == Constants::Scorer::ALGEE
    ) {
        Swerve::targetAngle = frc::DriverStation::GetAlliance() == frc::DriverStation::kRed ? 90_deg : -90_deg;
        Swerve::yAlign = false;
        Swerve::xAlign = false;
        Swerve::rotAlign = true;
    } else if (
        state.alignToTarget &&
        state.elevState == Constants::Scorer::ELEVATOR_STATE::FOUR &&
        state.gamePiece == Constants::Scorer::GAME_PIECE::ALGEE
    ) {
        state.netAngle = frc::DriverStation::GetAlliance() == frc::DriverStation::kRed ? 180_deg : 0_deg;
        if (units::math::abs((frc::Rotation2d(state.netAngle) - (calcEstimator.get()->GetEstimatedPosition().Rotation())).Degrees()) < 60_deg) {
            Swerve::targetAngle = state.netAngle;
            Swerve::yAlign = false;
            Swerve::xAlign = false;
            Swerve::rotAlign = true;
        }
    } else if (state.alignToTarget) {
        Swerve::yAlign = hasYReset && state.elevState != Constants::Scorer::ELEVATOR_STATE::ONE;
        Swerve::rotAlign = true;
        Swerve::xAlign = state.worldAlign || (
            Swerve::yAlign &&
            units::math::fabs(Swerve::yDistance - Swerve::yGoalAlign).value() < table->GetNumber("X SHUTOFF Y DISTANCE", X_SHUTOFF_Y_DISTANCE.value()) &&
            state.gamePiece == Constants::Scorer::GAME_PIECE::CORAL &&
            state.elevState != Constants::Scorer::ELEVATOR_STATE::ONE
        );
    } else {
        Swerve::yAlign = false;
        Swerve::xAlign = false;
        Swerve::rotAlign = false;
    }

    if (!state.alignToTarget) {
        Swerve::resetRotationAlignControllers();
        hasYReset = false;
        hasXReset = false;
    }

    valor::AprilTagsSensor* detectedAprilCam = nullptr;
    for (valor::AprilTagsSensor* aprilLime : aprilTagSensors) {
        if (
            aprilLime->hasTarget() &&
            state.reefTag.first == aprilLime->getTagID()
        ) {
            detectedAprilCam = aprilLime;
        }
    }

    frc::Transform2d reefSpace = reefTransformConversion(state.reefTag.second);

    state.yEstimate += Swerve::yTagRelativeVelocity.value() * LOOP_TIME;
    if (detectedAprilCam != nullptr && reefSpace.X() < 1.5_m){
        if (state.worldAlign) {
            valor::PIDF tempX;
            tempX.P = X_ALIGN_KP;
            tempX.I = X_ALIGN_KI;
            tempX.D = X_ALIGN_KD;
            valor::PIDF tempY;
            tempY.P = Y_ALIGN_KP;
            tempY.I = Y_ALIGN_KI;
            tempY.D = Y_ALIGN_KD;
            Swerve::setPIDx(tempX);
            Swerve::setPIDy(tempY);

            hasXReset = false;
            hasYReset = false;
        }
        state.worldAlign = false;

        if (Swerve::yAlign && Swerve::xAlign && !hasXReset) {
            Swerve::xDistance = reefSpace.X();
            state.xEstimate = Swerve::xDistance.value(); 
            Swerve::resetXAlignControllers();
            hasXReset = true;
        }
        if (!hasYReset) {
            Swerve::yDistance = reefSpace.Y();
            state.yEstimate = Swerve::yDistance.value();
            Swerve::resetYAlignControllers();
            hasYReset = true;
        }
        state.yEstimate = Y_FILTER_CONST * state.yEstimate + ((1 - Y_FILTER_CONST) * detectedAprilCam->get_botpose_targetspace().X().to<double>());
        state.xEstimate = -detectedAprilCam->get_botpose_targetspace().Z().to<double>();

        Swerve::xDistance = units::length::meter_t (state.xEstimate);
        Swerve::yDistance = units::length::meter_t (state.yEstimate);

    } else {
        if (!state.worldAlign) {
            Swerve::setPIDx(worldAlignpidx);
            Swerve::setPIDy(worldAlignpidy);
        }
        state.worldAlign = true;


        Swerve::yDistance = reefSpace.Y();
        Swerve::xDistance = reefSpace.X();

        if (!hasXReset || !hasYReset){
            Swerve::resetRotationAlignControllers();
            Swerve::resetXAlignControllers();
            Swerve::resetYAlignControllers();
            hasXReset = true;
            hasYReset = true;
        }
    }

    Swerve::analyzeDashboard();

    for (valor::AprilTagsSensor* aprilLime : aprilTagSensors) {
        if (aprilLime->hasTarget() &&
            (
                (6 <= aprilLime->getTagID() && aprilLime->getTagID() <= 11) ||
                (17 <= aprilLime->getTagID() && aprilLime->getTagID() <= 22)
            )
        ) {
            aprilLime->applyVisionMeasurement(
                calcEstimator.get(),
                getRobotSpeeds(),
                pigeon->GetAngularVelocityZWorld().GetValue(),
                doubtX,
                doubtY,
                doubtRot
            );

        }
        

        // aprilLime->applyVisionMeasurement(
        //     alignEstimator.get(),
        //     true,
        //     Constants::cameraStandardDeviationLBF(aprilLime->get_botpose_targetspace().Y()),
        //     Constants::cameraStandardDeviationLBF(aprilLime->get_botpose_targetspace().Y())
        // );
    }

    if (!driverGamepad || !driverGamepad->IsConnected() || !operatorGamepad || !operatorGamepad->IsConnected())
        return;

    if (frc::Timer::GetFPGATimestamp() - teleopStart > TELEOP_MAX_TIME - TIME_TO_RUMBLE && frc::Timer::GetFPGATimestamp() - teleopStart < TELEOP_MAX_TIME - (TIME_TO_RUMBLE - 3_s)) {
        operatorGamepad->setRumble(true, .4);
    } else {
        operatorGamepad->setRumble(false);
    }
}

void Drivetrain::assignOutputs()
{
    Swerve::assignOutputs();

    state.accel = {
        pigeon->GetAccelerationX().GetValue(),
        pigeon->GetAccelerationY().GetValue(),
        pigeon->GetAccelerationZ().GetValue()
    };
}

frc::Transform2d Drivetrain::reefTransformConversion(frc::Pose2d reefTagPose) {
            frc::Transform2d reefTagTransform{
                reefTagPose.Translation(),
                reefTagPose.Rotation()
            };

            frc::Transform2d robotTransform{
                getCalculatedPose().Translation(),
                getCalculatedPose().Rotation()
            };

            frc::Transform2d robotInTagSpaceTransform = reefTagTransform.Inverse() + robotTransform;

            robotInTagSpacePublisher.Set(robotInTagSpaceTransform);

            return robotInTagSpaceTransform;
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

std::bitset<5> Drivetrain::getAutoDunkAcceptance() {
    std::bitset<5> acceptance;

    acceptance[0] = withinYRange();
    acceptance[1] = withinXRange(
        frc::DriverStation::IsAutonomous() ? .34_m : (units::meter_t) table->GetNumber("Viable Dunk Distance (m)", VIABLE_DUNK_DISTANCE.value())
    );
    acceptance[2] = units::math::fabs(yTagRelativeVelocity) < Y_CONTROLLER_SPEED_LIMIT;
    acceptance[3] = units::math::fabs(getYawVelocity()) < ROT_CONTROLLER_SPEED_LIMIT;

    units::meters_per_second_t autoDunkSpeedThreshold = Constants::Scorer::getAutoDunkSpeedLimitation(xAlign);
    acceptance[4] = isSpeedBelowThreshold(frc::DriverStation::IsAutonomous() ? AUTO_VIABLE_DUNK_SPEED_L4 : autoDunkSpeedThreshold);
    return acceptance;
}

bool Drivetrain::autoAlignShutOff() {
    return getAutoDunkAcceptance().all();
}

frc2::CommandPtr Drivetrain::pitSequenceCommand(const frc::ChassisSpeeds& speeds, int offset, int numLeds) {
    auto targetStates = getModuleStates(speeds);
    return frc2::cmd::Sequence(
        frc2::cmd::Deadline(
            frc2::cmd::Wait(3_s),
            frc2::cmd::RunOnce([this, targetStates] {
                testModeDesiredStates = targetStates;
            })
        ),
        frc2::cmd::RunOnce([this, offset, numLeds] {
            for (int i = 0; i < MODULE_COUNT; i++)
                testModeDesiredStates[i].speed = 0_mps;

            for (int i = 0; i < numLeds; i++) {
                leds->setLED(8 + offset + i, valor::CANdleSensor::GREEN);
                leds->setLED(LEDConstants::LED_COUNT - offset - i, valor::CANdleSensor::GREEN);
            }
        }),
        frc2::cmd::Wait(1_s)
    );
}

frc2::CommandPtr Drivetrain::pitSequence() {
    return frc2::cmd::Sequence(
        pitSequenceCommand(frc::ChassisSpeeds{0_mps, maxDriveSpeed / 3.0, 0_rad_per_s}, 0, 2),
        pitSequenceCommand(frc::ChassisSpeeds(0_mps, 0_mps, maxRotationSpeed / 3.0), 2, 2),
        pitSequenceCommand(frc::ChassisSpeeds{maxDriveSpeed / 3.0, 0_mps, 0_rad_per_s}, 4, 2),
        pitSequenceCommand(frc::ChassisSpeeds{-maxDriveSpeed / 3.0, 0_mps, 0_rad_per_s}, 6, 2),
        frc2::cmd::RunOnce([this] {
            for (int i = 0; i < MODULE_COUNT; i++)
                testModeDesiredStates[i].angle = 0_deg;
        })
    );
}

void Drivetrain::alignAngleTags()
{
    // set each id angle to the correct angle
    // aprilTagSensors[4]->get_botpose_targetspace();
    setAngleBasedOnTag(state.reefTag.first);
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

bool Drivetrain::withinXRange(units::meter_t distance) {
    auto measuredDistance = lidarDistance();
    return (measuredDistance < distance);
}

units::meter_t Drivetrain::lidarDistance() {
    bool leftConnected = leftDistanceSensor.isConnected();
    bool rightConnected = rightdistanceSensor.isConnected();

    if (!rightConnected && !leftConnected){
        return leftDistanceSensor.getDefaultDistance();
    }

    if (state.dir == LEFT) {
        if (!rightConnected) {
            return leftDistanceSensor.getFilteredDistance();
        }
        return rightdistanceSensor.getFilteredDistance();
    } else {
        if (!leftConnected) {
            return rightdistanceSensor.getFilteredDistance();
        }
        return leftDistanceSensor.getFilteredDistance();
    }
}

bool Drivetrain::withinYRange() {
    return yControllerAligned();
}

units::degrees_per_second_t Drivetrain::getYawVelocity() {
    return pigeon->GetAngularVelocityZWorld().GetValue();
}

bool Drivetrain::isSpeedBelowThreshold(units::meters_per_second_t m) {
    frc::ChassisSpeeds speeds = getRobotRelativeSpeeds();
    
    units::meters_per_second_t totalSpeed = units::math::hypot(
        speeds.vx, 
        speeds.vy
    );
    return (totalSpeed < m);
    // (units::meters_per_second_t) table->GetNumber("Viable Dunk speed", VIABLE_DUNK_SPEED.value())
}

bool Drivetrain::isSpeedStopped() {
    frc::ChassisSpeeds speeds = getRobotRelativeSpeeds();
    
    units::meters_per_second_t totalSpeed = units::math::hypot(
        speeds.vx, 
        speeds.vy
    );
    return (totalSpeed < ROBOT_STOPPED_THRESHOLD);
}

#define SCORER_OFFSET 2_in

void Drivetrain::choosePoleDirection(Direction dir, Constants::AprilTag tag){
    
    std::unordered_map<Constants::AprilTag, Constants::DirectionalOffSet> poleOffset = frc::DriverStation::GetAlliance() == frc::DriverStation::kRed ? Constants::redPoleOffsets : Constants::bluePoleOffsets;
    units::inch_t offset = poleOffset.find(tag) != poleOffset.end() ? poleOffset.at(tag).at(dir) : 0.0_in;
    switch (dir) {
        case LEFT:
            Swerve::yGoalAlign = -units::math::abs(POLE_OFFSET - offset);
            break;
        case RIGHT:
            Swerve::yGoalAlign = units::math::abs(POLE_OFFSET + offset);
            break;
        default:
            Swerve::yGoalAlign = poleOffset.find(tag) != poleOffset.end() ? poleOffset.at(tag).at(NONE) : 0_in;
            break;
    }

}

void Drivetrain::setGamePieceInRobot(Constants::Scorer::GAME_PIECE piece){
    state.gamePiece = piece;
}

void Drivetrain::setTeleopStartTime(){
    teleopStart = frc::Timer::GetFPGATimestamp();
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
                acceleration.push_back(state.accel.x.value());
                acceleration.push_back(state.accel.y.value());
                acceleration.push_back(state.accel.z.value());
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
            [this] {return state.reefTag.first;},
            nullptr
        );

        builder.AddDoubleProperty(
            "Direction",
            [this] {return state.dir;},
            nullptr
        );
        
        builder.AddDoubleProperty(
            "Lidar Distance",
            [this] {return lidarDistance().value();},
            nullptr
        );
        builder.AddBooleanProperty(
            "Within Y Range",
            [this] {return withinYRange();},
            nullptr
        );
        builder.AddDoubleProperty(
            "left lidar distance",
            [this] {return leftDistanceSensor.getLidarData().value();},
            nullptr
        );
          builder.AddDoubleProperty(
            "right lidar distance",
            [this] {return rightdistanceSensor.getLidarData().value();},
            nullptr
        );

        builder.AddBooleanProperty(
            "State: Align To Target",
            [this] {return state.alignToTarget;},
            nullptr
        );
        builder.AddBooleanProperty(
            "State: World Align",
            [this] {return state.worldAlign;},
            nullptr
        );
    }

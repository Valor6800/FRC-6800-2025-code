#define _USE_MATH_DEFINES
#include "Drivetrain.h"
#include "Constants.h"
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
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
    valor::Swerve<SwerveAzimuthMotor, SwerveDriveMotor>(
        _robot,
        "SwerveDrive"
    ),
    teleopStart(999999999999)
    // lidarSensor(_robot, "Front Lidar Sensor", CANIDs::FRONT_LIDAR_SENSOR)
{
    xPIDF.P = KPX;
    xPIDF.I = KIX;
    xPIDF.D = KDX;

    thetaPIDF.P = KPT;
    thetaPIDF.I = KIT;
    thetaPIDF.D = KDT;

    // table->PutNumber("Vision Std", 3.0);
    // table->PutNumber("Vision Acceptance", VISION_ACCEPTANCE.to<double>() );
    // table->PutNumber("KPLIMELIGHT", KP_LIMELIGHT);
    // table->PutBoolean("Accepting Vision Measurements", true);

    // for (std::pair<const char*, frc::Pose3d> aprilCam : Constants::aprilCameras) {
    //     aprilTagSensors.push_back(new valor::AprilTagsSensor(robot, aprilCam.first, aprilCam.second));  
    //     aprilTagSensors.back()->setPipe(valor::VisionSensor::PIPELINE_0);
    // }

    // aprilTagSensors[4]->setPipe(valor::VisionSensor::PIPELINE_1);
    // aprilTagSensors[4]->setCameraPose(Constants::aprilCameras[4].second);

    // setupGyro(
    //     CANIDs::PIGEON_CAN,
    //     PIGEON_CAN_BUS,
    //     Constants::pigeonMountRoll(),
    //     Constants::pigeonMountPitch(),
    //     Constants::pigeonMountYaw()
    // );
    
    // /*
    //  * 3.8m/s, 5m/s^2, ~125lbs Apr. 2
    //  */
    // AutoBuilder::configure(
    //     [this](){ 
    //         wpi::println("Returning current pose...");
    //         return drivetrain.GetState().Pose;
    //     },
    //     [this](frc::Pose2d pose){
    //         wpi::println("Resetting current pose...");
    //         drivetrain.ResetPose(pose);
    //     }, // Method to reset odometry (will be called if your auto has a starting pose)
    //     [this](){
    //         wpi::println("Getting current ChassisSpeeds");
    //         return drivetrain.GetState().Speeds;
    //     }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
    //     [this](frc::ChassisSpeeds speeds) {
    //         wpi::println("Setting ChassisSpeeds");
    //         drivetrain.SetControl(autoRobotSpeedsRequest.WithSpeeds(speeds));
    //     }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
    //     std::make_shared<PPHolonomicDriveController>(
    //         PIDConstants(xPIDF.P, xPIDF.I, xPIDF.D), // Translation PID constants
    //         PIDConstants(thetaPIDF.P, thetaPIDF.I, thetaPIDF.D) // Rotation PID constants
    //     ),
    //     RobotConfig::fromGUISettings(),
    //     []() {
    //         // Boolean supplier that controls when the path will be mirrored for the red alliance
    //         // This will flip the path being followed to the red side of the field.
    //         // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

    //         auto alliance = frc::DriverStation::GetAlliance();
    //         if (alliance) {
    //             return alliance.value() == frc::DriverStation::Alliance::kRed;
    //         }
    //         return false;
    //     },
    //     this // Reference to this subsystem to set requirements
    // );
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

    // resetEncoders();
    // resetOdometry(frc::Pose2d{0_m, 0_m, 0_rad});
}

void Drivetrain::init()
{
    // Swerve::init();
}

void Drivetrain::assessInputs()
{
    Swerve::assessInputs();

    // if (!driverGamepad || !driverGamepad->IsConnected() || !operatorGamepad || !operatorGamepad->IsConnected())
    //     return;
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

    return std::move(currents);
}

void Drivetrain::assignOutputs()
{
    wpi::println("Drivetrain::assignOutputs()");
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

        // builder.AddDoubleArrayProperty(
        //     "Acceleration",
        //     [this] {
        //         std::vector<double> acceleration;
        //         acceleration.push_back(state.accel.x.to<double>());
        //         acceleration.push_back(state.accel.y.to<double>());
        //         acceleration.push_back(state.accel.z.to<double>());
        //         return acceleration;
        //     },
        //     nullptr
        // );
        // builder.AddBooleanProperty(
        //     "Bonk!",
        //     [this] {
        //         return units::acceleration::meters_per_second_squared_t{
        //         sqrtf(powf(state.accel.x.to<double>(), 2) + powf(state.accel.y.to<double>(), 2))
        //     } > 20.0_mps_sq; // ~60 kg bot -> 600 N, 5 measurements * 20ms = .1s, 
        //                                                                              // impulse = .1 * 600 = 60 Joules
        //     },
        //     nullptr
        // );
    }

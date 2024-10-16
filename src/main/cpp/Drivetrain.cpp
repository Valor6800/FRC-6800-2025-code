#include "Drivetrain.h"
#include <cmath>
#include <frc/DriverStation.h>
#include <iostream>
#include <math.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/util/HolonomicPathFollowerConfig.h>
#include <pathplanner/lib/util/PIDConstants.h>
#include <pathplanner/lib/util/ReplanningConfig.h>
#include <string>
#include "Constants.h"
#include "frc2/command/FunctionalCommand.h"
#include "frc2/command/SequentialCommandGroup.h"
#include "units/acceleration.h"
#include "units/length.h"
#include "units/velocity.h"
#include "valkyrie/sensors/AprilTagsSensor.h"
#include "units/length.h"
#include "valkyrie/sensors/VisionSensor.h"
#include <frc2/command/InstantCommand.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <utility>
#include "frc/geometry/Pose3d.h"
#include "frc/geometry/Rotation3d.h"
#include "units/angle.h"
#include <frc/sysid/SysIdRoutineLog.h>

using namespace pathplanner;

#define SPEAKER_Y 5.543042_m
#define SPEAKER_BLUE_X 0.0_m
#define SPEAKER_RED_X 16.4846_m

#define AMP_Y 8.1026_m
#define AMP_BLUE_X 1.637157_m
#define AMP_RED_X 14.899717_m
#define AMP_Y_OFFSET 0.0_m
#define AMP_X_OFFSET 0.0_m

#define TXRANGE  30.0f
#define KPIGEON 2.0f
#define KLIMELIGHT -29.8f
// #define KP_LOCK 0.2f
#define KP_LIMELIGHT 0.7f

#define KPX 7.0f // 30
#define KIX 0.0f //0
#define KDX 0.1f //.1

#define KPT 8.0f //15
#define KIT 0.0f
#define KDT 0.0f

#define WHEEL_DIAMETER_M 0.0973f //0.1016
#define DRIVE_GEAR_RATIO 5.51f
#define AZIMUTH_GEAR_RATIO 13.37f
#define ROT_SPEED_MUL 1.3f
#define SLIP_FACTOR 1.25f

#define AUTO_VISION_THRESHOLD 4.0f //meters

#define X_TIME 214.85f

#define MODULE_DIFF_XS {1, 1, -1, -1}
#define MODULE_DIFF_YS {1, -1, -1, 1}

#define DRIVETRAIN_CAN_BUS "" //rev
#define PIGEON_CAN_BUS "baseCAN" //phoenix

#define KP_ROTATE -1.4f
#define KD_ROTATE -30.0f
#define SPEAKER_X_OFFSET 0.00f
#define SPEAKER_Y_OFFSET 0.00f

#define VISION_ACCEPTANCE 3.5_m // meters

#define BLUE_AMP_ROT_ANGLE -90.0_deg
#define BLUE_SOURCE_ROT_ANGLE -30_deg
#define RED_SOURCE_ROT_ANGLE -150_deg
#define BLUE_LOCK_ANGLE 0_deg
#define RED_LOCK_ANGLE 180_deg

#define TIME_TELEOP_VERT 105.0f

#define MT2_POSE true

Drivetrain::Drivetrain(frc::TimedRobot *_robot, valor::CANdleSensor *_leds) : valor::BaseSubsystem(_robot, "Drivetrain"),
                        rotMaxSpeed(ROT_SPEED_MUL * 2 * M_PI),
                        pigeon(CANIDs::PIGEON_CAN, PIGEON_CAN_BUS),
                        motorLocations(wpi::empty_array),
                        kinematics(NULL),
                        estimator(NULL),
                        calculatedEstimator(NULL),
                        teleopStart(999999999999),
                        unfilteredPoseTracker(5),
                        swerveNoError(true),
                        leds(_leds),
                        sysid(frc2::sysid::Config{std::nullopt,std::nullopt,std::nullopt, std::nullopt},
                            frc2::sysid::Mechanism{
                                [this](units::volt_t driveVoltage){
                                    setSysIdVoltage(driveVoltage);
                                },
                                [this](frc::sysid::SysIdRoutineLog* log){
                                    logSysId(log);
                                },
                                this
                            }
                        ),
                        sysIdRunner(dynamicSysid(frc2::sysid::Direction::kForward))
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
    std::vector<std::string> pitSequenceNames = {
        "Swerves translation test", 
        "Swerves rotation test",
        "Straighten wheels",
        "Enable intake",
        "Disable intake",
        "Shoot amp",
        "Disable shoot",
        "Set pivot subwoofer",
        "Set pivot podium",
        "Set pivot wing"
    }; // WARNING: Jank and bad. 
       // Should either be automatically loaded from the auto or the pit sequence should be a list of commands.
       // Also shouldn't be stored here
    pathplanner::NamedCommands::registerCommand("Wait for A button", std::move(
        frc2::FunctionalCommand(
            [this, pitSequenceNames](){
                size_t i = state.pitSequenceCommandIndex;
                if (i <= pitSequenceNames.size() - 1)
                    table->PutString("Next command", pitSequenceNames[i]);
                state.pitSequenceCommandIndex += 1;
            }, 
            [](){}, 
            [this, pitSequenceNames](bool _b){
                size_t i = state.pitSequenceCommandIndex;
                if (i <= pitSequenceNames.size() - 1)
                    table->PutString("Current command", pitSequenceNames[i]);
                if (i < pitSequenceNames.size() - 1)
                    table->PutString("Next command", pitSequenceNames[i + 1]);
                state.pitSequenceCommandIndex += 1;
            }, 
            [this](){ return operatorGamepad->GetAButtonPressed(); }, 
            {this}
        )
    ).ToPtr());
}

Drivetrain::~Drivetrain()
{
    for (int i = 0; i < SWERVE_COUNT; i++)
    {
        delete azimuthControllers[i];
        delete driveControllers[i];
        delete swerveModules[i];
    }

    delete estimator;
    delete calculatedEstimator;
}

void Drivetrain::setSysIdVoltage(units::volt_t voltage){
    for(size_t i = 0; i < driveControllers.size(); i++){
        driveControllers[i]->setVoltage(voltage);
    }
}

void Drivetrain::logSysId(frc::sysid::SysIdRoutineLog* log){
    if(log == nullptr) return;
    for(size_t i = 0; i < driveControllers.size(); i++){
        log->Motor("module" + std::to_string(i))
        .voltage(driveControllers[i]->getVoltage())
        .position(units::meter_t{driveControllers[i]->getPosition()})
        .velocity(units::meters_per_second_t{driveControllers[i]->getSpeed()});
    }
}

frc2::CommandPtr Drivetrain::quaistaticSysid(frc2::sysid::Direction direction){
    return frc2::cmd::Sequence(
        frc2::InstantCommand([this]() {setSysIdVoltage(0_V);}).ToPtr(),
        frc2::cmd::Wait(0.5_s),
        sysid.Quasistatic(direction)
    );
}

frc2::CommandPtr Drivetrain::dynamicSysid(frc2::sysid::Direction direction){
    return frc2::cmd::Sequence(
        frc2::InstantCommand([this]() {setSysIdVoltage(0_V);}).ToPtr(),
        frc2::cmd::Wait(0.5_s),
        sysid.Dynamic(direction)
    );
}

void Drivetrain::configSwerveModule(int i)
{

   int MDX[] = MODULE_DIFF_XS;
   int MDY[] = MODULE_DIFF_YS;

    motorLocations[i] = frc::Translation2d{Constants::moduleDiff() * MDX[i],
                                           Constants::moduleDiff() * MDY[i]};

    valor::PIDF azimuthPID;
    azimuthPID.maxVelocity = Constants::azimuthKVel();
    azimuthPID.maxAcceleration = Constants::azimuthKAcc();
    azimuthPID.P = Constants::azimuthKP();
    azimuthPID.error = 0.0027;

    azimuthControllers.push_back(new SwerveAzimuthMotor(CANIDs::AZIMUTH_CANS[i],
                                                      valor::NeutralMode::Brake,
                                                      Constants::swerveAzimuthsReversals()[i],
                                                      AZIMUTH_GEAR_RATIO,
                                                      1.0,
                                                      azimuthPID,
                                                      8.0,
                                                      false,
                                                      PIGEON_CAN_BUS));
    
    azimuthControllers[i]->enableFOC(true);
    azimuthControllers[i]->setupCANCoder(CANIDs::CANCODER_CANS[i], Constants::swerveZeros()[i], false, PIGEON_CAN_BUS);

    valor::PIDF drivePID;
    drivePID.maxVelocity = Constants::driveKVel();
    drivePID.maxAcceleration = Constants::driveKAcc();
    drivePID.P = Constants::driveKP();
    drivePID.error = 0.0027;

    double conversion = DRIVE_GEAR_RATIO / (M_PI * WHEEL_DIAMETER_M);
    driveControllers.push_back(new SwerveDriveMotor(CANIDs::DRIVE_CANS[i],
                                                    valor::NeutralMode::Coast,
                                                    Constants::swerveDrivesReversals()[i],
                                                    1.0,
                                                    conversion,
                                                    drivePID,
                                                    12.0,
                                                    true,
                                                    PIGEON_CAN_BUS));
    driveControllers[i]->enableFOC(true);
    driveControllers[i]->setOpenLoopRamp(1.0);

    driveMaxSpeed = driveControllers[i]->getMaxMotorSpeed() / 60.0 / conversion;

    swerveModules.push_back(new valor::Swerve<SwerveAzimuthMotor, SwerveDriveMotor>(azimuthControllers[i], driveControllers[i], motorLocations[i]));
    swerveModules[i]->setMaxSpeed(driveMaxSpeed);

}

void Drivetrain::resetState()
{
    resetDriveEncoders();
    resetOdometry(frc::Pose2d{0_m, 0_m, 0_rad});
    state.manualFlag = false;
    state.pitMode = false;
}

void Drivetrain::init()
{

    for (std::pair<const char*, frc::Pose3d> aprilCam : Constants::aprilCameras) {
        aprilTagSensors.push_back(new valor::AprilTagsSensor(robot, aprilCam.first, aprilCam.second));
    }

    aprilTagSensors[0]->normalVisionOutlier = 6.0_m;

    for (valor::AprilTagsSensor* aprilLime : aprilTagSensors) {
        aprilLime->setPipe(valor::VisionSensor::PIPELINE_0);
    }

    for (int i = 0; i < SWERVE_COUNT; i++)
    {
        configSwerveModule(i);
    }

    pigeon.GetConfigurator().Apply(
        ctre::phoenix6::configs::Pigeon2Configuration{}
        .WithMountPose(
            ctre::phoenix6::configs::MountPoseConfigs{}
            .WithMountPosePitch(Constants::pigeonMountPitch().to<double>())
            .WithMountPoseRoll(Constants::pigeonMountRoll().to<double>())
            .WithMountPoseYaw(Constants::pigeonMountYaw().to<double>())
        )
    );

    kinematics = new frc::SwerveDriveKinematics<SWERVE_COUNT>(motorLocations);
    estimator = new frc::SwerveDrivePoseEstimator<SWERVE_COUNT>(*kinematics, pigeon.GetRotation2d(), getModuleStates(), frc::Pose2d{0_m, 0_m, 0_rad});
    calculatedEstimator = new frc::SwerveDrivePoseEstimator<SWERVE_COUNT>(*kinematics, pigeon.GetRotation2d(), getModuleStates(), frc::Pose2d{0_m, 0_m, 0_rad});

    xPIDF.P = KPX;
    xPIDF.I = KIX;
    xPIDF.D = KDX;

    thetaPIDF.P = KPT;
    thetaPIDF.I = KIT;
    thetaPIDF.D = KDT;

    state.lock = false;
    state.pitMode = false;

    table->PutNumber("Vision Std", 3.0);

    table->PutNumber("Vision Acceptance", VISION_ACCEPTANCE.to<double>() );

    table->PutNumber("KPLIMELIGHT", KP_LIMELIGHT);
    table->PutNumber("KP_ROTATION", KP_ROTATE);
    table->PutNumber("KD_ROTATION", KD_ROTATE);
    table->PutNumber("SPEAKER_X_OFFSET", SPEAKER_X_OFFSET);
    table->PutNumber("SPEAKER_Y_OFFSET", SPEAKER_Y_OFFSET);
    table->PutNumber("AMP_X_OFFSET", AMP_X_OFFSET.to<double>());
    table->PutNumber("AMP_Y_OFFSET", AMP_Y_OFFSET.to<double>());

    table->PutBoolean("Accepting Vision Measurements", true);
    table->PutBoolean("Pit Mode", false);

    table->PutBoolean("Run SysId Quasistatic Forward", false);
    table->PutBoolean("Run SysId Quasistatic Backward", false);
    table->PutBoolean("Run SysId Dynamic Forward", false);
    table->PutBoolean("Run SysId Dynamic Backward", false);


    resetState();

    state.useCalculatedEstimator = true;

    /*
     * 3.8m/s, 5m/s^2, ~125lbs Apr. 2
     */
    AutoBuilder::configureHolonomic(
        [this](){ 
            if (state.useCalculatedEstimator) {
                // estimator->ResetPosition(getPigeon(), getModuleStates(), {
                //     calculatedEstimator->GetEstimatedPosition().X(),
                //     calculatedEstimator->GetEstimatedPosition().Y(),
                //     estimator->GetEstimatedPosition().Rotation()
                // });
                return calculatedEstimator->GetEstimatedPosition();
            }
            return getPose_m();
        }, // Robot pose supplier
        [this](frc::Pose2d pose){ resetOdometry(pose); }, // Method to reset odometry (will be called if your auto has a starting pose)
        [this](){ return getRobotRelativeSpeeds(); table->PutNumber("timestamp of retrieving speeds", frc::Timer::GetFPGATimestamp().to<double>()); }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        [this](frc::ChassisSpeeds speeds){ driveRobotRelative(speeds); }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            PIDConstants(xPIDF.P, xPIDF.I, xPIDF.D), // Translation PID constants
            PIDConstants(thetaPIDF.P, thetaPIDF.I, thetaPIDF.D), // Rotation PID constants
            units::meters_per_second_t{driveMaxSpeed}, // Max module speed, in m/s
            Constants::driveBaseRadius(), // Drive base radius in meters. Distance from robot center to furthest module.
            ReplanningConfig(true, false, .1_m, .25_m) // Default path replanning config. See the API for the options here
        ),
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

    pathplanner::NamedCommands::registerCommand("Set Camera Estimator", std::move(
        frc2::SequentialCommandGroup(
            frc2::InstantCommand(
                [this]() {
                    state.useCalculatedEstimator = true;
                }
            )
        )
    ).ToPtr());

    pathplanner::NamedCommands::registerCommand("Set Estimator", std::move(
        frc2::SequentialCommandGroup(
            frc2::InstantCommand(
                [this]() {
                    state.useCalculatedEstimator = false;
                }
            )
        )
    ).ToPtr());

    pathplanner::NamedCommands::registerCommand("Reset gyro", std::move(
        frc2::InstantCommand(
            [this]() {
                resetGyro();
            }
        )
    ).ToPtr());
}

std::vector<valor::Swerve<Drivetrain::SwerveAzimuthMotor, Drivetrain::SwerveDriveMotor> *> Drivetrain::getSwerveModules()
{
    return swerveModules;
}

double Drivetrain::angleWrap(double degrees)
{
    double zeroTo360 = degrees + 180;
    double start = fmod(zeroTo360, 360); //will work for positive angles

    //angle is (-360, 0), add 360 to make (0, 360)
    if (start < 0)
    {
        start += 360;
    }

    //bring it back to (-180, 180)
    return start - 180;
}

void Drivetrain::assessInputs()
{
    if (!driverGamepad || !driverGamepad->IsConnected())
        return;

    if (driverGamepad->GetBackButtonPressed()) {
        resetGyro();
    }
    if (driverGamepad->DPadDown()){ // All 4 back buttons are mapped to this as of 3/8/2024
        state.manualFlag = true; 
    } else {
        state.manualFlag = false;
    }
    
    state.backshot = driverGamepad->DPadRight();

    state.ampAlign = driverGamepad->GetBButton();
    state.isHeadingTrack = (driverGamepad->leftTriggerActive() && !driverGamepad->GetAButton());
    state.ampHeadingTrack = driverGamepad->GetXButton();
    state.sourceAlign = driverGamepad->GetYButton();
    state.thetaLock = driverGamepad->GetAButton();

    state.xSpeed = driverGamepad->leftStickY(2);
    state.ySpeed = driverGamepad->leftStickX(2);

    state.rot = driverGamepad->rightStickX(3);
}

void Drivetrain::calculateCarpetPose()
{
    calculatedEstimator->UpdateWithTime(frc::Timer::GetFPGATimestamp(),
        getPigeon(),
        getModuleStates()
    );
    frc::Pose2d newPose = calculatedEstimator->GetEstimatedPosition();
    units::meter_t deltaX = newPose.X() - previousPose.X();
    double factor = 1.0;
    if (Constants::roughTowardsRedAllianceWall) {
        factor = deltaX < units::meter_t{0} ? Constants::carpetGrainMultipler : 1;
    } else {
        factor = deltaX > units::meter_t{0} ? Constants::carpetGrainMultipler : 1;
    }
    
    calculatedEstimator->AddVisionMeasurement(
        frc::Pose2d{
            factor * deltaX + previousPose.X(),
            newPose.Y(),
            newPose.Rotation()
        },
        frc::Timer::GetFPGATimestamp(),
        {0.1, 0.1, 0.1}
    );
    previousPose = calculatedEstimator->GetEstimatedPosition();
}

void Drivetrain::analyzeDashboard()
{
    if(table->GetBoolean("Run SysId Quasistatic Forward", false)){
        if(!sysIdRunner.IsScheduled()){
            sysIdRunner = quaistaticSysid(frc2::sysid::Direction::kForward);
            sysIdRunner.Schedule();
        }
    }
    else if(table->GetBoolean("Run SysId Quasistatic Backward", false)){
        if(!sysIdRunner.IsScheduled()){
            sysIdRunner = quaistaticSysid(frc2::sysid::Direction::kReverse);
            sysIdRunner.Schedule();
        }
    }
    else if(table->GetBoolean("Run SysId Dynamic Forward", false)){
        if(!sysIdRunner.IsScheduled()){
            sysIdRunner = dynamicSysid(frc2::sysid::Direction::kForward);
            sysIdRunner.Schedule();
        }
    }
    else if(table->GetBoolean("Run SysId Dynamic Backward", false)){
        if(!sysIdRunner.IsScheduled()){
            sysIdRunner = dynamicSysid(frc2::sysid::Direction::kReverse);
            sysIdRunner.Schedule();
        }
    }
    else if(sysIdRunner.IsScheduled()){
        sysIdRunner.Cancel();
    }

    table->PutBoolean("Calculated estimator?", state.useCalculatedEstimator);
    state.pitMode = table->GetBoolean("Pit Mode", false);

    if (state.pitMode){
        state.ampAlign = false;
        state.isHeadingTrack = false;
        state.trapAlign = false;
        state.sourceAlign = false;
        state.thetaLock = false;    
    }

    if(state.isHeadingTrack) getSpeakerLockAngleRPS();
    else if(state.ampHeadingTrack) getAmpLockAngle();
    else if(state.sourceAlign) setAlignmentAngle(Alignment::SOURCE);
    else if(state.ampAlign) setAlignmentAngle(Alignment::AMP);
    else if(state.trapAlign) setAlignmentAngle(Alignment::TRAP);
    else if(state.thetaLock) setAlignmentAngle(Alignment::LOCK);

    estimator->UpdateWithTime(frc::Timer::GetFPGATimestamp(),
        getPigeon(),
        getModuleStates()
    );
    calculateCarpetPose();

    frc::Pose2d botpose;
    
    visionAcceptanceRadius = (units::meter_t) table->GetNumber("Vision Acceptance", VISION_ACCEPTANCE.to<double>());

    for (valor::AprilTagsSensor* aprilLime : aprilTagSensors) {
            aprilLime->applyVisionMeasurement(
                calculatedEstimator,
                getRobotSpeeds(),
                table->GetBoolean("Accepting Vision Measurements", true),
                doubtX,
                doubtY
            );
    }

    state.distanceFromSpeaker = getDistanceFromSpeaker();
    state.distanceFromAmp = getDistanceFromAmp();

    auto ppTable = nt::NetworkTableInstance::GetDefault().GetTable("PathPlanner");
    
    std::vector<double> bp = ppTable->GetNumberArray("currentPose", std::array<double, 3>{0, 0, 0});
    std::vector<double> tp = ppTable->GetNumberArray("targetPose", std::array<double, 3>{0, 0, 0});

    currentPoseTracker.addReading(frc::Pose2d{
        units::meter_t{bp[0]},
        units::meter_t{bp[1]},
        units::radian_t{bp[2]}
    }, frc::Timer::GetFPGATimestamp());
    targetPoseTracker.addReading(frc::Pose2d{
        units::meter_t{tp[0]},
        units::meter_t{tp[1]},
        units::radian_t{tp[2]}
    }, frc::Timer::GetFPGATimestamp());
    unfilteredPoseTracker.addReading(getPose_m(), frc::Timer::GetFPGATimestamp());

    for (int i = 0; i < 4; i++) {
        int color = 0xAC41FF;
        switch (azimuthControllers[i]->getMagnetHealth().value) {
            case 1: 
                color = 0xFF0000;
                break;
            case 2:
                color = 0xFF8C00;
                break;
            case 3:
                color = 0x00FF00;
                break;
        }
        leds->setLED(i, color);
    }
    for (int i = 5; i < 8; i++)
        leds->setLED(i, 0x000000);
}

void Drivetrain::assignOutputs()
{
    double kPRot = table->GetNumber("KP_ROTATION", KP_ROTATE);
    double kDRot = table->GetNumber("KD_ROTATION", KD_ROTATE);
    double error = getAngleError().to<double>();
    state.angleRPS = units::angular_velocity::radians_per_second_t{error*kPRot*rotMaxSpeed + (error - state.prevError) * kDRot};
    state.prevError = error;
    state.xSpeedMPS = units::velocity::meters_per_second_t{state.xSpeed * driveMaxSpeed};
    state.ySpeedMPS = units::velocity::meters_per_second_t{state.ySpeed * driveMaxSpeed};
    if (frc::DriverStation::GetAlliance() == frc::DriverStation::kRed) {
        state.xSpeedMPS *= -1.0;
        state.ySpeedMPS *= -1.0;
    }
    state.rotRPS = units::angular_velocity::radians_per_second_t{state.rot * rotMaxSpeed};

    if(state.ampAlign || state.trapAlign || state.sourceAlign || state.isHeadingTrack || state.thetaLock || state.ampHeadingTrack){
        drive(state.xSpeedMPS, state.ySpeedMPS, state.angleRPS, true);
    } else {
        drive(state.xSpeedMPS, state.ySpeedMPS, state.rotRPS, true);
    }
    // driveRobotRelative(frc::ChassisSpeeds{6_mps});

    if (frc::Timer::GetFPGATimestamp().to<double>() - teleopStart > TIME_TELEOP_VERT && frc::Timer::GetFPGATimestamp().to<double>() - teleopStart < TIME_TELEOP_VERT + 3) {
        operatorGamepad->setRumble(true);
    } else {
        operatorGamepad->setRumble(false);
    }
}

frc::Pose2d Drivetrain::getPoseFromSpeaker() {
    valor::AprilTagsSensor* tagSensor = aprilTagSensors[0];
    if (state.backshot && aprilTagSensors.size() >= 5) {
        tagSensor = aprilTagSensors[4];
    }
    
    auto climberTable = nt::NetworkTableInstance::GetDefault().GetTable("Climber");
    bool ledsAvailable = !climberTable->GetBoolean("Climber overriding leds", false);
    table->PutNumber("translation norm", tagSensor->getPoseFromAprilTag().Translation().Norm().to<double>());

    valor::AprilTagsSensor::Orientation orient{
        estimator->GetEstimatedPosition().Rotation().Degrees(),
        pigeon.GetPitch().GetValue(),
        pigeon.GetRoll().GetValue(),
        pigeon.GetAngularVelocityZWorld().GetValue(),
        pigeon.GetAngularVelocityXWorld().GetValue(),
        pigeon.GetAngularVelocityYWorld().GetValue()
    };

    tagSensor->getMegaTagPose2(orient);
    units::meter_t megaTag2DistanceLimit = 5.5_m;
    units::meter_t distanceToTag = tagSensor->getPoseFromAprilTag().Translation().Norm();

    if (distanceToTag < 4.8_m && distanceToTag > 0.0_m) {
        if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue && (tagSensor->getTagID() == 7 || tagSensor->getTagID() == 8)) {
            if (ledsAvailable)
                leds->setColor(0, valor::CANdleSensor::LIGHT_BLUE);
            table->PutBoolean("good to shoot", true);
            // return (MT2_POSE  && distanceToTag > megaTag2DistanceLimit) ? tagSensor->getMegaTagPose2(orient).ToPose2d() : tagSensor->getSensor().ToPose2d();
            return tagSensor->getSensor().ToPose2d();
        } else if (frc::DriverStation::GetAlliance() == frc::DriverStation::kRed && (tagSensor->getTagID() == 4 || tagSensor->getTagID() == 3)) {
            if (ledsAvailable)
                leds->setColor(0, valor::CANdleSensor::LIGHT_BLUE);
            table->PutBoolean("good to shoot", true);
            // return (MT2_POSE && distanceToTag > megaTag2DistanceLimit) ? tagSensor->getMegaTagPose2(orient).ToPose2d() : tagSensor->getSensor().ToPose2d();
            return tagSensor->getSensor().ToPose2d();
        }
        table->PutBoolean("good to shoot", false);
    }
    if (ledsAvailable)
        leds->setColor(0, valor::CANdleSensor::RED);
    return calculatedEstimator->GetEstimatedPosition();
}

units::meter_t Drivetrain::getDistanceFromSpeaker() {
    units::meter_t x = frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue ? getPoseFromSpeaker().X() - SPEAKER_BLUE_X : getPoseFromSpeaker().X() - SPEAKER_RED_X;
    units::meter_t y = getPoseFromSpeaker().Y() - SPEAKER_Y;

    return units::meter_t{sqrtf(powf(x.to<double>(), 2) + powf(y.to<double>(), 2))};
}

units::meter_t Drivetrain::getDistanceFromAmp(){
    units::meter_t x = frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue ? calculatedEstimator->GetEstimatedPosition().X() - AMP_BLUE_X : calculatedEstimator->GetEstimatedPosition().X() - AMP_RED_X;
    units::meter_t y = calculatedEstimator->GetEstimatedPosition().Y() - AMP_Y;

    return units::meter_t{sqrtf(powf(x.to<double>(), 2) + powf(y.to<double>(), 2))};
}

units::meters_per_second_t Drivetrain::getRobotSpeeds(){
    return units::meters_per_second_t{sqrtf(powf(getRobotRelativeSpeeds().vx.to<double>(), 2) + powf(getRobotRelativeSpeeds().vy.to<double>(), 2))};
}

void Drivetrain::getSpeakerLockAngleRPS(){
    units::meter_t roboXPos = getPoseFromSpeaker().X();
    units::meter_t roboYPos = getPoseFromSpeaker().Y();

    double speakerX = (frc::DriverStation::GetAlliance() == frc::DriverStation::kRed ? SPEAKER_RED_X : SPEAKER_BLUE_X).to<double>();
    double redMultiplier = frc::DriverStation::GetAlliance() == frc::DriverStation::kRed ? -1.0 : 1.0;
    double speakerXOffset = table->GetNumber("SPEAKER_X_OFFSET", SPEAKER_X_OFFSET) * redMultiplier;
    double speakerYOffset = table->GetNumber("SPEAKER_Y_OFFSET", SPEAKER_Y_OFFSET);

    state.targetAngle = units::radian_t(atan2(
        (roboYPos.to<double>() - (SPEAKER_Y.to<double>() + speakerYOffset)),
        (roboXPos.to<double>() - (speakerX + speakerXOffset))
    ));
    if (state.backshot) {
        state.targetAngle += 180_deg;
    }
}

void Drivetrain::getAmpLockAngle(){
    units::meter_t roboXPos = calculatedEstimator->GetEstimatedPosition().X();
    units::meter_t roboYPos = calculatedEstimator->GetEstimatedPosition().Y();
    if(roboYPos > SPEAKER_Y){
        bool isRed = frc::DriverStation::GetAlliance() == frc::DriverStation::kRed;
        state.targetAngle = isRed ? RED_LOCK_ANGLE : BLUE_LOCK_ANGLE;
    }
    else{
        double ampX = (frc::DriverStation::GetAlliance() == frc::DriverStation::kRed ? AMP_RED_X : AMP_BLUE_X).to<double>();
        double redMultiplier = frc::DriverStation::GetAlliance() == frc::DriverStation::kRed ? -1.0 : 1.0;
        double ampXOffset = table->GetNumber("AMP_X_OFFSET", AMP_X_OFFSET.to<double>()) * redMultiplier;
        double ampYOffset = table->GetNumber("AMP_Y_OFFSET", AMP_Y_OFFSET.to<double>());
        state.targetAngle = units::radian_t(atan2(
            roboYPos.to<double>() - (AMP_Y.to<double>() + ampYOffset), 
            roboXPos.to<double>() - (ampX + ampXOffset)
        ));
    }
}


units::radian_t Drivetrain::getAngleError(){
    units::radian_t robotRotation = calculatedEstimator->GetEstimatedPosition().Rotation().Radians();
    double errorAngle = robotRotation.to<double>() - state.targetAngle.to<double>();
    if(errorAngle > PI) {
        return units::radian_t(errorAngle - 2*PI);
    }
    else if(errorAngle < -PI){
        return units::radian_t(2*PI + errorAngle);
    }
    else{
        return (robotRotation - state.targetAngle);
    }
}

void Drivetrain::setAlignmentAngle(Alignment align) {
    bool isRed = frc::DriverStation::GetAlliance() == frc::DriverStation::kRed;

    if (align == Alignment::AMP) state.targetAngle = BLUE_AMP_ROT_ANGLE;
    else if (align == Alignment::TRAP){
        for(valor::AprilTagsSensor* aprilLime :  aprilTagSensors){
            if(aprilLime->getTagID() == 16){
                // state.targetAngle = units::radian_t(BLUE_RIGHT_TRAP_ROT_ANGLE);
            }
            else if(aprilLime->getTagID() == 15){
                // state.targetAngle = units::radian_t(BLUE_LEFT_TRAP_ROT_ANGLE);
            }
            else if(aprilLime->getTagID() == 14){
                // state.targetAngle = units::radian_t(BLUE_CENTER_TRAP_ROT_ANGLE);
            }
        }
        state.targetAngle = 0_deg;
    }
    else if (align == Alignment::SOURCE)
        state.targetAngle = isRed ? RED_SOURCE_ROT_ANGLE : BLUE_SOURCE_ROT_ANGLE;
    else 
        state.targetAngle = isRed ? RED_LOCK_ANGLE : BLUE_LOCK_ANGLE;
}

double Drivetrain::clampAngleRadianRange(units::radian_t angle, double max){
    return ((angle)/(max)).to<double>();
}

frc::SwerveDriveKinematics<SWERVE_COUNT>* Drivetrain::getKinematics()
{
    return kinematics;
}

frc::Pose2d Drivetrain::getPose_m()
{
    return estimator->GetEstimatedPosition();
}

frc::Pose2d Drivetrain::getCalculatedPose_m()
{
    return calculatedEstimator->GetEstimatedPosition();
}

void Drivetrain::resetGyro(){
    frc::Pose2d initialPose = getPose_m();
    frc::Pose2d desiredPose = frc::Pose2d(initialPose.X(), initialPose.Y(), frc::Rotation2d(0_deg));
    resetOdometry(desiredPose);
}

wpi::array<frc::SwerveModulePosition, SWERVE_COUNT> Drivetrain::getModuleStates()
{
    wpi::array<frc::SwerveModulePosition, SWERVE_COUNT> modulePositions = wpi::array<frc::SwerveModulePosition, SWERVE_COUNT>(wpi::empty_array);
    for (size_t i = 0; i < swerveModules.size(); i++)
    {
        modulePositions[i] = swerveModules[i]->getModulePosition();
    }
    return modulePositions;
}

void Drivetrain::resetOdometry(frc::Pose2d pose)
{
    estimator->ResetPosition(getPigeon(), getModuleStates(), pose);
    calculatedEstimator->ResetPosition(getPigeon(), getModuleStates(), pose);
}

frc::Rotation2d Drivetrain::getPigeon() {
    state.accel = {
        pigeon.GetAccelerationX().GetValue(),
        pigeon.GetAccelerationY().GetValue(),
        pigeon.GetAccelerationZ().GetValue()
    };
    return pigeon.GetRotation2d();
}

void Drivetrain::resetDriveEncoders(){
    for (size_t i = 0; i < swerveModules.size(); i++)
    {
        swerveModules[i]->resetDriveEncoder();
    }
}

void Drivetrain::drive(units::velocity::meters_per_second_t vx_mps, units::velocity::meters_per_second_t vy_mps, units::angular_velocity::radians_per_second_t omega_radps, bool isFOC){
    auto desiredStates = getModuleStates(vx_mps,
                                  vy_mps,
                                  omega_radps,
                                  isFOC);
    setSwerveDesiredState(desiredStates, true);
}

void Drivetrain::driveRobotRelative(frc::ChassisSpeeds speeds) {
    table->PutNumberArray("target chassis speeds", std::vector<double>{speeds.vx.to<double>(), speeds.vy.to<double>(), speeds.omega.to<double>()});
    auto desiredStates = getModuleStates(speeds);
    setSwerveDesiredState(desiredStates, false);
    std::vector<double> sStates;
    for (size_t i = 0; i < swerveModules.size(); i++)
    {
        sStates.push_back(desiredStates[i].angle.Degrees().to<double>());
        sStates.push_back(desiredStates[i].speed.to<double>());
    }
    table->PutNumberArray("target swerves", sStates);
}

wpi::array<frc::SwerveModuleState, SWERVE_COUNT> Drivetrain::getModuleStates(units::velocity::meters_per_second_t vx_mps,
                                                                  units::velocity::meters_per_second_t vy_mps,
                                                                  units::angular_velocity::radians_per_second_t omega_radps,
                                                                  bool isFOC)
{
    frc::ChassisSpeeds chassisSpeeds = isFOC ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(vx_mps,
                                                                                           vy_mps,
                                                                                           omega_radps,
                                                                                           estimator->GetEstimatedPosition().Rotation())
                                             : frc::ChassisSpeeds{vx_mps, vy_mps, omega_radps};
    return getModuleStates(chassisSpeeds);
}

wpi::array<frc::SwerveModuleState, SWERVE_COUNT> Drivetrain::getModuleStates(frc::ChassisSpeeds chassisSpeeds)
{
    auto states = kinematics->ToSwerveModuleStates(chassisSpeeds);
    kinematics->DesaturateWheelSpeeds(&states, units::velocity::meters_per_second_t{driveMaxSpeed});
    return states;
}

frc::ChassisSpeeds Drivetrain::getRobotRelativeSpeeds(){
    wpi::array<frc::SwerveModuleState, 4> moduleStates = {
        swerveModules[0]->getState(),
        swerveModules[1]->getState(),
        swerveModules[2]->getState(),
        swerveModules[3]->getState()
    };
    return kinematics->ToChassisSpeeds(moduleStates);
}

void Drivetrain::setSwerveDesiredState(wpi::array<frc::SwerveModuleState, SWERVE_COUNT> desiredStates, bool isDriveOpenLoop)
{
    for (int i = 0; i < SWERVE_COUNT; i++) {
        swerveModules[i]->setDesiredState(desiredStates[i], isDriveOpenLoop);
    }
}

void Drivetrain::angleLock(){
    if (0 > getPose_m().Rotation().Degrees().to<double>()){
        state.rot = (-getPose_m().Rotation().Degrees().to<double>()/180) - 1.0;
    } else{
        state.rot = (-getPose_m().Rotation().Degrees().to<double>()/180) + 1.0;
    }
    
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

void Drivetrain::setXMode(){
    drive(static_cast<units::velocity::meters_per_second_t>(0),static_cast<units::velocity::meters_per_second_t>(0),static_cast<units::angular_velocity::radians_per_second_t>(0),true);
    azimuthControllers[0]->setPosition(std::round(azimuthControllers[0]->getPosition()) + 0.125);
    azimuthControllers[1]->setPosition(std::round(azimuthControllers[1]->getPosition()) + 0.375);
    azimuthControllers[2]->setPosition(std::round(azimuthControllers[2]->getPosition()) - 0.375);
    azimuthControllers[3]->setPosition(std::round(azimuthControllers[3]->getPosition()) - 0.125);
    setDriveMotorNeutralMode(valor::NeutralMode::Brake);
}

bool Drivetrain::isWheelSlip(int i)
{
    double tVel = sqrtf(powf(getRobotRelativeSpeeds().vx.to<double>(), 2) + powf(getRobotRelativeSpeeds().vy.to<double>(), 2));
    return fabs(driveControllers[i]->getSpeed()) > tVel * SLIP_FACTOR;
}

void Drivetrain::setDriveMotorNeutralMode(valor::NeutralMode mode) {
    for (int i = 0; i < SWERVE_COUNT; i++)
    {
        driveControllers[i]->setNeutralMode(mode);
    }
}

frc2::InstantCommand* Drivetrain::getSetXMode(){
    frc2::InstantCommand* cmd_XMode = new frc2::InstantCommand( [&] {
        setXMode();
    });
     return cmd_XMode;
}

void Drivetrain::InitSendable(wpi::SendableBuilder& builder)
    {
        builder.SetSmartDashboardType("Subsystem");

        builder.AddDoubleProperty(
            "xSpeed",
            [this] { return state.xSpeed; },
            nullptr
        );
        builder.AddDoubleProperty(
            "ySpeed",
            [this] { return state.ySpeed; },
            nullptr
        );

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

        builder.AddDoubleArrayProperty(
            "Auto Camera Pose",
            [this] {
                std::vector<double> autoCameraPose;
                autoCameraPose.push_back(calculatedEstimator->GetEstimatedPosition().X().to<double>());
                autoCameraPose.push_back(calculatedEstimator->GetEstimatedPosition().Y().to<double>());
                autoCameraPose.push_back(estimator->GetEstimatedPosition().Rotation().Radians().to<double>());
                return autoCameraPose;
            },
            nullptr
        );
        builder.AddDoubleProperty(
            "rotSpeed",
            [this] { return state.rot; },
            nullptr
        );

        builder.AddDoubleProperty(
            "xSpeedMPS",
            [this] { return state.xSpeedMPS.to<double>(); },
            nullptr
        );
        builder.AddDoubleProperty(
            "ySpeedMPS",
            [this] { return state.ySpeedMPS.to<double>(); },
            nullptr
        );
        builder.AddDoubleProperty(
            "rotSpeedMPS",
            [this] { return state.rotRPS.to<double>(); },
            nullptr
        );
        builder.AddDoubleProperty(
            "x",
            [this] { return getPose_m().X().to<double>(); },
            nullptr
        );
        builder.AddDoubleProperty(
            "y",
            [this] { return getPose_m().Y().to<double>(); },
            nullptr
        );
        builder.AddDoubleProperty(
            "theta",
            [this] { return getPose_m().Rotation().Degrees().to<double>(); },
            nullptr
        );
        builder.AddBooleanProperty(
            "swerveGood",
            [this] { return swerveNoError; },
            nullptr
        );
        builder.AddDoubleArrayProperty(
            "pose",
            [this] 
            { 
                std::vector<double> pose;
                pose.push_back(getPose_m().X().to<double>());
                pose.push_back(getPose_m().Y().to<double>());
                pose.push_back(getPose_m().Rotation().Radians().to<double>());
                return pose;
            },
            nullptr
        );
        builder.AddDoubleArrayProperty(
            "calculatedPose",
            [this] 
            { 
                frc::Pose2d estimatedPose = calculatedEstimator->GetEstimatedPosition();
                std::vector<double> pose;
                pose.push_back(estimatedPose.X().to<double>());
                pose.push_back(estimatedPose.Y().to<double>());
                pose.push_back(estimatedPose.Rotation().Radians().to<double>());
                return pose;
            },
            nullptr
        );
        builder.AddDoubleArrayProperty(
            "Pose From Speaker",
            [this]
            {
                std::vector<double> pose;
                frc::Pose2d myPos = getPoseFromSpeaker();
                pose.push_back(myPos.X().to<double>());
                pose.push_back(myPos.Y().to<double>());
                pose.push_back(myPos.Rotation().Radians().to<double>());
                return pose;
            },
            nullptr
        );
        builder.AddDoubleProperty(
            "pigeonPitch",
            [this]
            {
                return pigeon.GetPitch().GetValueAsDouble();
            },
            nullptr
        );
        builder.AddDoubleProperty(
            "pigeoYaw",
            [this]
            {
                return pigeon.GetYaw().GetValueAsDouble();
            },
            nullptr
        );
        builder.AddDoubleProperty(
            "pigeonRoll",
            [this]
            {
                return pigeon.GetRoll().GetValueAsDouble();
            },
            nullptr
        );
        builder.AddBooleanProperty(
            "Tilted",
            [this]
            {
                double yaw = pigeon.GetYaw().GetValueAsDouble(), roll = pigeon.GetRoll().GetValueAsDouble(); // in degrees
                double elevation = std::fabs(yaw) + std::fabs(roll); // not at all but close enough
                return elevation > 5.0;
            },
            nullptr
        );
        builder.AddIntegerProperty(
            "stage",
            [this]
            {
                return state.stage;
            },
            nullptr
        );
        builder.AddDoubleProperty(
            "xVelocity",
            [this]
            {
                return getRobotRelativeSpeeds().vx.to<double>();
            },
            nullptr
        );
        builder.AddDoubleProperty(
            "yVelocity",
            [this]
            {
                return getRobotRelativeSpeeds().vy.to<double>();
            },
            nullptr
        );
        builder.AddDoubleArrayProperty(
            "swerve states",
            [this] 
            { 
                std::vector<double> states;
                states.push_back(swerveModules[0]->getState().angle.Degrees().to<double>());
                states.push_back(swerveModules[0]->getState().speed.to<double>());
                states.push_back(swerveModules[1]->getState().angle.Degrees().to<double>());
                states.push_back(swerveModules[1]->getState().speed.to<double>());
                states.push_back(swerveModules[2]->getState().angle.Degrees().to<double>());
                states.push_back(swerveModules[2]->getState().speed.to<double>());
                states.push_back(swerveModules[3]->getState().angle.Degrees().to<double>());
                states.push_back(swerveModules[3]->getState().speed.to<double>());
                return states;
            },
            nullptr
        );
        builder.AddDoubleArrayProperty(
            "swerve azimuths",
            [this] 
            { 
                std::vector<double> states;
                for (int i = 0; i < 4; i++){
                    double ang = swerveModules[0]->getState().angle.Degrees().to<double>();
                    while (ang < 0)
                        ang += 180;
                    while (ang > 180)
                        ang -= 180;
                    states.push_back(ang);
                }
                return states;
            },
            nullptr
        );

        builder.AddDoubleProperty(
            "targetAngle",
            [this] {return (units::degree_t(state.targetAngle)).to<double>();},
            nullptr
        );

        builder.AddDoubleProperty(
            "errorAngle",
            [this] {return (units::degree_t(getAngleError()).to<double>());},
            nullptr
        );

        builder.AddDoubleProperty(
            "errorAngleRPS",
            [this] {return (state.angleRPS).to<double>();},
            nullptr
        );

        builder.AddDoubleProperty(
            "distanceFromSpeaker",
            [this] {return (state.distanceFromSpeaker).to<double>();},
            nullptr
        );
        builder.AddDoubleProperty(
            "velocity current",
            [this] {return currentPoseTracker.getAverageVelocity().to<double>();},
            nullptr
        );
        builder.AddDoubleProperty(
            "velocity target",
            [this] {return targetPoseTracker.getAverageVelocity().to<double>();},
            nullptr
        );
        builder.AddDoubleProperty(
            "acceleration current",
            [this] {return currentPoseTracker.getAverageAcceleration().to<double>();},
            nullptr
        );
        builder.AddDoubleProperty(
            "acceleration target",
            [this] {return targetPoseTracker.getAverageAcceleration().to<double>();},
            nullptr
        );
        builder.AddDoubleProperty(
            "angular velocity current",
            [this] {return currentPoseTracker.getAverageAngularVelocity().to<double>();},
            nullptr
        );
        builder.AddDoubleProperty(
            "angular velocity target",
            [this] {return targetPoseTracker.getAverageAngularVelocity().to<double>();},
            nullptr
        );
        builder.AddBooleanProperty(
            "Manually flaged error",
            [this] {return state.manualFlag;},
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
        builder.AddBooleanProperty(
            "Pit Mode",
            [this] {return state.pitMode;},
            nullptr
        );
        builder.AddBooleanProperty(
            "Swerve 0 Slip",
            [this] {return isWheelSlip(0);},
            nullptr
        );
        builder.AddBooleanProperty(
            "Swerve 1 Slip",
            [this] {return isWheelSlip(1);},
            nullptr
        );
        builder.AddBooleanProperty(
            "Swerve 2 Slip",
            [this] {return isWheelSlip(2);},
            nullptr
        );
        builder.AddBooleanProperty(
            "Swerve 3 Slip",
            [this] {return isWheelSlip(3);},
            nullptr
        );

        builder.AddBooleanProperty(
            "B***k shot",
            [this] {return state.backshot;},
            nullptr
        );
        builder.AddDoubleProperty(
            "distanceToAmp",
            [this] {return state.distanceFromAmp.to<double>();},
            nullptr
        );
    }

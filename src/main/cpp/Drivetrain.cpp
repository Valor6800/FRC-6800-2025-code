#include "Drivetrain.h"
#include <frc/DriverStation.h>
#include <iostream>
#include <math.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/util/HolonomicPathFollowerConfig.h>
#include <pathplanner/lib/util/PIDConstants.h>
#include <pathplanner/lib/util/ReplanningConfig.h>
#include <string>
#include "Constants.h"
#include "units/length.h"
#include "valkyrie/sensors/AprilTagsSensor.h"
#include "units/length.h"
#include "valkyrie/sensors/VisionSensor.h"
#include "frc/geometry/Pose3d.h"
#include "frc/geometry/Rotation3d.h"
#include "units/angle.h"

using namespace pathplanner;

#define SPEAKER_Y 5.543042_m
#define SPEAKER_BLUE_X 0.0_m
#define SPEAKER_RED_X 16.4846_m

#define TXRANGE  30.0f
#define KPIGEON 2.0f
#define KLIMELIGHT -29.8f
// #define KP_LOCK 0.2f
#define KP_LIMELIGHT 0.7f

#define KPX 60.0f //50
#define KIX 0.0f //0
#define KDX 0.0f //.1
#define KFX 0.0f

#define KPY 60.0f //65
#define KIY 0.0f //0
#define KDY 0.0f //.1
#define KFY 0.0f

#define KPT 15.0f
#define KIT 0.0f
#define KDT 0.0f
#define KFT 0.0f

#define AZIMUTH_K_P 0.00001f
#define AZIMUTH_K_I 0.0f
#define AZIMUTH_K_D 0.0f
#define AZIMUTH_K_E 0.0027f

#define AZIMUTH_K_VEL 10.0f
#define AZIMUTH_K_ACC_MUL 0.05f

#define DRIVE_K_P 0.001f
#define DRIVE_K_I 0.0f
#define DRIVE_K_D 0.0f
#define DRIVE_K_E 0.0027f

#define DRIVE_K_VEL 6.0f
#define DRIVE_K_ACC_MUL 0.05f

#define MOTOR_FREE_SPEED 6380.0f
#define WHEEL_DIAMETER_M 0.0973f //0.1016
#define DRIVE_GEAR_RATIO 5.51f
#define AZIMUTH_GEAR_RATIO 13.37f
#define AUTO_MAX_SPEED 10.0f
#define AUTO_MAX_ACCEL_SECONDS 5.33f //5.33
#define ROT_SPEED_MUL 2.0f

#define AUTO_VISION_THRESHOLD 4.0f //meters

#define X_TIME 214.85f

#define MODULE_DIFF_XS {1, 1, -1, -1}
#define MODULE_DIFF_YS {1, -1, -1, 1}

#define DRIVETRAIN_CAN_BUS ""
#define PIGEON_CAN_BUS "baseCAN"

#define KP_ROTATE -0.6f
#define SPEAKER_X_OFFSET 0.15f
#define SPEAKER_Y_OFFSET 0.00f

#define VISION_ACCEPTANCE 4.0_m // meters

Drivetrain::Drivetrain(frc::TimedRobot *_robot) : valor::BaseSubsystem(_robot, "Drivetrain"),
                        driveMaxSpeed(MOTOR_FREE_SPEED / 60.0 / DRIVE_GEAR_RATIO * WHEEL_DIAMETER_M * M_PI),
                        rotMaxSpeed(ROT_SPEED_MUL * 2 * M_PI),
                        autoMaxSpeed(AUTO_MAX_SPEED),
                        autoMaxAccel(AUTO_MAX_SPEED/AUTO_MAX_ACCEL_SECONDS),
                        rotMaxAccel(rotMaxSpeed * 0.5),
                        pigeon(CANIDs::PIGEON_CAN, PIGEON_CAN_BUS),
                        motorLocations(wpi::empty_array),
                        kinematics(NULL),
                        estimator(NULL),
                        calculatedEstimator(NULL),
                        config(NULL),
                        swerveNoError(true)
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
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
    delete config;
}

void Drivetrain::configSwerveModule(int i)
{

   int MDX[] = MODULE_DIFF_XS;
   int MDY[] = MODULE_DIFF_YS;

    motorLocations[i] = frc::Translation2d{Constants::moduleDiff() * MDX[i],
                                           Constants::moduleDiff() * MDY[i]};

    valor::PIDF azimuthPID;
    azimuthPID.velocity = AZIMUTH_K_VEL;
    azimuthPID.acceleration = AZIMUTH_K_ACC_MUL;
    azimuthPID.P = AZIMUTH_K_P;
    azimuthPID.I = AZIMUTH_K_I;
    azimuthPID.D = AZIMUTH_K_D;
    azimuthPID.error = AZIMUTH_K_E;

    azimuthControllers.push_back(new SwerveAzimuthMotor(CANIDs::AZIMUTH_CANS[i],
                                                      valor::NeutralMode::Brake,
                                                      Constants::swerveAzimuthsReversals()[i],
                                                      DRIVETRAIN_CAN_BUS));
    azimuthControllers[i]->setConversion(1.0 / AZIMUTH_GEAR_RATIO);
    azimuthControllers[i]->setPIDF(azimuthPID, 0);

    valor::PIDF drivePID;
    drivePID.velocity = DRIVE_K_VEL;
    drivePID.acceleration = DRIVE_K_ACC_MUL;
    drivePID.P = DRIVE_K_P;
    drivePID.I = DRIVE_K_I;
    drivePID.D = DRIVE_K_D;
    drivePID.error = DRIVE_K_E;

    driveControllers.push_back(new SwerveDriveMotor(CANIDs::DRIVE_CANS[i],
                                                    valor::NeutralMode::Coast,
                                                    Constants::swerveDrivesReversals()[i],
                                                    DRIVETRAIN_CAN_BUS));
    driveControllers[i]->setConversion(1.0 / DRIVE_GEAR_RATIO * M_PI * WHEEL_DIAMETER_M);
    driveControllers[i]->setPIDF(drivePID, 0);

    swerveModules.push_back(new valor::Swerve<SwerveAzimuthMotor, SwerveDriveMotor>(azimuthControllers[i], driveControllers[i], motorLocations[i]));
    swerveModules[i]->setMaxSpeed(driveMaxSpeed);
    

}

void Drivetrain::resetState()
{
    resetDriveEncoders();
    pullSwerveModuleZeroReference();
    resetOdometry(frc::Pose2d{0_m, 0_m, 0_rad});
}

void Drivetrain::init()
{

    for (std::pair<const char*, frc::Pose3d> aprilCam : Constants::aprilCameras) {
        aprilTagSensors.push_back(new valor::AprilTagsSensor(robot, aprilCam.first, aprilCam.second));
    }

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
    config = new frc::TrajectoryConfig(units::velocity::meters_per_second_t{autoMaxSpeed}, units::acceleration::meters_per_second_squared_t{autoMaxAccel});

    xPIDF.P = KPX;
    xPIDF.I = KIX;
    xPIDF.D = KDX;
    xPIDF.F = KFX;

    yPIDF.P = KPY;
    yPIDF.I = KIY;
    yPIDF.D = KDY;
    yPIDF.F = KFY;

    thetaPIDF.P = KPT;
    thetaPIDF.I = KIT;
    thetaPIDF.D = KDT;
    thetaPIDF.F = KFT;

    table->PutNumber("Vision Std", 3.0);
    table->PutBoolean("Load Swerve Mag Encoder", false);

    table->PutNumber("Vision Acceptance", VISION_ACCEPTANCE.to<double>() );
    table->PutNumber("DoubtX", 1.0);
    table->PutNumber("DoubtY", 1.0);
    table->PutNumber("DoubtRot", 1.0);

    table->PutNumber("KPLIMELIGHT", KP_LIMELIGHT);
    table->PutNumber("KP_ROTATION", KP_ROTATE);
    table->PutNumber("SPEAKER_X_OFFSET", SPEAKER_X_OFFSET);
    table->PutNumber("SPEAKER_Y_OFFSET", SPEAKER_Y_OFFSET);

    state.lock = false;

    resetState();

    AutoBuilder::configureHolonomic(
        [this](){ return getPose_m(); }, // Robot pose supplier
        [this](frc::Pose2d pose){ resetOdometry(pose); }, // Method to reset odometry (will be called if your auto has a starting pose)
        [this](){ return getRobotRelativeSpeeds(); }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        [this](frc::ChassisSpeeds speeds){ driveRobotRelative(speeds); }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            PIDConstants(getXPIDF().P, getXPIDF().I, getXPIDF().D), // Translation PID constants
            PIDConstants(getThetaPIDF().P, getThetaPIDF().I, getThetaPIDF().D), // Rotation PID constants
            units::meters_per_second_t{driveMaxSpeed}, // Max module speed, in m/s
            Constants::driveBaseRadius(), // Drive base radius in meters. Distance from robot center to furthest module.
            ReplanningConfig() // Default path replanning config. See the API for the options here
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
    if (!driverGamepad || !operatorGamepad) return;

    if (driverGamepad->GetBackButtonPressed()) {
        resetGyro();
    }


    state.topTape = operatorGamepad->DPadUp();
    state.bottomTape = operatorGamepad->DPadRight();
    state.lock = state.adas || driverGamepad->GetBButton();

    state.xSpeed = driverGamepad->leftStickY(2);
    state.ySpeed = driverGamepad->leftStickX(2);
    if (!state.lock){
    state.rot = driverGamepad->rightStickX(3);
    }
    state.isHeadingTrack = driverGamepad->GetAButton();

    state.xPose = driverGamepad->GetXButton();
}

void Drivetrain::calculateCarpetPose()
{
    calculatedEstimator->UpdateWithTime(frc::Timer::GetFPGATimestamp(),
        getPigeon(),
        getModuleStates()
    );
    frc::Pose2d newPose = calculatedEstimator->GetEstimatedPosition();
    units::meter_t deltaX = newPose.X() - previousPose.X();
    double factor = deltaX < units::meter_t{0} ? 1.05 : 1;
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
    if (table->GetBoolean("Load Swerve Mag Encoder",false))
        pullSwerveModuleZeroReference();

    estimator->UpdateWithTime(frc::Timer::GetFPGATimestamp(),
        getPigeon(),
        getModuleStates()
    );
    calculateCarpetPose();

    frc::Pose2d botpose;
    
    doubtX = table->GetNumber("DoubtX", 1.0);
    doubtY = table->GetNumber("DoubtY", 1.0);
    doubtRot = table->GetNumber("DoubtRot", 1.0);
    visionAcceptanceRadius = (units::meter_t) table->GetNumber("Vision Acceptance", VISION_ACCEPTANCE.to<double>());

    for (valor::AprilTagsSensor* aprilLime : aprilTagSensors) {
        aprilLime->applyVisionMeasurement(calculatedEstimator, visionAcceptanceRadius, doubtX, doubtY);
    }

    if (driverGamepad->GetStartButton()) {
        for (valor::AprilTagsSensor* aprilLime : aprilTagSensors) {
            if (aprilLime->hasTarget()) {
                botpose = aprilLime->getSensor().ToPose2d();
                resetOdometry(botpose);
                break;
            }
        }
    }

    getSpeakerLockAngleRPS();
    double kPRot = table->GetNumber("KP_ROTATION", KP_ROTATE);
    double speakerXOffset = table->GetNumber("SPEAKER_X_OFFSET", SPEAKER_X_OFFSET);
    double speakerYOffset = table->GetNumber("SPEAKER_Y_OFFSET", SPEAKER_Y_OFFSET);
    state.angleRPS = units::angular_velocity::radians_per_second_t(getAngleError().to<double>()*kPRot*rotMaxSpeed);
}

void Drivetrain::assignOutputs()
{
    if (state.lock){angleLock();}
    state.xSpeedMPS = units::velocity::meters_per_second_t{state.xSpeed * driveMaxSpeed};
    state.ySpeedMPS = units::velocity::meters_per_second_t{state.ySpeed * driveMaxSpeed};
    state.rotRPS = units::angular_velocity::radians_per_second_t{state.rot * rotMaxSpeed};
    if (state.xPose){
        setXMode();
    } else if(state.isHeadingTrack){

        drive(state.xSpeedMPS, state.ySpeedMPS, state.angleRPS, true);
    }
    else if (state.adas){
        drive(state.xSpeedMPS, state.ySpeedMPS, state.rotRPS, true);
    } 
    else {
        setDriveMotorNeutralMode(valor::NeutralMode::Coast);
        drive(state.xSpeedMPS, state.ySpeedMPS, state.rotRPS, true);
    }
}

void Drivetrain::getSpeakerLockAngleRPS(){
    units::radian_t targetRotAngle;
    units::meter_t roboXPos = calculatedEstimator->GetEstimatedPosition().X();
    units::meter_t roboYPos = calculatedEstimator->GetEstimatedPosition().Y();
    if(frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue){
        targetRotAngle = units::radian_t(atan2(
            (roboYPos.to<double>() - (SPEAKER_Y.to<double>() +  table->GetNumber("SPEAKER_Y_OFFSET", SPEAKER_Y_OFFSET))),
            (roboXPos.to<double>() - (SPEAKER_BLUE_X.to<double>() +  table->GetNumber("SPEAKER_X_OFFSET", SPEAKER_X_OFFSET)))
        ));
    }
    else{
        targetRotAngle = units::radian_t(atan2(
            (roboYPos.to<double>() - (SPEAKER_Y.to<double>() + table->GetNumber("SPEAKER_Y_OFFSET", SPEAKER_Y_OFFSET))),
            (roboXPos.to<double>() - (SPEAKER_RED_X.to<double>() - table->GetNumber("SPEAKER_X_OFFSET", SPEAKER_X_OFFSET)))
        )) + units::radian_t(PI);
    }
    state.targetAngle = targetRotAngle;
}

units::radian_t Drivetrain::getAngleError(){
    units::radian_t robotRotation = estimator->GetEstimatedPosition().Rotation().Radians();
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

double Drivetrain::clampAngleRadianRange(units::radian_t angle, double max){
    return ((angle)/(max)).to<double>();
}

void Drivetrain::pullSwerveModuleZeroReference(){
    swerveNoError = true;
    for (size_t i = 0; i < swerveModules.size(); i++) {
        swerveNoError &= swerveModules[i]->loadAndSetAzimuthZeroReference(Constants::swerveZeros());
    }
}

frc::SwerveDriveKinematics<SWERVE_COUNT>* Drivetrain::getKinematics()
{
    return kinematics;
}

frc::Pose2d Drivetrain::getPose_m()
{
    return estimator->GetEstimatedPosition();
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
    return pigeon.GetRotation2d();
}

void Drivetrain::resetDriveEncoders(){
    for (size_t i = 0; i < swerveModules.size(); i++)
    {
        swerveModules[i]->resetDriveEncoder();
    }
}

void Drivetrain::drive(units::velocity::meters_per_second_t vx_mps, units::velocity::meters_per_second_t vy_mps, units::angular_velocity::radians_per_second_t omega_radps, bool isFOC){
    auto states = getModuleStates(vx_mps,
                                  vy_mps,
                                  omega_radps,
                                  isFOC);
    for (size_t i = 0; i < swerveModules.size(); i++)
    {
        swerveModules[i]->setDesiredState(states[i], true);
    }
}

void Drivetrain::driveRobotRelative(frc::ChassisSpeeds speeds) {
    auto states = getModuleStates(speeds);
    for (size_t i = 0; i < swerveModules.size(); i++)
    {
        swerveModules[i]->setDesiredState(states[i], true);
    }
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
    auto states = kinematics->ToSwerveModuleStates(chassisSpeeds);
    kinematics->DesaturateWheelSpeeds(&states, units::velocity::meters_per_second_t{driveMaxSpeed});
    return states;
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

void Drivetrain::setModuleStates(wpi::array<frc::SwerveModuleState, SWERVE_COUNT> desiredStates){ 
    kinematics->DesaturateWheelSpeeds(&desiredStates, units::velocity::meters_per_second_t{autoMaxSpeed});
    for (int i = 0; i < SWERVE_COUNT; i++)
    {
        swerveModules[i]->setDesiredState(desiredStates[i], false);
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
                    aprilLime->applyVisionMeasurement(estimator, (units::meter_t) table->GetNumber("Vision Acceptance", VISION_ACCEPTANCE.to<double>()));
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

double Drivetrain::getDriveMaxSpeed() {
    return driveMaxSpeed;
}

double Drivetrain::getAutoMaxSpeed() {
    return autoMaxSpeed;
}

double Drivetrain::getAutoMaxAcceleration() {
    return autoMaxAccel;
}


double Drivetrain::getRotationMaxSpeed() {
    return rotMaxSpeed;
}

double Drivetrain::getRotationMaxAcceleration() {
    return rotMaxAccel;
}

frc::TrajectoryConfig & Drivetrain::getTrajectoryConfig() {    
    return *config;
}

valor::PIDF Drivetrain::getXPIDF() {
    return xPIDF;
}

valor::PIDF  Drivetrain::getYPIDF() {
    return yPIDF;
}

valor::PIDF Drivetrain::getThetaPIDF() {
    return thetaPIDF;
}

void Drivetrain::setAutoMaxAcceleration(double acceleration, double multiplier)  {

}

void Drivetrain::setXMode(){
    drive(static_cast<units::velocity::meters_per_second_t>(0),static_cast<units::velocity::meters_per_second_t>(0),static_cast<units::angular_velocity::radians_per_second_t>(0),true);
    azimuthControllers[0]->setPosition(std::round(azimuthControllers[0]->getPosition()) + 0.125);
    azimuthControllers[1]->setPosition(std::round(azimuthControllers[1]->getPosition()) + 0.375);
    azimuthControllers[2]->setPosition(std::round(azimuthControllers[2]->getPosition()) - 0.375);
    azimuthControllers[3]->setPosition(std::round(azimuthControllers[3]->getPosition()) - 0.125);
    setDriveMotorNeutralMode(valor::NeutralMode::Brake);
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
                pose.push_back(getPose_m().Rotation().Degrees().to<double>());
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
                pose.push_back(estimatedPose.Rotation().Degrees().to<double>());
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
        builder.AddIntegerProperty(
            "stage",
            [this]
            {
                return state.stage;
            },
            nullptr
        );
        builder.AddIntegerProperty(
            "xVelocity",
            [this]
            {
                return getRobotRelativeSpeeds().vx.to<double>();
            },
            nullptr
        );
        builder.AddIntegerProperty(
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
    }


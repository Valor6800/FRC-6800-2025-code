#include "valkyrie/drivetrain/Swerve.h"
#include "Eigen/Core"
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/length.h"
#include "units/math.h"
#include "units/velocity.h"
#include "valkyrie/controllers/PhoenixController.h"
#include "wpi/sendable/SendableRegistry.h"
#include <iostream>

#include <cstdio>
#include <filesystem>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/geometry/Translation2d.h>
#include <frc/DriverStation.h>
#include <memory>

#define MODULE_DIFF_XS {1, 1, -1, -1}
#define MODULE_DIFF_YS {1, -1, -1, 1}

const units::hertz_t KP_ROTATE(-90);
const units::hertz_t KD_ROTATE(-30);


#define MAKE_VECTOR(angle) Eigen::Vector2d{units::math::cos(angle).value(), units::math::sin(angle).value()}

using namespace valor;

// Explicit template instantiation
// This is needed for linking
template class valor::Swerve<valor::PhoenixController, valor::PhoenixController>;

template<class AzimuthMotor, class DriveMotor>
Swerve<AzimuthMotor, DriveMotor>::Swerve(frc::TimedRobot *_robot,
    const char* _name,
    std::vector<std::pair<AzimuthMotor*, DriveMotor*>> modules,
    units::meter_t _module_radius,
    units::meter_t _wheelDiameter
) : valor::BaseSubsystem(_robot, _name), lockingToTarget(false), useCarpetGrain(false)
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);

    wpi::SendableRegistry::Add(
        &rot_controller, "rot_controller"
    );

    int MDX[] = MODULE_DIFF_XS;
    int MDY[] = MODULE_DIFF_YS;
    wpi::array<frc::Translation2d, MODULE_COUNT> motorLocations = wpi::array<frc::Translation2d, MODULE_COUNT>(wpi::empty_array);
    for (size_t i = 0; i < MODULE_COUNT; i++) {
        motorLocations[i] = frc::Translation2d{_module_radius * MDX[i], _module_radius * MDY[i]};
        swerveModules.push_back(new valor::SwerveModule<AzimuthMotor, DriveMotor>(
            modules[i].first,
            modules[i].second,
            motorLocations[i],
            _wheelDiameter
        ));
    }


    rot_controller.SetTolerance(rotPosTolerance, rotVelTolerance);
    y_controller.SetTolerance(yPosTolerance, yVelTolerance);
    maxDriveSpeed = swerveModules[0]->getMaxDriveSpeed();
    maxRotationSpeed = units::radian_t{2.0 * M_PI} * swerveModules[0]->getMaxDriveSpeed() / _module_radius;

    kinematics = std::make_unique<frc::SwerveDriveKinematics<MODULE_COUNT>>(motorLocations);
    rawEstimator = std::make_unique<frc::SwerveDrivePoseEstimator<MODULE_COUNT>>(*kinematics, getGyro(), getModuleStates(), frc::Pose2d{0_m, 0_m, 0_rad});
    calcEstimator = std::make_unique<frc::SwerveDrivePoseEstimator<MODULE_COUNT>>(*kinematics, getGyro(), getModuleStates(), frc::Pose2d{0_m, 0_m, 0_rad});

    resetState();
}

template<class AzimuthMotor, class DriveMotor>
Swerve<AzimuthMotor, DriveMotor>::~Swerve()
{
    for (int i = 0; i < MODULE_COUNT; i++)
    {
        delete swerveModules[i];
    }

    rawEstimator.release();
    calcEstimator.release();
}

template<class AzimuthMotor, class DriveMotor>
void Swerve<AzimuthMotor, DriveMotor>::init()
{
    rot_controller.EnableContinuousInput(units::radian_t{-M_PI}, units::radian_t{M_PI});
}

template<class AzimuthMotor, class DriveMotor>
void Swerve<AzimuthMotor, DriveMotor>::assessInputs()
{
    if (!driverGamepad || !driverGamepad->IsConnected())
        return;

    if (driverGamepad->GetBackButtonPressed()) {
        resetGyro();
    }

    xSpeed = driverGamepad->leftStickY(2);
    ySpeed = driverGamepad->leftStickX(2);
    rotSpeed = driverGamepad->rightStickX(3);

    alignToTarget = driverGamepad->GetAButton();
    if (driverGamepad->GetAButtonPressed()) {
        rot_controller.Reset(getCalculatedPose().Rotation().Radians());
        y_controller.Reset(yDistance);
    }
}

template<class AzimuthMotor, class DriveMotor>
void Swerve<AzimuthMotor, DriveMotor>::analyzeDashboard()
{
    y_controller.SetP(Y_KP);
    y_controller.SetD(Y_KD);

    rot_controller.SetP(ROT_KP);
    rot_controller.SetD(ROT_KD);

    y_controller.SetTolerance(yPosTolerance, yVelTolerance);
    rot_controller.SetTolerance(rotPosTolerance, rotVelTolerance);

    rawEstimator->UpdateWithTime(frc::Timer::GetFPGATimestamp(), getGyro(), getModuleStates());
    calcEstimator->UpdateWithTime(frc::Timer::GetFPGATimestamp(), getGyro(), getModuleStates());

    if (useCarpetGrain)
        calculateCarpetPose();

    // Linear Speed calculations
    xSpeedMPS = units::meters_per_second_t{xSpeed * maxDriveSpeed};
    ySpeedMPS = units::meters_per_second_t{ySpeed * maxDriveSpeed};
    if (frc::DriverStation::GetAlliance() == frc::DriverStation::kRed) {
        xSpeedMPS *= -1.0;
        ySpeedMPS *= -1.0;
    }
    // Rotational Speed calculations
    if (alignToTarget) {
        rot_controller.SetGoal(units::radian_t{targetAngle});
        units::radian_t robotRotation = getCalculatedPose().Rotation().Radians();
        rot_controller.Calculate(robotRotation);
        rotSpeedRPS = units::radians_per_second_t{rot_controller.Calculate(robotRotation)} + rot_controller.GetSetpoint().velocity;
    } 
    else {
        rotSpeedRPS = rotSpeed * maxRotationSpeed;
    }

    joystickVector = Eigen::Vector2d{xSpeed, ySpeed};
    double dotProduct = joystickVector.dot(MAKE_VECTOR(targetAngle));
    joystickVector = dotProduct * MAKE_VECTOR(targetAngle);
    joystickVector *= maxDriveSpeed.value();
    // joystickVector = Eigen::Vector2d{joystickVector[0], joystickVector[1]};

    if (alignToTarget){
        y_controller.SetGoal(0.0_m);
        calculated_y_controller_val = y_controller.Calculate(yDistance, 0.0_m);
        relativeToTagSpeed = units::meters_per_second_t{calculated_y_controller_val} + y_controller.GetSetpoint().velocity;

        pidVector = MAKE_VECTOR(targetAngle - 90_deg) * relativeToTagSpeed.value();
        powerVector = joystickVector + pidVector;
        // powerVector *= dotProduct / fabs(dotProduct);
        xSpeedMPS = units::meters_per_second_t{powerVector[0]};
        ySpeedMPS = units::meters_per_second_t{powerVector[1]};
    }

    
    // Linear Speed calculations
}

template<class AzimuthMotor, class DriveMotor>
void Swerve<AzimuthMotor, DriveMotor>::assignOutputs()
{
    if (rotTest) {
        swerveModules[0]->setAzimuthPosition(frc::Rotation2d(-45_deg));
        swerveModules[1]->setAzimuthPosition(frc::Rotation2d(-135_deg));
        swerveModules[2]->setAzimuthPosition(frc::Rotation2d(-225_deg));
        swerveModules[3]->setAzimuthPosition(frc::Rotation2d(45_deg));
        for (size_t i = 0; i < MODULE_COUNT; i++) {
            swerveModules[i]->setDrivePower(units::volt_t (12));
        }
    } else if (strLineTest){
        for (size_t i = 0; i < MODULE_COUNT; i++) {
            swerveModules[i]->setAzimuthPosition(frc::Rotation2d());
            swerveModules[i]->setDrivePower(units::volt_t (12));
        }
    } else{
        drive(xSpeedMPS, ySpeedMPS, rotSpeedRPS, true);
    }
}

template<class AzimuthMotor, class DriveMotor>
void Swerve<AzimuthMotor, DriveMotor>::drive(
    units::velocity::meters_per_second_t vx_mps,
    units::velocity::meters_per_second_t vy_mps,
    units::angular_velocity::radians_per_second_t omega_radps,
    bool isFOC
)
{
    auto desiredStates = getModuleStates(vx_mps,
                                  vy_mps,
                                  omega_radps,
                                  isFOC);
    setSwerveDesiredState(desiredStates, true);
}

template<class AzimuthMotor, class DriveMotor>
void Swerve<AzimuthMotor, DriveMotor>::driveRobotRelative(frc::ChassisSpeeds speeds) {
    auto desiredStates = getModuleStates(speeds);
    setSwerveDesiredState(desiredStates, false);
}

template<class AzimuthMotor, class DriveMotor>
frc::ChassisSpeeds Swerve<AzimuthMotor, DriveMotor>::getRobotRelativeSpeeds(){
    wpi::array<frc::SwerveModuleState, 4> moduleStates = {
        swerveModules[0]->getState(),
        swerveModules[1]->getState(),
        swerveModules[2]->getState(),
        swerveModules[3]->getState()
    };
    return kinematics->ToChassisSpeeds(moduleStates);
}

template<class AzimuthMotor, class DriveMotor>
void Swerve<AzimuthMotor, DriveMotor>::setSwerveDesiredState(wpi::array<frc::SwerveModuleState, MODULE_COUNT> desiredStates, bool isDriveOpenLoop)
{
    for (size_t i = 0; i < MODULE_COUNT; i++) {
        swerveModules[i]->setDesiredState(desiredStates[i], isDriveOpenLoop);
    }
}

template<class AzimuthMotor, class DriveMotor>
void Swerve<AzimuthMotor, DriveMotor>::resetState()
{
    resetOdometry(frc::Pose2d{0_m, 0_m, 0_rad});
    rotTest = false;
    strLineTest = false;
}

template<class AzimuthMotor, class DriveMotor>
void Swerve<AzimuthMotor, DriveMotor>::resetOdometry(frc::Pose2d pose)
{
    rawEstimator->ResetPosition(getGyro(), getModuleStates(), pose);
    calcEstimator->ResetPosition(getGyro(), getModuleStates(), pose);
}

template<class AzimuthMotor, class DriveMotor>
void Swerve<AzimuthMotor, DriveMotor>::resetEncoders()
{
    for (size_t i = 0; i < swerveModules.size(); i++)
    {
        swerveModules[i]->resetDriveEncoder();
    }
}

template<class AzimuthMotor, class DriveMotor>
void Swerve<AzimuthMotor, DriveMotor>::enableCarpetGrain(double _grainMultiplier, bool _roughTowardsRed)
{
    carpetGrainMultiplier = _grainMultiplier;
    roughTowardsRed = _roughTowardsRed;
    useCarpetGrain = true;
}

template<class AzimuthMotor, class DriveMotor>
void  Swerve<AzimuthMotor, DriveMotor>::calculateCarpetPose()
{
    static frc::Pose2d previousPose;
    frc::Pose2d newPose = calcEstimator->GetEstimatedPosition();
    units::meter_t deltaX = newPose.X() - previousPose.X();
    double factor = 1.0;
    if (roughTowardsRed) {
        factor = deltaX < 0_m ? carpetGrainMultiplier : 1;
    } else {
        factor = deltaX > 0_m ? carpetGrainMultiplier : 1;
    }
    
    calcEstimator->AddVisionMeasurement(
        frc::Pose2d{
            factor * deltaX + previousPose.X(),
            newPose.Y(),
            newPose.Rotation()
        },
        frc::Timer::GetFPGATimestamp(),
        {0.1, 0.1, 0.1}
    );
    previousPose = calcEstimator->GetEstimatedPosition();
}

template<class AzimuthMotor, class DriveMotor>
void Swerve<AzimuthMotor, DriveMotor>::setupGyro(
    int _pigeonCanID,
    const char* _pigeonCanBus,
    units::degree_t mountRoll,
    units::degree_t mountPitch,
    units::degree_t mountYaw
)
{
    pigeon.reset();
    pigeon = std::make_unique<ctre::phoenix6::hardware::Pigeon2>(_pigeonCanID, _pigeonCanBus);
    pigeon->GetConfigurator().Apply(
        ctre::phoenix6::configs::Pigeon2Configuration{}
        .WithMountPose(
            ctre::phoenix6::configs::MountPoseConfigs{}
            .WithMountPoseRoll(mountRoll)
            .WithMountPosePitch(mountPitch)
            .WithMountPoseYaw(mountYaw)
        )
    );
}

template<class AzimuthMotor, class DriveMotor>
void Swerve<AzimuthMotor, DriveMotor>::resetGyro(){
    frc::Pose2d initialPose = getRawPose();
    frc::Pose2d desiredPose = frc::Pose2d(initialPose.X(), initialPose.Y(), frc::Rotation2d(0_deg));
    resetOdometry(desiredPose);
}

template<class AzimuthMotor, class DriveMotor>
frc::Rotation2d Swerve<AzimuthMotor, DriveMotor>::getGyro() {
    if (pigeon)
        return pigeon->GetRotation2d();
    return frc::Rotation2d();
}

template<class AzimuthMotor, class DriveMotor>
frc::Pose2d Swerve<AzimuthMotor, DriveMotor>::getRawPose()
{
    return rawEstimator->GetEstimatedPosition();
}

template<class AzimuthMotor, class DriveMotor>
frc::Pose2d Swerve<AzimuthMotor, DriveMotor>::getCalculatedPose()
{
    return calcEstimator->GetEstimatedPosition();
}

template<class AzimuthMotor, class DriveMotor>
wpi::array<frc::SwerveModulePosition, MODULE_COUNT> Swerve<AzimuthMotor, DriveMotor>::getModuleStates()
{
    wpi::array<frc::SwerveModulePosition, MODULE_COUNT> modulePositions = wpi::array<frc::SwerveModulePosition, MODULE_COUNT>(wpi::empty_array);
    for (size_t i = 0; i < swerveModules.size(); i++) {
        modulePositions[i] = swerveModules[i]->getModulePosition();
    }
    return modulePositions;
}

template<class AzimuthMotor, class DriveMotor>
wpi::array<frc::SwerveModuleState, MODULE_COUNT> Swerve<AzimuthMotor, DriveMotor>::getModuleStates(units::velocity::meters_per_second_t vx_mps,
                                                                  units::velocity::meters_per_second_t vy_mps,
                                                                  units::angular_velocity::radians_per_second_t omega_radps,
                                                                  bool isFOC)
{
    frc::ChassisSpeeds chassisSpeeds = isFOC ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(vx_mps,
                                                                                           vy_mps,
                                                                                           omega_radps,
                                                                                           rawEstimator->GetEstimatedPosition().Rotation())
                                             : frc::ChassisSpeeds{vx_mps, vy_mps, omega_radps};
    return getModuleStates(chassisSpeeds);
}

template<class AzimuthMotor, class DriveMotor>
wpi::array<frc::SwerveModuleState, MODULE_COUNT> Swerve<AzimuthMotor, DriveMotor>::getModuleStates(frc::ChassisSpeeds chassisSpeeds)
{
    auto states = kinematics->ToSwerveModuleStates(chassisSpeeds);
    kinematics->DesaturateWheelSpeeds(&states, maxDriveSpeed);
    return states;
}

template<class AzimuthMotor, class DriveMotor>
void Swerve<AzimuthMotor, DriveMotor>::setAzimuthPositions(frc::Rotation2d rot){
    for (size_t i = 0; i < swerveModules.size(); i++){
        swerveModules[i]->setAzimuthPosition(rot);
    }
}

template<class AzimuthMotor, class DriveMotor>
void Swerve<AzimuthMotor, DriveMotor>::setAzimuthPositions(frc::Rotation2d rot1, frc::Rotation2d rot2, frc::Rotation2d rot3, frc::Rotation2d rot4){
    swerveModules[0]->setAzimuthPosition(rot1);
    swerveModules[1]->setAzimuthPosition(rot2);
    swerveModules[2]->setAzimuthPosition(rot3);
    swerveModules[3]->setAzimuthPosition(rot4);
}

template<class AzimuthMotor, class DriveMotor>
void Swerve<AzimuthMotor, DriveMotor>::setDriveSpeeds(units::meters_per_second_t speed){
    for (size_t i = 0; i < swerveModules.size(); i++){
        swerveModules[i]->setDesiredState(frc::SwerveModuleState{speed, swerveModules[i]->getAzimuthPosition()});
    }
}

template<class AzimuthMotor, class DriveMotor>
void Swerve<AzimuthMotor, DriveMotor>::InitSendable(wpi::SendableBuilder& builder)
{
    builder.SetSmartDashboardType("Subsystem");
    builder.AddDoubleProperty(
        "Commanded X Speed",
        [this] { return xSpeed; },
        nullptr
    );
    builder.AddDoubleProperty(
        "Commanded Y Speed",
        [this] { return ySpeed; },
        nullptr
    );
    builder.AddDoubleProperty(
        "Commanded Rot Speed",
        [this] { return rotSpeed; },
        nullptr
    );
    builder.AddDoubleProperty(
        "Commanded X Speed MPS",
        [this] { return xSpeedMPS.value(); },
        nullptr
    );
    builder.AddDoubleProperty(
        "Commanded Y Speed MPS",
        [this] { return ySpeedMPS.value(); },
        nullptr
    );
    builder.AddDoubleProperty(
        "Commanded Rot Speed MPS",
        [this] { return rotSpeedRPS.value(); },
        nullptr
    );
    builder.AddDoubleProperty(
        "Actual Raw Pose X",
        [this] { return getRawPose().X().template to<double>(); },
        nullptr
    );
    builder.AddDoubleProperty(
        "Actual Raw Pose Y",
        [this] { return getRawPose().Y().template to<double>(); },
        nullptr
    );
    builder.AddDoubleProperty(
        "Actual Raw Pose Theta",
        [this] { return getRawPose().Rotation().Degrees().template to<double>(); },
        nullptr
    );
    builder.AddDoubleArrayProperty(
        "Actual Raw Pose",
        [this] 
        { 
            std::vector<double> pose;
            pose.push_back(getRawPose().X().template to<double>());
            pose.push_back(getRawPose().Y().template to<double>());
            pose.push_back(getRawPose().Rotation().Radians().template to<double>());
            return pose;
        },
        nullptr
    );
    builder.AddDoubleProperty(
        "Actual Calculated Pose X",
        [this] { return getCalculatedPose().X().template to<double>(); },
        nullptr
    );
    builder.AddDoubleProperty(
        "Actual Calculated Pose Y",
        [this] { return getCalculatedPose().Y().template to<double>(); },
        nullptr
    );
    builder.AddDoubleProperty(
        "Actual Calculated Pose Theta",
        [this] { return getCalculatedPose().Rotation().Degrees().template to<double>(); },
        nullptr
    );
    builder.AddDoubleArrayProperty(
        "Actual Calculated Pose",
        [this] 
        { 
            std::vector<double> pose;
            pose.push_back(getCalculatedPose().X().template to<double>());
            pose.push_back(getCalculatedPose().Y().template to<double>());
            pose.push_back(getCalculatedPose().Rotation().Radians().template to<double>());
            return pose;
        },
        nullptr
    );
    builder.AddDoubleProperty(
        "Gyro Pitch",
        [this]
        {
            return pigeon->GetPitch().GetValueAsDouble();
        },
        nullptr
    );
    builder.AddDoubleProperty(
        "Gyro Yaw",
        [this]
        {
            return pigeon->GetYaw().GetValueAsDouble();
        },
        nullptr
    );
    builder.AddDoubleProperty(
        "Gyro Roll",
        [this]
        {
            return pigeon->GetRoll().GetValueAsDouble();
        },
        nullptr
    );
    builder.AddBooleanProperty(
        "Tilted",
        [this]
        {
            double yaw = pigeon->GetYaw().GetValueAsDouble(), roll = pigeon->GetRoll().GetValueAsDouble(); // in degrees
            double elevation = std::fabs(yaw) + std::fabs(roll); // not at all but close enough
            return elevation > 5.0;
        },
        nullptr
    );
    builder.AddDoubleArrayProperty(
        "swerve states",
        [this] 
        { 
            std::vector<double> states;
            states.push_back(swerveModules[0]->getState().angle.Degrees().template to<double>());
            states.push_back(swerveModules[0]->getState().speed.template to<double>());
            states.push_back(swerveModules[1]->getState().angle.Degrees().template to<double>());
            states.push_back(swerveModules[1]->getState().speed.template to<double>());
            states.push_back(swerveModules[2]->getState().angle.Degrees().template to<double>());
            states.push_back(swerveModules[2]->getState().speed.template to<double>());
            states.push_back(swerveModules[3]->getState().angle.Degrees().template to<double>());
            states.push_back(swerveModules[3]->getState().speed.template to<double>());
            return states;
        },
        nullptr
    );
    builder.AddDoubleArrayProperty(
        "Robot Velocities",
        [this]
        {
            std::vector<double> states;
            states.push_back(getRobotRelativeSpeeds().vx.template to<double>());
            states.push_back(getRobotRelativeSpeeds().vy.template to<double>());
            return states;
        },
        nullptr
    );
    builder.AddDoubleProperty(
        "Max Drive Speed MPS",
        [this] {return maxDriveSpeed.value();},
        nullptr
    );
    builder.AddDoubleProperty(
        "Target Angle",
        [this] {return units::radian_t{targetAngle}.to<double>();},
        nullptr
    );
    builder.AddDoubleProperty(
        "Setpoint Velocity",
        [this] {return rot_controller.GetSetpoint().velocity.to<double>();},
        nullptr
    );
    builder.AddDoubleProperty(
        "Position Tolerance",
        [this] {return rot_controller.GetPositionTolerance();},
        nullptr
    );
    builder.AddBooleanProperty(
        "At Goal",
        [this] {return rot_controller.AtGoal();},
        nullptr
    );
    builder.AddDoubleProperty(
        "Y Distance",
        [this] {return yDistance.to<double>();},
        nullptr
    );
    builder.AddDoubleProperty(
        "Feedforward",
        [this] {return feedForward.to<double>();},
        nullptr
    );
    builder.AddDoubleProperty(
        "Calculated y_controller value",
        [this] {return calculated_y_controller_val;},
        nullptr
    );

    builder.AddDoubleArrayProperty(
        "Joystick Vector",
        [this] {return std::vector<double>{joystickVector[0], joystickVector[1]};},
        nullptr
    );
    builder.AddDoubleArrayProperty(
        "Y_Controller PID Vector",
        [this] {return std::vector<double>{pidVector[0], pidVector[1]};},
        nullptr
    );
    builder.AddDoubleArrayProperty(
        "Power Vector",
        [this] {return std::vector<double>{powerVector[0], powerVector[1]};},
        nullptr
    );
    builder.AddDoubleProperty(
        "Rotation Controller Setpoint",
        [this] {return static_cast<double>(rot_controller.GetSetpoint().position());},
        nullptr
    );
    builder.AddDoubleProperty(
        "Y Controller Setpoint",
        [this] {return static_cast<double>(y_controller.GetSetpoint().position());},
        nullptr
    );
    builder.AddDoubleProperty(
        "Y Controller Setpoint Velocity",
        [this] {return static_cast<double>(y_controller.GetSetpoint().velocity());},
        nullptr
    );
    
    builder.AddBooleanProperty(
        "Locking on Target",
        [this] {return alignToTarget;},
        nullptr
    );
}

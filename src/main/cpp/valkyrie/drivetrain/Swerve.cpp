#include "valkyrie/drivetrain/Swerve.h"
#include "valkyrie/controllers/PhoenixController.h"

#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/geometry/Translation2d.h>
#include <frc/DriverStation.h>


#define MODULE_DIFF_XS {1, 1, -1, -1}
#define MODULE_DIFF_YS {1, -1, -1, 1}

const units::hertz_t KP_ROTATE(-90);
const units::hertz_t KD_ROTATE(-30);

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
}

template<class AzimuthMotor, class DriveMotor>
void Swerve<AzimuthMotor, DriveMotor>::analyzeDashboard()
{
    rawEstimator->UpdateWithTime(frc::Timer::GetFPGATimestamp(), getGyro(), getModuleStates());
    calcEstimator->UpdateWithTime(frc::Timer::GetFPGATimestamp(), getGyro(), getModuleStates());

    if (useCarpetGrain){
        calculateCarpetPose();
    }
    /*if (charMode.GetSelected() = "Rot Test") {
        rotTest = true;
    }*/

    // Rotational Speed calculations
    if (lockingToTarget) {
        units::radian_t robotRotation = getCalculatedPose().Rotation().Radians();
        units::radian_t errorAngle = robotRotation - targetAngle;
        units::radian_t error = units::math::fmod(errorAngle + units::radian_t(M_PI), 2 * units::radian_t(M_PI)) - units::radian_t(M_PI);
        static units::radian_t prevError = error;
        rotSpeedRPS = error * KP_ROTATE + (error - prevError) * KD_ROTATE;
        prevError = error;
    } else {
        rotSpeedRPS = rotSpeed * maxRotationSpeed;
    }

    // Linear Speed calculations
    xSpeedMPS = units::meters_per_second_t{xSpeed * maxDriveSpeed};
    ySpeedMPS = units::meters_per_second_t{ySpeed * maxDriveSpeed};
    if (frc::DriverStation::GetAlliance() == frc::DriverStation::kRed) {
        xSpeedMPS *= -1.0;
        ySpeedMPS *= -1.0;
    }
}

template<class AzimuthMotor, class DriveMotor>
void Swerve<AzimuthMotor, DriveMotor>::assignOutputs()
{
    drive(xSpeedMPS, ySpeedMPS, rotSpeedRPS, true);
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
    /*builder.AddBooleanProperty(
        "Rot Test",
        [this] {return rotTest;},
        nullptr
    );*/
}

#pragma once

#include <frc2/command/InstantCommand.h>
#include <ctre/phoenix6/Pigeon2.hpp>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>

#include "valkyrie/BaseSubsystem.h"
#include "valkyrie/drivetrain/SwerveModule.h"
#include "valkyrie/CharMode.h"

#define MODULE_COUNT 4

namespace valor {

template<class AzimuthMotor, class DriveMotor>
class Swerve : public valor::BaseSubsystem
{
public:
    std::string selectedTest;

    Swerve(frc::TimedRobot *_robot,
                const char* _name,
                std::vector<std::pair<AzimuthMotor*, DriveMotor*>> modules,
                units::meter_t module_radius,
                units::meter_t _wheelDiameter);
    ~Swerve();

    void init() override;

    void assessInputs() override;
    void analyzeDashboard() override;
    void assignOutputs() override;
    void resetState() override;

     /**
      * Returns the position of the robot on the field in meters
      * @return the pose of the robot
      */
    frc::Pose2d getRawPose();
    frc::Pose2d getCalculatedPose();

    void setupGyro(int, const char*, units::degree_t, units::degree_t, units::degree_t);
    void resetGyro();
    frc::Rotation2d getGyro();

    /**
     * Reset the robot's position on the field. Any accumulted gyro drift will be noted and
     *   accounted for in subsequent calls to getPoseMeters()
     * @param pose The robot's actual position on the field
     */
    void resetOdometry(frc::Pose2d pose);
    void resetEncoders();

    wpi::array<frc::SwerveModulePosition, MODULE_COUNT> getModuleStates();

    /**
     * Drive the robot with given x, y and rotational velocities using open loop velocity control
     * @param vx_mps the desired x velocity component in meters per second
     * @param vy_mps the desired y velocity component in meters per second
     * @param omega_radps the desired rotational velocity component in radians per second
     * @param isFOC true if driving field oriented
     */
    void drive(
        units::velocity::meters_per_second_t vx_mps,
        units::velocity::meters_per_second_t vy_mps,
        units::angular_velocity::radians_per_second_t omega_radps,
        bool isFOC
    );

    void driveRobotRelative(frc::ChassisSpeeds speeds);

    void InitSendable(wpi::SendableBuilder& builder) override;

protected:
    double xSpeed;
    double ySpeed;
    double rotSpeed;

    units::meters_per_second_t xSpeedMPS;
    units::meters_per_second_t ySpeedMPS;
    units::radians_per_second_t rotSpeedRPS;

    units::meters_per_second_t maxDriveSpeed;
    units::radians_per_second_t maxRotationSpeed;

    std::unique_ptr<ctre::phoenix6::hardware::Pigeon2> pigeon;
    std::unique_ptr<frc::SwerveDriveKinematics<MODULE_COUNT>> kinematics;
    std::unique_ptr<frc::SwerveDrivePoseEstimator<MODULE_COUNT>> rawEstimator;
    std::unique_ptr<frc::SwerveDrivePoseEstimator<MODULE_COUNT>> calcEstimator;

    bool lockingToTarget;
    units::degree_t targetAngle;

    void enableCarpetGrain(double grainMultiplier, bool roughTowardsRed);
    
    frc::ChassisSpeeds getRobotRelativeSpeeds();
    void setSwerveDesiredState(wpi::array<frc::SwerveModuleState, MODULE_COUNT> desiredStates, bool isDriveOpenLoop);

private:

    std::vector<valor::SwerveModule<AzimuthMotor, DriveMotor> *> swerveModules;

    wpi::array<frc::SwerveModuleState, MODULE_COUNT> getModuleStates(units::velocity::meters_per_second_t,
                                                                    units::velocity::meters_per_second_t,
                                                                    units::angular_velocity::radians_per_second_t,
                                                                    bool);
    wpi::array<frc::SwerveModuleState, MODULE_COUNT> getModuleStates(frc::ChassisSpeeds chassisSpeeds);

    double _drivetrain_accel;

    bool useCarpetGrain;
    double carpetGrainMultiplier;
    bool roughTowardsRed;
    void calculateCarpetPose();

    bool charMode;
    bool rotTest;
    bool strLineTest;
    
};

}
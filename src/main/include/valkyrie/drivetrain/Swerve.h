#pragma once

#include <frc2/command/InstantCommand.h>
#include <ctre/phoenix6/Pigeon2.hpp>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>

#include "Eigen/Core"
#include "units/velocity.h"
#include "valkyrie/BaseSubsystem.h"
#include "valkyrie/drivetrain/SwerveModule.h"
#include "valkyrie/CharMode.h"

#define MODULE_COUNT 4

namespace valor {

template<class AzimuthMotor, class DriveMotor>
class Swerve : public valor::BaseSubsystem
{
public:
    CharMode::MODE_OPTIONS selectedTest;

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

    units::meters_per_second velocity;
    units::meters_per_second_t feedForward = 0_mps;

    std::unique_ptr<ctre::phoenix6::hardware::Pigeon2> pigeon;
    std::unique_ptr<frc::SwerveDriveKinematics<MODULE_COUNT>> kinematics;
    std::unique_ptr<frc::SwerveDrivePoseEstimator<MODULE_COUNT>> rawEstimator;
    std::unique_ptr<frc::SwerveDrivePoseEstimator<MODULE_COUNT>> calcEstimator;

    bool lockingToTarget;
    units::degree_t targetAngle;

    bool alignToTarget = false;
    units::meter_t yDistance = 0.0_m;

    void enableCarpetGrain(double grainMultiplier, bool roughTowardsRed);
    
    frc::ChassisSpeeds getRobotRelativeSpeeds();
    void setSwerveDesiredState(wpi::array<frc::SwerveModuleState, MODULE_COUNT> desiredStates, bool isDriveOpenLoop);

private:
    const units::radians_per_second_t MAX_ROTATION_VEL = 16_rad_per_s;
    const units::radians_per_second_squared_t MAX_ROTATION_ACCEL = 12_rad_per_s_sq;
    const units::meters_per_second_t MAX_Y_VEL = 5.5_mps;
    const units::meters_per_second_squared_t MAX_Y_ACCEL = 3_mps_sq;
    double ROT_KP = 9;
    double ROT_KD = 0.25;
    double Y_KP = 0.0;
    double Y_KD = 0;

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

    bool rotTest;
    bool strLineTest;
    
    CharMode charac;

    frc::TrapezoidProfile<units::radian>::Constraints rot_constraints{MAX_ROTATION_VEL, MAX_ROTATION_ACCEL};
    frc::ProfiledPIDController<units::radian> rot_controller{ROT_KP, 0.0, ROT_KD, rot_constraints};

    frc::TrapezoidProfile<units::meter>::Constraints y_constraints{MAX_Y_VEL, MAX_Y_ACCEL};
    frc::ProfiledPIDController<units::meter> y_controller{Y_KP, 0.0, Y_KD, y_constraints};
    double calculated_y_controller_val;
    units::meters_per_second_t relativeToTagSpeed;

    Eigen::Vector2d joystickVector, pidVector, powerVector;
};

}

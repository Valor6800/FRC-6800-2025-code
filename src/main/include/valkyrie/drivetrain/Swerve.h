#pragma once

#include <frc2/command/InstantCommand.h>
#include <ctre/phoenix6/Pigeon2.hpp>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>

#include <frc/controller/ProfiledPIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>

#include <deque>
#include "Eigen/Core"
#include "frc/EigenCore.h"
#include "networktables/StructTopic.h"
#include "units/angle.h"
#include "units/length.h"
#include "units/time.h"
#include "units/velocity.h"
#include "units/acceleration.h"
#include "valkyrie/BaseSubsystem.h"
#include "valkyrie/controllers/PIDF.h"
#include "valkyrie/drivetrain/SwerveModule.h"
#include "valkyrie/CharMode.h"
#include <networktables/StructArrayTopic.h>
#include <networktables/StructTopic.h>
#include <pathplanner/lib/util/swerve/SwerveSetpointGenerator.h>
#include <pathplanner/lib/util/swerve/SwerveSetpoint.h>
#include <pathplanner/lib/config/RobotConfig.h>

#include "Constants.h"

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
    wpi::array<frc::SwerveModuleState, MODULE_COUNT> getModuleStates(frc::ChassisSpeeds chassisSpeeds);
    void updateAngularPosition();

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

    double getSkiddingRatio();
    bool isRobotSkidding();
    
    wpi::array<frc::SwerveModuleState, MODULE_COUNT> getAllModuleStates();
    std::vector<frc::SwerveModuleState> getAllModuleStatesAsVector();
    void updateAngularAcceleration();
    units::angular_acceleration::radians_per_second_squared_t getSmoothedAngularAcceleration();
    double rotationLerping(double);

    virtual bool autoAlignShutOff() {return false;};

    void resetYAlignControllers();
    void resetXAlignControllers();
    void resetRotationAlignControllers();

    bool xAlign = false;
    bool dumbAutoAlign = false;

    void InitSendable(wpi::SendableBuilder& builder) override;

    units::meters_per_second_t yControllerInitialVelocity;
    units::meters_per_second_t xControllerInitialVelocity;

    const units::radians_per_second_t MAX_ROTATION_VEL = 16_rad_per_s;
    const units::radians_per_second_squared_t MAX_ROTATION_ACCEL = 12_rad_per_s_sq;
    const units::meters_per_second_t MAX_Y_VEL = 5.5_mps;
    const units::meters_per_second_squared_t MAX_Y_ACCEL = 1.5_mps_sq;
    const units::meters_per_second_t MAX_X_VEL = 1.5_mps;
    const units::meters_per_second_t MAX_X_VEL_L4 = 1.0_mps;
    const units::meters_per_second_squared_t MAX_X_ACCEL = 2.0_mps_sq;

    const frc::TrapezoidProfile<units::radian>::Constraints rot_constraints{MAX_ROTATION_VEL, MAX_ROTATION_ACCEL};

    const frc::TrapezoidProfile<units::meter>::Constraints y_constraints{MAX_Y_VEL, MAX_Y_ACCEL};
    const frc::TrapezoidProfile<units::meter>::Constraints x_constraints{MAX_X_VEL, MAX_X_ACCEL};
    const frc::TrapezoidProfile<units::meter>::Constraints x_constraints_l4{MAX_X_VEL_L4, MAX_X_ACCEL};

    void setXConstraints(frc::TrapezoidProfile<units::meter>::Constraints constraints);
    frc::TrapezoidProfile<units::meter>::Constraints getXConstraints();

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
    std::unique_ptr<frc::SwerveDrivePoseEstimator<MODULE_COUNT>> alignEstimator;

    bool toast;
    bool lockingToTarget;
    units::degree_t targetAngle;
    void setRotAlignOffset(units::degree_t angle);

    bool yAlign, rotAlign = false;
    units::meter_t yDistance = 0.0_m;
    units::meter_t xDistance = 0.0_m;

    void enableCarpetGrain(double grainMultiplier, bool roughTowardsRed);
    
    frc::ChassisSpeeds getRobotRelativeSpeeds();
    frc::ChassisSpeeds getFieldRelativeSpeeds();
    void setSwerveDesiredState(wpi::array<frc::SwerveModuleState, MODULE_COUNT> desiredStates, bool isDriveOpenLoop);

    double ROT_KP = 4;
    double ROT_KD = 0.1;
    double Y_KP = 0.0;
    double Y_KI = 0.0;
    double Y_KD = 0;
    double X_KP = 0.0;
    double X_KI = 0.0;
    double X_KD = 0;

    units::degree_t rotPosTolerance = 0.0_deg; //1.0_deg;
    units::degrees_per_second_t rotVelTolerance = 0_deg_per_s; //1_deg_per_s;

    units::millimeter_t yPosTolerance = 50_mm;  //20_mm;
    units::millimeter_t xPosTolerance = 50_mm;  //20_mm;
    units::meters_per_second_t yVelTolerance = 0.0_mps; //0.01_mps;
    units::meters_per_second_t xVelTolerance = 0.0_mps; //0.01_mps;
    units::radian_t angularPosition = 0_rad;
    units::meter_t goalAlign = 0.0_m;
    // units::meter_t xGoalAlign = 17.7_in;
    units::meter_t xGoalAlign = 23_in;

    bool yControllerAligned();
    wpi::array<frc::SwerveModuleState, MODULE_COUNT> testModeDesiredStates{wpi::empty_array};

    std::vector<valor::SwerveModule<AzimuthMotor, DriveMotor> *> swerveModules;

    units::degree_t getRotControllerError();
    void transformControllerSpeeds();

    void setPIDx(valor::PIDF pid);
    void setPIDy(valor::PIDF pid);
    
private:

    std::deque<units::angular_acceleration::radians_per_second_squared_t> yawRateBuffer;

    static constexpr size_t ACCEL_BUFFER_SIZE = 10; //need to adjust
    units::angular_velocity::radians_per_second_t lastYawRate = 0_rad_per_s;
    units::angular_acceleration::radians_per_second_squared_t angularAcceleration = 0_rad_per_s_sq;

    wpi::array<frc::SwerveModuleState, MODULE_COUNT> getModuleStates(units::velocity::meters_per_second_t,
                                                                    units::velocity::meters_per_second_t,
                                                                    units::angular_velocity::radians_per_second_t,
                                                                    bool);

    double _drivetrain_accel;

    bool useCarpetGrain;
    double carpetGrainMultiplier;
    bool roughTowardsRed;
    void calculateCarpetPose();

    frc::ChassisSpeeds discretize(frc::ChassisSpeeds speeds);
    frc::Twist2d log(frc::Pose2d transform);

    bool rotTest;
    bool strLineTest;
    
    CharMode charac;

    
    frc::ProfiledPIDController<units::radian> rot_controller{ROT_KP, 0.0, ROT_KD, rot_constraints};


    frc::ProfiledPIDController<units::meter> y_controller{Y_KP, 0.0, Y_KD, y_constraints};
    frc::ProfiledPIDController<units::meter> x_controller{X_KP, 0.0, X_KD, x_constraints};
    double calculated_y_controller_val;
    double calculated_x_controller_val;
    units::meters_per_second_t relativeToTagSpeed;
    units::meters_per_second_t relativeToTagXSpeed;

    Eigen::Vector2d joystickVector, yAlignVector, powerVector, xAlignVector;
    units::angle::degree_t rotAlignOffset;

    nt::StructPublisher<frc::Pose2d> rawPosePublisher, calculatedPosePublisher, alignPosePublisher;
    nt::StructPublisher<frc::ChassisSpeeds> robotVelocitiesPublisher;

    // SwerveSetpointGenerator setpointGenerator;
    // SwerveSetpoint previousSetpoint;
};

}

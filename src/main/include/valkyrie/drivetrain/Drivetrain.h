#pragma once

#include "valkyrie/BaseSubsystem.h"

#include <memory>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/estimator/PoseEstimator.h>
#include <frc2/command/InstantCommand.h>
#include <ctre/phoenix6/Pigeon2.hpp>

namespace valor {

template <typename PoseEstimatorType>
class Drivetrain : public valor::BaseSubsystem
{
public:
    Drivetrain(frc::TimedRobot *_robot, const char* _name);
    ~Drivetrain();

    void init() override;

    void assessInputs() override;
    void analyzeDashboard() override;
    void assignOutputs() override;
    void resetState() override;

    void setupGyro(
        int _pigeonCanID,
        const char* _pigeonCanBus,
        double mountRoll,
        double mountPitch,
        double mountYaw
    );
    void resetGyro();
    frc::Rotation2d getGyro();

    frc::Pose2d getRawPose();
    frc::Pose2d getCalculatedPose();

    /**
     * Reset the robot's position on the field. Any accumulted gyro drift will be noted and
     *   accounted for in subsequent calls to getPoseMeters()
     * @param pose The robot's actual position on the field
     */
    virtual void resetOdometry(frc::Pose2d pose) = 0;
    virtual void resetEncoders() = 0;

    void InitSendable(wpi::SendableBuilder& builder) override;

protected:
    std::unique_ptr<ctre::phoenix6::hardware::Pigeon2> pigeon;
    std::unique_ptr<PoseEstimatorType> rawEstimator;
    std::unique_ptr<PoseEstimatorType> calcEstimator;

private:
    double _drivetrain_accel;
};

}
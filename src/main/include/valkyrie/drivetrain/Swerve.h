#pragma once

#include "valkyrie/drivetrain/Drivetrain.h"

namespace valor {

template <size_t NumModules>
class Swerve : public valor::Drivetrain<SwervePoseEstimator<NumModules>>
{
public:
    Swerve(frc::TimedRobot *_robot);
    ~Swerve();

    void init() override;

    void assessInputs() override;
    void analyzeDashboard() override;
    void assignOutputs() override;
    void resetState() override;

    void resetOdometry(frc::Pose2d pose) override;
    void resetEncoders() override;

    void setXMode();
    frc2::InstantCommand* cmd_XMode();

    void InitSendable(wpi::SendableBuilder& builder) override;

protected:

private:
};

}
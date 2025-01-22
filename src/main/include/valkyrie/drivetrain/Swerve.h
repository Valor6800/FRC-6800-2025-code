#pragma once
#include <networktables/StructArrayTopic.h>
#include <networktables/StructTopic.h>
#include "valkyrie/BaseSubsystem.h"
#include "valkyrie/CharMode.h"
#include "TunerConstants.h"

namespace valor {

class Swerve : public valor::BaseSubsystem
{
public:
    CharMode::MODE_OPTIONS selectedTest;

    Swerve(frc::TimedRobot *_robot,
                std::string _name);
    ~Swerve();

    void init() override;

    void assessInputs() override;
    void analyzeDashboard() override;
    void assignOutputs() override;
    void resetState() override;

    void InitSendable(wpi::SendableBuilder& builder) override;

protected:
    void enableCarpetGrain(double grainMultiplier, bool roughTowardsRed);

    TunerSwerveDrivetrain drivetrain{
        TunerConstants::DrivetrainConstants,
        TunerConstants::FrontLeft,
        TunerConstants::FrontRight,
        TunerConstants::BackLeft,
        TunerConstants::BackRight,
    };

    // Default uses open loop voltage for drive, position for azimuth
    ctre::phoenix6::swerve::requests::FieldCentric fieldCentricRequest;
    ctre::phoenix6::swerve::requests::FieldCentricFacingAngle fieldCentricFacingAngleRequest;

private:
    bool useCarpetGrain;
    double carpetGrainMultiplier;
    bool roughTowardsRed;
    void calculateCarpetPose();

    bool rotTest;
    bool strLineTest;

    nt::StructPublisher<frc::Pose2d> posePublisher;
    nt::StructPublisher<frc::ChassisSpeeds> chassisSpeedsPublisher;
    nt::StructArrayPublisher<frc::SwerveModuleState> currentModuleStatesPublisher;
    nt::StructArrayPublisher<frc::SwerveModuleState> targetModuleStatesPublisher;
    
    CharMode charac;
};

}
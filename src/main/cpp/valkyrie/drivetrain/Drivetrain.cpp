#include "valkyrie/drivetrain/Drivetrain.h"

using namespace valor;

template <typename PoseEstimatorType>
Drivetrain<PoseEstimatorType>::Drivetrain(
    frc::TimedRobot *_robot,
    const char* _name
) : valor::BaseSubsystem(_robot, _name)
{

}

template <typename PoseEstimatorType>
Drivetrain<PoseEstimatorType>::~Drivetrain()
{

}

template <typename PoseEstimatorType>
void Drivetrain<PoseEstimatorType>::setupGyro(
    int _pigeonCanID,
    const char* _pigeonCanBus,
    double mountRoll,
    double mountPitch,
    double mountYaw
)
{
    pigeon = std::make_unique<ctre::phoenix6::hardware::Pigeon2>(_pigeonCanID, _pigeonCanBus);
    pigeon->GetConfigurator().Apply(
        ctre::phoenix6::configs::Pigeon2Configuration{}
        .WithMountPose(
            ctre::phoenix6::configs::MountPoseConfigs{}
            .WithMountPosePitch(mountPitch)
            .WithMountPoseRoll(mountRoll)
            .WithMountPoseYaw(mountYaw)
        )
    );
}

template <typename PoseEstimatorType>
void Drivetrain<PoseEstimatorType>::init()
{
    resetState();
}

template <typename PoseEstimatorType>
frc::Pose2d Drivetrain<PoseEstimatorType>::getRawPose()
{
    return rawEstimator->GetEstimatedPosition();
}

template <typename PoseEstimatorType>
frc::Pose2d Drivetrain<PoseEstimatorType>::getCalculatedPose()
{
    return calcEstimator->GetEstimatedPosition();
}

template <typename PoseEstimatorType>
void Drivetrain<PoseEstimatorType>::resetGyro(){
    frc::Pose2d initialPose = getPose_m();
    frc::Pose2d desiredPose = frc::Pose2d(initialPose.X(), initialPose.Y(), frc::Rotation2d(0_deg));
    resetOdometry(desiredPose);
}

template <typename PoseEstimatorType>
frc::Rotation2d Drivetrain<PoseEstimatorType>::getGyro() {
    return pigeon.GetRotation2d();
}

template <typename PoseEstimatorType>
void Drivetrain<PoseEstimatorType>::assessInputs()
{
    if (!driverGamepad || !driverGamepad->IsConnected())
        return;

    if (driverGamepad->GetBackButtonPressed()) {
        resetGyro();
    }
}

template <typename PoseEstimatorType>
void Drivetrain<PoseEstimatorType>::analyzeDashboard()
{
    if (pigeon)
        this._drivetrain_accel = sqrtf(
            powf(pigeon.GetAccelerationX().GetValue().to<double>(), 2) +
            powf(pigeon.GetAccelerationY().GetValue().to<double>(), 2)
        );
}

template <typename PoseEstimatorType>
void Drivetrain<PoseEstimatorType>::assignOutputs()
{

}

template <typename PoseEstimatorType>
void Drivetrain<PoseEstimatorType>::resetState()
{
    resetOdometry(frc::Pose2d{0_m, 0_m, 0_rad});
}

template <typename PoseEstimatorType>
void Drivetrain<PoseEstimatorType>::InitSendable(wpi::SendableBuilder& builder)
{
    // ~60 kg bot -> 600 N, 5 measurements * 20ms = .1s,
    // impulse = .1 * 600 = 60 Joules
    builder.AddBooleanProperty(
        "Bonk!",
        [this] {
            return units::acceleration::meters_per_second_squared_t{this._drivetrain_accel} > 20.0_mps_sq; 
        },
        nullptr
    );
}
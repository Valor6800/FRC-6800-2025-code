#include "valkyrie/sensors/GamePieceSensor.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Pose3d.h"
#include "units/angle.h"
#include "units/length.h"
#include "wpi/detail/value_t.h"
#include <cmath>
#include <vector>
// clang-format off
using namespace valor;

GamePieceSensor::GamePieceSensor(frc::TimedRobot* robot, const char *name, frc::Pose3d _cameraPose, frc::SwerveDrivePoseEstimator<4>* estimator) : valor::VisionSensor(robot, name, _cameraPose),
    estimator(estimator)
{
    setGetter([this](){return getGlobalPose();});
}

frc::Pose3d GamePieceSensor::getGlobalPose() {


    if (!hasTarget()) return frc::Pose3d();

    updateRelative();

    if (estimator == nullptr) return frc::Pose3d();

    units::radian_t robotTheta = estimator->GetEstimatedPosition().Rotation().Radians(); //Get robot theta from pigeon

    units::meter_t robotX = estimator->GetEstimatedPosition().X(); //Get robot X from odom
    units::meter_t robotY = estimator->GetEstimatedPosition().Y(); //Get robot Y from odom

    units::meter_t globalX = -sin(robotTheta.value()) * relativePoseFromCenter.y + cos(robotTheta.value()) * relativePoseFromCenter.x + robotX;
    units::meter_t globalY = cos(robotTheta.value()) * relativePoseFromCenter.y + sin(robotTheta.value()) * relativePoseFromCenter.x + robotY;

    return frc::Pose3d(
       globalY,
       globalX,
       0_m,
       frc::Rotation3d()
    );
}
// clang-format on
void GamePieceSensor::updateRelative() {
    if (!hasTarget()) return;
    
    relativePoseFromCamera.x = (cameraPose.Z() - 2.0_in) * tan((M_PI/2.0) + cameraPose.Rotation().Y().value() + units::angle::degree_t(ty).convert<units::angle::radian>().value());
    relativePoseFromCamera.y = -relativePoseFromCamera.x / tan((M_PI/2.0) - cameraPose.Rotation().Z().value() + units::angle::degree_t(tx).convert<units::angle::radian>().value());

    updateRelativeToCenter();
}

void GamePieceSensor::updateRelativeToCenter() {
    relativePoseFromCenter.x = relativePoseFromCamera.x * cos(cameraPose.Rotation().Z().value()) - relativePoseFromCamera.y * sin(cameraPose.Rotation().Z().value()) + cameraPose.X();
    relativePoseFromCenter.y = relativePoseFromCamera.x * sin(cameraPose.Rotation().Z().value()) + relativePoseFromCamera.y * cos(cameraPose.Rotation().Z().value()) + cameraPose.Y();

}

void GamePieceSensor::InitSendable(wpi::SendableBuilder& builder) {
    builder.SetSmartDashboardType("Subsystem");

    builder.AddDoubleArrayProperty(
        "gamePiece Pos",
        [this] {
            frc::Pose2d gamePos = getSensor().ToPose2d();
            std::vector<double> gamePosVector{
                gamePos.X().value(),
                gamePos.Y().value(),
            };
            return gamePosVector;
        },
        nullptr
    );
    builder.AddDoubleArrayProperty(
        "Relative Pos From Camera",
        [this]
        {
            return std::vector<double>{
                relativePoseFromCamera.x.value(), // Fd
                relativePoseFromCamera.y.value() // lt and rt
            };
        },
        nullptr
    );
    builder.AddDoubleArrayProperty(
        "Relative Pos From Center",
        [this]
        {
            return std::vector<double>{
                relativePoseFromCenter.x.value(), // Fd
                relativePoseFromCenter.y.value() // lt and rt
            };
        },
        nullptr
    );
}

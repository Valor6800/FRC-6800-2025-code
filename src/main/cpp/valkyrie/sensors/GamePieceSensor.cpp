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

    units::meter_t globalX = -sin(robotTheta.to<double>()) * relativePoseFromCenter.y + cos(robotTheta.to<double>()) * relativePoseFromCenter.x + robotX;
    units::meter_t globalY = cos(robotTheta.to<double>()) * relativePoseFromCenter.y + sin(robotTheta.to<double>()) * relativePoseFromCenter.x + robotY;

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

    relativePoseFromCamera.x = cameraPose.Z() * tanf((M_PI/2.0) + cameraPose.Rotation().Y().to<double>() + units::angle::degree_t(tx).convert<units::angle::radian>().to<double>());
    relativePoseFromCamera.y = relativePoseFromCamera.x / (tanf((M_PI/2.0) - cameraPose.Rotation().Z().to<double>() + units::angle::degree_t(ty).convert<units::angle::radian>().to<double>()));

    updateRelativeToCenter();
}

void GamePieceSensor::updateRelativeToCenter() {
    relativePoseFromCenter.x = relativePoseFromCamera.x * cos(cameraPose.Rotation().Z().to<double>()) - relativePoseFromCamera.y * sin(cameraPose.Rotation().Z().to<double>()) + cameraPose.X();
    relativePoseFromCenter.y = relativePoseFromCamera.x * sin(cameraPose.Rotation().Z().to<double>()) + relativePoseFromCamera.y * cos(cameraPose.Rotation().Z().to<double>()) + cameraPose.Y();

}

void GamePieceSensor::InitSendable(wpi::SendableBuilder& builder) {
    builder.SetSmartDashboardType("Subsystem");

    builder.AddDoubleArrayProperty(
        "gamePiece Pos",
        [this] {
            frc::Pose2d gamePos = getSensor().ToPose2d();
            std::vector<double> gamePosVector{
                gamePos.Y().to<double>(),
                gamePos.X().to<double>(),
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
                relativePoseFromCamera.x.to<double>(), // Fd
                relativePoseFromCamera.y.to<double>() // lt and rt
            };
        },
        nullptr
    );
    builder.AddDoubleArrayProperty(
        "Relative Pos From Center",
        [this]
        {
            return std::vector<double>{
                relativePoseFromCenter.x.to<double>(), // Fd
                relativePoseFromCenter.y.to<double>() // lt and rt
            };
        },
        nullptr
    );
}

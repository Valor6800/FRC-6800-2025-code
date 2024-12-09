#include "valkyrie/sensors/GamePieceSensor.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Pose3d.h"
#include "units/angle.h"
#include "units/length.h"
#include "wpi/detail/value_t.h"
#include <cmath>
#include <iostream>
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

    units::meter_t globalX = robotX + relativePoseFromCenter.x * cos(robotTheta.value()) + relativePoseFromCenter.y * sin(robotTheta.value());
    units::meter_t globalY = robotY - relativePoseFromCenter.x * sin(robotTheta.value()) + relativePoseFromCenter.y * cos(robotTheta.value());

    return frc::Pose3d(
       globalX,
       globalY,
       0_m,
       frc::Rotation3d()
    );
}
// clang-format on
#define ANGLE_ERR 1.972405059
void GamePieceSensor::updateRelative() {
    if (!hasTarget()) return;
    double angle = cameraPose.Rotation().Y().value() + (units::angle::degree_t(ty).convert<units::angle::radian>().value() + (ANGLE_ERR * (M_PI/180.0)) /* * 1.36586656 */) /* + (((ty * M_PI) / 180.0) / 3.0) */; 
    std::cout << "\n\n\t\t" << angle << "\n\n";
    relativePoseFromCamera.x = cameraPose.Z() * tan((M_PI / 2.0) + angle);
    relativePoseFromCamera.y = relativePoseFromCamera.x / tan((M_PI/2.0) - cameraPose.Rotation().Z().value() + units::angle::degree_t(tx).convert<units::angle::radian>().value());

    updateRelativeToCenter();
}

void GamePieceSensor::updateRelativeToCenter() {
    relativePoseFromCenter.x = relativePoseFromCamera.x * cos(cameraPose.Rotation().Z().value()) - relativePoseFromCamera.y * sin(cameraPose.Rotation().Z().value()) + cameraPose.X();
    relativePoseFromCenter.y = relativePoseFromCamera.x * sin(cameraPose.Rotation().Z().value()) - relativePoseFromCamera.y * cos(cameraPose.Rotation().Z().value()) + cameraPose.Y();
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

    VisionSensor::InitSendable(builder);
}

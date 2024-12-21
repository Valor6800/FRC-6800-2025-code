#include "valkyrie/sensors/GamePieceSensor.h"
#include "Eigen/Core"
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

    //TODO: Make these constants
    rollMatrix = RotationMatrix{
        {cos(cameraPose.Rotation().X().value()), -sin(cameraPose.Rotation().X().value()), 0},
        {sin(cameraPose.Rotation().X().value()), cos(cameraPose.Rotation().X().value()), 0},
        {0, 0, 1}
    };

    //TODO: Make these constants
    yawMatrix = RotationMatrix{
        {1, 0, 0},
        {0, cos(cameraPose.Rotation().Z().value()), -sin(cameraPose.Rotation().Z().value())},
        {0, sin(cameraPose.Rotation().Z().value()), cos(cameraPose.Rotation().Z().value())}
    };
    
    //TODO: Make these constants
    pitchMatrix = RotationMatrix{
        {cos(cameraPose.Rotation().Y().value()), 0, sin(cameraPose.Rotation().Y().value())},
        {0, 1, 0},
        {-sin(cameraPose.Rotation().Y().value()), 0, cos(cameraPose.Rotation().Y().value())}
    };

    fullRotationMatrix = rollMatrix * yawMatrix * pitchMatrix;

    //TODO: Make these constants
    cameraTranslation = Eigen::Matrix<double, 3, 1>{
        {cameraPose.X().value()},
        {cameraPose.Y().value()},
        {cameraPose.Z().value()}
    };
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
void GamePieceSensor::updateRelative() {

    //TODO: Make this an input and class memmber constant
    Eigen::Matrix<double, 3, 3> intrinsicMatrix{
        {1038.543, 0, 609.345},
        {0, 1037.537, 469.070},
        {0, 0, 1}
    };

    //TODO: Do this on init
    Eigen::Matrix<double, 3, 4> CameraPoseMatrix{
        {fullRotationMatrix.row(0)[0], fullRotationMatrix.row(0)[1], fullRotationMatrix.row(0)[2], cameraTranslation[0]},
        {fullRotationMatrix.row(1)[0], fullRotationMatrix.row(1)[1], fullRotationMatrix.row(1)[2], cameraTranslation[1]},
        {fullRotationMatrix.row(2)[0], fullRotationMatrix.row(2)[1], fullRotationMatrix.row(2)[2], cameraTranslation[2]},
    };

    auto inverseHomography = Eigen::Inverse(intrinsicMatrix * CameraPoseMatrix);
    
    if (!hasTarget()) return;
    
    //TODO: make this suitable with enums for different types of limelights
    Eigen::Matrix<double, 4, 1> pixelCoord {
        {(62.5 - (2 * tx)) / (2 * 62.5)},
        {(48.9 - (2 * ty)) / (2 * 48.9)},
        {1}
    };

    Eigen::Matrix<double, 3, 1> gamePieceMatrix = inverseHomography * pixelCoord;

    relativePoseFromCamera.x = units::meter_t{gamePieceMatrix.col(0)[0]};
    relativePoseFromCamera.y = units::meter_t{gamePieceMatrix.col(0)[1]};

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

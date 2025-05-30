#include "valkyrie/sensors/AprilTagsSensor.h"
#include "frc/geometry/Rotation3d.h"
#include "units/angle.h"
#include <array>
#include "units/length.h"
#include "units/time.h"
#include "units/velocity.h"
#include <chrono>
#include <frc/Timer.h>
#include <cmath>
#include <limits>
#include <math.h>
#include <span>
#include <vector>

#define OUTLIER_EDGE 4.0f //meters
#define DP 0.1f // 0.1
#define VP 0.2f // 

using namespace valor;

AprilTagsSensor::AprilTagsSensor(frc::TimedRobot* robot, const char *name, frc::Pose3d _cameraPose) : valor::VisionSensor(robot, name, _cameraPose) {
    setGetter([this](){return getGlobalPose();});
    megaTag2Pose = frc::Pose3d();
    dp = DP;
    vp = VP;
}

frc::Pose3d AprilTagsSensor::getGlobalPose() {
    if (!hasTarget()) return frc::Pose3d();
    std::vector<double> botPose = limeTable->GetNumberArray("botpose_wpiblue", std::span<double>());
    
    std::vector<double> botToTargetPose = limeTable->GetNumberArray("botpose_targetspace", std::span<const double>());

    if (botPose.size() == 0 || botToTargetPose.size() == 0) {distance = 0_m;return frc::Pose3d();}

    if (botToTargetPose.size() == 6) distance = units::meter_t(sqrtf(powf(botToTargetPose[0], 2) + powf(botToTargetPose[1], 2) + powf(botToTargetPose[2], 2)));

    return frc::Pose3d(
        (units::meter_t) botPose[0],
        (units::meter_t) botPose[1],
        (units::meter_t) botPose[2],
        frc::Rotation3d(
            (units::degree_t) botPose[3],
            (units::degree_t) botPose[4],
            (units::degree_t) botPose[5]
        )
    );
}

frc::Pose3d AprilTagsSensor::getPoseFromAprilTag() {
    if (!hasTarget()) return frc::Pose3d();

    std::vector<double> botToTargetPose = limeTable->GetNumberArray("botpose_targetspace", std::span<const double>());

    if (botToTargetPose.size() == 0) return frc::Pose3d();

    return frc::Pose3d(
        (units::meter_t) botToTargetPose[0],
        (units::meter_t) botToTargetPose[1],
        (units::meter_t) botToTargetPose[2],
        frc::Rotation3d(
            (units::degree_t) botToTargetPose[3],
            (units::degree_t) botToTargetPose[4],
            (units::degree_t) botToTargetPose[5]
        )
    ); 
}
void AprilTagsSensor::applyVisionMeasurement(
    frc::SwerveDrivePoseEstimator<4> *estimator,
    units::velocity::meters_per_second_t speed,
    units::angular_velocity::radians_per_second_t angular_velocity,
    double doubtX,
    double doubtY, 
    double doubtRot
) {
    if (!hasTarget()) return;
    dp = DP;
    vp = VP;
    double avp = 2;
 
    //std::vector<double> botToTargetPose = limeTable->GetNumberArray("botpose_targetspace", std::span<const double>());
    //if (botToTargetPose.size() == 6) distance = units::meter_t(sqrtf(powf(botToTargetPose[0], 2) + powf(botToTargetPose[1], 2)));
    //else distance = 0_m; return;

    double newDoubtX = doubtX + (distance.to<double>() * dp) + (vp * speed.to<double>());
    double newDoubtY = doubtY + (distance.to<double>() * dp) + (vp * speed.to<double>());
    double newDoubtRot = doubtRot + (distance.to<double>() * dp) + (vp * speed.to<double>()) + (avp * fabs(angular_velocity.value()));

    if (distance >= normalVisionOutlier) return;

    units::millisecond_t totalLatency = getTotalLatency();

    frc::Rotation2d newAngle = estimator->GetEstimatedPosition().Rotation();
    if (distance < 1_m) {
        newAngle = currState.ToPose2d().Rotation();
    }

    if (units::math::fabs(angular_velocity) > 1_tps/30.0) {
        newDoubtRot = std::numeric_limits<double>::max();
    }

    frc::Pose2d tGone = frc::Pose2d{
        currState.ToPose2d().X(),
        currState.ToPose2d().Y(),
        newAngle
    };
    limeTable->PutNumber("new doubt x", newDoubtX);
    limeTable->PutNumber("new doubt y", newDoubtY);
    limeTable->PutNumber("new doubt rot", newDoubtRot);

    if (tGone.X() == 0.0_m || tGone.Y() == 0.0_m)
        return;

    estimator->AddVisionMeasurement(
        tGone,  
        frc::Timer::GetFPGATimestamp() - totalLatency,
        {newDoubtX, newDoubtY, newDoubtRot}
    );
}

int AprilTagsSensor::getTagID(){
    if (!hasTarget()) return -1;

    return limeTable->GetNumber("tid", -1);
}

frc::Pose3d AprilTagsSensor::getMegaTagPose2(AprilTagsSensor::Orientation orient) {

    if (!hasTarget()) return frc::Pose3d();

    limeTable->PutNumberArray(
        "robot_orientation_set",
        std::vector<double>{
            orient.yaw.to<double>(),
            orient.yawVel.to<double>(),
            0,
            0,
            0,
            0
        }
    );

    std::vector<double> mt2Array = limeTable->GetNumberArray("botpose_orb_wpiblue", std::span<double>());

    if (mt2Array.size() == 0) return frc::Pose3d();

    megaTag2Pose = frc::Pose3d(
        units::meter_t{mt2Array[0]},
        units::meter_t{mt2Array[1]},
        units::meter_t{mt2Array[2]},
        frc::Rotation3d(
            units::degree_t{mt2Array[3]},
            units::degree_t{mt2Array[4]},
            units::degree_t{mt2Array[5]}
        )
    );
    return megaTag2Pose;

}

frc::Pose3d AprilTagsSensor::getTargetToBotPose() {
    if (!hasTarget()) return frc::Pose3d();

    std::vector<double> targetToBot = limeTable->GetNumberArray("targetpose_robotspace", std::span<double>());

    if (targetToBot.size() == 0) return frc::Pose3d();

    return frc::Pose3d(
        units::meter_t{targetToBot[0]},
        units::meter_t{targetToBot[1]},
        units::meter_t{targetToBot[2]},
        frc::Rotation3d(
            units::degree_t{targetToBot[3]},
            units::degree_t{targetToBot[4]},
            units::degree_t{targetToBot[5]}
        )
    );

}

frc::Pose3d AprilTagsSensor::get_botpose_targetspace() {
    if (!hasTarget()) return frc::Pose3d();

    std::vector<double> targetToBot = limeTable->GetNumberArray("botpose_targetspace", std::span<double>());

    if (targetToBot.size() == 0) return frc::Pose3d();

    return frc::Pose3d(
        units::meter_t{targetToBot[0]},
        units::meter_t{targetToBot[1]},
        units::meter_t{targetToBot[2]},
        frc::Rotation3d(
            units::degree_t{targetToBot[3]},
            units::degree_t{targetToBot[4]},
            units::degree_t{targetToBot[5]}
        )
    );

}

void AprilTagsSensor::InitSendable(wpi::SendableBuilder& builder) {
    builder.SetSmartDashboardType("Subsystem");

    VisionSensor::InitSendable(builder);

    builder.AddDoubleArrayProperty(
        "globalPos",
        [this]
        {
            std::vector<double> botPose;
            botPose.push_back(currState.X().to<double>());
            botPose.push_back(currState.Y().to<double>());
            botPose.push_back(currState.ToPose2d().Rotation().Degrees().to<double>());
            return botPose;
        },
        nullptr
    );
    builder.AddDoubleArrayProperty(
        "globalPosMegaTag2",
        [this]
        {
            std::vector<double> botPose;
            botPose.push_back(megaTag2Pose.X().to<double>());
            botPose.push_back(megaTag2Pose.Y().to<double>());
            botPose.push_back(megaTag2Pose.ToPose2d().Rotation().Degrees().to<double>());
            return botPose;
        },
        nullptr
    );
    builder.AddDoubleArrayProperty(
        "cameraPose",
        [this]
        {
            return limeTable->GetNumberArray(
                "camerapose_robotspace",
                std::array<double, 6> {
                    cameraPose.X().to<double>(),
                    cameraPose.Y().to<double>(),
                    cameraPose.Z().to<double>(),
                    cameraPose.Rotation().X().convert<units::degree>().to<double>(),
                    cameraPose.Rotation().Y().convert<units::degree>().to<double>(),
                    cameraPose.Rotation().Z().convert<units::degree>().to<double>()
                }
            );
        },
        nullptr
    );
    builder.AddDoubleProperty("distanceFromTarget", [this] {return distance.to<double>();}, nullptr);
    builder.AddDoubleProperty("Vision acceptance outlier", [this] {return normalVisionOutlier.to<double>();}, nullptr);
    builder.AddDoubleProperty("tid", [this] {return getTagID();}, nullptr);
}

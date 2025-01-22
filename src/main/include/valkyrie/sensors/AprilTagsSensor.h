#pragma once

#include "frc/estimator/PoseEstimator.h"
#include "frc/estimator/SwerveDrivePoseEstimator.h"
#include "frc/geometry/Pose3d.h"
#include "frc/geometry/Rotation3d.h"
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/length.h"
#include "units/velocity.h"
#include "valkyrie/sensors/VisionSensor.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"

namespace valor
{
    class AprilTagsSensor : public VisionSensor {
        public:
            /**
             * @brief Constructor for AprilTagsSensor
            *
            * @param _robot Pass in the Robot reference so the calculate can be auto-scheduled
            * @param _name The name of the specific sensor for logging and reporting
            */
            AprilTagsSensor(frc::TimedRobot* robot, const char *name, frc::Pose3d _cameraPose);

            int getTagID();

            void InitSendable(wpi::SendableBuilder& builder) override;

            units::meter_t normalVisionOutlier = 5.5_m;
            void applyVisionMeasurement(
                frc::SwerveDrivePoseEstimator<4> *estimator,
                units::velocity::meters_per_second_t speed,
                bool accept = true,
                double doubtX = 1, double doubtY = 1, double doubtRot = 1
            );
            frc::Pose3d getPoseFromAprilTag();
            frc::Pose3d getTargetToBotPose();
            frc::Pose3d get_botpose_targetspace();

            struct Orientation {
                units::degree_t yaw, pitch, roll;
                units::degrees_per_second_t yawVel, pitchVel, rollVel;
            };

            frc::Pose3d getMegaTagPose2(Orientation orient);

        private:
            frc::Pose3d getGlobalPose() override;
            frc::Pose3d megaTag2Pose = frc::Pose3d();
            units::meter_t distance{0_m};
            double dp, vp;
            
    };
} // namespace valor

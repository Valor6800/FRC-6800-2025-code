#include "frc/estimator/SwerveDrivePoseEstimator.h"
#include "units/length.h"
#include "valkyrie/sensors/VisionSensor.h"

namespace valor {


class GamePieceSensor : public valor::VisionSensor {
    public:

        GamePieceSensor(frc::TimedRobot* robot, const char *name, frc::Pose3d _cameraPose);
        GamePieceSensor(frc::TimedRobot* robot, const char *name, frc::Pose3d _cameraPose, frc::SwerveDrivePoseEstimator<4>* estimator);
           
        void InitSendable(wpi::SendableBuilder& builder) override;
        
        void setEstimator(frc::SwerveDrivePoseEstimator<4>* e);

        struct {units::meter_t x, y = 0_m;} relativePoseFromCamera, relativePoseFromCenter;
        
    private:

        frc::Pose3d getGlobalPose() override;
        void updateRelative();
        void updateRelativeToCenter();

        frc::SwerveDrivePoseEstimator<4>* estimator = nullptr;
};
}

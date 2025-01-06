#include "Eigen/Core"
#include "frc/estimator/SwerveDrivePoseEstimator.h"
#include "units/length.h"
#include "valkyrie/sensors/VisionSensor.h"

namespace valor {

typedef Eigen::Matrix<double, 3, 3> RotationMatrix;

//TODO: Take input for the camera type
//TODO: Take input for the camera's intrinsic matrix
class GamePieceSensor : public valor::VisionSensor {
    public:
        GamePieceSensor(frc::TimedRobot* robot, const char *name, frc::Pose3d _cameraPose, frc::SwerveDrivePoseEstimator<4>* estimator);
           
        void InitSendable(wpi::SendableBuilder& builder) override;
        
        struct {units::meter_t x, y = 0_m;} relativePoseFromCamera, relativePoseFromCenter;
        
    private:

        frc::Pose3d getGlobalPose() override;
        void updateRelative();
        void updateRelativeToCenter();

        RotationMatrix rollMatrix, yawMatrix, pitchMatrix, fullRotationMatrix;
        Eigen::Matrix<double, 3, 1> cameraTranslation;
        Eigen::Matrix3d homographyMatrix;
        frc::SwerveDrivePoseEstimator<4>* estimator;
};
}

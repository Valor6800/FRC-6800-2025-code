#include <functional>
#include <deque>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <frc/geometry/Pose2d.h>

class PoseTracker {
    public:
        PoseTracker(int bufferSize=30);
        units::meters_per_second_squared_t getAverageAcceleration();
        units::meters_per_second_t getAverageVelocity();
        units::degrees_per_second_t getAverageAngularVelocity();
        void addReading(frc::Pose2d pose, units::second_t timestamp);
    private:
        void resetBuffers(uint count);
        std::deque<std::pair<frc::Pose2d, units::second_t> > poseBuffer;
        std::deque<std::pair<units::meters_per_second_t, units::second_t>> velBuffer;
        std::deque<std::pair<units::meters_per_second_squared_t, units::second_t>> accelBuffer;
        std::deque<std::pair<units::degrees_per_second_t, units::second_t>> angVelBuffer;
        double bufferSize;
};

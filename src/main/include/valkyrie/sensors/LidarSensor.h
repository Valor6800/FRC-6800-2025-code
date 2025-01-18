#pragma once

#include "valkyrie/sensors/DebounceSensor.h"
#include <frc/TimedRobot.h>
#include <units/length.h>

namespace valor {

/**
 * @brief Specific implementation of the Lidar Sensor for Grapple Robotics LidarCAN device
 */
template <class T>
class LidarSensor : public DebounceSensor
{
public:

    LidarSensor(frc::TimedRobot *_robot, const char* name);
    
    void reset();

    void InitSendable(wpi::SendableBuilder& builder) override;

    void setGetter(std::function<T()> _lambda);

    T getMaxDistance();
    void setMaxDistance(T);

private:
    T maxDistance;
    T currentDistance;
};
}

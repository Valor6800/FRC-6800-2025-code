#pragma once

#include "valkyrie/sensors/BaseSensor.h"
#include <frc/TimedRobot.h>
#include <units/length.h>

namespace valor {

/**
 * @brief Specific implementation of the Lidar Sensor for Grapple Robotics LidarCAN device
 */
template <class T>
class LidarSensor : public BaseSensor<T>
{
public:

    /**
     * @brief Construct a new Valor Lidar Sensor object
     * 
    * Usage:
    * \code {.cpp}
     * valor::LidarSensor lidarDevice = valor::LidarSensor(robot, "LidarDevice", 23);
     * T distance = lidarDevice.getLatestSensorData();
    * \endcode
     * 
     * @param _robot Pass in the Robot reference so the calculate method can be auto-scheduled
     */
    LidarSensor(frc::TimedRobot *_robot, const char* name);
    
    virtual void reset();

    virtual void setGetter(std::function<T()> _lambda);

    void InitSendable(wpi::SendableBuilder& builder) override;

    T getMaxDistance();
    void setMaxDistance(T);

private:
    void calculate();
    T maxDistance;

};
}

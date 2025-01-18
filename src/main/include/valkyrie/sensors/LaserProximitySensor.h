#pragma once

#include "valkyrie/sensors/LidarSensor.h"
#include "valkyrie/sensors/DebounceSensor.h"
#include <units/length.h>

namespace valor {

/**
 * @brief Combines Lidar distance measurements and debounce logic.
 */
template <class T>
class LaserProximitySensor : public LidarSensor<T>, public DebounceSensor {
public:
    /**
     * @brief Constructor for LaserProximitySensor.
     * 
     * @param _robot Pointer to the robot instance.
     * @param name Name of the sensor.
     */
    LaserProximitySensor(frc::TimedRobot* _robot, const char* name);

    /**
     * @brief Set the threshold distance for debounce triggering.
     * 
     * @param threshold Distance in millimeters.
     */
    void setThresholdDistance(T threshold);

    /**
     * @brief Reset both the lidar and debounce states.
     */
    void reset() override;

    /**
     * @brief Set the getter for the lidar sensor.
     * 
     * @param getter Lambda function to fetch lidar distance.
     */
    void setGetter(std::function<T()> getter) override;

    /**
     * @brief Retrieve the latest lidar distance measurement.
     * 
     * @return Latest distance in millimeters.
     */
    T getLidarData() const;

private:
    T thresholdDistance;
};

}

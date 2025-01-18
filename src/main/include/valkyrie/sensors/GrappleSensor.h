#pragma once

#include "valkyrie/sensors/LidarSensor.h"
#include <frc/TimedRobot.h>
#include <units/length.h>
#include <grpl/LaserCan.h>

namespace valor {

/**
 * @brief Specific implementation of the Lidar Sensor for Grapple Robotics LidarCAN device
 */
class GrappleSensor : public LidarSensor<units::millimeter_t>
{
public:

    GrappleSensor(frc::TimedRobot *_robot, const char* name, int canId);

private:
    grpl::LaserCan *device;

};
}

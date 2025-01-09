#pragma once

#include "valkyrie/sensors/BaseSensor.h"
#include <frc/TimedRobot.h>
#include <units/length.h>
#include <grpl/LaserCan.h>

namespace valor {

/**
 * @brief Specific implementation of the Lidar Sensor for Grapple Robotics LidarCAN device
 */
class GrappleLidarSensor : public BaseSensor<units::length::millimeter_t>
{
public:

    /**
     * @brief Construct a new Valor Lidar Sensor object
     * 
    * Usage:
    * \code {.cpp}
     * valor::GrappleLidarSensor lidarDevice = valor::GrappleLidarSensor(robot, "GrappleLidarDevice", 23);
     * units::millimeter_t distance = lidarDevice.getLatestSensorData();
    * \endcode
     * 
     * @param _robot Pass in the Robot reference so the calculate method can be auto-scheduled
     */
    GrappleLidarSensor(frc::TimedRobot *_robot, const char* name, int canId);
    
    void reset();

    void InitSendable(wpi::SendableBuilder& builder) override;

    units::length::millimeter_t getLidarDistance();

private:

    void calculate();

    grpl::LaserCan *device;

};
}

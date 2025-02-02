#include "valkyrie/sensors/LaserProximitySensor.h"

using namespace valor;

template class valor::LaserProximitySensor<units::millimeter_t>;

template <class T>
LaserProximitySensor<T>::LaserProximitySensor(frc::TimedRobot* _robot, const char* name)
    : LidarSensor<T>(_robot, name),
    DebounceSensor(_robot, name),
    thresholdDistance(T{0})
{

}

template <class T>
void LaserProximitySensor<T>::setThresholdDistance(T threshold)
{
    thresholdDistance = threshold;
}

template <class T>
void LaserProximitySensor<T>::reset()
{
    // Reset both base classes
    LidarSensor<T>::reset();
    DebounceSensor::reset();
}

template <class T>
void LaserProximitySensor<T>::setGetter(std::function<T()> getter)
{
    // Set the getter for the lidar sensor
    LidarSensor<T>::setGetter(getter);

    // Configure the debounce sensor to use the lidar distance threshold
    DebounceSensor::setGetter([this]() {
        return LidarSensor<T>::getLatestSensorData() < thresholdDistance;
    });
}

template <class T>
T LaserProximitySensor<T>::getLidarData() {
    return LidarSensor<T>::getLatestSensorData();
}

template <class T>
bool LaserProximitySensor<T>::isTriggered() {
    return DebounceSensor::getLatestSensorData();
}

#include "valkyrie/sensors/LidarSensor.h"

using namespace valor;

template class valor::LidarSensor<units::millimeter_t>;
template class valor::LidarSensor<units::meter_t>;

#define MAX_RANGE 1.2f

template <class T>
LidarSensor<T>::LidarSensor(frc::TimedRobot *_robot, const char *_name) :
    DebounceSensor(_robot, _name),
    maxDistance(MAX_RANGE)
{
    wpi::SendableRegistry::AddLW(this, "LidarSensor", _name);
    reset();
}

template <class T>
void LidarSensor<T>::reset()
{
    DebounceSensor::reset();
    currentDistance = T{-1};
}

template <class T>
void LidarSensor<T>::setMaxDistance(T _maxDistance)
{
    this->maxDistance = _maxDistance;
}

template <class T>
T LidarSensor<T>::getMaxDistance()
{
    return this->maxDistance;
}

template <class T>
void LidarSensor<T>::setGetter(std::function<T()> _lambda)
{
    DebounceSensor::setGetter([this] {
        return true;
    });
}

template <class T>
void LidarSensor<T>::InitSendable(wpi::SendableBuilder& builder)
{
    builder.SetSmartDashboardType("Subsystem");
    builder.AddDoubleProperty(
        "Current State", 
        [this] { return currentDistance.template to<double>(); },
        nullptr);
}

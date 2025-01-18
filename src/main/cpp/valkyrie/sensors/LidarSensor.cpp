#include "valkyrie/sensors/LidarSensor.h"

using namespace valor;

#define MAX_RANGE 1.2f

template <class T>
LidarSensor<T>::LidarSensor(frc::TimedRobot *_robot, const char *_name) :
    BaseSensor<T>(_robot, _name),
    maxDistance(MAX_RANGE)
{
    wpi::SendableRegistry::AddLW(this, "LidarSensor", _name);

    reset();
}

template <class T>
void LidarSensor<T>::reset()
{
    BaseSensor<T>::prevState = T{0};
    BaseSensor<T>::currState = BaseSensor<T>::prevState;
}

template <class T>
void LidarSensor<T>::setGetter(std::function<T()> _lambda)
{
    BaseSensor<T>::setGetter(_lambda);
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
void LidarSensor<T>::calculate()
{
    BaseSensor<T>::prevState = BaseSensor<T>::currState;
    T latestUpdate = BaseSensor<T>::getSensor();
    if (T{this->maxDistance} > latestUpdate && latestUpdate > T{0})
        BaseSensor<T>::currState = latestUpdate;
}

template <class T>
void LidarSensor<T>::InitSendable(wpi::SendableBuilder& builder)
{
    builder.SetSmartDashboardType("Subsystem");
    builder.AddDoubleProperty(
        "Previous State", 
        [this] { return BaseSensor<T>::prevState.template to<double>(); },
        nullptr);
    builder.AddDoubleProperty(
        "Current State", 
        [this] { return BaseSensor<T>::currState.template to<double>(); },
        nullptr);
}

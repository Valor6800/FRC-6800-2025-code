#include "valkyrie/sensors/DebounceSensor.h"

using namespace valor;

DebounceSensor::DebounceSensor(frc::TimedRobot *_robot, const char *_name) : BaseSensor(_robot, _name)
{
    wpi::SendableRegistry::AddLW(this, "DebounceSensor", sensorName);
    reset();
}

void DebounceSensor::reset()
{
    prevState = false;
    currState = false;
}

void DebounceSensor::setEdgeCallback(std::function<void()> _lambda)
{
    edge = _lambda;
}

void DebounceSensor::setRisingEdgeCallback(std::function<void()> _lambda)
{
    risingEdge = _lambda;
}

void DebounceSensor::setFallingEdgeCallback(std::function<void()> _lambda)
{
    fallingEdge = _lambda;
}

void DebounceSensor::calculate()
{
    prevState = currState;
    currState = getSensor();
    if (currState != prevState && edge)
        edge();
    if (currState && !prevState && risingEdge)
        risingEdge();
    if (!currState && prevState && fallingEdge)
        fallingEdge();
}

void DebounceSensor::InitSendable(wpi::SendableBuilder& builder)
{
    builder.SetSmartDashboardType("Subsystem");
    builder.AddBooleanProperty(
        "Previous State", 
        [this] { return prevState; },
        nullptr);
    builder.AddBooleanProperty(
        "Current State", 
        [this] { return currState; },
        nullptr);
}

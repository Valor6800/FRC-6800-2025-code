#include "valkyrie/sensors/GrappleLidarSensor.h"

using namespace valor;

GrappleLidarSensor::GrappleLidarSensor(frc::TimedRobot *_robot, const char *_name, int _canId) :
    BaseSensor(_robot, _name),
    device(new grpl::LaserCan(_canId))
{
    wpi::SendableRegistry::AddLW(this, "GrappleLidarSensor", sensorName);

    device->set_ranging_mode(grpl::LaserCanRangingMode::Long);
    device->set_timing_budget(grpl::LaserCanTimingBudget::TB20ms);
    device->set_roi(grpl::LaserCanROI{ 8, 8, 16, 16 });

    setGetter([this](){
        std::optional<grpl::LaserCanMeasurement> measurement = device->get_measurement();
        if (measurement.has_value() && measurement.value().status == grpl::LASERCAN_STATUS_VALID_MEASUREMENT) {
            return units::length::millimeter_t{static_cast<double>(measurement.value().distance_mm)};
        }
        return units::length::millimeter_t{-1};
    });

    reset();
}

void GrappleLidarSensor::reset()
{
    prevState = units::length::millimeter_t{0};
    currState = prevState;
}

void GrappleLidarSensor::calculate()
{
    prevState = currState;
    units::millimeter_t latestUpdate = getSensor();
    if (latestUpdate > units::millimeter_t{0})
        currState = latestUpdate;
}

void GrappleLidarSensor::InitSendable(wpi::SendableBuilder& builder)
{
    builder.SetSmartDashboardType("Subsystem");
    builder.AddBooleanProperty(
        "Previous State", 
        [this] { return prevState.to<double>(); },
        nullptr);
    builder.AddBooleanProperty(
        "Current State", 
        [this] { return currState.to<double>(); },
        nullptr);
}

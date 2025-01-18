#include "valkyrie/sensors/GrappleSensor.h"

using namespace valor;

GrappleSensor::GrappleSensor(frc::TimedRobot *_robot, const char *_name, int _canId) :
    LidarSensor(_robot, _name),
    device(new grpl::LaserCan(_canId))
{
    device->set_ranging_mode(grpl::LaserCanRangingMode::Short);
    device->set_timing_budget(grpl::LaserCanTimingBudget::TB50ms);
    device->set_roi(grpl::LaserCanROI{ 8, 8, 16, 16 });

    LidarSensor<units::millimeter_t>::setGetter([this](){
        std::optional<grpl::LaserCanMeasurement> measurement = device->get_measurement();
        if (measurement.has_value() && measurement.value().status == grpl::LASERCAN_STATUS_VALID_MEASUREMENT) {
            return units::length::millimeter_t{static_cast<double>(measurement.value().distance_mm)};
        }
        return units::length::millimeter_t{-1};
    });
}

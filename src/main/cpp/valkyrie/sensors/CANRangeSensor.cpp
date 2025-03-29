#include "valkyrie/sensors/CANRangeSensor.h"
using namespace valor;

CANrangeSensor::CANrangeSensor(frc::TimedRobot *_robot, const char *name, int deviceId, std::string canbus, units::millimeter_t defaultDistance) :
    LaserProximitySensor<units::millimeter_t>(_robot, name),
    device(new ctre::phoenix6::hardware::CANrange(deviceId, canbus))
{
    device->ClearStickyFaults();
    ctre::phoenix6::configs::CANrangeConfiguration config;
    config.ToFParams.UpdateFrequency = 50_Hz;
    config.ToFParams.UpdateMode = ctre::phoenix6::signals::UpdateModeValue::ShortRange100Hz;
    config.ProximityParams.ProximityThreshold = 1_cm;
    config.ProximityParams.ProximityHysteresis = 0.5_cm;
    config.FovParams.FOVRangeX = 15_deg;
    config.FovParams.FOVRangeY = 15_deg;
    device->GetConfigurator().Apply(config);
    this->defaultDistance = defaultDistance;

    LaserProximitySensor<units::millimeter_t>::setGetter(
        [this] () {
            units::millimeter_t measurement = device->GetDistance().GetValue();
            if(!isFaulting() && isConnected()){
                return measurement;
            }
            return this->defaultDistance;
        }
    );

    reset();
}

units::millimeter_t valor::CANrangeSensor::getDefaultDistance(){
    return this->defaultDistance;
}

bool valor::CANrangeSensor::isConnected() {
    auto status = device->GetDistance().Refresh().GetStatus();
    return status.IsOK();
}

bool CANrangeSensor::isFaulting(){
    return (device->GetFault_BootDuringEnable().GetValue() || 
            device->GetFault_Hardware().GetValue() || 
            device->GetFault_Undervoltage().GetValue() || 
            device->GetFault_UnlicensedFeatureInUse().GetValue());
}

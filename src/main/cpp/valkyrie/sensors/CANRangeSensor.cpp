#include "valkyrie/sensors/CANRangeSensor.h"

using namespace valor;

CANrangeSensor::CANrangeSensor(frc::TimedRobot *_robot, const char *name, int deviceId, std::string canbus) :
    LaserProximitySensor<units::millimeter_t>(_robot, name),
    device(new ctre::phoenix6::hardware::CANrange(deviceId, canbus))
{
    device->ClearStickyFaults();
    ctre::phoenix6::configs::CANrangeConfiguration config;
    config.ToFParams.UpdateFrequency = 50_Hz;
    config.ToFParams.UpdateMode = ctre::phoenix6::signals::UpdateModeValue::ShortRange100Hz;
    config.ProximityParams.ProximityThreshold = 1_cm;
    config.ProximityParams.ProximityHysteresis = 0.5_cm;
    device->GetConfigurator().Apply(config);

    LaserProximitySensor<units::millimeter_t>::setGetter(
        [this] () {
            units::millimeter_t measurement = device->GetDistance().GetValue();
            if(!isFaulting()){
                return measurement;
            }
            return -1_mm;
        }
    );

    reset();
}

bool CANrangeSensor::isFaulting(){
    return (device->GetFault_BootDuringEnable().GetValue() || 
            device->GetFault_Hardware().GetValue() || 
            device->GetFault_Undervoltage().GetValue() || 
            device->GetFault_UnlicensedFeatureInUse().GetValue());
}

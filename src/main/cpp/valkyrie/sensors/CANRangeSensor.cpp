#include "valkyrie/sensors/CANRangeSensor.h"

using namespace valor;

CANrangeSensor::CANrangeSensor(frc::TimedRobot *_robot, const char *name, int deviceId, std::string canbus) :
    LaserProximitySensor<units::millimeter_t>(_robot, name),
    device(new ctre::phoenix6::hardware::CANrange(deviceId, canbus))
{
    device->ClearStickyFaults();

    LaserProximitySensor<units::millimeter_t>::setGetter(
        [this] () {
            units::millimeter_t measurement = device->GetDistance().GetValue();
            if(!isFaulting()){
                return measurement;
            }
            return measurement;
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

bool CANrangeSensor::isDetected(){
    return device->GetIsDetected().GetValue();
}
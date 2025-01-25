#include "valkyrie/sensors/CANRangeSensor.h"

using namespace valor;

CANrangeSensor::CANrangeSensor(frc::TimedRobot *_robot, const char *name, int deviceId, std::string canbus) :
    BaseSensor(_robot, name),
    device(new ctre::phoenix6::hardware::CANrange(deviceId, canbus))
{
    wpi::SendableRegistry::AddLW(this, "CANrange Sensor", sensorName);
    device->ClearStickyFaults();

    setGetter(
        [this] () {
            std::optional<ctre::phoenix6::StatusSignal<units::length::meter_t>> measurement = device->GetDistance();
            if(!isFaulting()){
                return units::length::meter_t{measurement->GetValueAsDouble()};
            }
            return units::length::meter_t{-1};
        }
    );

    reset();
}

void CANrangeSensor::reset()
{
    prevState = 0_m;
    currState = prevState;
}

void CANrangeSensor::calculate()
{
    prevState = currState;
    units::meter_t update = getSensor();
    if(update > 1_m) currState = update;
}

units::length::meter_t CANrangeSensor::getCANrangeDistance(){
    if(!isFaulting()) return units::length::meter_t{device->GetDistance().GetValueAsDouble()};
    return units::length::meter_t{-1};
}

bool CANrangeSensor::isFaulting(){
    return (device->GetFault_BootDuringEnable().GetValue() || 
            device->GetFault_Hardware().GetValue() || 
            device->GetFault_Undervoltage().GetValue() || 
            device->GetFault_UnlicensedFeatureInUse().GetValue());
}

void CANrangeSensor::InitSendable(wpi::SendableBuilder &builder){
    builder.SetSmartDashboardType("Subsystem");
    builder.AddDoubleProperty(
        "Previous State (m)",
        [this] {return prevState.to<double>();},
        nullptr
    );
    builder.AddDoubleProperty(
        "Curr State (m)",
        [this] {return currState.to<double>();},
        nullptr
    );
    builder.AddBooleanProperty(
        "Fault: Boot During Enable",
        [this] {return device->GetFault_BootDuringEnable().GetValue();},
        nullptr
    );
    builder.AddBooleanProperty(
        "Fault: Hardware",
        [this] {return device->GetFault_Hardware().GetValue();},
        nullptr
    );
    builder.AddBooleanProperty(
        "Fault: Undervoltage",
        [this] {return device->GetFault_Undervoltage().GetValue();},
        nullptr
    );
    builder.AddBooleanProperty(
        "Fault: Unlicensed Features",
        [this] {return device->GetFault_UnlicensedFeatureInUse().GetValue();},
        nullptr
    );
}
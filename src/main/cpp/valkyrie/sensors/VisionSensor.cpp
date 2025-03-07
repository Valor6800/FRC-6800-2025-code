#include "valkyrie/sensors/VisionSensor.h"
#include "networktables/NetworkTableInstance.h"
#include "units/angle.h"
#include "units/temperature.h"
#include "units/time.h"
#include "valkyrie/sensors/BaseSensor.h"
#include "wpi/array.h"
#include "wpi/sendable/SendableBuilder.h"
#include <array>
#include <cstddef>
#include <span>
#include <vector>

using namespace valor;

VisionSensor::VisionSensor(frc::TimedRobot* robot, const char *name, frc::Pose3d _cameraPose) : BaseSensor(robot, name),
            cameraPose(_cameraPose),
            limeTable(nt::NetworkTableInstance::GetDefault().GetTable(name))
{
    wpi::SendableRegistry::AddLW(this, "VisionSensor", sensorName);
    reset();
    setCameraPose(cameraPose);
}

void VisionSensor::setCameraPose(frc::Pose3d camPose){
    if (limeTable == nullptr) return;
    double x = camPose.X().to<double>();
    double y = -camPose.Y().to<double>();
    double z = camPose.Z().to<double>();
    double roll = camPose.Rotation().X().convert<units::angle::degrees>().to<double>();
    double pitch = camPose.Rotation().Y().convert<units::angle::degrees>().to<double>();
    double yaw = camPose.Rotation().Z().convert<units::angle::degrees>().to<double>();

    std::array<double, 6> camPosition = std::array<double, 6>{x, y, z, roll, pitch, yaw};

    limeTable->PutNumberArray("camerapose_robotspace_set", camPosition); 
}

const char* VisionSensor::getName() {
    return BaseSensor::sensorName;
}

void VisionSensor::reset() {
    currState = frc::Pose3d();
    tx = 0;
    tv = 0;
    ty = 0;
    pipe = 0;
}

void VisionSensor::setPipe(PipeLines _pipe) {
    if (limeTable == nullptr) return;
    limeTable->PutNumber("pipeline", static_cast<int>(_pipe));
}

bool VisionSensor::hasTarget() {
    return limeTable != nullptr && tv == 1;
}

units::millisecond_t VisionSensor::getTotalLatency() {
    return (units::millisecond_t) (limeTable->GetNumber("cl", 0.0) + limeTable->GetNumber("tl", 0.0));
}


units::velocity::meters_per_second_t VisionSensor::getError(int pipe, double kPLimeLight) {
    if (limeTable != nullptr && hasTarget()) {
        double normalizedTx = tx / KLIMELIGHT;
        return units::velocity::meters_per_second_t(((std::fabs(normalizedTx) <= 1 ? normalizedTx : std::copysignf(1.0, normalizedTx) ) * kPLimeLight));
    }
    return units::velocity::meters_per_second_t{0};
}

void VisionSensor::calculate(){
    prevState = currState;

    if (limeTable == nullptr) {
        reset();
        return;
    }

    tv = limeTable->GetNumber("tv", 0.0);
    tx = limeTable->GetNumber("tx", 0.0);
    ty = limeTable->GetNumber("ty", 0.0);
    pipe = limeTable->GetNumber("pipeline", 0);

    std::vector<double> hw = limeTable->GetNumberArray(
        "hw",
        std::array<double, 4>()
    );

    fps = hw[0];
    cpuTemp = units::celsius_t{hw[1]};
    ramUsage = hw[2];
    temp = units::celsius_t{hw[3]};
    
    currState = getSensor();
    
}

void VisionSensor::InitSendable(wpi::SendableBuilder& builder) {
    builder.AddDoubleProperty("totalLatency", [this] {return getTotalLatency().to<double>();}, nullptr);
    builder.AddBooleanProperty("hasTarget", [this]{ return hasTarget();}, nullptr);
}

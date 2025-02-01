#pragma once

#include "networktables/NetworkTableInstance.h"
#include "units/temperature.h"
#include "units/time.h"
#include "units/velocity.h"
#include "valkyrie/sensors/BaseSensor.h"
#include <memory>
#include <networktables/NetworkTable.h>
#include <frc/geometry/Pose3d.h>

namespace valor{

#define KLIMELIGHT -29.8f // degrees; TODO: Make Modular

class VisionSensor : public valor::BaseSensor<frc::Pose3d> {
    public:

        /**
        * @brief Top level constructor for Base Vision Sensor
        *
        * @param robot Top level robot object to parse out smart dashboard
        * @param name HostName of the Limelight
        * @param cameraPose Physical 3D Position of the Limelights **Lens**
        */
        VisionSensor(frc::TimedRobot* robot, const char *name, frc::Pose3d _cameraPose);
        //std::string getPipeType(); //Later

        enum PipeLines{
            PIPELINE_0,
            PIPELINE_1,
            PIPELINE_2,
            PIPELINE_3,
            PIPELINE_4,
            PIPELINE_5,
        };

        void reset() override;

        void setCameraPose(frc::Pose3d camPose);

        void setPipe(PipeLines _pipe);

        bool hasTarget();
        
        units::velocity::meters_per_second_t getError(int pipe, double kPLimeLight=0.7);

        virtual void InitSendable(wpi::SendableBuilder& builder) override = 0;

        const char* getName();

        

    protected:
        double tx, ty, tv, fps, ramUsage;
        int pipe;
        units::celsius_t cpuTemp, temp;
        

        units::millisecond_t getTotalLatency();
        virtual frc::Pose3d getGlobalPose() = 0;

        frc::Pose3d cameraPose;
        std::shared_ptr<nt::NetworkTable> limeTable;

        void calculate() override;
        
};

}

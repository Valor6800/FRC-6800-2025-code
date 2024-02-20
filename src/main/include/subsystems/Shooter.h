# pragma once 

#include "units/angular_velocity.h"
#include "valkyrie/BaseSubsystem.h"
#include "valkyrie/controllers/NeoController.h"
#include "valkyrie/controllers/PhoenixController.h"
#include "Constants.h"
#include <vector>
#include "valkyrie/controllers/PIDF.h"
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/DigitalOutput.h>

#include <frc2/command/FunctionalCommand.h>
#include <unordered_map>
#include "valkyrie/Gamepad.h"

class Shooter : public valor::BaseSubsystem
{
public:

    Shooter(frc::TimedRobot *robot);

    void resetState();

    void init();

    void assessInputs();
    void analyzeDashboard();
    void assignOutputs();

    units::degree_t calculatePivotAngle();

    void calculateRootsT();
    void bisectionTheorem();
    
    void InitSendable(wpi::SendableBuilder& builder);

    enum FLYWHEEL_STATE
    {
        NOT_SHOOTING,
        SPOOLED,
        SHOOTING
    };

    enum PIVOT_STATE
    {
        DISABLED,
        SUBWOOFER,
        PODIUM,
        WING,
        TRACKING,
        TESTING
    };

    struct x
    {
        PIVOT_STATE pivotState;
        FLYWHEEL_STATE flywheelState;

        units::angular_velocity::revolutions_per_minute_t leftFlywheelTargetVelocity;
        units::angular_velocity::revolutions_per_minute_t rightFlywheelTargetVelocity;
        units::degree_t pivotAngle;
        units::degree_t calculatingPivotingAngle;

        double manualPivotAngle;
        double testingAngle;
    } state;

private:
    valor::PhoenixController* pivotMotors;

    valor::NeoController leftFlywheelMotor;
    valor::NeoController rightFlywheelMotor;
};

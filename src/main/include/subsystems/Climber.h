#pragma once

#include "units/angular_velocity.h"
#include "valkyrie/BaseSubsystem.h"
#include "valkyrie/controllers/PhoenixController.h"
#include "Constants.h"
#include "valkyrie/controllers/PIDF.h"
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "valkyrie/Gamepad.h"


class Climber : public valor::BaseSubsystem
{
public:

    Climber(frc::TimedRobot *robot);

    ~Climber();

    void init();
    void resetState();
    void assessInputs();
    void analyzeDashboard();
    void assignOutputs();

    void setClimbPID();

    void InitSendable(wpi::SendableBuilder& builder);

    enum CLIMB_STATE
    {
        
    };

    struct x
    {
        CLIMB_STATE climbState;

    }state;


private:
    valor::PhoenixController *climbMotor;
};


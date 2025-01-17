#include "subsystems/Climber.h"
#include <iostream>
#include <math.h>

#include "valkyrie/controllers/NeutralMode.h"
#include "Constants.h"

#include <pathplanner/lib/auto/NamedCommands.h>
#include <frc/DriverStation.h>

#define FORWARD_LIMIT 

#define CLIMB_MAX_SPEED 50.0f


Climber::Climber(frc::TimedRobot *_robot) : valor::BaseSubsystem(_robot, "Climber"),
    climbMotor(nullptr)
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
    table->PutBoolean("Climber overriding leds", false);
}

Climber::~Climber()
{

}

void Climber::resetState()
{

}

void Climber::init()
{
    
}

void Climber::assessInputs()
{
    
}

void Climber::analyzeDashboard()
{

}

void Climber::assignOutputs()
{
    
}


void Climber::InitSendable(wpi::SendableBuilder& builder)
{
    builder.SetSmartDashboardType("Subsystem");

}



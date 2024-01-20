#pragma once

#include "valkyrie/BaseSubsystem.h"
#include "Constants.h"
#include <vector>
#include "valkyrie/controllers/NeoController.h"
#include "valkyrie/controllers/PIDF.h"
#include "valkyrie/sensors/DebounceSensor.h"

#include "frc/DigitalInput.h"

class Intake : public valor::BaseSubsystem
{
public:
    valor::NeoController RollerMotor;
    valor::NeoController ActivationMotor;

    frc::DigitalInput* beam;
    valor::DebounceSensor debounce;

    Intake(frc::TimedRobot *robot, frc::DigitalInput *_beamBreak);

    ~Intake();

    void resetState();
    void init();

    void assessInputs();
    void analyzeDashboard();
    void assignOutputs();

    double getOTBRollerSpeed();

    void InitSendable(wpi::SendableBuilder& builder);

    enum Activation_State
    {
        DEPOLOYED,
        STOWED
    };

    enum Intake_State
    {
        INTAKING,
        STAGNANT,
        OUTTAKE
    };

    enum Detection_State
    {
        NOTE_DETECTED,
        NOTE_NOTDETECTED
    };

    struct x
    {
        Activation_State activation;
        Intake_State intake;
        Detection_State detection;

    } state;

private:

    double IntakeRotMaxSpeed;
    double dropDownMaxSpeed;
};
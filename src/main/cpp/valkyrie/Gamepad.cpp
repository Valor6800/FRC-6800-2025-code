#include "valkyrie/Gamepad.h"

#include <cmath>

#define DPAD_UP 0
#define DPAD_DOWN 180
#define DPAD_RIGHT 90
#define DPAD_LEFT 270

#define DEADBAND_X 0.15f
#define DEADBAND_Y 0.1f
#define DEADBAND_TRIGGER 0.05f

using namespace valor;

Gamepad::Gamepad(int id) :
    frc::XboxController(id),
    deadbandX(DEADBAND_X),
    deadbandY(DEADBAND_Y)
{
}

void Gamepad::setDeadbandX(double deadband)
{
    deadbandX = deadband;
}

void Gamepad::setDeadbandY(double deadband)
{
    deadbandY = deadband;
}

double Gamepad::deadband(double input, double deadband, int polynomial)
{
    return std::fabs(input) > deadband ? copysign(std::pow(std::abs(input), polynomial), input) : 0;
}

double Gamepad::leftStickX(int polynomial)
{
    return -deadband(GetLeftX(), deadbandX, polynomial);
}

bool Gamepad::leftStickXActive(int polynomial)
{
    return leftStickX(polynomial) != 0;
}

double Gamepad::leftStickY(int polynomial)
{
    return -deadband(GetLeftY(), deadbandY, polynomial);
}

bool Gamepad::leftStickYActive(int polynomial)
{
    return leftStickY(polynomial) != 0;
}

double Gamepad::rightStickX(int polynomial)
{
    return -deadband(GetRightX(), deadbandX, polynomial);
}

bool Gamepad::rightStickXActive(int polynomial)
{
    return rightStickX(polynomial) != 0;
}

double Gamepad::rightStickY(int polynomial)
{
    return -deadband(GetRightY(), deadbandY, polynomial);
}

bool Gamepad::rightStickYActive(int polynomial)
{
    return rightStickY(polynomial) != 0;
}

double Gamepad::leftTrigger()
{
    return GetLeftTriggerAxis() > DEADBAND_TRIGGER ? GetLeftTriggerAxis() : 0;
}

bool Gamepad::leftTriggerActive()
{
    return leftTrigger() != 0;
}

double Gamepad::rightTrigger()
{
    return GetRightTriggerAxis() > DEADBAND_TRIGGER ? GetRightTriggerAxis() : 0;
}

bool Gamepad::rightTriggerActive()
{
    return rightTrigger() != 0;
}

bool Gamepad::DPadUp()
{
    return GetPOV() == DPAD_UP;
}
bool Gamepad::DPadDown()
{
    return GetPOV() == DPAD_DOWN;
}
bool Gamepad::DPadLeft()
{
    return GetPOV() == DPAD_LEFT;
}
bool Gamepad::DPadRight()
{
    return GetPOV() == DPAD_RIGHT;
}
bool Gamepad::GetLeftStickButton()
{
    return GetLeftStickButton();
}
bool Gamepad::GetRightStickButton()
{
    return GetRightStickButton();
}

void Gamepad::setRumble(bool turnOn, double intensity)
{
    SetRumble(RumbleType::kBothRumble, turnOn ? intensity: 0.0);
}
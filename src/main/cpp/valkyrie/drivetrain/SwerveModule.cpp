#include "valkyrie/drivetrain/SwerveModule.h"
#include "valkyrie/controllers/PhoenixController.h"
#include "valkyrie/controllers/NeoController.h"
#include "Constants.h"
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include <iostream>
#include <string>

#include <frc/RobotController.h>

const units::meters_per_second_t DRIVE_DEADBAND(0.05);
#define MAG_ENCODER_TICKS_PER_REV 4096.0f

using namespace valor;

SwerveModule::SwerveModule(BaseController* _azimuthMotor,
                           BaseController* _driveMotor,
                           frc::Translation2d _wheelLocation,
                           units::meter_t wheelDiameter) :
    azimuthMotor(_azimuthMotor),
    driveMotor(_driveMotor),
    wheelConversion(M_PI * wheelDiameter / 1_tr)
{
    if (_wheelLocation.X() > 0_m && _wheelLocation.Y() > 0_m) wheelIdx = 0;
    else if (_wheelLocation.X() > 0_m && _wheelLocation.Y() < 0_m) wheelIdx = 1;
    else if (_wheelLocation.X() < 0_m && _wheelLocation.Y() > 0_m) wheelIdx = 3;
    else wheelIdx = 2;

    wpi::SendableRegistry::AddLW(this, "SwerveModule", "Module " + std::to_string(wheelIdx));
}

frc::Rotation2d SwerveModule::getAzimuthPosition()
{
    return frc::Rotation2d{azimuthMotor->getPosition()};
}

units::meter_t SwerveModule::getDrivePosition()
{
    return driveMotor->getPosition() * wheelConversion;
}

units::meters_per_second_t SwerveModule::getDriveSpeed()
{
    return driveMotor->getSpeed() * wheelConversion;
}

units::meters_per_second_t SwerveModule::getMaxDriveSpeed()
{
    return driveMotor->getMaxMechSpeed() * wheelConversion;
}

frc::SwerveModulePosition SwerveModule::getModulePosition()
{
    return { getDrivePosition(), getAzimuthPosition()};
}

frc::SwerveModuleState SwerveModule::getState()
{
    return frc::SwerveModuleState{getDriveSpeed(), getAzimuthPosition()};
}

void SwerveModule::setDesiredState(frc::SwerveModuleState _desiredState, bool isDriveOpenLoop)
{
    // Deadband
    if (_desiredState.speed < DRIVE_DEADBAND) {
        setDriveOpenLoop(0_mps);
        return;
    }

    // Get current angle, optimize drive state
    frc::Rotation2d currentAngle = getAzimuthPosition();
    _desiredState.Optimize(currentAngle);

    // Output optimized rotation and speed
    setAzimuthPosition(_desiredState.angle);
    if (isDriveOpenLoop)
        setDriveOpenLoop(_desiredState.speed);
    else
        setDriveClosedLoop(_desiredState.speed);
    desiredState = _desiredState;
}

void SwerveModule::resetDriveEncoder()
{
    driveMotor->reset();
}

// The angle coming in is an optimized angle. No further calcs should be done on 'angle'
void SwerveModule::setAzimuthPosition(frc::Rotation2d desiredAngle)
{
    azimuthMotor->setPosition(units::turn_t{desiredAngle.Radians().value() / (2.0 * M_PI)});
}

void SwerveModule::setAzimuthPower(units::volt_t voltage)
{
    azimuthMotor->setPower(voltage);
}

void SwerveModule::setDrivePower(units::volt_t voltage)
{
    driveMotor->setPower(voltage);
}

void SwerveModule::setDriveOpenLoop(units::meters_per_second_t mps)
{
    driveMotor->setPower(mps / getMaxDriveSpeed() * units::volt_t{12});
}

void SwerveModule::setDriveClosedLoop(units::meters_per_second_t speed)
{
    auto outSpeed = units::turns_per_second_t{speed.value() / wheelConversion.value()};
    driveMotor->setSpeed(outSpeed);
}

frc::Translation2d SwerveModule::convertSwerveStateToVelocityVector(frc::SwerveModuleState state)
{
    return frc::Translation2d{units::meter_t{state.speed.to<double>()}, state.angle};
}

void SwerveModule::InitSendable(wpi::SendableBuilder& builder)
{
    builder.SetSmartDashboardType("Subsystem");
    builder.AddDoubleProperty
    (
        "state: angle",
        [this] { return getState().angle.Degrees().template to<double>(); },
        nullptr
    );
    builder.AddDoubleProperty
    (
        "state: speed",
        [this] { return getState().speed.template to<double>(); },
        nullptr
    );
    builder.AddDoubleProperty
    (
        "desired state: angle",
        [this] { return desiredState.angle.Degrees().template to<double>(); },
        nullptr
    );
    builder.AddDoubleProperty
    (
        "desired state: speed",
        [this] { return desiredState.speed.value(); },
        nullptr
    );
    builder.AddDoubleProperty
    (
        "state: distance",
        [this] { return getModulePosition().distance.template to<double>(); },
        nullptr
    );
}

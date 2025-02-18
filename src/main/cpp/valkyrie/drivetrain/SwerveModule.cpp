#include "valkyrie/drivetrain/SwerveModule.h"
#include "valkyrie/controllers/PhoenixController.h"
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

// Explicit template instantiation
// This is needed for linking
template class valor::SwerveModule<valor::PhoenixController<>, valor::PhoenixController<>>;

template<class AzimuthMotor, class DriveMotor>
SwerveModule<AzimuthMotor, DriveMotor>::SwerveModule(AzimuthMotor* _azimuthMotor,
                                                    DriveMotor* _driveMotor,
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
    initialMagEncoderValue = getMagEncoderCount();
}

template<class AzimuthMotor, class DriveMotor>
frc::Rotation2d SwerveModule<AzimuthMotor, DriveMotor>::getAzimuthPosition()
{
    return frc::Rotation2d{azimuthMotor->getPosition()};
}

template<class AzimuthMotor, class DriveMotor>
units::meter_t SwerveModule<AzimuthMotor, DriveMotor>::getDrivePosition()
{
    return driveMotor->getPosition() * wheelConversion;
}

template<class AzimuthMotor, class DriveMotor>
units::meters_per_second_t SwerveModule<AzimuthMotor, DriveMotor>::getDriveSpeed()
{
    return driveMotor->getSpeed() * wheelConversion;
}

template<class AzimuthMotor, class DriveMotor>
units::meters_per_second_t SwerveModule<AzimuthMotor, DriveMotor>::getMaxDriveSpeed()
{
    return driveMotor->getMaxMechSpeed() * wheelConversion;
}

template<class AzimuthMotor, class DriveMotor>
frc::SwerveModulePosition SwerveModule<AzimuthMotor, DriveMotor>::getModulePosition()
{
    return { getDrivePosition(), getAzimuthPosition()};
}

template<class AzimuthMotor, class DriveMotor>
frc::SwerveModuleState SwerveModule<AzimuthMotor, DriveMotor>::getState()
{
    return frc::SwerveModuleState{getDriveSpeed(), getAzimuthPosition()};
}

template<class AzimuthMotor, class DriveMotor>
AzimuthMotor::MagnetHealth SwerveModule<AzimuthMotor, DriveMotor>::getAzimuthMagnetHealth() 
{
    return azimuthMotor->getMagnetHealth();
}

template<class AzimuthMotor, class DriveMotor>
void SwerveModule<AzimuthMotor, DriveMotor>::setDesiredState(frc::SwerveModuleState _desiredState, bool isDriveOpenLoop)
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

template<class AzimuthMotor, class DriveMotor>
void SwerveModule<AzimuthMotor, DriveMotor>::resetDriveEncoder()
{
    driveMotor->reset();
}

template<class AzimuthMotor, class DriveMotor>
bool SwerveModule<AzimuthMotor, DriveMotor>::loadAndSetAzimuthZeroReference(std::vector<units::turn_t> offsets)
{
    // Read the encoder position. If the encoder position isn't returned, set the position to what the wheels
    //   are currently. The pit crew sets the wheels straight in pre-match setup. They should be close enough
    //   if the mag encoders aren't working.
    //   Protects against issues as seen in: https://www.youtube.com/watch?v=MGxpWNcv-VM
    units::turn_t currPos = getMagEncoderCount();
    if (currPos == 0_tr) {
        return false;
    }

    units::turn_t storedPos = 0_tr;

    if (wheelIdx >= 0 && wheelIdx <= 3){
        storedPos = offsets[wheelIdx];
    }
    
    // Get the remainder of the delta so the encoder can wrap
    azimuthMotor->setEncoderPosition(currPos - storedPos);
    return true;
}

template<class AzimuthMotor, class DriveMotor>
units::turn_t SwerveModule<AzimuthMotor, DriveMotor>::getMagEncoderCount()
{
    units::turn_t readValue = azimuthMotor->getAbsEncoderPosition();
    if (initialMagEncoderValue == 0_tr && readValue != 0_tr)
        initialMagEncoderValue = readValue;
    if (readValue != initialMagEncoderValue)
        return readValue;
    return 0_tr;
}

// The angle coming in is an optimized angle. No further calcs should be done on 'angle'
template<class AzimuthMotor, class DriveMotor>
void SwerveModule<AzimuthMotor, DriveMotor>::setAzimuthPosition(frc::Rotation2d desiredAngle)
{
    azimuthMotor->setPosition(units::turn_t{desiredAngle.Radians().value() / (2.0 * M_PI)});
}

template<class AzimuthMotor, class DriveMotor>
void SwerveModule<AzimuthMotor, DriveMotor>::setAzimuthPower(units::volt_t voltage)
{
    azimuthMotor->setPower(voltage);
}

template<class AzimuthMotor, class DriveMotor>
void SwerveModule<AzimuthMotor, DriveMotor>::setDrivePower(units::volt_t voltage)
{
    driveMotor->setPower(voltage);
}

template<class AzimuthMotor, class DriveMotor>
void SwerveModule<AzimuthMotor, DriveMotor>::setDriveOpenLoop(units::meters_per_second_t mps)
{
    driveMotor->setPower(mps / getMaxDriveSpeed() * units::volt_t{12});
}

template<class AzimuthMotor, class DriveMotor>
void SwerveModule<AzimuthMotor, DriveMotor>::setDriveClosedLoop(units::meters_per_second_t speed)
{
    auto outSpeed = units::turns_per_second_t{speed.value() / wheelConversion.value()};
    driveMotor->setSpeed(outSpeed);
}

template<class AzimuthMotor, class DriveMotor>
void SwerveModule<AzimuthMotor, DriveMotor>::setUpdateFrequency(units::frequency::hertz_t freq)
{
    azimuthMotor->setPositionUpdateFrequency(freq);
    azimuthMotor->setSpeedUpdateFrequency(freq);
    driveMotor->setPositionUpdateFrequency(freq);
    driveMotor->setSpeedUpdateFrequency(freq);
}

template<class AzimuthMotor, class DriveMotor>
frc::Translation2d SwerveModule<AzimuthMotor, DriveMotor>::convertSwerveStateToVelocityVector(frc::SwerveModuleState state)
{
    return frc::Translation2d{units::meter_t{state.speed.to<double>()}, state.angle};
}

template<class AzimuthMotor, class DriveMotor>
void SwerveModule<AzimuthMotor, DriveMotor>::InitSendable(wpi::SendableBuilder& builder)
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

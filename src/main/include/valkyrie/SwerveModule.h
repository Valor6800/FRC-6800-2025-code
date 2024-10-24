#pragma once

#include <frc/Filesystem.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/DutyCycleEncoder.h>

#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableBuilder.h>
#include <wpi/sendable/SendableHelper.h>

namespace valor {

/**
 * @brief SwerveModule
 * @tparam AzimuthMotor Azimuth motor on the swerve module (Neo or Falcon)
 * @tparam DriveMotor Drive motor on the swerve module (Neo or Falcon)
 * 
 * Usage:
 * \code {.cpp}
 * SwerveModule<ValorNeoController, ValorFalconController> swerve;
 * \endcode
 */
template<class AzimuthMotor, class DriveMotor>
class SwerveModule: public wpi::Sendable, public wpi::SendableHelper<SwerveModule<AzimuthMotor, DriveMotor>>
{
public: 

    SwerveModule(AzimuthMotor* _azimuthMotor,
                DriveMotor* _driveMotor,
                frc::Translation2d _wheelLocation);

    frc::SwerveModulePosition getModulePosition();

    /**
     * Get the current state of the swerve module
     * @return current state of the swerve module
     */
    frc::SwerveModuleState getState();

    /**
     * Command the swerve module motors to the desired state
     * @param desiredState the desired swerve module speed and angle
     * @param isDriveOpen true if drive should set speed using closed-loop velocity control
     */
    void setDesiredState(frc::SwerveModuleState desiredState, bool isDriveOpenLoop);

    /**
     * Command the swerve module motors to the desired state using closed-loop drive speed control
     * @param desiredState the desired swerve module speed and angle
     */
    void setDesiredState(frc::SwerveModuleState desiredState)
    {
        setDesiredState(desiredState, false);
    }

    void setMaxSpeed(double _maxSpeed)
    {
        maxSpeed = _maxSpeed;
    }

    /**
     * Resets the drive encoders to currently read a position of 0
     */
    void resetDriveEncoder();

    /**
     * Save the current azimuth absolute encoder reference position in NetworkTables preferences.
     * Call this method following physical alignment of the module wheel in its zeroed position.
     * Used during module instantiation to initialize the relative encoder.
     */

    /**
     * Loads the current azimuth absolute encoder reference position and sets selected sensor encoder
     * @return if the mag encoder was successfully 
     */
    bool loadAndSetAzimuthZeroReference(std::vector<double> offsets);

    frc::Rotation2d getAzimuthPosition();

    void setAzimuthPosition(frc::Rotation2d angle);

    void InitSendable(wpi::SendableBuilder& builder) override;

private:

    /**
     * Get the encoder position reported by the mag encoder
     * @return encoder position reported by the mag encoder
     */
    double getMagEncoderCount();

    double maxSpeed;

    void setDriveOpenLoop(double mps);

    void setDriveClosedLoop(double mps);

    AzimuthMotor* azimuthMotor;
    DriveMotor* driveMotor;

    int wheelIdx;
    double initialMagEncoderValue;
};
}

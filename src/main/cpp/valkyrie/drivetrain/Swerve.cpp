#include "valkyrie/drivetrain/Swerve.h"

#include <frc/estimator/SwerveDrivePoseEstimator.h>

using namespace valor;

template <size_t NumModules>
Swerve<NumModules>::Swerve(frc::TimedRobot *_robot) : valor::Drivetrain(_robot, "SwerveDrive")
{
    rawEstimator = std::make_unique<SwerveDrivePoseEstimator<NumModules>>();
}

template <size_t NumModules>
Swerve<NumModules>::~Swerve()
{
    Drivetrain::~Drivetrain();
}

template <size_t NumModules>
void Swerve<NumModules>::init()
{
    Drivetrain::init();
}

template <size_t NumModules>
void Swerve<NumModules>::assessInputs()
{
    Drivetrain::assessInputs();
    
    if (!driverGamepad || !driverGamepad->IsConnected())
        return;
}

template <size_t NumModules>
void Swerve<NumModules>::analyzeDashboard()
{
    Drivetrain::analyzeDashboard();
}

template <size_t NumModules>
void Swerve<NumModules>::assignOutputs()
{
    Drivetrain::assignOutputs();
}

template <size_t NumModules>
void Swerve<NumModules>::resetState()
{
    Drivetrain::resetState();
}

template <size_t NumModules>
void Swerve<NumModules>::resetOdometry(frc::Pose2d pose)
{
    rawEstimator->ResetPosition(getPigeon(), getModuleStates(), pose);
    calculatedEstimator->ResetPosition(getPigeon(), getModuleStates(), pose);
}

template <size_t NumModules>
void Swerve<NumModules>::resetEncoders()
{
    for (size_t i = 0; i < swerveModules.size(); i++)
    {
        swerveModules[i]->resetDriveEncoder();
    }
}

template <size_t NumModules>
void Swerve<NumModules>::setXMode(){
    drive(
        static_cast<units::velocity::meters_per_second_t>(0),
        static_cast<units::velocity::meters_per_second_t>(0),
        static_cast<units::angular_velocity::radians_per_second_t>(0),
        true
    );
    azimuthControllers[0]->setPosition(std::round(azimuthControllers[0]->getPosition()) + 0.125);
    azimuthControllers[1]->setPosition(std::round(azimuthControllers[1]->getPosition()) + 0.375);
    azimuthControllers[2]->setPosition(std::round(azimuthControllers[2]->getPosition()) - 0.375);
    azimuthControllers[3]->setPosition(std::round(azimuthControllers[3]->getPosition()) - 0.125);
    setDriveMotorNeutralMode(valor::NeutralMode::Brake);
}

template <size_t NumModules>
frc2::InstantCommand* Swerve<NumModules>::cmd_XMode()
{
    return new frc2::InstantCommand([&] {
        setXMode();
    });
}

template <size_t NumModules>
void Swerve<NumModules>::InitSendable(wpi::SendableBuilder& builder)
{
    Drivetrain::InitSendable(builder);
}
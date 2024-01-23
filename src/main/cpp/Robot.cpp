#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>
#include <frc/RobotController.h>
#include <Constants.h>

#include <ctime>

Robot::Robot() : 
    drivetrain(this), 
    valorAuto(), 
    beamBreak(DIOPorts::BEAM_BREAK_PORT), 
    shooter(this, &beamBreak), 
    intake(this, &beamBreak),
    feeder(this)
{
    frc::TimedRobot();
}

void Robot::RobotInit() {
    drivetrain.setGamepads(&gamepadOperator, &gamepadDriver);
    drivetrain.resetState();

    intake.setGamepads(&gamepadOperator, &gamepadDriver);
    intake.resetState();

    shooter.setGamepads(&gamepadOperator, &gamepadDriver);
    shooter.resetState();

    feeder.setGamepads(&gamepadOperator, &gamepadDriver);
    feeder.resetState();

    frc::LiveWindow::EnableAllTelemetry();
    frc::DataLogManager::Start();

    valorAuto.fillAutoList();

}
/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() { frc2::CommandScheduler::GetInstance().Run(); }

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() { }

void Robot::DisabledPeriodic() { }

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {
    drivetrain.resetState();
    drivetrain.state.matchStart = frc::Timer::GetFPGATimestamp().to<double>();
    drivetrain.setDriveMotorNeutralMode(valor::NeutralMode::Brake);
    drivetrain.pullSwerveModuleZeroReference();

    intake.resetState();
    feeder.resetState();

    autoCommand = valorAuto.getCurrentAuto();
    autoCommand.Schedule();
}

void Robot::AutonomousExit() {
    drivetrain.state.xPose = true;
}

void Robot::AutonomousPeriodic() {
}

void Robot::TeleopInit() {
    drivetrain.pullSwerveModuleZeroReference();
    drivetrain.setDriveMotorNeutralMode(valor::NeutralMode::Coast);

    autoCommand.Cancel();
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif

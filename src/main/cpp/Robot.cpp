#include "Robot.h"
#include "frc/AnalogTriggerType.h"
#include "frc2/command/Commands.h"

#include <frc/simulation/RoboRioSim.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>
#include <frc/RobotController.h>
#include <Constants.h>

#include <pathplanner/lib/auto/NamedCommands.h>

#include <ctime>
#define AUTO_DOUBTX 3.0f;
#define AUTO_DOUBTY 3.0f;
#define TELE_DOUBTX 0.75f;
#define TELE_DOUBTY 0.75f;

#define LED_COUNT 86
#define SEGMENTS 2

Robot::Robot() :
    drivetrain(this),
    valorAuto()
{}

void Robot::RobotInit() {
    drivetrain.setGamepads(&gamepadOperator, &gamepadDriver);
    drivetrain.resetState();

    frc::LiveWindow::EnableAllTelemetry();
    frc::LiveWindow::SetEnabled(true);
    frc::DataLogManager::Start();

    // charMode.init();
    valorAuto.fillAutoList();
    // valorAuto.preloadAuto("A1-");
    // valorAuto.preloadAuto("A1-2");
    // charMode.fillSelectList();
}
/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
    // std::vector<units::ampere_t> currents{std::move(drivetrain.getCurrents())};
    // units::volt_t batteryVoltage = batterySim.Calculate(currents);
    // wpi::println("Battery Voltage: {}", batteryVoltage);
    // frc::sim::RoboRioSim::SetVInVoltage(batteryVoltage);
    frc2::CommandScheduler::GetInstance().Run();
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() { }

void Robot::DisabledPeriodic() { 
    // valorAuto.preloadSelectedAuto();
}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
// TODO: re-add the neutral mode back in
void Robot::AutonomousInit() {
    drivetrain.resetState();
    drivetrain.doubtX = AUTO_DOUBTX;
    drivetrain.doubtY = AUTO_DOUBTY;

    autoCommands = valorAuto.getSelectedAuto();
    autoCommands.Schedule();
    // autoCommands.clear();
    // autoCommands.push_back(valorAuto.getSelectedAuto());
    // autoCommands.back().Schedule();
}

void Robot::AutonomousExit() {
}

void Robot::AutonomousPeriodic() {
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {
    drivetrain.selectedTest = charMode.getSelected();
}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif

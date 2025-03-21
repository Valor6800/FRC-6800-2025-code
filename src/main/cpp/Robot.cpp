#include "Robot.h"
#include "frc/AnalogTriggerType.h"
#include "frc2/command/Commands.h"
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

Robot::Robot() :
    leds{this, LEDConstants::LED_COUNT, LEDConstants::LED_SEGMENTS, CANIDs::CANDLE},
    drivetrain(this, &leds),
    scorer(this, &drivetrain, &leds),
    climber(this, &leds),
    valorAuto()
{
    frc::TimedRobot();
    leds.setColor(0, valor::CANdleSensor::OFF);
    // pathplanner::NamedCommands::registerCommand("Reschedule", std::move(
    //     frc2::InstantCommand([this](){
    //         autoCommands.back().Schedule();
    //     })
    // ).ToPtr());
}

void Robot::RobotInit() {
    frc::SmartDashboard::PutString("Robot Serial Number", Constants::serialNumber);
    // We could use frc2::cmd::None() instead of nullptr but it means that we have to hardcode the index
    // of the command in PIT_SEQUENCES or store it elsewhere otherwise Command* will be dangling
    // Probably easier to use nullptr and check when scheduling

    gamepadOperator.setDeadbandY(0.13);
    gamepadOperator.setDeadbandX(0.13);
    drivetrain.setGamepads(&gamepadOperator, &gamepadDriver);
    drivetrain.resetState();

    scorer.setGamepads(&gamepadOperator, &gamepadDriver);
    scorer.resetState();

    climber.setGamepads(&gamepadOperator, &gamepadDriver);
    climber.resetState();

    frc::LiveWindow::EnableAllTelemetry();
    frc::DataLogManager::Start();
    frc::DriverStation::StartDataLog(frc::DataLogManager::GetLog());

    charMode.init();
    valorAuto.fillAutoList();
    // valorAuto.preloadAuto("A1-");
    // valorAuto.preloadAuto("A1-2");
    charMode.fillSelectList();
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
    frc::SmartDashboard::PutNumber("Battery Voltage", frc::RobotController::GetBatteryVoltage().value());
    frc2::CommandScheduler::GetInstance().Run(); 
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() { }

void Robot::DisabledPeriodic() { 
    valorAuto.preloadSelectedAuto();
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
    scorer.state.autoDunkEnabled = true;
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

void Robot::TeleopInit() {
    scorer.resetState();
    drivetrain.setTeleopStartTime();
}
/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {
    drivetrain.selectedTest = charMode.getSelected();
}

void Robot::TestInit() {
    auto subsystemGateCommand = [this] {
        return frc2::FunctionalCommand{
            [this] { subsystemGate.Set(true); },
            [] {},
            [this](bool) { subsystemGate.Set(false); },
            [this] { return gamepadDriver.GetAButton(); }
        }.ToPtr();
    };
    pitSequenceCommand = frc2::cmd::Sequence(
        drivetrain.pitSequence(),
        subsystemGateCommand(),
        scorer.scorerPitSequence()
    );
    pitSequenceCommand.Schedule();
}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

void Robot::TestExit() {
    pitSequenceCommand.Cancel();
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif

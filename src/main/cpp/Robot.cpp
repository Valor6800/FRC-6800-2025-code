#include "Robot.h"
#include "frc/AnalogTriggerType.h"
#include "frc/geometry/Pose2d.h"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/InstantCommand.h"
#include "pathplanner/lib/auto/AutoBuilder.h"
#include "pathplanner/lib/path/PathPlannerPath.h"
#include "units/length.h"
#include "valkyrie/Auto.h"

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
{
    frc::TimedRobot();

    // pathplanner::NamedCommands::registerCommand("Reschedule", std::move(
    //     frc2::InstantCommand([this](){
    //         autoCommands.back().Schedule();
    //     })
    // ).ToPtr());
    //
    pathplanner::NamedCommands::registerCommand(
            "Schedule next", 
            frc2::InstantCommand(
                [this](){
                    autoCommands.back().Schedule();
                }
        ).ToPtr()
    );

    pathplanner::NamedCommands::registerCommand(
            "Check condition", 
            frc2::InstantCommand(
                [this](){
                    autoCommands.back().Schedule();
                }
        ).ToPtr()
    );
}

void Robot::RobotInit() {
    drivetrain.setGamepads(&gamepadOperator, &gamepadDriver);
    drivetrain.resetState();

    frc::LiveWindow::EnableAllTelemetry();
    frc::DataLogManager::Start();

    valorAuto.fillAutoList();
    // valorAuto.preloadAuto("A1-");
    // valorAuto.preloadAuto("A1-2");
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

void Robot::DisabledPeriodic() { 
    valorAuto.preloadSelectedAuto();
}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {
    drivetrain.resetState();
    printf("starting auto init");
    // drivetrain.setDriveMotorNeutralMode(valor::NeutralMode::Brake);
    // drivetrain.doubtX = AUTO_DOUBTX;
    // drivetrain.doubtY = AUTO_DOUBTY;
    

    std::string initPathName = "Copy of Af BLUE";
    auto path = pathplanner::PathPlannerPath::fromPathFile(initPathName);
    auto initpos = path->getWaypoints().front().anchor;
    drivetrain.resetOdometry(frc::Pose2d(initpos.X(), initpos.Y(), path->getInitialHeading()));
    // creating a commandptr and using std::move on it twice doesn't work
    auto opt1 = valor::Auto::makePathCommand("a1-a2");
    auto opt2 = valor::Auto::composePaths({"a1-shoot", "shoot-a2"});
    frc2::CommandPtr a = valor::Auto::composePaths({initPathName, "Af-A1 pt1"}).AndThen(
        valor::Auto::buildDynamicStep(
            [](){return true;},
            std::move(opt1),
            // valor::Auto::composePaths({"a1-shoot", "shoot-a2"}).AndThen(
            //     valor::Auto::buildDynamicStep(
            //         [this](){return true;},
            //         valor::Auto::makePathCommand("a2-shoot"),
            //         valor::Auto::makePathCommand("a2-a3-shoot")
            //     )
            // ),
            std::move(opt2)
            // valor::Auto::makePathCommand("a1-a2").AndThen(
            //     valor::Auto::buildDynamicStep(
            //         [this](){return true;},
            //         valor::Auto::makePathCommand("a2-shoot"),
            //         valor::Auto::makePathCommand("a2-a3-shoot")
            //     )
            // )
        )
    );

    autoCommands.clear();
    autoCommands.push_back(std::move(a));
    printf("Added auto command");
    autoCommands.back().Schedule();
    printf("Scheduled auto");
}

void Robot::AutonomousExit() {
    // drivetrain.state.xPose = true;
}

void Robot::AutonomousPeriodic() {
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

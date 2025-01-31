#pragma once

#include <frc/TimedRobot.h>
#include <frc2/command/Command.h>
#include <frc2/command/Commands.h>

#include "Constants.h"
#include "frc/AnalogTrigger.h"
#include "frc/AnalogTriggerOutput.h"
#include "valkyrie/Gamepad.h"

#include "valkyrie/drivetrain/Swerve.h"
#include "subsystems/Climber.h"

#include "valkyrie/Auto.h"
#include "Drivetrain.h"
#include "subsystems/Scorer.h"
#include "valkyrie/CharMode.h"

#include <frc/DriverStation.h>
#include <frc/DataLogManager.h>

#include <frc/livewindow/LiveWindow.h>

#include <fstream>
#include "frc/DigitalInput.h"

class Robot : public frc::TimedRobot {
    public:
        Robot();
        ~Robot() {}

        void RobotInit() override;
        void RobotPeriodic() override;
        void DisabledInit() override;
        void DisabledPeriodic() override;
        void AutonomousInit() override;
        void AutonomousPeriodic() override;
        void AutonomousExit() override;
        void TeleopInit() override;
        void TeleopPeriodic() override;
        void TestInit() override;
        void TestPeriodic() override;
        
    private:
        valor::Gamepad gamepadOperator{OIConstants::GAMEPAD_OPERATOR_LOCATION};
        valor::Gamepad gamepadDriver{OIConstants::GAMEPAD_BASE_LOCATION};

        frc2::CommandPtr autoCommands = frc2::cmd::None();
        Drivetrain drivetrain;
        Scorer scorer;
        Climber climber;

        valor::Auto valorAuto;
        valor::CharMode charMode;

        frc::SendableChooser<frc2::Command*> pitSequenceChooser;
};

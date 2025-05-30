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
#include "valkyrie/sensors/CANdleSensor.h"
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
        void TestExit() override;
        
    private:
        valor::Gamepad gamepadOperator{OIConstants::GAMEPAD_OPERATOR_LOCATION};
        valor::Gamepad gamepadDriver{OIConstants::GAMEPAD_BASE_LOCATION};

        frc2::CommandPtr autoCommands = frc2::cmd::None();

        // std::vector<frc2::CommandPtr> autoCommands;
        
        valor::CANdleSensor leds;
        Drivetrain drivetrain;
        Scorer scorer;
        Climber climber;

        static int cancoderMagnetHealthGetter(ctre::phoenix6::hardware::CANcoder& cancoder);

        valor::Auto valorAuto;
        valor::CharMode charMode;

        frc::Alert subsystemGate{"Subsystem test complete, press A button on driver gamepad to continue", frc::Alert::AlertType::kInfo};
        frc2::CommandPtr pitSequenceCommand = frc2::cmd::None();
};

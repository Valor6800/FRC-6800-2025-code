#pragma once

#include "frc2/command/CommandPtr.h"
#include <frc/TimedRobot.h>
#include <unordered_map>
#include <frc2/command/Command.h>
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <networktables/NetworkTable.h>

namespace valor {

class Auto
{
    public:
        Auto();

        /**
         * Fills the SendableChooser with a list of .auto filenames found in ~/deploy/pathplanner/autos key'd with a 
         * corresponding "human readable" name.
         */
        void fillAutoList();
        
        /**
         * Loads a selected pathplanner auto or returns the preloaded auto.
         *
         * @param filename Filename of the pathplanner auto to get
         * @returns The generated auto
         */
        frc2::CommandPtr getAuto(std::string filename);

        /**
         * Loads the auto with the filename selected in the SendableChooser.
         *
         * @returns The generated auto.
         */
        frc2::CommandPtr getSelectedAuto();

        /**
         * Loads and stores an auto to be retrieved with getAuto. This cuts out the delay that would normally happen if the auto 
         * was generated on AutonomousInit.
         *
         * Does nothing if an auto with the given filename is already loaded. Use clearAutos to clear the preloaded autos list,
         * then rebuild them.
         *
         * @param filename Filename of the pathplanner auto to preload
         */
        void preloadAuto(std::string filename);

        /**
         * Preloads the auto selected in the SendableChooser.
         */
        void preloadSelectedAuto();

        /**
         * Clears all preloaded autos.
         */
        void clearAutos();

        /**
         * Builds a command to run one of two commands based on a condition.
         * The returned command is the frc2::InstantCommand that schedules one of the two commands, not one of the two commands.
         * Commands intended to be run after either command should not be composed with the returned command, but should instead be
         * composed with each of the two commands passed in.
         *
         * @param condition The condition to check
         * @param commandIfTrue The command to run if the condition is true
         * @param commandIfFalse The command to run if the condition is false
         * @returns The frc2::InstantCommand that schedules one of the two commands
         */
        static frc2::CommandPtr buildDynamicStep(std::function<bool ()> condition, frc2::CommandPtr commandIfTrue, frc2::CommandPtr commandIfFalse);

        static frc2::CommandPtr makePathCommand(std::string pathName);
        
        static frc2::CommandPtr composePaths(std::vector<std::string> pathNames);

    private:
        std::vector<std::pair<std::string, frc2::CommandPtr>> loadedAutos;
        frc2::CommandPtr makeAuto(std::string autoName);
        std::shared_ptr<nt::NetworkTable> table;
        frc::SendableChooser<std::string> m_chooser;
};

}

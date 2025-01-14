#pragma once

#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

#include <map>


namespace valor {

class CharMode
{
    public:
        void init();

        /**
         * Fills the SendableChooser with a list of characterization modes
         */
        void fillSelectList();

        enum MODE_OPTIONS{
            NONE,
            ROT,
            STR_LINE
        };

        std::unordered_map<MODE_OPTIONS, std::string> modeMap {
            {NONE, "None"},
            {ROT, "Rot"},
            {STR_LINE, "Straight line test"}
        };

        MODE_OPTIONS getSelected();

    private:
        
        std::shared_ptr<nt::NetworkTable> table;
        frc::SendableChooser<int> chooser;

};

}

#pragma once

#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>


namespace valor {

class CharMode
{
    public:
        CharMode();

        /**
         * Fills the SendableChooser with a list of characterization modes
         */
        void fillSelecList();

        void GetSelected();
       

    private:
        
        std::shared_ptr<nt::NetworkTable> table;
        frc::SendableChooser<std::string> chooser;

};

}

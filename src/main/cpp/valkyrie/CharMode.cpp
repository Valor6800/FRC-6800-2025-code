#include "valkyrie/CharMode.h"
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <string>

using namespace valor;

const std::string list[2] = {"Straight line test", "Rot test"};

CharMode::CharMode(){
    table = nt::NetworkTableInstance::GetDefault().GetTable("list");
}

void CharMode::fillSelecList(){
    for (std::string option : list){
        chooser.AddOption(option, option);
    }
    frc::SmartDashboard::PutData(&chooser);
}

std::string CharMode::getSelected(){
    return chooser.GetSelected();
}


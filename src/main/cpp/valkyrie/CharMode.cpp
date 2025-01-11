#include "valkyrie/CharMode.h"
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <string>
#include <iostream>

using namespace valor;

const std::string list[2] = {"Straight line test", "Rot test"};

CharMode::CharMode(){
    table = nt::NetworkTableInstance::GetDefault().GetTable("list");
    chooser.SetDefaultOption("None", "None");
}

void CharMode::fillSelectList(){
    for (std::string option : list){
        chooser.AddOption(static_cast<std::string_view> (option), option);//static_cast<std::string_view>(
    }
    frc::SmartDashboard::PutData("CharMode", &chooser);
}


std::string CharMode::getSelected(){
    std::string selected = chooser.GetSelected();
    return selected;
}

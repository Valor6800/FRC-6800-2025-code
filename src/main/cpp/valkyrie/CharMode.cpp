/*                                 Valor 6800                                 */
/* Copyright (c) 2025 Company Name. All Rights Reserved.                      */

#include "valkyrie/CharMode.h"

#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <iostream>
#include <string>

using namespace valor;

void CharMode::init() {
  table = nt::NetworkTableInstance::GetDefault().GetTable("list");
  chooser.SetDefaultOption("None", MODE_OPTIONS::NONE);
}

void CharMode::fillSelectList() {
  for (std::pair<MODE_OPTIONS, std::string> i : modeMap) {
    chooser.AddOption(
        static_cast<std::string_view>(i.second),
        static_cast<int>(i.first)); // static_cast<std::string_view>(
  }
  frc::SmartDashboard::PutData("CharMode", &chooser);
}

CharMode::MODE_OPTIONS CharMode::getSelected() {
  MODE_OPTIONS selected = static_cast<MODE_OPTIONS>(chooser.GetSelected());
  return selected;
}

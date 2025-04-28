/*                                 Valor 6800                                 */
/* Copyright (c) 2025 Company Name. All Rights Reserved.                      */

#pragma once

#include <map>
#include <memory>
#include <string>
#include <unordered_map>

#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

namespace valor {

class CharMode {
public:
  void init();

  /**
   * Fills the SendableChooser with a list of characterization modes
   */
  void fillSelectList();

  enum MODE_OPTIONS { NONE, ROT, STR_LINE };

  std::unordered_map<MODE_OPTIONS, std::string> modeMap{
      {NONE, "None"}, {ROT, "Rot"}, {STR_LINE, "Straight line test"}};

  MODE_OPTIONS getSelected();

private:
  std::shared_ptr<nt::NetworkTable> table;
  frc::SendableChooser<int> chooser;
};

} // namespace valor

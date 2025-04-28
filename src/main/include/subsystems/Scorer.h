/*                                 Valor 6800                                 */
/* Copyright (c) 2025 Company Name. All Rights Reserved.                      */

#pragma once
#include <unordered_map>
#include <vector>

#include <ctre/phoenix6/core/CoreCANdi.hpp>
#include <frc/Alert.h>
#include <frc/DigitalInput.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/FunctionalCommand.h>
#include <networktables/StructTopic.h>

#include "Constants.h"
#include "Drivetrain.h"
#include "subsystems/Climber.h"
#include "valkyrie/BaseSubsystem.h"
#include "valkyrie/Gamepad.h"
#include "valkyrie/controllers/PIDF.h"
#include "valkyrie/controllers/PhoenixController.h"
#include "valkyrie/sensors/CANRangeSensor.h"
#include "valkyrie/sensors/CANdleSensor.h"
#include "valkyrie/sensors/CurrentSensor.h"
#include "valkyrie/sensors/DebounceSensor.h"
#include "valkyrie/sensors/GrappleSensor.h"

class Scorer : public valor::BaseSubsystem {
public:
  Scorer(frc::TimedRobot *robot, Drivetrain *drivetrain, Climber *climber,
         valor::CANdleSensor *);

  void resetState();

  void init();
  void assessInputs();
  void analyzeDashboard();
  void assignOutputs();
  void InitSendable(wpi::SendableBuilder &builder);

  frc2::CommandPtr scorerPitSequence();

  enum SCORE_STATE { HOLD, INTAKING, SCORING };

  enum SCOPED_STATE {
    UNSCOPED,
    MANUAL_SCOPE,
    SCOPED,
  };

  void setScopedState(SCOPED_STATE);

  struct x {
    SCORE_STATE scoringState;
    Constants::Scorer::ELEVATOR_STATE elevState;
    Constants::Scorer::GAME_PIECE gamePiece;
    SCOPED_STATE scopedState;

    units::meter_t targetHeight;
    units::volt_t manualSpeed;

    bool hasAlgae;
    bool hasCoral;
    bool intaking;
    double algaeSpikeCurrent;

    bool hasZeroed;
    bool autoDunkEnabled;
    bool shootOverCoral;
  } state;

private:
  frc2::CommandPtr scorerPitSequenceStage(Constants::Scorer::GAME_PIECE,
                                          Constants::Scorer::ELEVATOR_STATE,
                                          int, int);

  units::meter_t convertToMechSpace(units::turn_t turns);
  units::turn_t convertToMotorSpace(units::meter_t meters);

  bool hallEffectSensorActive();
  bool cancoderSensorBad();
  units::turn_t absSensorCorrect;

  valor::DebounceSensor hallEffectDebounceSensor;

  ctre::phoenix6::hardware::core::CoreCANdi candi;

  valor::PhoenixController<> *elevatorMotor;
  valor::PhoenixController<> *scorerMotor;
  valor::PhoenixController<> *scorerPivotMotor;
  valor::PhoenixController<> *funnelMotor;

  valor::CANrangeSensor scorerStagingSensor;
  valor::CurrentSensor currentSensor;
  valor::CurrentSensor coralCurrentSensor;

  Constants::Scorer::PositionMap positionMap;
  Constants::Scorer::ScoringSpeedMap scoringSpeedMap;

  Drivetrain *drivetrain;
  Climber *climber;
  valor::CANdleSensor *leds;

  frc::Alert elevatorStage{"Elevator going to stage",
                           frc::Alert::AlertType::kInfo};
  frc::Alert elevatorPositionSuccess{"Elevator position within tolerance",
                                     frc::Alert::AlertType::kInfo};
  frc::Alert elevatorPositionFail{"Elevator position outside tolerance",
                                  frc::Alert::AlertType::kError};
  frc::Alert elevatorBButtonWait{"Elevator waiting for driver B button...",
                                 frc::Alert::AlertType::kInfo};

  nt::StructPublisher<frc::Pose3d> visualizerStage1;
  nt::StructPublisher<frc::Pose3d> visualizerStage2;
  nt::StructPublisher<frc::Pose3d> visualizerStage3;

  bool elevatorWithinThreshold;
};

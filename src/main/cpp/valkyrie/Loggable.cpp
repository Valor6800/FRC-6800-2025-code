#include "valkyrie/Loggable.h"
#include <frc/DriverStation.h>
#include <frc/livewindow/LiveWindow.h>
#include <iostream>

using namespace valor;

std::vector<Loggable*> LoggableInstance::subsystems;

inline bool inMatch() {
    return frc::DriverStation::GetMatchType() != frc::DriverStation::MatchType::kNone;
}

/// This is so that the LoggableInstance is initialized and LiveWindow::EnableAllTelemetry will be called
/// after the HAL has been initialized
LoggableInstance& LoggableInstance::GetInstance() {
    static LoggableInstance instance;
    return instance;
}

LoggableInstance::LoggableInstance() {
    Register();

    if (!inMatch()) frc::LiveWindow::EnableAllTelemetry();
}

void LoggableInstance::Periodic() {
    // If not in match, do nothing. AddChild + AddLW will take care of automatically calling InitSendable
    if (frc::DriverStation::GetMatchType() == frc::DriverStation::MatchType::kNone) return;

    for (auto subsystem : subsystems) subsystem->UpdateValues();
}

void LoggableInstance::AddSubsystem(std::string name, Loggable *subsystem) {
    subsystems.push_back(subsystem);
    if (inMatch()) {
        subsystem->infos = Loggable::LogInfo{};
        std::get<Loggable::LogInfo>(subsystem->infos).path = "NT:/LiveWindow/" + name;
    } else {
        subsystem->infos = Loggable::NTInfo{};
        std::get<Loggable::NTInfo>(subsystem->infos).table = nt::NetworkTableInstance::GetDefault().GetTable("/LiveWindow/" + name);
        wpi::SendableRegistry::AddLW(subsystem, name);
    }
}

void Loggable::AddChildLoggable(std::string name, Loggable *child) {
    if (inMatch()) {
        child->infos = Loggable::LogInfo{};
        Loggable::LogInfo& logInfo = std::get<Loggable::LogInfo>(infos);
        std::get<Loggable::LogInfo>(child->infos).path = logInfo.path + "/" + name;
        logInfo.children.push_back(child);
    } else {
        std::get<Loggable::NTInfo>(child->infos).table = std::get<Loggable::NTInfo>(infos).table->GetSubTable(name);
        wpi::SendableRegistry::AddChild(this, child);
    }
}

void Loggable::InitSendable(wpi::SendableBuilder& _builder) {
    std::get<Loggable::NTInfo>(infos).builder = &_builder;
    InitLoggable();
}

void Loggable::UpdateValues() {
    entryIdx = 0;
    InitLoggable();
    entriesAdded = true;
    for (auto child : std::get<Loggable::LogInfo>(infos).children) child->UpdateValues();
}

// FIXME: Destructor for entries + fields

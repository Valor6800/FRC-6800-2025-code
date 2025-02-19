#pragma once
#include <string>
#include <functional>
#include <vector>
#include <frc2/command/SubsystemBase.h>
#include <wpi/sendable/SendableBuilder.h>
#include <wpi/DataLog.h>
#include <frc/DataLogManager.h>
#include <networktables/StructTopic.h>
#include <networktables/StructArrayTopic.h>

namespace valor {

class Loggable : public wpi::Sendable {
    friend class LoggableInstance;

protected:
    virtual void InitLoggable() = 0;
    void AddChildLoggable(std::string, Loggable*);

    inline void AddBoolProperty(std::string_view name, std::function<bool()> getter, std::function<void(bool)> setter) {
        AddGenericLoggableProperty<wpi::log::BooleanLogEntry>(&wpi::SendableBuilder::AddBooleanProperty, name, getter, setter);
    }

    inline void AddBoolArrayProperty(std::string_view name, std::function<std::vector<int>()> getter, std::function<void(std::span<const int>)> setter) {
        AddGenericLoggableProperty<wpi::log::BooleanArrayLogEntry>(&wpi::SendableBuilder::AddBooleanArrayProperty, name, getter, setter);
    }

    inline void AddIntProperty(std::string_view name, std::function<int64_t()> getter, std::function<void(int64_t)> setter) {
        AddGenericLoggableProperty<wpi::log::IntegerLogEntry>(&wpi::SendableBuilder::AddIntegerProperty, name, getter, setter);
    }

    inline void AddIntArrayProperty(std::string_view name, std::function<std::vector<int64_t>()> getter, std::function<void(std::span<const int64_t>)> setter) {
        AddGenericLoggableProperty<wpi::log::IntegerArrayLogEntry>(&wpi::SendableBuilder::AddIntegerArrayProperty, name, getter, setter);
    }

    inline void AddDoubleProperty(std::string_view name, std::function<double()> getter, std::function<void(double)> setter) {
        AddGenericLoggableProperty<wpi::log::DoubleLogEntry>(&wpi::SendableBuilder::AddDoubleProperty, name, getter, setter);
    }

    inline void AddDoubleArrayProperty(std::string_view name, std::function<std::vector<double>()> getter, std::function<void(std::span<const double>)> setter) {
        AddGenericLoggableProperty<wpi::log::DoubleArrayLogEntry>(&wpi::SendableBuilder::AddDoubleArrayProperty, name, getter, setter);
    }

    inline void AddStringProperty(std::string_view name, std::function<std::string()> getter, std::function<void(std::string_view)> setter) {
        AddGenericLoggableProperty<wpi::log::StringLogEntry>(&wpi::SendableBuilder::AddStringProperty, name, getter, setter);
    }

    inline void AddStringArrayProperty(std::string_view name, std::function<std::vector<std::string>()> getter, std::function<void(std::span<const std::string>)> setter) {
        AddGenericLoggableProperty<wpi::log::StringArrayLogEntry>(&wpi::SendableBuilder::AddStringArrayProperty, name, getter, setter);
    }

    template<class T>
    void AddStructProperty(std::string_view name, std::function<T()> getter, std::function<void(const T)> setter) {
        if (std::holds_alternative<NTInfo>(infos)) {
            NTInfo& ntinfo = std::get<NTInfo>(infos);
            // FIXME: Setter
            if (!entriesAdded) {
                nt::StructTopic topic = ntinfo.table->GetStructTopic<T>(name);
                ntinfo.fields.push_back(std::make_pair(
                    new nt::StructPublisher{topic.Publish()},
                    new nt::StructSubscriber{topic.Subscribe()}
                ));
            }
            auto field = ntinfo.fields[entryIdx++];
            static_cast<nt::StructPublisher<T>*>(field.first.get())->Set(getter());
        } else if (std::holds_alternative<LogInfo>(infos)) {
            LogInfo& logInfo = std::get<LogInfo>(infos);
            if (!entriesAdded) logInfo.entries.push_back(new wpi::log::StructLogEntry<T>{frc::DataLogManager::GetLog(), logInfo.path + "/" + std::string{name}});
            static_cast<wpi::log::StructLogEntry<T>*>(logInfo.entries[entryIdx++])->Update(getter());
        }
    }

    template<class T>
    void AddStructArrayProperty(std::string_view name, std::function<std::vector<T>()> getter, std::function<void(std::span<const T>)>) {
        if (std::holds_alternative<NTInfo>(infos)) {
            // FIXME: Setter
            NTInfo& ntinfo = std::get<NTInfo>(infos);
            if (!entriesAdded) {
                nt::StructArrayTopic topic = ntinfo.table->GetStructArrayTopic<T>(name);
                ntinfo.fields.push_back(std::make_pair(
                    new nt::StructArrayPublisher{topic.Publish()},
                    new nt::StructArraySubscriber{topic.Subscribe()}
                ));
            }
            auto field = ntinfo.fields[entryIdx++];
            static_cast<nt::StructArrayPublisher<T>*>(field.first.get())->Set(getter());
        } else {
            LogInfo& logInfo = std::get<LogInfo>(infos);
            if (!entriesAdded) logInfo.entries.push_back(new wpi::log::StructArrayLogEntry<T>{frc::DataLogManager::GetLog(), logInfo.path + "/" + std::string{name}});
            static_cast<wpi::log::StructArrayLogEntry<T>*>(logInfo.entries[entryIdx++])->Update(getter());
        }
    }

private:
    void UpdateValues();
    void InitSendable(wpi::SendableBuilder&) override;

    template<typename DataLogEntryType, typename Getter, typename Setter>
    void AddGenericLoggableProperty(
        void (wpi::SendableBuilder::*func)(std::string_view, std::function<Getter()>, std::function<void(Setter)>),
        std::string_view name,
        std::function<Getter()> getter,
        std::function<void(Setter)> setter
        ) {
        if (std::holds_alternative<NTInfo>(infos))
            (std::get<NTInfo>(infos).builder->*func)(name, getter, setter);
        else if (std::holds_alternative<LogInfo>(infos)) {
            LogInfo& logInfo = std::get<LogInfo>(infos);
            if (!entriesAdded) logInfo.entries.push_back(new DataLogEntryType{frc::DataLogManager::GetLog(), logInfo.path + "/" + std::string{name}});
            static_cast<DataLogEntryType*>(logInfo.entries[entryIdx++])->Update(getter());
        }
    }

    struct LogInfo {
        std::string path;
        std::vector<Loggable*> children;
        std::vector<wpi::log::DataLogEntry*> entries;
    };

    struct NTInfo {
        std::shared_ptr<nt::NetworkTable> table;
        wpi::SendableBuilder *builder;
        // Array of publishers and subscribers for types that SendableBuilder doesn't have by default
        // Structs, protobufs, etc.
        std::vector<std::pair<std::unique_ptr<nt::Publisher>, std::unique_ptr<nt::Subscriber>>> fields;
    };

    std::variant<LogInfo, NTInfo> infos;

    /*
    If the loggable entries never change, the order in which they were added to the vector is the same order that we will run the AddLoggableProperty's every time
    We can essentially just make it a giant for loop by storing the index between each function call
    */
    int entryIdx;
    bool entriesAdded{false};
};

class LoggableInstance : frc2::Subsystem {
public:
    static LoggableInstance& GetInstance();
    void AddSubsystem(std::string, Loggable*);

private:
    LoggableInstance();
    void Periodic() override;

    static std::vector<Loggable*> subsystems;
};

}
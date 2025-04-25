#pragma once

#include "units/voltage.h"
#include "valkyrie/controllers/BaseController.h"
#include <ctre/phoenix6/TalonFX.hpp>
#include <string>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/CANBus.hpp>

// Conversion guide: https://v6.docs.ctr-electronics.com/en/latest/docs/migration/migration-guide/closed-loop-guide.html

namespace valor {

enum PhoenixControllerType {
    KRAKEN_X60_FOC,
    KRAKEN_X60,
    KRAKEN_X44_FOC,
    KRAKEN_X44,
    FALCON_FOC,
    FALCON
}; 

template <class PositionOutput = ctre::phoenix6::controls::MotionMagicVoltage, class VelocityOutput = ctre::phoenix6::controls::VelocityVoltage>
class PhoenixController : public BaseController, ctre::phoenix6::hardware::TalonFX
{
    static_assert(std::is_base_of_v<ctre::phoenix6::controls::ControlRequest, VelocityOutput>);
    static_assert(std::is_base_of_v<ctre::phoenix6::controls::ControlRequest, PositionOutput>);

public:
    PhoenixController(valor::PhoenixControllerType controllerType, int canID, valor::NeutralMode mode, bool inverted, double _rotorToSensor, double _sensorToMech, std::string canbus = "") :
        BaseController{getPhoenixControllerMotorSpeed(controllerType), _rotorToSensor, _sensorToMech},
        ctre::phoenix6::hardware::TalonFX{canID, canbus},
        position_res(GetPosition()),
        velocity_res{GetVelocity()}
    {
        valor::PIDF pidf;
        pidf.P = FALCON_PIDF_KP;
        pidf.I = FALCON_PIDF_KI;
        pidf.D = FALCON_PIDF_KD;
        pidf.error = 0_tr;
        pidf.maxVelocity = FALCON_PIDF_KV;
        pidf.maxAcceleration = FALCON_PIDF_KA;

        position_req.Slot = 0;
        position_req.UpdateFreqHz = 0_Hz;
        velocity_req.Slot = 0;
        velocity_req.UpdateFreqHz = 0_Hz;
        voltage_req.UpdateFreqHz = 0_Hz;
        duty_cycle_req.UpdateFreqHz = 0_Hz;
        current_req.UpdateFreqHz = 0_Hz;

        config.MotorOutput.Inverted = inverted;

        config.MotorOutput.DutyCycleNeutralDeadband = DEADBAND.value();
        config.MotorOutput.NeutralMode = mode == valor::NeutralMode::Brake ?
            ctre::phoenix6::signals::NeutralModeValue::Brake : 
            ctre::phoenix6::signals::NeutralModeValue::Coast;

        config.Feedback.RotorToSensorRatio = rotorToSensor;
        config.Feedback.SensorToMechanismRatio = sensorToMech;

        GetConfigurator().Apply(config.MotorOutput);
        GetConfigurator().Apply(config.Feedback);

        setCurrentLimits(STATOR_CURRENT_LIMIT, SUPPLY_CURRENT_LIMIT, SUPPLY_CURRENT_THRESHOLD, SUPPLY_TIME_THRESHOLD);
        setPIDF(pidf, 0);

        wpi::SendableRegistry::AddLW(this, "PhoenixController", "ID " + std::to_string(GetDeviceID()));
    }

    static units::revolutions_per_minute_t getPhoenixControllerMotorSpeed(PhoenixControllerType controllerType)
    {
        switch (controllerType) {
            case KRAKEN_X60_FOC:
                return FREE_SPD_KRAKEN_X60_FOC;
            case KRAKEN_X60:
                return FREE_SPD_KRAKEN_X60;
            case KRAKEN_X44_FOC:
                return FREE_SPD_KRAKEN_X44_FOC;
            case KRAKEN_X44:
                return FREE_SPD_KRAKEN_X44;
            case FALCON_FOC:
                return FREE_SPD_FALCON_FOC;
            case FALCON:
                return FREE_SPD_FALCON;
            default:
                return FREE_SPD_KRAKEN_X60;
        }
    }

    void enableFOC() {
        tryEnableFOC(voltage_req);
        tryEnableFOC(current_req);
        tryEnableFOC(duty_cycle_req);
        tryEnableFOC(velocity_req);
        tryEnableFOC(position_req);
    }

    void applyConfig() {
        GetConfigurator().Apply(config, 5_s);
    }

    void reset() override {
        setEncoderPosition(0_tr);
    }
    
    void setCurrentLimits(units::ampere_t statorCurrentLimit, units::ampere_t supplyCurrentLimit, units::ampere_t supplyCurrentThreshold, units::second_t supplyTimeThreshold, bool saveImmediately = false) {
        config.CurrentLimits.StatorCurrentLimit = statorCurrentLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimit;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLowerTime = supplyTimeThreshold;
        config.CurrentLimits.SupplyCurrentLowerLimit = supplyCurrentThreshold;
        config.TorqueCurrent.PeakForwardTorqueCurrent = PEAK_TORQUE_CURRENT;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -PEAK_TORQUE_CURRENT;

        if (saveImmediately) {
            GetConfigurator().Apply(config.CurrentLimits);
            GetConfigurator().Apply(config.TorqueCurrent);
        }
    }

    void setVoltageLimits(units::volt_t reverseLimit, units::volt_t forwardLimit, bool saveImmediately = false) {
        config.Voltage.PeakForwardVoltage = forwardLimit;
        config.Voltage.PeakReverseVoltage = reverseLimit;

        if (saveImmediately) {
            GetConfigurator().Apply(config.Voltage);
        }
    }

    units::turn_t getPosition() override {
        return position_res.Refresh().GetValue();
    }

    units::turns_per_second_t getSpeed() override {
        return velocity_res.Refresh().GetValue();
    }

    void setEncoderPosition(units::turn_t position) override {
        SetPosition(position);
    }

    void setPosition(units::turn_t position, int slot = 0) override {
        SetControl(position_req.WithSlot(slot).WithPosition(position));
    }

    void setSpeed(units::turns_per_second_t velocity, int slot = 0) override {
        SetControl(velocity_req.WithSlot(slot).WithVelocity(velocity));
    }

    void setPower(units::volt_t voltage) override { SetControl(voltage_req.WithOutput(voltage)); }
    void setPower(units::ampere_t current) override { SetControl(current_req.WithOutput(current)); }
    void setPower(units::scalar_t dutyCycle) override { SetControl(duty_cycle_req.WithOutput(dutyCycle)); }

    units::volt_t getVoltage() override { return GetMotorVoltage().GetValue(); }
    units::ampere_t getCurrent() override { return GetStatorCurrent().GetValue(); }
    units::scalar_t getDutyCycle() override { return GetDutyCycle().GetValue(); }

    void setupFollower(int canID, bool followerInverted = false) {
        // TODO: can "baseCAN" be replaced by GetNetwork()?
        followerMotor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(canID, "baseCAN");
        ctre::phoenix6::configs::MotorOutputConfigs config;
        config.Inverted = followerInverted;
        config.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
        followerMotor->GetConfigurator().Apply(config);
        followerMotor->SetControl(ctre::phoenix6::controls::StrictFollower(GetDeviceID()));
    }
    
    void setPIDF(valor::PIDF pidf, int slot = 0, bool saveImmediately = false) {
        // Motion magic configuration
        config.MotionMagic.MotionMagicCruiseVelocity = pidf.maxVelocity;
        config.MotionMagic.MotionMagicAcceleration = pidf.maxAcceleration;
        config.MotionMagic.MotionMagicJerk = pidf.maxJerk;

        if (slot == 1) setPIDFSlot(config.Slot1, pidf, saveImmediately);
        else if (slot == 2) setPIDFSlot(config.Slot2, pidf, saveImmediately);
        else setPIDFSlot(config.Slot0, pidf, saveImmediately); // Default case

        if (saveImmediately) GetConfigurator().Apply(config.MotionMagic);
    }

    void setupReverseHardwareLimit(int canID, ctre::phoenix6::signals::ReverseLimitTypeValue type, units::turn_t autosetPosition = 0_tr, bool saveImmediately = false) {
        config.HardwareLimitSwitch.ReverseLimitType = type;
        config.HardwareLimitSwitch.ReverseLimitEnable = true;
        config.HardwareLimitSwitch.ReverseLimitRemoteSensorID = canID;
        config.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = autosetPosition;
        config.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = false;
        config.HardwareLimitSwitch.ReverseLimitSource = ctre::phoenix6::signals::ReverseLimitSourceValue::RemoteCANdiS1;

        if (saveImmediately) GetConfigurator().Apply(config.HardwareLimitSwitch);
    }

    void setupForwardHardwareLimit(int canID, ctre::phoenix6::signals::ForwardLimitTypeValue type, units::turn_t autosetPosition = 0_tr, bool saveImmediately = false) {
        config.HardwareLimitSwitch.ForwardLimitType = type;
        config.HardwareLimitSwitch.ForwardLimitEnable = true;
        config.HardwareLimitSwitch.ForwardLimitRemoteSensorID = canID;
        config.HardwareLimitSwitch.ForwardLimitAutosetPositionValue = autosetPosition;
        config.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = true;
        config.HardwareLimitSwitch.ForwardLimitSource = ctre::phoenix6::signals::ReverseLimitSourceValue::RemoteCANdiS1;

        if (saveImmediately) GetConfigurator().Apply(config.HardwareLimitSwitch);
    }

    void setForwardLimit(units::turn_t forward, bool saveImmediately = false) {
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = forward;

        if (saveImmediately) GetConfigurator().Apply(config.SoftwareLimitSwitch);
    }

    void setReverseLimit(units::turn_t reverse, bool saveImmediately = false) {
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = reverse;

        if (saveImmediately) GetConfigurator().Apply(config.SoftwareLimitSwitch);
    }

    ctre::phoenix6::signals::MagnetHealthValue getMagnetHealth() {
        if (!cancoder) return ctre::phoenix6::signals::MagnetHealthValue::Magnet_Invalid;
        return cancoder->GetMagnetHealth().GetValue();
    }

    units::turn_t getAbsEncoderPosition() {
        if (cancoder == nullptr) return 0_tr;
        return cancoder->GetAbsolutePosition().GetValue();
    }

    void setupCANCoder(int deviceId, units::turn_t offset, bool clockwise, std::string canbus = "", units::turn_t absoluteRange=1_tr, bool saveImmediately = false) {
        cancoder = new ctre::phoenix6::hardware::CANcoder(deviceId, canbus);
    
        ctre::phoenix6::configs::MagnetSensorConfigs cancoderConfig;
        cancoderConfig.AbsoluteSensorDiscontinuityPoint = absoluteRange;
        cancoderConfig.SensorDirection = clockwise ? ctre::phoenix6::signals::SensorDirectionValue::Clockwise_Positive :
                                            ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive;
        cancoderConfig.MagnetOffset = -offset;
        cancoder->GetConfigurator().Apply(cancoderConfig);

        config.Feedback.FeedbackRemoteSensorID = cancoder->GetDeviceID();
        config.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::FusedCANcoder;
        config.Feedback.SensorToMechanismRatio = sensorToMech;
        config.Feedback.RotorToSensorRatio = rotorToSensor;

        if (saveImmediately) GetConfigurator().Apply(config.Feedback);
    }

    float getBusUtil(const char* canBusName) {
        // @todo Initialize a CANBus, and get utilization
        return 0;
    }

    bool GetFault_BadMagnet(bool refresh = true) {
        if (cancoder) {
            auto& faultSignal = cancoder->GetFault_BadMagnet();
            if (refresh) {
                faultSignal.Refresh();
            }
            return faultSignal.GetValue();
        }
        return false;
    }

    // This literally replaces the default TalonFX LiveWindow entry that gives zero useful information - shouldn't have any side effects
    void InitSendable(wpi::SendableBuilder& builder) override {
        builder.SetSmartDashboardType("Subsystem");
        builder.AddDoubleProperty(
            "Stator Current", 
            [this] { return GetStatorCurrent().GetValueAsDouble(); },
            nullptr);
        builder.AddDoubleProperty(
            "Supply Current", 
            [this] { return GetSupplyCurrent().GetValueAsDouble(); },
            nullptr);
        builder.AddDoubleProperty(
            "Position", 
            [this] { return getPosition().template to<double>(); },
            nullptr);
        builder.AddDoubleProperty(
            "Absolute Position",
            [this] { return getAbsEncoderPosition().template to<double>(); },
            nullptr);
        builder.AddDoubleProperty(
            "Speed", 
            [this] { return getSpeed().template to<double>(); },
            nullptr);
        builder.AddDoubleProperty(
            "Out Volt", 
            [this] { return GetMotorVoltage().GetValueAsDouble(); },
            nullptr);
        builder.AddDoubleProperty(
            "reqPosition", 
            [this] { return position_req.Position.template to<double>(); },
            nullptr);
        builder.AddDoubleProperty(
            "reqSpeed", 
            [this] { return velocity_req.Velocity.template to<double>(); },
            nullptr);
        builder.AddStringProperty(
            "Magnet Health",
            [this] { return getMagnetHealth().ToString(); },
            nullptr);
        builder.AddBooleanProperty(
            "Undervolting",
            [this] { return GetFault_Undervoltage().GetValue(); },
            nullptr);
    }

    // This is to provide compatibility with the BaseController API
    ctre::phoenix6::controls::DutyCycleOut duty_cycle_req{0};
    ctre::phoenix6::controls::VoltageOut voltage_req{0_V};
    ctre::phoenix6::controls::TorqueCurrentFOC current_req{0_A};

    VelocityOutput velocity_req{0_tps};
    PositionOutput position_req{0_tr};

    ctre::phoenix6::StatusSignal<units::turn_t>& position_res;
    ctre::phoenix6::StatusSignal<units::turns_per_second_t>& velocity_res;
    ctre::phoenix6::configs::TalonFXConfiguration config;
    ctre::phoenix6::hardware::CANcoder *cancoder;

private:
    static constexpr units::ampere_t SUPPLY_CURRENT_THRESHOLD = 65_A;
    static constexpr units::ampere_t STATOR_CURRENT_LIMIT = 80_A;
    static constexpr units::ampere_t SUPPLY_CURRENT_LIMIT = 80_A;
    static constexpr units::millisecond_t SUPPLY_TIME_THRESHOLD = 1000_ms;
    static constexpr units::ampere_t PEAK_TORQUE_CURRENT = 100_A;
    static constexpr units::turn_t DEADBAND = 0.01_tr;

    static constexpr double FALCON_PIDF_KP = 10.0f;
    static constexpr double FALCON_PIDF_KI = 0.0f;
    static constexpr double FALCON_PIDF_KD = 0.0f;
    static constexpr units::turns_per_second_t FALCON_PIDF_KV = 6_tps;
    static constexpr units::turns_per_second_squared_t FALCON_PIDF_KA = 130_tr_per_s_sq;
    static constexpr units::turns_per_second_cubed_t FALCON_PIDF_KJ = 650_tr_per_s_cu;

    static constexpr units::revolutions_per_minute_t FREE_SPD_KRAKEN_X60 = 6000_rpm;
    static constexpr units::revolutions_per_minute_t FREE_SPD_KRAKEN_X60_FOC = 5800_rpm;
    static constexpr units::revolutions_per_minute_t FREE_SPD_KRAKEN_X44 = 7530_rpm;
    static constexpr units::revolutions_per_minute_t FREE_SPD_KRAKEN_X44_FOC = 7530_rpm;
    static constexpr units::revolutions_per_minute_t FREE_SPD_FALCON = 6380_rpm;
    static constexpr units::revolutions_per_minute_t FREE_SPD_FALCON_FOC = 6080_rpm;

    template <class S>
    void setPIDFSlot(S& slot, valor::PIDF pidf, bool saveImmediately) {
        // Generic PIDF configurations
        // Numerator for closed loop controls will be in volts
        slot.kP = pidf.P;
        slot.kI = pidf.I;
        slot.kD = pidf.D;
        slot.kS = pidf.S;
        if (pidf.kV < 0)
            slot.kV = (voltageCompensation / getMaxMechSpeed()).value();
        else
            slot.kV = pidf.kV;

        // Feedforward gain configuration
        if (pidf.aFF != 0) {
            slot.GravityType = pidf.aFFType == valor::FeedForwardType::LINEAR ?
                ctre::phoenix6::signals::GravityTypeValue::Elevator_Static :
                ctre::phoenix6::signals::GravityTypeValue::Arm_Cosine;
            slot.kG = pidf.aFF;
        }

        if (saveImmediately) GetConfigurator().Apply(slot);
    }

    // Helper function for enableFOC that only enables if the template has the field "EnableFOC"
    template <typename T>
    static inline void tryEnableFOC(T& request) {
        constexpr bool hasEnableFOC = requires (T& t) {
            t.EnableFOC;
        };
        if constexpr (hasEnableFOC) request.EnableFOC = true;
    }

    valor::PIDF pidf;
    std::unique_ptr<ctre::phoenix6::hardware::TalonFX> followerMotor;
};

namespace phoenix {
    /**
     * @brief Wrapper over DynamicMotionMagic* control requests
     * @tparam T Specific DynamicMotionMagic control request
     * 
     * Since PhoenixController initializes the position control request with a single position argument,
     * a compile error will be thrown when DynamicMotionMagic control requests are used as the constructor expects three arguments.
     * This very simple class zeroes acceleration and velocity of the control request.
     */
    template<class T>
    class DynamicMotionMagic : public T {
    public:
        DynamicMotionMagic(units::turn_t tr) : T{tr, 0_tps, 0_tr_per_s_sq, 0_tr_per_s_cu} {}
    };
}
}

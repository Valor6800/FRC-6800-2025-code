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

template <class RawOutput = ctre::phoenix6::controls::VoltageOut, class VelocityOutput = ctre::phoenix6::controls::VelocityVoltage, class PositionOutput = ctre::phoenix6::controls::MotionMagicVoltage>
class PhoenixController : public BaseController<ctre::phoenix6::hardware::TalonFX>
{
    static_assert(std::is_base_of_v<ctre::phoenix6::controls::ControlRequest, RawOutput>);
    static_assert(std::is_base_of_v<ctre::phoenix6::controls::ControlRequest, VelocityOutput>);
    static_assert(std::is_base_of_v<ctre::phoenix6::controls::ControlRequest, PositionOutput>);

public:
    typedef decltype(RawOutput::Output) RawOutputUnit;

    PhoenixController(valor::PhoenixControllerType controllerType, int canID, valor::NeutralMode _mode, bool _inverted, std::string canbus = "") :
        BaseController{new ctre::phoenix6::hardware::TalonFX(canID, canbus), _inverted, _mode, getPhoenixControllerMotorSpeed(controllerType)},
        req_raw_out{RawOutputUnit{}},
        req_vel_out{0_tps},
        req_pos_out{0_tr},
        cancoder{nullptr},
        res_position(getMotor()->GetPosition()),
        res_velocity{getMotor()->GetVelocity()}
    {
        init();
    };

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

    void init() override {
        valor::PIDF motionPIDF;
        motionPIDF.P = FALCON_PIDF_KP;
        motionPIDF.I = FALCON_PIDF_KI;
        motionPIDF.D = FALCON_PIDF_KD;
        motionPIDF.error = 0_tr;
        motionPIDF.maxVelocity = FALCON_PIDF_KV;
        motionPIDF.maxAcceleration = FALCON_PIDF_KA;

        req_pos_out.Slot = 0;
        req_pos_out.UpdateFreqHz = 0_Hz;
        req_vel_out.Slot = 0;
        req_vel_out.UpdateFreqHz = 0_Hz;
        req_raw_out.UpdateFreqHz = 0_Hz;

        setNeutralMode(neutralMode);
        setCurrentLimits(STATOR_CURRENT_LIMIT, SUPPLY_CURRENT_LIMIT, SUPPLY_CURRENT_THRESHOLD, SUPPLY_TIME_THRESHOLD);
        setGearRatios(rotorToSensor, sensorToMech);
        setPIDF(pidf, 0);

        wpi::SendableRegistry::AddLW(this, "PhoenixController", "ID " + std::to_string(getMotor()->GetDeviceID()));
    }

    void enableFOC(bool enableFOC) {
        tryEnableFOC(req_raw_out, enableFOC);
        tryEnableFOC(req_vel_out, enableFOC);
        tryEnableFOC(req_pos_out, enableFOC);
    }

    void applyConfig() override {
        getMotor()->GetConfigurator().Apply(config, 5_s);
    }

    void reset() override {
        setEncoderPosition(0_tr);
    }

    void setNeutralMode(valor::NeutralMode mode, bool saveImmediately = false) override {
        neutralMode = mode;
        config.MotorOutput.DutyCycleNeutralDeadband = DEADBAND.value();
        config.MotorOutput.Inverted = inverted;
        config.MotorOutput.NeutralMode = neutralMode == valor::NeutralMode::Brake ?
            ctre::phoenix6::signals::NeutralModeValue::Brake : 
            ctre::phoenix6::signals::NeutralModeValue::Coast;
        
        if (saveImmediately) getMotor()->GetConfigurator().Apply(config.MotorOutput);
    }
    
    void setCurrentLimits(units::ampere_t statorCurrentLimit, units::ampere_t supplyCurrentLimit, units::ampere_t supplyCurrentThreshold, units::second_t supplyTimeThreshold, bool saveImmediately = false) override {
        config.CurrentLimits.StatorCurrentLimit = statorCurrentLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimit;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLowerTime = supplyTimeThreshold;
        config.CurrentLimits.SupplyCurrentLowerLimit = supplyCurrentThreshold;
        config.TorqueCurrent.PeakForwardTorqueCurrent = PEAK_TORQUE_CURRENT;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -PEAK_TORQUE_CURRENT;

        if (saveImmediately) {
            getMotor()->GetConfigurator().Apply(config.CurrentLimits);
            getMotor()->GetConfigurator().Apply(config.TorqueCurrent);
        }
    }

    void setVoltageLimits(units::volt_t reverseLimit, units::volt_t forwardLimit, bool saveImmediately = false) {
        config.Voltage.PeakForwardVoltage = forwardLimit;
        config.Voltage.PeakReverseVoltage = reverseLimit;

        if (saveImmediately) {
            getMotor()->GetConfigurator().Apply(config.Voltage);
        }
    }

    units::turn_t getPosition() override {
        return res_position.Refresh().GetValue();
    }

    units::turns_per_second_t getSpeed() override {
        return res_velocity.Refresh().GetValue();
    }

    units::ampere_t getCurrent() override {
        return getMotor()->GetStatorCurrent().GetValue();
    }

    void setPositionUpdateFrequency(units::hertz_t hertz) {
        res_position.SetUpdateFrequency(hertz);
        // Do we really need to set CANcoder update frequency because we always access through motor controller res_position
        if (cancoder != nullptr) cancoder->GetPosition().SetUpdateFrequency(hertz);
    }

    void setSpeedUpdateFrequency(units::hertz_t hertz) {
        res_velocity.SetUpdateFrequency(hertz);
        // Do we really need to set CANcoder update frequency because we always access through motor controller res_velocity
        if (cancoder != nullptr) cancoder->GetVelocity().SetUpdateFrequency(hertz);
    }

    void setEncoderPosition(units::turn_t position) override {
        getMotor()->SetPosition(position);
    }

    void setContinuousWrap(bool continuousWrap, bool saveImmediately = false) {
        config.ClosedLoopGeneral.ContinuousWrap = continuousWrap;

        if (saveImmediately) getMotor()->GetConfigurator().Apply(config.ClosedLoopGeneral);
    }
    
    void setPosition(units::turn_t position, int slot = 0) override {
        getMotor()->SetControl(req_pos_out.WithSlot(slot).WithPosition(position));
    }

    void setSpeed(units::turns_per_second_t velocity, int slot = 0) override {
        getMotor()->SetControl(req_vel_out.WithSlot(slot).WithVelocity(velocity));
    }

    void setPower(RawOutputUnit out) {
        getMotor()->SetControl(req_raw_out.WithOutput(out));
    }

    void setupFollower(int canID, bool followerInverted = false) override {
        followerMotor = new ctre::phoenix6::hardware::TalonFX(canID, "baseCAN");
        ctre::phoenix6::configs::MotorOutputConfigs config{};
        config.Inverted = followerInverted;
        config.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
        followerMotor->GetConfigurator().Apply(config);
        followerMotor->SetControl(ctre::phoenix6::controls::StrictFollower(getMotor()->GetDeviceID()));
    }
    
    void setPIDF(valor::PIDF _pidf, int slot = 0, bool saveImmediately = false) override {
        pidf = _pidf;

        // Motion magic configuration
        config.MotionMagic.MotionMagicCruiseVelocity = pidf.maxVelocity;
        config.MotionMagic.MotionMagicAcceleration = pidf.maxAcceleration;
        config.MotionMagic.MotionMagicJerk = pidf.maxJerk;

        if (slot == 1) setPIDFSlot(config.Slot1, saveImmediately);
        else if (slot == 2) setPIDFSlot(config.Slot2, saveImmediately);
        else setPIDFSlot(config.Slot0, saveImmediately); // Default case

        if (saveImmediately) getMotor()->GetConfigurator().Apply(config.MotionMagic);
    }

    void setupReverseHardwareLimit(int canID, ctre::phoenix6::signals::ReverseLimitTypeValue type, units::turn_t autosetPosition = 0_tr, bool saveImmediately = false) {
        config.HardwareLimitSwitch.ReverseLimitType = type;
        config.HardwareLimitSwitch.ReverseLimitEnable = true;
        config.HardwareLimitSwitch.ReverseLimitRemoteSensorID = canID;
        config.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = autosetPosition;
        config.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = false;
        config.HardwareLimitSwitch.ReverseLimitSource = ctre::phoenix6::signals::ReverseLimitSourceValue::RemoteCANdiS1;

        if (saveImmediately) getMotor()->GetConfigurator().Apply(config.HardwareLimitSwitch);
    }

    void setupForwardHardwareLimit(int canID, ctre::phoenix6::signals::ForwardLimitTypeValue type, units::turn_t autosetPosition = 0_tr, bool saveImmediately = false) {
        config.HardwareLimitSwitch.ForwardLimitType = type;
        config.HardwareLimitSwitch.ForwardLimitEnable = true;
        config.HardwareLimitSwitch.ForwardLimitRemoteSensorID = canID;
        config.HardwareLimitSwitch.ForwardLimitAutosetPositionValue = autosetPosition;
        config.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = true;
        config.HardwareLimitSwitch.ForwardLimitSource = ctre::phoenix6::signals::ReverseLimitSourceValue::RemoteCANdiS1;

        if (saveImmediately) getMotor()->GetConfigurator().Apply(config.HardwareLimitSwitch);
    }

    void setForwardLimit(units::turn_t forward, bool saveImmediately = false) override {
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = forward;

        if (saveImmediately) getMotor()->GetConfigurator().Apply(config.SoftwareLimitSwitch);
    }

    void setReverseLimit(units::turn_t reverse, bool saveImmediately = false) override {
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = reverse;

        if (saveImmediately) getMotor()->GetConfigurator().Apply(config.SoftwareLimitSwitch);
    }
    
    void setGearRatios(double _rotorToSensor, double _sensorToMech, bool saveImmediately = false) override {
        rotorToSensor = _rotorToSensor;
        sensorToMech = _sensorToMech;

        config.Feedback.RotorToSensorRatio = rotorToSensor;
        config.Feedback.SensorToMechanismRatio = sensorToMech;

        if (saveImmediately)
            getMotor()->GetConfigurator().Apply(config.Feedback);
    }

    ctre::phoenix6::signals::MagnetHealthValue getMagnetHealth() {
        if (!cancoder) return ctre::phoenix6::signals::MagnetHealthValue::Magnet_Invalid;
        return cancoder->GetMagnetHealth().GetValue();
    }

    units::turn_t getAbsEncoderPosition() {
        if (cancoder == nullptr) return 0_tr;
        return cancoder->GetAbsolutePosition().GetValue();
    }

    void setupCANCoder(int deviceId, units::turn_t offset, bool clockwise, std::string canbus = "", units::turn_t absoluteRange=1_tr, bool saveImmediately = false) override {
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

        if (saveImmediately) getMotor()->GetConfigurator().Apply(config.Feedback);
    }

    ctre::phoenix6::hardware::CANcoder *getCANCoder() override {
        return cancoder;
    }

    float getBusUtil(const char* canBusName) {
        // @todo Initialize a CANBus, and get utilization
        return 0;
    }

    void setOpenLoopRamp(units::second_t time, bool saveImmediately = false) override {
        config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = time;
        if (saveImmediately) getMotor()->GetConfigurator().Apply(config.OpenLoopRamps);
    }

    units::frequency::hertz_t getPositionUpdateFrequency() {
        // Should we return CANcoder vs motor position update frequency
        return res_position.GetAppliedUpdateFrequency();
    }

    units::frequency::hertz_t getSpeedUpdateFrequency() {
        return res_velocity.GetAppliedUpdateFrequency();
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

    void InitSendable(wpi::SendableBuilder& builder) override {
        builder.SetSmartDashboardType("Subsystem");
        builder.AddDoubleProperty(
            "Stator Current", 
            [this] { return getMotor()->GetStatorCurrent().GetValueAsDouble(); },
            nullptr);
        builder.AddDoubleProperty(
            "Supply Current", 
            [this] { return getMotor()->GetSupplyCurrent().GetValueAsDouble(); },
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
            [this] { return getMotor()->GetMotorVoltage().GetValueAsDouble(); },
            nullptr);
        builder.AddDoubleProperty(
            "reqPosition", 
            [this] { return req_pos_out.Position.template to<double>(); },
            nullptr);
        builder.AddDoubleProperty(
            "reqSpeed", 
            [this] { return req_vel_out.Velocity.template to<double>(); },
            nullptr);
        builder.AddStringProperty(
            "Magnet Health",
            [this] { return getMagnetHealth().ToString(); },
            nullptr);
        builder.AddBooleanProperty(
            "Undervolting",
            [this] { return getMotor()->GetFault_Undervoltage().GetValue(); },
            nullptr);
    }

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
    void setPIDFSlot(S& slot, bool saveImmediately) {
        // Generic PIDF configurations
        // Numerator for closed loop controls will be in volts
        slot.kP = pidf.P;
        slot.kI = pidf.I;
        slot.kD = pidf.D;
        slot.kS = pidf.S;
        slot.kV = 0;
        if constexpr (std::is_same_v<RawOutputUnit, units::volt_t>) {
            if (pidf.kV < 0)
                slot.kV = (voltageCompenstation / getMaxMechSpeed()).value();
            else
                slot.kV = pidf.kV;
        } else if constexpr (std::is_same_v<RawOutputUnit, units::ampere_t>) {
            if (pidf.kV < 0)
                slot.kV = 0;
            else
                slot.kV = pidf.kV;
        }

        // Feedforward gain configuration
        if (pidf.aFF != 0) {
            slot.GravityType = pidf.aFFType == valor::FeedForwardType::LINEAR ?
                ctre::phoenix6::signals::GravityTypeValue::Elevator_Static :
                ctre::phoenix6::signals::GravityTypeValue::Arm_Cosine;
            slot.kG = pidf.aFF;
        }

        if (saveImmediately) getMotor()->GetConfigurator().Apply(slot);
    }

    // Helper function for enableFOC that only enables if the template has the field "EnableFOC"
    template <typename T>
    static inline void tryEnableFOC(T& request, bool enableFOC) {
        constexpr bool hasEnableFOC = requires (T& t) {
            t.EnableFOC;
        };
        if constexpr (hasEnableFOC) request.EnableFOC = enableFOC;
    }

    valor::PIDF pidf;

    RawOutput req_raw_out;
    VelocityOutput req_vel_out;
    PositionOutput req_pos_out;

    ctre::phoenix6::hardware::CANcoder *cancoder;

    ctre::phoenix6::StatusSignal<units::turn_t>& res_position;
    ctre::phoenix6::StatusSignal<units::turns_per_second_t>& res_velocity;

    ctre::phoenix6::configs::TalonFXConfiguration config;
};
}

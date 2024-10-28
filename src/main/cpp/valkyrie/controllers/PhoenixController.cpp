#include "valkyrie/controllers/PhoenixController.h"

// Conversion guide: https://v6.docs.ctr-electronics.com/en/latest/docs/migration/migration-guide/closed-loop-guide.html

#define FALCON_PIDF_KP 10.0f
#define FALCON_PIDF_KI 0.0f
#define FALCON_PIDF_KD 0.0f

const units::turns_per_second_t FALCON_PIDF_KV(6); // RPS cruise velocity
const units::turns_per_second_squared_t FALCON_PIDF_KA(130.0); // RPS/S acceleration (6.5/130 = 0.05 seconds to max speed)
const units::turns_per_second_cubed_t FALCON_PIDF_KJ(650.0); // RPS/S^2 jerk (4000/40000 = 0.1 seconds to max acceleration)

const units::ampere_t SUPPLY_CURRENT_THRESHOLD(60);
const units::ampere_t STATOR_CURRENT_LIMIT(80);
const units::ampere_t SUPPLY_CURRENT_LIMIT(45);
const units::millisecond_t SUPPLY_TIME_THRESHOLD(500);

const units::turn_t DEADBAND(0.01);

using namespace valor;
using namespace ctre::phoenix6;



PhoenixController::PhoenixController(valor::PhoenixControllerType controllerType,
                                    int canID,
                                    valor::NeutralMode _mode,
                                    bool _inverted,
                                    std::string canbus) :
    BaseController(new hardware::TalonFX{canID, canbus}, _inverted, _mode, getPhoenixControllerMotorSpeed(controllerType)),
    req_position(units::turn_t{0}),
    req_velocity(units::turns_per_second_t{0}),
    req_voltage(units::volt_t{0}),
    cancoder(nullptr),
    res_position(getMotor()->GetPosition()),
    res_velocity(getMotor()->GetVelocity())
{
    init();
}

PhoenixController::PhoenixController(valor::PhoenixControllerType controllerType,
                                    int canID,
                                    valor::NeutralMode _mode,
                                    bool _inverted,
                                    double _rotorToSensor,
                                    double _sensorToMech,
                                    valor::PIDF pidf,
                                    std::string canbus) :
    BaseController(new hardware::TalonFX{canID, canbus}, _inverted, _mode, getPhoenixControllerMotorSpeed(controllerType)),
    req_position(units::turn_t{0}),
    req_velocity(units::turns_per_second_t{0}),
    req_voltage(units::volt_t{0}),
    res_position(getMotor()->GetPosition()),
    res_velocity(getMotor()->GetVelocity())
{
    init(_rotorToSensor, _sensorToMech, pidf);
}

void PhoenixController::init()
{
    valor::PIDF motionPIDF;
    motionPIDF.P = FALCON_PIDF_KP;
    motionPIDF.I = FALCON_PIDF_KI;
    motionPIDF.D = FALCON_PIDF_KD;
    motionPIDF.error = 0_tr;
    motionPIDF.maxVelocity = FALCON_PIDF_KV;
    motionPIDF.maxAcceleration = FALCON_PIDF_KA;

    init(1, 1, motionPIDF);
}

void PhoenixController::init(double _rotorToSensor, double _sensorToMech, valor::PIDF pidf)
{
    req_position.Slot = 0;
    req_position.UpdateFreqHz = 0_Hz;
    req_velocity.Slot = 0;
    req_velocity.UpdateFreqHz = 0_Hz;
    
    configs::TalonFXConfiguration config;

    setNeutralMode(config.MotorOutput, neutralMode);

    // Current limiting configuration
    config.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerTime = SUPPLY_TIME_THRESHOLD;
    config.CurrentLimits.SupplyCurrentLowerLimit = SUPPLY_CURRENT_THRESHOLD;

    setGearRatios(config.Feedback, rotorToSensor, sensorToMech);
    setPIDF(config.Slot0, config.MotionMagic, pidf);

    getMotor()->GetConfigurator().Apply(config, units::second_t{5});

    wpi::SendableRegistry::AddLW(this, "PhoenixController", "ID " + std::to_string(getMotor()->GetDeviceID()));
}

void PhoenixController::setupCANCoder(int deviceId, units::turn_t offset, bool clockwise, std::string canbus, ctre::phoenix6::signals::AbsoluteSensorRangeValue absoluteRange)
{
    cancoder = new ctre::phoenix6::hardware::CANcoder(deviceId, canbus);
    ctre::phoenix6::configs::MagnetSensorConfigs config;
    config.AbsoluteSensorRange = absoluteRange;
    config.SensorDirection = clockwise ? signals::SensorDirectionValue::Clockwise_Positive :
                                         signals::SensorDirectionValue::CounterClockwise_Positive;
    config.MagnetOffset = -offset;
    cancoder->GetConfigurator().Apply(config);

    ctre::phoenix6::configs::FeedbackConfigs fx_cfg{};
    fx_cfg.FeedbackRemoteSensorID = cancoder->GetDeviceID();
    fx_cfg.FeedbackSensorSource = signals::FeedbackSensorSourceValue::FusedCANcoder;
    fx_cfg.SensorToMechanismRatio = sensorToMech;
    fx_cfg.RotorToSensorRatio = rotorToSensor;
    getMotor()->GetConfigurator().Apply(fx_cfg);
}

void PhoenixController::applyConfig()
{
    //used to apply current config to pheonix device
    //new comment
}

units::turn_t PhoenixController::getCANCoder()
{
    return cancoder ? cancoder->GetAbsolutePosition().GetValue() : 0_tr;
}

void PhoenixController::reset()
{
    getMotor()->SetPosition(0_tr);
}

void PhoenixController::setEncoderPosition(units::turn_t position)
{
    getMotor()->SetPosition(position);
}

void PhoenixController::setupFollower(int canID, bool followerInverted)
{
    followerMotor = new hardware::TalonFX(canID, "baseCAN");
    followerMotor->SetInverted(followerInverted);
    followerMotor->SetNeutralMode(signals::NeutralModeValue::Coast);
    followerMotor->SetControl(controls::StrictFollower{getMotor()->GetDeviceID()});
}

void PhoenixController::setForwardLimit(units::turn_t forward)
{
    configs::SoftwareLimitSwitchConfigs config{};
    config.ForwardSoftLimitEnable = true;
    config.ForwardSoftLimitThreshold = forward;
    getMotor()->GetConfigurator().Apply(config);
}

void PhoenixController::setReverseLimit(units::turn_t reverse)
{
    configs::SoftwareLimitSwitchConfigs config{};
    config.ReverseSoftLimitEnable = true;
    config.ReverseSoftLimitThreshold = reverse;
    getMotor()->GetConfigurator().Apply(config);
}


void PhoenixController::setPIDF(valor::PIDF _pidf, int slot)
{
    configs::Slot0Configs slotConfig{};
    configs::MotionMagicConfigs motionMagicConfig{};
    setPIDF(slotConfig, motionMagicConfig, _pidf);
    getMotor()->GetConfigurator().Apply(slotConfig);
    getMotor()->GetConfigurator().Apply(motionMagicConfig);
}

void PhoenixController::setPIDF(configs::Slot0Configs & slotConfig, configs::MotionMagicConfigs & motionMagicConfig, valor::PIDF _pidf)
{
    pidf = _pidf;

    // Generic PIDF configurations
    // Numerator for closed loop controls will be in volts
    // Feedback and feedforward gains are in volts / rpm of motor, NOT mechanism
    slotConfig.kP = pidf.P;
    slotConfig.kI = pidf.I;
    slotConfig.kD = pidf.D;
    slotConfig.kV = voltageCompenstation / (maxMotorSpeed / (rotorToSensor * sensorToMech));
    slotConfig.kS = pidf.S;

    // Feedforward gain configuration
    if (pidf.aFF != 0) {
        slotConfig.GravityType = pidf.aFFType == valor::FeedForwardType::LINEAR ?
            signals::GravityTypeValue::Elevator_Static :
            signals::GravityTypeValue::Arm_Cosine;
        slotConfig.kG = pidf.aFF;
    }

    // Motion magic configuration
    motionMagicConfig.MotionMagicCruiseVelocity = pidf.maxVelocity;
    motionMagicConfig.MotionMagicAcceleration = pidf.maxAcceleration;
    motionMagicConfig.MotionMagicJerk = pidf.maxJerk;
}

void PhoenixController::setGearRatios(double _rotorToSensor, double _sensorToMech)
{
    configs::FeedbackConfigs config{};
    setGearRatios(config, _rotorToSensor, _sensorToMech);
    getMotor()->GetConfigurator().Apply(config);
}

void PhoenixController::setGearRatios(configs::FeedbackConfigs & config, double _rotorToSensor, double _sensorToMech)
{
    rotorToSensor = _rotorToSensor;
    sensorToMech = _sensorToMech;
    config.RotorToSensorRatio = rotorToSensor;
    config.SensorToMechanismRatio = sensorToMech;
}

units::ampere_t PhoenixController::getCurrent()
{
    return getMotor()->GetStatorCurrent().GetValue();
}

/**
 * Output is in mechanism rotations!
*/
units::turn_t PhoenixController::getPosition()
{
    // @TODO Use FPGA - latency to identify timestamp of calculation
    // units::second_t latency = rotorPosSignal.GetTimestamp().GetLatency();
    return res_position.Refresh().GetValue();
}

/**
 * Output is in mechanism rotations!
*/
units::turns_per_second_t PhoenixController::getSpeed()
{
    // @TODO Use FPGA - latency to identify timestamp of calculation
    // units::second_t latency = rotorPosSignal.GetTimestamp().GetLatency();
    return res_velocity.Refresh().GetValue();
}

// Sets signal update rate for position
void PhoenixController::setPositionUpdateFrequency(units::frequency::hertz_t hertz)
{
    res_position.SetUpdateFrequency(hertz);
}

// Sets signal update rate for speed
void PhoenixController::setSpeedUpdateFrequency(units::frequency::hertz_t hertz)
{
    res_velocity.SetUpdateFrequency(hertz);
}

/**
 * Set a position in mechanism rotations
*/
void PhoenixController::setPosition(units::turn_t position)
{
    req_position.Position = position; // Mechanism rotations
    getMotor()->SetControl(req_position);
}

void PhoenixController::enableFOC(bool enableFOC)
{
    req_position.EnableFOC = enableFOC;
    req_velocity.EnableFOC = enableFOC;
    req_voltage.EnableFOC = enableFOC;
}

void PhoenixController::setSpeed(units::turns_per_second_t speed)
{
    req_velocity.Velocity = speed; // Mechanism rotations
    getMotor()->SetControl(req_velocity);
}

void PhoenixController::setPower(units::volt_t voltage)
{
    req_voltage.Output = voltage;
    getMotor()->SetControl(req_voltage);
}

void PhoenixController::setProfile(int profile)
{
    currentProfile = profile;
}

units::turn_t PhoenixController::getAbsEncoderPosition()
{
    return 0_tr;
}

void PhoenixController::setNeutralMode(configs::MotorOutputConfigs & config, valor::NeutralMode mode)
{
    neutralMode = mode;

    // Deadband configuration
    config.DutyCycleNeutralDeadband = DEADBAND;
    config.Inverted = inverted;
    config.NeutralMode = neutralMode == valor::NeutralMode::Brake ?
        signals::NeutralModeValue::Brake :
        signals::NeutralModeValue::Coast;
}

void PhoenixController::setNeutralMode(valor::NeutralMode mode)
{
    configs::MotorOutputConfigs config{};
    setNeutralMode(config, mode);
    getMotor()->GetConfigurator().Apply(config);
}

void PhoenixController::setOpenLoopRamp(units::second_t time)
{
    configs::OpenLoopRampsConfigs config{};
    config.DutyCycleOpenLoopRampPeriod = time;
    getMotor()->GetConfigurator().Apply(config);
}

float PhoenixController::getRevBusUtil()
{
    return CANBus::GetStatus("").BusUtilization;
}

float PhoenixController::getCANivoreBusUtil()
{
    return CANBus::GetStatus("baseCAN").BusUtilization;
}

ctre::phoenix6::signals::MagnetHealthValue PhoenixController::getMagnetHealth()
{
    return cancoder->GetMagnetHealth().GetValue();
}

void PhoenixController::InitSendable(wpi::SendableBuilder& builder)
{
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
        "Device Temp", 
        [this] { return getMotor()->GetDeviceTemp().GetValueAsDouble(); },
        nullptr);
    builder.AddDoubleProperty(
        "Processor Temp", 
        [this] { return getMotor()->GetProcessorTemp().GetValueAsDouble(); },
        nullptr);
    builder.AddDoubleProperty(
        "Position", 
        [this] { return getPosition().to<double>(); },
        nullptr);
    builder.AddDoubleProperty(
        "Speed", 
        [this] { return getSpeed().to<double>(); },
        nullptr);
    builder.AddDoubleProperty(
        "Out Volt", 
        [this] { return getMotor()->GetMotorVoltage().GetValueAsDouble(); },
        nullptr);
    builder.AddDoubleProperty(
        "CANCoder", 
        [this] { return getCANCoder().to<double>(); },
        nullptr);
    builder.AddDoubleProperty(
        "reqPosition", 
        [this] { return req_position.Position.to<double>(); },
        nullptr);
    builder.AddDoubleProperty(
        "reqSpeed", 
        [this] { return req_velocity.Velocity.to<double>(); },
        nullptr);
    builder.AddIntegerProperty(
        "Magnet Health",
        [this] { return cancoder ? cancoder->GetMagnetHealth().GetValue().value : -1; },
        nullptr);
    builder.AddFloatProperty(
        "Rev CAN Bus Utilization",
        [this] { return getRevBusUtil(); },
        nullptr
    );
    builder.AddFloatProperty(
        "CANivore Bus Utilization",
        [this] { return getCANivoreBusUtil(); },
        nullptr
    );
    builder.AddBooleanProperty(
        "Undervolting",
        [this] { return getMotor()->GetFault_Undervoltage().GetValue(); },
        nullptr);
    builder.AddIntegerProperty(
        "Device ID",
        [this] { return getMotor()->GetDeviceID(); },
        nullptr
        );
}

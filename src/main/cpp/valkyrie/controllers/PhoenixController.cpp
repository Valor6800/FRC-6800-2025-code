#include "valkyrie/controllers/PhoenixController.h"

// Conversion guide: https://v6.docs.ctr-electronics.com/en/latest/docs/migration/migration-guide/closed-loop-guide.html

#define FALCON_PIDF_KP 10.0f
#define FALCON_PIDF_KI 0.0f
#define FALCON_PIDF_KD 0.0f

#define FALCON_PIDF_KV 6.0f // RPS cruise velocity
#define FALCON_PIDF_KA 130.0f // RPS/S acceleration (6.5/130 = 0.05 seconds to max speed)
#define FALCON_PIDF_KJ 650.0f // RPS/S^2 jerk (4000/40000 = 0.1 seconds to max acceleration)

#define SUPPLY_CURRENT_THRESHOLD 60.0f
#define STATOR_CURRENT_LIMIT 80.0f
#define SUPPLY_CURRENT_LIMIT 45.0f
#define SUPPLY_TIME_THRESHOLD 0.5f

#define FALCON_DEADBAND 0.01f

using namespace valor;
using namespace ctre::phoenix6;

PhoenixController::PhoenixController(int canID,
                                             valor::NeutralMode _mode,
                                             bool _inverted,
                                             std::string canbus) :
    BaseController(new hardware::TalonFX{canID, canbus}, _inverted, _mode, 6380),
    status(),
    req_position(units::turn_t{0}),
    req_velocity(units::turns_per_second_t{0}),
    req_voltage(units::volt_t{0}),
    voltageCompenstation(12.0),
    cancoder(nullptr),
    res_position(motor->GetPosition()),
    res_velocity(motor->GetVelocity())
{
    init();
}

PhoenixController::PhoenixController(int canID,
                                             valor::NeutralMode _mode,
                                             bool _inverted,
                                             double _rotorToSensor,
                                             double _sensorToMech,
                                             valor::PIDF pidf,
                                             double voltageComp,
                                             bool isKraken, 
                                             std::string canbus) :
    BaseController(new hardware::TalonFX{canID, canbus}, _inverted, _mode, isKraken ? 5800 : 6380),
    status(),
    req_position(units::turn_t{0}),
    req_velocity(units::turns_per_second_t{0}),
    req_voltage(units::volt_t{0}),
    voltageCompenstation(voltageComp),
    res_position(motor->GetPosition()),
    res_velocity(motor->GetVelocity())
{
    init(_rotorToSensor, _sensorToMech, pidf);
}


void PhoenixController::init()
{
    valor::PIDF motionPIDF;
    motionPIDF.P = FALCON_PIDF_KP;
    motionPIDF.I = FALCON_PIDF_KI;
    motionPIDF.D = FALCON_PIDF_KD;
    motionPIDF.error = 0;
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
    config.CurrentLimits.SupplyTimeThreshold = SUPPLY_TIME_THRESHOLD;
    config.CurrentLimits.SupplyCurrentThreshold = SUPPLY_CURRENT_THRESHOLD;

    setConversion(config.Feedback, _rotorToSensor, _sensorToMech);
    setPIDF(config.Slot0, config.MotionMagic, pidf);

    auto _status = motor->GetConfigurator().Apply(config, units::second_t{5});
    if (_status.IsError()) status = _status;

    wpi::SendableRegistry::AddLW(this, "PhoenixController", "ID " + std::to_string(motor->GetDeviceID()));
}

void PhoenixController::setupCANCoder(int deviceId, double offset, bool clockwise, std::string canbus, ctre::phoenix6::signals::AbsoluteSensorRangeValue absoluteRange)
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
    auto _status = motor->GetConfigurator().Apply(fx_cfg);
    if (_status.IsError()) status = _status;
}

double PhoenixController::getCANCoder()
{
    return cancoder ? cancoder->GetAbsolutePosition().GetValueAsDouble() : 0;
}

void PhoenixController::reset()
{
    motor->SetPosition(0_tr);
}

void PhoenixController::setVoltageCompensation(double volts)
{
    voltageCompenstation = volts;
}

void PhoenixController::setEncoderPosition(double position)
{
    motor->SetPosition(units::make_unit<units::turn_t>(position));
}

void PhoenixController::setupFollower(int canID, bool followerInverted)
{
    followerMotor = new hardware::TalonFX(canID, "baseCAN");
    // configs::TalonFXConfiguration config;
    //
    // setNeutralMode(config.MotorOutput, valor::NeutralMode::Coast);
    //
    // setConversion(config.Feedback, rotorToSensor, sensorToMech);
    //
    // auto _status = followerMotor->GetConfigurator().Apply(config, units::second_t{5});
    followerMotor->SetInverted(followerInverted);
    followerMotor->SetNeutralMode(signals::NeutralModeValue::Coast);
    followerMotor->SetControl(controls::StrictFollower{motor->GetDeviceID()});
}

void PhoenixController::setForwardLimit(double forward)
{
    configs::SoftwareLimitSwitchConfigs config{};
    config.ForwardSoftLimitEnable = true;
    config.ForwardSoftLimitThreshold = forward;
    auto _status = motor->GetConfigurator().Apply(config);
    if (_status.IsError()) status = _status;
}

void PhoenixController::setReverseLimit(double reverse)
{
    configs::SoftwareLimitSwitchConfigs config{};
    config.ReverseSoftLimitEnable = true;
    config.ReverseSoftLimitThreshold = reverse;
    auto _status = motor->GetConfigurator().Apply(config);
    if (_status.IsError()) status = _status;
}


void PhoenixController::setPIDF(valor::PIDF _pidf, int slot)
{
    configs::Slot0Configs slotConfig{};
    configs::MotionMagicConfigs motionMagicConfig{};
    setPIDF(slotConfig, motionMagicConfig, _pidf);
    auto _status =  motor->GetConfigurator().Apply(slotConfig);
    if (_status.IsError()) status = _status;
    _status = motor->GetConfigurator().Apply(motionMagicConfig);
    if (_status.IsError()) status = _status;
}

void PhoenixController::setPIDF(configs::Slot0Configs & slotConfig, configs::MotionMagicConfigs & motionMagicConfig, valor::PIDF _pidf)
{
    pidf = _pidf;

    // config.ClosedLoopGeneral.ContinuousWrap = true;

    // Generic PIDF configurations
    // Numerator for closed loop controls will be in volts
    // Feedback and feedforward gains are in volts / rpm of motor, NOT mechanism
    slotConfig.kP = pidf.P;
    slotConfig.kI = pidf.I;
    slotConfig.kD = pidf.D;
    slotConfig.kV = voltageCompenstation / (maxMotorSpeed / 60.0 / (rotorToSensor * sensorToMech));
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

void PhoenixController::setConversion(double _rotorToSensor, double _sensorToMech)
{
    configs::FeedbackConfigs config{};
    setConversion(config, _rotorToSensor, _sensorToMech);
    auto _status = motor->GetConfigurator().Apply(config);
    if (_status.IsError()) status = _status;
}

void PhoenixController::setConversion(configs::FeedbackConfigs & config, double _rotorToSensor, double _sensorToMech)
{
    rotorToSensor = _rotorToSensor;
    sensorToMech = _sensorToMech;
    config.RotorToSensorRatio = rotorToSensor;
    config.SensorToMechanismRatio = sensorToMech;
}

void PhoenixController::setMotorInversion(bool invert){
    motor->SetInverted(invert);
}

bool PhoenixController::getMotorInversion(){
    return motor->GetInverted();
}

double PhoenixController::getCurrent()
{
    return motor->GetStatorCurrent().GetValueAsDouble();
}

/**
 * Output is in mechanism rotations!
*/
double PhoenixController::getPosition()
{
    // @TODO Use FPGA - latency to identify timestamp of calculation
    // units::second_t latency = rotorPosSignal.GetTimestamp().GetLatency();
    return res_position.Refresh().GetValueAsDouble();
}

/**
 * Output is in mechanism rotations!
*/
double PhoenixController::getSpeed()
{
    // @TODO Use FPGA - latency to identify timestamp of calculation
    // units::second_t latency = rotorPosSignal.GetTimestamp().GetLatency();
    return res_velocity.Refresh().GetValueAsDouble();
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

void PhoenixController::setRange(int slot, double min, double max)
{
    configs::SoftwareLimitSwitchConfigs config{};
    config.ForwardSoftLimitEnable = true;
    config.ForwardSoftLimitThreshold = max;
    config.ReverseSoftLimitEnable = true;
    config.ReverseSoftLimitThreshold = min;
    auto _status = motor->GetConfigurator().Apply(config);
    if (_status.IsError()) status = _status;
}

/**
 * Set a position in mechanism rotations
*/
void PhoenixController::setPosition(double position)
{
    req_position.Position = units::make_unit<units::turn_t>(position); // Mechanism rotations
    auto _status = motor->SetControl(req_position);
    if (_status.IsError()) status = _status;
}

void PhoenixController::enableFOC(bool enableFOC)
{
    req_position.EnableFOC = enableFOC;
    req_velocity.EnableFOC = enableFOC;
    req_voltage.EnableFOC = enableFOC;
}

void PhoenixController::setSpeed(double speed)
{
    req_velocity.Velocity = units::make_unit<units::turns_per_second_t>(speed); // Mechanism rotations
    auto _status = motor->SetControl(req_velocity);
    if (_status.IsError()) status = _status;
}

void PhoenixController::setPower(double speed)
{
    req_voltage.Output = units::make_unit<units::volt_t>(speed * 12);
    auto _status = motor->SetControl(req_voltage);
    if (_status.IsError()) status = _status;
}

void PhoenixController::setVoltage(units::volt_t voltage)
{
    auto _status = motor->SetControl(voltage);
    if(_status.IsError()) status = _status;
}

units::volt_t PhoenixController::getVoltage(){
    return units::make_unit<units::volt_t>(motor->GetMotorVoltage().GetValueAsDouble());
}

void PhoenixController::setProfile(int profile)
{
    currentProfile = profile;
}

double PhoenixController::getAbsEncoderPosition()
{
    return 0;
}

void PhoenixController::setNeutralMode(configs::MotorOutputConfigs & config, valor::NeutralMode mode)
{
    neutralMode = mode;

    // Deadband configuration
    config.DutyCycleNeutralDeadband = FALCON_DEADBAND;
    config.Inverted = inverted;
    config.NeutralMode = neutralMode == valor::NeutralMode::Brake ?
        signals::NeutralModeValue::Brake :
        signals::NeutralModeValue::Coast;
}

void PhoenixController::setNeutralMode(valor::NeutralMode mode)
{
    configs::MotorOutputConfigs config{};
    setNeutralMode(config, mode);
    auto _status = motor->GetConfigurator().Apply(config);
    if (_status.IsError()) status = _status;
}

void PhoenixController::setOpenLoopRamp(double time)
{
    configs::OpenLoopRampsConfigs config{};
    config.DutyCycleOpenLoopRampPeriod = time;
    auto _status = motor->GetConfigurator().Apply(config);
    if (_status.IsError()) status = _status;
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
        [this] { return motor->GetStatorCurrent().GetValueAsDouble(); },
        nullptr);
    builder.AddDoubleProperty(
        "Supply Current", 
        [this] { return motor->GetSupplyCurrent().GetValueAsDouble(); },
        nullptr);
    builder.AddDoubleProperty(
        "Device Temp", 
        [this] { return motor->GetDeviceTemp().GetValueAsDouble(); },
        nullptr);
    builder.AddDoubleProperty(
        "Processor Temp", 
        [this] { return motor->GetProcessorTemp().GetValueAsDouble(); },
        nullptr);
    builder.AddDoubleProperty(
        "Position", 
        [this] { return getPosition(); },
        nullptr);
    builder.AddDoubleProperty(
        "Position (degrees)", 
        [this] { return getPosition() * 360.0; },
        nullptr);
    builder.AddDoubleProperty(
        "Speed", 
        [this] { return getSpeed(); },
        nullptr);
    builder.AddBooleanProperty(
        "Inverted", 
        [this] { return inverted; },
        nullptr);

    builder.AddDoubleProperty(
        "Out Volt", 
        [this] { return motor->GetMotorVoltage().GetValueAsDouble(); },
        nullptr);
    builder.AddDoubleProperty(
        "CANCoder", 
        [this] { return getCANCoder(); },
        nullptr);
    builder.AddDoubleProperty(
        "reqPosition", 
        [this] { return req_position.Position.to<double>(); },
        nullptr);
    builder.AddDoubleProperty(
        "reqSpeed", 
        [this] { return req_velocity.Velocity.to<double>(); },
        nullptr);
    builder.AddDoubleProperty(
        "rotorToSensor", 
        [this] { return rotorToSensor; },
        nullptr);
    builder.AddDoubleProperty(
        "sensorToMech", 
        [this] { return sensorToMech; },
        nullptr);

    builder.AddIntegerProperty(
        "Config Status Code", 
        [this] { return status; },
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
        [this] { return motor->GetFault_Undervoltage().GetValue(); },
        nullptr);
    builder.AddIntegerProperty(
        "Device ID",
        [this] { return motor->GetDeviceID(); },
        nullptr
        );
}

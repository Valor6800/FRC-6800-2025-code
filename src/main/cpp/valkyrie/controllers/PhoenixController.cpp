#include "valkyrie/controllers/PhoenixController.h"

// Conversion guide: https://v6.docs.ctr-electronics.com/en/latest/docs/migration/migration-guide/closed-loop-guide.html

#define FALCON_PIDF_KP 10.0f
#define FALCON_PIDF_KI 0.0f
#define FALCON_PIDF_KD 0.0f

const units::turns_per_second_t FALCON_PIDF_KV(6); // RPS cruise velocity
const units::turns_per_second_squared_t FALCON_PIDF_KA(130.0); // RPS/S acceleration (6.5/130 = 0.05 seconds to max speed)
const units::turns_per_second_cubed_t FALCON_PIDF_KJ(650.0); // RPS/S^2 jerk (4000/40000 = 0.1 seconds to max acceleration)

const units::ampere_t SUPPLY_CURRENT_THRESHOLD(45);
const units::ampere_t STATOR_CURRENT_LIMIT(80);
const units::ampere_t SUPPLY_CURRENT_LIMIT(60);
const units::millisecond_t SUPPLY_TIME_THRESHOLD(500);
const units::ampere_t PEAK_TORQUE_CURRENT(100);

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

void PhoenixController::init()
{
    valor::PIDF motionPIDF;
    motionPIDF.P = FALCON_PIDF_KP;
    motionPIDF.I = FALCON_PIDF_KI;
    motionPIDF.D = FALCON_PIDF_KD;
    motionPIDF.error = 0_tr;
    motionPIDF.maxVelocity = FALCON_PIDF_KV;
    motionPIDF.maxAcceleration = FALCON_PIDF_KA;

    req_position.Slot = 0;
    req_position.UpdateFreqHz = 0_Hz;
    req_velocity.Slot = 0;
    req_velocity.UpdateFreqHz = 0_Hz;

    setNeutralMode(neutralMode);
    setCurrentLimits(STATOR_CURRENT_LIMIT, SUPPLY_CURRENT_LIMIT, SUPPLY_CURRENT_THRESHOLD, SUPPLY_TIME_THRESHOLD);
    setGearRatios(rotorToSensor, sensorToMech);
    setPIDF(pidf, 0);

    wpi::SendableRegistry::AddLW(this, "PhoenixController", "ID " + std::to_string(getMotor()->GetDeviceID()));
}

void PhoenixController::setupCANCoder(int deviceId, units::turn_t offset, bool clockwise, std::string canbus, units::turn_t absoluteRange, bool saveImmediately)
{
    cancoder = new ctre::phoenix6::hardware::CANcoder(deviceId, canbus);
    
    ctre::phoenix6::configs::MagnetSensorConfigs cancoderConfig;
    cancoderConfig.AbsoluteSensorDiscontinuityPoint = absoluteRange;
    cancoderConfig.SensorDirection = clockwise ? signals::SensorDirectionValue::Clockwise_Positive :
                                         signals::SensorDirectionValue::CounterClockwise_Positive;
    cancoderConfig.MagnetOffset = -offset;
    cancoder->GetConfigurator().Apply(cancoderConfig);

    config.Feedback.FeedbackRemoteSensorID = cancoder->GetDeviceID();
    config.Feedback.FeedbackSensorSource = signals::FeedbackSensorSourceValue::FusedCANcoder;
    config.Feedback.SensorToMechanismRatio = sensorToMech;
    config.Feedback.RotorToSensorRatio = rotorToSensor;

    if (saveImmediately) {
        getMotor()->GetConfigurator().Apply(config.Feedback);
    }
}

void PhoenixController::setContinuousWrap(bool continuousWrap, bool saveImmediately)
{
    config.ClosedLoopGeneral.ContinuousWrap = continuousWrap;

    if (saveImmediately) {
        getMotor()->GetConfigurator().Apply(config.ClosedLoopGeneral);
    }
}

void PhoenixController::applyConfig()
{
    getMotor()->GetConfigurator().Apply(config, units::second_t{5});
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
    configs::MotorOutputConfigs config{};
    config.Inverted = followerInverted;
    config.NeutralMode = signals::NeutralModeValue::Coast;
    followerMotor->GetConfigurator().Apply(config);

    followerMotor->SetControl(controls::StrictFollower{getMotor()->GetDeviceID()});
}

void PhoenixController::setForwardLimit(units::turn_t forward, bool saveImmediately)
{
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = forward;

    if (saveImmediately) {
        getMotor()->GetConfigurator().Apply(config.SoftwareLimitSwitch);
    }
}

void PhoenixController::setReverseLimit(units::turn_t reverse, bool saveImmediately)
{
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = reverse;

    if (saveImmediately) {
        getMotor()->GetConfigurator().Apply(config.SoftwareLimitSwitch);
    }
}

void PhoenixController::setCurrentLimits(units::ampere_t statorCurrentLimit, units::ampere_t supplyCurrentLimit, units::ampere_t supplyCurrentThreshold, units::second_t supplyTimeThreshold, bool saveImmediately)
{
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

void PhoenixController::setPIDF(valor::PIDF _pidf, int slot, bool saveImmediately)
{
    pidf = _pidf;

    // Generic PIDF configurations
    // Numerator for closed loop controls will be in volts
    // Feedback and feedforward gains are in volts / rpm of motor, NOT mechanism
    config.Slot0.kP = pidf.P;
    config.Slot0.kI = pidf.I;
    config.Slot0.kD = pidf.D;
    config.Slot0.kV = (voltageCompenstation / (maxMechSpeed / (rotorToSensor * sensorToMech))).value();
    config.Slot0.kS = pidf.S;

    // Feedforward gain configuration
    if (pidf.aFF != 0) {
        config.Slot0.GravityType = pidf.aFFType == valor::FeedForwardType::LINEAR ?
            signals::GravityTypeValue::Elevator_Static :
            signals::GravityTypeValue::Arm_Cosine;
        config.Slot0.kG = pidf.aFF;
    }

    // Motion magic configuration
    config.MotionMagic.MotionMagicCruiseVelocity = pidf.maxVelocity;
    config.MotionMagic.MotionMagicAcceleration = pidf.maxAcceleration;
    config.MotionMagic.MotionMagicJerk = pidf.maxJerk;

    if (saveImmediately) {
        getMotor()->GetConfigurator().Apply(config.Slot0);
        getMotor()->GetConfigurator().Apply(config.MotionMagic);
    }
}

void PhoenixController::setGearRatios(double _rotorToSensor, double _sensorToMech, bool saveImmediately)
{
    rotorToSensor = _rotorToSensor;
    sensorToMech = _sensorToMech;

    maxMechSpeed /= (rotorToSensor * sensorToMech);

    config.Feedback.RotorToSensorRatio = rotorToSensor;
    config.Feedback.SensorToMechanismRatio = sensorToMech;

    if (saveImmediately)
        getMotor()->GetConfigurator().Apply(config.Feedback);
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
    // VelocityTorqueCurrentFOC requires FOC
    req_voltage.EnableFOC = enableFOC;
}

void PhoenixController::setSpeed(units::turns_per_second_t speed)
{
    req_velocity.Velocity = speed;
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

void PhoenixController::setNeutralMode(valor::NeutralMode mode, bool saveImmediately)
{
    neutralMode = mode;    
    config.MotorOutput.DutyCycleNeutralDeadband = DEADBAND.value();
    config.MotorOutput.Inverted = inverted;
    config.MotorOutput.NeutralMode = neutralMode == valor::NeutralMode::Brake ?
        signals::NeutralModeValue::Brake :
        signals::NeutralModeValue::Coast;

    if (saveImmediately) {
        getMotor()->GetConfigurator().Apply(config.MotorOutput);
    }
}

void PhoenixController::setOpenLoopRamp(units::second_t time, bool saveImmediately)
{
    config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = time;

    if (saveImmediately) {
        getMotor()->GetConfigurator().Apply(config.OpenLoopRamps);
    }
}

float PhoenixController::getBusUtil(const char* canBusName)
{
    // @todo Initialize a CANBus, and get utilization
    return 0;
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
        "CANivore Bus Utilization",
        [this] { return getBusUtil("baseCAN"); },
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
    builder.AddIntegerProperty(
        "RotorToSensor",
        [this] { return rotorToSensor; },
        nullptr);
    builder.AddIntegerProperty(
        "SensorToMech",
        [this] { return sensorToMech; },
        nullptr
        );
    builder.AddDoubleProperty(
        "Module Max Speed TPS",
        [this] {return getMaxMechSpeed().value();},
        nullptr
    );
}

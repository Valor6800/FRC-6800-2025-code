#include "valkyrie/controllers/NeoController.h"

#define NEO_PIDF_KP 10.0f
#define NEO_PIDF_KI 0.0f
#define NEO_PIDF_KD 0.0f

const units::turns_per_second_t NEO_PIDF_KV(6); // RPS cruise velocity
const units::turns_per_second_squared_t NEO_PIDF_KA(130.0); // RPS/S acceleration (6.5/130 = 0.05 seconds to max speed)
const units::turns_per_second_cubed_t NEO_PIDF_KJ(650.0); // RPS/S^2 jerk (4000/40000 = 0.1 seconds to max acceleration)

const units::ampere_t SUPPLY_CURRENT_THRESHOLD(60);
const units::ampere_t STATOR_CURRENT_LIMIT(80);
const units::ampere_t SUPPLY_CURRENT_LIMIT(45);
const units::millisecond_t SUPPLY_TIME_THRESHOLD(500);

const units::turn_t DEADBAND(0.01);

using namespace valor;


NeoController::NeoController(valor::NeoControllerType controllerType,
                             int canID,
                             valor::NeutralMode _mode,
                             bool _inverted) :
    BaseController{new rev::spark::SparkMax{canID, rev::spark::SparkMax::MotorType::kBrushless}, _inverted, _mode, getNeoControllerMotorSpeed(controllerType)}
{
    init();
    wpi::SendableRegistry::AddLW(this, "NeoController", "ID " + std::to_string(canID));
}

void NeoController::init() {
    valor::PIDF _pidf;
    _pidf.P = NEO_PIDF_KP;
    _pidf.I = NEO_PIDF_KI;
    _pidf.D = NEO_PIDF_KD;
    _pidf.error = 0_tr;
    _pidf.maxVelocity = NEO_PIDF_KV;
    _pidf.maxAcceleration = NEO_PIDF_KA;

    config.Inverted(inverted);

    setNeutralMode(neutralMode);
    setCurrentLimits(STATOR_CURRENT_LIMIT, SUPPLY_CURRENT_LIMIT, SUPPLY_CURRENT_THRESHOLD, SUPPLY_TIME_THRESHOLD);

    setGearRatios(rotorToSensor, sensorToMech);
    setPIDF(_pidf);
}

void NeoController::applyConfig()
{
    getMotor()->Configure(config, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
}

void NeoController::reset()
{
    setEncoderPosition(0_tr);
}

void NeoController::setEncoderPosition(units::turn_t position)
{
    getMotor()->GetEncoder().SetPosition(0);
}

void NeoController::setupFollower(int canID, bool followerInverted)
{
    followerMotor = new rev::spark::SparkMax{canID, rev::spark::SparkMax::MotorType::kBrushless};
    rev::spark::SparkBaseConfig followerConfig;
    followerConfig.Follow(*getMotor(), followerInverted);
    followerMotor->Configure(
        followerConfig,
        rev::spark::SparkMax::ResetMode::kResetSafeParameters,
        rev::spark::SparkMax::PersistMode::kNoPersistParameters
    );
}

void NeoController::setupCANCoder(int, units::turn_t zeroOffset, bool, std::string, units::turn_t, bool saveImmediately) {
    config.closedLoop.SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kAbsoluteEncoder);
    config.absoluteEncoder.ZeroOffset(zeroOffset.value());

    if (saveImmediately) applyConfig();
}

void NeoController::setForwardLimit(units::turn_t forward, bool saveImmediately)
{
    config.softLimit
        .ForwardSoftLimit(forward.value())
        .ForwardSoftLimitEnabled(true);
    if (saveImmediately) applyConfig();
}

void NeoController::setReverseLimit(units::turn_t reverse, bool saveImmediately)
{
    config.softLimit
        .ReverseSoftLimit(reverse.value())
        .ReverseSoftLimitEnabled(true);
    if (saveImmediately) applyConfig();
}

void NeoController::setCurrentLimits(units::ampere_t statorCurrentLimit, units::ampere_t supplyCurrentLimit, units::ampere_t supplyCurrentThreshold, units::second_t supplyTimeThreshold, bool saveImmediately)
{
    // Linear the entire RPM range
    config.SmartCurrentLimit(statorCurrentLimit.value(), statorCurrentLimit.value(), 0);
    if (saveImmediately) applyConfig();
}

void NeoController::setPIDF(valor::PIDF _pidf, int _slot, bool saveImmediately)
{
    pidf = _pidf;
    rev::spark::ClosedLoopSlot slot = rev::spark::ClosedLoopSlot::kSlot0;
    if (_slot == 1) slot = rev::spark::ClosedLoopSlot::kSlot1;
    else if (_slot == 2) slot = rev::spark::ClosedLoopSlot::kSlot2;
    else if (_slot == 3) slot = rev::spark::ClosedLoopSlot::kSlot3;
    if (pidf.kV < 0)
        pidf.kV = (voltageCompenstation / getMaxMechSpeed()).value();
    config.closedLoop.Pidf(pidf.P, pidf.I, pidf.D, pidf.kV, slot);
    config.closedLoop.maxMotion.MaxVelocity(units::revolutions_per_minute_t{pidf.maxVelocity}.value(), slot);
    config.closedLoop.maxMotion.MaxAcceleration(units::revolutions_per_minute_per_second_t{pidf.maxAcceleration}.value(), slot);
    config.closedLoop.maxMotion.AllowedClosedLoopError(pidf.error.value(), slot);

    if (saveImmediately) applyConfig();
}

void NeoController::setGearRatios(double _rotorToSensor, double _sensorToMech, bool saveImmediately)
{
    rotorToSensor = _rotorToSensor;
    sensorToMech = _sensorToMech;
    double conversion = 1 / (rotorToSensor * sensorToMech);

    config.encoder.PositionConversionFactor(conversion);
    config.encoder.VelocityConversionFactor(conversion);
    if (saveImmediately) applyConfig();
}

void NeoController::setContinuousWrap(bool continuousWrap, bool saveImmediately) {
    config.closedLoop.PositionWrappingEnabled(continuousWrap);
    config.closedLoop.PositionWrappingInputRange(-0.5, 0.5);
    if (saveImmediately) applyConfig();
}

units::ampere_t NeoController::getCurrent()
{
    return units::ampere_t{getMotor()->GetOutputCurrent()};
}

/**
 * Output is in mechanism rotations!
*/
units::turn_t NeoController::getPosition()
{
    switch (getMotor()->configAccessor.closedLoop.GetFeedbackSensor()) {
    case rev::spark::ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder: return units::turn_t{getMotor()->GetEncoder().GetPosition()};
    case rev::spark::ClosedLoopConfig::FeedbackSensor::kAbsoluteEncoder: return units::turn_t{getMotor()->GetAbsoluteEncoder().GetPosition()};
    case rev::spark::ClosedLoopConfig::FeedbackSensor::kAlternateOrExternalEncoder: return units::turn_t{getMotor()->GetAlternateEncoder().GetPosition()};
    case rev::spark::ClosedLoopConfig::FeedbackSensor::kAnalogSensor: return units::turn_t{getMotor()->GetAnalog().GetPosition()};
    default: return 0_tr;
    }
}

/**
 * Output is in mechanism rotations!
*/
units::turns_per_second_t NeoController::getSpeed()
{
    return units::revolutions_per_minute_t{getMotor()->GetEncoder().GetVelocity()};
}

/**
 * Set a position in mechanism rotations
*/
void NeoController::setPosition(units::turn_t position, int _slot)
{
    auto slot = rev::spark::ClosedLoopSlot::kSlot0;
    if (_slot == 1) slot = rev::spark::ClosedLoopSlot::kSlot1;
    else if (_slot == 2) slot = rev::spark::ClosedLoopSlot::kSlot2;
    else if (_slot == 3) slot = rev::spark::ClosedLoopSlot::kSlot3;
    getMotor()->GetClosedLoopController().SetReference(
        position.value(),
        rev::spark::SparkMax::ControlType::kPosition,
        slot,
        pidf.S
    );
}

void NeoController::setSpeed(units::turns_per_second_t speed, int _slot)
{
    auto slot = rev::spark::ClosedLoopSlot::kSlot0;
    if (_slot == 1) slot = rev::spark::ClosedLoopSlot::kSlot1;
    else if (_slot == 2) slot = rev::spark::ClosedLoopSlot::kSlot2;
    else if (_slot == 3) slot = rev::spark::ClosedLoopSlot::kSlot3;
    getMotor()->GetClosedLoopController().SetReference(
        speed.value(),
        rev::spark::SparkMax::ControlType::kVelocity,
        slot,
        pidf.S
    );
}

void NeoController::setPower(units::volt_t voltage) {
    getMotor()->SetVoltage(voltage);
}

void NeoController::setNeutralMode(valor::NeutralMode mode, bool saveImmediately)
{
    config.SetIdleMode(
        mode == valor::NeutralMode::Brake ?
        rev::spark::SparkBaseConfig::IdleMode::kBrake :
        rev::spark::SparkBaseConfig::IdleMode::kCoast
    );
    if (saveImmediately) applyConfig();
}

void NeoController::setOpenLoopRamp(units::second_t time, bool saveImmediately)
{
    config.OpenLoopRampRate(time.value());
    if (saveImmediately) applyConfig();
}

units::turn_t NeoController::getAbsEncoderPosition() {
    return units::turn_t{getMotor()->GetAbsoluteEncoder().GetPosition()};
}

void NeoController::InitSendable(wpi::SendableBuilder& builder)
{
    builder.SetSmartDashboardType("Subsystem");
    builder.AddDoubleProperty(
        "Stator Current", 
        [this] { return getMotor()->GetOutputCurrent(); },
        nullptr);
    // builder.AddDoubleProperty(
    //     "Supply Current", 
    //     [this] { return getMotor()->GetSupplyCurrent().GetValueAsDouble(); },
    //     nullptr);
    // builder.AddDoubleProperty(
    //     "Device Temp", 
    //     [this] { return getMotor()->GetDeviceTemp().GetValueAsDouble(); },
    //     nullptr);
    // builder.AddDoubleProperty(
    //     "Processor Temp", 
    //     [this] { return getMotor()->GetProcessorTemp().GetValueAsDouble(); },
    //     nullptr);
    builder.AddDoubleProperty(
        "Position", 
        [this] { return getPosition().to<double>(); },
        nullptr);
    builder.AddDoubleProperty(
        "Speed", 
        [this] { return getSpeed().to<double>(); },
        nullptr);
    // builder.AddDoubleProperty(
    //     "Out Volt", 
    //     [this] { return getMotor()->GetMotorVoltage().GetValueAsDouble(); },
    //     nullptr);
    // builder.AddDoubleProperty(
    //     "CANCoder", 
    //     [this] { return getCANCoder().to<double>(); },
    //     nullptr);
    // builder.AddBooleanProperty(
    //     "Undervolting",
    //     [this] { return getMotor()->GetFault_Undervoltage().GetValue(); },
    //     nullptr);
    // builder.AddIntegerProperty(
    //     "Device ID",
    //     [this] { return getMotor()->GetDeviceID(); },
    //     nullptr
    //     );
    // builder.AddIntegerProperty(
    //     "RotorToSensor",
    //     [this] { return rotorToSensor; },
    //     nullptr);
    // builder.AddIntegerProperty(
    //     "SensorToMech",
    //     [this] { return sensorToMech; },
    //     nullptr
    //     );
}

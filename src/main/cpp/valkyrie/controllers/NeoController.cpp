#include <valkyrie/controllers/NeoController.h>

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
                             valor::NeutralMode mode,
                             bool inverted,
                             double _rotorToSensor,
                             double _sensorToMech) :
    BaseController{getNeoControllerMotorSpeed(controllerType), _rotorToSensor, _sensorToMech},
    rev::spark::SparkMax{canID, rev::spark::SparkMax::MotorType::kBrushless}
{
    wpi::SendableRegistry::AddLW(this, "NeoController", "ID " + std::to_string(canID));

    pidf.P = NEO_PIDF_KP;
    pidf.I = NEO_PIDF_KI;
    pidf.D = NEO_PIDF_KD;
    pidf.maxVelocity = NEO_PIDF_KV;
    pidf.maxAcceleration = NEO_PIDF_KA;

    config.Inverted(inverted);

    config.SetIdleMode(
        mode == valor::NeutralMode::Brake ?
        rev::spark::SparkBaseConfig::IdleMode::kBrake :
        rev::spark::SparkBaseConfig::IdleMode::kCoast
    );

    double conversion = 1 / (rotorToSensor * sensorToMech);

    // The primary encoder is the one inside the motor so this is the only one that it makes sense to apply conversion to
    config.encoder.PositionConversionFactor(conversion);
    config.encoder.VelocityConversionFactor(conversion);

    config.SmartCurrentLimit(STATOR_CURRENT_LIMIT.value(), STATOR_CURRENT_LIMIT.value(), 0);
    setPIDF(pidf);

    applyConfig();
}

void NeoController::applyConfig() {
    Configure(config, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kNoPersistParameters);
}

void NeoController::reset() {
    setEncoderPosition(0_tr);
}

void NeoController::setEncoderPosition(units::turn_t pos) {
    using rev::spark::ClosedLoopConfig;
    switch (configAccessor.closedLoop.GetFeedbackSensor()) {
    case ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder: GetEncoder().SetPosition(pos.value()); break;
    case ClosedLoopConfig::FeedbackSensor::kAlternateOrExternalEncoder: GetAlternateEncoder().SetPosition(pos.value()); break;
    default: {} // Analog + Absolute encoders don't support setting positions
    }
}

void NeoController::setupFollower(int canID, bool followerInverted) {
    followerMotor = std::make_unique<rev::spark::SparkMax>(canID, rev::spark::SparkMax::MotorType::kBrushless);
    rev::spark::SparkBaseConfig followerConfig;
    followerConfig.Follow(*this, followerInverted);
    followerMotor->Configure(
        followerConfig,
        rev::spark::SparkMax::ResetMode::kResetSafeParameters,
        rev::spark::SparkMax::PersistMode::kNoPersistParameters
    );
}

void NeoController::setPIDF(valor::PIDF _pidf, int _slot, bool saveImmediately) {
    pidf = _pidf;
    rev::spark::ClosedLoopSlot slot = rev::spark::ClosedLoopSlot::kSlot0;
    if (_slot == 1) slot = rev::spark::ClosedLoopSlot::kSlot1;
    else if (_slot == 2) slot = rev::spark::ClosedLoopSlot::kSlot2;
    else if (_slot == 3) slot = rev::spark::ClosedLoopSlot::kSlot3;
    if (pidf.kV < 0)
        pidf.kV = (voltageCompensation / getMaxMechSpeed()).value();
    config.closedLoop.Pidf(pidf.P, pidf.I, pidf.D, pidf.kV, slot);
    config.closedLoop.maxMotion.MaxVelocity(units::revolutions_per_minute_t{pidf.maxVelocity}.value(), slot);
    config.closedLoop.maxMotion.MaxAcceleration(units::revolutions_per_minute_per_second_t{pidf.maxAcceleration}.value(), slot);
    config.closedLoop.maxMotion.AllowedClosedLoopError(pidf.error.value(), slot);

    if (saveImmediately) applyConfig();
}

/**
 * Output is in mechanism rotations!
*/
units::turn_t NeoController::getPosition() {
    using rev::spark::ClosedLoopConfig;
    switch (configAccessor.closedLoop.GetFeedbackSensor()) {
    case ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder: return units::turn_t{GetEncoder().GetPosition()};
    case ClosedLoopConfig::FeedbackSensor::kAbsoluteEncoder: return units::turn_t{GetAbsoluteEncoder().GetPosition()};
    case ClosedLoopConfig::FeedbackSensor::kAlternateOrExternalEncoder: return units::turn_t{GetAlternateEncoder().GetPosition()};
    case ClosedLoopConfig::FeedbackSensor::kAnalogSensor: return units::turn_t{GetAnalog().GetPosition()};
    default: return 0_tr;
    }
}

units::turns_per_second_t NeoController::getSpeed() {
    using rev::spark::ClosedLoopConfig;
    switch (configAccessor.closedLoop.GetFeedbackSensor()) {
    case ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder: return units::revolutions_per_minute_t{GetEncoder().GetVelocity()};
    case ClosedLoopConfig::FeedbackSensor::kAbsoluteEncoder: return units::revolutions_per_minute_t{GetAbsoluteEncoder().GetVelocity()};
    case ClosedLoopConfig::FeedbackSensor::kAlternateOrExternalEncoder: return units::revolutions_per_minute_t{GetAlternateEncoder().GetVelocity()};
    case ClosedLoopConfig::FeedbackSensor::kAnalogSensor: return units::revolutions_per_minute_t{GetAnalog().GetVelocity()};
    default: return 0_tps;
    };
}

/**
 * Set a position in mechanism rotations
*/
void NeoController::setPosition(units::turn_t position, int _slot) {
    auto slot = rev::spark::ClosedLoopSlot::kSlot0;
    if (_slot == 1) slot = rev::spark::ClosedLoopSlot::kSlot1;
    else if (_slot == 2) slot = rev::spark::ClosedLoopSlot::kSlot2;
    else if (_slot == 3) slot = rev::spark::ClosedLoopSlot::kSlot3;
    GetClosedLoopController().SetReference(
        position.value(),
        rev::spark::SparkMax::ControlType::kPosition,
        slot,
        pidf.S
    );
}

void NeoController::setSpeed(units::turns_per_second_t speed, int _slot) {
    auto slot = rev::spark::ClosedLoopSlot::kSlot0;
    if (_slot == 1) slot = rev::spark::ClosedLoopSlot::kSlot1;
    else if (_slot == 2) slot = rev::spark::ClosedLoopSlot::kSlot2;
    else if (_slot == 3) slot = rev::spark::ClosedLoopSlot::kSlot3;
    GetClosedLoopController().SetReference(
        speed.value(),
        rev::spark::SparkMax::ControlType::kVelocity,
        slot,
        pidf.S
    );
}

void NeoController::InitSendable(wpi::SendableBuilder& builder) {
    builder.SetSmartDashboardType("Subsystem");
    builder.AddDoubleProperty(
        "Stator Current", 
        [this] { return GetOutputCurrent(); },
        nullptr
    );
    builder.AddDoubleProperty(
        "Position", 
        [this] { return getPosition().to<double>(); },
        nullptr
    );
    builder.AddDoubleProperty(
        "Speed", 
        [this] { return getSpeed().to<double>(); },
        nullptr
    );
}

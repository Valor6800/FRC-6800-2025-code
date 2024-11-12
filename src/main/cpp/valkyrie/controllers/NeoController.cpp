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
                                    bool _inverted,
                                    std::string canbus) :
    BaseController(new rev::spark::SparkMax(canID, rev::spark::SparkMax::MotorType::kBrushless), _inverted, _mode, getNeoControllerMotorSpeed(controllerType)),
    cancoder(nullptr),
    pidController(motor->GetClosedLoopController())
{
    init();
}

void NeoController::init()
{
    valor::PIDF motionPIDF;
    motionPIDF.P = NEO_PIDF_KP;
    motionPIDF.I = NEO_PIDF_KI;
    motionPIDF.D = NEO_PIDF_KD;
    motionPIDF.error = 0_tr;
    motionPIDF.maxVelocity = NEO_PIDF_KV;
    motionPIDF.maxAcceleration = NEO_PIDF_KA;

    setNeutralMode(neutralMode);
    setCurrentLimits(STATOR_CURRENT_LIMIT, SUPPLY_CURRENT_LIMIT, SUPPLY_CURRENT_THRESHOLD, SUPPLY_TIME_THRESHOLD);
    setGearRatios(rotorToSensor, sensorToMech);
    setPIDF(pidf, 0);

    wpi::SendableRegistry::AddLW(this, "NeoController", "ID " + std::to_string(0));
}

void NeoController::setupCANCoder(int deviceId, units::turn_t offset, bool clockwise, std::string canbus, ctre::phoenix6::signals::AbsoluteSensorRangeValue absoluteRange, bool saveImmediately)
{
    cancoder = new ctre::phoenix6::hardware::CANcoder(deviceId, canbus);
}

void NeoController::applyConfig()
{
}

units::turn_t NeoController::getCANCoder()
{
    return cancoder ? cancoder->GetAbsolutePosition().GetValue() : 0_tr;
}

void NeoController::reset()
{
}

void NeoController::setEncoderPosition(units::turn_t position)
{
}

void NeoController::setupFollower(int canID, bool followerInverted)
{
}

void NeoController::setForwardLimit(units::turn_t forward, bool saveImmediately)
{
}

void NeoController::setReverseLimit(units::turn_t reverse, bool saveImmediately)
{
}

void NeoController::setCurrentLimits(units::ampere_t statorCurrentLimit, units::ampere_t supplyCurrentLimit, units::ampere_t supplyCurrentThreshold, units::second_t supplyTimeThreshold, bool saveImmediately)
{
}

void NeoController::setPIDF(valor::PIDF _pidf, int slot, bool saveImmediately)
{
    pidf = _pidf;
}

void NeoController::setGearRatios(double _rotorToSensor, double _sensorToMech, bool saveImmediately)
{
    rotorToSensor = _rotorToSensor;
    sensorToMech = _sensorToMech;
}

units::ampere_t NeoController::getCurrent()
{
    return units::ampere_t{0};
}

/**
 * Output is in mechanism rotations!
*/
units::turn_t NeoController::getPosition()
{
    return 0_tr;
}

/**
 * Output is in mechanism rotations!
*/
units::turns_per_second_t NeoController::getSpeed()
{
    return 0_tps;
}

/**
 * Set a position in mechanism rotations
*/
void NeoController::setPosition(units::turn_t position)
{
}

void NeoController::setSpeed(units::turns_per_second_t speed)
{
}

void NeoController::setPower(units::volt_t voltage)
{
}

void NeoController::setProfile(int profile)
{
    currentProfile = profile;
}

void NeoController::setNeutralMode(valor::NeutralMode mode, bool saveImmediately)
{
    neutralMode = mode;
}

void NeoController::setOpenLoopRamp(units::second_t time, bool saveImmediately)
{
}

void NeoController::InitSendable(wpi::SendableBuilder& builder)
{
    builder.SetSmartDashboardType("Subsystem");
    // builder.AddDoubleProperty(
    //     "Stator Current", 
    //     [this] { return getMotor()->GetStatorCurrent().GetValueAsDouble(); },
    //     nullptr);
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
    // builder.AddDoubleProperty(
    //     "Position", 
    //     [this] { return getPosition().to<double>(); },
    //     nullptr);
    // builder.AddDoubleProperty(
    //     "Speed", 
    //     [this] { return getSpeed().to<double>(); },
    //     nullptr);
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

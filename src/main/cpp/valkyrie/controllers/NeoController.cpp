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
                                    valor::NeutralMode _mode,
                                    bool _inverted,
                                    double _rotorToSensor,
                                    double _sensorToMech,
                                    std::string canbus) :
    BaseController(getNeoControllerMotorSpeed(controllerType), _rotorToSensor, _sensorToMech),
    rev::spark::SparkMax(canID, rev::spark::SparkMax::MotorType::kBrushless),
    pidController{GetClosedLoopController()}
{
    valor::PIDF motionPIDF;
    motionPIDF.P = NEO_PIDF_KP;
    motionPIDF.I = NEO_PIDF_KI;
    motionPIDF.D = NEO_PIDF_KD;
    motionPIDF.error = 0_tr;
    motionPIDF.maxVelocity = NEO_PIDF_KV;
    motionPIDF.maxAcceleration = NEO_PIDF_KA;

    wpi::SendableRegistry::AddLW(this, "NeoController", "ID " + std::to_string(0));
}

void NeoController::reset()
{
}

void NeoController::setEncoderPosition(units::turn_t) {}

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
void NeoController::setPosition(units::turn_t position, int slot)
{
}

void NeoController::setSpeed(units::turns_per_second_t speed, int slot)
{
}

void NeoController::setPower(units::volt_t) {}
void NeoController::setPower(units::scalar_t) {}

units::volt_t NeoController::getVoltage() { return 0_V; }
units::ampere_t NeoController::getCurrent() { return 0_A; }
units::scalar_t NeoController::getDutyCycle() { return 0; }

void NeoController::InitSendable(wpi::SendableBuilder& builder)
{
    builder.SetSmartDashboardType("Subsystem");
}

#include "valkyrie/drivetrain/TractionController.h"

#define MIN_SLIP_RATIO 0.01f
#define MAX_SLIP_RATIO 0.40f
#define SIGMOID_K 10
#define FORCE_ACCELERATION_MULTIPLIER 0.7f
#define INTERTIAL_VELOCITY_THRESHOLD 0.01_mps
#define VELOCITY_DIFFERENCE_THRESHOLD 1.0_mps
#define MIN_SLIPPING_TIME 0.5_s
#define VELOCITY_REQUEST_THRESHOLD 0.05_mps
#define WHEEL_SPEED_THRESHOLD 0.05_mps
#define STATIC_FORCE_ACCELERATE_TRIGGER_TIME 0.1_s
#define DYNAMIC_FORCE_ACCELERATE_TRIGGER_TIME 0.2_s
#define FORCE_ACCELERATE_TIME 2.0_s

using namespace valor;

TractionController::TractionController(units::dimensionless::dimensionless_t staticCOF, 
                                       units::dimensionless::dimensionless_t dynamicCOF, 
                                       units::dimensionless::dimensionless_t optimalSlipRatio, 
                                       units::mass::kilogram_t mass, 
                                       units::meters_per_second_t maxLinearSpeed) :
    
    _optimalSlipRatio(std::clamp(optimalSlipRatio.to<float>(), MIN_SLIP_RATIO, MAX_SLIP_RATIO)),
    _mass(mass.value()/4),
    _maxLinearVelocity(std::floor(maxLinearSpeed.value()*1000)/1000),
    _staticCOF(staticCOF),
    _dynamicCOF(dynamicCOF),
    _maxAcceleration(_staticCOF * 9.81),
    _slippingDebouncer(MIN_SLIPPING_TIME, frc::Debouncer::kRising),
    _maxPredictedSlipRatio((_maxAcceleration * 50 ) / (_staticCOF * _mass * 9.81)),
    _isSlipping(false),
    _staticForceAccelerationDebouncer(STATIC_FORCE_ACCELERATE_TRIGGER_TIME, frc::Debouncer::kRising),
    _dyamicForceAccelerationDebouncer(DYNAMIC_FORCE_ACCELERATE_TRIGGER_TIME, frc::Debouncer::kRising),
    _state(TractionController::State::DISABLE)
{
    _forceAccelerationTimer.Reset();
    _forceAccelerationTimer.Start();
}

units::meters_per_second_t TractionController::calculate(units::meters_per_second_t velocityRequest, units::meters_per_second_t inertialVelocity, units::meters_per_second_t wheelSpeed){
    
}

bool TractionController::isModuleSlipping(){
    return _isSlipping;
}

units::meters_per_second_t TractionController::getMaxLinearVelocity(){
    return units::meters_per_second_t{_maxLinearVelocity};
}

void TractionController::toggleTractionControl(){
    if(_state == TractionController::State::DISABLE) _state = TractionController::State::ENABLE;
    else _state = TractionController::State::DISABLE;
}

void TractionController::enableTractionControl(){
    _state = TractionController::State::ENABLE;
}

void TractionController::disableTractionControl(){
    _state = TractionController::State::DISABLE;
}

bool TractionController::isEnabled(){
    return _state == TractionController::State::ENABLE;
}
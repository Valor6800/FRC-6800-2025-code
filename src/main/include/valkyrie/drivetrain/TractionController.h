#include <frc/filter/Debouncer.h>
#include <frc/Timer.h>
#include <math.h>

#include "units/velocity.h"
#include "units/time.h"
#include "units/mass.h"
#include "units/constants.h"

namespace valor {

    class TractionController {
        public:
            TractionController(units::dimensionless::dimensionless_t staticCOF, 
                               units::dimensionless::dimensionless_t dynamicCOF, 
                               units::dimensionless::dimensionless_t optimalSlipRatio, 
                               units::mass::kilogram_t mass, 
                               units::meters_per_second_t maxLinearSpeed);
            units::meters_per_second_t calculate(units::meters_per_second_t velocityRequest, 
                                                 units::meters_per_second_t inertialVelocity, 
                                                 units::meters_per_second_t wheelSpeed);
            bool isModuleSlipping();
            units::meters_per_second_t getMaxLinearVelocity();
            void toggleTractionControl();
            void enableTractionControl();
            void disableTractionControl();
            bool isEnabled();
        private:
            enum State{
                DISABLE,
                ENABLE
            };
            double _optimalSlipRatio;
            double _mass;
            double _maxLinearVelocity;
            double _staticCOF;
            double _dynamicCOF;
            double _maxAcceleration;
            double _maxPredictedSlipRatio;
            bool _isSlipping;
            frc::Debouncer _slippingDebouncer;
            frc::Debouncer _staticForceAccelerationDebouncer;
            frc::Debouncer _dyamicForceAccelerationDebouncer;
            State _state;
            frc::Timer _forceAccelerationTimer;
    };
}
#include <frc/filter/Debouncer.h>
#include <frc/Timer.h>

#include "units/velocity.h"
#include "units/time.h"

namespace valor {

    class TractionController{
        public:
            TractionController();
        private:
            double _optimalSlipRatio;
            double _mass;
            double _maxLinearVelocity;
            
    };
}
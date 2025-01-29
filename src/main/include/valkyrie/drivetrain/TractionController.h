#include "units/velocity.h"
#include "units/time.h"

namespace valor {

    class TractionController{
        public:
            TractionController();
        private:
            double MIN_SLIP_RATIO = 0.01;
            double MAX_SLIP_RATIO = 0.40;
            int SIGMOID_K = 10;
            double FORCE_ACCELERATION_MULTIPLIER = 0.7;
            units::meters_per_second_t INTERTIAL_VELOCITY_THRESHOLD = 0.01_mps;
            units::meters_per_second_t VELOCITY_DIFFERENCE_THRESHOLD = 1.0_mps;
            units::second_t MIN_SLIPPING_TIME = 0.5_s;
            units::meters_per_second_t VELOCITY_REQUEST_THRESHOLD = 0.05_mps;
            units::meters_per_second_t WHEEL_SPEED_THRESHOLD = 0.05_mps;
            units::second_t STATIC_FORCE_ACCELERATE_TRIGGER_TIME = 0.1_s;
            units::second_t DYNAMIC_FORCE_ACCELERATE_TRIGGER_TIME = 0.2_s;
            units::second_t FORCE_ACCELERATE_TIME = 2.0_s;
    };
}
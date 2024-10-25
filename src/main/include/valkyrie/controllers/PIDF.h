#pragma once

namespace valor {

enum FeedForwardType
{
    LINEAR,
    CIRCULAR
};

/**
 * @brief Container to hold PID and feed forward values for the motor controller
 */
struct PIDF
{
    /// Proportion control of the feedback term
    units::scalar_t P = 0.0;
    /// Integral control of the feedback term
    units::scalar_t I = 0.0;
    /// Derivative control of the feedback term
    units::scalar_t D = 0.0;
    /// Max velocity: revolutions per 1s
    units::turns_per_second_t maxVelocity = 0_tps;
    /// Max acceleration: revolutions per 1s^2
    units::turns_per_second_squared_t maxAcceleration = 0_tr_per_s_sq;
    /// Max jerk: revolutions per 1s^3
    units::turns_per_second_cubed_t maxJerk = 0_tr_per_s_cu;
    /// Minimum error threshold
    units::turn_t error = 0_tr;

    units::scalar_t S = 0.19;

    units::scalar_t aFF = 0;
    units::turn_t aFFTarget = 90_deg;
    FeedForwardType aFFType = FeedForwardType::LINEAR;
};
}

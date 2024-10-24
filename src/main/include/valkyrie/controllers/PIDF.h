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
    double P = 0.0;
    /// Integral control of the feedback term
    double I = 0.0;
    /// Derivative control of the feedback term
    double D = 0.0;
    /// Max velocity: revolutions per 1s
    units::turns_per_second_t maxVelocity = 0_tps;
    /// Max acceleration: revolutions per 1s^2
    units::turns_per_second_squared_t maxAcceleration = 0_tr_per_s_sq;
    /// Max jerk: revolutions per 1s^3
    units::turns_per_second_cubed_t maxJerk = 0_tr_per_s_cu;
    /// Minimum error threshold
    double error = 0.0;

    double S = 0.19;

    double aFF = 0;
    double aFFTarget = 90;
    FeedForwardType aFFType = FeedForwardType::LINEAR;
};
}

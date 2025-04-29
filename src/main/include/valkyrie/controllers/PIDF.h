#pragma once

#include <units/base.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/angular_jerk.h>
#include <units/velocity.h>
#include <units/acceleration.h>

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
    void setMaxVelocity(units::turns_per_second_t _maxVelocity)
    {
        maxVelocity = _maxVelocity;
    }

    void setMaxVelocity(units::meters_per_second_t _maxVelocityMPS, units::meter_t wheelDiameter)
    {
        units::turns_per_second_t _maxVelocity = _maxVelocityMPS * 1_tr / (M_PI * wheelDiameter);
        maxVelocity = _maxVelocity;
    }

    void setMaxAcceleration(units::turns_per_second_squared_t _maxAcceleration)
    {
        maxAcceleration = _maxAcceleration;
    }

    void setMaxAcceleration(units::meters_per_second_squared_t _maxAccelerationMPS, units::meter_t wheelDiameter)
    {
        units::turns_per_second_squared_t _maxAcceleration = _maxAccelerationMPS * 1_tr / (M_PI * wheelDiameter);
        maxAcceleration = _maxAcceleration;
    }

    void setMaxJerk(units::turns_per_second_cubed_t _maxJerk)
    {
        maxJerk = _maxJerk;
    }



    /// Proportion control of the feedback term
    double P = 0.0;
    /// Integral control of the feedback term
    double I = 0.0;
    /// Derivative control of the feedback term
    double D = 0.0;
    
    double kV = -1;
    /// Max velocity: revolutions per 1s
    units::turns_per_second_t maxVelocity = 0_tps;
    /// Max acceleration: revolutions per 1s^2
    units::turns_per_second_squared_t maxAcceleration = 0_tr_per_s_sq;
    /// Max jerk: revolutions per 1s^3
    units::turns_per_second_cubed_t maxJerk = 0_tr_per_s_cu;
    /// Minimum error threshold
    units::turn_t error = 0_tr;

    double S = 0.19;

    double aFF = 0;
    units::turn_t aFFTarget = 90_deg;
    FeedForwardType aFFType = FeedForwardType::LINEAR;
};
}

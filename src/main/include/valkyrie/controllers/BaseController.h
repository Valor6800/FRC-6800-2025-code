#pragma once
#include <valkyrie/controllers/PIDF.h>
#include <units/angular_velocity.h>
#include <units/voltage.h>
#include <units/current.h>
#include <ctre/phoenix6/CANcoder.hpp>

namespace valor {

/**
 * @brief Neutral / Idle Modes for motorcontrollers. Either Brake or Coast.
 */
enum NeutralMode
{
    Brake,
    Coast
};

/**
 * @brief Abstract class that all Valor controllers's should implement
 * @tparam T Motor data type
 * 
 * To make developer's lives easier and to prevent any mistakes in a quick build season,
 * BaseController is used to organize code and abstract a lot of the base code that is often
 * repetitive in all motor controllers.
 * 
 * The idea is that motor controllers on the robot implement BaseController and logic for that
 * motor controller is run by the implemented class. Setup for the motors should also occur
 * in the implemented BaseController, and the motor pointer itself also lives in this class.
 * 
 * Helper methods exist to make it easier for subsystems to control motors in a variety of ways.
 * 
 * Usage:
 * \code {.cpp}
 * public class ValorFalconController : public BaseController<WPI_TalonFX> { };
 * \endcode
 */
class BaseController
{
public:

    /**
     * @brief Construct a new Valor Controller object
     * 
     * @param _motor The motor that will be controlled. Setup by the implemented class
     */
    BaseController(units::turns_per_second_t _maxMotorSpeed, double _rotorToSensor, double _sensorToMech) :
        maxMotorSpeed(_maxMotorSpeed),
        rotorToSensor(_rotorToSensor),
        sensorToMech(_sensorToMech) {}

    /**
     * @brief Get the motor's maximum speed
     * 
     * @return units::turns_per_second_t Motor's maximum angular speed (at the rotor)
     */
    units::turns_per_second_t getMaxMotorSpeed() { return maxMotorSpeed; }

    /**
     * @brief Get the mechanisms's maximum speed
     * 
     * @return units::turns_per_second_t Mechanism's maximum angular speed (at the mech)
     */
    units::turns_per_second_t getMaxMechSpeed() { return maxMotorSpeed / (rotorToSensor * sensorToMech); }

    /**
     * @brief Resets the motor and any state
     * 
     * Clear the encoders for the motor and set to 0.
     * 
     * Additionally, should be called by the constructor to set default values
     * before any logic is run.
     * 
     * To be defined by the implemented BaseController class
     */
    virtual void reset() = 0;

    /**
     * @brief Get the motors position
     * 
     * To be defined by the implemented BaseController class
     * 
     * @return units::turn_t Position of the motor
     */
    virtual units::turn_t getPosition() = 0;

    /**
     * @brief Get the motors speed
     * 
     * To be defined by the implemented BaseController class
     * 
     * @return units::turns_per_second_t Speed of the motor
     */
    virtual units::turns_per_second_t getSpeed() = 0;
    
    virtual void setEncoderPosition(units::turn_t position) = 0;

    /**
     * @brief Send the motor to a specific position
     * 
     * Will use the motor's native trapezoidal motion profile to get the motor to that position.
     * Can be tuned using the velocity and acceleration components of valor::PIDF via @link setPIDF @endlink
     * 
     * To be defined by the implemented BaseController class
     * 
     * @param position The position to send the motor to
     */
    virtual void setPosition(units::turn_t position, int slot = 0) = 0;

    /**
     * @brief Send the motor to a specific speed
     * 
     * Will use the motor's native trapezoidal motion profile to get the motor to that position.
     * Can be tuned using the PIDF components of valor::PIDF via @link setPIDF @endlink
     * 
     * To be defined by the implemented BaseController class
     * 
     * @param speed The speed to set the motor to
     */
    virtual void setSpeed(units::turns_per_second_t speed, int slot = 0) = 0;

    virtual void setPower(units::volt_t) = 0;
    virtual void setPower(units::ampere_t) = 0;
    virtual void setPower(units::scalar_t) = 0;

    virtual units::volt_t getVoltage() = 0;

    /**
     * @brief Get the motors output current
     * 
     * Get the instantaneous current of the motor that the controller owns
     * 
     * @return units::ampere_t Instantaneous amperage of the motor
     */
    virtual units::ampere_t getCurrent() = 0;

    virtual units::scalar_t getDutyCycle() = 0;

protected:
    units::turns_per_second_t maxMotorSpeed;
    units::volt_t voltageCompensation;
    double rotorToSensor;
    double sensorToMech;
};
}

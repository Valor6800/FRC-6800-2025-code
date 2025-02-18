#pragma once

#include "valkyrie/controllers/PIDF.h"
#include "valkyrie/controllers/NeutralMode.h"

#include <string>

#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableBuilder.h>
#include <wpi/sendable/SendableHelper.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <string>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/CANBus.hpp>

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/voltage.h>
#include <units/current.h>

namespace valor {

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
template <class T>
class BaseController : public wpi::Sendable, public wpi::SendableHelper<BaseController<T>>
{
public:

    /**
     * @brief Construct a new Valor Controller object
     * 
     * @param _motor The motor that will be controlled. Setup by the implemented class
     */
    BaseController(T* _motor, bool _inverted, valor::NeutralMode _neutralMode, units::turns_per_second_t _maxMotorSpeed) :
        maxMotorSpeed(_maxMotorSpeed),
        voltageCompenstation(units::volt_t{12.0}),
        motor(_motor),
        inverted(_inverted),
        neutralMode(_neutralMode),
        rotorToSensor(1),
        sensorToMech(1) {}

    /**
     * @brief Destroy the Valor Controller object
     * 
     * Due to the BaseController implementation owning the motor, a destructor
     * is needed to clean up the motor and any followers. The motor and follower
     * are stored on the heap so use the delete keyword to remove them from the heap.
     */
    ~BaseController()
    {
        if (motor) {
            delete motor;
            motor = nullptr;
        };
        if (followerMotor) {
            delete followerMotor;
            followerMotor = nullptr;
        };
    }

    virtual void applyConfig() = 0;

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
     * @brief Setup the voltage compensation for the motor
     * @note Make sure to call first before setting up any PIDF profiles
     */
    void setVoltageCompensation(units::volt_t _voltageCompensation) { voltageCompenstation = _voltageCompensation; }

    units::volt_t getVoltageCompensation() { return voltageCompenstation; }

    /**
     * @brief Get a pointer to the Motor object that the BaseController implementation owns
     * 
     * @return T* Pointer to the motor object. Allows developers to call functions attached to the motor
     */
    T* getMotor() { return motor; }

    /**
     * @brief Initialize the motor. Setup any parameters that get burned to the motor's flash memory
     * 
     * Usually used to setup motor parameters, and called via the constructor.
     * Needs to be defined in the implemented class
     * 
     * To be defined by the implemented BaseController class
     */
    virtual void init() = 0;

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
     * @brief Get the motors output current
     * 
     * Get the instantaneous current of the motor that the controller owns
     * 
     * @return units::ampere_t Instantaneous amperage of the motor
     */
    virtual units::ampere_t getCurrent() = 0;

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

    /**
     * @brief If a motor is paired with another motor, setup that other motor as a follower
     * 
     * The follower motor will need a CAN ID, and then it will mimic and assume
     * all other parameters of the lead motor
     * 
     * To be defined by the implemented BaseController class
     * 
     * @param canID The CAN ID of the follower motor
     */
    virtual void setupFollower(int canID, bool followerInverted = false) = 0;
    
    /**
     * @brief Change the PIDF values for the motor
     * 
     * valor::PIDF has some default values already set and tested, but if the system
     * requires some changes, use this method to change those defaults.
     * 
     * @param pidf The new PIDF values to use for the system
     * @param slot Set which slot of the motor to apply the PIDF. 0 if slots aren't compatible
     */
    virtual void setPIDF(valor::PIDF pidf, int slot = 0, bool saveImmediately = false) = 0;

    /**
     * @brief Set both soft limits for the motor
     * 
     * Soft limits restrict the reverse and forward direction to a certain range.
     * 
     * Calls out to the pure virtual functions @link setForwardLimit @endlink
     * and @link setReverseLimit @endlink
     * 
     * @param reverse The reverse soft limit
     * @param forward The forward soft limit
     */
    void setLimits(units::turn_t reverse, units::turn_t forward)
    {
        setForwardLimit(forward);
        setReverseLimit(reverse);
    }

    /**
     * @brief Set the forward soft limit for the motor
     * 
     * Soft limits restrict the reverse and forward direction to a certain range.
     * 
     * @param forward The forward soft limit
     */
    virtual void setForwardLimit(units::turn_t forward, bool saveImmediately = false) = 0;

    /**
     * @brief Set the reverse soft limit for the motor
     * 
     * Soft limits restrict the reverse and reverse direction to a certain range.
     * 
     * @param reverse The reverse soft limit
     */
    virtual void setReverseLimit(units::turn_t reverse, bool saveImmediately = false) = 0;

    /**
     * @brief Set the gear ratios for the motor
     * 
     * Used to convert between the motor's rotor and the output shaft of the mechanism
     * 
     * To be defined by the implemented BaseController class
     * 
     * @param rotorToSensor The gear ratio from rotor to where the sensor is. Should be 1 if no external sensor
     * @param sensorToMech The gear ratio from the sensor to the mechanism's output shaft. Should be the gear ratio if no external sensor
     * @param saveImmediately Tell the underlying controller to apply the changes immediately, or to wait until a manual apply has been called
     */
    virtual void setGearRatios(double rotorToSensor, double sensorToMech, bool saveImmediately = false) = 0;

    /**
     * @brief Set the current limits for the motor
     * 
     * To be defined by the implemented BaseController class
     * 
     * @param statorCurrentLimit The stator's current limit in amps
     * @param supplyCurrentLimit The supply current limit in amps
     * @param supplyCurrentThreshold The supply current threshold in amps
     * @param supplyTimeThreshold The amount of time before re-allowing current to pass through
     */
    virtual void setCurrentLimits(units::ampere_t statorCurrentLimit, units::ampere_t supplyCurrentLimit, units::ampere_t supplyCurrentThreshold, units::second_t supplyTimeThreshold, bool saveImmediately = false) = 0;

    virtual void setNeutralMode(valor::NeutralMode mode, bool saveImmediately = false) = 0;

    virtual void setOpenLoopRamp(units::second_t time, bool saveImmediately = false) = 0;

    valor::NeutralMode getNeutralMode(){
        return neutralMode;
    }

    virtual void InitSendable(wpi::SendableBuilder& builder) = 0;

    virtual units::turn_t getAbsEncoderPosition() = 0;

    virtual void setupCANCoder(int deviceId, units::turn_t offset, bool clockwise = false, std::string canbus = "", units::turn_t absoluteRange=1_tr, bool saveImmediately = false) = 0;
    virtual ctre::phoenix6::hardware::CANcoder *getCANCoder() = 0;

protected:

    units::turns_per_second_t maxMotorSpeed;
    units::volt_t voltageCompenstation;
    
    T* motor;
    
    bool inverted;
    valor::NeutralMode neutralMode;
    double rotorToSensor;
    double sensorToMech;
    T* followerMotor;
};
}

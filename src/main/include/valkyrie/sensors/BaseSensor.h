#pragma once

#include <functional>
#include <frc/TimedRobot.h>

#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableBuilder.h>
#include <wpi/sendable/SendableHelper.h>

namespace valor {

/**
 * @brief Abstract class that all Valor sensors's should implement
 * @tparam T Sensor data type
 * 
 * To make developer's lives easier and to prevent any mistakes in a quick build season,
 * BaseSensor is used to organize code and abstract a lot of the base code that is often
 * repetitive in all sensors.
 * 
 * The idea is that sensors on the robot implement BaseSensor and logic for that
 * sensor is controlled by the implemented class.
 * 
 * The meat and potatoes of BaseSensor are within the protected @link calculate @endlink function.
 * This function runs every 10ms (twice as much as the normal periodic loop on the robot) and is
 * automatically scheduled by the BaseSensor. Developers do not need to call the calculate function
 * as it will be done automatically.
 * 
 * What does the @link calculate @endlink function actually calculate and how does it obtain sensor
 * information? BaseSensor implementations do not actually own the sensor - the sensor ownership
 * should reside with the subsystem. Then, developers can pass a lambda function to BaseSensor which
 * will be called from within calculate. This means every 10 milliseconds when @link calculate @endlink
 * runs, the function will reach out to the owning subsystem via the passed in lambda function to
 * obtain the sensor data, then parse and save the resulting data.
 * 
 * The primary purpose of this architecture is that there is no generic method for sensor data collection.
 * Each sensor is different, and therefore lambda functions are necessary so the developer can specify
 * the method of obtaining data from the sensor in their subsystem.
 * 
 * For those of you who are unaware of what a lambda function is, it is an async function - or a function
 * that will run in the future due to a trigger (in our case, it will be a time based trigger).
 * 
 * Usage:
 * \code {.cpp}
 * public class ValorLighBeamSensor : public BaseSensor<boolean> { };
 * \endcode
 */
template <class T>
class BaseSensor : public wpi::Sendable, public wpi::SendableHelper<BaseSensor<T>>
{
public:

    /**
     * @brief Construct a new Valor Sensor object
     * 
     * @param _robot Pass in the Robot reference so the calculate method can be auto-scheduled
     */
    BaseSensor(frc::TimedRobot *_robot, const char *_name) : robot(_robot), sensorName(_name) {}
    
    /**
     * @brief Reset the sensor state
     * 
     * Clear any member variables, and set the default value and previous value
     * Needs to be defined in all BaseSensor implementations.
     * 
     * Additionally, should be called by the constructor to set default values
     * before any logic is run.
     */
    virtual void reset() = 0;

    /**
     * @brief Specify the function that should be called every 10ms to obtain sensor data
     * 
     * The goal of this function is to allow subsystem developers to create a function in 
     * the subsystem that will obtain data from the sensor. Then, that function can be passed
     * to the implemented BaseSensor to be run every 10ms and do further analysis and calculations
     * on that returned sensor value.
     * 
     * This is all done in the background so students do not have to worry about making sure the
     * sensor is always updated.
     * 
     * The reason this was implemented this way was because we could still have a line break sensor
     * for different vendors. Say Rev and CTRE both have line break sensors that they sell, and have
     * different libraries for obtaining the data from that sensor. Both are still classified as a line
     * break sensor and our @link calculate @endlink logic should be the same for both, but the method
     * of obtaining the data is different. Therefore we would write different lambda functions and pass
     * in the specified one into multiple BaseSensor implementations to yield the same result.
     * 
     * Usage:
     * \code {.cpp}
     * ValorCurrentSensor currentSensor(robot);
     * WPI_TalonFX motor_intake;
     * currentSensor.setGetter([this]() { return motor_intake.GetOutputCurrent(); });
     * \endcode
     * 
     * @param _lambda Lambda function that will be called every 10ms to obtain sensor data from the subsystem
     */
    void setGetter(std::function<T()> _lambda)
    {
        sensorLambda = _lambda;
        robot->AddPeriodic([this] {
            calculate();
        }, 10_ms, 5_ms);
    }

    /**
     * @brief Get the Sensor object
     * 
     * If a lambda function has been set via @link setGetter @endlink, then
     * this function will call that lambda function and return the results of the function.
     * If a lambda function has not been set, it will return 0.
     * 
     * Intended to be utilized in @link calculate @endlink to get sensor information.
     * Additionally, can be used in the subsystem class.
     * 
     * @return T Returns the sensor object
     */
    T getSensor() { return sensorLambda ? sensorLambda() : T(); }

    virtual void InitSendable(wpi::SendableBuilder& builder) = 0;

protected:

    /**
     * @brief Get the sensor value and post-process any metrics associated with the sensor
     * 
     * This function runs every 10ms (twice as much as the normal periodic loop on the robot) and is
     * automatically scheduled once a getter has been set via @link setGetter @endlink.
     * Developers do not need to call the calculate function as it will be done automatically.
     * 
     * What does the calculate function actually calculate and how does it obtain sensor
     * information? BaseSensor implementations do not actually own the sensor - the sensor ownership
     * should reside with the subsystem. Then, developers can pass a lambda function to BaseSensor which
     * will be called from within calculate. This means every 10 milliseconds when calculate
     * runs, the function will reach out to the owning subsystem via the passed in lambda function to
     * obtain the sensor data, then parse and save the resulting data.
     * 
     * Needs to be defined in any BaseSensor implementations.
     */
    virtual void calculate() = 0;

    /**
     * @brief Member lambda function to be run by @link getSensor @endlink
     * 
     * @return T Returns the sensor object
     */
    std::function<T()> sensorLambda;

    /**
     * @brief Holds the current sensor state
     */
    T currState;

    /**
     * @brief Holds the previous sensor state
     */
    T prevState;
    
    frc::TimedRobot *robot;
    
    const char* sensorName;

};
}

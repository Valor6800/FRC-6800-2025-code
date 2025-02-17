#pragma once

#include "valkyrie/sensors/BaseSensor.h"
#include <frc/TimedRobot.h>
#include <functional>

namespace valor {

/**
 * @brief Sensor - debouncing and identifying rising/falling edges of boolean inputs
 * 
 * Class centered around debouncing boolean inputs.
 * 
 * <b>What is a rising/falling edge?</b> When a boolean input changes, that is called
 * an "edge". When a boolean input goes from false to true, that is called a rising
 * edge. When a boolean input goes from true to false, that is called a falling edge.
 * 
 * <b>What is debouncing?</b> Debouncing is the process of filtering out edges and making
 * sure that if an edge triggers multiple times in a very quick amount of time, it only
 * counts as a single rising or falling edge.
 */
class DebounceSensor : public BaseSensor<bool>
{
public:

    /**
     * @brief Construct a new Valor Debounce Sensor object
     * 
     * @param _robot Pass in the Robot reference so the calculate method can be auto-scheduled
     */
    DebounceSensor(frc::TimedRobot *_robot, const char* name);
    
    virtual void reset();

    /**
     * @brief Setup a lambda function to detect a rising or falling edge of the sensor
     * 
     * @param _lambda Function to run when an edge has been detected
     */
    void setEdgeCallback(std::function<void()> _lambda);

    /**
     * @brief Setup a lambda function to detect a rising edge of the sensor
     * 
     * @param _lambda Function to run when a rising edge has been detected
     */
    void setRisingEdgeCallback(std::function<void()> _lambda);

    /**
     * @brief Setup a lambda function to detect a falling edge of the sensor
     * 
     * @param _lambda Function to run when a falling edge has been detected
     */
    void setFallingEdgeCallback(std::function<void()> _lambda);

    void InitSendable(wpi::SendableBuilder& builder) override;

private:

    void calculate();

    std::function<void()> edge;
    std::function<void()> fallingEdge;
    std::function<void()> risingEdge;
};
}

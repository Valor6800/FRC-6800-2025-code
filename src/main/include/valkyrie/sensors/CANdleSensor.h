#pragma once

#include "valkyrie/sensors/BaseSensor.h"
#include <ctre/phoenix6/CANcoder.hpp>
#include <frc/TimedRobot.h>

#include "ctre/phoenix/led/CANdle.h"

#include "ctre/phoenix/led/ColorFlowAnimation.h"
#include "ctre/phoenix/led/FireAnimation.h"
#include "ctre/phoenix/led/LarsonAnimation.h"
#include "ctre/phoenix/led/RainbowAnimation.h"
#include "ctre/phoenix/led/RgbFadeAnimation.h"
#include "ctre/phoenix/led/SingleFadeAnimation.h"
#include "ctre/phoenix/led/StrobeAnimation.h"
#include "ctre/phoenix/led/TwinkleAnimation.h"
#include "ctre/phoenix/led/TwinkleOffAnimation.h"

#include <iostream>
#include <unordered_map>

namespace valor {

/**
 * @brief Sensor - control the CANdle and associated LEDs
 * 
 * This sensor owns a CANdle device, and any connected LEDs to the CANdle.
 * Documentation on wiring the CANdle can be found here:
 * @link https://store.ctr-electronics.com/content/user-manual/CANdle%20User's%20Guide.pdf @endlink
 * 
 * Animations can be set, or static colors can be set. Note that animations only apply
 * to the extra LED strips. 
 * 
 * Example from: @link https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/C%2B%2B%20General/CANdle/src/main/cpp/subsystems/CANdleSystem.cpp @endlink
 */
class CANdleSensor : public valor::BaseSensor<int>{
public:

    static const int VALOR_GOLD = 0xEEA800;
    static const int VALOR_PURPLE = 0xFF00FF;
    static const int RED = 0xFF0000;
    static const int ORANGE = 0xFFA500;
    static const int GREEN = 0x00FF00;
    static const int LIGHT_BLUE = 0x00FFF9;
    static const int WHITE = 0xFFFFFF;
    static const int OFF = 0x000000;

    /**
     * @brief Declares the type of animation to apply.
     */
    enum AnimationType {
        None,
        ColorFlow,
        Fire,
        Larson,
        Rainbow,
        RgbFade,
        SingleFade,
        Strobe,
        Twinkle,
        TwinkleOff
    };

    enum Priority {
        PRIORITY_DEFAULT = 1,
        PRIORITY_SCORER = 2,
        PRIORITY_CLIMBER = 3
    };


    /**
     * @brief Represents an RGB hex code in 3 separate integers
     * Example: Hex 0xFF00AA
     * Red: 255
     * Green: 0
     * Blue: 170
     */
    struct RGBColor {
        int red;
        int green;
        int blue;
    };

    
    /**
     * @brief Represents a Segment of LEDs with their own settings
    */
    struct SegmentSettings{
        RGBColor currentColor;
        ctre::phoenix::led::Animation *activeAnimation;
        AnimationType activeAnimationType;
        int startLed;
        int ledCount;
        bool recentlyChanged;
    };


    /**
     * @brief Convert an RGB hex code to the RGBColor struct
     * 
     * @param color RGB hex code
     * @return RGBColor struct containing red, green, blue values matching the hex code
     */
    static RGBColor toRGB(int color);

    static int cancoderMagnetHealthGetter(ctre::phoenix6::hardware::CANcoder* cancoder);

    /**
     * @brief Construct a new Valor C A Ndle Sensor object
     * 
     * @param _robot Pointer to main robot
     * @param _ledCount How many external LEDs are connected to the CANdle
     * @param _canID the CAN ID the CANdle is assigned to
     * @param _canbus the CAN bus the CANdle is attached to
     */
    CANdleSensor(frc::TimedRobot *_robot, int _ledCount, int _segments, int _canID, std::string _canbus = "baseCAN");

    /**
     * @brief Destroy the Valor CANdle Sensor object
     * 
     */
    ~CANdleSensor();

    void setLED(int led, RGBColor rgb);
    void setLED(int led, int color);
    /**
     * @brief Set the color of the CANdle LEDs and attached LEDs
     * 
     * @param segment The segment that will be changed
     * @param color The color to change all the LEDs in the segment to.
     */
    void setColor(int segment, int color, Priority priority);

        /**
     * @brief Set the color of the CANdle LEDs and attached LEDs
     * 
     * @param segment The segment that will be changed
     * @param rgb The RGB code to change all the LEDs in the segment to.
     */
    void setColor(int segment, RGBColor rgb);
    /**
     * @brief Sets the color of the entire strip of LEDs
     * 
     * @param rgb The RGB code to change all the LEDs in the strip to.
    */
    void setColor(RGBColor rgb);
    /**
     * @brief Set the animation the LEDs should follow
     * 
     * @param segment The segment that will get animated
     * @param animation Animation to set. Will clear the previous color
     * @param color Color of the animation
     * @param speed The speed that the animation will go at
     */
    void setAnimation(int segment, AnimationType animation, RGBColor color, double speed=1.0);

    /**
     * @brief Sets the animation for all segments
     * 
     * @param animation Animation to set
     * @param color Color of the animation
     * @param speed The speed that the animation will go at
    */
    void setAnimation(AnimationType animation, RGBColor, double speed=1.0);

    /**
     * @brief Clears any active animation
     * 
     * Also responsible for clearing the appropriate memory associated with the animation
     * @param segment The segment that will be cleared
     */
    void clearAnimation(int segment);
    /**
     * @brief Clears all active animations
    */

    void resetPriority();
    void clearAnimation();

    
    /**
     * @brief Resets the CANdle and its' configuration
     */
    void reset();

    void InitSendable(wpi::SendableBuilder& builder) override;

    /**
     * @brief Gets the animation type of the segment
     * 
     * @param segment the segment to get the animation type from
     * @return The active animation type
    */
    CANdleSensor::AnimationType getActiveAnimationType(int segment);

    /**
     * @brief Gets the color of the segment
     * 
     * @param segment The segment to get the color from
     * @return The color of the segment
    */
    CANdleSensor::RGBColor getColor(int segment);

private:
    void setAnimation(SegmentSettings *segment, AnimationType animation, RGBColor color, double speed=1.0);
    std::mutex mutex;
    Priority currentPriority = PRIORITY_DEFAULT;
    ctre::phoenix::led::CANdle candle;
    int ledCount;
    int segments;

    std::unordered_map<int, SegmentSettings> segmentMap;
    SegmentSettings allSegments;

    void calculate();

};
}

#include "valkyrie/sensors/CANdleSensor.h"
#include <cmath>
#include <string>

using namespace valor;

CANdleSensor::CANdleSensor(frc::TimedRobot *_robot, int _ledCount, int _segments, int _canID, std::string _canbus) :
    BaseSensor(_robot, std::string("ID ").append(std::to_string(_canID)).c_str()),
    candle(_canID, _canbus),
    ledCount(_ledCount),
    segments(_segments)
{
    wpi::SendableRegistry::AddLW(this, "CANdleSensor", sensorName);

    reset();

    ctre::phoenix::led::CANdleConfiguration config;
    // Should match the type of LED strip connected to the CANdle
    config.stripType = ctre::phoenix::led::LEDStripType::GRB;
    config.brightnessScalar = 0.75;
    config.statusLedOffWhenActive = true;
    config.disableWhenLOS = false;
    // If the 12V line should be on, off, or modulated (for single LED colors)
    config.vBatOutputMode = ctre::phoenix::led::VBatOutputMode::On;
    candle.ConfigFactoryDefault(100);
    candle.ConfigAllSettings(config, 100);
    int segmentLEDCount = std::floor((ledCount - 8) / segments);
    int remainderLEDCount = (ledCount - 8) % segments;
    int currentStartLED = 0;

    //54
    for (int i = 0; i < segments + 1; i++){
        SegmentSettings newSegment;
        newSegment.recentlyChanged = true;
        newSegment.currentColor = toRGB(OFF);
        newSegment.activeAnimation = NULL;
        newSegment.activeAnimationType = AnimationType::None;
        if (i == 0){
            newSegment.startLed = 0;
            newSegment.ledCount = 8;
        } else {
            newSegment.startLed = currentStartLED;
            
            // Distribute extra LEDs in an alternating way
            if ((i % 2 == 0 && i != 0 && remainderLEDCount > 0) || (i == segments && remainderLEDCount > 0)) {
                newSegment.ledCount = segmentLEDCount + 1;
                remainderLEDCount--;
            } else {
                newSegment.ledCount = segmentLEDCount;
            }
        }
        // Update start LED for next segment
        currentStartLED = newSegment.startLed + newSegment.ledCount;
        segmentMap[i] = newSegment;
    }
    allSegments.startLed = 0;
    allSegments.ledCount = ledCount;
    allSegments.activeAnimation = NULL;
    allSegments.currentColor = toRGB(OFF);
    allSegments.recentlyChanged = false;
    allSegments.activeAnimationType = AnimationType::None;
    setGetter([this] { return 0; });
}

int CANdleSensor::cancoderMagnetHealthGetter(ctre::phoenix6::hardware::CANcoder* cancoder) {
    using namespace ctre::phoenix6::signals;
    if (cancoder && cancoder->IsConnected()) {
        auto value = cancoder->GetMagnetHealth().GetValue();
        if (value == MagnetHealthValue::Magnet_Green) return valor::CANdleSensor::GREEN;
        else if (value == MagnetHealthValue::Magnet_Orange) return valor::CANdleSensor::ORANGE;
    }
    return valor::CANdleSensor::RED;
}

CANdleSensor::RGBColor CANdleSensor::toRGB(int color)
{
    RGBColor outColor;
    outColor.red = ((color & 0xFF0000) >> 16);
    outColor.green = ((color & 0x00FF00) >> 8);
    outColor.blue = ((color & 0x0000FF));
    return outColor;
}

CANdleSensor::~CANdleSensor()
{
    for (int i = 0; i <= segments; i++){
        clearAnimation(i);
    }
    delete allSegments.activeAnimation;
}

void CANdleSensor::setLED(int led, RGBColor rgb) {
    if (led <= 8) {
        rgb.red *= 0.4;
        rgb.green *= 0.4;
        rgb.blue *= 0.4;
    }
    candle.SetLEDs(
        rgb.red,
        rgb.green,
        rgb.blue,
        0,
        led,
        1
    );
}

void CANdleSensor::setLED(int led, int color) {
    setLED(led, toRGB(color));
}

void CANdleSensor::setColor(int segment, RGBColor rgb)
{
    segment++;
    if (segment >= static_cast<int>(segmentMap.size())) return;
    segmentMap[segment].recentlyChanged = true;
    segmentMap[segment].currentColor = rgb;
}

void CANdleSensor::setColor(RGBColor rgb)
{
    allSegments.recentlyChanged = true;
    allSegments.currentColor = rgb;
}

void CANdleSensor::setColor(int segment, int color, Priority priority)
{
    std::lock_guard<std::mutex> lock(mutex);
    if (priority >= currentPriority) {
        currentPriority = priority;
        setColor(segment, toRGB(color));
    }
}

void CANdleSensor::setColorAll(int color, Priority priority)
{   
    std::lock_guard<std::mutex> lock(mutex);
    if (priority >= currentPriority) {
        currentPriority = priority;
        for (int i = 0; i <= segments; i++){
            setColor(i, toRGB(color));
        }
    } 
}

void CANdleSensor::setAnimation(AnimationType animation, RGBColor color, double speed) {
    clearAnimation(0);
    setAnimation(&allSegments, animation, color, speed);
}

void CANdleSensor::setAnimation(int segment, AnimationType animation,RGBColor color, double speed)
{
    segment++;
    if (segment >=  static_cast<int>(segmentMap.size())) return;
    clearAnimation(segment);
    setAnimation(&segmentMap[segment], animation, color, speed);
}

void CANdleSensor::resetPriority() {
    std::lock_guard<std::mutex> lock(mutex);
    currentPriority = PRIORITY_DEFAULT;
}


void CANdleSensor::setAnimation(CANdleSensor::SegmentSettings *segment, AnimationType animation,RGBColor color, double speed)
{
    segment->recentlyChanged = true;
    int brightness = 1;

    segment->activeAnimationType = animation;

    if (animation == AnimationType::ColorFlow){
        segment->activeAnimation = new ctre::phoenix::led::ColorFlowAnimation(
            segment->currentColor.red,
            segment->currentColor.green,
            segment->currentColor.blue,
            0,
            speed,
            segment->ledCount,
            ctre::phoenix::led::ColorFlowAnimation::Forward,
            segment->startLed
        );
    } else if (animation == AnimationType::Fire){
        segment->activeAnimation = new ctre::phoenix::led::FireAnimation(
            brightness,
            speed,
            segment->ledCount,
            1,
            1,
            false,
            segment->startLed
        );
    } else if (animation == AnimationType::Larson){
        segment->activeAnimation = new ctre::phoenix::led::LarsonAnimation(
            segment->currentColor.red,
            segment->currentColor.green,
            segment->currentColor.blue,
            0,
            speed,
            segment->ledCount,
            ctre::phoenix::led::LarsonAnimation::Front,
            2,
            segment->startLed
        );
    } else if (animation == AnimationType::Rainbow){
        segment->activeAnimation = new ctre::phoenix::led::RainbowAnimation(
            brightness,
            speed,
            segment->ledCount,
            false,
            segment->startLed
        );

    } else if (animation == AnimationType::RgbFade){
        segment->activeAnimation = new ctre::phoenix::led::RgbFadeAnimation(
            brightness,
            speed,
            segment->ledCount,
            segment->startLed
        );
    } else if (animation == AnimationType::SingleFade){
        segment->activeAnimation = new ctre::phoenix::led::SingleFadeAnimation(
            segment->currentColor.red,
            segment->currentColor.green,
            segment->currentColor.blue,
            0,
            speed,
            segment->ledCount,
            segment->startLed
        );
    } else if (animation == AnimationType::Strobe){
        segment->activeAnimation = new ctre::phoenix::led::StrobeAnimation(
            segment->currentColor.red,
            segment->currentColor.green,
            segment->currentColor.blue,
            0,
            speed,
            segment->ledCount,
            segment->startLed
        );
    } else if (animation == AnimationType::Twinkle){
        segment->activeAnimation = new ctre::phoenix::led::TwinkleAnimation(
            segment->currentColor.red,
            segment->currentColor.green,
            segment->currentColor.blue,
            0,
            speed,
            segment->ledCount,
            ctre::phoenix::led::TwinkleAnimation::TwinklePercent::Percent100,
            segment->startLed
        );
    } else if (animation == AnimationType::TwinkleOff){
        segment->activeAnimation = new ctre::phoenix::led::TwinkleOffAnimation(
            segment->currentColor.red,
            segment->currentColor.green,
            segment->currentColor.blue,
            0,
            speed,
            segment->ledCount,
            ctre::phoenix::led::TwinkleOffAnimation::TwinkleOffPercent::Percent100,
            segment->startLed
        );
    }
}

void CANdleSensor::clearAnimation(int segment)
{
    segment++;
    if (segment >=  static_cast<int>(segmentMap.size())) return;
    if (segmentMap[segment].activeAnimation != nullptr) {
        candle.ClearAnimation(0);
        delete segmentMap[segment].activeAnimation;
    }
    segmentMap[segment].activeAnimationType = AnimationType::None;
}

void CANdleSensor::clearAnimation()
{
    clearAnimation(-1);
}

CANdleSensor::AnimationType CANdleSensor::getActiveAnimationType(int segment) {
    segment++;
    if (segment >= static_cast<int>(segmentMap.size())) segment = 0;
    return segmentMap[segment].activeAnimationType;
}

CANdleSensor::RGBColor CANdleSensor::getColor(int segment) {
    segment++;
    if (segment >= static_cast<int>(segmentMap.size())) segment = 0;
    return segmentMap[segment].currentColor;
}

void CANdleSensor::reset()
{
    clearAnimation();
    prevState = OFF;
    currState = OFF;
}

void CANdleSensor::calculate()
{
    for (auto segment : segmentMap) {
        if (segmentMap[segment.first].activeAnimationType != AnimationType::None) {
            if (segmentMap[segment.first].activeAnimation) {
                candle.Animate(*(segmentMap[segment.first].activeAnimation));
            }
            segmentMap[segment.first].activeAnimationType = AnimationType::None;
        }
        if (segmentMap[segment.first].recentlyChanged) {
            candle.SetLEDs(
                segmentMap[segment.first].currentColor.green,
                segmentMap[segment.first].currentColor.red,
                segmentMap[segment.first].currentColor.blue,
                0,
                segmentMap[segment.first].startLed,
                segmentMap[segment.first].ledCount
            );
            segmentMap[segment.first].recentlyChanged = false;
        }
    }
}

void CANdleSensor::InitSendable(wpi::SendableBuilder& builder)
{
    builder.SetSmartDashboardType("Subsystem");
    builder.AddDoubleProperty(
        "Current State", 
        [this] { return prevState; },
        nullptr);
    builder.AddDoubleProperty(
        "Current State", 
        [this] { return currState; },
        nullptr);
    builder.AddDoubleProperty(
        "Max Animations",
        [this] {return candle.GetMaxSimultaneousAnimationCount();},
        nullptr);
}

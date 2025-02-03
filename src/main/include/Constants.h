/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */ /* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "frc/RobotController.h"
#include <cmath>
#include <iostream> 
#include <fstream>
#include <frc/RobotController.h>
#include <frc/geometry/Rotation3d.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <string>
#include <vector>
#include <cscore.h>
#include <frc/geometry/Pose3d.h>
#include <networktables/NetworkTable.h>
#include "valkyrie/controllers/PhoenixController.h"


#define ALPHA_SERIAL_NUMBER "03260AF3"
#define GOLD_SERIAL_NUMBER "033E1BEA"
// When trying to compile against other targets for simulation, cmath doesn't include M_PI
//   Therefore if not defined, define M_PI for use on other targets
#ifndef M_PI
#define M_PI (3.14159265358979323846264338327950288)
#endif


/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace OIConstants {
    constexpr static int GAMEPAD_BASE_LOCATION = 1;
    constexpr static int GAMEPAD_OPERATOR_LOCATION = 0;
}

namespace LimelightConstants {
    constexpr static int LED_MODE_ON = 3;
    constexpr static int LED_MODE_OFF = 1;
    constexpr static int TRACK_MODE_ON = 0;
    constexpr static int TRACK_MODE_OFF = 1;
}

namespace DIOPorts {
    constexpr static int MAG_ENCODER_PORTS[4] = {1, 2, 3, 4};
    constexpr static int BEAM_BREAK_PORT = 0;
    constexpr static int BLINKIN = 0;
}

namespace AnalogPorts {
    constexpr static int STAGE_BEAM_BREAK_PORT = 1;
    constexpr static int FEEDER_BEAM_BREAK_PORT = 0;
    constexpr static int INTAKE_BEAM_BREAK_PORT = 2;
}

namespace CANIDs {
    constexpr static int DRIVE_CANS[4] = {2, 4, 6, 8};
    constexpr static int AZIMUTH_CANS[4] = {1, 3, 5, 7};
    constexpr static int CANCODER_CANS[4] = {20, 21, 22, 23};
    constexpr static int PIGEON_CAN = 61;
    constexpr static int SCORER_WHEEL = 51;
    constexpr static int ELEV_WHEEL = 30;
    constexpr static int CLIMBER_LEAD = 53;
    constexpr static int CLIMBER_FOLLOW = 54;
    constexpr static int HALL_EFFECT = 46;
    constexpr static int CANDLE = 60;
    constexpr static int STAGING_LIDAR_SENSOR = 47;
    constexpr static int CRAB = 49;
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Wunused-variable"

// Constants that stay the same across bots should not go here
namespace Constants {
    // public:
        /*
        To use a global Constants:: instead of using an object, the function/variable used must be "static".
        The reason that can't happen here is that for functions/variables to be static, the values they use must also be static.
        Because GetTeamNumber isn't static, team number can't be static, and therefore none of the getters can be static either. 
        */

        enum Robot {
            Alpha,
            Gold,
            Black
        };

        static std::string getSerialNumber() {
            std::string env = std::getenv("serialnum");
            if (!env.empty()) return env;
            char buffer[8];
            std::ifstream{"/sys/firmware/devicetree/base/serial-number"}.read(buffer, 8);
            return std::string{buffer, 8};
        }
        static std::string serialNumber = getSerialNumber();
        static Robot getRobot() {
            if (serialNumber == ALPHA_SERIAL_NUMBER) return Robot::Alpha;
            else if (serialNumber == GOLD_SERIAL_NUMBER) return Robot::Gold;
            else return Robot::Black;
        }
        static Robot robot = getRobot();

        static bool roughTowardsRedAllianceWall = true;
        static double carpetGrainMultipler = 1.05;

        static units::degree_t pigeonMountPitch(){ switch (robot){ 
            case Robot::Alpha: return 0.037622284_deg; 
            case Robot::Gold: return 0.037622284_deg;  
            default: return 0.037622284_deg;
        }};
        static units::degree_t pigeonMountRoll(){ switch (robot){ 
            case Robot::Alpha: return -0.784180343_deg;
            case Robot::Gold: return -0.784180343_deg;
            default: return -0.784180343_deg;
        }};
        static units::degree_t pigeonMountYaw(){ switch (robot){ 
            case Robot::Alpha: return -90.230049133_deg; 
            case Robot::Gold: return -90.230049133_deg;
            default: return -90.230049133_deg; 
        }};

        static std::vector<units::turn_t> swerveZeros(){ switch (robot){
            case Robot::Alpha: return {0.4240722_tr, 0.85498046875_tr, 0.471924_tr, 0.081299_tr};
            case Robot::Gold: return {0.661133_tr, 0.093018_tr, 0.345703_tr, 0.38208_tr};
            default: return {0.4240722_tr, 0.85498046875_tr, 0.471924_tr, 0.081299_tr};
        }};
        static double driveGearRatio(){ switch (robot){
            case Robot::Alpha: return 5.51f;
            case Robot::Gold: return 6.48;
            default: return 6.48f;
        }};
        static double azimuthGearRatio(){ switch (robot){
            case Robot::Alpha: return 13.37f;
            case Robot::Gold: return 12.1f;
            default: return 12.1f;
        }};
        static units::meter_t moduleDiff(){ switch (robot){
            case Robot::Alpha: return 0.295_m;
            case Robot::Gold: return 0.295_m;
            default: return 0.295_m;
        }};
        static units::meter_t driveBaseRadius(){ switch (robot){
            case Robot::Alpha: return 0.4175_m; 
            case Robot::Gold: return 0.413_m; 
            default: return 0.413_m;
        }};

        static std::vector<bool> swerveDrivesReversals(){ switch (robot){
            case Robot::Alpha: return {true, false, true, false};
            case Robot::Gold: return {true, true, true, true};
            default: return {true, true, true, true};
        }};
        static std::vector<bool> swerveAzimuthsReversals(){ switch (robot){
            case Robot::Alpha: return {true, false, true, false};
            case Robot::Gold: return {false, false, false, false};
            default: return {false, false, false, false};
        }};

        static double azimuthKP(){ switch (robot) {
            case Robot::Alpha: return 100.0;
            case Robot::Gold: return 100.0;
            default: return 100.0;
        }};
        static units::turns_per_second_t azimuthKVel(){ switch (robot) {
            case Robot::Alpha: return 7.9_tps;
            case Robot::Gold: return 8.26_tps;
            default: return 8.26_tps;
        }};
        static units::turns_per_second_squared_t azimuthKAcc() { switch (robot) {
            case Robot::Alpha: return 1000_tr_per_s_sq;
            case Robot::Gold: return 1000_tr_per_s_sq;
            default: return 1000_tr_per_s_sq;
        }};

        static double driveKP(){ switch (robot) {
            case Robot::Alpha: return 5.0;
            case Robot::Gold: return 5.0;
            default: return 5.0;
        }};
        static units::meters_per_second_t driveKVel(){ switch (robot) {
            case Robot::Alpha: return 5.36_mps;
            case Robot::Gold: return 4.717_mps;
            default: return 4.717_mps;
        }};
        static units::meters_per_second_squared_t driveKAcc(){ switch (robot) {
            case Robot::Alpha: return 100_mps_sq;
            case Robot::Gold: return 100_mps_sq;
            default: return 100_mps_sq;
        }};

        static frc::Pose3d mintCameraPosition(){ switch (robot) {
            case Robot::Alpha: return frc::Pose3d(
                (6_in + 1.5625_in) - 14_in,
                6.75_in,
                1.25_in + 1.75_in + 1.5_in + 6_in + 2_in,
                frc::Rotation3d(
                    -3.3_deg,
                    -1.7_deg,
                    90_deg
                )
            );
            case Robot::Gold: return frc::Pose3d();
            default: return frc::Pose3d();
        }};

        static frc::Pose3d vanillaCameraPosition(){ switch (robot) {
            case Robot::Alpha: return frc::Pose3d();
            case Robot::Gold: return frc::Pose3d();
            default: return frc::Pose3d();
        }};

        static frc::Pose3d chocolateCameraPosition(){ switch (robot) {
            case Robot::Alpha: return frc::Pose3d();
            case Robot::Gold: return frc::Pose3d();
            default: return frc::Pose3d();
        }};

        static frc::Pose3d lemonCameraPosition(){ switch (robot){
            case Robot::Alpha: return frc::Pose3d();
            case Robot::Gold: return frc::Pose3d();
            default: return frc::Pose3d();
        }};

        static frc::Pose3d mangoCameraPosition(){ switch (robot){
            case Robot::Alpha: return frc::Pose3d(
                14_in - (1.5625_in + 5.875_in),
                8.5_in,
                10.625_in,
                frc::Rotation3d(
                    1.4_deg,
                    0.4_deg,
                    90_deg
                )
            );
            case Robot::Gold: return frc::Pose3d();
            default: return frc::Pose3d();
        }};

        static frc::Pose3d berryCameraPosition(){ switch (robot){
            case Robot::Alpha: return frc::Pose3d();
            case Robot::Gold: return frc::Pose3d();
            default: return frc::Pose3d();
        }};

        static std::vector<std::pair<const char*, frc::Pose3d>> aprilCameras{
                    std::pair("limelight-mint", mintCameraPosition()),
                    std::pair("limelight-choco", chocolateCameraPosition()),
                    std::pair("limelight-mango", mangoCameraPosition()),
                    std::pair("limelight-vanilla", vanillaCameraPosition()),
                    std::pair("limelight-berry", berryCameraPosition())
        };
}
#pragma GCC diagnostic pop

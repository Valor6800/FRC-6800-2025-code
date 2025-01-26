/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */ /* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "frc/RobotController.h"
#include <cmath>
#include <iostream> 
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


#define ALPHA_TEAM_NUMBER 6800
#define GOLD_TEAM_NUMBER 6808
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
    constexpr static int SHOOTER_CANCODER = 24;
    constexpr static int PIGEON_CAN = 61;
    constexpr static int INTERNAL_INTAKE = 8;
    constexpr static int INTERNAL_INTAKE_V2 = 9;
    constexpr static int PIVOT_LEAD = 30;
    constexpr static int PIVOT_FOLLOW = 31;
    constexpr static int RIGHT_SHOOTER_WHEEL= 16;
    constexpr static int RIGHT_SHOOTER_WHEEL2 = 18;
    constexpr static int LEFT_SHOOTER_WHEEL = 15; // Relative to shooter being forward
    constexpr static int LEFT_SHOOTER_WHEEL2 = 17;
    constexpr static int SCORER_WHEEL = 51;
    constexpr static int ELEV_WHEEL = 30;
    constexpr static int FEEDER = 14;
    constexpr static int CLIMBER_LEAD = 53;
    constexpr static int CLIMBER_FOLLOW = 54;
    constexpr static int AMP = 10;
    constexpr static int HALL_EFFECT = 46;
    constexpr static int CANDLE = 60;
    constexpr static int FRONT_LIDAR_SENSOR = 49;
    constexpr static int CRABB = 56;
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
        static int teamNumber = frc::RobotController::GetTeamNumber();;

        static bool roughTowardsRedAllianceWall = true;
        static double carpetGrainMultipler = 1.05;

        static units::degree_t pigeonMountPitch(){ switch (teamNumber){ 
            case ALPHA_TEAM_NUMBER: return 0.037622284_deg; 
            case GOLD_TEAM_NUMBER: return 0.037622284_deg;  
            default: return 0.037622284_deg;
        }};
        static units::degree_t pigeonMountRoll(){ switch (teamNumber){ 
            case ALPHA_TEAM_NUMBER: return -0.784180343_deg;
            case GOLD_TEAM_NUMBER: return -0.784180343_deg;
            default: return -0.784180343_deg;
        }};
        static units::degree_t pigeonMountYaw(){ switch (teamNumber){ 
            case ALPHA_TEAM_NUMBER: return -90.230049133_deg; 
            case GOLD_TEAM_NUMBER: return -90.230049133_deg;
            default: return -90.230049133_deg; 
        }};

        static std::vector<units::turn_t> swerveZeros(){ switch (teamNumber){
            case ALPHA_TEAM_NUMBER: return {0.4240722_tr, 0.85498046875_tr, 0.471924_tr, 0.081299_tr};
            case GOLD_TEAM_NUMBER: return {0.4240722_tr, 0.85498046875_tr, 0.471924_tr, 0.081299_tr};
            default: return {0.4240722_tr, 0.85498046875_tr, 0.471924_tr, 0.081299_tr};
        }};
        static double driveGearRatio(){ switch (teamNumber){
            case ALPHA_TEAM_NUMBER: return 5.51f;
            case GOLD_TEAM_NUMBER: return 5.51f;
            default: return 5.51f;
        }};
        static double azimuthGearRatio(){ switch (teamNumber){
            case ALPHA_TEAM_NUMBER: return 13.37f;
            case GOLD_TEAM_NUMBER: return 13.37f;
            default: return 13.37f;
        }};
        static units::meter_t moduleDiff(){ switch (teamNumber){
            case ALPHA_TEAM_NUMBER: return 0.295_m;
            case GOLD_TEAM_NUMBER: return 0.295_m;
            default: return 0.295_m;
        }};
        static units::meter_t driveBaseRadius(){ switch (teamNumber){
            case ALPHA_TEAM_NUMBER: return 0.4175_m; 
            case GOLD_TEAM_NUMBER: return 0.4175_m; 
            default: return 0.4175_m;
        }};

        static std::vector<bool> swerveDrivesReversals(){ switch (teamNumber){
            case ALPHA_TEAM_NUMBER: return {true, false, true, false};
            case GOLD_TEAM_NUMBER: return {false, false, false, false};
            default: return {false, false, false, false};
        }};
        static std::vector<bool> swerveAzimuthsReversals(){ switch (teamNumber){
            case ALPHA_TEAM_NUMBER: return {true, false, true, false};
            case GOLD_TEAM_NUMBER: return {false, false, false, false};
            default: return {false, false, false, false};
        }};

        static double azimuthKP(){ switch (teamNumber) {
            case ALPHA_TEAM_NUMBER: return 100.0;
            case GOLD_TEAM_NUMBER: return 100.0;
            default: return 100.0;
        }};
        static units::turns_per_second_t azimuthKVel(){ switch (teamNumber) {
            case ALPHA_TEAM_NUMBER: return 7.9_tps;
            case GOLD_TEAM_NUMBER: return 7.9_tps;
            default: return 7.9_tps;
        }};
        static units::turns_per_second_squared_t azimuthKAcc() { switch (teamNumber) {
            case ALPHA_TEAM_NUMBER: return 1000_tr_per_s_sq;
            case GOLD_TEAM_NUMBER: return 1000_tr_per_s_sq;
            default: return 1000_tr_per_s_sq;
        }};

        static double driveKP(){ switch (teamNumber) {
            case ALPHA_TEAM_NUMBER: return 5.0;
            case GOLD_TEAM_NUMBER: return 5.0;
            default: return 5.0;
        }};
        static units::meters_per_second_t driveKVel(){ switch (teamNumber) {
            case ALPHA_TEAM_NUMBER: return 5.36_mps;
            case GOLD_TEAM_NUMBER: return 5.36_mps;
            default: return 5.36_mps;
        }};
        static units::meters_per_second_squared_t driveKAcc(){ switch (teamNumber) {
            case ALPHA_TEAM_NUMBER: return 100_mps_sq;
            case GOLD_TEAM_NUMBER: return 100_mps_sq;
            default: return 100_mps_sq;
        }};

        static frc::Pose3d mintCameraPosition(){ switch (teamNumber) {
            case ALPHA_TEAM_NUMBER: return frc::Pose3d(
                (6_in + 1.5625_in) - 14_in,
                6.75_in,
                1.25_in + 1.75_in + 1.5_in + 6_in,
                frc::Rotation3d(
                    -2.1_deg,
                    -1.6_deg,
                    90_deg
                )
            );
            case GOLD_TEAM_NUMBER: return frc::Pose3d();
            default: return frc::Pose3d();
        }};

        static frc::Pose3d vanillaCameraPosition(){ switch (teamNumber) {
            case ALPHA_TEAM_NUMBER: return frc::Pose3d();
            case GOLD_TEAM_NUMBER: return frc::Pose3d();
            default: return frc::Pose3d();
        }};

        static frc::Pose3d chocolateCameraPosition(){ switch (teamNumber) {
            case ALPHA_TEAM_NUMBER: return frc::Pose3d();
            case GOLD_TEAM_NUMBER: return frc::Pose3d();
            default: return frc::Pose3d();
        }};

        static frc::Pose3d lemonCameraPosition(){ switch (teamNumber){
            case ALPHA_TEAM_NUMBER: return frc::Pose3d();
            case GOLD_TEAM_NUMBER: return frc::Pose3d();
            default: return frc::Pose3d();
        }};

        static frc::Pose3d mangoCameraPosition(){ switch (teamNumber){
            case ALPHA_TEAM_NUMBER: return frc::Pose3d(
                14_in - (1.5625_in + 5.875_in),
                8.5_in,
                10.625_in,
                frc::Rotation3d(
                    1.4_deg,
                    0.4_deg,
                    90_deg
                )
            );
            case GOLD_TEAM_NUMBER: return frc::Pose3d();
            default: return frc::Pose3d();
        }};

        static frc::Pose3d berryCameraPosition(){ switch (teamNumber){
            case ALPHA_TEAM_NUMBER: return frc::Pose3d();
            case GOLD_TEAM_NUMBER: return frc::Pose3d();
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

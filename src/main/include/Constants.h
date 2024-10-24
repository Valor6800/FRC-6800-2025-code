/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <cmath>
#include <iostream>
#include <frc/RobotController.h>
#include "frc/geometry/Rotation3d.h"
#include "units/angle.h"
#include "units/length.h"
#include <string>
#include <vector>
#include <cscore.h>
#include <frc/geometry/Pose3d.h>
#include "networktables/NetworkTable.h"


#define ALPHA_TEAM_NUMBER 6801
#define SIDE_SWIPE_TEAM_NUMBER 6808
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
    constexpr static int HALL_EFFECT = 2; //rando number
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
    constexpr static int FEEDER = 14;
    constexpr static int CLIMBER_LEAD = 32;
    constexpr static int CLIMBER_FOLLOW = 33;
    constexpr static int AMP = 10;
    constexpr static int CANDLE = 60;
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
        // void getMacAddress(){
        //     std::vector<std::string> v = cs::GetNetworkInterfaces();
        //     nt::NetworkTableInstance::GetDefault().GetTable("constants maybe")
        // }

        static bool roughTowardsRedAllianceWall = true;
        static double carpetGrainMultipler = 1.05;

        static double shooterPivotOffset(){ switch (teamNumber) {
            case ALPHA_TEAM_NUMBER: return 0;
            case SIDE_SWIPE_TEAM_NUMBER: return 0;
            default: return 51.9;
        }};
        static units::degree_t pigeonMountPitch(){ switch (teamNumber){ 
            case ALPHA_TEAM_NUMBER: return 0_deg; 
            case SIDE_SWIPE_TEAM_NUMBER: return 0_deg;  
            default: return 0.037622284_deg;
        }};
        static units::degree_t pigeonMountRoll(){ switch (teamNumber){ 
            case ALPHA_TEAM_NUMBER: return -0.395508_deg;
            case SIDE_SWIPE_TEAM_NUMBER: return 0_deg; // Temp value; TODO: Change it  
            default: return -0.784180343_deg;
        }};
        static units::degree_t pigeonMountYaw(){ switch (teamNumber){ 
            case ALPHA_TEAM_NUMBER: return -1.477661_deg; 
            case SIDE_SWIPE_TEAM_NUMBER: return 0_deg; // Temp value; TODO: Change it  
            default: return -90.230049133_deg; 
        }};

        static std::vector<double> swerveZeros(){ switch (teamNumber){
            case ALPHA_TEAM_NUMBER: return {0.3867, 0.8890, 0.0763, 0.610};
            case SIDE_SWIPE_TEAM_NUMBER: return {0.3106, 0.4369, 0.4780, 0.7372};
            default: return  {0.4240722, .85498046875, 0.471924, 0.081299}; // 0.86669921875 {0.4240722, 0.857666, 0.471924, 0.078125};
        }};

        static units::meter_t moduleDiff(){ switch (teamNumber){
            case ALPHA_TEAM_NUMBER: return 0.2413_m;
            case SIDE_SWIPE_TEAM_NUMBER: return 0.2_m;
            default: return 0.295_m;
        }};
        static units::meter_t driveBaseRadius(){ switch (teamNumber){
            case ALPHA_TEAM_NUMBER: return 0.36_m; 
            case SIDE_SWIPE_TEAM_NUMBER: return 0.3_m; 
            default: return 0.4175_m;
        }};

        static std::vector<bool> swerveDrivesReversals(){ switch (teamNumber){
            case ALPHA_TEAM_NUMBER: return {true, false, false, false};
            case SIDE_SWIPE_TEAM_NUMBER: return {false, false, false, false};
            default: return {true, false, true, false};
        }};
        static std::vector<bool> swerveAzimuthsReversals(){ switch (teamNumber){
            case ALPHA_TEAM_NUMBER: return {true, false, true, true};
            case SIDE_SWIPE_TEAM_NUMBER: return {true, true, true, true};
            default: return {true, false, true, false};
        }};

        static double azimuthKP(){ switch (teamNumber) {
            case ALPHA_TEAM_NUMBER: return 0.00025;
            case SIDE_SWIPE_TEAM_NUMBER: return 0.00001;
            default: return 100.0;
        }};
        static double azimuthKVel(){ switch (teamNumber) {
            case ALPHA_TEAM_NUMBER: return 6.983;
            case SIDE_SWIPE_TEAM_NUMBER: return 6.983;
            default: return 7.9;
        }};
        static double azimuthKAcc(){ switch (teamNumber) {
            case ALPHA_TEAM_NUMBER: return 200.0;
            case SIDE_SWIPE_TEAM_NUMBER: return 200.0;
            default: return 1000.0;
        }};

        static double driveKP(){ switch (teamNumber) {
            case ALPHA_TEAM_NUMBER: return 0.00001;
            case SIDE_SWIPE_TEAM_NUMBER: return 0.00001;
            default: return 5.0;
        }};
        static double driveKVel(){ switch (teamNumber) {
            case ALPHA_TEAM_NUMBER: return 6.0;
            case SIDE_SWIPE_TEAM_NUMBER: return 6.0;
            default: return 5.36;
        }};
        static double driveKAcc(){ switch (teamNumber) {
            case ALPHA_TEAM_NUMBER: return 120.0;
            case SIDE_SWIPE_TEAM_NUMBER: return 120.0;
            default: return 100.0;
        }};

        static frc::Pose3d mintCameraPosition(){ switch (teamNumber) {
            case ALPHA_TEAM_NUMBER: return frc::Pose3d();
            case SIDE_SWIPE_TEAM_NUMBER: return frc::Pose3d{
                0.2921_m, //x
                -0.16906875_m, //y
                0.28575_m, //z
                frc::Rotation3d{
                    -180_deg, 
                    28_deg,
                    0_deg
                }
            }; // Temp value; TODO: Change it
            default: return frc::Pose3d{
                -1_in, //x
                .675_in, //y
                23.0675_in, //z
                frc::Rotation3d{
                    0_deg, 
                    27_deg,
                    -180_deg
                }
            };
        }};

        /* Back Front Camera Mount
         * 
         * x: -1_in
         * y: +0.675_in
         * z: 23.0675_in
         * yaw: -180_deg
         * pitch: 32.3_deg
         * roll: 0_deg
         */

        /* Top Front Camera Mount
         * 
         * x: 4_in
         * y: -3.5_in
         * z: 21.75_in
         * yaw: 0_deg
         * pitch: 32.7_deg
         * roll: 0_deg
         */

        static frc::Pose3d vanillaCameraPosition(){ switch (teamNumber) {
            case ALPHA_TEAM_NUMBER: return frc::Pose3d();
            case SIDE_SWIPE_TEAM_NUMBER: return frc::Pose3d(
                0.2921_m, //x
                -0.16906875_m, //y
                0.28575_m, //z
                frc::Rotation3d{
                    -180_deg, 
                    28_deg,
                    0_deg
                }); // Temp value; TODO: Change it
            default: return frc::Pose3d(
                -11.535_in, //x
                11.513_in, //y
                10.175_in, //z
                frc::Rotation3d(
                    0_deg,
                    37_deg,
                    125_deg
                )
            );
        }};

        static frc::Pose3d chocolateCameraPosition(){ switch (teamNumber) {
            case ALPHA_TEAM_NUMBER: return frc::Pose3d();
            case SIDE_SWIPE_TEAM_NUMBER: return frc::Pose3d();
            default: return frc::Pose3d(
                -11.675_in, //x
                -11.217_in, //y
                10.175_in, //z
                frc::Rotation3d(
                    0_deg,
                    39.7_deg,
                    -125_deg
                )
            );
        }};

        static frc::Pose3d lemonCameraPosition(){ switch (teamNumber){
            case ALPHA_TEAM_NUMBER: return frc::Pose3d{
                -0.0635_m,
                0.3175_m,
                0.6731_m,
                frc::Rotation3d{
                    0_deg,
                    28_deg,
                    0_deg
                }
            };
            case SIDE_SWIPE_TEAM_NUMBER: return frc::Pose3d{
                -0.0635_m,
                0.3175_m,
                0.6731_m,
                frc::Rotation3d{
                    0_deg,
                    28_deg,
                    0_deg
                }
            }; // Temp value; TODO: Change it
            default: return frc::Pose3d{
                -0.0635_m,
                0.3175_m,
                0.6731_m,
                frc::Rotation3d{
                    0_deg,
                    28_deg,
                    0_deg
                }
            }; // Temp value; TODO: Change it
        }};

        static frc::Pose3d mangoCameraPosition(){ switch (teamNumber){
            case ALPHA_TEAM_NUMBER: return frc::Pose3d{
                4_in,
                -3.5_in,
                21.75_in,
                frc::Rotation3d{
                    0_deg,
                    32.7_deg,
                    0_deg
                }
            }; // Temp value; TODO: Change it
            case SIDE_SWIPE_TEAM_NUMBER: return frc::Pose3d{
                4_in,
                -3.5_in,
                21.75_in,
                frc::Rotation3d{
                    0_deg,
                    32.7_deg,
                    0_deg
                }
            }; // Temp value; TODO: Change it
            default: return frc::Pose3d{
                3.701_in, // 4
                -2.964_in, // -3.5
                23.915_in, // 21.75 
                frc::Rotation3d{
                    0_deg,
                    27_deg, //32.7
                    0_deg
                }
            };
        }};

        static std::vector<std::pair<const char*, frc::Pose3d>> aprilCameras{
                    std::pair("limelight-mint", mintCameraPosition()),
                    std::pair("limelight-lemon", lemonCameraPosition()),
                    std::pair("limelight-choco", chocolateCameraPosition()),
                    std::pair("limelight-vanilla", vanillaCameraPosition()),
                    std::pair("limelight-mango", mangoCameraPosition())
        };
}
#pragma GCC diagnostic pop

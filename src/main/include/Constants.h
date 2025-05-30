/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */ /* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "Eigen/Core"
#include "frc/DriverStation.h"
#include "frc/RobotController.h"
#include <cmath>
#include <iostream> 
#include <fstream>
#include <frc/RobotController.h>
#include <frc/geometry/Rotation3d.h>
#include <numbers>
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

enum Direction {
    LEFT,
    RIGHT,
    NONE
};

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

namespace LEDConstants {
    constexpr static int LED_COUNT = 62;
    constexpr static int LED_SEGMENTS = 6;

    constexpr static int LED_POS_ELEVATOR = 4;
    constexpr static int LED_POS_CANDI = 6;
    constexpr static int LED_POS_CLIMBER = 5;
    constexpr static int LED_POS_ELEVATOR_NOT_ZERO = 7;
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
    constexpr static int SCORER_WHEEL = 11;
    constexpr static int FUNNEL = 12;
    constexpr static int ELEV_WHEEL = 30;
    constexpr static int CLIMBER_LEAD = 32;
    constexpr static int CLIMBER_FOLLOW = 31;
    constexpr static int HALL_EFFECT = 46;
    constexpr static int CANDLE = 60;
    constexpr static int STAGING_LIDAR_SENSOR = 47;
    constexpr static int LEFT_CAN_RANGE_DRIVETRAIN_SENSOR = 48;
    constexpr static int RIGHT_CAN_RANGE_DRIVETRAIN_SENSOR = 49;
    constexpr static int ELEVATOR_CAN = 24;
    constexpr static int CLIMBER_CAN = 25;
    constexpr static int SCORER_PIVOT_MOTOR = 33;
    constexpr static int SCORER_PIVOT_CAN = 26;
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
            case Robot::Gold: return 1.0285918_deg;  
            default: return 0.037622284_deg;
        }};
        static units::degree_t pigeonMountRoll(){ switch (robot){ 
            case Robot::Alpha: return -0.784180343_deg;
            case Robot::Gold: return -179.45845_deg;
            default: return -0.784180343_deg;
        }};
        static units::degree_t pigeonMountYaw(){ switch (robot){ 
            case Robot::Alpha: return -90.230049133_deg; 
            case Robot::Gold: return 88.945129_deg;
            default: return -90.230049133_deg; 
        }};

        static std::vector<units::turn_t> swerveZeros(){ switch (robot){
            case Robot::Alpha: return {0.4240722_tr, 0.85498046875_tr, 0.471924_tr, 0.081299_tr};
            case Robot::Gold: return {0.4111328_tr, 0.8432617_tr, 0.09814453_tr, 0.134521484_tr};
            default: return {0.88134765625_tr, 0.684326171875_tr, 0.183349609375_tr, 0.9716796875_tr};
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
            default: return {false, false, false, false};
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
            case Robot::Gold: return 3.0; // 4
            default: return 3.0;
        }};
        static double driveKD(){ switch(robot){
            case Robot::Alpha: return 0.01;
            case Robot::Gold: return 0; // 0.001
            default: return 0.0;
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

        static double thresholdDistance() { switch (robot) {
            case Robot::Alpha: return 0.14;
            case Robot::Gold: return 0.1;
            default: return 0.1;
        }};

        static std::vector<int> getModuleCoordsX(){switch(robot){
            case Robot::Alpha: return {1, 1, -1, -1}; // {1, -1, -1, 1}
            case Robot::Gold: return {-1, 1, 1, -1};
            default: return {-1, 1, 1, -1};
        }};

        static std::vector<int> getModuleCoordsY(){switch(robot){
            case Robot::Alpha: return {1, -1, -1, 1}; // {-1, -1, 1, 1}
            case Robot::Gold: return {1, 1, -1, -1};
            default: return {1, 1, -1, -1}; 
        }};

        static valor::PhoenixControllerType getScorerMotorType() { switch (robot) {
            case Robot::Alpha: return valor::PhoenixControllerType::FALCON_FOC;
            case Robot::Gold: return valor::PhoenixControllerType::KRAKEN_X44;
            default: return valor::PhoenixControllerType::KRAKEN_X44;
        }};

        static valor::PhoenixControllerType getScorerPivotMotorType() { switch (robot) {
            case Robot::Alpha: return valor::PhoenixControllerType::FALCON_FOC;
            case Robot::Gold: return valor::PhoenixControllerType::KRAKEN_X44;
            default: return valor::PhoenixControllerType::KRAKEN_X44;
        }};

        static valor::PhoenixControllerType getFunnelMotorType() { switch (robot) {
            case Robot::Alpha: return valor::PhoenixControllerType::FALCON_FOC;
            case Robot::Gold: return valor::PhoenixControllerType::KRAKEN_X44;
            default: return valor::PhoenixControllerType::KRAKEN_X44;
        }};

        static valor::PhoenixControllerType getAzimuthMotorType() {switch (robot ){
            case Robot::Alpha: return valor::PhoenixControllerType::FALCON_FOC;
            case Robot::Gold: return valor::PhoenixControllerType::KRAKEN_X60_FOC;
            default: return valor::PhoenixControllerType::KRAKEN_X60_FOC;
        }}

        /** Process for ReZeroing:
         *      1. Move elevator position to when the hall effect just trips
         *      2. ReZero elevator cancoder using phoenix tuner and record negated value
         *      3. Put in value from phoenix tuner into getElevatorMagnetOffset
         *      4. Deploy and validate position
         *      5. Position should be roughly 2.85_in when at the bottom
        */
        static units::angle::turn_t getElevatorMagnetOffset() { switch (robot) {
            case Robot::Alpha: return 0.10327_tr;
            case Robot::Gold: return 0.218017578125_tr; // Phoenix Tuner: -.218017578125
            default: return 0.764893_tr;
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
            default: return frc::Pose3d(
                10_in,
                -6.5_in,
                8.62_in, // change Z
                frc::Rotation3d(
                    0_deg,
                    18_deg,
                    0_deg
                )
            );
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

        static frc::Pose3d rockyCameraPosition(){ switch (robot){
            case Robot::Alpha: return frc::Pose3d();
            case Robot::Gold: return frc::Pose3d(
                10_in,
                -6.5_in,
                8.62_in, // change Z
                frc::Rotation3d(
                    0_deg,
                    18_deg,
                    0_deg
                )
            );
            default: return frc::Pose3d();
        }};

        static frc::Pose3d limeCameraPosition(){ switch (robot){
            case Robot::Alpha: return frc::Pose3d();
            case Robot::Gold: return frc::Pose3d( 
                10_in,
                6.5_in,
                8.62_in, // change Z
                frc::Rotation3d(
                    0_deg,
                    18_deg,
                    0_deg
                )
);
            default: return frc::Pose3d(
                10_in,
                6.5_in,
                8.62_in, // change Z
                frc::Rotation3d(
                    0_deg,
                    18_deg,
                    0_deg
                )
            );
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
            default: return frc::Pose3d(
                10_in,
                6.5_in,
                8.62_in, // change Z
                frc::Rotation3d(
                    0_deg,
                    18_deg,
                    0_deg
                )
            );
        }};

        static frc::Pose3d berryCameraPosition(){ switch (robot){
            case Robot::Alpha: return frc::Pose3d();
            case Robot::Gold: return frc::Pose3d();
            default: return frc::Pose3d();
        }};

        typedef std::vector<std::pair<const char*, frc::Pose3d>> CameraVector;

        static CameraVector getAprilCameras(){ switch (robot){
            case Robot::Alpha: return CameraVector{
                std::pair("limelight-choco", chocolateCameraPosition()),
                std::pair("limelight-vanilla", vanillaCameraPosition()),
                std::pair("limelight-berry", berryCameraPosition()), 
            };
            case Robot::Gold: return CameraVector{
                std::pair("limelight-rocky", rockyCameraPosition()),
                std::pair("limelight-lime", limeCameraPosition())
            };
            default: return CameraVector{
                std::pair("limelight-mint", mintCameraPosition()),
                std::pair("limelight-mango", mangoCameraPosition())
            };
        }
        }

        static double cameraStandardDeviationLBF(units::meter_t distance) {
            return 3.73 * powf(10, -5) * powf(M_E, 3.43 * distance.value());
        }
    
        // Negative is leftwards; Positive is rightwards
        typedef int AprilTag;
        typedef std::unordered_map<Direction, units::inch_t> DirectionalOffSet;

        static std::unordered_map<AprilTag, DirectionalOffSet> bluePoleOffsets {
            {17,
                {
                    {LEFT, 0.5_in},
                    {RIGHT, 0.5_in},
                    {NONE, 2_in}
                }},
            {18,
                {
                    {LEFT, 0.5_in},
                    {RIGHT, 0.5_in},
                    {NONE, 2_in}
                }},
            {19,
                {
                    {LEFT, 0.5_in},
                    {RIGHT, 0.5_in},
                    {NONE, 2_in}
                }},
            {20,
                {
                    {LEFT, 0.5_in},
                    {RIGHT, 0.5_in},
                    {NONE, 2_in}
                }},
            {21,
                {
                    {LEFT, 0.5_in},
                    {RIGHT, 0.5_in},
                    {NONE, 2_in}
                }},
            {22,
                {
                    {LEFT, 0.5_in},
                    {RIGHT, 0.5_in},
                    {NONE, 2_in}
                }}
        };

        static std::unordered_map<AprilTag, DirectionalOffSet> redPoleOffsets {
            {6,
                {
                    {LEFT, 0.5_in},
                    {RIGHT, 0.5_in},
                    {NONE, 2_in}
                }},
            {7,
                {
                    {LEFT, 0.5_in},
                    {RIGHT, 0.5_in},
                    {NONE, 2_in}
                }},
            {8,
                {
                    {LEFT, 0.5_in},
                    {RIGHT, 0.5_in},
                    {NONE, 2_in}
                }},
            {9,
                {
                    {LEFT, 0.5_in},
                    {RIGHT, 0.5_in},
                    {NONE, 2_in}
                }},
            {10,
                {
                    {LEFT, 0.5_in},
                    {RIGHT, 0.5_in},
                    {NONE, 2_in}
                }},
            {11,
                {
                    {LEFT, 0.5_in},
                    {RIGHT, 0.5_in},
                    {NONE, 2_in}
                }},
        };

        namespace Scorer {
            enum ELEVATOR_STATE
            {
                MANUAL,
                STOWED,
                HP,
                ONE,
                TWO,
                THREE,
                FOUR,
                FLOOR,
            };

            enum PIVOT_STATE
            {
                CORAL_STOW,
                CORAL_GROUND,
                GROUND,
                PICK,
                PRESCORE,
                CARRY,
            };

            enum GAME_PIECE
            {
                CORAL,
                ALGEE,
            };

            typedef std::unordered_map<ELEVATOR_STATE, units::turns_per_second_t> ScoringSpeedMap;
            typedef std::unordered_map<GAME_PIECE, std::unordered_map<ELEVATOR_STATE, units::meter_t>> PositionMap;
            typedef std::unordered_map<PIVOT_STATE, units::turn_t> PositionAngleMap;

            static units::meters_per_second_t getAutoDunkSpeedLimitation(bool autoDunking) { switch (robot) {
                case Robot::Alpha: return autoDunking ? 0.25_mps : 0.6_mps ;
                case Robot::Gold: return autoDunking ? 0.35_mps : 0.6_mps ;
                default: return autoDunking ? 0.25_mps : 0.6_mps;
            }}

            static ScoringSpeedMap getScoringSpeedMap() { switch (robot) {
                case Robot::Alpha: return {
                    {ELEVATOR_STATE::FLOOR, 5_tps},
                    {ELEVATOR_STATE::ONE, 5_tps},
                    {ELEVATOR_STATE::TWO, 10_tps},
                    {ELEVATOR_STATE::THREE, 15_tps},
                    {ELEVATOR_STATE::FOUR, 20_tps}
                };
                case Robot::Gold: return {
                    {ELEVATOR_STATE::FLOOR, 27_tps},
                    {ELEVATOR_STATE::ONE, 12_tps},
                    {ELEVATOR_STATE::TWO, 9_tps},
                    {ELEVATOR_STATE::THREE, 9_tps},
                    {ELEVATOR_STATE::FOUR, 12_tps} // 25
                };
                default: return {
                    {ELEVATOR_STATE::FLOOR, 5_tps},
                    {ELEVATOR_STATE::ONE, 12_tps},
                    {ELEVATOR_STATE::TWO, 9_tps},
                    {ELEVATOR_STATE::THREE, 9_tps},
                    {ELEVATOR_STATE::FOUR, 12_tps} // 25
                };
            }}

            static PositionMap getPositionMap() { switch (robot) {
                case Robot::Alpha: 
                    return {
                        {
                            GAME_PIECE::CORAL,
                            {
                                { ELEVATOR_STATE::FLOOR, 5_in },
                                { ELEVATOR_STATE::STOWED, 5_in },
                                { ELEVATOR_STATE::HP, 7.5_in },
                                { ELEVATOR_STATE::ONE, 10_in },
                                { ELEVATOR_STATE::TWO, 12.5_in }, // raise by . an inch
                                { ELEVATOR_STATE::THREE, 15_in },
                                { ELEVATOR_STATE::FOUR, 17.5_in }
                            }
                        },
                        {
                            GAME_PIECE::ALGEE,
                            {
                                { ELEVATOR_STATE::FLOOR, 5_in },
                                { ELEVATOR_STATE::STOWED, 7.5_in },
                                { ELEVATOR_STATE::HP, 5_in },
                                { ELEVATOR_STATE::ONE, 5_in },
                                { ELEVATOR_STATE::TWO, 13.7_in },
                                { ELEVATOR_STATE::THREE, 14.5_in },
                                { ELEVATOR_STATE::FOUR, 17.5_in }
                            }
                        }
                    };
                case Robot::Gold:
                    return {
                        {
                            GAME_PIECE::CORAL,
                            {
                                { ELEVATOR_STATE::FLOOR, 3.625_in },
                                { ELEVATOR_STATE::STOWED, 10_in },
                                { ELEVATOR_STATE::HP, 3.25_in },
                                { ELEVATOR_STATE::ONE, 3.25_in },
                                { ELEVATOR_STATE::TWO, 13.4_in }, // raised by point 0.1 from 13.3
                                { ELEVATOR_STATE::THREE, 18.61667_in },
                                { ELEVATOR_STATE::FOUR, 26.5_in }
                            }
                        },
                        {
                            GAME_PIECE::ALGEE,
                            {
                                { ELEVATOR_STATE::FLOOR, 6.0_in },
                                { ELEVATOR_STATE::STOWED, 8.0_in},
                                { ELEVATOR_STATE::HP, 4.5_in},
                                { ELEVATOR_STATE::ONE, 3.15_in },
                                { ELEVATOR_STATE::TWO, 7.24_in},
                                { ELEVATOR_STATE::THREE, 12.43_in},
                                { ELEVATOR_STATE::FOUR, 29.9_in }
                            }
                        }
                    };
                default:
                    return {
                        {
                            GAME_PIECE::CORAL,
                            {
                                { ELEVATOR_STATE::FLOOR, 3.625_in },
                                { ELEVATOR_STATE::STOWED, 10_in },
                                { ELEVATOR_STATE::HP, 3.25_in },
                                { ELEVATOR_STATE::ONE, 3.25_in },
                                { ELEVATOR_STATE::TWO, 14_in },
                                { ELEVATOR_STATE::THREE, 19.25_in },
                                { ELEVATOR_STATE::FOUR, 26.5_in }
                            }
                        },
                        {
                            GAME_PIECE::ALGEE,
                            {
                                { ELEVATOR_STATE::FLOOR, 6.0_in },
                                { ELEVATOR_STATE::STOWED, 8.0_in},
                                { ELEVATOR_STATE::HP, 4.5_in},
                                { ELEVATOR_STATE::ONE, 3.15_in },
                                { ELEVATOR_STATE::TWO, 7.24_in},
                                { ELEVATOR_STATE::THREE, 12.43_in},
                                { ELEVATOR_STATE::FOUR, 29.9_in }
                            }
                        }
                    };
            }}

            static PositionAngleMap getPivotPositionMap() { switch (robot) {
                case Robot::Gold:
                    return {
                        { PIVOT_STATE::CORAL_STOW, 0.1250_tr },
                        { PIVOT_STATE::CORAL_GROUND, 0.637_tr },
                        { PIVOT_STATE::GROUND, 0.637_tr },
                        { PIVOT_STATE::PICK, 0.4_tr },
                        { PIVOT_STATE::PRESCORE, 0.4625_tr },
                        { PIVOT_STATE::CARRY, 0.27_tr },
                    };
                default:
                    return {
                        { PIVOT_STATE::CORAL_STOW, 0.1259_tr },
                        { PIVOT_STATE::CORAL_GROUND, 0.637_tr },
                        { PIVOT_STATE::GROUND, 0.637_tr },
                        { PIVOT_STATE::PICK, 0.4_tr },
                        { PIVOT_STATE::PRESCORE, 0.4625_tr },
                        { PIVOT_STATE::CARRY, 0.27_tr },
                    };
            }}

            static bool elevatorMotorInverted() { switch (robot) {
                case Robot::Alpha: return true;
                case Robot::Gold: return false;
                default: return false;
            }}

            static bool scorerMotorInverted() { switch (robot) {
                case Robot::Alpha: return true;
                case Robot::Gold: return true;
                default: return true;
            }}

            static bool scorerPivotInverted() { switch (robot) {
                case Robot::Alpha: return true;
                case Robot::Gold: return false;
                default: return false;
            }}

            static bool funnelInverted() { switch (robot) {
                case Robot::Alpha: return true;
                case Robot::Gold: return true;
                default: return true;
            }}

            static units::angle::turn_t scorerPivotMagnetOffset() { switch (robot) {
                case Robot::Alpha: return 1.37_tr;
                case Robot::Gold: return -0.846435125_tr; //setPoint - currentPos +/"-" currentOffset
                default: return -0.7521969219_tr;
            }}

            /// Amount of rotations needed for after detecting coral intake
            static units::angle::turn_t getIntakeTurns() { switch (robot) {
                case Robot::Alpha: return 1.37_tr;
                case Robot::Gold: return 0.25_tr;
                default: return 0.25_tr;
            }};

            /// Time to reach max velocity
            static units::second_t getElevMaxVelRampTime() { switch (robot) {
                case Robot::Alpha: return 1.0_s / 5;
                case Robot::Gold: return 12.6_s/60.0;
                default: return 12.6_s/60.0; // Use 12.6 / 16
            }}

            static double getScorerSensorToMech() { switch (robot) {
                case Robot::Alpha: return 2;
                case Robot::Gold: return 2.75;
                default: return 2.75;
            }}

            static double getFunnelSensorToMech() { switch (robot) {
                case Robot::Alpha: return 40/24;
                case Robot::Gold: return 40/24;
                default: return 40/24;
            }}

            static bool isElevatorClockwise() { switch (robot) {
                case Robot::Alpha: return false;
                case Robot::Gold: return false;
                default: return true;
            }};

            static units::angle::turn_t getElevatorAbsoluteRange() { switch (robot) {
                case Robot::Alpha: return .75_tr;
                case Robot::Gold: return 0.5_tr;
                default: return 0.5_tr;
            }};

            static valor::PIDF getElevatorPIDF() { switch (robot) {
                case Robot::Alpha: {
                    valor::PIDF pidf;
                    pidf.P = 10;
                    pidf.aFF = 0.5;
                    pidf.maxJerk = 80_tr_per_s_cu;
                    return pidf;
                }
                case Robot::Gold: {
                    valor::PIDF pidf;
                    pidf.P = 20;
                    pidf.aFF = 0.8;
                    pidf.maxJerk = 150_tr_per_s_cu;
                    pidf.D = 1.0;
                    return pidf;
                }
                default: {
                    valor::PIDF pidf;
                    pidf.P = 15;
                    pidf.aFF = 0.75;
                    pidf.maxJerk = 135_tr_per_s_cu;
                    return pidf;
                }
            }}

            static valor::PIDF getScorerPivotPIDF() { switch (robot) {
                case Robot::Alpha: {
                    valor::PIDF pidf;
                    pidf.P = 20;
                    return pidf;
                }
                default: {
                    valor::PIDF pidf;
                    pidf.P = 100;
                    pidf.aFF = 0.0;
                    pidf.aFFType = valor::FeedForwardType::CIRCULAR;
                    return pidf;
                }
            }}

            static valor::PIDF getScorerPIDF() { switch (robot) {
                case Robot::Alpha: {
                    valor::PIDF pidf;
                    pidf.P = 0.5;
                    pidf.S = 0.58;
                    return pidf;
                }
                case Robot::Gold: {
                    valor::PIDF pidf;
                    pidf.P = 1;
                    pidf.S = 0.32;
                    pidf.kV = 0.32;
                    return pidf;
                }
                default: {
                    valor::PIDF pidf;
                    pidf.P = 1;
                    pidf.S = 0.32;
                    pidf.kV = 0.32;
                    return pidf;
                }
            }}

            static valor::PIDF getFunnelPIDF() { switch (robot) {
                case Robot::Alpha: {
                    valor::PIDF pidf;
                    pidf.P = 0.25;
                    pidf.S = 0.1201;
                    pidf.kV = 0.12;
                    return pidf;
                }
                case Robot::Gold: {
                    valor::PIDF pidf;
                    pidf.P = 0.25;
                    pidf.S = 0.1201;
                    pidf.kV = 0.12;
                    return pidf;
                }
                default: {
                    valor::PIDF pidf;
                    pidf.P = 0.25;
                    pidf.S = 0.1201;
                    pidf.kV = 0.12;
                    return pidf;
                }
            }}

            static units::second_t getScorerMaxVelRampTime() { switch (robot) {
                case Robot::Alpha: return 1.0_s / 2;
                case Robot::Gold: return 1.0_s / 5;
                default: return 1.0_s / 5;
            }}
        }

        namespace Climber {
            static units::turn_t magnetOffset() { switch (robot) {
                case Robot::Alpha: return 0_tr;
                case Robot::Gold: return 0.0875_tr;
                default: return 0.0046386563_tr;
            }}

            static bool climbMotorInverted() { switch (robot) {
                case Robot::Alpha: return false;
                default: return true;
            }}

            static valor::PIDF getClimberPIDF() { switch (robot) {
                case Robot::Alpha: {
                    valor::PIDF pidf;
                    pidf.P = 100;
                    return pidf;
                }
                default: {
                    valor::PIDF pidf;
                    pidf.P = 10; //10
                    return pidf;
                }
            }}

            static valor::PIDF getClimberRetractPIDF() { switch (robot) {
                case Robot::Alpha: {
                    valor::PIDF pidf;
                    pidf.P = 100;
                    return pidf;
                }
                default: {
                    valor::PIDF pidf;
                    pidf.P = 50;
                    return pidf;
                }
            }}

            static units::second_t maxVelocityRampTime() { switch (robot) {
                case Robot::Alpha: return 1.0_s / 3;
                default: return 1.0_s / 5;
            }}

            static units::turn_t getForwardLimit() { switch (robot) {
                case Robot::Alpha: return 0.25_tr;
                case Robot::Gold: return 3.646_tr;
                default: return 3.646_tr;
            }}

            static units::turn_t getReverseLimit() { switch (robot) {
                case Robot::Alpha: return -0.2_tr;
                default: return 0.5_tr;
            }}
        }
}
#pragma GCC diagnostic pop

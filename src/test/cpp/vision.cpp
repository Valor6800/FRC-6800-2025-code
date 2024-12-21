#include "Eigen/Core"
#include <frc/geometry/Pose3d.h>
#include <iostream>

typedef Eigen::Matrix<double, 3, 3> RotationMatrix;

struct Test{
    double input_y ;
    double input_x ;
    double output_x ;
    double output_y ;
};

std::pair<double,double> test(Test t) {

    auto cameraPose =frc::Pose3d{
        14.0_in - 1.875_in, // 4
        -.25_in, // -3.5
        42.2275_cm, // 21.75 
        frc::Rotation3d{
            0_deg,
            -15.0_deg, //32.7
            0_deg
        }
    };
    auto cameraTranslation = Eigen::Matrix<double, 3, 1>{
        {cameraPose.X().value()},
        {cameraPose.Y().value()},
        {cameraPose.Z().value()}
    };
    //TODO: Make these constants
    auto rollMatrix = RotationMatrix{
        {cos(cameraPose.Rotation().X().value()), -sin(cameraPose.Rotation().X().value()), 0},
        {sin(cameraPose.Rotation().X().value()), cos(cameraPose.Rotation().X().value()), 0},
        {0, 0, 1}
    };

    //TODO: Make these constants
    auto yawMatrix = RotationMatrix{
        {1, 0, 0},
        {0, cos(cameraPose.Rotation().Z().value()), -sin(cameraPose.Rotation().Z().value())},
        {0, sin(cameraPose.Rotation().Z().value()), cos(cameraPose.Rotation().Z().value())}
    };
    
    //TODO: Make these constants
    auto pitchMatrix = RotationMatrix{
        {cos(cameraPose.Rotation().Y().value()), 0, sin(cameraPose.Rotation().Y().value())},
        {0, 1, 0},
        {-sin(cameraPose.Rotation().Y().value()), 0, cos(cameraPose.Rotation().Y().value())}
    };

    auto fullRotationMatrix = rollMatrix * yawMatrix * pitchMatrix;
    Eigen::Matrix<double, 3, 3> intrinsicMatrix{
        {1038.543, 0, 609.345},
        {0, 1037.537, 469.070},
        {0, 0, 1}
    };

    //TODO: Do this on init
    Eigen::Matrix<double, 3, 4> CameraPoseMatrix{
        {fullRotationMatrix.row(0)[0], fullRotationMatrix.row(0)[1], fullRotationMatrix.row(0)[2], cameraTranslation[0]},
        {fullRotationMatrix.row(1)[0], fullRotationMatrix.row(1)[1], fullRotationMatrix.row(1)[2], cameraTranslation[1]},
        {fullRotationMatrix.row(2)[0], fullRotationMatrix.row(2)[1], fullRotationMatrix.row(2)[2], cameraTranslation[2]},
    };

    Eigen::Matrix<double, 3, 4> inverseHomography = intrinsicMatrix * CameraPoseMatrix;
    
    // if (!hasTarget()) return;
    
    //TODO: make this suitable with enums for different types of limelights
    Eigen::Vector3d pixelCoord {
        {(62.5 - (2 * t.input_x)) / (2 * 62.5)},
        {(48.9 - (2 * t.input_y)) / (2 * 48.9)},
        {1}
    };

    auto gamePieceMatrix = inverseHomography.transpose() * pixelCoord;

    return std::pair(
        gamePieceMatrix.col(0)[0],
        gamePieceMatrix.col(0)[1]
    );



}

int main() {

    auto tests = {
        Test{0.91	,14.48	,187.96	,-61},
        Test{2.2	,13.39	,212.09	,-61},
        Test{0.23	,15.94	,171.1325	,-61},
        Test{-3.6	,19.49	,128.27	,-61},
        Test{-8.15	,22.77	,98.425	,-61},
        Test{4.56	,10.72	,269.24	,-61},
    };

    for (auto t : tests) {
        auto [x, y] = test(t);
        if ( x != t.output_x || y != t.output_y) {
            std::cout << "Not Equal: expected: (" << t.output_x << ", " << t.output_y << ") \n";
            std::cout << "got: (" << x << ", " << y << ")" << std::endl;
        }
    }
}


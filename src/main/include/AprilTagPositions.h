/*                                 Valor 6800                                 */
/* Copyright (c) 2025 Company Name. All Rights Reserved.                      */

#pragma once
#include <algorithm>
#include <unordered_map>
#include <utility>

#include <Eigen/Core>
#include <frc/DriverStation.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Transform3d.h>
#include <frc/geometry/Translation3d.h>
#include <gcem_incl/abs.hpp>

// aprilTagPoses is the tag position expressed in center space
namespace valor {

static const std::unordered_map<int, frc::Pose3d> blueReefAprilTagPoses{
    {17, frc::Pose3d(Eigen::Matrix4d{
             {-0.5000000000000002, 0.8660254037844384, 0, -4.700094},
             {-0.8660254037844384, -0.5000000000000002, 0, -0.7196820000000002},
             {0, 0, 1, 0.308102},
             {0, 0, 0, 1}})},

    {18, frc::Pose3d(Eigen::Matrix4d{
             {-1, -1.2246467991473532e-16, 0, -5.116399999999999},
             {1.2246467991473532e-16, -1, 0, -0.00009999999999976694},
             {0, 0, 1, 0.308102},
             {0, 0, 0, 1}})},

    {19, frc::Pose3d(Eigen::Matrix4d{
             {-0.4999999999999998, -0.8660254037844388, 0, -4.700094},
             {0.8660254037844388, -0.4999999999999998, 0, 0.7194820000000002},
             {0, 0, 1, 0.308102},
             {0, 0, 0, 1}})},

    {20, frc::Pose3d(Eigen::Matrix4d{
             {0.5000000000000001, -0.8660254037844386, 0, -3.8692599999999997},
             {0.8660254037844386, 0.5000000000000001, 0, 0.7194820000000002},
             {0, 0, 1, 0.308102},
             {0, 0, 0, 1}})},

    {21, frc::Pose3d(Eigen::Matrix4d{{1, 0, 0, -3.452953999999999},
                                     {0, 1, 0, -0.00009999999999976694},
                                     {0, 0, 1, 0.308102},
                                     {0, 0, 0, 1}})},

    {22, frc::Pose3d(Eigen::Matrix4d{
             {0.5000000000000001, 0.8660254037844386, 0, -3.8692599999999997},
             {-0.8660254037844386, 0.5000000000000001, 0, -0.7196820000000002},
             {0, 0, 1, 0.308102},
             {0, 0, 0, 1}})},
};

static const std::unordered_map<int, frc::Pose3d> redReefAprilTagPoses{
    {6, frc::Pose3d(Eigen::Matrix4d{
            {0.5000000000000001, 0.8660254037844386, 0, 4.700446000000001},
            {-0.8660254037844386, 0.5000000000000001, 0, -0.7196820000000002},
            {0, 0, 1, 0.308102},
            {0, 0, 0, 1}})},
    {7, frc::Pose3d(Eigen::Matrix4d{{1, 0, 0, 5.116498},
                                    {0, 1, 0, -0.00009999999999976694},
                                    {0, 0, 1, 0.308102},
                                    {0, 0, 0, 1}})},
    {8, frc::Pose3d(Eigen::Matrix4d{
            {0.5000000000000001, -0.8660254037844386, 0, 4.700446000000001},
            {0.8660254037844386, 0.5000000000000001, 0, 0.7194820000000002},
            {0, 0, 1, 0.308102},
            {0, 0, 0, 1}})},
    {9, frc::Pose3d(Eigen::Matrix4d{
            {-0.4999999999999998, -0.8660254037844388, 0, 3.869358},
            {0.8660254037844388, -0.4999999999999998, 0, 0.7194820000000002},
            {0, 0, 1, 0.308102},
            {0, 0, 0, 1}})},
    {10, frc::Pose3d(Eigen::Matrix4d{
             {-1, -1.2246467991473532e-16, 0, 3.4533059999999995},
             {1.2246467991473532e-16, -1, 0, -0.00009999999999976694},
             {0, 0, 1, 0.308102},
             {0, 0, 0, 1}})},

    {11, frc::Pose3d(Eigen::Matrix4d{
             {-0.5000000000000002, 0.8660254037844384, 0, 3.869358},
             {-0.8660254037844384, -0.5000000000000002, 0, -0.7196820000000002},
             {0, 0, 1, 0.308102},
             {0, 0, 0, 1}})},
};

static const std::unordered_map<int, frc::Pose3d> aprilTagPoses{
    {1, frc::Pose3d(Eigen::Matrix4d{
            {-0.5877852522924729, -0.8090169943749473, 0, 7.923198000000001},
            {0.8090169943749473, -0.5877852522924729, 0, -3.3706799999999997},
            {0, 0, 1, 1.4859},
            {0, 0, 0, 1}})},
    {2, frc::Pose3d(Eigen::Matrix4d{
            {-0.5877852522924734, 0.8090169943749473, 0, 7.923198000000001},
            {-0.8090169943749473, -0.5877852522924734, 0, 3.3704799999999997},
            {0, 0, 1, 1.4859},
            {0, 0, 0, 1}})},
    {3, frc::Pose3d(
            Eigen::Matrix4d{{-2.220446049250313e-16, 1, 0, 2.786809999999999},
                            {-1, -2.220446049250313e-16, 0, 4.02961},
                            {0, 0, 1, 1.30175},
                            {0, 0, 0, 1}})},
    {4, frc::Pose3d(Eigen::Matrix4d{
            {0.8660254037844387, 0, 0.49999999999999994, 0.5020799999999994},
            {0, 1, 0, 2.111656},
            {-0.49999999999999994, 0, 0.8660254037844387, 1.8679160000000001},
            {0, 0, 0, 1}})},
    {5, frc::Pose3d(Eigen::Matrix4d{
            {0.8660254037844387, 0, 0.49999999999999994, 0.5020799999999994},
            {0, 1, 0, -2.1110939999999996},
            {-0.49999999999999994, 0, 0.8660254037844387, 1.8679160000000001},
            {0, 0, 0, 1}})},
    {12, frc::Pose3d(Eigen::Matrix4d{
             {0.5877852522924731, -0.8090169943749473, 0, -7.922845999999999},
             {0.8090169943749473, 0.5877852522924731, 0, -3.3706799999999997},
             {0, 0, 1, 1.4859},
             {0, 0, 0, 1}})},

    {13, frc::Pose3d(Eigen::Matrix4d{
             {0.587785252292473, 0.8090169943749475, 0, -7.922845999999999},
             {-0.8090169943749475, 0.587785252292473, 0, 3.3704799999999997},
             {0, 0, 1, 1.4859},
             {0, 0, 0, 1}})},

    {14, frc::Pose3d(Eigen::Matrix4d{
             {-0.8660254037844388, -1.2246467991473532e-16,
              -0.49999999999999994, -0.501728},
             {1.0605752387249069e-16, -1, 6.123233995736766e-17, 2.111656},
             {-0.49999999999999994, 0, 0.8660254037844387, 1.8679160000000001},
             {0, 0, 0, 1}})},

    {15, frc::Pose3d(Eigen::Matrix4d{
             {-0.8660254037844388, -1.2246467991473532e-16,
              -0.49999999999999994, -0.501728},
             {1.0605752387249069e-16, -1, 6.123233995736766e-17,
              -2.1110939999999996},
             {-0.49999999999999994, 0, 0.8660254037844387, 1.8679160000000001},
             {0, 0, 0, 1}})},

    {16,
     frc::Pose3d(Eigen::Matrix4d{
         {-2.220446049250313e-16, -1.0000000000000002, 0, -2.7864579999999997},
         {1.0000000000000002, -2.220446049250313e-16, 0, -4.0298099999999994},
         {0, 0, 1, 1.30175},
         {0, 0, 0, 1}})},
};

inline bool isReefTag(int tagID) {
  bool isRedTag =
      frc::DriverStation::GetAlliance() == frc::DriverStation::kRed &&
      tagID >= 6 && tagID <= 11;
  bool isBlueTag =
      frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue &&
      tagID >= 17 && tagID <= 22;
  return isRedTag || isBlueTag;
}

inline std::pair<int, frc::Pose3d>
getNearestTag(frc::Pose2d source,
              std::unordered_map<int, frc::Pose3d> tagPoses) {
  return *std::min_element(
      tagPoses.begin(), tagPoses.end(),
      [source](const std::pair<int, frc::Pose3d> &a,
               const std::pair<int, frc::Pose3d> &b) {
        units::meter_t aDistance =
            source.Translation().Distance(a.second.ToPose2d().Translation());
        units::meter_t bDistance =
            source.Translation().Distance(b.second.ToPose2d().Translation());

        if (aDistance == bDistance) {
          return gcem::abs((source.Rotation() - a.second.ToPose2d().Rotation())
                               .Radians()
                               .value()) <
                 gcem::abs((source.Rotation() - b.second.ToPose2d().Rotation())
                               .Radians()
                               .value());
        }

        return aDistance < bDistance;
      });
}

inline std::pair<int, frc::Pose3d> getNearestWorldTag(frc::Pose2d source) {
  std::unordered_map<int, frc::Pose3d> allTags = aprilTagPoses;
  allTags.insert(blueReefAprilTagPoses.begin(), blueReefAprilTagPoses.end());
  allTags.insert(redReefAprilTagPoses.begin(), redReefAprilTagPoses.end());

  return getNearestTag(source, allTags);
}
} // namespace valor

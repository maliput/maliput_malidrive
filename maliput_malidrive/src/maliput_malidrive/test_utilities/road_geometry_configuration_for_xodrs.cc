// Copyright 2021 Toyota Research Institute
#include "maliput_malidrive/test_utilities/road_geometry_configuration_for_xodrs.h"

#include <unordered_map>

#include <maliput/api/road_geometry.h>
#include <maliput/math/vector.h>

#include "maliput_malidrive/constants.h"

namespace malidrive {
namespace test {

std::optional<builder::RoadGeometryConfiguration> GetRoadGeometryConfigurationFor(const std::string& xodr_file_name) {
  const static maliput::math::Vector3 kZeroVector{0., 0., 0.};
  const builder::BuildPolicy kBuildPolicy{builder::BuildPolicy::Type::kSequential};
  const builder::RoadGeometryConfiguration::SimplificationPolicy kSimplificationPolicy{
      builder::RoadGeometryConfiguration::SimplificationPolicy::kNone};
  const builder::RoadGeometryConfiguration::StandardStrictnessPolicy kStandardStrictnessPolicy{
      builder::RoadGeometryConfiguration::StandardStrictnessPolicy::kPermissive};
  const bool kOmitNondrivableLanes{false};

  const static std::unordered_map<std::string, builder::RoadGeometryConfiguration> kXodrConfigurations{
      {"SingleLane.xodr",
       builder::RoadGeometryConfiguration{
           maliput::api::RoadGeometryId{"LineSingleLane"},
           {"odr/SingleLane.xodr"},
           builder::RoadGeometryConfiguration::BuildTolerance{
               5e-2 /* linear_tolerance */, 5e-1 /*max_linear_tolerance*/, 1e-3 /* angular_tolerance */},
           constants::kScaleLength,
           kZeroVector,
           kBuildPolicy,
           kSimplificationPolicy,
           kStandardStrictnessPolicy,
           kOmitNondrivableLanes}},
      {"ArcLane.xodr",
       builder::RoadGeometryConfiguration{
           maliput::api::RoadGeometryId{"ArcSingleLane"},
           {"odr/ArcLane.xodr"},
           builder::RoadGeometryConfiguration::BuildTolerance{
               5e-2 /* linear_tolerance */, 5e-1 /*max_linear_tolerance*/, 1e-3 /* angular_tolerance */},
           constants::kScaleLength,
           kZeroVector,
           kBuildPolicy,
           kSimplificationPolicy,
           kStandardStrictnessPolicy,
           kOmitNondrivableLanes}},
      {"BikingLineLane.xodr",
       builder::RoadGeometryConfiguration{
           maliput::api::RoadGeometryId{"BikingLineLane"},
           {"odr/BikingLineLane.xodr"},
           builder::RoadGeometryConfiguration::BuildTolerance{
               5e-2 /* linear_tolerance */, 5e-1 /*max_linear_tolerance*/, 1e-3 /* angular_tolerance */},
           constants::kScaleLength,
           kZeroVector,
           kBuildPolicy,
           kSimplificationPolicy,
           kStandardStrictnessPolicy,
           kOmitNondrivableLanes}},
      {"DisconnectedRoadInJunction.xodr",
       builder::RoadGeometryConfiguration{
           maliput::api::RoadGeometryId{"DisconnectedRoadInJunction"},
           {"odr/DisconnectedRoadInJunction.xodr"},
           builder::RoadGeometryConfiguration::BuildTolerance{
               5e-2 /* linear_tolerance */, 5e-1 /*max_linear_tolerance*/, 1e-3 /* angular_tolerance */},
           constants::kScaleLength,
           kZeroVector,
           kBuildPolicy,
           kSimplificationPolicy,
           kStandardStrictnessPolicy,
           kOmitNondrivableLanes}},
      {"SShapeRoad.xodr",
       builder::RoadGeometryConfiguration{
           maliput::api::RoadGeometryId{"SShapeRoad"},
           {"odr/SShapeRoad.xodr"},
           builder::RoadGeometryConfiguration::BuildTolerance{
               5e-2 /* linear_tolerance */, 5e-1 /*max_linear_tolerance*/, 1e-3 /* angular_tolerance */},
           constants::kScaleLength,
           kZeroVector,
           kBuildPolicy,
           kSimplificationPolicy,
           kStandardStrictnessPolicy,
           kOmitNondrivableLanes}},
      {"LShapeRoad.xodr",
       builder::RoadGeometryConfiguration{
           maliput::api::RoadGeometryId{"LShapeRoad"},
           {"odr/LShapeRoad.xodr"},
           builder::RoadGeometryConfiguration::BuildTolerance{
               5e-2 /* linear_tolerance */, 5e-1 /*max_linear_tolerance*/, 1e-3 /* angular_tolerance */},
           constants::kScaleLength,
           kZeroVector,
           kBuildPolicy,
           kSimplificationPolicy,
           kStandardStrictnessPolicy,
           kOmitNondrivableLanes}},
      {"LShapeRoadVariableLanes.xodr",
       builder::RoadGeometryConfiguration{
           maliput::api::RoadGeometryId{"LShapeRoadVariableLanes"},
           {"odr/LShapeRoadVariableLanes.xodr"},
           builder::RoadGeometryConfiguration::BuildTolerance{
               5e-2 /* linear_tolerance */, 5e-1 /*max_linear_tolerance*/, 1e-3 /* angular_tolerance */},
           constants::kScaleLength,
           kZeroVector,
           kBuildPolicy,
           kSimplificationPolicy,
           kStandardStrictnessPolicy,
           kOmitNondrivableLanes}},
      {"LineMultipleSections.xodr",
       builder::RoadGeometryConfiguration{
           maliput::api::RoadGeometryId{"LineMultipleSections"},
           {"odr/LineMultipleSections.xodr"},
           builder::RoadGeometryConfiguration::BuildTolerance{
               5e-2 /* linear_tolerance */, 5e-1 /*max_linear_tolerance*/, 1e-3 /* angular_tolerance */},
           constants::kScaleLength,
           kZeroVector,
           kBuildPolicy,
           kSimplificationPolicy,
           kStandardStrictnessPolicy,
           kOmitNondrivableLanes}},
      {"LineMultipleSectionsMoreCases.xodr",
       builder::RoadGeometryConfiguration{
           maliput::api::RoadGeometryId{"LineMultipleSectionsMoreCases"},
           {"odr/LineMultipleSectionsMoreCases.xodr"},
           builder::RoadGeometryConfiguration::BuildTolerance{
               5e-2 /* linear_tolerance */, 5e-1 /*max_linear_tolerance*/, 1e-3 /* angular_tolerance */},
           constants::kScaleLength,
           kZeroVector,
           kBuildPolicy,
           kSimplificationPolicy,
           kStandardStrictnessPolicy,
           kOmitNondrivableLanes}},
      {"LineMultipleSpeeds.xodr",
       builder::RoadGeometryConfiguration{
           maliput::api::RoadGeometryId{"LineMultipleSpeeds"},
           {"odr/LineMultipleSpeeds.xodr"},
           builder::RoadGeometryConfiguration::BuildTolerance{
               5e-2 /* linear_tolerance */, 5e-1 /*max_linear_tolerance*/, 1e-3 /* angular_tolerance */},
           constants::kScaleLength,
           kZeroVector,
           kBuildPolicy,
           kSimplificationPolicy,
           kStandardStrictnessPolicy,
           kOmitNondrivableLanes}},
      {"LineVariableOffset.xodr",
       builder::RoadGeometryConfiguration{
           maliput::api::RoadGeometryId{"LineVariableOffset"},
           {"odr/LineVariableOffset.xodr"},
           builder::RoadGeometryConfiguration::BuildTolerance{
               5e-2 /* linear_tolerance */, 5e-1 /*max_linear_tolerance*/, 1e-3 /* angular_tolerance */},
           constants::kScaleLength,
           kZeroVector,
           kBuildPolicy,
           kSimplificationPolicy,
           kStandardStrictnessPolicy,
           kOmitNondrivableLanes}},
      {"LineVariableWidth.xodr",
       builder::RoadGeometryConfiguration{
           maliput::api::RoadGeometryId{"LineVariableWidth"},
           {"odr/LineVariableWidth.xodr"},
           builder::RoadGeometryConfiguration::BuildTolerance{
               5e-2 /* linear_tolerance */, 5e-1 /*max_linear_tolerance*/, 1e-3 /* angular_tolerance */},
           constants::kScaleLength,
           kZeroVector,
           kBuildPolicy,
           kSimplificationPolicy,
           kStandardStrictnessPolicy,
           kOmitNondrivableLanes}},
      {"ParkingGarageRamp.xodr",
       builder::RoadGeometryConfiguration{
           maliput::api::RoadGeometryId{"ParkingGarageRamp"},
           {"odr/ParkingGarageRamp.xodr"},
           builder::RoadGeometryConfiguration::BuildTolerance{
               5e-2 /* linear_tolerance */, 5e-1 /*max_linear_tolerance*/, 1e-3 /* angular_tolerance */},
           constants::kScaleLength,
           kZeroVector,
           kBuildPolicy,
           kSimplificationPolicy,
           kStandardStrictnessPolicy,
           kOmitNondrivableLanes}},
      {"RRLongRoad.xodr",
       builder::RoadGeometryConfiguration{
           maliput::api::RoadGeometryId{"RRLongRoad"},
           {"odr/RRLongRoad.xodr"},
           builder::RoadGeometryConfiguration::BuildTolerance{
               5e-2 /* linear_tolerance */, 5e-1 /*max_linear_tolerance*/, 1e-3 /* angular_tolerance */},
           constants::kScaleLength,
           kZeroVector,
           kBuildPolicy,
           kSimplificationPolicy,
           kStandardStrictnessPolicy,
           kOmitNondrivableLanes}},
      {"SShapeSuperelevatedRoad.xodr",
       builder::RoadGeometryConfiguration{
           maliput::api::RoadGeometryId{"SShapeSuperelevatedRoad"},
           {"odr/SShapeSuperelevatedRoad.xodr"},
           builder::RoadGeometryConfiguration::BuildTolerance{
               5e-2 /* linear_tolerance */, 5e-1 /*max_linear_tolerance*/, 1e-3 /* angular_tolerance */},
           constants::kScaleLength,
           kZeroVector,
           kBuildPolicy,
           kSimplificationPolicy,
           kStandardStrictnessPolicy,
           kOmitNondrivableLanes}},
      {"TShapeRoad.xodr",
       builder::RoadGeometryConfiguration{
           maliput::api::RoadGeometryId{"TShapeRoad"},
           {"odr/TShapeRoad.xodr"},
           builder::RoadGeometryConfiguration::BuildTolerance{
               5e-2 /* linear_tolerance */, 5e-1 /*max_linear_tolerance*/, 1e-3 /* angular_tolerance */},
           constants::kScaleLength,
           kZeroVector,
           kBuildPolicy,
           kSimplificationPolicy,
           kStandardStrictnessPolicy,
           kOmitNondrivableLanes}},
      {"Highway.xodr",
       builder::RoadGeometryConfiguration{
           maliput::api::RoadGeometryId{"Highway"},
           {"odr/Highway.xodr"},
           builder::RoadGeometryConfiguration::BuildTolerance{
               5e-2 /* linear_tolerance */, 5e-1 /*max_linear_tolerance*/, 1e-3 /* angular_tolerance */},
           constants::kScaleLength,
           kZeroVector,
           kBuildPolicy,
           kSimplificationPolicy,
           kStandardStrictnessPolicy,
           kOmitNondrivableLanes}},
      {"Figure8.xodr",
       builder::RoadGeometryConfiguration{
           maliput::api::RoadGeometryId{"Figure8"},
           {"odr/Figure8.xodr"},
           builder::RoadGeometryConfiguration::BuildTolerance{
               1e-3 /* linear_tolerance */, 1e-2 /*max_linear_tolerance*/, 1e-3 /* angular_tolerance */},
           constants::kScaleLength,
           kZeroVector,
           kBuildPolicy,
           kSimplificationPolicy,
           kStandardStrictnessPolicy,
           kOmitNondrivableLanes}},
      {"RRFigure8.xodr",
       builder::RoadGeometryConfiguration{maliput::api::RoadGeometryId{"RRFigure8"},
                                          {"odr/RRFigure8.xodr"},
                                          builder::RoadGeometryConfiguration::BuildTolerance{
                                              5e-1 /* linear_tolerance */, 1e-3 /* angular_tolerance */},
                                          constants::kScaleLength,
                                          kZeroVector,
                                          kBuildPolicy,
                                          kSimplificationPolicy,
                                          kStandardStrictnessPolicy,
                                          kOmitNondrivableLanes}},
      {"StraightForward.xodr",
       builder::RoadGeometryConfiguration{
           maliput::api::RoadGeometryId{"StraightForward"},
           {"odr/StraightForward.xodr"},
           builder::RoadGeometryConfiguration::BuildTolerance{
               1e-3 /* linear_tolerance */, 1e-2 /*max_linear_tolerance*/, 1e-3 /* angular_tolerance */},
           constants::kScaleLength,
           kZeroVector,
           kBuildPolicy,
           kSimplificationPolicy,
           kStandardStrictnessPolicy,
           kOmitNondrivableLanes}},
      {"SingleRoadComplexDescription.xodr",
       builder::RoadGeometryConfiguration{
           maliput::api::RoadGeometryId{"SingleRoadComplexDescription"},
           {"odr/SingleRoadComplexDescription.xodr"},
           builder::RoadGeometryConfiguration::BuildTolerance{
               5e-2 /* linear_tolerance */, 5e-1 /*max_linear_tolerance*/, 1e-3 /* angular_tolerance */},
           constants::kScaleLength,
           kZeroVector,
           kBuildPolicy,
           kSimplificationPolicy,
           kStandardStrictnessPolicy,
           kOmitNondrivableLanes}},
      {"SingleRoadComplexDescription2.xodr",
       builder::RoadGeometryConfiguration{
           maliput::api::RoadGeometryId{"SingleRoadComplexDescription2"},
           {"odr/SingleRoadComplexDescription2.xodr"},
           builder::RoadGeometryConfiguration::BuildTolerance{
               5e-2 /* linear_tolerance */, 5e-1 /*max_linear_tolerance*/, 1e-3 /* angular_tolerance */},
           constants::kScaleLength,
           kZeroVector,
           kBuildPolicy,
           kSimplificationPolicy,
           kStandardStrictnessPolicy,
           kOmitNondrivableLanes}},
      {"SingleRoadNanValues.xodr",
       builder::RoadGeometryConfiguration{
           maliput::api::RoadGeometryId{"SingleRoadNanValues"},
           {"odr/SingleRoadNanValues.xodr"},
           builder::RoadGeometryConfiguration::BuildTolerance{
               5e-2 /* linear_tolerance */, 5e-1 /*max_linear_tolerance*/, 1e-3 /* angular_tolerance */},
           constants::kScaleLength,
           kZeroVector,
           kBuildPolicy,
           kSimplificationPolicy,
           kStandardStrictnessPolicy,
           kOmitNondrivableLanes}},
      {"SingleRoadNegativeWidth.xodr",
       builder::RoadGeometryConfiguration{
           maliput::api::RoadGeometryId{"SingleRoadNegativeWidth"},
           {"odr/SingleRoadNegativeWidth.xodr"},
           builder::RoadGeometryConfiguration::BuildTolerance{
               5e-2 /* linear_tolerance */, 5e-1 /*max_linear_tolerance*/, 1e-3 /* angular_tolerance */},
           constants::kScaleLength,
           kZeroVector,
           kBuildPolicy,
           kSimplificationPolicy,
           kStandardStrictnessPolicy,
           kOmitNondrivableLanes}},
      {"SingleRoadHighCoefficients.xodr",
       builder::RoadGeometryConfiguration{
           maliput::api::RoadGeometryId{"SingleRoadHighCoefficients"},
           {"odr/SingleRoadHighCoefficients.xodr"},
           builder::RoadGeometryConfiguration::BuildTolerance{
               5e-2 /* linear_tolerance */, 5e-1 /*max_linear_tolerance*/, 1e-3 /* angular_tolerance */},
           constants::kScaleLength,
           kZeroVector,
           kBuildPolicy,
           kSimplificationPolicy,
           kStandardStrictnessPolicy,
           kOmitNondrivableLanes}},
      {"SingleRoadTinyGeometry.xodr",
       builder::RoadGeometryConfiguration{
           maliput::api::RoadGeometryId{"SingleRoadTinyGeometry"},
           {"odr/SingleRoadTinyGeometry.xodr"},
           builder::RoadGeometryConfiguration::BuildTolerance{
               5e-2 /* linear_tolerance */, 5e-1 /*max_linear_tolerance*/, 1e-3 /* angular_tolerance */},
           constants::kScaleLength,
           kZeroVector,
           kBuildPolicy,
           kSimplificationPolicy,
           kStandardStrictnessPolicy,
           kOmitNondrivableLanes}},
      {"SingleRoadTwoGeometries.xodr",
       builder::RoadGeometryConfiguration{
           maliput::api::RoadGeometryId{"SingleRoadTwoGeometries"},
           {"odr/SingleRoadTwoGeometries.xodr"},
           builder::RoadGeometryConfiguration::BuildTolerance{
               5e-2 /* linear_tolerance */, 5e-1 /*max_linear_tolerance*/, 1e-3 /* angular_tolerance */},
           constants::kScaleLength,
           kZeroVector,
           kBuildPolicy,
           kSimplificationPolicy,
           kStandardStrictnessPolicy,
           kOmitNondrivableLanes}},
      {"FlatTown01.xodr",
       builder::RoadGeometryConfiguration{
           maliput::api::RoadGeometryId{"FlatTown01"},
           {"odr/FlatTown01.xodr"},
           builder::RoadGeometryConfiguration::BuildTolerance{
               5e-2 /* linear_tolerance */, 5e-1 /*max_linear_tolerance*/, 1e-3 /* angular_tolerance */},
           constants::kScaleLength,
           kZeroVector,
           kBuildPolicy,
           kSimplificationPolicy,
           kStandardStrictnessPolicy,
           kOmitNondrivableLanes}},
      {"Town01.xodr", builder::RoadGeometryConfiguration{maliput::api::RoadGeometryId{"Town01"},
                                                         {"odr/Town01.xodr"},
                                                         builder::RoadGeometryConfiguration::BuildTolerance{
                                                             5e-2 /* linear_tolerance */, 5e-1 /*max_linear_tolerance*/,
                                                             1e-3 /* angular_tolerance */},
                                                         constants::kScaleLength,
                                                         kZeroVector,
                                                         kBuildPolicy,
                                                         kSimplificationPolicy,
                                                         kStandardStrictnessPolicy,
                                                         kOmitNondrivableLanes}},
      {"Town02.xodr", builder::RoadGeometryConfiguration{maliput::api::RoadGeometryId{"Town02"},
                                                         {"odr/Town02.xodr"},
                                                         builder::RoadGeometryConfiguration::BuildTolerance{
                                                             5e-2 /* linear_tolerance */, 5e-1 /*max_linear_tolerance*/,
                                                             1e-3 /* angular_tolerance */},
                                                         constants::kScaleLength,
                                                         kZeroVector,
                                                         kBuildPolicy,
                                                         kSimplificationPolicy,
                                                         kStandardStrictnessPolicy,
                                                         kOmitNondrivableLanes}},
      {"Town03.xodr", builder::RoadGeometryConfiguration{maliput::api::RoadGeometryId{"Town03"},
                                                         {"odr/Town03.xodr"},
                                                         builder::RoadGeometryConfiguration::BuildTolerance{
                                                             5e-2 /* linear_tolerance */, 5e-1 /*max_linear_tolerance*/,
                                                             1e-3 /* angular_tolerance */},
                                                         constants::kScaleLength,
                                                         kZeroVector,
                                                         kBuildPolicy,
                                                         kSimplificationPolicy,
                                                         kStandardStrictnessPolicy,
                                                         kOmitNondrivableLanes}},
      {"Town04.xodr",
       /* linear tolerance restricted by 0.052m elevation gap in Road 735 */
       builder::RoadGeometryConfiguration{
           maliput::api::RoadGeometryId{"Town04"},
           {"odr/Town04.xodr"},
           builder::RoadGeometryConfiguration::BuildTolerance{
               6e-2 /* linear_tolerance */, 5e-1 /*max_linear_tolerance*/, 1e-3 /* angular_tolerance */},
           constants::kScaleLength,
           kZeroVector,
           kBuildPolicy,
           kSimplificationPolicy,
           kStandardStrictnessPolicy,
           kOmitNondrivableLanes}},
      {"Town05.xodr", builder::RoadGeometryConfiguration{maliput::api::RoadGeometryId{"Town05"},
                                                         {"odr/Town05.xodr"},
                                                         builder::RoadGeometryConfiguration::BuildTolerance{
                                                             5e-2 /* linear_tolerance */, 5e-1 /*max_linear_tolerance*/,
                                                             1e-3 /* angular_tolerance */},
                                                         constants::kScaleLength,
                                                         kZeroVector,
                                                         kBuildPolicy,
                                                         kSimplificationPolicy,
                                                         kStandardStrictnessPolicy,
                                                         kOmitNondrivableLanes}},
      {"Town06.xodr", builder::RoadGeometryConfiguration{maliput::api::RoadGeometryId{"Town06"},
                                                         {"odr/Town06.xodr"},
                                                         builder::RoadGeometryConfiguration::BuildTolerance{
                                                             5e-2 /* linear_tolerance */, 5e-1 /*max_linear_tolerance*/,
                                                             1e-3 /* angular_tolerance */},
                                                         constants::kScaleLength,
                                                         kZeroVector,
                                                         kBuildPolicy,
                                                         kSimplificationPolicy,
                                                         kStandardStrictnessPolicy,
                                                         kOmitNondrivableLanes}},
      {"Town07.xodr", builder::RoadGeometryConfiguration{maliput::api::RoadGeometryId{"Town07"},
                                                         {"odr/Town07.xodr"},
                                                         builder::RoadGeometryConfiguration::BuildTolerance{
                                                             5e-2 /* linear_tolerance */, 5e-1 /*max_linear_tolerance*/,
                                                             1e-3 /* angular_tolerance */},
                                                         constants::kScaleLength,
                                                         kZeroVector,
                                                         kBuildPolicy,
                                                         kSimplificationPolicy,
                                                         kStandardStrictnessPolicy,
                                                         kOmitNondrivableLanes}},
      {"GapInElevationNonDrivableRoad.xodr",
       builder::RoadGeometryConfiguration{maliput::api::RoadGeometryId{"GapInElevationNonDrivableRoad"},
                                          {"odr/GapInElevationNonDrivableRoad.xodr"},
                                          builder::RoadGeometryConfiguration::BuildTolerance{
                                              5e-2 /* linear_tolerance */, 1e-3 /* angular_tolerance */},
                                          constants::kScaleLength,
                                          kZeroVector,
                                          kBuildPolicy,
                                          kSimplificationPolicy,
                                          kStandardStrictnessPolicy,
                                          kOmitNondrivableLanes}},
      {"GapInSuperelevationNonDrivableRoad.xodr",
       builder::RoadGeometryConfiguration{maliput::api::RoadGeometryId{"GapInSuperelevationNonDrivableRoad"},
                                          {"odr/GapInSuperelevationNonDrivableRoad.xodr"},
                                          builder::RoadGeometryConfiguration::BuildTolerance{
                                              5e-2 /* linear_tolerance */, 1e-3 /* angular_tolerance */},
                                          constants::kScaleLength,
                                          kZeroVector,
                                          kBuildPolicy,
                                          kSimplificationPolicy,
                                          kStandardStrictnessPolicy,
                                          kOmitNondrivableLanes}},
      {"GapInLaneWidthNonDrivableLane.xodr",
       builder::RoadGeometryConfiguration{maliput::api::RoadGeometryId{"GapInLaneWidthNonDrivableLane"},
                                          {"odr/GapInLaneWidthNonDrivableLane.xodr"},
                                          builder::RoadGeometryConfiguration::BuildTolerance{
                                              5e-2 /* linear_tolerance */, 1e-3 /* angular_tolerance */},
                                          constants::kScaleLength,
                                          kZeroVector,
                                          kBuildPolicy,
                                          kSimplificationPolicy,
                                          kStandardStrictnessPolicy,
                                          kOmitNondrivableLanes}},
  };
  return kXodrConfigurations.find(xodr_file_name) != kXodrConfigurations.end()
             ? std::make_optional<builder::RoadGeometryConfiguration>(kXodrConfigurations.at(xodr_file_name))
             : std::nullopt;
}

}  // namespace test
}  // namespace malidrive

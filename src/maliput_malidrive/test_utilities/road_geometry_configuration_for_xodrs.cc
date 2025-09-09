// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2021-2022, Toyota Research Institute. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
  const double kIntegratorAccuracyMultiplier{1.0};
  const bool kUseUserDataTrafficDirection{true};

  const static std::unordered_map<std::string, builder::RoadGeometryConfiguration> kXodrConfigurations{
      {"SingleLane.xodr",
       builder::RoadGeometryConfiguration{
           maliput::api::RoadGeometryId{"LineSingleLane"},
           {"SingleLane.xodr"},
           builder::RoadGeometryConfiguration::BuildTolerance{
               5e-2 /* linear_tolerance */, 5e-2 /*max_linear_tolerance*/, 1e-3 /* angular_tolerance */},
           constants::kScaleLength,
           kZeroVector,
           kBuildPolicy,
           kSimplificationPolicy,
           kStandardStrictnessPolicy,
           kOmitNondrivableLanes,
           kIntegratorAccuracyMultiplier,
           kUseUserDataTrafficDirection}},
      {"ArcLane.xodr",
       builder::RoadGeometryConfiguration{
           maliput::api::RoadGeometryId{"ArcSingleLane"},
           {"ArcLane.xodr"},
           builder::RoadGeometryConfiguration::BuildTolerance{
               5e-2 /* linear_tolerance */, 5e-2 /*max_linear_tolerance*/, 1e-3 /* angular_tolerance */},
           constants::kScaleLength,
           kZeroVector,
           kBuildPolicy,
           kSimplificationPolicy,
           kStandardStrictnessPolicy,
           kOmitNondrivableLanes,
           kIntegratorAccuracyMultiplier,
           kUseUserDataTrafficDirection}},
      {"SpiralRoad.xodr",
       builder::RoadGeometryConfiguration{
           maliput::api::RoadGeometryId{"SpiralRoad"},
           {"SpiralRoad.xodr"},
           builder::RoadGeometryConfiguration::BuildTolerance{
               1e-3 /* linear_tolerance */, 5e-2 /*max_linear_tolerance*/, 1e-3 /* angular_tolerance */},
           constants::kScaleLength,
           kZeroVector,
           kBuildPolicy,
           kSimplificationPolicy,
           kStandardStrictnessPolicy,
           kOmitNondrivableLanes,
           kIntegratorAccuracyMultiplier,
           kUseUserDataTrafficDirection}},
      {"SmallTownRoads.xodr",
       builder::RoadGeometryConfiguration{
           maliput::api::RoadGeometryId{"SmallTownRoads"},
           {"SmallTownRoads.xodr"},
           builder::RoadGeometryConfiguration::BuildTolerance{
               1e-2 /* linear_tolerance */, 1e-2 /*max_linear_tolerance*/, 1e-3 /* angular_tolerance */},
           constants::kScaleLength,
           kZeroVector,
           kBuildPolicy,
           kSimplificationPolicy,
           kStandardStrictnessPolicy,
           kOmitNondrivableLanes,
           kIntegratorAccuracyMultiplier,
           kUseUserDataTrafficDirection}},
      {"BikingLineLane.xodr",
       builder::RoadGeometryConfiguration{
           maliput::api::RoadGeometryId{"BikingLineLane"},
           {"BikingLineLane.xodr"},
           builder::RoadGeometryConfiguration::BuildTolerance{
               5e-2 /* linear_tolerance */, 5e-2 /*max_linear_tolerance*/, 1e-3 /* angular_tolerance */},
           constants::kScaleLength,
           kZeroVector,
           kBuildPolicy,
           kSimplificationPolicy,
           kStandardStrictnessPolicy,
           kOmitNondrivableLanes,
           kIntegratorAccuracyMultiplier,
           kUseUserDataTrafficDirection}},
      {"DisconnectedRoadInJunction.xodr",
       builder::RoadGeometryConfiguration{
           maliput::api::RoadGeometryId{"DisconnectedRoadInJunction"},
           {"DisconnectedRoadInJunction.xodr"},
           builder::RoadGeometryConfiguration::BuildTolerance{
               5e-2 /* linear_tolerance */, 5e-2 /*max_linear_tolerance*/, 1e-3 /* angular_tolerance */},
           constants::kScaleLength,
           kZeroVector,
           kBuildPolicy,
           kSimplificationPolicy,
           kStandardStrictnessPolicy,
           kOmitNondrivableLanes,
           kIntegratorAccuracyMultiplier,
           kUseUserDataTrafficDirection}},
      {"SShapeRoad.xodr",
       builder::RoadGeometryConfiguration{
           maliput::api::RoadGeometryId{"SShapeRoad"},
           {"SShapeRoad.xodr"},
           builder::RoadGeometryConfiguration::BuildTolerance{
               5e-2 /* linear_tolerance */, 5e-2 /*max_linear_tolerance*/, 1e-3 /* angular_tolerance */},
           constants::kScaleLength,
           kZeroVector,
           kBuildPolicy,
           kSimplificationPolicy,
           kStandardStrictnessPolicy,
           kOmitNondrivableLanes,
           kIntegratorAccuracyMultiplier,
           kUseUserDataTrafficDirection}},
      {"LShapeRoad.xodr",
       builder::RoadGeometryConfiguration{
           maliput::api::RoadGeometryId{"LShapeRoad"},
           {"LShapeRoad.xodr"},
           builder::RoadGeometryConfiguration::BuildTolerance{
               5e-2 /* linear_tolerance */, 5e-2 /*max_linear_tolerance*/, 1e-3 /* angular_tolerance */},
           constants::kScaleLength,
           kZeroVector,
           kBuildPolicy,
           kSimplificationPolicy,
           kStandardStrictnessPolicy,
           kOmitNondrivableLanes,
           kIntegratorAccuracyMultiplier,
           kUseUserDataTrafficDirection}},
      {"LShapeRoadVariableLanes.xodr",
       builder::RoadGeometryConfiguration{
           maliput::api::RoadGeometryId{"LShapeRoadVariableLanes"},
           {"LShapeRoadVariableLanes.xodr"},
           builder::RoadGeometryConfiguration::BuildTolerance{
               5e-2 /* linear_tolerance */, 5e-2 /*max_linear_tolerance*/, 1e-3 /* angular_tolerance */},
           constants::kScaleLength,
           kZeroVector,
           kBuildPolicy,
           kSimplificationPolicy,
           kStandardStrictnessPolicy,
           kOmitNondrivableLanes,
           kIntegratorAccuracyMultiplier,
           kUseUserDataTrafficDirection}},
      {"LineMultipleSections.xodr",
       builder::RoadGeometryConfiguration{
           maliput::api::RoadGeometryId{"LineMultipleSections"},
           {"LineMultipleSections.xodr"},
           builder::RoadGeometryConfiguration::BuildTolerance{
               5e-2 /* linear_tolerance */, 5e-2 /*max_linear_tolerance*/, 1e-3 /* angular_tolerance */},
           constants::kScaleLength,
           kZeroVector,
           kBuildPolicy,
           kSimplificationPolicy,
           kStandardStrictnessPolicy,
           kOmitNondrivableLanes,
           kIntegratorAccuracyMultiplier,
           kUseUserDataTrafficDirection}},
      {"LineMultipleSectionsMoreCases.xodr",
       builder::RoadGeometryConfiguration{
           maliput::api::RoadGeometryId{"LineMultipleSectionsMoreCases"},
           {"LineMultipleSectionsMoreCases.xodr"},
           builder::RoadGeometryConfiguration::BuildTolerance{
               5e-2 /* linear_tolerance */, 5e-2 /*max_linear_tolerance*/, 1e-3 /* angular_tolerance */},
           constants::kScaleLength,
           kZeroVector,
           kBuildPolicy,
           kSimplificationPolicy,
           kStandardStrictnessPolicy,
           kOmitNondrivableLanes,
           kIntegratorAccuracyMultiplier,
           kUseUserDataTrafficDirection}},
      {"LineMultipleSpeeds.xodr",
       builder::RoadGeometryConfiguration{
           maliput::api::RoadGeometryId{"LineMultipleSpeeds"},
           {"LineMultipleSpeeds.xodr"},
           builder::RoadGeometryConfiguration::BuildTolerance{
               5e-2 /* linear_tolerance */, 5e-2 /*max_linear_tolerance*/, 1e-3 /* angular_tolerance */},
           constants::kScaleLength,
           kZeroVector,
           kBuildPolicy,
           kSimplificationPolicy,
           kStandardStrictnessPolicy,
           kOmitNondrivableLanes,
           kIntegratorAccuracyMultiplier,
           kUseUserDataTrafficDirection}},
      {"LineVariableOffset.xodr",
       builder::RoadGeometryConfiguration{
           maliput::api::RoadGeometryId{"LineVariableOffset"},
           {"LineVariableOffset.xodr"},
           builder::RoadGeometryConfiguration::BuildTolerance{
               5e-2 /* linear_tolerance */, 5e-2 /*max_linear_tolerance*/, 1e-3 /* angular_tolerance */},
           constants::kScaleLength,
           kZeroVector,
           kBuildPolicy,
           kSimplificationPolicy,
           kStandardStrictnessPolicy,
           kOmitNondrivableLanes,
           kIntegratorAccuracyMultiplier,
           kUseUserDataTrafficDirection}},
      {"LineVariableWidth.xodr",
       builder::RoadGeometryConfiguration{
           maliput::api::RoadGeometryId{"LineVariableWidth"},
           {"LineVariableWidth.xodr"},
           builder::RoadGeometryConfiguration::BuildTolerance{
               5e-2 /* linear_tolerance */, 5e-2 /*max_linear_tolerance*/, 1e-3 /* angular_tolerance */},
           constants::kScaleLength,
           kZeroVector,
           kBuildPolicy,
           kSimplificationPolicy,
           kStandardStrictnessPolicy,
           kOmitNondrivableLanes,
           kIntegratorAccuracyMultiplier,
           kUseUserDataTrafficDirection}},
      {"LongRoad.xodr",
       builder::RoadGeometryConfiguration{
           maliput::api::RoadGeometryId{"LongRoad"},
           {"LongRoad.xodr"},
           builder::RoadGeometryConfiguration::BuildTolerance{
               5e-2 /* linear_tolerance */, 5e-2 /*max_linear_tolerance*/, 1e-3 /* angular_tolerance */},
           constants::kScaleLength,
           kZeroVector,
           kBuildPolicy,
           kSimplificationPolicy,
           kStandardStrictnessPolicy,
           kOmitNondrivableLanes,
           kIntegratorAccuracyMultiplier,
           kUseUserDataTrafficDirection}},
      {"ParkingGarageRamp.xodr",
       builder::RoadGeometryConfiguration{
           maliput::api::RoadGeometryId{"ParkingGarageRamp"},
           {"ParkingGarageRamp.xodr"},
           builder::RoadGeometryConfiguration::BuildTolerance{
               5e-2 /* linear_tolerance */, 5e-2 /*max_linear_tolerance*/, 1e-3 /* angular_tolerance */},
           constants::kScaleLength,
           kZeroVector,
           kBuildPolicy,
           kSimplificationPolicy,
           kStandardStrictnessPolicy,
           kOmitNondrivableLanes,
           kIntegratorAccuracyMultiplier,
           kUseUserDataTrafficDirection}},
      {"RRLongRoad.xodr",
       builder::RoadGeometryConfiguration{
           maliput::api::RoadGeometryId{"RRLongRoad"},
           {"RRLongRoad.xodr"},
           builder::RoadGeometryConfiguration::BuildTolerance{
               5e-2 /* linear_tolerance */, 5e-2 /*max_linear_tolerance*/, 1e-3 /* angular_tolerance */},
           constants::kScaleLength,
           kZeroVector,
           kBuildPolicy,
           kSimplificationPolicy,
           kStandardStrictnessPolicy,
           kOmitNondrivableLanes,
           kIntegratorAccuracyMultiplier,
           kUseUserDataTrafficDirection}},
      {"SShapeSuperelevatedRoad.xodr",
       builder::RoadGeometryConfiguration{
           maliput::api::RoadGeometryId{"SShapeSuperelevatedRoad"},
           {"SShapeSuperelevatedRoad.xodr"},
           builder::RoadGeometryConfiguration::BuildTolerance{
               5e-2 /* linear_tolerance */, 5e-2 /*max_linear_tolerance*/, 1e-3 /* angular_tolerance */},
           constants::kScaleLength,
           kZeroVector,
           kBuildPolicy,
           kSimplificationPolicy,
           kStandardStrictnessPolicy,
           kOmitNondrivableLanes,
           kIntegratorAccuracyMultiplier,
           kUseUserDataTrafficDirection}},
      {"TShapeRoad.xodr",
       builder::RoadGeometryConfiguration{
           maliput::api::RoadGeometryId{"TShapeRoad"},
           {"TShapeRoad.xodr"},
           builder::RoadGeometryConfiguration::BuildTolerance{
               5e-2 /* linear_tolerance */, 5e-2 /*max_linear_tolerance*/, 1e-3 /* angular_tolerance */},
           constants::kScaleLength,
           kZeroVector,
           kBuildPolicy,
           kSimplificationPolicy,
           kStandardStrictnessPolicy,
           kOmitNondrivableLanes,
           kIntegratorAccuracyMultiplier,
           kUseUserDataTrafficDirection}},
      {"Highway.xodr",
       builder::RoadGeometryConfiguration{
           maliput::api::RoadGeometryId{"Highway"},
           {"Highway.xodr"},
           builder::RoadGeometryConfiguration::BuildTolerance{
               5e-2 /* linear_tolerance */, 5e-2 /*max_linear_tolerance*/, 1e-3 /* angular_tolerance */},
           constants::kScaleLength,
           kZeroVector,
           kBuildPolicy,
           kSimplificationPolicy,
           kStandardStrictnessPolicy,
           kOmitNondrivableLanes,
           kIntegratorAccuracyMultiplier,
           kUseUserDataTrafficDirection}},
      {"Figure8.xodr",
       builder::RoadGeometryConfiguration{
           maliput::api::RoadGeometryId{"Figure8"},
           {"Figure8.xodr"},
           builder::RoadGeometryConfiguration::BuildTolerance{
               1e-3 /* linear_tolerance */, 1e-3 /*max_linear_tolerance*/, 1e-3 /* angular_tolerance */},
           constants::kScaleLength,
           kZeroVector,
           kBuildPolicy,
           kSimplificationPolicy,
           kStandardStrictnessPolicy,
           kOmitNondrivableLanes,
           kIntegratorAccuracyMultiplier,
           kUseUserDataTrafficDirection}},
      {"RRFigure8.xodr",
       builder::RoadGeometryConfiguration{
           maliput::api::RoadGeometryId{"RRFigure8"},
           {"RRFigure8.xodr"},
           builder::RoadGeometryConfiguration::BuildTolerance{
               5e-1 /* linear_tolerance */, 5e-1 /*max_linear_tolerance*/, 1e-3 /* angular_tolerance */},
           constants::kScaleLength,
           kZeroVector,
           kBuildPolicy,
           kSimplificationPolicy,
           kStandardStrictnessPolicy,
           kOmitNondrivableLanes,
           kIntegratorAccuracyMultiplier,
           kUseUserDataTrafficDirection}},
      {"StraightForward.xodr",
       builder::RoadGeometryConfiguration{
           maliput::api::RoadGeometryId{"StraightForward"},
           {"StraightForward.xodr"},
           builder::RoadGeometryConfiguration::BuildTolerance{
               1e-3 /* linear_tolerance */, 1e-3 /*max_linear_tolerance*/, 1e-3 /* angular_tolerance */},
           constants::kScaleLength,
           kZeroVector,
           kBuildPolicy,
           kSimplificationPolicy,
           kStandardStrictnessPolicy,
           kOmitNondrivableLanes,
           kIntegratorAccuracyMultiplier,
           kUseUserDataTrafficDirection}},
      {"StraightRoadMultipleLaneDirections.xodr",
       builder::RoadGeometryConfiguration{
           maliput::api::RoadGeometryId{"StraightRoadMultipleLaneDirections"},
           {"StraightRoadMultipleLaneDirections.xodr"},
           builder::RoadGeometryConfiguration::BuildTolerance{
               1e-3 /* linear_tolerance */, 1e-3 /*max_linear_tolerance*/, 1e-3 /* angular_tolerance */},
           constants::kScaleLength,
           kZeroVector,
           kBuildPolicy,
           kSimplificationPolicy,
           kStandardStrictnessPolicy,
           kOmitNondrivableLanes,
           kIntegratorAccuracyMultiplier,
           kUseUserDataTrafficDirection}},
      {"SingleRoadComplexDescription.xodr",
       builder::RoadGeometryConfiguration{
           maliput::api::RoadGeometryId{"SingleRoadComplexDescription"},
           {"SingleRoadComplexDescription.xodr"},
           builder::RoadGeometryConfiguration::BuildTolerance{
               5e-2 /* linear_tolerance */, 5e-2 /*max_linear_tolerance*/, 1e-3 /* angular_tolerance */},
           constants::kScaleLength,
           kZeroVector,
           kBuildPolicy,
           kSimplificationPolicy,
           kStandardStrictnessPolicy,
           kOmitNondrivableLanes,
           kIntegratorAccuracyMultiplier,
           kUseUserDataTrafficDirection}},
      {"SingleRoadComplexDescription2.xodr",
       builder::RoadGeometryConfiguration{
           maliput::api::RoadGeometryId{"SingleRoadComplexDescription2"},
           {"SingleRoadComplexDescription2.xodr"},
           builder::RoadGeometryConfiguration::BuildTolerance{
               5e-2 /* linear_tolerance */, 5e-2 /*max_linear_tolerance*/, 1e-3 /* angular_tolerance */},
           constants::kScaleLength,
           kZeroVector,
           kBuildPolicy,
           kSimplificationPolicy,
           kStandardStrictnessPolicy,
           kOmitNondrivableLanes,
           kIntegratorAccuracyMultiplier,
           kUseUserDataTrafficDirection}},
      {"SingleRoadNanValues.xodr",
       builder::RoadGeometryConfiguration{
           maliput::api::RoadGeometryId{"SingleRoadNanValues"},
           {"SingleRoadNanValues.xodr"},
           builder::RoadGeometryConfiguration::BuildTolerance{
               5e-1 /* linear_tolerance */, 5e-1 /*max_linear_tolerance*/, 1e-3 /* angular_tolerance */},
           constants::kScaleLength,
           kZeroVector,
           kBuildPolicy,
           kSimplificationPolicy,
           kStandardStrictnessPolicy,
           kOmitNondrivableLanes,
           kIntegratorAccuracyMultiplier,
           kUseUserDataTrafficDirection}},
      {"SingleRoadNegativeWidth.xodr",
       builder::RoadGeometryConfiguration{
           maliput::api::RoadGeometryId{"SingleRoadNegativeWidth"},
           {"SingleRoadNegativeWidth.xodr"},
           builder::RoadGeometryConfiguration::BuildTolerance{
               5e-2 /* linear_tolerance */, 5e-2 /*max_linear_tolerance*/, 1e-3 /* angular_tolerance */},
           constants::kScaleLength,
           kZeroVector,
           kBuildPolicy,
           kSimplificationPolicy,
           kStandardStrictnessPolicy,
           kOmitNondrivableLanes,
           kIntegratorAccuracyMultiplier,
           kUseUserDataTrafficDirection}},
      {"SingleRoadHighCoefficients.xodr",
       builder::RoadGeometryConfiguration{
           maliput::api::RoadGeometryId{"SingleRoadHighCoefficients"},
           {"SingleRoadHighCoefficients.xodr"},
           builder::RoadGeometryConfiguration::BuildTolerance{
               5e-2 /* linear_tolerance */, 5e-2 /*max_linear_tolerance*/, 1e-3 /* angular_tolerance */},
           constants::kScaleLength,
           kZeroVector,
           kBuildPolicy,
           kSimplificationPolicy,
           kStandardStrictnessPolicy,
           kOmitNondrivableLanes,
           kIntegratorAccuracyMultiplier,
           kUseUserDataTrafficDirection}},
      {"SingleRoadTinyGeometry.xodr",
       builder::RoadGeometryConfiguration{
           maliput::api::RoadGeometryId{"SingleRoadTinyGeometry"},
           {"SingleRoadTinyGeometry.xodr"},
           builder::RoadGeometryConfiguration::BuildTolerance{
               5e-2 /* linear_tolerance */, 5e-2 /*max_linear_tolerance*/, 1e-3 /* angular_tolerance */},
           constants::kScaleLength,
           kZeroVector,
           kBuildPolicy,
           kSimplificationPolicy,
           kStandardStrictnessPolicy,
           kOmitNondrivableLanes,
           kIntegratorAccuracyMultiplier,
           kUseUserDataTrafficDirection}},
      {"SingleRoadTwoGeometries.xodr",
       builder::RoadGeometryConfiguration{
           maliput::api::RoadGeometryId{"SingleRoadTwoGeometries"},
           {"SingleRoadTwoGeometries.xodr"},
           builder::RoadGeometryConfiguration::BuildTolerance{
               5e-2 /* linear_tolerance */, 5e-2 /*max_linear_tolerance*/, 1e-3 /* angular_tolerance */},
           constants::kScaleLength,
           kZeroVector,
           kBuildPolicy,
           kSimplificationPolicy,
           kStandardStrictnessPolicy,
           kOmitNondrivableLanes,
           kIntegratorAccuracyMultiplier,
           kUseUserDataTrafficDirection}},
      {"FlatTown01.xodr",
       builder::RoadGeometryConfiguration{
           maliput::api::RoadGeometryId{"FlatTown01"},
           {"FlatTown01.xodr"},
           builder::RoadGeometryConfiguration::BuildTolerance{
               5e-2 /* linear_tolerance */, 5e-2 /*max_linear_tolerance*/, 1e-3 /* angular_tolerance */},
           constants::kScaleLength,
           kZeroVector,
           kBuildPolicy,
           kSimplificationPolicy,
           kStandardStrictnessPolicy,
           kOmitNondrivableLanes,
           kIntegratorAccuracyMultiplier,
           kUseUserDataTrafficDirection}},
      {"Town01.xodr", builder::RoadGeometryConfiguration{maliput::api::RoadGeometryId{"Town01"},
                                                         {"Town01.xodr"},
                                                         builder::RoadGeometryConfiguration::BuildTolerance{
                                                             5e-2 /* linear_tolerance */, 5e-2 /*max_linear_tolerance*/,
                                                             1e-3 /* angular_tolerance */},
                                                         constants::kScaleLength,
                                                         kZeroVector,
                                                         kBuildPolicy,
                                                         kSimplificationPolicy,
                                                         kStandardStrictnessPolicy,
                                                         kOmitNondrivableLanes,
                                                         kIntegratorAccuracyMultiplier,
                                                         kUseUserDataTrafficDirection}},
      {"Town02.xodr", builder::RoadGeometryConfiguration{maliput::api::RoadGeometryId{"Town02"},
                                                         {"Town02.xodr"},
                                                         builder::RoadGeometryConfiguration::BuildTolerance{
                                                             5e-2 /* linear_tolerance */, 5e-2 /*max_linear_tolerance*/,
                                                             1e-3 /* angular_tolerance */},
                                                         constants::kScaleLength,
                                                         kZeroVector,
                                                         kBuildPolicy,
                                                         kSimplificationPolicy,
                                                         kStandardStrictnessPolicy,
                                                         kOmitNondrivableLanes,
                                                         kIntegratorAccuracyMultiplier,
                                                         kUseUserDataTrafficDirection}},
      {"Town03.xodr", builder::RoadGeometryConfiguration{maliput::api::RoadGeometryId{"Town03"},
                                                         {"Town03.xodr"},
                                                         builder::RoadGeometryConfiguration::BuildTolerance{
                                                             5e-2 /* linear_tolerance */, 5e-2 /*max_linear_tolerance*/,
                                                             1e-3 /* angular_tolerance */},
                                                         constants::kScaleLength,
                                                         kZeroVector,
                                                         kBuildPolicy,
                                                         kSimplificationPolicy,
                                                         kStandardStrictnessPolicy,
                                                         kOmitNondrivableLanes,
                                                         kIntegratorAccuracyMultiplier,
                                                         kUseUserDataTrafficDirection}},
      {"Town04.xodr",
       /* linear tolerance restricted by 0.052m elevation gap in Road 735 */
       builder::RoadGeometryConfiguration{
           maliput::api::RoadGeometryId{"Town04"},
           {"Town04.xodr"},
           builder::RoadGeometryConfiguration::BuildTolerance{
               6e-2 /* linear_tolerance */, 6e-2 /*max_linear_tolerance*/, 1e-3 /* angular_tolerance */},
           constants::kScaleLength,
           kZeroVector,
           kBuildPolicy,
           kSimplificationPolicy,
           kStandardStrictnessPolicy,
           kOmitNondrivableLanes,
           kIntegratorAccuracyMultiplier,
           kUseUserDataTrafficDirection}},
      {"Town05.xodr", builder::RoadGeometryConfiguration{maliput::api::RoadGeometryId{"Town05"},
                                                         {"Town05.xodr"},
                                                         builder::RoadGeometryConfiguration::BuildTolerance{
                                                             5e-2 /* linear_tolerance */, 5e-2 /*max_linear_tolerance*/,
                                                             1e-3 /* angular_tolerance */},
                                                         constants::kScaleLength,
                                                         kZeroVector,
                                                         kBuildPolicy,
                                                         kSimplificationPolicy,
                                                         kStandardStrictnessPolicy,
                                                         kOmitNondrivableLanes,
                                                         kIntegratorAccuracyMultiplier,
                                                         kUseUserDataTrafficDirection}},
      {"Town06.xodr", builder::RoadGeometryConfiguration{maliput::api::RoadGeometryId{"Town06"},
                                                         {"Town06.xodr"},
                                                         builder::RoadGeometryConfiguration::BuildTolerance{
                                                             5e-2 /* linear_tolerance */, 5e-2 /*max_linear_tolerance*/,
                                                             1e-3 /* angular_tolerance */},
                                                         constants::kScaleLength,
                                                         kZeroVector,
                                                         kBuildPolicy,
                                                         kSimplificationPolicy,
                                                         kStandardStrictnessPolicy,
                                                         kOmitNondrivableLanes,
                                                         kIntegratorAccuracyMultiplier,
                                                         kUseUserDataTrafficDirection}},
      {"Town07.xodr", builder::RoadGeometryConfiguration{maliput::api::RoadGeometryId{"Town07"},
                                                         {"Town07.xodr"},
                                                         builder::RoadGeometryConfiguration::BuildTolerance{
                                                             5e-2 /* linear_tolerance */, 5e-2 /*max_linear_tolerance*/,
                                                             1e-3 /* angular_tolerance */},
                                                         constants::kScaleLength,
                                                         kZeroVector,
                                                         kBuildPolicy,
                                                         kSimplificationPolicy,
                                                         kStandardStrictnessPolicy,
                                                         kOmitNondrivableLanes,
                                                         kIntegratorAccuracyMultiplier,
                                                         kUseUserDataTrafficDirection}},
      {"GapInElevationNonDrivableRoad.xodr",
       builder::RoadGeometryConfiguration{
           maliput::api::RoadGeometryId{"GapInElevationNonDrivableRoad"},
           {"GapInElevationNonDrivableRoad.xodr"},
           builder::RoadGeometryConfiguration::BuildTolerance{
               5e-2 /* linear_tolerance */, 5e-2 /*max_linear_tolerance*/, 1e-3 /* angular_tolerance */},
           constants::kScaleLength,
           kZeroVector,
           kBuildPolicy,
           kSimplificationPolicy,
           kStandardStrictnessPolicy,
           kOmitNondrivableLanes,
           kIntegratorAccuracyMultiplier,
           kUseUserDataTrafficDirection}},
      {"GapInSuperelevationNonDrivableRoad.xodr",
       builder::RoadGeometryConfiguration{
           maliput::api::RoadGeometryId{"GapInSuperelevationNonDrivableRoad"},
           {"GapInSuperelevationNonDrivableRoad.xodr"},
           builder::RoadGeometryConfiguration::BuildTolerance{
               5e-2 /* linear_tolerance */, 5e-2 /*max_linear_tolerance*/, 1e-3 /* angular_tolerance */},
           constants::kScaleLength,
           kZeroVector,
           kBuildPolicy,
           kSimplificationPolicy,
           kStandardStrictnessPolicy,
           kOmitNondrivableLanes,
           kIntegratorAccuracyMultiplier,
           kUseUserDataTrafficDirection}},
      {"GapInLaneWidthNonDrivableLane.xodr",
       builder::RoadGeometryConfiguration{
           maliput::api::RoadGeometryId{"GapInLaneWidthNonDrivableLane"},
           {"GapInLaneWidthNonDrivableLane.xodr"},
           builder::RoadGeometryConfiguration::BuildTolerance{
               5e-2 /* linear_tolerance */, 5e-2 /*max_linear_tolerance*/, 1e-3 /* angular_tolerance */},
           constants::kScaleLength,
           kZeroVector,
           kBuildPolicy,
           kSimplificationPolicy,
           kStandardStrictnessPolicy,
           kOmitNondrivableLanes,
           kIntegratorAccuracyMultiplier,
           kUseUserDataTrafficDirection}},
      {"GapInLaneWidthDrivableLane.xodr",
       builder::RoadGeometryConfiguration{
           maliput::api::RoadGeometryId{"GapInLaneWidthDrivableLane"},
           {"GapInLaneWidthDrivableLane.xodr"},
           builder::RoadGeometryConfiguration::BuildTolerance{
               5e-2 /* linear_tolerance */, 5e-2 /*max_linear_tolerance*/, 1e-3 /* angular_tolerance */},
           constants::kScaleLength,
           kZeroVector,
           kBuildPolicy,
           kSimplificationPolicy,
           kStandardStrictnessPolicy,
           kOmitNondrivableLanes,
           kIntegratorAccuracyMultiplier,
           kUseUserDataTrafficDirection}},
  };
  return kXodrConfigurations.find(xodr_file_name) != kXodrConfigurations.end()
             ? std::make_optional<builder::RoadGeometryConfiguration>(kXodrConfigurations.at(xodr_file_name))
             : std::nullopt;
}

}  // namespace test
}  // namespace malidrive

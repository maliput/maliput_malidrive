// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2020-2022, Toyota Research Institute. All rights reserved.
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
#pragma once

#include <optional>
#include <string>

#include <maliput/api/lane_data.h>
#include <maliput/api/road_geometry.h>

#include "maliput_malidrive/base/lane.h"
#include "maliput_malidrive/builder/rule_tools.h"
#include "maliput_malidrive/common/macros.h"
#include "maliput_malidrive/xodr/junction.h"
#include "maliput_malidrive/xodr/lane.h"
#include "maliput_malidrive/xodr/road_header.h"

namespace malidrive {
namespace builder {

/// Convenient enumeration to identify which type of connection is needed.
enum class XodrConnectionType { kSuccessor = 0, kPredecessor };

/// Holds useful XODR Lane properties.
struct MalidriveXodrLaneProperties {
  MALIDRIVE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MalidriveXodrLaneProperties)
  MalidriveXodrLaneProperties() = delete;

  /// Constructs a MalidriveXodrLaneProperties.
  ///
  /// @param _road_header is the pointer to the XODR Road that holds this lane.
  ///        It must not be nullptr.
  /// @param _lane_section is the pointer to the XODR LaneSection that holds
  ///        this lane. It must not be nullptr.
  /// @param _lane_section_index is index of the LaneSection within the XODR
  ///        road. It must not be negative.
  /// @param _lane is the pointer to the XODR Lane. It must not be nullptr.
  ///
  /// @throws maliput::common::assertion_error When either `road_header`,
  ///        `lane_section` or `lane` are nullptrs, or `lane_section_index`
  ///        is negative.
  MalidriveXodrLaneProperties(const xodr::RoadHeader* _road_header, const xodr::LaneSection* _lane_section,
                              int _lane_section_index, const xodr::Lane* _lane)
      : road_header(_road_header), lane(_lane), lane_section(_lane_section), lane_section_index(_lane_section_index) {
    MALIDRIVE_THROW_UNLESS(road_header != nullptr);
    MALIDRIVE_THROW_UNLESS(lane != nullptr);
    MALIDRIVE_THROW_UNLESS(lane_section != nullptr);
    MALIDRIVE_THROW_UNLESS(lane_section_index >= 0);
  }

  const xodr::RoadHeader* road_header{};
  const xodr::Lane* lane{};
  const xodr::LaneSection* lane_section{};
  int lane_section_index{};
};

/// Searches which LaneEnds connect to `xodr_lane_properties.lane` in `connection_type` direction
/// considering the LaneEnd belongs to an external interface with other XODR Roads, not XODR Junctions.
///
/// @param rg Is the pointer to the RoadGeometry is being built. It must not be
///        nullptr.
/// @param xodr_lane_properties Contains useful XODR Lane Properties.
/// @param road_headers RoadHeaders of the XODR Map.
/// @param connection_type Is the type (successor or predecessor) of link that
///        is solved.
///
/// @throws maliput::common::assertion_error When `rg` is nullptr.
/// @throws maliput::common::assertion_error When there isn't a valid RoadLink.
/// @throws maliput::common::assertion_error When there isn't a valid LaneLink.
std::vector<maliput::api::LaneEnd> SolveLaneEndsForConnectingRoad(
    const maliput::api::RoadGeometry* rg, const MalidriveXodrLaneProperties& xodr_lane_properties,
    const std::map<xodr::RoadHeader::Id, xodr::RoadHeader>& road_headers, XodrConnectionType connection_type);

/// Searches which LaneEnds connect to `xodr_lane_properties.lane` in `connection_type` direction
/// considering the LaneEnd belongs to an external interface with a XODR Junction but the XODR Road does
/// not live in a XODR Junction.
///
/// @param rg Is the pointer to the RoadGeometry is being built. It must not be
///        nullptr.
/// @param xodr_lane_properties Contains useful XODR Lane Properties.
/// @param road_headers RoadHeaders of the XODR Map.
/// @param junctions Junctions of the XODR Map.
/// @param connection_type Is the type (successor or predecessor) of link that
///        is solved.
///
/// @throws maliput::common::assertion_error When either `rg` is nullptr.
/// @throws maliput::common::assertion_error When there isn't a valid RoadLink.
/// @throws maliput::common::assertion_error When the junction where the road is connected to doesn't exist.
std::vector<maliput::api::LaneEnd> SolveLaneEndsForJunction(
    const maliput::api::RoadGeometry* rg, const MalidriveXodrLaneProperties& xodr_lane_properties,
    const std::map<xodr::RoadHeader::Id, xodr::RoadHeader>& road_headers,
    const std::unordered_map<xodr::Junction::Id, xodr::Junction>& junctions, XodrConnectionType connection_type);

/// Searches which LaneEnds connect to `xodr_lane_properties.lane` in `connection_type` direction
/// considering the LaneEnd belongs to an external interface. The XODR Road that contains the LaneEnd is
/// assumed to live within an XODR Junction and it *could* connect to another XODR Road.
///
/// @param rg Is the pointer to the RoadGeometry is being built. It must not be
///        nullptr.
/// @param xodr_lane_properties Contains useful XODR Lane Properties.
/// @param road_headers RoadHeaders of the XODR Map.
/// @param connection_type Is the type (successor or predecessor) of link that
///        is solved.
///
/// @throws maliput::common::assertion_error When either `rg` is nullptr.
/// @throws maliput::common::assertion_error When the RoadLink links to a junction.
std::vector<maliput::api::LaneEnd> SolveLaneEndsWithinJunction(
    const maliput::api::RoadGeometry* rg, const MalidriveXodrLaneProperties& xodr_lane_properties,
    const std::map<xodr::RoadHeader::Id, xodr::RoadHeader>& road_headers, XodrConnectionType connection_type);

/// Searches which LaneEnds connect to `lane_end` considering it
/// belongs to an inner interface.
///
/// @param rg is the pointer to the RoadGeometry is being built. It must not be
///        nullptr.
/// @param lane_end is the LaneEnd to which this method looks for connections.
/// @param xodr_lane_properties Contains useful XODR Lane Properties.
///
/// @throws maliput::common::assertion_error When either `rg` is
///         nullptr.
std::vector<maliput::api::LaneEnd> SolveLaneEndsForInnerLaneSection(
    const maliput::api::RoadGeometry* rg, const maliput::api::LaneEnd& lane_end,
    const MalidriveXodrLaneProperties& xodr_lane_properties);

/// Hold the travel direction of a lane obtained from parsing a userData XML node.
/// An userData node example:
/// @code{.xml}
///  <userData>
///      <vectorLane travelDir="undirected"/>
///  </userData>
/// @endcode
class LaneTravelDirection {
 public:
  MALIDRIVE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(LaneTravelDirection)
  LaneTravelDirection() = delete;

  /// Constructs a LaneTravelDirection.
  /// @param user_data Contains a UserData XMLNode that holds information
  ///                  about the direction of the lane.
  /// @throw maliput::common::assertion_error When the travel direction information is not compatible.
  explicit LaneTravelDirection(const std::optional<std::string>& user_data);

  /// Possible directions obtained from the XODR.
  /// Note: "Undefined" refer to when travel direction information is omitted or node isn't well formed.
  enum class Direction { kUndirected = 0, kForward, kBackward, kBidirectional, kUndefined };

  /// @returns The direction of the lane describend in the userData XML node.
  Direction GetXodrTravelDir() const { return travel_dir_; };

  /// Matches the travel direction with Maliput's standard.
  /// @see maliput::BuildDirectionUsageRuleType().
  /// @returns The direction usage of the lane.
  std::string GetMaliputTravelDir() const;

 private:
  // Matches Direction with a string.
  // @param direction Is a string.
  // @returns A Direction that matches with `direction`.
  // @throw maliput::common::assertion_error When `direction` doesn't match with a Direction.
  Direction str_to_direction(const std::string& direction) const;

  // Holds tags of the userData node.
  static constexpr const char* kVectorLaneTag{"vectorLane"};
  static constexpr const char* kTravelDirTag{"travelDir"};
  static constexpr const char* kUserDataTag{"userData"};
  // Direction of the lane.
  Direction travel_dir_{Direction::kUndefined};
};

/// Determines whether or not an `xodr_lane` is driveable by any vehicle.
/// This includes lanes that OpenDRIVE's Standard marks as driveable for motorized
/// vehicles when the xml attribute in the node is any of the following
/// xodr::Lane::Type:
///
/// - kDriving
/// - kMwyEntry
/// - kMwyExit
/// - kEntry
/// - kExit
/// - kOnRamp
/// - kOffRamp
/// - kConnectingRamp
/// - kRoadWorks
///
/// Note that xodr::Lane::Type::kBiking and xodr::Lane::Type::kSpecial1 are not included
/// but this function also returns true when `xodr_lane`s type matches any of them
/// too. xodr::Lane::Type::kBiking is driveable because non motorized vehicles are
/// allowed to drive through. For a full description why xodr::Lane::Type::kSpecial1
/// is considered, see malidrive#272 and malidrive#273.
///
/// This function considers as non-driveable the following list of
/// xodr::Lane::Type:
///
/// - kNone
/// - kStop
/// - kShoulder
/// - kSidewalk
/// - kBorder
/// - kRestricted
/// - kParking
/// - kSpecial2
/// - kSpecial3
/// - kTram
/// - kRail
/// - kMedian
///
/// @returns True when `xodr_lane` is not the TRACK-lane and it is drivable.
///
/// @param xodr_lane An Xodr Lane that will be evaluated as drivable or non-drivable
/// @throws maliput::common::assertion_error When `xodr_lane`'s type is not a
///         valid type.
bool is_driveable_lane(const xodr::Lane& xodr_lane);

/// Determines whether or not an `xodr_road` contains only non-drivable lanes.
/// #is_driveable_lane method is used to determine the driveability of the lanes in the road.
/// @param xodr_road An Xodr RoadHeader whose lanes will be evaluated as drivable or non-drivable.
///
/// @returns True when `xodr_road` only contains non-drivable lanes.
bool AreOnlyNonDrivableLanes(const xodr::RoadHeader& xodr_road);

/// Determines the vehicle usage rule state value for `xodr_lane` based on its
/// XODR Lane type.
///
/// @param xodr_lane An XODR Lane. It must not be nullptr.
///
/// @returns "NonPedestrians" when `is_driveable_lane(xodr_lane)` returns true
///          and XODR Lane type is not xodr::Lane::Type::kParking. "Unrestricted"
///          when XODR Lane type is xodr::Lane::Type::kParking or xodr::Lane::Type::kNone.
///          Otherwise, it returns "NonVehicles".
///
/// @throws maliput::common::assertion_error When `xodr_lane`'s type is not a
///         valid type.
std::string VehicleUsageValueForXodrLane(const xodr::Lane& xodr_lane);

/// Determines the vehicle exclusive rule state value for `xodr_lane` based on
/// its XODR Lane type.
///
/// @param xodr_lane An XODR Lane. It must not be nullptr.
///
/// @returns "NonMotorizedVehicleOnly" when `xodr_lane`'s type is
///          xodr::Lane::Type::kBiking. "MotorizedVehicleOnly" when `xodr_lane`'s
///          type is xodr::Lane::Type::kMwyEntry or xodr::Lane::Type::kMwyExit.
///          Otherwise, it returns nullopt.
///
/// @throws maliput::common::assertion_error When `xodr_lane`'s type is not a
///         valid type.
std::optional<std::string> VehicleExclusiveValueForXodrLane(const xodr::Lane& xodr_lane);

/// @returns A vector of XodrSpeedProperties which contains the speed limits obtained from
///          the RoadTypes of `xodr_road` for the [s_track_start , s_track_end] range.
///
/// @param xodr_road Is a XODR::RoadHeader.
/// @param s_track_start Track s-coordinate that specifies the start of the range.
/// @param s_track_end Track s-coordinate that specifies the end of the range.
///
/// @throw maliput::common::assertion_error When `s_track_start` is greater than `s_track_end`.
/// @throw maliput::common::assertion_error When `s_track_start` is negative.
std::vector<rules::XodrSpeedProperties> GetRoadTypeSpeedPropertiesInRange(const xodr::RoadHeader& xodr_road,
                                                                          double s_track_start, double s_track_end);

/// @returns A vector of XodrSpeedProperties which contains the speed limits obtained from
///          the Speed records of `xodr_lane`.
///
/// @param xodr_lane Is a xodr::Lane.
/// @param s_track_start Track s-coordinate that specifies the start of the lane.
/// @param s_track_end Track s-coordinate that specifies the end of the lane.
///
/// @throw maliput::common::assertion_error When `s_track_start` is greater than `s_track_end`.
/// @throw maliput::common::assertion_error When `s_track_start` is negative.
std::vector<rules::XodrSpeedProperties> GetLaneSpeedProperties(const xodr::Lane& xodr_lane, double s_track_start,
                                                               double s_track_end);

/// @returns The xodr::RoadHeader that matches with the Road that contains `lane`.
/// @param lane The Lane to retrieve its correspondant xodr::RoadHeader. It must not be nullptr.
///
/// @throws maliput::common::assertion_error When lane is nullptr.
/// @throws maliput::common::assertion_error When the correspondant xodr::RoadHeader cannot be found.
const xodr::RoadHeader& GetXodrRoadFromMalidriveLane(const Lane* lane);

/// @returns The xodr::Lane that matches with `lane`.
/// @param lane The Lane to retrieve its correspondant xodr::Lane. It must not be nullptr.
///
/// @throws maliput::common::assertion_error When lane is nullptr.
/// @throws maliput::common::assertion_error When the correspondant xodr::Lane cannot be found.
const xodr::Lane& GetXodrLaneFromMalidriveLane(const Lane* lane);

/// @returns The direction usage value of `lane`. It is one in the string set
/// `{"AgainstS", "WithS", "Bidirectional", "Undefined"}`.
///
/// @param lane The Lane to retrieve its direction usage. It must not be nullptr.
///
/// @throws maliput::common::assertion_error When lane is nullptr.
std::string GetDirectionUsageRuleStateType(const Lane* lane);

/// @returns A vector of XodrSpeedProperties which contains the speed limits information of a `lane`.
/// This method attempts to obtain the speed records for the entire `lane`'s range.
/// The speed records are obtained according the following priorities.
/// 1 - Speed records from the XODR::Lane.
/// 2 - Speed records from the RoadTypes of the Road.
/// 3 - When no speed limit record is provided in a certain range of the lane, a
///     constants::kDefaultMaxSpeedLimit value is provided to fullfil the gap.
///
/// @param lane A Lane to obtain the maximum speed limit for. It must not be
/// nullptr.
///
/// @throws maliput::common::assertion_error When `lane` is nullptr.
std::vector<rules::XodrSpeedProperties> GetMaxSpeedLimitFor(const Lane* lane);

/// @returns A pair whose first element is vehicle usage rule value and
///          second element is the vehicle exclusive rule value for
///          `lane`.
/// @throws maliput::common::assertion_error When `lane` is nullptr.
std::pair<std::string, std::optional<std::string>> VehicleUsageAndExclusiveRuleStateValues(const Lane* lane);

/// Finds the local minimum of the cubic polynomial.
///
/// @f$ f(p) = a p^3 + b p^2 + c p + d / p,a,b,c ∈ ℝ @f$.
///
/// Note:
/// 1 - Local minimum is only related to cubic polynomial. So when a = 0 and the parabola is ascending (b > 0) it
///     will return the absolute minimum.
/// 2 - There is no need to know `d` coefficient to get the minimum local, but it is
///     asked just for consistency.
/// @param a Cubic coefficient.
/// @param b Quadratic coefficient.
/// @param c Linear coefficient.
/// @param d Constant coefficient.
/// @returns The p value that matches a local min in the cubic, if exists.
std::optional<double> FindLocalMinFromCubicPol(double a, double b, double c, double d);

}  // namespace builder
}  // namespace malidrive

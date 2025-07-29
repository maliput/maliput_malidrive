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

#include <memory>
#include <optional>
#include <vector>

#include <maliput/geometry_base/road_geometry.h>

#include "maliput_malidrive/base/segment.h"
#include "maliput_malidrive/common/macros.h"
#include "maliput_malidrive/road_curve/road_curve.h"
#include "maliput_malidrive/xodr/db_manager.h"

namespace malidrive {

/// Maliput implementation of the malidrive backend.
class RoadGeometry final : public maliput::geometry_base::RoadGeometry {
 public:
  MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN(RoadGeometry);

  /// @brief Represents a OSC-XML Lane Position.
  /// See
  /// https://publications.pages.asam.net/standards/ASAM_OpenSCENARIO/ASAM_OpenSCENARIO_XML/latest/generated/content/LanePosition.html
  struct OpenScenarioLanePosition {
    /// The id of the road in the OpenDrive file.
    int road_id;
    /// The s-coordinate taken along the road's reference line from the start point of the target road.
    double s;
    /// The lane id.
    int lane_id;
    /// The lateral offset to the center line of the target lane.
    double offset;
  };

  /// @brief Represents a OSC-XML Road Position.
  /// See
  /// https://publications.pages.asam.net/standards/ASAM_OpenSCENARIO/ASAM_OpenSCENARIO_XML/latest/generated/content/RoadPosition.html
  struct OpenScenarioRoadPosition {
    /// The id of the road in the OpenDrive file.
    int road_id;
    /// The s-coordinate taken along the road's reference line from the start point of the target road.
    double s;
    /// The t-coordinate taken on the axis orthogonal to the reference line of the road.
    double t;
  };

  /// Constructs a RoadGeometry.
  ///
  /// @param id see @ref
  /// maliput::api::RoadGeometry::id() for reference.
  /// @param manager An xodr::DBManager that contains a parsed XODR description. It must not be nullptr.
  /// @param linear_tolerance see @ref
  /// maliput::api::RoadGeometry::linear_tolerance() for reference.
  /// @param angular_tolerance see @ref
  /// maliput::api::RoadGeometry::angular_tolerance() for reference.
  /// @param scale_length see @ref
  /// maliput::api::RoadGeometry::scale_length() for reference.
  /// @param inertial_to_backend_frame_translation maliput's Inertial Frame to Backend Frame translation vector.
  /// @throw maliput::common::assertion_error When @p manager_ is nullptr.
  RoadGeometry(const maliput::api::RoadGeometryId& id, std::unique_ptr<xodr::DBManager> manager,
               double linear_tolerance, double angular_tolerance, double scale_length,
               const maliput::math::Vector3& inertial_to_backend_frame_translation)
      : maliput::geometry_base::RoadGeometry(id, linear_tolerance, angular_tolerance, scale_length,
                                             inertial_to_backend_frame_translation),
        manager_(std::move(manager)) {
    MALIDRIVE_THROW_UNLESS(manager_ != nullptr);
  }

  /// Returns a xodr::DBManager.
  xodr::DBManager* get_manager() const { return manager_.get(); }

  /// Adds the description of a Road.
  /// @param road_id Is the xodr id of the road that `road_curve` belongs to.
  /// @param road_curve Is the RoadCurve to be added.
  /// @param reference_line_offset Is a Function that describes the lateral shift of the road reference line.
  ///
  /// @throw maliput::common::assertion_error When `road_curve` is nullptr.
  /// @throw maliput::common::assertion_error When `reference_line_offset` is nullptr.
  /// @throw maliput::common::assertion_error When `road_id` is duplicated.
  void AddRoadCharacteristics(const xodr::RoadHeader::Id& road_id, std::unique_ptr<road_curve::RoadCurve> road_curve,
                              std::unique_ptr<road_curve::Function> reference_line_offset);

  /// Gets the RoadCurve of `road_id`.
  /// @param road_id Is the xodr id of the road that `road_curve` belongs to.
  /// @returns A RoadCurve pointer.
  ///
  /// @throw maliput::common::assertion_error When there is no RoadCurve for `road_id`.
  const road_curve::RoadCurve* GetRoadCurve(const xodr::RoadHeader::Id& road_id) const;

  /// Gets the reference line offset function of `road_id`.
  /// @param road_id Is the xodr id of the road that `road_curve` belongs to.
  /// @returns A Function pointer to the road reference line offset function.
  ///
  /// @throw maliput::common::assertion_error When there is no a function described for `road_id`.
  const road_curve::Function* GetReferenceLineOffset(const xodr::RoadHeader::Id& road_id) const;

  /// Converts an OpenScenario LanePosition to a maliput RoadPosition.
  /// OS LanePosition's s-coordinate must be within the lane's length, but its offset can extend beyond the lane's
  /// surface. See
  /// https://publications.pages.asam.net/standards/ASAM_OpenSCENARIO/ASAM_OpenSCENARIO_XML/latest/generated/content/LanePosition.html
  ///
  /// @param xodr_lane_position The lane position to get the maliput road position from.
  /// @returns A maliput RoadPosition.
  ///
  /// @throws When the correspondent maliput lane is not found.
  maliput::api::RoadPosition OpenScenarioLanePositionToMaliputRoadPosition(
      const OpenScenarioLanePosition& xodr_lane_position) const;

  /// Converts a maliput RoadPosition to an OpenScenario LanePosition.
  /// See
  /// https://publications.pages.asam.net/standards/ASAM_OpenSCENARIO/ASAM_OpenSCENARIO_XML/latest/generated/content/LanePosition.html
  ///
  /// @param road_position The road position to get the OS lane position from.
  /// @returns An Open Scenario LanePosition.
  ///
  /// @throws When the correspondent open scenario road is not found.
  OpenScenarioLanePosition MaliputRoadPositionToOpenScenarioLanePosition(
      const maliput::api::RoadPosition& road_position) const;

  /// Converts an OpenScenario RoadPosition to a maliput RoadPosition.
  /// See
  /// https://publications.pages.asam.net/standards/ASAM_OpenSCENARIO/ASAM_OpenSCENARIO_XML/latest/generated/content/RoadPosition.html
  ///
  /// @returns A maliput RoadPosition.
  maliput::api::RoadPosition OpenScenarioRoadPositionToMaliputRoadPosition(
      const OpenScenarioRoadPosition& xodr_road_position) const;

  /// Converts a maliput RoadPosition to an OpenScenario RoadPosition.
  /// See
  /// https://publications.pages.asam.net/standards/ASAM_OpenSCENARIO/ASAM_OpenSCENARIO_XML/latest/generated/content/RoadPosition.html
  ///
  /// @param road_position The road position to get the OS road position from.
  /// @returns An Open Scenario RoadPosition.
  OpenScenarioRoadPosition MaliputRoadPositionToOpenScenarioRoadPosition(
      const maliput::api::RoadPosition& road_position) const;

  /// Converts an OpenScenario RelativeRoadPosition to a maliput RoadPosition.
  /// See
  /// https://publications.pages.asam.net/standards/ASAM_OpenSCENARIO/ASAM_OpenSCENARIO_XML/latest/generated/content/RelativeRoadPosition.html
  ///
  /// When xodr_ds makes xodr_s to be out of bounds of the road, the calculation continues with the connecting road.
  /// This make sense when only having a single connecting road (road successor in the xodr). When having multiple
  /// connecting roads, the calculation will arbitrarily choose one of the connecting roads. (Default branch via maliput
  /// api). This might need to change in the future as we probably want to have a more deterministic behavior that could
  /// reflect the expecting routing of the vehicle acting as reference.
  /// @param xodr_reference_road_position The OpenScenario RoadPosition used as reference.
  /// @param xodr_ds The offset along the road's reference line relative to the s-coordinate of the reference.
  /// @param xodr_dt The offset orthogonal to the road's reference line relative to the t-coordinate of the reference.
  ///
  /// @returns A maliput RoadPosition.
  maliput::api::RoadPosition OpenScenarioRelativeRoadPositionToMaliputRoadPosition(
      const OpenScenarioRoadPosition& xodr_reference_road_position, double xodr_ds, double xodr_dt) const;

  /// Converts an OpenScenario RelativeLanePosition to a maliput RoadPosition.
  /// See
  /// https://publications.pages.asam.net/standards/ASAM_OpenSCENARIO/ASAM_OpenSCENARIO_XML/latest/generated/content/RelativeLanePosition.html
  ///
  /// When ds_lane makes xodr_s to be out of bounds of the road, the calculation continues with the connecting road.
  /// This make sense when only having a single connecting road (road successor in the xodr). When having multiple
  /// connecting roads, the calculation will arbitrarily choose one of the connecting roads. (Default branch via maliput
  /// api). This might need to change in the future as we probably want to have a more deterministic behavior that could
  /// reflect the expecting routing of the vehicle acting as reference.
  /// @param xodr_reference_lane_position The OpenScenario LanePosition used as reference.
  /// @param d_lane The lane offset relative to the lane the reference.
  /// @param ds_lane The offset along the center line of the lane, where the reference entity is located.
  /// @param offset The lateral offset to the center line of the target lane (along the t-axis of the target lane center
  /// line). Offset may extend beyond the lane's surface.
  ///
  /// @returns A maliput RoadPosition.
  maliput::api::RoadPosition OpenScenarioRelativeLanePositionWithDsLaneToMaliputRoadPosition(
      const OpenScenarioLanePosition& xodr_reference_lane_position, int d_lane, double ds_lane, double offset) const;

  /// Converts an OpenScenario RelativeLanePosition to a maliput RoadPosition.
  /// See
  /// https://publications.pages.asam.net/standards/ASAM_OpenSCENARIO/ASAM_OpenSCENARIO_XML/latest/generated/content/RelativeRoadPosition.html
  ///
  /// When xodr_ds makes xodr_s to be out of bounds of the road, the calculation continues with the connecting road.
  /// This make sense when only having a single connecting road (road successor in the xodr). When having multiple
  /// connecting roads, the calculation will arbitrarily choose one of the connecting roads. (Default branch via maliput
  /// api). This might need to change in the future as we probably want to have a more deterministic behavior that could
  /// reflect the expecting routing of the vehicle acting as reference.
  /// @param xodr_reference_lane_position The OpenScenario LanePosition used as reference.
  /// @param d_lane The lane offset relative to the lane the reference.
  /// @param xodr_ds The offset along the road's reference line relative to the s-coordinate of the reference entity.
  /// @param offset The lateral offset to the center line of the target lane (along the t-axis of the target lane center
  /// line).
  ///
  /// @returns A maliput RoadPosition.
  maliput::api::RoadPosition OpenScenarioRelativeLanePositionWithDsToMaliputRoadPosition(
      const OpenScenarioLanePosition& xodr_reference_lane_position, int d_lane, double xodr_ds, double offset) const;

  /// Obtains the road's roll, pitch and yaw angles at the specified OpenScenario RoadPosition.
  ///
  /// @param xodr_road_position The OpenScenario RoadPosition to get the angles at.
  ///
  /// @returns A maliput RollPitchYaw.
  maliput::math::RollPitchYaw GetRoadOrientationAtOpenScenarioRoadPosition(
      const OpenScenarioRoadPosition& xodr_road_position) const;

  /// Finds the maliput lane that corresponds to the given OpenScenario LanePosition.
  ///
  /// @param xodr_lane_position The OpenScenario LanePosition to find the corresponding maliput lane.
  /// @returns A pointer to the corresponding maliput Lane, or nullptr if not found.
  const Lane* GetMaliputLaneFromOpenScenarioLanePosition(const OpenScenarioLanePosition& xodr_lane_position) const;

 private:
  // Holds the description of the Road.
  struct RoadCharacteristics {
    std::unique_ptr<road_curve::RoadCurve> road_curve;
    std::unique_ptr<road_curve::Function> reference_line_offset;
  };

  // Provide custom commands support for the backend.
  //
  // @details This function is expected to be called by the user via maliput::api::RoadGeometry::BackendCustomCommand.
  // The user can provide a command string that will be parsed and executed by the backend.
  // The backend returns a string with the result of the command execution.
  // The format defined for the command string and the output string is up to the backend implementation.
  // In this case the format is defined as follows:
  //  - Command: "<command_name>,<arg1>,<arg2>,...,<argN>"
  //  - Output: "<output1>,<output2>,...,<outputN>"
  //
  // Continue reading the documentation for the available commands and how they should be used.
  //
  // Available Commands:
  // - OpenScenarioLanePositionToMaliputRoadPosition
  //   - Converts an OpenScenario LanePosition to a maliput RoadPosition.
  //   - In/Out:
  //     - Input: "<xodr_road_id>,<xodr_s>,<xodr_lane_id>,<offset>"
  //     - Output: "<lane_id>,<s>,<r>,<h>"
  //   - See
  //   https://publications.pages.asam.net/standards/ASAM_OpenSCENARIO/ASAM_OpenSCENARIO_XML/latest/generated/content/LanePosition.html
  //
  // - OpenScenarioRoadPositionToMaliputRoadPosition
  //   - Converts an OpenScenario RoadPosition to a maliput RoadPosition.
  //   - In/Out:
  //     - Input: "<xodr_road_id>,<xodr_s>,<xodr_t>"
  //     - Output: "<lane_id>,<s>,<r>,<h>"
  //   - See
  //   https://publications.pages.asam.net/standards/ASAM_OpenSCENARIO/ASAM_OpenSCENARIO_XML/latest/generated/content/RoadPosition.html
  //
  // - MaliputRoadPositionToOpenScenarioLanePosition
  //   - Converts a maliput RoadPosition to an OpenScenario Lane Position
  //   - In/Out:
  //     - Input: "<lane_id>,<s>,<r>,<h>"
  //     - Output: "<xodr_road_id>,<xodr_s>,<xodr_lane_id>,<offset>"
  //   - See
  //   https://publications.pages.asam.net/standards/ASAM_OpenSCENARIO/ASAM_OpenSCENARIO_XML/latest/generated/content/LanePosition.html
  //
  // - MaliputRoadPositionToOpenScenarioRoadPosition
  //   - Converts a maliput RoadPosition to an OpenScenario Road Position
  //   - In/Out:
  //     - Input: "<lane_id>,<s>,<r>,<h>"
  //     - Output: "<xodr_road_id>,<xodr_s>,<xodr_t>"
  //   - See
  //   https://publications.pages.asam.net/standards/ASAM_OpenSCENARIO/ASAM_OpenSCENARIO_XML/latest/generated/content/RoadPosition.html
  //
  // - OpenScenarioRelativeRoadPositionToMaliputRoadPosition
  //   - Converts an OpenScenario RelativeRoadPosition to a maliput RoadPosition.
  //   - In/Out:
  //     - Input: "<xodr_road_id>,<xodr_s>,<xodr_t>,<ds>,<dt>"
  //     - Output: "<lane_id>,<s>,<r>,<h>"
  //   - See
  //   https://publications.pages.asam.net/standards/ASAM_OpenSCENARIO/ASAM_OpenSCENARIO_XML/latest/generated/content/RelativeRoadPosition.html
  //
  // - OpenScenarioRelativeLanePositionWithDsToMaliputRoadPosition
  //   - Converts an OpenScenario RelativeLanePosition to a maliput RoadPosition.
  //   - In/Out:
  //     - Input: "<xodr_road_id>,<xodr_lane_id>,<xodr_s>,<d_lane>,<xodr_ds>,<offset>"
  //     - Output: "<lane_id>,<s>,<r>,<h>"
  //   - See
  //   https://publications.pages.asam.net/standards/ASAM_OpenSCENARIO/ASAM_OpenSCENARIO_XML/latest/generated/content/RelativeLanePosition.html
  //
  // - OpenScenarioRelativeLanePositionWithDsLaneToMaliputRoadPosition
  //   - Converts an OpenScenario RelativeLanePosition to a maliput RoadPosition.
  //   - In/Out:
  //     - Input: "<xodr_road_id>,<xodr_lane_id>,<xodr_s>,<d_lane>,<ds_lane>,<offset>"
  //     - Output: "<lane_id>,<s>,<r>,<h>"
  //   - See
  //   https://publications.pages.asam.net/standards/ASAM_OpenSCENARIO/ASAM_OpenSCENARIO_XML/latest/generated/content/RelativeLanePosition.html
  //
  // - GetRoadOrientationAtOpenScenarioRoadPosition
  //   - Obtains the road orientation at a OpenScenario RoadPosition.
  //   - In/Out:
  //     - Input: "<xodr_road_id>,<xodr_s>,<xodr_t>"
  //     - Output: "<roll>,<pitch>,<yaw>"
  //   - See
  //   https://publications.pages.asam.net/standards/ASAM_OpenSCENARIO/ASAM_OpenSCENARIO_XML/latest/generated/content/RoadPosition.html
  //
  // @param command The command string to be executed by the backend.
  // @returns The output string of the command execution.
  //
  // @throws When the command is unknown or can't be executed.
  std::string DoBackendCustomCommand(const std::string& command) const override;

  // Finds the maliput segment that corresponds to the given OpenScenario RoadPosition.
  const Segment* FindSegmentByOpenScenarioRoadPosition(const OpenScenarioRoadPosition& xodr_road_position) const;

  // Finds the maliput lane that corresponds to the offset from the intial_lane.
  const Lane* ApplyOffsetToLane(const Lane* initial_lane, int lane_offset) const;

  // Get the Geo Reference info of the RoadGeometry.
  //
  // @returns A string with the Geo Reference info.
  std::string DoGeoReferenceInfo() const override;

  std::unique_ptr<xodr::DBManager> manager_;
  std::unordered_map<xodr::RoadHeader::Id, RoadCharacteristics> road_characteristics_;
};

}  // namespace malidrive

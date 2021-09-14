// Copyright 2020 Toyota Research Institute
#pragma once

#include <memory>
#include <optional>
#include <vector>

#include <maliput/geometry_base/road_geometry.h>

#include "maliput_malidrive/common/macros.h"
#include "maliput_malidrive/road_curve/road_curve.h"
#include "maliput_malidrive/xodr/db_manager.h"

namespace malidrive {

/// Maliput implementation of the malidrive backend.
class RoadGeometry final : public maliput::geometry_base::RoadGeometry {
 public:
  MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN(RoadGeometry);

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

 private:
  // Holds the description of the Road.
  struct RoadCharacteristics {
    std::unique_ptr<road_curve::RoadCurve> road_curve;
    std::unique_ptr<road_curve::Function> reference_line_offset;
  };

  maliput::api::RoadPositionResult DoToRoadPosition(
      const maliput::api::InertialPosition& inertial_pos,
      const std::optional<maliput::api::RoadPosition>& hint) const override;

  std::vector<maliput::api::RoadPositionResult> DoFindRoadPositions(
      const maliput::api::InertialPosition& inertial_position, double radius) const override;

  std::unique_ptr<xodr::DBManager> manager_;
  std::unordered_map<xodr::RoadHeader::Id, RoadCharacteristics> road_characteristics_;
};

}  // namespace malidrive

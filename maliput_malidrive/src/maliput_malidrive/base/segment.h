// Copyright 2020 Toyota Research Institute
#pragma once

#include <memory>
#include <utility>
#include <vector>

#include <maliput/geometry_base/segment.h>

#include "maliput_malidrive/common/macros.h"
#include "maliput_malidrive/road_curve/road_curve.h"
#include "maliput_malidrive/road_curve/road_curve_offset.h"

namespace malidrive {

/// Segment implementation that holds a road_curve::RoadCurve.
///
/// Child MalidriveLanes of this segment will be constructed as offsets of the
/// reference curve.
class Segment : public maliput::geometry_base::Segment {
 public:
  MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN(Segment)

  /// Constructs a Segment.
  /// The incidence region of the Segment on the @p road_curve will be
  /// delimited by the range composed by @p p0 and @p p1.
  ///
  /// @param id The id of the Segment.
  /// @param road_curve The reference curve of the Segment shared by all Lanes.
  ///        It must not be nullptr.
  /// @param reference_line_offset The road reference line offset function of the Segment shared by all Lanes.
  ///        It must not be nullptr.
  /// @param p0 The value of the @f$ p @f$ parameter of @p road_curve that matches the start of the Segment.
  /// @param p1 The value of the @f$ p @f$ parameter of @p road_curve that matches the finish of the Segment.
  ///
  /// @throws maliput::common::assertion_error When @p road_curve is nullptr.
  /// @throws maliput::common::assertion_error When @p reference_line_offset is nullptr.
  /// @throws maliput::common::assertion_error When @p p0 is negative.
  /// @throws maliput::common::assertion_error When @p p0 is greater than @p p1.
  Segment(const maliput::api::SegmentId& id, const road_curve::RoadCurve* road_curve,
          const road_curve::Function* reference_line_offset, double p0, double p1)
      : maliput::geometry_base::Segment(id),
        road_curve_(road_curve),
        reference_line_offset_(reference_line_offset),
        p0_(p0),
        p1_(p1) {
    MALIDRIVE_THROW_UNLESS(road_curve_ != nullptr);
    MALIDRIVE_THROW_UNLESS(reference_line_offset_ != nullptr);
    MALIDRIVE_THROW_UNLESS(p1 >= 0.);
    MALIDRIVE_THROW_UNLESS(p1 >= p0);
  }

  /// @returns The lower bound range of @f$ p @f$.
  double p0() const { return p0_; }

  /// @returns The upper bound range of @f$ p @f$.
  double p1() const { return p1_; }

  /// @return The reference curve.
  const road_curve::RoadCurve* road_curve() const { return road_curve_; }

  /// @return The reference line offset function.
  const road_curve::Function* reference_line_offset() const { return reference_line_offset_; }

 private:
  const road_curve::RoadCurve* road_curve_;
  const road_curve::Function* reference_line_offset_;
  const double p0_{};
  const double p1_{};
};

}  // namespace malidrive

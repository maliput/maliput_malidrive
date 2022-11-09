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

#include <maliput/common/range_validator.h>

#include "maliput_malidrive/common/macros.h"
#include "maliput_malidrive/road_curve/function.h"

namespace malidrive {
namespace road_curve {

/// Describes a LaneOffset function behind Function interface.
///
/// It is built from the previous laneOffset and laneWidth and the current laneWidth,
/// according with:
///
/// @f$ LaneOffset_{i}(p) = LaneOffset_{i-1}(p) + LaneWidth_{i-1}(p) / 2 + LaneWidth_{i}(p) / 2  @f$
/// And  @f$ p ∈ [p0; p1] @f$
///
/// The sign of @f$ LaneWidth_{i-1} and LaneWidth_{i }@f$ depend on whether the Lane is at right(-) or left(+) of the
/// center lane.
///
/// The innest LaneOffset (immediate to the center lane) uses the information of the ReferenceLineOffset function.
///
/// TODO(#594): Generalize idea of composing cubic polynomials.
class LaneOffset : public Function {
 public:
  MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN(LaneOffset);

  static constexpr const bool kAtRightFromCenterLane{true};
  static constexpr const bool kAtLeftFromCenterLane{false};

  /// Holds lane offset and width of the immediate previous lane.
  struct AdjacentLaneFunctions {
    /// LaneOffset's function.
    const Function* offset{nullptr};
    /// LaneWidth's function.
    const Function* width{nullptr};
  };

  /// Constructs a LaneOffset.
  ///
  /// @param adjacent_lane_functions Holds LaneOffset's function and LaneWidth's function of the immediate previous
  /// lane. It is nullopt if the lane is the centerlane.
  /// @param lane_width LaneWidth's function of the current lane.
  /// @param reference_line_offset Functions that describes the lateral shift of the road reference line.
  /// @param at_right True if the lane is at the right of the center lane.
  /// @param p0 Lower bound extreme of the parameter range. It must not be
  ///        negative and must be less than @p p1.
  /// @param p1 Upper bound extreme of the parameter range. It must be greater
  ///        than @p p0.
  /// @param linear_tolerance Tolerance of the range [ @p p0; @p p1] that will be
  ///        used to evaluate the parameter.
  /// @throws maliput::common::assertion_error When @p linear_tolerance is not
  ///         positive.
  /// @throws maliput::common::assertion_error When @p lane_width is nullptr.
  /// @throws maliput::common::assertion_error When @p reference_line_offset is nullptr.
  /// @throws maliput::common::assertion_error When @p adjacent_lane_functions 's offset value is nullptr.
  /// @throws maliput::common::assertion_error When @p adjacent_lane_functions 's width value is nullptr.
  LaneOffset(const std::optional<AdjacentLaneFunctions>& adjacent_lane_functions, const Function* lane_width,
             const Function* reference_line_offset, bool at_right, double p0, double p1, double linear_tolerance);

 private:
  double do_f(double p) const override;

  double do_f_dot(double p) const override;

  double do_f_dot_dot(double p) const override;

  double do_p0() const override { return p0_; }
  double do_p1() const override { return p1_; }
  bool DoIsG1Contiguous() const override { return true; }

  const std::optional<AdjacentLaneFunctions> adjacent_lane_functions_;
  const Function* lane_width_;
  const Function* reference_line_offset_;
  const bool at_right_{false};
  const double p0_{};
  const double p1_{};
  // Validates that p is within [p0_, p1_] with linear_tolerance.
  const maliput::common::RangeValidator validate_p_;
};

}  // namespace road_curve
}  // namespace malidrive

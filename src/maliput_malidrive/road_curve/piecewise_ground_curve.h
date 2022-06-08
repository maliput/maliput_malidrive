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

#include <functional>
#include <map>
#include <memory>
#include <vector>

#include "maliput_malidrive/common/macros.h"
#include "maliput_malidrive/road_curve/ground_curve.h"

namespace malidrive {
namespace road_curve {

/// GroundCurve specification for a reference curve that is described as a piecewise ground curve.
///
/// Queries accept p ∈ [p0, p1] with a linear tolerance.
class PiecewiseGroundCurve : public GroundCurve {
 public:
  MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN(PiecewiseGroundCurve);

  PiecewiseGroundCurve() = delete;

  /// Constructs a PiecewiseGroundCurve.
  ///
  /// @param ground_curves Contains the ground curves to combine.
  /// @param linear_tolerance A non-negative value expected to be the same as
  /// maliput::api::RoadGeometry::linear_tolerance().
  /// @param angular_tolerance A non-negative value expected to be the same as
  /// maliput::api::RoadGeometry::angular_tolerance().
  ///
  /// @throw maliput::common::assertion_error When @p ground_curves ' size is zero.
  /// @throw maliput::common::assertion_error When @p ground_curves contains a nullptr.
  /// @throw maliput::common::assertion_error When @p ground_curves ' curves are not G1 contiguous.
  /// @throw maliput::common::assertion_error When @p linear_tolerance is non-positive.
  /// @throw maliput::common::assertion_error When @p angular_tolerance is non-positive.
  /// @throw maliput::common::assertion_error When the resultant p1 is not sufficiently larger than the resultant p0.
  PiecewiseGroundCurve(std::vector<std::unique_ptr<GroundCurve>>&& ground_curves, double linear_tolerance,
                       double angular_tolerance);

 private:
  // Holds the p parameter range of a road curve.
  struct RoadCurveInterval {
    // Creates a RoadCurveInterval.
    // @param min_in Is the minimum value of the interval.
    // @param max_in Is the maximum value of the interval.
    // @throw maliput::common::assertion_error When `min_in` is greater than `max_in`.
    RoadCurveInterval(double min_in, double max_in) : min(min_in), max(max_in) {
      MALIDRIVE_THROW_UNLESS(min_in <= max_in);
    }

    // Creates a RoadCurveInterval where
    // the minimum value is equal to the maximum value.
    // @param min_max Is the minimum and maximum value of the interval.
    RoadCurveInterval(double min_max) : min(min_max), max(min_max) {}
    const double min{};
    const double max{};

    // Less than operator.
    bool operator<(const RoadCurveInterval& rhs) const;
  };

  // Maps `p` parameter with the GroundCurve and its corresponding p parameter.
  //
  // @param p Is the parameter in the PiecewiseGroundCurve domain.
  // @returns A pair where the first value is the GroundCurve and the second value is the p parameter for that
  // GroundCurve that matches with `p`.
  // @throws maliput::common::assertion_error When @p p is not in
  //         @f$ [`p0()`; `p1()`] @f$.
  std::pair<const GroundCurve*, double> GetGroundCurveFromP(double p) const;

  // Maps `p_i` parameter and `ground_curve` with the p parameter from the PiecewiseGroundCurve.
  // @remarks @p p_i is not the PiecewiseGroundCurve parameter but @p ground_curve parameter.
  //
  // @param ground_curve Is a GroundCurve, part of the PiecewiseGroundCurve.
  // @param p_i Is p parameter of `ground_curve`.
  // @returns The p parameter in the PiecewiseGroundCurve domain.
  // @throws maliput::common::assertion_error When @p ground_curve is nullptr.
  // @throws maliput::common::assertion_error When @p p_i is not in
  //         @f$ [ground_curve->p0(); ground_curve->p1()] @f$.
  double GetPiecewiseP(const GroundCurve* ground_curve, double p_i) const;

  double DoPFromP(double xodr_p) const override;

  maliput::math::Vector2 DoG(double p) const override;

  maliput::math::Vector2 DoGDot(double p) const override;

  double DoGInverse(const maliput::math::Vector2&) const override;

  double DoHeading(double p) const override;

  double DoHeadingDot(double p) const override;

  double DoArcLength() const override { return arc_length_; }
  double do_linear_tolerance() const override { return linear_tolerance_; }
  double do_p0() const override { return p0_; }
  double do_p1() const override { return p1_; }
  bool DoIsG1Contiguous() const override { return true; }

  // Contains all the ground curves of the PiecewiseGroundCurve.
  const std::vector<std::unique_ptr<GroundCurve>> ground_curves_{};
  // The linear tolerance.
  const double linear_tolerance_{};
  // The length of the curve.
  double arc_length_{};
  // The value of the p parameter at the start of the curve.
  double p0_{};
  // The value of the p parameter at the end of the curve.
  double p1_{};
  // Convenient map to efficiently obtain the correspondent GroundCurve from a RoadCurveInterval key.
  // @see RoadCurveInterval.
  std::map<RoadCurveInterval, GroundCurve*> interval_ground_curve_;
  // Validates that p is within [p0, p1] with linear_tolerance.
  std::function<double(double)> validate_p_{};
};

}  // namespace road_curve
}  // namespace malidrive

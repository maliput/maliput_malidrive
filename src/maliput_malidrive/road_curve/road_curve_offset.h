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

#include <maliput/drake/systems/analysis/antiderivative_function.h>
#include <maliput/drake/systems/analysis/scalar_initial_value_problem.h>

#include "maliput_malidrive/common/macros.h"
#include "maliput_malidrive/road_curve/road_curve.h"

namespace malidrive {
namespace road_curve {

/// Creates functors to compute numerical arc length integral of an offset of a
/// RoadCurve and its inverse function.
///
/// This class implements piecewise cubic polynomial integration via samples of
/// a RoadCurve::WDot() at different @f$ (p, r, h=0) @f$ points
/// generating functors that wrap the results of those integrals and allow
/// interpolation for faster computation.
///
/// Implementation of this class controls the tolerance via a heuristic. It uses
/// a relative tolerance based on the GroundCurve's arc length and the linear
/// tolerance. Careful revision of complex compound MalidriveRoadCurves must be
/// done to assure RoadGeometry's linear tolerance.
class RoadCurveOffset {
 public:
  MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN(RoadCurveOffset);

  /// Constructs a RoadCurveOffset.
  ///
  /// @param road_curve The RoadCurve to compute offsets. It must not
  ///        be nullptr.
  /// @param lane_offset Needed to calculate the derivative of the lateral component. It must not
  ///        be nullptr.
  /// @param p0 Initial @f$ p @f$ parameter value of the curve.
  /// @param p1 Final @f$ p @f$ parameter value of the curve.
  /// @throws maliput::common::assertion_error When @p road_curve is nullptr.
  /// @throws maliput::common::assertion_error When @p lane_offset is nullptr.
  /// @throws maliput::common::assertion_error When @p p0 is negative.
  /// @throws maliput::common::assertion_error When @p p0 is greater than @p p1.
  RoadCurveOffset(const RoadCurve* road_curve, const Function* lane_offset, double p0, double p1);

  /// Returns the constructor RoadCurve argument.
  const RoadCurve* road_curve() const { return road_curve_; }

  /// The relative tolerance is computed as the proportion of the `road_curve`'s
  /// linear tolerance with respect to its GroundCurve arc length.
  ///
  /// @returns The relative tolerance.
  double relative_tolerance() const { return relative_tolerance_; }

  /// Computes the arc length of a constant @p r lateral offset at @p p.
  ///
  /// @param p The parameter of `road_curve`.
  /// @return The arc length of the offset `road_curve` at @p p.
  double CalcSFromP(double p) const;

  /// Builds a functor that represents the relation @f$ P(s, r) -> p @f$ which
  /// is a function that maps the arc length @f$ s @f$ of an offset road curve.
  ///
  /// @return A functor that returns @f$ p @f$ given @f$ s @f$.
  std::function<double(double)> PFromS() const;

  /// Builds a functor that represents the relation @f$ S(p, r) -> s @f$ which
  /// is a function that maps the parameter @f$ p @f$ of the reference road
  /// curve and a lateral offset with the arc length of that
  /// offset road curve.
  ///
  /// @return A functor that returns @f$ s @f$ given @f$ p @f$.
  std::function<double(double)> SFromP() const;

  /// @returns The lower bound range of @f$ p @f$.
  double p0() const { return p0_; }

  /// @returns The upper bound range of @f$ p @f$.
  double p1() const { return p1_; }

 private:
  // Minimum value that #relative_tolerance_ could take.
  // Given that the accuracy of the integrator is the product
  // of #relative_tolerance_ and #kAccuracyMultiplier this minimum value
  // guarantees that the accuracy isn't set beyond the integrator limits.
  static constexpr double kMinRelativeTolerance{1e-8};

  // The result of multiplying it with #relative_tolerance_ is the accuracy of the integrator.
  // The value of this multiplier was empirically obtained out of all the maps
  // this package provides.
  static constexpr double kAccuracyMultiplier{1e-4};

  // Holds the RoadCurve.
  const RoadCurve* road_curve_{};

  // Holds the LaneOffset function.
  const Function* lane_offset_{};

  // Initial p parameter value of the curve.
  const double p0_{};

  // Final p parameter value of the curve.
  const double p1_{};

  // Holds the relative tolerance, a relative tolerance computed as the ratio
  // of `road_curve_->linear_tolerance()` and `road_curve_`'s GroundCurve arc
  // length.
  double relative_tolerance_{};

  // The inverse arc length IVP, or the parameter track_s as a function of the
  // arc length lane_s.
  std::unique_ptr<maliput::drake::systems::ScalarInitialValueProblem<double>> p_from_s_ivp_{};

  // The arc length function, or the arc length lane_s as a function of the
  // parameter track_s.
  std::unique_ptr<maliput::drake::systems::AntiderivativeFunction<double>> s_from_p_func_{};
};

}  // namespace road_curve
}  // namespace malidrive

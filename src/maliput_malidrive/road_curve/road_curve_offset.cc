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
#include "maliput_malidrive/road_curve/road_curve_offset.h"

#include <maliput/common/logger.h>
#include <maliput/common/maliput_unused.h>
#include <maliput/drake/integrator_configuration.h>
#include <maliput/math/saturate.h>
#include <maliput/math/vector.h>

namespace malidrive {
namespace road_curve {
namespace {

// Computes the maximum heading dot in the range [p0, p1] for a given road curve.
// Uses the underlying ground curve's HeadingDot method to evaluate the heading dot at various points
// within the specified range.
double GetMaximumHeadingDot(const RoadCurve& road_curve, double p0, double p1) {
  double max_heading_dot = 0.0;
  for (double p = p0; p < p1; p += road_curve.scale_length()) {
    max_heading_dot = std::max(max_heading_dot, std::abs(road_curve.ground_curve()->HeadingDot(p)));
  }
  return max_heading_dot;
}

// Arc length derivative function @f$ ds/dp = f(p; [r, h]) @f$ for
// numerical resolution of the @f$ s(p) @f$ mapping as an antiderivative
// computation (i.e. quadrature).
struct ArcLengthDerivativeFunction {
  MALIDRIVE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ArcLengthDerivativeFunction)

  ArcLengthDerivativeFunction() = delete;

  // Constructs the arc length derivative function for the given @p road_curve and @lane_offset.
  //
  // @throws maliput::common::road_geometry_construction_error When `road_curve` or `lane_offset` is nullptr.
  ArcLengthDerivativeFunction(const RoadCurve* road_curve, const Function* lane_offset, double p0, double p1)
      : road_curve_(road_curve), lane_offset_(lane_offset), p0_(p0), p1_(p1) {
    MALIDRIVE_THROW_UNLESS(road_curve_ != nullptr, maliput::common::road_geometry_construction_error);
    MALIDRIVE_THROW_UNLESS(lane_offset_ != nullptr, maliput::common::road_geometry_construction_error);
    MALIDRIVE_THROW_UNLESS(p0_ >= 0., maliput::common::road_geometry_construction_error);
    MALIDRIVE_THROW_UNLESS(p1_ >= p0_, maliput::common::road_geometry_construction_error);
  }

  // Computes the arc length derivative for the RoadCurve specified
  // at construction.
  //
  // @param p The parameterization value to evaluate the derivative at.
  // @param k The reference curve parameter vector, containing r and h
  //        coordinates respectively.
  // @return The arc length derivative value at the specified point.
  // @pre The given parameter vector @p k is bi-dimensional (holding r and h
  //      coordinates only).
  // @throws std::logic_error if preconditions are not met.
  double operator()(double p, const maliput::math::Vector2& k) const {
    // The integrator may exceed the integration more than the allowed tolerance.
    if (p > p1_) {
      maliput::log()->debug("The p value calculated by the integrator is ", p,
                            " and exceeds the p1 of the road curve which is: ", p1_, ".");
      p = p1_;
    }
    // The integrator may exceed the minimum allowed p value for the lane offset function. See #123.
    if (p < p0_) {
      maliput::log()->debug("The p value calculated by the integrator is ", p,
                            " and is lower than the p0 of the road curve which is: ", p0_, ".");
      p = p0_;
    }
    return road_curve_->WDot({p, lane_offset_->f(p), k[1]}, lane_offset_).norm();
  }

 private:
  // Associated RoadCurve instance.
  const RoadCurve* road_curve_{};
  const Function* lane_offset_{};
  double p0_{};
  double p1_{};
};

// Inverse arc length ODE function @f$ dp/ds = f(s, p; [r, h]) @f$
// for numerical resolution of the @f$ p(s) @f$ mapping as an scalar initial
// value problem for a given RoadCurve.
struct InverseArcLengthODEFunction {
  MALIDRIVE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(InverseArcLengthODEFunction)

  InverseArcLengthODEFunction() = delete;

  // Constructs an inverse arc length ODE for the given @p road_curve and @lane_offset.
  //
  // @throws maliput::common::road_geometry_construction_error When `road_curve` or `lane_offset` is nullptr.
  InverseArcLengthODEFunction(const RoadCurve* road_curve, const Function* lane_offset, double p0, double p1)
      : road_curve_(road_curve), lane_offset_(lane_offset), p0_(p0), p1_(p1) {
    MALIDRIVE_THROW_UNLESS(road_curve_ != nullptr, maliput::common::road_geometry_construction_error);
    MALIDRIVE_THROW_UNLESS(lane_offset_ != nullptr, maliput::common::road_geometry_construction_error);
    MALIDRIVE_THROW_UNLESS(p0_ >= 0., maliput::common::road_geometry_construction_error);
    MALIDRIVE_THROW_UNLESS(p1_ >= p0_, maliput::common::road_geometry_construction_error);
  }

  // Computes the inverse arc length derivative for the RoadCurve
  // specified at construction.
  //
  // @param s The arc length to evaluate the derivative at.
  // @param p The reference curve parameterization value to evaluate the
  //        derivative at.
  // @param k The parameter vector, containing r and h coordinates respectively.
  // @return The inverse arc length derivative value at the specified point.
  // @pre The given parameter vector @p k is bi-dimensional (holding r and h
  //      coordinates only).
  // @throws std::logic_error if preconditions are not met.
  double operator()(double s, double p, const maliput::math::Vector2& k) {
    maliput::common::unused(s);
    // The integrator may exceed the integration more than the allowed tolerance.
    if (p > p1_) {
      maliput::log()->debug("The p value calculated by the integrator is ", p,
                            " and exceeds the p1 of the road curve which is: ", p1_, ".");
      p = p1_;
    }
    // The integrator may exceed the minimum allowed p value for the lane offset function. See #123.
    if (p < p0_) {
      maliput::log()->debug("The p value calculated by the integrator is ", p,
                            " and is lower than the p0 of the road curve which is: ", p0_, ".");
      p = p0_;
    }
    return 1.0 / road_curve_->WDot({p, lane_offset_->f(p), k[1]}, lane_offset_).norm();
  }

 private:
  // Associated RoadCurve instance.
  const RoadCurve* road_curve_{};
  const Function* lane_offset_{};
  double p0_{};
  double p1_{};
};

}  // namespace

RoadCurveOffset::RoadCurveOffset(const RoadCurve* road_curve, const Function* lane_offset, double p0, double p1,
                                 double integrator_accuracy_multiplier)
    : road_curve_(road_curve), lane_offset_(lane_offset), p0_(p0), p1_(p1) {
  MALIDRIVE_THROW_UNLESS(road_curve_ != nullptr, maliput::common::road_geometry_construction_error);
  MALIDRIVE_THROW_UNLESS(lane_offset_ != nullptr, maliput::common::road_geometry_construction_error);
  MALIDRIVE_THROW_UNLESS(p0 >= 0., maliput::common::road_geometry_construction_error);
  MALIDRIVE_THROW_UNLESS(p0 <= p1, maliput::common::road_geometry_construction_error);
  MALIDRIVE_THROW_UNLESS(integrator_accuracy_multiplier > 0., maliput::common::road_geometry_construction_error);

  maliput::log()->trace("Creating RoadCurveOffset with p0: ", p0, " and p1: ", p1);

  // Sets default parameter value at the beginning of the curve to p0() by
  // default.
  const double initial_p_value = p0_;
  // Sets default arc length at the beginning of the curve to 0 by default.
  const double initial_s_value = 0.;
  // Sets default r and h coordinates to 0 by default.
  const maliput::math::Vector2 default_parameters(0., 0.);
  // Relative tolerance in path length is roughly bounded by e/L, where e is
  // the linear tolerance and L is the scale length. This can be seen by
  // considering straight path one scale length (or spatial period) long, and
  // then another path, whose deviation from the first is a sine function with
  // the same period and amplitude equal to the specified tolerance. The
  // difference in path length is bounded by 4e and the relative error is thus
  // bounded by 4e/L.
  // Relative tolerance is clamped to kMinRelativeTolerance to avoid setting
  // accuracy of the integrator that goes beyond the limit of the integrator.
  // The integrator_accuracy_multiplier is used to scale the relative tolerance based on the
  // desired accuracy of the integrator.
  relative_tolerance_ = std::max(integrator_accuracy_multiplier * road_curve_->linear_tolerance() / road_curve_->LMax(),
                                 kMinRelativeTolerance);

  // Note: Setting this tolerance is necessary to satisfy the
  // road geometry invariants (i.e., CheckInvariants()) in Builder::Build().
  // Consider modifying this accuracy if other tolerances are modified
  // elsewhere.
  const double integrator_accuracy{relative_tolerance_ * kAccuracyMultiplier};

  // Compute a step_multiplier to scale the initial step size and maximum step size
  // of the integrators. Use the maximum heading dot of the road curve
  // to determine the step_multiplier. This is a heuristic to ensure that the
  // integrators can handle the curvature of the road curve effectively.
  const double max_heading_dot = GetMaximumHeadingDot(*road_curve_, p0_, p1_);
  maliput::log()->trace("Maximum heading dot of the road curve: ", max_heading_dot);
  double step_multiplier = 1.;
  if (max_heading_dot > kCurvatureThresholdToOptimizeStep) {
    // When the maximum heading dot is greater than kCurvatureThresholdToOptimizeStep, we scale the
    // step sizes to ensure that the integrators can handle the curvature
    // effectively. This magic value (kCurvatureThresholdToOptimizeStep) is chosen based on empirical
    // observations and may need adjustment based on the specific road curve
    // characteristics. kCurvatureThresholdToOptimizeStep corresponds to a arc length with curvature of
    // kCurvatureThresholdToOptimizeStep.
    step_multiplier = std::min(max_heading_dot / kCurvatureThresholdToOptimizeStep, kMaxStepMultiplier);
    maliput::log()->trace("RoadCurve with high curvature detected.");
  }
  maliput::log()->trace("Integrator Config: Step multiplier for integrator step sizes: ", step_multiplier);

  const struct maliput::drake::IntegratorConfiguration arc_length_integrator_config {
    initial_p_value,        /* parameter_lower_bound */
        initial_s_value,    /* image_lower_bound */
        default_parameters, /* k */
        // Sets `s_from_p`'s integration accuracy and step sizes. Said steps
        // should not be too large, because that could make accuracy control
        // fail, nor too small to avoid wasting cycles. The nature of the
        // problem at hand varies with the parameterization of the RoadCurve,
        // and so will optimal step sizes (in terms of their efficiency vs.
        // accuracy balance). However, for the time being, the following
        // constants (considering p0_ <= p <= p1_) work well as a heuristic
        // approximation to appropriate step sizes.
        road_curve_->scale_length() * 0.1 / step_multiplier, /* initial_step_size_target */
        road_curve_->scale_length() * 0.5 / step_multiplier, /* maximum_step_size */
        integrator_accuracy                                  /* target_accuracy */
  };

  const struct maliput::drake::IntegratorConfiguration inverse_arc_length_integrator_config {
    initial_p_value,        /* parameter_lower_bound */
        initial_s_value,    /* image_lower_bound */
        default_parameters, /* k */
        // Sets `p_from_s`'s integration accuracy and step sizes. Said steps
        // should not be too large, because that could make accuracy control
        // fail, nor too small to avoid wasting cycles. The nature of the
        // problem at hand varies with the shape of the RoadCurve, and so will
        // optimal step sizes (in terms of their efficiency vs. accuracy balance).
        // However, for the time being, the following proportions of the scale
        // length work well as a heuristic approximation to appropriate step sizes.
        road_curve_->scale_length() * 0.1 / step_multiplier, /* initial_step_size_target */
        road_curve_->scale_length() * 0.5 / step_multiplier, /* maximum_step_size */
        integrator_accuracy                                  /* target_accuracy */
  };

  // Instantiates s(p) and p(s) mappings with default values.
  s_from_p_func_ = std::make_unique<maliput::drake::ArcLengthIntegrator>(
      ArcLengthDerivativeFunction(road_curve_, lane_offset_, p0_, p1_), arc_length_integrator_config);
  p_from_s_ivp_ = std::make_unique<maliput::drake::InverseArcLengthIntegrator>(
      InverseArcLengthODEFunction(road_curve_, lane_offset_, p0_, p1_), inverse_arc_length_integrator_config);
}

double RoadCurveOffset::CalcSFromP(double p) const {
  // Populates parameter vector with (r, h) coordinate values.
  return s_from_p_func_->Evaluate(p, maliput::math::Vector2(0.0, 0.0));
}

std::function<double(double)> RoadCurveOffset::SFromP() const {
  // Populates parameter vector with (r, h) coordinate values.
  return s_from_p_func_->IntegralFunction(p0_, p1_, maliput::math::Vector2(0.0, 0.0), road_curve_->linear_tolerance());
}

std::function<double(double)> RoadCurveOffset::PFromS() const {
  const double full_length = CalcSFromP(p1_);
  return p_from_s_ivp_->InverseFunction(0.0, full_length, maliput::math::Vector2(0.0, 0.0),
                                        road_curve_->linear_tolerance(), GroundCurve::kEpsilon);
}

}  // namespace road_curve
}  // namespace malidrive

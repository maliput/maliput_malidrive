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

#include <Eigen/Dense>
#include <maliput/common/logger.h>
#include <maliput/common/maliput_unused.h>
#include <maliput/drake/common/eigen_types.h>
#include <maliput/math/saturate.h>

namespace malidrive {
namespace road_curve {
namespace {

// Arc length derivative function @f$ ds/dp = f(p; [r, h]) @f$ for
// numerical resolution of the @f$ s(p) @f$ mapping as an antiderivative
// computation (i.e. quadrature).
struct ArcLengthDerivativeFunction {
  MALIDRIVE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ArcLengthDerivativeFunction)

  ArcLengthDerivativeFunction() = delete;

  // Constructs the arc length derivative function for the given @p road_curve and @lane_offset.
  //
  // @throws maliput::common::assertion_error When `road_curve` or `lane_offset` is nullptr.
  ArcLengthDerivativeFunction(const RoadCurve* road_curve, const Function* lane_offset, double p0, double p1)
      : road_curve_(road_curve), lane_offset_(lane_offset), p0_(p0), p1_(p1) {
    MALIDRIVE_THROW_UNLESS(road_curve_ != nullptr);
    MALIDRIVE_THROW_UNLESS(lane_offset_ != nullptr);
    MALIDRIVE_THROW_UNLESS(p0_ >= 0.);
    MALIDRIVE_THROW_UNLESS(p1_ >= p0_);
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
  double operator()(double p, const maliput::drake::VectorX<double>& k) const {
    if (k.size() != 2) {
      throw std::logic_error(
          "Arc length derivative param vector expects only r and h coordinates"
          " as parameters, respectively.");
    }
    // The integrator may exceed the integration more than the allowed tolerance.
    if (p > p1_) {
      const std::string msg{
          "The p value calculated by the integrator is {} and exceeds the p1 of the road curve which is: {}."};
      maliput::log()->warn(msg, p, p1_);
      p = p1_;
    }
    // The integrator may exceed the minimum allowed p value for the lane offset function. See #123.
    if (p < p0_) {
      const std::string msg{
          "The p value calculated by the integrator is {} and is lower than the p0 of the road curve which is: {}."};
      maliput::log()->warn(msg, p, p0_);
      p = p0_;
    }
    return road_curve_->WDot({p, lane_offset_->f(p), k(1)}, lane_offset_).norm();
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
  // @throws maliput::common::assertion_error When `road_curve` or `lane_offset` is nullptr.
  InverseArcLengthODEFunction(const RoadCurve* road_curve, const Function* lane_offset, double p0, double p1)
      : road_curve_(road_curve), lane_offset_(lane_offset), p0_(p0), p1_(p1) {
    MALIDRIVE_THROW_UNLESS(road_curve_ != nullptr);
    MALIDRIVE_THROW_UNLESS(lane_offset_ != nullptr);
    MALIDRIVE_THROW_UNLESS(p0_ >= 0.);
    MALIDRIVE_THROW_UNLESS(p1_ >= p0_);
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
  double operator()(double s, double p, const maliput::drake::VectorX<double>& k) {
    maliput::common::unused(s);
    if (k.size() != 2) {
      throw std::logic_error(
          "Inverse arc length ODE param vector expects only r and h "
          "coordinates as parameters, respectively.");
    }
    // The integrator may exceed the integration more than the allowed tolerance.
    if (p > p1_) {
      const std::string msg{
          "The p value calculated by the integrator is {} and exceeds the p1 of the road curve which is: {}."};
      maliput::log()->debug(msg, p, p1_);
      p = p1_;
    }
    // The integrator may exceed the minimum allowed p value for the lane offset function. See #123.
    if (p < p0_) {
      const std::string msg{
          "The p value calculated by the integrator is {} and is lower than the p0 of the road curve which is: {}."};
      maliput::log()->warn(msg, p, p0_);
      p = p0_;
    }
    return 1.0 / road_curve_->WDot({p, lane_offset_->f(p), k(1)}, lane_offset_).norm();
  }

 private:
  // Associated RoadCurve instance.
  const RoadCurve* road_curve_{};
  const Function* lane_offset_{};
  double p0_{};
  double p1_{};
};

}  // namespace

RoadCurveOffset::RoadCurveOffset(const RoadCurve* road_curve, const Function* lane_offset, double p0, double p1)
    : road_curve_(road_curve), lane_offset_(lane_offset), p0_(p0), p1_(p1) {
  MALIDRIVE_THROW_UNLESS(road_curve_ != nullptr);
  MALIDRIVE_THROW_UNLESS(lane_offset_ != nullptr);
  MALIDRIVE_THROW_UNLESS(p0 >= 0.);
  MALIDRIVE_THROW_UNLESS(p0 <= p1);

  // Sets default parameter value at the beginning of the curve to p0() by
  // default.
  const double initial_p_value = p0_;
  // Sets default arc length at the beginning of the curve to 0 by default.
  const double initial_s_value = 0.;
  // Sets default r and h coordinates to 0 by default.
  const maliput::drake::VectorX<double> default_parameters = maliput::drake::VectorX<double>::Zero(2);

  // Instantiates s(p) and p(s) mappings with default values.
  const maliput::drake::systems::AntiderivativeFunction<double>::IntegrableFunctionContext s_from_p_func_values(
      /* lower integration bound */ initial_p_value, /* parameter vector 𝐤 */ default_parameters);
  s_from_p_func_ = std::make_unique<maliput::drake::systems::AntiderivativeFunction<double>>(
      ArcLengthDerivativeFunction(road_curve_, lane_offset_, p0_, p1_), s_from_p_func_values);

  const maliput::drake::systems::ScalarInitialValueProblem<double>::ScalarOdeContext p_from_s_ivp_values(
      /* initial time t₀ for the IVP */ initial_s_value, /* initial state x₀ for the IVP */ initial_p_value,
      /* parameter vector 𝐤 for the IVP */ default_parameters);
  p_from_s_ivp_ = std::make_unique<maliput::drake::systems::ScalarInitialValueProblem<double>>(
      InverseArcLengthODEFunction(road_curve_, lane_offset_, p0_, p1_), p_from_s_ivp_values);

  // Relative tolerance in path length is roughly bounded by e/L, where e is
  // the linear tolerance and L is the scale length. This can be seen by
  // considering straight path one scale length (or spatial period) long, and
  // then another path, whose deviation from the first is a sine function with
  // the same period and amplitude equal to the specified tolerance. The
  // difference in path length is bounded by 4e and the relative error is thus
  // bounded by 4e/L.
  // Relative tolerance is clamped to kMinRelativeTolerance to avoid setting
  // accuracy of the integrator that goes beyond the limit of the integrator.
  relative_tolerance_ = std::max(road_curve_->linear_tolerance() / road_curve_->LMax(), kMinRelativeTolerance);
  // Sets `s_from_p`'s integration accuracy and step sizes. Said steps
  // should not be too large, because that could make accuracy control
  // fail, nor too small to avoid wasting cycles. The nature of the
  // problem at hand varies with the parameterization of the RoadCurve,
  // and so will optimal step sizes (in terms of their efficiency vs.
  // accuracy balance). However, for the time being, the following
  // constants (considering p0_ <= p <= p1_) work well as a heuristic
  // approximation to appropriate step sizes.
  maliput::drake::systems::IntegratorBase<double>& s_from_p_integrator = s_from_p_func_->get_mutable_integrator();
  s_from_p_integrator.request_initial_step_size_target(road_curve_->scale_length() * 0.1);
  s_from_p_integrator.set_maximum_step_size(road_curve_->scale_length());
  // Note: Setting this tolerance is necessary to satisfy the
  // road geometry invariants (i.e., CheckInvariants()) in Builder::Build().
  // Consider modifying this accuracy if other tolerances are modified
  // elsewhere.
  const double integrator_accuracy{relative_tolerance_ * kAccuracyMultiplier};
  s_from_p_integrator.set_target_accuracy(integrator_accuracy);

  // Sets `p_from_s`'s integration accuracy and step sizes. Said steps
  // should not be too large, because that could make accuracy control
  // fail, nor too small to avoid wasting cycles. The nature of the
  // problem at hand varies with the shape of the RoadCurve, and so will
  // optimal step sizes (in terms of their efficiency vs. accuracy balance).
  // However, for the time being, the following proportions of the scale
  // length work well as a heuristic approximation to appropriate step sizes.
  maliput::drake::systems::IntegratorBase<double>& p_from_s_integrator = p_from_s_ivp_->get_mutable_integrator();
  p_from_s_integrator.request_initial_step_size_target(road_curve_->scale_length() * 0.1);
  p_from_s_integrator.set_maximum_step_size(road_curve_->scale_length() / 2.);
  p_from_s_integrator.set_target_accuracy(integrator_accuracy);
}

double RoadCurveOffset::CalcSFromP(double p) const {
  // Populates parameter vector with (r, h) coordinate values.
  maliput::drake::systems::AntiderivativeFunction<double>::IntegrableFunctionContext context;
  context.k = (maliput::drake::VectorX<double>(2) << 0.0, 0.0).finished();
  return s_from_p_func_->Evaluate(p, context);
}

std::function<double(double)> RoadCurveOffset::SFromP() const {
  const double absolute_tolerance = relative_tolerance_ * road_curve_->LMax();
  // Populates parameter vector with (r, h) coordinate values.
  maliput::drake::systems::AntiderivativeFunction<double>::IntegrableFunctionContext context;
  context.k = (maliput::drake::VectorX<double>(2) << 0.0, 0.0).finished();
  // Prepares dense output for shared ownership, as std::function
  // instances only take copyable callables.
  const std::shared_ptr<maliput::drake::systems::ScalarDenseOutput<double>> dense_output{
      s_from_p_func_->MakeDenseEvalFunction(p1_, context)};
  MALIDRIVE_THROW_UNLESS(dense_output->start_time() <= p0_);
  MALIDRIVE_THROW_UNLESS(dense_output->end_time() >= p1_);
  return [dense_output, absolute_tolerance, p0 = p0_, p1 = p1_](double p) -> double {
    // Saturates p to lie within the [0., 1.] interval.
    const double saturated_p = maliput::math::saturate(p, p0, p1);
    MALIDRIVE_THROW_UNLESS(std::abs(saturated_p - p) < absolute_tolerance);
    return dense_output->EvaluateScalar(saturated_p);
  };
}

std::function<double(double)> RoadCurveOffset::PFromS() const {
  const double full_length = CalcSFromP(p1_);
  const double absolute_tolerance = relative_tolerance_ * full_length;

  // Populates parameter vector with (r, h) coordinate values.
  maliput::drake::systems::ScalarInitialValueProblem<double>::ScalarOdeContext context;
  context.k = (maliput::drake::VectorX<double>(2) << 0.0, 0.0).finished();
  // Prepares dense output for shared ownership, as std::function
  // instances only take copyable callables.
  const std::shared_ptr<maliput::drake::systems::ScalarDenseOutput<double>> dense_output{
      p_from_s_ivp_->DenseSolve(full_length, context)};
  MALIDRIVE_THROW_UNLESS(dense_output->start_time() <= 0.);
  // In order to avoid a numerical error issue, GroundCurve::kEpsilon is added to the equation.
  MALIDRIVE_THROW_UNLESS(dense_output->end_time() >= full_length - GroundCurve::kEpsilon);
  return [dense_output, full_length, absolute_tolerance](double s) -> double {
    // Saturates s to lie within the [0., full_length] interval.
    const double saturated_s = maliput::math::saturate(s, 0., full_length);
    MALIDRIVE_THROW_UNLESS(std::abs(saturated_s - s) < absolute_tolerance);
    return dense_output->EvaluateScalar(saturated_s);
  };
}

}  // namespace road_curve
}  // namespace malidrive

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

#include "maliput_malidrive/common/macros.h"
#include "maliput_malidrive/road_curve/function.h"
#include "maliput_malidrive/road_curve/open_range_validator.h"

namespace malidrive {
namespace road_curve {

/// Wrapper around a Function that composes a linear polynomial to scale
/// Function's domain. In Mathematical terms:
///
/// @f[
/// ScaledFunction(p*) = F(G(p*))
/// p = G(p*)
/// G(p*) = α p* + β
/// @f]
class ScaledDomainFunction : public Function {
 public:
  MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN(ScaledDomainFunction);

  /// Constructs a ScaledDomainFunction.
  ///
  /// @param function The Function to modify its domain.
  /// @param p0 The new p0 value to use as domain's lower boundary.
  /// @param p1 The new p1 value to use as domain's upper boundary.
  /// @param linear_tolerance Is the range extension to be accepted.
  ///
  /// @throws maliput::common::assertion_error When @p function is nullptr.
  /// @throws maliput::common::assertion_error When @p p0 is negative.
  /// @throws maliput::common::assertion_error When @p p1 is less than @p p0.
  ScaledDomainFunction(std::unique_ptr<Function> function, double p0, double p1, double linear_tolerance)
      : function_(std::move(function)),
        p0_(p0),
        p1_(p1),
        validate_p_(OpenRangeValidator::GetAbsoluteEpsilonValidator(p0_, p1_, linear_tolerance, Function::kEpsilon)) {
    MALIDRIVE_THROW_UNLESS(function_ != nullptr);
    MALIDRIVE_THROW_UNLESS(p0_ >= 0.);
    MALIDRIVE_THROW_UNLESS(p1_ > p0_);

    alpha_ = (function_->p1() - function_->p0()) / (p1_ - p0_);
    beta_ = function_->p0() - alpha_ * p0_;
  }

 private:
  double p_of_p(double p) const {
    p = validate_p_(p);
    return alpha_ * p + beta_;
  }

  double do_f(double p) const override {
    p = validate_p_(p);
    return function_->f(p_of_p(p));
  }

  double do_f_dot(double p) const override {
    p = validate_p_(p);
    return function_->f_dot(p_of_p(p)) * alpha_;
  }

  double do_f_dot_dot(double p) const override {
    p = validate_p_(p);
    return function_->f_dot_dot(p_of_p(p)) * alpha_ * alpha_;
  }

  double do_p0() const override { return p0_; }
  double do_p1() const override { return p1_; }
  bool DoIsG1Contiguous() const override { return true; }

  std::unique_ptr<Function> function_;
  const double p0_{};
  const double p1_{};
  double alpha_{};
  double beta_{};
  // Validates that p is within [p0, p1] with linear_tolerance.
  const OpenRangeValidator validate_p_;
};

}  // namespace road_curve
}  // namespace malidrive

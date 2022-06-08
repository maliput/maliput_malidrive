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

#include <maliput/math/vector.h>

#include "maliput_malidrive/common/macros.h"

namespace malidrive {
namespace road_curve {

/// Describes a parametric, flat and on-the-ground 2D curve. It will be also
/// referred as the reference curve.
///
/// In mathematical terms:
///
/// Let @f$ G(p) @f$ be a curve that maps @f$ p, p0, p1 ∈ ℝ / p ⊂ [p0, p1] ∧
/// p >= 0 @f$, to the @f$ (x, y) @f$ space, i.e. ℝ² in the INERTIAL Frame. Its
/// arc-length is @f$ L @f$ and the heading of the tangent vector in the
/// INERTIAL Frame is @f$ θ @f$. @f$ G(p) @f$ is G¹ in the domain of @f$ p @f$.
///
/// There exist:
///
/// - @f$ G(p) \mapsto ℝ² @f$.
/// - @f$ G'(p) \mapsto ℝ² @f$.
/// - @f$ θ(p) \mapsto [-π; π] @f$.
/// - @f$ θ'(p) \mapsto [-π; π] @f$ where @f$ θ' @f$ is the derivative of the
///   heading with respect to @f$ p @f$.
/// - @f$ G⁻¹(x, y) \mapsto [p0, p1] @f$, where @f$ G⁻¹@f$ is the inverse
///   function of @f$ G @f$.
class GroundCurve {
 public:
  /// Implementations may opt to allow a tolerance or be up to `kEpsilon` away
  /// from [p0(); p1()].
  static constexpr double kEpsilon = 1e-13;

  MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN(GroundCurve);
  virtual ~GroundCurve() = default;

  /// Calculates the @f$ p @f$ value that matches with the @f$ p @f$ value in the XODR description.
  /// @param xodr_p The parameter in the XODR description.
  /// @returns The @f$ p @f$ value in the GroundCurve domain.
  ///
  /// @throws maliput::common::assertion_error When @p xodr_p doesn't match to any parameter value in the XODR domain.
  double PFromP(double xodr_p) const { return DoPFromP(xodr_p); }

  /// Evaluates @f$ G(p) @f$.
  ///
  /// @param p The parameter. It must be in the range @f$ [`p0()`; `p1()`] @f$.
  /// @throws maliput::common::assertion_error When @p p is not in
  ///         @f$ [`p0()`; `p1()`] @f$.
  /// @return The image of @f$ G(p) @f$.
  maliput::math::Vector2 G(double p) const { return DoG(p); }

  /// Evaluates @f$ G'(p) @f$.
  ///
  /// @param p The parameter. It must be in the range @f$ [`p0()`; `p1()`] @f$.
  /// @throws maliput::common::assertion_error When @p p is not in
  ///         @f$ [`p0()`; `p1()`] @f$.
  /// @return The image of @f$ G'(p) @f$, i.e. the tangent vector.
  maliput::math::Vector2 GDot(double p) const { return DoGDot(p); }

  /// Evaluates @f$ θ(p) @f$.
  ///
  /// @param p The parameter. It must be in the range @f$ [`p0()`; `p1()`] @f$.
  /// @throws maliput::common::assertion_error When @p p is not in
  ///         @f$ [`p0()`; `p1()`] @f$.
  /// @return The heading of @f$ G(p) @f$ at @p p.
  double Heading(double p) const { return DoHeading(p); }

  /// Evaluates @f$ θ'(p) @f$.
  ///
  /// @param p The parameter. It must be in the range @f$ [`p0()`; `p1()`] @f$.
  /// @throws maliput::common::assertion_error When @p p is not in
  ///         @f$ [`p0()`; `p1()`] @f$.
  /// @return The derivative of the heading of @f$ G(p) @f$ at @p p.
  double HeadingDot(double p) const { return DoHeadingDot(p); }

  /// Evaluates @f$ G⁻¹(x, y) @f$.
  ///
  /// @param xy A point in ℝ² that is used as a point in the domain of
  ///        @f$ G⁻¹ @f$. When it does not belong to the domain of the inverse
  ///        function, a point that minimizes its distance is used instead.
  /// @return The p parameter that would minimize the Euclidean distance of
  ///         `G(p)` to @p xy.
  double GInverse(const maliput::math::Vector2& xy) const { return DoGInverse(xy); }

  /// @return The arc-length of @f$ G(p) @f$ in the range
  ///         @f$ [`p0()`; `p1()`] @f$.
  double ArcLength() const { return DoArcLength(); }

  /// @return The linear tolerance used by implementations to adapt to
  /// `maliput::api::RoadGeometry` requirements.
  double linear_tolerance() const { return do_linear_tolerance(); }

  /// @returns The lower bound range of @f$ p @f$.
  double p0() const { return do_p0(); }

  /// @returns The upper bound range of @f$ p @f$.
  double p1() const { return do_p1(); }

  /// @return True when @f$ F(p) @f$ is G¹ in the interval
  ///         @f$ [`p0()`; `p1()`] @f$.
  bool IsG1Contiguous() const { return DoIsG1Contiguous(); }

 protected:
  GroundCurve() = default;

 private:
  // Virtual private definitions of the public members. Public member
  // constraints apply the same.
  //@{
  virtual double DoPFromP(double xodr_p) const = 0;
  virtual maliput::math::Vector2 DoG(double p) const = 0;
  virtual maliput::math::Vector2 DoGDot(double p) const = 0;
  virtual double DoHeading(double p) const = 0;
  virtual double DoHeadingDot(double p) const = 0;
  virtual double DoGInverse(const maliput::math::Vector2& xy) const = 0;
  virtual double DoArcLength() const = 0;
  virtual double do_linear_tolerance() const = 0;
  virtual double do_p0() const = 0;
  virtual double do_p1() const = 0;
  virtual bool DoIsG1Contiguous() const = 0;
  //@}
};

}  // namespace road_curve
}  // namespace malidrive

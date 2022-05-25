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

#include "maliput_malidrive/common/macros.h"

namespace malidrive {
namespace road_curve {

/// Describes a G¹ scalar function.
///
/// This interface will be the interface for other specializations which are
/// used by RoadCurve to compose the path of maliput::api::Lane.
///
/// In mathematical terms, let:
///
/// @f$ F(p) @f$ be a function of a real and non-negative parameter.
///
/// Implementations of @f$ F(p) @f$ *should* enforce:
///
/// - @f$ F(p) @f$ is G¹ in the interval @f$ [p_0; p_1] @f$.
///
/// Implementations of @f$ F(p) @f$ *must* assure:
///
/// - There exists @f$ F'(p) @f$ and @f$ F''(p) @f$ in the interval @f$ [p_0; p_1] @f$.
/// - Proper G¹ checks via IsG1Contiguous(), specially when it is piecewise-defined.
class Function {
 public:
  /// Implementations may opt to allow a tolerance or be up to `kEpsilon` away
  /// from [p0(); p1()].
  static constexpr double kEpsilon = 0.;

  MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN(Function);
  virtual ~Function() = default;

  /// Evaluates @f$ F(p) @f$.
  ///
  /// @param p The parameter. It must be in the range @f$ [`p0()`; `p1()`] @f$.
  /// @throws maliput::common::assertion_error When @p p is not in
  ///         @f$ [`p0()`; `p1()`] @f$.
  /// @return The image of @f$ F(p) @f$.
  double f(double p) const { return do_f(p); }

  /// Evaluates @f$ F'(p) @f$.
  ///
  /// @param p The parameter. It must be in the range @f$ [`p0()`; `p1()`] @f$.
  /// @throws maliput::common::assertion_error When @p p is not in
  ///         @f$ [`p0()`; `p1()`] @f$.
  /// @return The image of @f$ F'(p) @f$.
  double f_dot(double p) const { return do_f_dot(p); }

  /// Evaluates @f$ F''(p) @f$.
  ///
  /// @param p The parameter. It must be in the range @f$ [`p0()`; `p1()`] @f$.
  /// @throws maliput::common::assertion_error When @p p is not in
  ///         @f$ [`p0()`; `p1()`] @f$.
  /// @return The image of @f$ F''(p) @f$.
  double f_dot_dot(double p) const { return do_f_dot_dot(p); }

  /// @returns The lower bound range of @f$ p @f$.
  double p0() const { return do_p0(); }

  /// @returns The upper bound range of @f$ p @f$.
  double p1() const { return do_p1(); }

  /// @return True when @f$ F(p) @f$ is G¹ in the interval @f$ [p_0; p_1] @f$.
  bool IsG1Contiguous() const { return DoIsG1Contiguous(); }

 protected:
  Function() = default;

 private:
  // Virtual private definitions of the public members. Public member
  // constraints apply the same.
  //@{
  virtual double do_f(double p) const = 0;
  virtual double do_f_dot(double p) const = 0;
  virtual double do_f_dot_dot(double p) const = 0;
  virtual double do_p0() const = 0;
  virtual double do_p1() const = 0;
  virtual bool DoIsG1Contiguous() const = 0;
  //@}
};

}  // namespace road_curve
}  // namespace malidrive

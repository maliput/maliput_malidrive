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
#include "maliput_malidrive/road_curve/function.h"

namespace malidrive {
namespace road_curve {
namespace test {

/// Stub class that forwards the argument list of its constructor to each
/// private interface implementation.
class FunctionStub : public Function {
 public:
  MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN(FunctionStub);

  FunctionStub(double f_result, double f_dot_result, double f_dot_dot_result, double p0_result, double p1_result,
               bool is_g1_contiguous)
      : f_result_(f_result),
        f_dot_result_(f_dot_result),
        f_dot_dot_result_(f_dot_dot_result),
        p0_result_(p0_result),
        p1_result_(p1_result),
        is_g1_contiguous_(is_g1_contiguous) {}

 private:
  // Virtual private definitions of the public members. Public member
  // constraints apply the same.
  //@{
  double do_f(double) const override { return f_result_; }
  double do_f_dot(double) const override { return f_dot_result_; }
  double do_f_dot_dot(double) const override { return f_dot_dot_result_; }
  double do_p0() const override { return p0_result_; }
  double do_p1() const override { return p1_result_; }
  bool DoIsG1Contiguous() const override { return is_g1_contiguous_; }
  //@}

  const double f_result_{};
  const double f_dot_result_{};
  const double f_dot_dot_result_{};
  const double p0_result_{};
  const double p1_result_{};
  const bool is_g1_contiguous_{};
};

}  // namespace test
}  // namespace road_curve
}  // namespace malidrive

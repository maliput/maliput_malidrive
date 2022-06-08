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
#include "maliput_malidrive/road_curve/open_range_validator.h"

#include <algorithm>

namespace malidrive {
namespace road_curve {

OpenRangeValidator OpenRangeValidator::GetRelativeEpsilonValidator(double min, double max, double tolerance,
                                                                   double epsilon) {
  return OpenRangeValidator{min, max, tolerance, epsilon, EpsilonUse::kRelative};
}

OpenRangeValidator OpenRangeValidator::GetAbsoluteEpsilonValidator(double min, double max, double tolerance,
                                                                   double epsilon) {
  return OpenRangeValidator{min, max, tolerance, epsilon, EpsilonUse::kAbsolute};
}

OpenRangeValidator::OpenRangeValidator(double min, double max, double tolerance, double epsilon,
                                       const EpsilonUse& epsilon_mode)
    : min_(min), max_(max), tolerance_(tolerance), epsilon_(epsilon) {
  MALIDRIVE_THROW_UNLESS(tolerance_ > 0.);

  if (epsilon_mode == EpsilonUse::kRelative) {
    // Multiplying the range by kEpsilon creates an epsilon value that is relative to the
    // length of the range. This avoids numerical errors when having a long range and a very small epsilon value
    // (~1e-14) leads to calculation that goes beyond the minimal useful digit of the double type.
    epsilon_ = (max_ - min_) * epsilon_;
  }
  MALIDRIVE_IS_IN_RANGE(epsilon_, 0., tolerance_);
  MALIDRIVE_VALIDATE((min_ + epsilon_) <= max_, maliput::common::assertion_error,
                     std::string("Open range lower bound <") + std::to_string((min_ + epsilon_)) +
                         "> is greater than <" + std::to_string(max_) + ">");
  MALIDRIVE_VALIDATE(min <= (max_ - epsilon_), maliput::common::assertion_error,
                     std::string("Open range upper bound <") + std::to_string((max_ - epsilon_)) + "> is less than <" +
                         std::to_string(min) + ">");
}

double OpenRangeValidator::operator()(double s) const {
  MALIDRIVE_IS_IN_RANGE(s, min_ - tolerance_, max_ + tolerance_);
  return std::clamp(s, min_ + epsilon_, max_ - epsilon_);
}

}  // namespace road_curve
}  // namespace malidrive

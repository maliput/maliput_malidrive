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
#include "maliput_malidrive/road_curve/lane_offset.h"

namespace malidrive {
namespace road_curve {
namespace {

// @returns -1 or 1 when `at_right` is LaneOffset::kAtRightFromCenterLane or
//          LaneOffset::kAtLeftFromCenterLane respectively.
double LaneSign(bool at_right) { return at_right == LaneOffset::kAtRightFromCenterLane ? -1. : 1.; }

}  // namespace

LaneOffset::LaneOffset(const std::optional<AdjacentLaneFunctions>& adjacent_lane_functions, const Function* lane_width,
                       const Function* reference_line_offset, bool at_right, double p0, double p1,
                       double linear_tolerance)
    : adjacent_lane_functions_(adjacent_lane_functions),
      lane_width_(lane_width),
      reference_line_offset_(reference_line_offset),
      at_right_(at_right),
      p0_(p0),
      p1_(p1),
      validate_p_(maliput::common::RangeValidator::GetAbsoluteEpsilonValidator(p0_, p1_, linear_tolerance,
                                                                               Function::kEpsilon)) {
  MALIDRIVE_THROW_UNLESS(p0_ >= 0.);
  MALIDRIVE_THROW_UNLESS(p1_ > p0_);
  MALIDRIVE_THROW_UNLESS(linear_tolerance > 0.);
  MALIDRIVE_THROW_UNLESS(lane_width_ != nullptr);
  MALIDRIVE_THROW_UNLESS(reference_line_offset_ != nullptr);
  MALIDRIVE_THROW_UNLESS(std::abs(lane_width_->p0() - p0_) <= linear_tolerance);
  MALIDRIVE_THROW_UNLESS(std::abs(lane_width_->p1() - p1_) <= linear_tolerance);
  MALIDRIVE_THROW_UNLESS(reference_line_offset_->p0() <= p0 + linear_tolerance);
  MALIDRIVE_THROW_UNLESS(reference_line_offset_->p1() >= p1_ - linear_tolerance);
  if (adjacent_lane_functions_.has_value()) {
    MALIDRIVE_THROW_UNLESS(adjacent_lane_functions_->offset != nullptr);
    MALIDRIVE_THROW_UNLESS(adjacent_lane_functions_->width != nullptr);
    MALIDRIVE_THROW_UNLESS(std::abs(adjacent_lane_functions_->offset->p0() - p0_) <= linear_tolerance);
    MALIDRIVE_THROW_UNLESS(std::abs(adjacent_lane_functions_->offset->p1() - p1_) <= linear_tolerance);
    MALIDRIVE_THROW_UNLESS(std::abs(adjacent_lane_functions_->width->p0() - p0_) <= linear_tolerance);
    MALIDRIVE_THROW_UNLESS(std::abs(adjacent_lane_functions_->width->p1() - p1_) <= linear_tolerance);
  }
}

double LaneOffset::do_f(double p) const {
  p = validate_p_(p);
  const double lane_offset_i_1{adjacent_lane_functions_.has_value() ? adjacent_lane_functions_->offset->f(p)
                                                                    : reference_line_offset_->f(p)};
  const double lane_width_i_1{adjacent_lane_functions_.has_value() ? adjacent_lane_functions_->width->f(p) : 0.};
  return lane_offset_i_1 + LaneSign(at_right_) * lane_width_i_1 / 2 + LaneSign(at_right_) * lane_width_->f(p) / 2;
}

double LaneOffset::do_f_dot(double p) const {
  p = validate_p_(p);
  const double lane_offset_i_1_dot{adjacent_lane_functions_.has_value() ? adjacent_lane_functions_->offset->f_dot(p)
                                                                        : reference_line_offset_->f_dot(p)};
  const double lane_width_i_1_dot{adjacent_lane_functions_.has_value() ? adjacent_lane_functions_->width->f_dot(p)
                                                                       : 0.};
  return lane_offset_i_1_dot + LaneSign(at_right_) * lane_width_i_1_dot / 2 +
         LaneSign(at_right_) * lane_width_->f_dot(p) / 2;
}

double LaneOffset::do_f_dot_dot(double p) const {
  p = validate_p_(p);
  const double lane_offset_i_1_dot_dot{adjacent_lane_functions_.has_value()
                                           ? adjacent_lane_functions_->offset->f_dot_dot(p)
                                           : reference_line_offset_->f_dot_dot(p)};
  const double lane_width_i_1_dot_dot{
      adjacent_lane_functions_.has_value() ? adjacent_lane_functions_->width->f_dot_dot(p) : 0.};
  return lane_offset_i_1_dot_dot + LaneSign(at_right_) * lane_width_i_1_dot_dot / 2 +
         LaneSign(at_right_) * lane_width_->f_dot_dot(p) / 2;
}

}  // namespace road_curve
}  // namespace malidrive

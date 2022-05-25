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
#include "maliput_malidrive/road_curve/piecewise_ground_curve.h"

#include <algorithm>
#include <cmath>

#include <maliput/common/assertion_error.h>
#include <maliput/common/logger.h>

#include "maliput_malidrive/road_curve/open_range_validator.h"

namespace malidrive {
namespace road_curve {
namespace {

// Functor that evalutes G1 contiguity between two ground curves.
struct G1ContiguityChecker {
  G1ContiguityChecker() = delete;
  // Creates a G1ContiguityChecker.
  // @param linear_tolerance_in A non-negative value expected to be the same as
  // maliput::api::RoadGeometry::linear_tolerance().
  // @param angular_tolerance_in A non-negative value expected to be the same as
  // maliput::api::RoadGeometry::linear_tolerance().
  G1ContiguityChecker(double linear_tolerance_in, double angular_tolerance_in)
      : linear_tolerance(linear_tolerance_in), angular_tolerance(angular_tolerance_in) {}

  // Determines whether the `lhs` and `rhs` ground curves are contiguous
  // in their endpoint and startpoint, respectively.
  // @throws maliput::common::assertion_error When linear tolerance constraint is not met.
  // @throws maliput::common::assertion_error When angular tolerance constraint is not met.
  void operator()(const GroundCurve* lhs, GroundCurve* rhs) const {
    const double ground_curve_endpoint_distance = (lhs->G(lhs->p1()) - rhs->G(rhs->p0())).norm();
    MALIDRIVE_VALIDATE(ground_curve_endpoint_distance <= linear_tolerance, maliput::common::assertion_error,
                       std::string("Error when constructing piecewise ground curve. Endpoint distance is <") +
                           std::to_string(ground_curve_endpoint_distance) +
                           "> which is greater than linear_tolerance: " + std::to_string(linear_tolerance) + ">.");
    double delta_heading = std::fmod(std::abs(lhs->Heading(lhs->p1()) - rhs->Heading(rhs->p0())), M_PI);
    if (std::abs(delta_heading - M_PI) < angular_tolerance) {
      delta_heading = 0.;
    }
    MALIDRIVE_VALIDATE(delta_heading <= angular_tolerance, maliput::common::assertion_error,
                       std::string("Error when constructing piecewise ground curve. Angular distance is <") +
                           std::to_string(delta_heading) +
                           "> which is greater than angular_tolerance: " + std::to_string(angular_tolerance) + ">.");
  }

  double linear_tolerance{};
  double angular_tolerance{};
};

}  // namespace

PiecewiseGroundCurve::PiecewiseGroundCurve(std::vector<std::unique_ptr<GroundCurve>>&& ground_curves,
                                           double linear_tolerance, double angular_tolerance)
    : ground_curves_(std::move(ground_curves)), linear_tolerance_(linear_tolerance) {
  MALIDRIVE_THROW_UNLESS(ground_curves_.size() > 0);
  MALIDRIVE_THROW_UNLESS(linear_tolerance_ > 0);
  MALIDRIVE_THROW_UNLESS(angular_tolerance > 0);

  MALIDRIVE_THROW_UNLESS(ground_curves_[0] != nullptr);
  double cumulative_p{0.};
  GroundCurve* previous_ground_curve{nullptr};
  const G1ContiguityChecker contiguity_checker(linear_tolerance_, angular_tolerance);
  for (const auto& ground_curve : ground_curves_) {
    MALIDRIVE_THROW_UNLESS(ground_curve != nullptr);
    MALIDRIVE_THROW_UNLESS(ground_curve->IsG1Contiguous());
    arc_length_ += ground_curve->ArcLength();
    const double delta_p = ground_curve->p1() - ground_curve->p0();
    if (previous_ground_curve != nullptr) {
      contiguity_checker(previous_ground_curve, ground_curve.get());
    }
    interval_ground_curve_.emplace(RoadCurveInterval{cumulative_p, cumulative_p + delta_p}, ground_curve.get());
    cumulative_p += delta_p;
    previous_ground_curve = ground_curve.get();
  }
  p0_ = 0.;
  p1_ = cumulative_p;
  MALIDRIVE_THROW_UNLESS(p1_ - p0_ >= kEpsilon);
  validate_p_ = OpenRangeValidator::GetRelativeEpsilonValidator(p0_, p1_, linear_tolerance_, kEpsilon);
}

std::pair<const GroundCurve*, double> PiecewiseGroundCurve::GetGroundCurveFromP(double p) const {
  p = validate_p_(p);
  auto search_it = interval_ground_curve_.find(RoadCurveInterval(p));
  if (search_it == interval_ground_curve_.end()) {
    MALIDRIVE_THROW_MESSAGE(std::string("p = ") + std::to_string(p) +
                            std::string(" doesn't match with any GroundCurve interval."));
  }
  const GroundCurve* gc = search_it->second;
  const RoadCurveInterval interval = search_it->first;

  const double p_result = gc->p0() + p - interval.min;

  return {gc, p_result};
}

double PiecewiseGroundCurve::GetPiecewiseP(const GroundCurve* ground_curve, double p_i) const {
  MALIDRIVE_THROW_UNLESS(ground_curve != nullptr);
  p_i = validate_p_(p_i);
  const auto range_gc =
      std::find_if(interval_ground_curve_.begin(), interval_ground_curve_.end(),
                   [ground_curve](std::pair<RoadCurveInterval, const GroundCurve*> interval_ground_curve) {
                     return interval_ground_curve.second == ground_curve;
                   });
  return range_gc->first.min + p_i - ground_curve->p0();
}

double PiecewiseGroundCurve::DoPFromP(double xodr_p) const {
  // The RoadCurveInterval map can't be used because those intervals are processed in the PiecewiseGroundCurve domain.
  const auto ground_curve_it = std::find_if(ground_curves_.begin(), ground_curves_.end(),
                                            [tol = linear_tolerance_, xodr_p](const std::unique_ptr<GroundCurve>& gc) {
                                              return (xodr_p >= gc->p0() - tol) && (xodr_p < gc->p1() + tol);
                                            });
  MALIDRIVE_THROW_UNLESS(ground_curve_it != ground_curves_.end());
  return GetPiecewiseP(ground_curve_it->get(), xodr_p);
}

double PiecewiseGroundCurve::DoGInverse(const maliput::math::Vector2& xy) const {
  std::pair<double, GroundCurve*> p_ground_curve;
  double minimum_distance{std::numeric_limits<double>::infinity()};
  for (const auto& ground_curve : ground_curves_) {
    double p_i{};
    try {
      p_i = ground_curve->GInverse(xy);
    } catch (const maliput::common::assertion_error&) {
      maliput::log()->info("Mapping to p parameter is not possible at [{}, {}].", xy.x(), xy.y());
      p_i = ground_curve->p0();
    }
    const maliput::math::Vector2 xy_i = ground_curve->G(p_i);
    const double distance = (xy - xy_i).norm();
    if (distance < minimum_distance) {
      minimum_distance = distance;
      p_ground_curve = {p_i, ground_curve.get()};
    }
  }
  return GetPiecewiseP(p_ground_curve.second, p_ground_curve.first);
};

maliput::math::Vector2 PiecewiseGroundCurve::DoG(double p) const {
  auto ground_curve_p = GetGroundCurveFromP(p);
  return ground_curve_p.first->G(ground_curve_p.second);
}

maliput::math::Vector2 PiecewiseGroundCurve::DoGDot(double p) const {
  auto ground_curve_p = GetGroundCurveFromP(p);
  return ground_curve_p.first->GDot(ground_curve_p.second);
}

double PiecewiseGroundCurve::DoHeading(double p) const {
  auto ground_curve_p = GetGroundCurveFromP(p);
  return ground_curve_p.first->Heading(ground_curve_p.second);
}

double PiecewiseGroundCurve::DoHeadingDot(double p) const {
  auto ground_curve_p = GetGroundCurveFromP(p);
  return ground_curve_p.first->HeadingDot(ground_curve_p.second);
}

bool PiecewiseGroundCurve::RoadCurveInterval::operator<(const RoadCurveInterval& rhs) const {
  if (min < rhs.min) {
    return max <= rhs.max ? true : false;
  } else {
    return false;
  }
}

}  // namespace road_curve
}  // namespace malidrive

// Copyright 2020 Toyota Research Institute
#pragma once

#include <maliput/math/vector.h>
#include "maliput_malidrive/common/macros.h"
#include "maliput_malidrive/road_curve/ground_curve.h"

namespace malidrive {
namespace road_curve {
namespace test {

/// Stub type that returns initialization values on each interface API.
class GroundCurveStub : public GroundCurve {
 public:
  MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN(GroundCurveStub);
  GroundCurveStub(const maliput::math::Vector2& g_result, const maliput::math::Vector2& g_dot_result,
                  double heading_result, double heading_dot_result, double d_g_inverse_result, double arc_length_result,
                  double linear_tolerance_result, double p0_result, double p1_result, bool is_g1_contiguous_result)
      : g_result_(g_result),
        g_dot_result_(g_dot_result),
        heading_result_(heading_result),
        heading_dot_result_(heading_dot_result),
        d_g_inverse_result_(d_g_inverse_result),
        arc_length_result_(arc_length_result),
        linear_tolerance_result_(linear_tolerance_result),
        p0_result_(p0_result),
        p1_result_(p1_result),
        is_g1_contiguous_result_(is_g1_contiguous_result) {}

 private:
  double DoPFromP(double xodr_p) const override { return xodr_p; }
  maliput::math::Vector2 DoG(double) const override { return g_result_; }
  maliput::math::Vector2 DoGDot(double) const override { return g_dot_result_; }
  double DoHeading(double) const override { return heading_result_; }
  double DoHeadingDot(double) const override { return heading_dot_result_; }
  double DoGInverse(const maliput::math::Vector2&) const override { return d_g_inverse_result_; }
  double DoArcLength() const override { return arc_length_result_; }
  double do_linear_tolerance() const override { return linear_tolerance_result_; }
  double do_p0() const override { return p0_result_; }
  double do_p1() const override { return p1_result_; }
  bool DoIsG1Contiguous() const override { return is_g1_contiguous_result_; }

  const maliput::math::Vector2 g_result_{};
  const maliput::math::Vector2 g_dot_result_{};
  const double heading_result_{};
  const double heading_dot_result_{};
  const double d_g_inverse_result_{};
  const double arc_length_result_{};
  const double linear_tolerance_result_{};
  const double p0_result_{};
  const double p1_result_{};
  const bool is_g1_contiguous_result_{};
};

}  // namespace test
}  // namespace road_curve
}  // namespace malidrive

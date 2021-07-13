// Copyright 2020 Toyota Research Institute
#pragma once

namespace malidrive {
namespace constants {

/// maliput::api::RoadGeometry properties.
/// Empirically good enough configurations based on internal ODRM settings and
/// available testing geometries.
static constexpr double kLinearTolerance{5e-2};   // [m]
static constexpr double kAngularTolerance{1e-2};  // [rad]
static constexpr double kScaleLength{1.};         // [m]
/// Stricter tolerances.
static constexpr double kStrictLinearTolerance{1e-12};   // [m]
static constexpr double kStrictAngularTolerance{1e-12};  // [rad]

/// Builder - SpeedLimitRule arbitrary defaults.
static constexpr double kSpeedTolerance{1e-4};             // [m/s]
static constexpr double kDefaultMinSpeedLimit{0.};         // [m/s]
static constexpr double kDefaultMaxSpeedLimit{40. / 3.6};  // [m/s] --> 40km/h
/// Number of trials the Builder will run with different tolerances.
/// Using default linear tolerance value it is expected that the builder tries with
/// tolerances in a range between 0.05 and 0.49. (`kLinearTolerance` and @f$ kLinearTolerance*1.1^24 @f$)
static constexpr int kMaxToleranceSelectionRounds{24};

/// TODO(#701): Remove the following two constants once #701 is merged.
static constexpr double kExplorationRadius{1.};  // [m]
static constexpr int kNumIterations{200};        // [u]

}  // namespace constants
}  // namespace malidrive

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
/// Base linear tolerance used as minimum value in the linear tolerance range
/// when tolerance selection mechanism is enabled and no linear_tolerance parameter
/// is passed to the builder.
static constexpr double kBaseLinearTolerance{1e-6};  // [m]
/// Multiplier used to increase the tolerance by the Builder.
static constexpr double kToleranceStepMultiplier{1.1};

/// Stricter tolerances.
static constexpr double kStrictLinearTolerance{1e-12};   // [m]
static constexpr double kStrictAngularTolerance{1e-12};  // [rad]

/// Builder - SpeedLimitRule arbitrary defaults.
static constexpr double kSpeedTolerance{1e-4};             // [m/s]
static constexpr double kDefaultMinSpeedLimit{0.};         // [m/s]
static constexpr double kDefaultMaxSpeedLimit{40. / 3.6};  // [m/s] --> 40km/h

}  // namespace constants
}  // namespace malidrive

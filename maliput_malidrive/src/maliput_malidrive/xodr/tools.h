// Copyright 2020 Toyota Research Institute
#pragma once

#include <maliput/math/vector.h>
#include "maliput_malidrive/xodr/geometry.h"

namespace malidrive {
namespace xodr {

/// Calculates the Euclidean distance between the endpoint of `lhs` and the startpoint of `rhs`.
/// @param lhs Geometry that connects with its endpoint to next Geometry.
/// @param rhs Geometry that connects with its startpoint to previous Geometry.
/// @returns The Euclidean distance.
/// @throw maliput::common::assertion_error When `lhs.type` is neither Geometry::Type::kLine nor Geometry::Type::kArc.
double GetDistanceBetweenGeometries(const Geometry& lhs, const Geometry& rhs);

/// Computes the distance between the images of `lhs` and `rhs` polynomials.
/// @param lhs Left hand polynomial to evaluate on `rhs.s_0`.
/// @param rhs Right hand polynomial to evaluate on `rhs.s_0`.
/// @returns The absolute difference between the two polynomial images.
/// @tparam XodrFunction One of the Xodr cubic polynomial images:
///         - ElevationProfile::Elevation
template <class XodrFunction>
double GetDistanceBetweenFunctions(const XodrFunction& lhs, const XodrFunction& rhs);

/// Calculates the position of the geometry that starts in
/// `geometry.start_point` and extends `length` distance.
/// @param geometry A Geometry to evaluate its endpoint. Its type must be a valid Geometry::Type.
/// @param length A positive distance to evaluate `geometry`'s endpoint.
/// @returns The position of the endpoint.
/// @throw maliput::common::assertion_error When `tolerance` is not positive.
/// @throw maliput::common::assertion_error When `geometry.type` is neither Geometry::Type::kLine nor
/// Geometry::Type::kArc.
maliput::math::Vector2 ComputeEndpointWithNewLength(const Geometry& geometry, double length);

}  // namespace xodr
}  // namespace malidrive

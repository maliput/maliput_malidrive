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

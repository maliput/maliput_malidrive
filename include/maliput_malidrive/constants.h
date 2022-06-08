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

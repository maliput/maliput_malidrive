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

#include "maliput_malidrive/xodr/db_manager.h"

namespace malidrive {
namespace builder {

static constexpr double kMinLinearTolerance{1e-3};

/// Determines the runtime tolerance a RoadGeometry should be set from the
/// characteristics in a XODR map.
///
/// The selection is based on two constraints:
/// - tolerance must be bigger than the largest gap --> hard constraint
/// - tolerance is desired to be smaller than the smallest geometry piece -->
///   soft constraint.
///
/// The hard constraint is required because that would make the builder or the
/// RoadGeometry itself to throw because of not meeting the G1 constraint.
/// The soft constraint is desired when the previous cannot be used. However,
/// because synthesized maps might have relatively small road descriptions which
/// are several order of magnitude less than the road dimension the tolerance
/// is saturated between kMinLinearTolerance and constants::kLinearTolerance.
///
/// The tolerance assignment is done as follows:
/// - The maximum between the largest geometry gap and elevation gap is
///   selected and inflated a 50% to avoid any tolerance issue. The value is
///   lower bound saturated with kMinLinearTolerance. If any of the magnitudes
///   is not valid the other is used. When none are used, the soft constraint is
///   used.
/// - The minimum between the geometries and the lane section extension is used.
///   The value is reduced to a half to avoid any numerical discrepancy and
///   saturated between kMinLinearTolerance and constants::kLinearTolerance.
///
/// @param xodr_manager A pointer to a xodr::DBManager with a XODR map loaded.
/// @return A proposed linear tolerance.
/// @throws maliput::common::assertion_error When `xodr_manager` is nullptr.
double DetermineRoadGeometryLinearTolerance(const xodr::DBManager* xodr_manager);

/// Determines the runtime angular a RoadGeometry should be set from the
/// characteristics in a XODR map.
///
/// @param xodr_manager A pointer to a xodr::DBManager with a XODR map loaded.
/// @return The proposed angular tolerance.
/// @throws maliput::common::assertion_error When `xodr_manager` is nullptr.
// TODO(#571): Implement constraints to determine the angular tolerance.
double DetermineRoadGeometryAngularTolerance(const xodr::DBManager* xodr_manager);

/// Determines the runtime scale length a RoadGeometry should be set from the
/// characteristics in a XODR map.
///
/// @param xodr_manager A pointer to a xodr::DBManager with a XODR map loaded.
/// @param linear_tolerance The proposed linear tolerance. It must be positive.
/// @param angular_tolerance The proposed angular tolerance. It must be positive.
/// @return The proposed scale length.
/// @throws maliput::common::assertion_error When `xodr_manager` is nullptr.
// TODO(#571): Implement constraints to determine the scale length.
double DetermineRoadGeometryScaleLength(const xodr::DBManager* xodr_manager, double linear_tolerance,
                                        double angular_tolerance);

}  // namespace builder
}  // namespace malidrive

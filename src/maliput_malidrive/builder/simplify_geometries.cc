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
#include "maliput_malidrive/builder/simplify_geometries.h"

#include <algorithm>

namespace malidrive {
namespace builder {
namespace {

// Finds in `geometries_to_simplify` the struct that holds `i` within
// xodr::DBManager::XodrGeometriesToSimplify::geometries.
//
// @param geometries_to_simplify A vector of xodr::DBManager::XodrGeometriesToSimplify
//        that belong to the same RoadHeader::Id.
// @param i The index of the geometry to identify.
// @return An optional wrapping the xodr::DBManager::XodrGeometriesToSimplify
//         that contains `i`. Otherwise, std::nullopt.
std::optional<xodr::DBManager::XodrGeometriesToSimplify> FindGeometryIndexIn(
    const std::vector<xodr::DBManager::XodrGeometriesToSimplify>& geometries_to_simplify, int i) {
  const auto it =
      std::find_if(geometries_to_simplify.begin(), geometries_to_simplify.end(),
                   [i](const xodr::DBManager::XodrGeometriesToSimplify& geometry_to_simplify) {
                     return std::find(geometry_to_simplify.geometries.begin(), geometry_to_simplify.geometries.end(),
                                      i) != geometry_to_simplify.geometries.end();
                   });
  return it != geometries_to_simplify.end() ? std::make_optional<>(*it) : std::nullopt;
}

}  // namespace

std::vector<xodr::Geometry> SimplifyGeometries(
    const std::vector<xodr::Geometry>& geometries,
    const std::vector<xodr::DBManager::XodrGeometriesToSimplify>& geometries_to_simplify) {
  if (geometries_to_simplify.empty()) {
    return geometries;
  }
  std::vector<xodr::Geometry> simplified_geometries;
  int i = 0;
  while (i < static_cast<int>(geometries.size())) {
    const std::optional<xodr::DBManager::XodrGeometriesToSimplify> geometry_to_simplify =
        FindGeometryIndexIn(geometries_to_simplify, i);
    if (geometry_to_simplify.has_value()) {
      xodr::Geometry simple_geometry = geometries[i];
      // Computes the total length of the simplified geometry.
      simple_geometry.length = 0.;
      for (int j : geometry_to_simplify->geometries) {
        simple_geometry.length += geometries[j].length;
      }
      i = geometry_to_simplify->geometries.back() + 1;
      simplified_geometries.push_back(simple_geometry);
    } else {
      simplified_geometries.push_back(geometries[i]);
      ++i;
    }
  }
  return simplified_geometries;
}

}  // namespace builder
}  // namespace malidrive

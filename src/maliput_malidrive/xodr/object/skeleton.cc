// BSD 3-Clause License
//
// Copyright (c) 2025, Woven Planet. All rights reserved.
// Copyright (c) 2025, Toyota Research Institute. All rights reserved.
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
#include "maliput_malidrive/xodr/object/skeleton.h"

namespace malidrive {
namespace xodr {
namespace object {

bool VertexRoad::operator==(const VertexRoad& other) const {
  return dz == other.dz && id == other.id && intersection_point == other.intersection_point && radius == other.radius &&
         s == other.s && t == other.t;
}

bool VertexRoad::operator!=(const VertexRoad& other) const { return !(*this == other); }

bool VertexLocal::operator==(const VertexLocal& other) const {
  return id == other.id && intersection_point == other.intersection_point && u == other.u && v == other.v &&
         z == other.z && radius == other.radius;
}

bool VertexLocal::operator!=(const VertexLocal& other) const { return !(*this == other); }

bool Polyline::operator==(const Polyline& other) const {
  return vertex_road == other.vertex_road && vertex_local == other.vertex_local && id == other.id;
}

bool Polyline::operator!=(const Polyline& other) const { return !(*this == other); }

bool Skeleton::operator==(const Skeleton& other) const { return polylines == other.polylines; }

bool Skeleton::operator!=(const Skeleton& other) const { return !(*this == other); }

}  // namespace object
}  // namespace xodr
}  // namespace malidrive

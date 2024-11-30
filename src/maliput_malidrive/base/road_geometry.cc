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
#include "maliput_malidrive/base/road_geometry.h"

#include <algorithm>

#include <maliput/geometry_base/brute_force_find_road_positions_strategy.h>
#include <maliput/geometry_base/filter_positions.h>

#include "maliput_malidrive/constants.h"

namespace malidrive {

void RoadGeometry::AddRoadCharacteristics(const xodr::RoadHeader::Id& road_id,
                                          std::unique_ptr<road_curve::RoadCurve> road_curve,
                                          std::unique_ptr<road_curve::Function> reference_line_offset) {
  MALIDRIVE_THROW_UNLESS(road_curve != nullptr);
  MALIDRIVE_THROW_UNLESS(reference_line_offset != nullptr);
  if (road_characteristics_.find(road_id) != road_characteristics_.end()) {
    MALIDRIVE_THROW_MESSAGE(std::string("Duplicated Road characteristics for RoadID: ") + road_id.string());
  }
  road_characteristics_.emplace(road_id, RoadCharacteristics{std::move(road_curve), std::move(reference_line_offset)});
}

const road_curve::RoadCurve* RoadGeometry::GetRoadCurve(const xodr::RoadHeader::Id& road_id) const {
  if (road_characteristics_.find(road_id) == road_characteristics_.end()) {
    MALIDRIVE_THROW_MESSAGE(std::string("There is no RoadCurve for RoadID: ") + road_id.string());
  }
  return road_characteristics_.at(road_id).road_curve.get();
}

const road_curve::Function* RoadGeometry::GetReferenceLineOffset(const xodr::RoadHeader::Id& road_id) const {
  if (road_characteristics_.find(road_id) == road_characteristics_.end()) {
    MALIDRIVE_THROW_MESSAGE(std::string("There is no reference line offset function for RoadID: ") + road_id.string());
  }
  return road_characteristics_.at(road_id).reference_line_offset.get();
}

}  // namespace malidrive

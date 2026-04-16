// BSD 3-Clause License
//
// Copyright (c) 2026, Woven by Toyota. All rights reserved.
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
#include "maliput_malidrive/builder/road_object_book_builder.h"

#include <utility>

#include <maliput/base/road_object_book.h>
#include <maliput/common/logger.h>

#include "maliput_malidrive/base/road_geometry.h"
#include "maliput_malidrive/builder/road_object_builder.h"
#include "maliput_malidrive/common/macros.h"

namespace malidrive {
namespace builder {

RoadObjectBookBuilder::RoadObjectBookBuilder(const maliput::api::RoadGeometry* road_geometry)
    : road_geometry_(road_geometry) {
  MALIDRIVE_VALIDATE(road_geometry_ != nullptr, maliput::common::assertion_error, "road_geometry must not be nullptr.");
}

std::unique_ptr<maliput::api::objects::RoadObjectBook> RoadObjectBookBuilder::operator()() const {
  maliput::log()->trace("Building RoadObjectBook...");

  auto book = std::make_unique<maliput::RoadObjectBook>();

  const auto* mali_rg = dynamic_cast<const malidrive::RoadGeometry*>(road_geometry_);
  MALIDRIVE_VALIDATE(mali_rg != nullptr, maliput::common::assertion_error,
                     "RoadGeometry cannot be cast to malidrive::RoadGeometry.");

  for (const auto& [road_id, road_header] : mali_rg->get_manager()->GetRoadHeaders()) {
    if (!road_header.objects.has_value()) {
      continue;
    }
    for (const auto& object : road_header.objects->objects) {
      try {
        auto road_object = RoadObjectBuilder(object, road_id, road_geometry_)();
        if (road_object) {
          book->AddRoadObject(std::move(road_object));
        }
      } catch (const std::exception& e) {
        maliput::log()->warn("RoadObjectBookBuilder: failed to build RoadObject id='", object.id.string(),
                             "' on road '", road_id.string(), "': ", e.what(), ". Skipping.");
      }
    }
  }

  maliput::log()->trace("Built RoadObjectBook.");
  return book;
}

}  // namespace builder
}  // namespace malidrive

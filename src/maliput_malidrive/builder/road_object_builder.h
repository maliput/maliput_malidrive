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
#pragma once

#include <memory>

#include <maliput/api/objects/road_object.h>
#include <maliput/api/road_geometry.h>

#include "maliput_malidrive/xodr/object/object.h"
#include "maliput_malidrive/xodr/road_header.h"

namespace malidrive {
namespace builder {

/// Builds a single @ref maliput::api::objects::RoadObject from an XODR
/// @ref xodr::object::Object.
///
/// The builder follows the functor pattern: construct it with all required
/// inputs and call @ref operator()() to produce the result.
class RoadObjectBuilder {
 public:
  /// Constructs a RoadObjectBuilder.
  ///
  /// @param object The XODR object to build a RoadObject from.
  /// @param road_id The XODR road's ID the @p object belongs to.
  /// @param road_geometry Pointer to the road geometry. Must not be nullptr.
  /// @throws std::invalid_argument if @p road_geometry is nullptr.
  RoadObjectBuilder(const xodr::object::Object& object, const xodr::RoadHeader::Id& road_id,
                    const maliput::api::RoadGeometry* road_geometry);

  /// Builds and returns the @ref maliput::api::objects::RoadObject.
  ///
  /// @returns A unique_ptr to the constructed RoadObject.
  std::unique_ptr<maliput::api::objects::RoadObject> operator()() const;

 private:
  const xodr::object::Object& object_;
  const xodr::RoadHeader::Id& road_id_;
  const maliput::api::RoadGeometry* road_geometry_;
};

}  // namespace builder
}  // namespace malidrive

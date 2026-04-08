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

#include <maliput/api/objects/road_object_book.h>
#include <maliput/api/road_geometry.h>

#include "maliput_malidrive/common/macros.h"

namespace malidrive {
namespace builder {

/// Builds a @ref maliput::api::objects::RoadObjectBook populated with
/// @ref maliput::api::objects::RoadObject instances derived from the XODR
/// `<object>` elements across all roads.
///
/// Follows the functor pattern: construct with the required inputs and call
/// @ref operator()() to produce the book.
class RoadObjectBookBuilder {
 public:
  MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN(RoadObjectBookBuilder)

  /// Constructs a RoadObjectBookBuilder.
  ///
  /// @param road_geometry Pointer to the built RoadGeometry. Must not be nullptr.
  /// @throws maliput::common::assertion_error When @p road_geometry is nullptr.
  explicit RoadObjectBookBuilder(const maliput::api::RoadGeometry* road_geometry);

  RoadObjectBookBuilder() = delete;

  /// Builds and returns the populated RoadObjectBook.
  ///
  /// @returns A unique_ptr to the constructed RoadObjectBook.
  std::unique_ptr<maliput::api::objects::RoadObjectBook> operator()() const;

 private:
  const maliput::api::RoadGeometry* road_geometry_;
};

}  // namespace builder
}  // namespace malidrive

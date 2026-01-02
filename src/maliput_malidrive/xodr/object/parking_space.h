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
#pragma once

#include <map>
#include <optional>
#include <string>

#include "maliput_malidrive/common/macros.h"

namespace malidrive {
namespace xodr {
namespace object {

/// Holds the values of a XODR Object parkingSpace element.
struct ParkingSpace {
  /// Convenient constants that hold the tag names in the XODR element attributes description.
  static constexpr const char* kParkingSpaceTag = "parkingSpace";
  static constexpr const char* kAccess = "access";
  static constexpr const char* kRestrictions = "restrictions";

  enum class Access { kAll, kBus, kCar, kElectric, kHandicapped, kResidents, kTruck, kWomen };

  /// Access definitions for the parking space. Parking spaces tagged with "women" and "handicapped" are vehicles of
  /// type car.
  Access access{};
  /// Free text, depending on application.
  std::optional<std::string> restrictions{std::nullopt};

  /// Matches string with an Access.
  /// @param access Is a Access.
  /// @returns A string that matches with `access`.
  static std::string access_to_str(Access access);

  /// Matches Access with a string.
  /// @param access Is a string.
  /// @returns A Access that matches with `Access`.
  /// @throw maliput::common::assertion_error When `access` doesn't match with a Access.
  static Access str_to_access(const std::string& access);

  bool operator==(const ParkingSpace& other) const;
  bool operator!=(const ParkingSpace& other) const;
};

}  // namespace object
}  // namespace xodr
}  // namespace malidrive

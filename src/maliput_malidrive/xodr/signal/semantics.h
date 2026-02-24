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

#include <optional>
#include <string>
#include <vector>

namespace malidrive {
namespace xodr {
namespace signal {

/// Enumeration mapped from e_signals_semantics_speed
enum class SemanticsSpeed {
  kUnknown = 0,
  kMaximum,
  kMaximumEnd,
  kMinimum,
  kMinimumEnd,
  kRecommended,
  kRecommendedEnd,
  Zone,
  ZoneEnd,
};

/// Enumeration mapped from e_unitSpeed
enum class UnitSpeed {
  kUnknown = 0,
  kMS,   // "m/s"
  kMph,  // "mph"
  kKmh   // "km/h"
};

/// Enumeration mapped from e_signals_semantics_lane
enum class SemanticsLane {
  kUnknown = 0,
  kNoOvertakeCarsEnd,
  kNoOvertakeCars,
  kNoOvertakeTrucksEnd,
  kNoOvertakeTrucks,
  kPriorityOverOncoming,
  kRoundabout,
  kYieldForOncoming,
};

/// Enumeration mapped from e_signals_semantics_priority
enum class SemanticsPriority {
  kUnknown = 0,
  k4Way,
  kKeepClearLine,
  kNoParkingLine,
  kNoTurnOnRed,
  kPriorityRoadEnd,
  kPriorityRoad,
  kPriorityToTheRightRule,
  kStopLine,
  kStop,
  kTrafficLight,
  kTurnOnRedAllowed,
  kWaitingLine,
  kYield,
};

/// Enumeration mapped from e_signals_semantics_supplementaryTime
enum class SemanticsSupplementaryTime {
  kUnknown = 0,
  kTime,
  kDay,
};

/// Enumeration mapped from e_signals_semantics_supplementaryDistance
enum class SemanticsSupplementaryDistance {
  kUnknown = 0,
  kFor,
  kIn,
};

/// Enumeration mapped from e_unitDistance
enum class UnitDistance {
  kUnknown = 0,
  kM,    // "m"
  kKm,   // "km"
  kFt,   // "ft"
  kMile  // "mile"
};

/// Enumeration mapped from e_signals_semantics_supplementaryEnvironment
enum class SemanticsSupplementaryEnvironment {
  kUnknown = 0,
  kRain,
  kSnow,
  kFog,
};

/// Specifies speed regulations.
struct Speed {
  static constexpr const char* kSpeedTag = "speed";
  static constexpr const char* kType = "type";
  static constexpr const char* kUnit = "unit";
  static constexpr const char* kValue = "value";

  bool operator==(const Speed& other) const;
  bool operator!=(const Speed& other) const;

  SemanticsSpeed type{SemanticsSpeed::kUnknown};
  UnitSpeed unit{UnitSpeed::kUnknown};
  double value{};
};

/// Specifies lane regulations.
struct Lane {
  static constexpr const char* kLaneTag = "lane";
  static constexpr const char* kType = "type";

  bool operator==(const Lane& other) const;
  bool operator!=(const Lane& other) const;

  SemanticsLane type{SemanticsLane::kUnknown};
};

/// Specifies priority regulations.
struct Priority {
  static constexpr const char* kPriorityTag = "priority";
  static constexpr const char* kType = "type";

  bool operator==(const Priority& other) const;
  bool operator!=(const Priority& other) const;

  SemanticsPriority type{SemanticsPriority::kUnknown};
};

/// Specifies that certain types of traffic participants are not allowed to enter.
struct Prohibited {
  static constexpr const char* kProhibitedTag = "prohibited";

  bool operator==(const Prohibited& other) const;
  bool operator!=(const Prohibited& other) const;
};

/// Specifies warnings for traffic participant.
struct Warning {
  static constexpr const char* kWarningTag = "warning";

  bool operator==(const Warning& other) const;
  bool operator!=(const Warning& other) const;
};

/// Specifies routing information.
struct Routing {
  static constexpr const char* kRoutingTag = "routing";

  bool operator==(const Routing& other) const;
  bool operator!=(const Routing& other) const;
};

/// Specifies the name of a street.
struct StreetName {
  static constexpr const char* kStreetNameTag = "streetname";

  bool operator==(const StreetName& other) const;
  bool operator!=(const StreetName& other) const;
};

/// Specifies parking regulations.
struct Parking {
  static constexpr const char* kParkingTag = "parking";

  bool operator==(const Parking& other) const;
  bool operator!=(const Parking& other) const;
};

/// Specifies tourist information.
struct Tourist {
  static constexpr const char* kTouristTag = "tourist";

  bool operator==(const Tourist& other) const;
  bool operator!=(const Tourist& other) const;
};

/// Specifies supplementary time.
struct SupplementaryTime {
  static constexpr const char* kSupplementaryTimeTag = "supplementaryTime";
  static constexpr const char* kType = "type";
  static constexpr const char* kValue = "value";

  bool operator==(const SupplementaryTime& other) const;
  bool operator!=(const SupplementaryTime& other) const;

  SemanticsSupplementaryTime type{SemanticsSupplementaryTime::kUnknown};
  double value{};
};

/// Specifies the type of the traffic participant an exception is made for.
struct SupplementaryAllows {
  static constexpr const char* kSupplementaryAllowsTag = "supplementaryAllows";

  bool operator==(const SupplementaryAllows& other) const;
  bool operator!=(const SupplementaryAllows& other) const;
};

/// Specifies supplementary prohibitions.
struct SupplementaryProhibits {
  static constexpr const char* kSupplementaryProhibitsTag = "supplementaryProhibits";

  bool operator==(const SupplementaryProhibits& other) const;
  bool operator!=(const SupplementaryProhibits& other) const;
};

/// Specifies the distance after a sign becomes valid or the range in which the sign is valid.
struct SupplementaryDistance {
  static constexpr const char* kSupplementaryDistanceTag = "supplementaryDistance";
  static constexpr const char* kType = "type";
  static constexpr const char* kUnit = "unit";
  static constexpr const char* kValue = "value";

  bool operator==(const SupplementaryDistance& other) const;
  bool operator!=(const SupplementaryDistance& other) const;

  SemanticsSupplementaryDistance type{SemanticsSupplementaryDistance::kUnknown};
  UnitDistance unit{UnitDistance::kUnknown};
  double value{};
};

/// Specifies under which environmental conditions a sign is valid.
struct SupplementaryEnvironment {
  static constexpr const char* kSupplementaryEnvironmentTag = "supplementaryEnvironment";
  static constexpr const char* kType = "type";

  bool operator==(const SupplementaryEnvironment& other) const;
  bool operator!=(const SupplementaryEnvironment& other) const;

  SemanticsSupplementaryEnvironment type{SemanticsSupplementaryEnvironment::kUnknown};
};

/// Specifies supplementary explanatory information.
struct SupplementaryExplanatory {
  static constexpr const char* kSupplementaryExplanatoryTag = "supplementaryExplanatory";

  bool operator==(const SupplementaryExplanatory& other) const;
  bool operator!=(const SupplementaryExplanatory& other) const;
};

/// Holds the semantics elements of a XODR Signal.
struct Semantics {
  static constexpr const char* kSemanticsTag = "semantics";

  bool operator==(const Semantics& other) const;
  bool operator!=(const Semantics& other) const;

  std::vector<Speed> speeds{};
  std::vector<Lane> lanes{};
  std::vector<Priority> priorities{};
  std::vector<Prohibited> prohibiteds{};
  std::vector<Warning> warnings{};
  std::vector<Routing> routings{};
  std::vector<StreetName> street_names{};
  std::vector<Parking> parkings{};
  std::vector<Tourist> tourists{};
  std::vector<SupplementaryTime> supplementary_times{};
  std::vector<SupplementaryAllows> supplementary_allows{};
  std::vector<SupplementaryProhibits> supplementary_prohibits{};
  std::vector<SupplementaryDistance> supplementary_distances{};
  std::vector<SupplementaryEnvironment> supplementary_environments{};
  std::vector<SupplementaryExplanatory> supplementary_explanatories{};
};

}  // namespace signal
}  // namespace xodr
}  // namespace malidrive

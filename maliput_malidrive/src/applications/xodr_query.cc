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

// Queries a XODR map.
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include <gflags/gflags.h>
#include <maliput/common/logger.h>

#include "applications/log_level_flag.h"
#include "maliput_malidrive/xodr/db_manager.h"

namespace malidrive {
namespace applications {
namespace xodr {
namespace {
// @{ CLI Arguments
DEFINE_double(tolerance, 1e-3, "Tolerance to validate continuity in piecewise defined geometries.");
DEFINE_bool(allow_schema_errors, false, "If true, the XODR parser will attempt to work around XODR schema violations.");
DEFINE_bool(allow_semantic_errors, false,
            "If true, the XODR parser will attempt to work around XODR semantic violations.");
MALIPUT_MALIDRIVE_APPLICATION_DEFINE_LOG_LEVEL_FLAG();
// @}

struct Command {
  std::string name{"default"};
  std::string usage{"default"};
  // Descriptions are represented as a sequence of lines to
  // ease formatting (e.g. indentation, line wraps) in the
  // interest of readability.
  std::vector<std::string> description{{"default"}};
  int num_arguments{0};
};

// @return A map of command name to usage message.
const std::map<const std::string, const Command> CommandsUsage() {
  return {
      {"GetHeader", {"GetHeader", "<xodr_file> GetHeader", {"Prints the XODR Header."}, 2}},
      {"FindRoad", {"FindRoad", "<xodr_file> FindRoad RoadId", {"Obtains the XODR Road whose ID is RoadId."}, 3}},
      {"FindJunction",
       {"FindJunction",
        "<xodr_file> FindJunction JunctionId",
        {"Obtains the XODR Junction hose ID is JunctionId."},
        3}},
      {"FindShortestGeometry",
       {"FindShortestGeometry", "<xodr_file> FindShortestGeometry", {"Obtains the shortest geometry of the XODR."}, 2}},
      {"FindLargestGeometry",
       {"FindLargestGeometry", "<xodr_file> FindLargestGeometry", {"Obtains the largest geometry of the XODR."}, 2}},
      {"FindShortestLaneSection",
       {"FindShortestLaneSection",
        "<xodr_file> FindShortestLaneSection",
        {"Obtains the shortest laneSection of the XODR."},
        2}},
      {"FindLargestLaneSection",
       {"FindLargestLaneSection",
        "<xodr_file> FindLargestLaneSection",
        {"Obtains the largest laneSection of the XODR."},
        2}},
      {"FindShortestGap",
       {"FindShortestGap", "<xodr_file> FindShortestGap", {"Obtains the shortest gap of the XODR."}, 2}},
      {"FindLargestGap", {"FindLargestGap", "<xodr_file> FindLargestGap", {"Obtains the largest gap of the XODR."}, 2}},
      {"FindShortestElevationGap",
       {"FindShortestElevationGap",
        "<xodr_file> FindShortestElevationGap",
        {"Obtains the shortest gap in elevation functions of the XODR."},
        2}},
      {"FindLargestElevationGap",
       {"FindLargestElevationGap",
        "<xodr_file> FindLargestElevationGap",
        {"Obtains the largest gap in elevation functions of the XODR."},
        2}},
      {"FindShortestSuperelevationGap",
       {"FindShortestSuperelevationGap",
        "<xodr_file> FindShortestSuperelevationGap",
        {"Obtains the shortest superelevation gap of the XODR."},
        2}},
      {"FindLargestSuperelevationGap",
       {"FindLargestSuperelevationGap",
        "<xodr_file> FindLargestSuperelevationGap",
        {"Obtains the largest superelevation gap of the XODR."},
        2}},
      {"GetGeometriesToSimplify",
       {"GetGeometriesToSimplify",
        "<xodr_file> GetGeometriesToSimplify tolerance",
        {"Retrieves a list of geometries that can be simplified into simpler geometry descriptions and the type of "
         "geometry that would do it."},
        3}},
      {"GetGeometries",
       {"GetGeometries",
        "<xodr_file> GetGeometries RoadId",
        {"Retrieves a list of the geometries of the correspondant road."},
        3}},
  };
}

// @return A string with the usage message.
std::string GetUsageMessage() {
  std::stringstream ss;
  ss << "CLI for easy XODR map introspection querying" << std::endl << std::endl;
  ss << "  Supported commands:" << std::endl;
  const std::map<const std::string, const Command> command_usage = CommandsUsage();
  for (auto it = command_usage.begin(); it != command_usage.end(); ++it) {
    ss << "    " << it->second.usage << std::endl << std::endl;
    for (const std::string& line : it->second.description) {
      ss << "        " << line << std::endl;
    }
    ss << std::endl;
  }
  return ss.str();
}

/// Query and logs results to DBManager minimizing the overhead of getting the
/// right calls / asserting conditions.
class DBManagerQuery {
 public:
  MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN(DBManagerQuery)

  /// Constructs a DBManagerQuery.
  ///
  /// @param out A pointer to an output stream where results will be logged.
  /// It must not be nullptr.
  /// @param db_manager A pointer to a malidrive::xodr::DBManager. It must not
  /// be nullptr.
  /// @throws maliput::common::assertion_error When `out` or `db_manager` are
  /// nullptr.
  DBManagerQuery(std::ostream* out, malidrive::xodr::DBManager* db_manager) : out_(out), db_manager_(db_manager) {
    MALIDRIVE_THROW_UNLESS(out_ != nullptr);
    MALIDRIVE_THROW_UNLESS(db_manager_ != nullptr);
  }

  /// Prints the header of the XODR map.
  void GetHeader() const {
    const malidrive::xodr::Header& header = db_manager_->GetXodrHeader();
    (*out_) << "Header:" << header << std::endl;
  }

  /// Finds a malidrive::xodr::RoadHeader by its `road_id`.
  ///
  /// @param road_id The malidrive::xodr::RoadHeader::Id of the road to look
  /// for.
  void FindRoad(const malidrive::xodr::RoadHeader::Id& road_id) const {
    const std::map<malidrive::xodr::RoadHeader::Id, malidrive::xodr::RoadHeader>& road_headers =
        db_manager_->GetRoadHeaders();
    const auto road_it = road_headers.find(road_id);
    if (road_it == road_headers.end()) {
      (*out_) << "RoadHaderId: " << road_id.string() << " could not be found." << std::endl;
    } else {
      (*out_) << "RoadHaderId: " << road_id.string() << ", RoadHeader: " << road_it->second << std::endl;
    }
  }

  /// Finds a malidrive::xodr::Junction by its `junction_id`.
  ///
  /// @param junction_id The malidrive::xodr::Junction::Id of the junction to
  /// look for.
  void FindJunction(const malidrive::xodr::Junction::Id& junction_id) const {
    const std::unordered_map<malidrive::xodr::Junction::Id, malidrive::xodr::Junction>& junctions =
        db_manager_->GetJunctions();
    const auto junction_it = junctions.find(junction_id);
    if (junction_it == junctions.end()) {
      (*out_) << "JunctionId: " << junction_id.string() << " could not be found." << std::endl;
    } else {
      (*out_) << "JunctionId: " << junction_id.string() << ", Junction: " << junction_it->second << std::endl;
    }
  }

  /// Finds the shortest or the largest Geometry in the XODR.
  ///
  /// @param shortest If 'true', the shortest Geometry will be found, otherwise the largest one.
  void FindGeometryByLength(bool shortest) const {
    const auto& geometry_data = shortest ? db_manager_->GetShortestGeometry() : db_manager_->GetLargestGeometry();
    (*out_) << (shortest ? "Shortest" : "Largest") << " Geometry in the XODR: " << geometry_data.length << "\n";
    (*out_) << "Located at RoadHaderId: " << geometry_data.road_header_id.string()
            << ", Geometry Index: " << geometry_data.geometry_index << std::endl;
  }

  /// Finds the shortest or the largest LaneSection in the XODR.
  ///
  /// @param shortest If 'true', the shortest LaneSection will be found, otherwise the largest one.
  void FindLaneSectionByLength(bool shortest) const {
    const auto& lane_section_data =
        shortest ? db_manager_->GetShortestLaneSection() : db_manager_->GetLargestLaneSection();
    (*out_) << (shortest ? "Shortest" : "Largest") << " LaneSection in the XODR: " << lane_section_data.length << "\n";
    (*out_) << "Located at RoadHaderId: " << lane_section_data.road_header_id.string()
            << ", LaneSection Index: " << lane_section_data.lane_section_index << std::endl;
  }

  /// Finds the shortest or the largest gap between Geometries in the XODR.
  ///
  /// @param shortest If 'true', the shortest gap will be found, otherwise the largest one.
  void FindGap(bool shortest) const {
    const auto& geometry_data = shortest ? db_manager_->GetShortestGap() : db_manager_->GetLargestGap();
    (*out_) << (shortest ? "Shortest" : "Largest") << " Gap between Geometries in the XODR: " << geometry_data.distance
            << "\n";
    (*out_) << "Located at RoadHaderId: " << geometry_data.road_header_id.string() << ", Geometry Indexes: ["
            << geometry_data.geometry_index_pair.first;
    (*out_) << "," << geometry_data.geometry_index_pair.second << "]" << std::endl;
  }

  /// Finds the shortest or the largest gap between elevation functions in the XODR.
  ///
  /// @param shortest If 'true', the shortest gap will be found, otherwise the largest one.
  void FindElevationGap(bool shortest) const {
    const auto& geometry_data =
        shortest ? db_manager_->GetShortestElevationGap() : db_manager_->GetLargestElevationGap();
    (*out_) << (shortest ? "Shortest" : "Largest")
            << " Gap between elevation functions in the XODR: " << geometry_data.distance << "\n";
    (*out_) << "Located at RoadHaderId: " << geometry_data.road_header_id.string() << ", elevation indexes: ["
            << geometry_data.function_index_pair.first;
    (*out_) << "," << geometry_data.function_index_pair.second << "]" << std::endl;
  }

  /// Finds the shortest or the largest gap between superelevations in the XODR.
  ///
  /// @param shortest If 'true', the shortest gap will be found, otherwise the largest one.
  void FindSuperelevationGap(bool shortest) const {
    const auto& geometry_data =
        shortest ? db_manager_->GetShortestSuperelevationGap() : db_manager_->GetLargestSuperelevationGap();
    (*out_) << (shortest ? "Shortest" : "Largest")
            << " Gap between superelevations in the XODR: " << geometry_data.distance << "\n";
    (*out_) << "Located at RoadHaderId: " << geometry_data.road_header_id.string() << ", superelevation indexes: ["
            << geometry_data.function_index_pair.first;
    (*out_) << "," << geometry_data.function_index_pair.second << "]" << std::endl;
  }

  /// Logs the geometries in <plainView> that can be simplified in a map.
  ///
  /// @param tolerance The threshold of the tolerance to use for geometry
  ///        simplification by length.
  void GetGeometriesToSimplify(double tolerance) const {
    const std::vector<malidrive::xodr::DBManager::XodrGeometriesToSimplify> result =
        db_manager_->GetGeometriesToSimplify(tolerance);
    if (result.empty()) {
      (*out_) << "There are no roads to simplify." << std::endl;
    } else {
      for (const auto& geometry_to_simplify : result) {
        (*out_) << "- RoadHeader::Id: " << geometry_to_simplify.road_header_id.string() << std::endl;
        (*out_) << "  Action: "
                << (geometry_to_simplify.action ==
                            malidrive::xodr::DBManager::XodrGeometriesToSimplify::Action::kReplaceByLine
                        ? "replace by lines."
                        : "replace by arcs.")
                << std::endl;
        (*out_) << "  Geometry Ids: {";
        for (const int geometry_index : geometry_to_simplify.geometries) {
          (*out_) << geometry_index << ", ";
        }
        (*out_) << "}" << std::endl;
      }
    }
  }

  /// Logs the all geometries in <plainView>
  /// @param road_id The malidrive::xodr::RoadHeader::Id of the road to look
  /// for the geometries.
  void GetGeometries(const malidrive::xodr::RoadHeader::Id& road_id) const {
    const auto road_headers = db_manager_->GetRoadHeaders();
    const auto road = road_headers.find(road_id);
    if (road == road_headers.cend()) {
      maliput::log()->error("RoadId: {} hasn't been found.", road_id);
      return;
    }
    (*out_).precision(std::numeric_limits<double>::digits10);
    (*out_) << "planView description for RoadId " << road_id.string() << ":\n";
    int i{1};
    for (const auto& geometry : road->second.reference_geometry.plan_view.geometries) {
      (*out_) << "\t" << std::to_string(i) << " - " << geometry;
      i++;
    }
  }

 private:
  mutable std::ostream* out_{};
  const malidrive::xodr::DBManager* db_manager_{};
};

// @{ Convert CLI arguments into query keys.
malidrive::xodr::Junction::Id JunctionIdFromCli(char** argv) {
  return malidrive::xodr::Junction::Id(std::string(*argv));
}

malidrive::xodr::RoadHeader::Id RoadHeaderIdFromCli(char** argv) {
  return malidrive::xodr::RoadHeader::Id(std::string(*argv));
}

double ToleranceFromCli(char** argv) { return std::stod(*argv); }
// @}

int Main(int argc, char** argv) {
  // Handles CLI arguments.
  gflags::SetUsageMessage(GetUsageMessage());
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  if (argc < 3 || CommandsUsage().find(argv[2]) == CommandsUsage().end()) {
    gflags::ShowUsageWithFlags(argv[0]);
    return 1;
  }
  const Command command = CommandsUsage().find(argv[2])->second;
  if (argc != command.num_arguments + 1) {
    gflags::ShowUsageWithFlags(argv[0]);
    return 1;
  }

  maliput::common::set_log_level(FLAGS_log_level);

  // Tries to load the XODR map.
  const std::string xodr_path = std::string(argv[1]);
  maliput::log()->trace("About to load: {}", xodr_path);
  maliput::log()->info("Parser: Allow schema errors: {}", (FLAGS_allow_schema_errors ? "enabled" : "disabled"));
  maliput::log()->info("Parser: Allow semantic errors: {}", (FLAGS_allow_semantic_errors ? "enabled" : "disabled"));
  auto db_manager = malidrive::xodr::LoadDataBaseFromFile(
      xodr_path, {FLAGS_tolerance, FLAGS_allow_schema_errors, FLAGS_allow_semantic_errors});
  if (db_manager == nullptr) {
    maliput::log()->error("Error loading the XODR file.");
    return 0;
  }
  maliput::log()->trace("XODR map was successfully loaded.");

  const constexpr bool kShortest{true};
  const constexpr bool kLargest{!kShortest};

  const DBManagerQuery query(&std::cout, db_manager.get());
  if (command.name.compare("GetHeader") == 0) {
    query.GetHeader();
  } else if (command.name.compare("FindRoad") == 0) {
    query.FindRoad(RoadHeaderIdFromCli(&(argv[3])));
  } else if (command.name.compare("FindJunction") == 0) {
    query.FindJunction(JunctionIdFromCli(&(argv[3])));
  } else if (command.name.compare("FindShortestGeometry") == 0) {
    query.FindGeometryByLength(kShortest);
  } else if (command.name.compare("FindLargestGeometry") == 0) {
    query.FindGeometryByLength(kLargest);
  } else if (command.name.compare("FindShortestLaneSection") == 0) {
    query.FindLaneSectionByLength(kShortest);
  } else if (command.name.compare("FindLargestLaneSection") == 0) {
    query.FindLaneSectionByLength(kLargest);
  } else if (command.name.compare("FindShortestGap") == 0) {
    query.FindGap(kShortest);
  } else if (command.name.compare("FindLargestGap") == 0) {
    query.FindGap(kLargest);
  } else if (command.name.compare("FindShortestElevationGap") == 0) {
    query.FindElevationGap(kShortest);
  } else if (command.name.compare("FindLargestElevationGap") == 0) {
    query.FindElevationGap(kLargest);
  } else if (command.name.compare("FindShortestSuperelevationGap") == 0) {
    query.FindSuperelevationGap(kShortest);
  } else if (command.name.compare("FindLargestSuperelevationGap") == 0) {
    query.FindSuperelevationGap(kLargest);
  } else if (command.name.compare("GetGeometriesToSimplify") == 0) {
    query.GetGeometriesToSimplify(ToleranceFromCli(&(argv[3])));
  } else if (command.name.compare("GetGeometries") == 0) {
    query.GetGeometries(RoadHeaderIdFromCli(&(argv[3])));
  }

  return 1;
}

}  // namespace
}  // namespace xodr
}  // namespace applications
}  // namespace malidrive

int main(int argc, char** argv) { return malidrive::applications::xodr::Main(argc, argv); }

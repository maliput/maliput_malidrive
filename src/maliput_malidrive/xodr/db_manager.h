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

#include <map>
#include <memory>
#include <unordered_map>
#include <utility>

#include <tinyxml2.h>

#include "maliput_malidrive/common/macros.h"
#include "maliput_malidrive/xodr/header.h"
#include "maliput_malidrive/xodr/junction.h"
#include "maliput_malidrive/xodr/parser_configuration.h"
#include "maliput_malidrive/xodr/road_header.h"

namespace malidrive {
namespace xodr {

/// Database Manager in charge of:
///  - loading the XODR file (XML format).
///  - parsing the XML nodes into known structures.
///  - providing XODR data.
///
/// The common workflow is to:
/// 1 - Construct a `DBManager` object using the provided methods.
///     `std::unique_ptr<xodr::DBManager> manager = xodr::LoadDataBaseFromFile(<path_to_xodr_file>)`
///     @see LoadDataBaseFromFile LoadDataBaseFromStr.
/// 2 - Query the data base, e.g.:
///     `manager->GetRoadHeaders();`
class DBManager {
 public:
  MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN(DBManager);

  /// Holds Geometry related information:
  struct XodrGeometryLengthData {
    /// Id of the Road that the Geometry belongs to.
    RoadHeader::Id road_header_id{"none"};
    /// Index of the Geometry within PlanView.
    int geometry_index{};
    /// Length of the Geometry.
    double length{};
  };

  /// Holds gap between cubic polynomials of a specific feature of the road such
  /// as elevation and superelevation.
  struct XodrGapBetweenFunctions {
    /// Id of the Road where the function belongs to.
    RoadHeader::Id road_header_id{"none"};
    /// Holds the two functions' indexes responsible for the gap. The gap is
    /// defined as the difference in the images of the two functions at the
    /// adjacent @f$ p @f$ value.
    std::pair<int, int> function_index_pair{};
    /// Value of the gap.
    double distance{};
  };

  /// Holds gap between Geometries related information.
  struct XodrGapBetweenGeometries {
    /// Id of the Road where the Geometries belongs to.
    RoadHeader::Id road_header_id{"none"};
    /// Holds the two Geometries' index responsible for the gap.
    /// Gap formed by endpoint of `geometry_index_pair.first` and startpoint of `geometry_index_pair.second`
    std::pair<int, int> geometry_index_pair{};
    /// Distance between first geometry's endpoint and second geometry's startpoint.
    double distance{};
  };

  /// Defines a structure that informs which actions can be performed to
  /// simplify the map geometry description.
  struct XodrGeometriesToSimplify {
    /// Defines actions that a builder tools may opt to do to simplify the
    /// geometry model.
    enum class Action {
      /// All the geometries in the vector might be replaced by one line whose
      /// length is the total length of the listed geometries. The heading would
      /// be the same as the first geometry.
      kReplaceByLine,
      /// All the geometries in the vector might be replaced by one arc whose
      /// length is the total length of the listed geometries and the curvature
      /// is the same as the first geometry. The heading would be the same as
      /// the first geometry.
      kReplaceByArc,
    };

    RoadHeader::Id road_header_id{"none"};
    Action action{Action::kReplaceByLine};
    std::vector<int> geometries{};
  };

  /// Holds LaneSection related information:
  struct XodrLaneSectionLengthData {
    /// Id of the Road that the LaneSection belongs to.
    RoadHeader::Id road_header_id{"none"};
    /// Index of the LaneSection.
    int lane_section_index{};
    /// Length of the LaneSection.
    double length{};
  };

  /// Creates a database manager from a XMLDocument which contains
  /// a XODR description.
  /// @param xodr_doc Contains the XODR description.
  /// @param parser_configuration Holds the configuration for the parser.
  /// @throw maliput::common::assertion_error When `xodr_doc` is nullptr.
  /// @throw maliput::common::assertion_error When `parser_configuration.tolerance` is negative.
  DBManager(tinyxml2::XMLDocument* xodr_doc, const ParserConfiguration& parser_configuration);
  DBManager() = delete;

  ~DBManager();

  /// @returns A xodr::Header which contains general information about the XODR description.
  const Header& GetXodrHeader() const;

  /// @returns A xodr::RoadHeader map which contains all the road information about the XODR description.
  const std::map<RoadHeader::Id, RoadHeader>& GetRoadHeaders() const;

  /// @returns A xodr::Junction map which contains all the junction information about the XODR description.
  const std::unordered_map<Junction::Id, Junction>& GetJunctions() const;

  /// @{ Xodr geometry introspection queries.

  /// @returns Data from the shortest Geometry in the entire XODR description.
  const XodrGeometryLengthData& GetShortestGeometry() const;

  /// @returns Data from the largest Geometry in the entire XODR description.
  const XodrGeometryLengthData& GetLargestGeometry() const;

  /// @returns Data from the shortest LaneSection in the entire XODR description.
  const XodrLaneSectionLengthData& GetShortestLaneSection() const;

  /// @returns Data from the largest LaneSection in the entire XODR description.
  const XodrLaneSectionLengthData& GetLargestLaneSection() const;

  /// Get the shortest gap between Geometries.
  /// DBManager extracts this information from the XODR only the first time that either this or `GetLargestGap()`
  /// methods are called. Results are stored to be consulted as many times as needed but without carrying out the
  /// analysis more than once in order to avoid performance drop.
  /// @returns Data from the shortest gap between Geometries in the entire XODR description.
  ///          When RoadHeaderId is "none" it means there are no possible gaps to be analyzed: Roads only have one
  ///          Geometry.
  const XodrGapBetweenGeometries& GetShortestGap() const;

  /// Get the largest gap between Geometries.
  /// DBManager extracts this information from the XODR only the first time that either this or 'GetShortestGap()'
  /// methods are called. Results are stored to be consulted as many times as needed but without carrying out the
  /// analysis more than once in order to avoid performance drop.
  /// @returns Data from the largest gap between Geometries in the entire XODR description.
  ///          When RoadHeaderId is "none" it means there are no possible gaps to be analyzed: Roads only have one
  ///          Geometry.
  const XodrGapBetweenGeometries& GetLargestGap() const;

  /// Get the shortest gap between elevations.
  ///
  /// DBManager extracts this information from the XODR only the first time
  /// that either this or `GetLargestElevationGap()` methods are called. Results
  /// are stored to be consulted as many times as needed but without carrying
  /// out the analysis more than once in order to avoid performance drop.
  ///
  /// @returns Data from the shortest gap between elevations in the entire XODR
  ///          description. When RoadHeaderId is "none" it means there are no
  ///          possible gaps to be analyzed: roads only have one or none
  ///          elevations.
  const XodrGapBetweenFunctions& GetShortestElevationGap() const;

  /// Get the largest gap between elevations.
  ///
  /// DBManager extracts this information from the XODR only the first time
  /// that either this or `GetShortestElevationGap()` methods are called.
  /// Results are stored to be consulted as many times as needed but without
  /// carrying out the analysis more than once in order to avoid performance
  /// drop.
  ///
  /// @returns Data from the largest gap between elevations in the entire XODR
  ///          description. When RoadHeaderId is "none" it means there are no
  ///          possible gaps to be analyzed: roads only have one or none
  ///          elevations.
  const XodrGapBetweenFunctions& GetLargestElevationGap() const;

  /// Get the shortest gap between superelevations.
  ///
  /// DBManager extracts this information from the XODR only the first time
  /// that either this or `GetLargestSuperelevationGap()` methods are called.
  /// Results are stored to be consulted as many times as needed but without
  /// carrying out the analysis more than once in order to avoid performance
  /// drop.
  ///
  /// @returns Data from the shortest gap between superelevations in the entire
  ///          XODR description. When RoadHeaderId is "none" it means there are
  ///          no possible gaps to be analyzed: roads only have one or none
  ///          superelevations.
  const XodrGapBetweenFunctions& GetShortestSuperelevationGap() const;

  /// Get the largest gap between superelevations.
  ///
  /// DBManager extracts this information from the XODR only the first time
  /// that either this or `GetShortestSuperelevationGap()` methods are called.
  /// Results are stored to be consulted as many times as needed but without
  /// carrying out the analysis more than once in order to avoid performance
  /// drop.
  ///
  /// @returns Data from the largest gap between superelevations in the entire
  ///          XODR description. When RoadHeaderId is "none" it means there are
  ///          no possible gaps to be analyzed: roads only have one or none
  ///          superelevations.
  const XodrGapBetweenFunctions& GetLargestSuperelevationGap() const;

  /// @} Xodr geometry introspection queries.

  /// Analyses all the RoadHeaders' geometries looking for pieces that can be
  /// unified or merged together into a simpler description.
  ///
  /// Candidate geometries to be merged should meet one of the following
  /// conditions:
  ///
  /// - Two or more consecutive lines that share the same heading and are
  ///   contiguous.
  /// - Two or more consecutive arcs that share the same curvature and are
  ///   contiguous.
  /// - Two or more consecutive geometries are shorter than @p tolerance.
  ///
  /// @param tolerance Threshold that defines when whether two geometries are
  ///        simplified when their accumulated length is below this value. It
  ///        must be non negative. When it is zero, two consecutive geometries
  ///        will not be simplified because of length.
  /// @return A dictionary of RoadHeader::Id to a vector of vectors of indices
  ///         of geometries. Indices that are grouped together are contiguous
  ///         and given that there might be more than one group per
  ///         RoadHeader::Id we have a vector of vectors.
  /// @throws maliput::common::assertion::error When @p tolerance is negative.
  // TODO(malidrive#571): Implement length criteria.
  const std::vector<XodrGeometriesToSimplify> GetGeometriesToSimplify(double tolerance) const;

 private:
  // Implementation of DBManager.
  class Impl;

  // Pointer to `Impl`ementation.
  mutable std::unique_ptr<Impl> impl_;
};

/// Loads a XODR description from a file.
/// @param filepath Filepath to the XODR file.
/// @param parser_configuration Holds the configuration for the parser.
/// @returns A DBManager.
/// @throw maliput::common::assertion_error When XODR description couldn't be correctly loaded.
std::unique_ptr<DBManager> LoadDataBaseFromFile(const std::string& filepath,
                                                const ParserConfiguration& parser_configuration);

/// Loads a XODR description from a string.
/// @param xodr_str String containing the XODR description.
/// @param parser_configuration Holds the configuration for the parser.
/// @returns A DBManager.
/// @throw maliput::common::assertion_error When XODR description couldn't be correctly loaded.
std::unique_ptr<DBManager> LoadDataBaseFromStr(const std::string& xodr_str,
                                               const ParserConfiguration& parser_configuration);

}  // namespace xodr
}  // namespace malidrive

// Copyright 2021 Toyota Research Institute
#pragma once

#include <string>
#include <vector>

#include <tinyxml2.h>

namespace malidrive {
namespace xodr {

/// Extracts Roads out of a XODR file and creates a new XODR description.
///  - Default header file with minimum information will be used.
///  - Roads that aren't found will be skipped with the correspondant logging message.
///  - Road linkage and junction value information can be updated depending on `update_linkage` argument.
/// @param xodr_doc XMLDocument pointer with a parsed XODR file.
/// @param road_ids Collection of road ids that are wanted to be extracted.
/// @param update_linkage True if predecessor,successor nodes and the junction id of the of the Roads should be updated
///                       by removing them. Otherwise those nodes won't be modified.
/// @returns A XODR description with the selected Road ids.
///
/// @throws maliput::common::assertion_error When the XML file contained in `xodr_doc` isn't a XODR file.
/// @throws maliput::common::assertion_error When XODR file's roads doesn't have 'id' attributes.
/// @throws maliput::common::assertion_error When road_ids is empty.
std::string XodrExtract(tinyxml2::XMLDocument* xodr_doc, const std::vector<std::string>& road_ids, bool update_linkage);

}  // namespace xodr
}  // namespace malidrive

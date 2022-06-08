// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2021-2022, Toyota Research Institute. All rights reserved.
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

// BSD 3-Clause License
//
// Copyright (c) 2023, Woven Planet. All rights reserved.
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

// Generates an OBJ from an XODR map.
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include <gflags/gflags.h>
#include <maliput/common/filesystem.h>
#include <maliput/common/logger.h>
#include <maliput/utility/generate_obj.h>

#include "applications/log_level_flag.h"
#include "maliput_malidrive/builder/road_network_builder.h"
#include "maliput_malidrive/loader/loader.h"

namespace malidrive {
namespace applications {
namespace xodr {
namespace {

// @return A string with the usage message.
std::string GetUsageMessage() {
  std::stringstream ss;
  ss << "CLI for XODR to OBJ conversion:" << std::endl << std::endl;
  ss << "  xodr_to_obj --xodr_file=<path> --out-dir=<path> --tolerance=<float> [--allow-schema-errors] "
        "[--allow-semantic-errors]"
     << std::endl;
  return ss.str();
}

// @{ CLI Arguments
DEFINE_string(xodr_file, "", "XODR input file defining a Malidrive road geometry");
DEFINE_string(out_dir, ".", "Directory to save the OBJ to");
DEFINE_double(tolerance, 1e-3, "Tolerance to validate continuity in piecewise defined geometries.");
DEFINE_bool(allow_schema_errors, false, "If true, the XODR parser will attempt to work around XODR schema violations.");
DEFINE_bool(allow_semantic_errors, false,
            "If true, the XODR parser will attempt to work around XODR semantic violations.");
MALIPUT_MALIDRIVE_APPLICATION_DEFINE_LOG_LEVEL_FLAG();
// @}

int Main(int argc, char** argv) {
  // Handles CLI arguments.
  gflags::SetUsageMessage(GetUsageMessage());
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  maliput::common::set_log_level(FLAGS_log_level);
  if (FLAGS_xodr_file.empty()) {
    maliput::log()->error("No input file specified.");
    return 1;
  }
  maliput::log()->info("Parser: Allow schema errors: ", (FLAGS_allow_schema_errors ? "enabled" : "disabled"));
  maliput::log()->info("Parser: Allow semantic errors: ", (FLAGS_allow_semantic_errors ? "enabled" : "disabled"));

  // Load the road network
  std::map<std::string, std::string> road_network_configuration;
  road_network_configuration.emplace("opendrive_file", FLAGS_xodr_file);
  auto road_network = malidrive::loader::Load<malidrive::builder::RoadNetworkBuilder>(road_network_configuration);

  std::string name = FLAGS_xodr_file;
  name.erase(name.begin(), name.begin() + name.rfind("/") + 1);
  name.erase(name.begin() + name.rfind("."), name.end());

  maliput::utility::ObjFeatures features;
  features.min_grid_resolution = 5.0;
  maliput::utility::GenerateObjFile(road_network->road_geometry(), FLAGS_out_dir, name, features);
  return 0;
}

}  // namespace
}  // namespace xodr
}  // namespace applications
}  // namespace malidrive

int main(int argc, char** argv) { return malidrive::applications::xodr::Main(argc, argv); }

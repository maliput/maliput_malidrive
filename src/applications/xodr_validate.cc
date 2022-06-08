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

// Validates a XODR map.
#include <iostream>
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

// @return A string with the usage message.
std::string GetUsageMessage() {
  std::stringstream ss;
  ss << "CLI for XODR validation:" << std::endl << std::endl;
  ss << "  xodr_validate --xodr_file=<xodr_file_path>" << std::endl;
  return ss.str();
}

// @{ CLI Arguments
DEFINE_string(xodr_file, "", "XODR input file defining a Malidrive road geometry");
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
  maliput::log()->info("Parser: Allow schema errors: {}", (FLAGS_allow_schema_errors ? "enabled" : "disabled"));
  maliput::log()->info("Parser: Allow semantic errors: {}", (FLAGS_allow_semantic_errors ? "enabled" : "disabled"));

  // Tries to load the XODR map and logs the result.
  try {
    auto db_manager = malidrive::xodr::LoadDataBaseFromFile(
        FLAGS_xodr_file, {FLAGS_tolerance, FLAGS_allow_schema_errors, FLAGS_allow_semantic_errors});
    std::cout << "Successfully loaded the map." << std::endl;
  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    std::cerr << "The map could not be loaded." << std::endl;
  }
  return 1;
}

}  // namespace
}  // namespace xodr
}  // namespace applications
}  // namespace malidrive

int main(int argc, char** argv) { return malidrive::applications::xodr::Main(argc, argv); }

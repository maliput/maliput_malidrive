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
#include <map>
#include <memory>
#include <string>

#include <gtest/gtest.h>
#include <maliput/common/filesystem.h>
#include <maliput/plugin/maliput_plugin.h>
#include <maliput/plugin/maliput_plugin_manager.h>
#include <maliput/plugin/maliput_plugin_type.h>
#include <maliput/plugin/road_network_loader.h>

#include "maliput_malidrive/base/road_geometry.h"
#include "utility/resources.h"

namespace malidrive {
namespace {

// Resource folder path defined via compile definition.
static constexpr char kMalidriveResourceFolder[] = DEF_MALIDRIVE_RESOURCES;

GTEST_TEST(RoadNetworkLoader, VerifyRoadNetworkPlugin) {
  setenv("MALIPUT_PLUGIN_PATH", DEF_ROAD_NETWORK_PLUGIN, 1);
  const std::string kXodrFilePath{utility::FindResourceInPath("TShapeRoad.xodr", kMalidriveResourceFolder)};

  // RoadNetworkLoader plugin id.
  const maliput::plugin::MaliputPlugin::Id kMaliputMalidrivePluginId{"maliput_malidrive"};
  // maliput malidirve properties needed for loading a road geometry.
  const std::map<std::string, std::string> rg_maliput_malidrive_properties{
      {"road_geometry_id", "maliput_malidrive road geometry"},
      {"opendrive_file", kXodrFilePath},
      {"linear_tolerance", "1e-3"},
      {"angular_tolerance", "1e-3"},
      {"scale_length", "1"},
      {"build_policy", "sequential"},
      {"simplification_policy", "none"},
      {"tolerance_selection_policy", "manual"},
      {"standard_strictness_policy", "strict"},
      {"inertial_to_backend_frame_translation", "{1., 2., 3.}"},
  };
  // Check MaliputPlugin existence.
  maliput::plugin::MaliputPluginManager manager{};
  const maliput::plugin::MaliputPlugin* rn_plugin{manager.GetPlugin(kMaliputMalidrivePluginId)};
  ASSERT_NE(nullptr, rn_plugin);

  // Check maliput_malidrive plugin is obtained.
  EXPECT_EQ(kMaliputMalidrivePluginId.string(), rn_plugin->GetId());
  EXPECT_EQ(maliput::plugin::MaliputPluginType::kRoadNetworkLoader, rn_plugin->GetType());
  maliput::plugin::RoadNetworkLoaderPtr rn_loader_ptr{nullptr};
  EXPECT_NO_THROW(rn_loader_ptr = rn_plugin->ExecuteSymbol<maliput::plugin::RoadNetworkLoaderPtr>(
                      maliput::plugin::RoadNetworkLoader::GetEntryPoint()));
  ASSERT_NE(nullptr, rn_loader_ptr);
  std::unique_ptr<maliput::plugin::RoadNetworkLoader> rn_loader{
      reinterpret_cast<maliput::plugin::RoadNetworkLoader*>(rn_loader_ptr)};

  // Check maliput_malidrive RoadNetwork is constructible.
  std::unique_ptr<const maliput::api::RoadNetwork> rn = (*rn_loader)(rg_maliput_malidrive_properties);
  ASSERT_NE(nullptr, rn);
  ASSERT_NE(nullptr, rn->road_geometry());
}

}  // namespace
}  // namespace malidrive

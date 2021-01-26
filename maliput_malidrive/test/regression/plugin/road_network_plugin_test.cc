// Copyright 2021 Toyota Research Institute
#include <map>
#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "maliput/common/filesystem.h"
#include "maliput/plugin/maliput_plugin.h"
#include "maliput/plugin/maliput_plugin_manager.h"
#include "maliput/plugin/maliput_plugin_type.h"
#include "maliput/plugin/road_network_loader.h"

#include "maliput_malidrive/base/road_geometry.h"

#include "utility/resources.h"

namespace malidrive {
namespace {

GTEST_TEST(RoadNetworkLoader, VerifyRoadNetworkPlugin) {
  const std::string kXodrFilePath{utility::FindResource("odr/TShapeRoad.xodr")};

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
  };
  // Check MaliputPlugin existence.
  maliput::plugin::MaliputPluginManager manager{};
  const maliput::plugin::MaliputPlugin* rn_plugin{manager.GetPlugin(kMaliputMalidrivePluginId)};
  ASSERT_NE(nullptr, rn_plugin);

  // Check maliput_malidrive plugin is obtained.
  EXPECT_EQ(kMaliputMalidrivePluginId.string(), rn_plugin->GetId());
  EXPECT_EQ(maliput::plugin::MaliputPluginType::kRoadNetworkLoader, rn_plugin->GetType());
  auto rn_loader = rn_plugin->ExecuteSymbol<std::unique_ptr<maliput::plugin::RoadNetworkLoader>>(
      maliput::plugin::RoadNetworkLoader::GetEntryPoint());
  ASSERT_NE(nullptr, rn_loader);

  // Check maliput_malidrive RoadNetwork is constructible.
  std::unique_ptr<const maliput::api::RoadNetwork> rn = (*rn_loader)(rg_maliput_malidrive_properties);
  ASSERT_NE(nullptr, rn);
  ASSERT_NE(nullptr, rn->road_geometry());
}

}  // namespace
}  // namespace malidrive

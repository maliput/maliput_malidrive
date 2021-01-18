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

namespace malidrive {
namespace {

GTEST_TEST(RoadNetworkLoader, VerifyRoadNetworkPlugin) {
  // Get absolute path to a xodr file to be used for testing.
  const std::string MALIPUT_MALIDRIVE_RESOURCE_ROOT{"MALIPUT_MALIDRIVE_RESOURCE_ROOT"};
  const std::string kFileName{"/resources/odr/TShapeRoad.xodr"};
  const std::string env_path = maliput::common::Filesystem::get_env_path(MALIPUT_MALIDRIVE_RESOURCE_ROOT);
  ASSERT_TRUE(!env_path.empty());

  // RoadNetworkLoader plugin id.
  const maliput::plugin::MaliputPlugin::Id kMaliputMalidrivePluginId{"maliput_malidrive"};
  // maliput malidirve properties needed for loading a road geometry.
  const std::map<std::string, std::string> rg_maliput_malidrive_properties{
      {"road_geometry_id", "maliput_malidrive road geometry"},
      {"opendrive_file", env_path + kFileName},
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
  std::unique_ptr<maliput::plugin::RoadNetworkLoader> rn_loader;
  EXPECT_EQ(kMaliputMalidrivePluginId.string(), rn_plugin->GetId());
  EXPECT_EQ(maliput::plugin::MaliputPluginType::kRoadNetworkLoader, rn_plugin->GetType());
  EXPECT_NO_THROW(rn_loader = rn_plugin->ExecuteSymbol<std::unique_ptr<maliput::plugin::RoadNetworkLoader>>(
                      maliput::plugin::RoadNetworkLoader::GetEntryPoint()));
  ASSERT_NE(nullptr, rn_loader);

  // Check maliput_malidrive RoadNetwork is constructible.
  std::unique_ptr<const maliput::api::RoadNetwork> rn;
  EXPECT_NO_THROW(rn = (*rn_loader)(rg_maliput_malidrive_properties));
  ASSERT_NE(nullptr, rn);
  auto maliput_malidrive_rg = dynamic_cast<const RoadGeometry*>(rn->road_geometry());
  ASSERT_NE(nullptr, maliput_malidrive_rg);
}

}  // namespace
}  // namespace malidrive

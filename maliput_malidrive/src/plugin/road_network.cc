// Copyright 2021 Toyota Research Institute
#include <map>
#include <memory>
#include <string>

#include <maliput/plugin/road_network_loader.h>

#include "maliput_malidrive/builder/road_network_builder.h"

namespace malidrive {
namespace plugin {
namespace {

// Implementation of a maliput::plugin::RoadNetworkLoader using maliput_malidrive backend.
class RoadNetworkLoader : public maliput::plugin::RoadNetworkLoader {
 public:
  std::unique_ptr<maliput::api::RoadNetwork> operator()(
      const std::map<std::string, std::string>& properties) const override {
    return malidrive::builder::RoadNetworkBuilder(properties)();
  }
};

}  // namespace

REGISTER_ROAD_NETWORK_LOADER_PLUGIN("maliput_malidrive", RoadNetworkLoader);

}  // namespace plugin
}  // namespace malidrive

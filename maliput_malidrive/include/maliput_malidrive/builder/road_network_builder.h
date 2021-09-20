// Copyright 2020 Toyota Research Institute
#pragma once

#include <map>
#include <memory>
#include <string>

#include <maliput/api/road_network.h>

#include "maliput_malidrive/common/macros.h"

namespace malidrive {
namespace builder {

class RoadNetworkBuilder {
 public:
  MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN(RoadNetworkBuilder);

  /// Constructs a RoadNetworkBuilder.
  ///
  /// @param road_network_configuration Holds the information of all the
  ///        RoadNetwork entities.
  explicit RoadNetworkBuilder(const std::map<std::string, std::string>& road_network_configuration)
      : road_network_configuration_(road_network_configuration) {}

  /// @return A maliput_malidrive RoadNetwork.
  std::unique_ptr<maliput::api::RoadNetwork> operator()() const;

 private:
  const std::map<std::string, std::string> road_network_configuration_;
};

}  // namespace builder
}  // namespace malidrive

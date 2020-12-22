##############################################################################
# Sources
##############################################################################

add_library(malidrive_road_network malidrive_road_network.cc)

set_target_properties(malidrive_road_network
  PROPERTIES
    OUTPUT_NAME malidrive_road_network
)

ament_target_dependencies(malidrive_road_network
    "maliput"
)

target_link_libraries(malidrive_road_network
  maliput::api
  maliput::common
  maliput::base
  maliput_malidrive::builder
)

target_include_directories(malidrive_road_network
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

##############################################################################
# Export
##############################################################################

install(
  TARGETS malidrive_road_network
  EXPORT ${PROJECT_NAME}-targets
  DESTINATION "${CMAKE_INSTALL_PREFIX}/../plugins"
)

##############################################################################
# Malidrive builder
##############################################################################
add_library(builder
  builder_tools.cc
  determine_tolerance.cc
  direction_usage_builder.cc
  discrete_value_rule_state_provider_builder.cc
  id_providers.cc
  range_value_rule_state_provider_builder.cc
  road_curve_factory.cc
  road_geometry_builder.cc
  road_geometry_configuration.cc
  road_network_builder.cc
  road_network_configuration.cc
  rule_registry_builder.cc
  road_rulebook_builder.cc
  rule_tools.cc
  simplify_geometries.cc
  speed_limit_builder.cc
  xodr_parser_configuration.cc
)
add_library(maliput_malidrive::builder ALIAS builder)
set_target_properties(builder
  PROPERTIES
    OUTPUT_NAME maliput_malidrive_builder
)

target_include_directories(
  builder
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(
  builder
    drake::drake
    maliput::api
    maliput::base
    maliput::geometry_base
    maliput::math
    maliput::utilities
    maliput_malidrive::base
    maliput_malidrive::common
    maliput_malidrive::road_curve
    maliput_malidrive::xodr
)

install(TARGETS builder
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

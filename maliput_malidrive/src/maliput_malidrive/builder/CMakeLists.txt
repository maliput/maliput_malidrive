##############################################################################
# Malidrive builder
##############################################################################
add_library(builder
  builder_tools.cc
  determine_tolerance.cc
  direction_usage_builder.cc
  discrete_value_rule_state_provider_builder.cc
  id_providers.cc
  phase_provider_builder.cc
  range_value_rule_state_provider_builder.cc
  road_curve_factory.cc
  road_geometry_builder.cc
  road_geometry_configuration.cc
  road_network_builder.cc
  road_network_configuration.cc
  rule_registry_builder.cc
  road_rulebook_builder.cc
  road_rulebook_builder_old_rules.cc
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
    PUBLIC
      maliput::api
      maliput::base
      maliput::geometry_base
      maliput::math
      maliput::utility
      maliput_malidrive::common
    PRIVATE
      maliput_drake::analysis
      maliput_drake::common
      maliput_drake::framework
      maliput_drake::math
      maliput_drake::trajectories
      maliput_malidrive::base
      maliput_malidrive::road_curve
      maliput_malidrive::xodr
)

install(TARGETS builder
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

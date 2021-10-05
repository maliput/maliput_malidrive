add_library(test_utilities
  road_geometry_configuration_for_xodrs.cc
)
add_library(maliput_malidrive::test_utilities ALIAS test_utilities)

set_target_properties(test_utilities
  PROPERTIES
    OUTPUT_NAME maliput_malidrive_test_utilities
)

target_include_directories(
  test_utilities
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(test_utilities
  PUBLIC
    maliput::api
    maliput::math
    maliput_malidrive::builder
    maliput_malidrive::common
)

# Install the exported target
install(
  TARGETS test_utilities
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

ament_export_include_directories(${CMAKE_CURRENT_SOURCE_DIR})



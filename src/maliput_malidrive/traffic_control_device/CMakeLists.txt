##############################################################################
# Malidrive traffic signal
##############################################################################
add_library(traffic_control_device
  traffic_control_device_database_loader.cc
  yaml_helper.cc
  parser.cc
)
add_library(maliput_malidrive::traffic_control_device ALIAS traffic_control_device)
set_target_properties(traffic_control_device
  PROPERTIES
    OUTPUT_NAME maliput_malidrive_traffic_control_device
)

target_include_directories(
  traffic_control_device
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(
  traffic_control_device
      maliput::api
      maliput::common
      maliput::math
      maliput_malidrive::common
      yaml-cpp
)

install(TARGETS traffic_control_device
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

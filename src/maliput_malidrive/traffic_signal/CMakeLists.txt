##############################################################################
# Malidrive traffic signal
##############################################################################
add_library(traffic_signal
  yaml_helper.cc
  parser.cc
)
add_library(maliput_malidrive::traffic_signal ALIAS traffic_signal)
set_target_properties(traffic_signal
  PROPERTIES
    OUTPUT_NAME maliput_malidrive_traffic_signal
)

target_include_directories(
  traffic_signal
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(
  traffic_signal
      maliput::api
      maliput::math
      maliput_malidrive::common
      yaml-cpp
)

install(TARGETS traffic_signal
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

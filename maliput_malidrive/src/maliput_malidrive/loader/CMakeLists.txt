##############################################################################
# Malidrive loader
##############################################################################
add_library(loader
  loader.cc
)
add_library(maliput_malidrive::loader ALIAS loader)
set_target_properties(loader
  PROPERTIES
    OUTPUT_NAME maliput_malidrive_loader
)

target_include_directories(
  loader
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(
  loader
    PUBLIC
      maliput::api
      maliput::base
      maliput::geometry_base
      maliput::math
      maliput_malidrive::builder
      maliput_malidrive::common
)

install(TARGETS loader
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
